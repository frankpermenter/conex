#include "conex/sparse_linear_constraint.h"

#include <algorithm>
#include <chrono>
#include <numeric>

#include "conex/clique_ordering.h"
#include "conex/cone_program.h"
#include "conex/constraint_manager.h"
#include "conex/kkt_solver_factory.h"
#include "conex/kkt_tree_solver.h"
#include "conex/linear_constraint.h"
#include "conex/workspace.h"

namespace conex {

namespace {

// Returns true if a ⊆ b (both sorted).
bool IsSubset(const std::vector<int>& a, const std::vector<int>& b) {
  return std::includes(b.begin(), b.end(), a.begin(), a.end());
}

}  // namespace

SparseLinearConstraint::SparseLinearConstraint(
    const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b) {
  CONEX_DEMAND(A.rows() == b.rows(),
               "A and b must have the same number of rows.");

  // Build row supports from column-major sparse matrix.
  std::vector<std::vector<int>> row_supports(A.rows());
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      row_supports[it.row()].push_back(it.col());
    }
  }
  for (auto& s : row_supports) {
    std::sort(s.begin(), s.end());
  }

  // Sort row indices by support size (descending) so that the largest
  // supports become group leaders and smaller supports are absorbed.
  std::vector<int> order(A.rows());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    return row_supports[a].size() > row_supports[b].size();
  });

  // Greedy grouping: assign each row to the first group whose support
  // contains the row's support.  Since rows are processed largest-first,
  // a row's support is never a strict superset of an existing group's.
  struct GroupInfo {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<GroupInfo> group_infos;

  for (int row : order) {
    const auto& supp = row_supports[row];
    bool merged = false;
    for (auto& gi : group_infos) {
      if (IsSubset(supp, gi.support)) {
        gi.rows.push_back(row);
        merged = true;
        break;
      }
    }
    if (!merged) {
      group_infos.push_back({supp, {row}});
    }
  }

  // Create dense sub-blocks for each group.
  for (auto& gi : group_infos) {
    RowGroup group;
    group.variables = gi.support;
    int nrows = static_cast<int>(gi.rows.size());
    int ncols = static_cast<int>(gi.support.size());
    group.A.resize(nrows, ncols);
    group.b.resize(nrows);
    for (int i = 0; i < nrows; ++i) {
      group.b(i) = b(gi.rows[i]);
      for (int j = 0; j < ncols; ++j) {
        group.A(i, j) = A.coeff(gi.rows[i], gi.support[j]);
      }
    }
    groups_.push_back(std::move(group));
  }
}

std::vector<int> SparseLinearConstraint::AddToProgram(Program& prog) {
  std::vector<int> ids;
  for (auto& group : groups_) {
    ids.push_back(
        prog.AddConstraint(LinearConstraint(group.A, group.b), group.variables));
  }
  return ids;
}

SparseLeastSquaresResult SparseLeastSquares(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  // Phase 1: Decompose A into row groups.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  SparseLinearConstraint slc(A, b_zero);

  auto t_grouped = clock::now();

  // Phase 2: Add constraints to ConstraintManager.
  ConstraintManager cm(num_vars);
  for (auto& group : slc.groups()) {
    cm.AddConstraint(LinearConstraint(group.A, group.b), group.variables);
  }

  auto t_added = clock::now();

  // Phase 3: Initialize workspace and set W = identity.
  cm.InitializeWorkspace();
  for (auto* c : cm.cone_inequalities()) {
    c->constraint()->SetIdentity();
  }

  auto t_init = clock::now();

  // Phase 4: Extract cliques and build clique tree.
  auto clique_assemblers = cm.clique_assemblers();
  std::vector<std::vector<int>> cliques;
  for (const auto& assembler : clique_assemblers) {
    auto c = assembler->get_cliques();
    cliques.insert(cliques.end(), c.begin(), c.end());
  }

  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, /*maximal_cliques_out=*/nullptr,
      /*max_merge_supernode_size=*/5, SUPERNODE_REORDER_BFS_GREEDY, {});

  auto t_clique = clock::now();

  // Phase 5: Build tree solver (adapters + Finalize).
  auto tree_solver =
      std::make_unique<SymmetricLinearSystemTreeSolver>();

  for (auto* assembler : clique_assemblers) {
    auto adapter =
        std::make_unique<KKTAssemblerToSubsystemAdapter>(assembler);
    adapter->set_contribution_type(ContributionType::kPositiveDefinite);
    tree_solver->push_back(std::move(adapter));
  }
  tree_solver->Finalize(clique_tree);
  tree_solver->SetFactorizationMode(true);
  tree_solver->EnableAutoUpdateAtAssemble(true);

  auto t1 = clock::now();

  // Assemble A^T A and factor.
  bool ok = tree_solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // Solve A^T A x = rhs.
  result.x = tree_solver->Solve(rhs);

  auto t3 = clock::now();

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.assemble_and_factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  // Sub-phase breakdown.
  result.grouping_us =
      std::chrono::duration<double, std::micro>(t_grouped - t0).count();
  result.add_constraints_us =
      std::chrono::duration<double, std::micro>(t_added - t_grouped).count();
  result.init_workspace_us =
      std::chrono::duration<double, std::micro>(t_init - t_added).count();
  result.clique_extraction_us =
      std::chrono::duration<double, std::micro>(t_clique - t_init).count();
  result.finalize_us =
      std::chrono::duration<double, std::micro>(t1 - t_clique).count();

  return result;
}

}  // namespace conex
