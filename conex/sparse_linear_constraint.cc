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

  // Step 1: Build row supports in O(nnz).
  // A is column-major, so iterating columns gives sorted supports directly.
  std::vector<std::vector<int>> row_supports(A.rows());
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      row_supports[it.row()].push_back(it.col());
    }
  }

  // Step 2: Group rows with identical support.
  // Sort row indices by support to cluster identical supports together.
  std::vector<int> row_order(A.rows());
  std::iota(row_order.begin(), row_order.end(), 0);
  std::sort(row_order.begin(), row_order.end(), [&](int a, int b) {
    return row_supports[a] < row_supports[b];
  });

  // Collect unique-support groups.
  struct UniqueGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<UniqueGroup> unique_groups;
  for (int row : row_order) {
    if (unique_groups.empty() ||
        unique_groups.back().support != row_supports[row]) {
      unique_groups.push_back({row_supports[row], {}});
    }
    unique_groups.back().rows.push_back(row);
  }

  // Step 3: Containment merging on unique supports only.
  // Sort by support size descending so larger supports are group leaders.
  std::sort(unique_groups.begin(), unique_groups.end(),
            [](const UniqueGroup& a, const UniqueGroup& b) {
              return a.support.size() > b.support.size();
            });

  struct GroupInfo {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<GroupInfo> group_infos;

  for (auto& ug : unique_groups) {
    bool merged = false;
    for (auto& gi : group_infos) {
      if (IsSubset(ug.support, gi.support)) {
        gi.rows.insert(gi.rows.end(), ug.rows.begin(), ug.rows.end());
        merged = true;
        break;
      }
    }
    if (!merged) {
      group_infos.push_back({ug.support, std::move(ug.rows)});
    }
  }

  // Step 4: Create dense sub-blocks.
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

SparseLeastSquaresResult SparseLeastSquaresMaximalClique(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs) {
  using clock = std::chrono::high_resolution_clock;
  SparseLeastSquaresResult result;
  const int num_vars = A.cols();

  auto t0 = clock::now();

  // Step 1: Build unique row supports.
  std::vector<std::vector<int>> row_supports(A.rows());
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      row_supports[it.row()].push_back(it.col());
    }
  }

  // Deduplicate: group rows by identical support.
  std::vector<int> row_order(A.rows());
  std::iota(row_order.begin(), row_order.end(), 0);
  std::sort(row_order.begin(), row_order.end(), [&](int a, int b) {
    return row_supports[a] < row_supports[b];
  });

  struct SupportGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<SupportGroup> support_groups;
  for (int row : row_order) {
    if (row_supports[row].empty()) continue;
    if (support_groups.empty() ||
        support_groups.back().support != row_supports[row]) {
      support_groups.push_back({row_supports[row], {}});
    }
    support_groups.back().rows.push_back(row);
  }

  // Collect unique supports for clique ordering.
  std::vector<std::vector<int>> unique_supports;
  unique_supports.reserve(support_groups.size());
  for (const auto& sg : support_groups) {
    unique_supports.push_back(sg.support);
  }

  // Step 2: Get maximal cliques from clique ordering.
  std::vector<std::vector<int>> maximal_cliques;
  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      unique_supports, &maximal_cliques,
      /*max_merge_supernode_size=*/5, SUPERNODE_REORDER_BFS_GREEDY, {});

  // Step 3: Assign each support group to the smallest maximal clique
  // that contains it.
  struct CliqueGroup {
    int clique_index;
    std::vector<int> rows;
  };
  std::vector<CliqueGroup> clique_groups(maximal_cliques.size());
  for (size_t ci = 0; ci < maximal_cliques.size(); ++ci) {
    clique_groups[ci].clique_index = static_cast<int>(ci);
  }

  for (const auto& sg : support_groups) {
    int best = -1;
    size_t best_size = std::numeric_limits<size_t>::max();
    for (size_t ci = 0; ci < maximal_cliques.size(); ++ci) {
      const auto& clique = maximal_cliques[ci];
      if (clique.size() < sg.support.size()) continue;
      if (clique.size() >= best_size) continue;
      if (IsSubset(sg.support, clique)) {
        best = static_cast<int>(ci);
        best_size = clique.size();
      }
    }
    CONEX_DEMAND(best >= 0,
                 "No maximal clique contains a support set.");
    clique_groups[best].rows.insert(
        clique_groups[best].rows.end(), sg.rows.begin(), sg.rows.end());
  }

  // Step 4: Build LinearConstraints per maximal clique and add to
  // ConstraintManager.  Each non-empty clique group becomes one
  // LinearConstraint whose variable set is the maximal clique.
  auto t_grouped = clock::now();

  ConstraintManager cm(num_vars);
  for (size_t ci = 0; ci < clique_groups.size(); ++ci) {
    const auto& cg = clique_groups[ci];
    if (cg.rows.empty()) continue;
    const auto& vars = maximal_cliques[ci];
    int nrows = static_cast<int>(cg.rows.size());
    int ncols = static_cast<int>(vars.size());
    Eigen::MatrixXd Ag(nrows, ncols);
    Eigen::VectorXd bg = Eigen::VectorXd::Zero(nrows);
    for (int i = 0; i < nrows; ++i) {
      for (int j = 0; j < ncols; ++j) {
        Ag(i, j) = A.coeff(cg.rows[i], vars[j]);
      }
    }
    cm.AddConstraint(LinearConstraint(Ag, bg), vars);
  }

  auto t_added = clock::now();

  // Phase 3: Initialize workspace and set W = identity.
  cm.InitializeWorkspace();
  for (auto* c : cm.cone_inequalities()) {
    c->constraint()->SetIdentity();
  }

  auto t_init = clock::now();

  // Phase 4: Build tree solver using the clique tree from step 2.
  auto clique_assemblers = cm.clique_assemblers();

  auto t_clique = clock::now();

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
