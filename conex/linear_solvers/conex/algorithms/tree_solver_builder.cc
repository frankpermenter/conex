#include "conex/algorithms/tree_solver_builder.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <sstream>

#include "conex/common/clique_tree.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/static_subsystem.h"

namespace conex {

int TreeSolverBuilder::AddClique(int parent) {
  int id = static_cast<int>(cliques_.size());
  CONEX_DEMAND(parent == -1 || (parent >= 0 && parent < id),
               "Invalid parent: must be -1 (root) or an existing clique id.");
  cliques_.push_back({parent, {}, {}});
  if (parent >= 0) {
    cliques_[parent].children.push_back(id);
  }
  return id;
}

void TreeSolverBuilder::AddCost(int clique, const Eigen::MatrixXd& Q,
                                const std::vector<int>& vars) {
  CONEX_DEMAND(clique >= 0 && clique < static_cast<int>(cliques_.size()),
               "Invalid clique id.");
  CONEX_DEMAND(Q.rows() == static_cast<int>(vars.size()) &&
                   Q.cols() == static_cast<int>(vars.size()),
               "Q dimensions must match vars size.");
  cliques_[clique].all_vars.insert(vars.begin(), vars.end());
  cost_assemblers_.emplace_back(Q, vars);
  pending_.push_back(
      {&cost_assemblers_.back(), clique, ContributionType::kPositiveDefinite});
}

void TreeSolverBuilder::AddEquality(int clique, const Eigen::MatrixXd& C,
                                    const Eigen::VectorXd& d,
                                    const std::vector<int>& primal_vars,
                                    const std::vector<int>& dual_vars) {
  CONEX_DEMAND(clique >= 0 && clique < static_cast<int>(cliques_.size()),
               "Invalid clique id.");
  CONEX_DEMAND(C.rows() == static_cast<int>(dual_vars.size()),
               "C rows must match dual_vars size.");
  CONEX_DEMAND(C.cols() == static_cast<int>(primal_vars.size()),
               "C cols must match primal_vars size.");
  cliques_[clique].all_vars.insert(primal_vars.begin(), primal_vars.end());
  cliques_[clique].all_vars.insert(dual_vars.begin(), dual_vars.end());
  eq_assemblers_.emplace_back(C, d, primal_vars, dual_vars);
  pending_.push_back(
      {&eq_assemblers_.back(), clique, ContributionType::kIndefinite});
}

TreeSolverBuilder::Result TreeSolverBuilder::Build() {
  const int num_cliques = static_cast<int>(cliques_.size());
  CONEX_DEMAND(num_cliques > 0, "No cliques added.");

  // Validate: exactly one root.
  int root = -1;
  for (int i = 0; i < num_cliques; ++i) {
    if (cliques_[i].parent == -1) {
      CONEX_DEMAND(root == -1, "Multiple roots found.");
      root = i;
    }
  }
  CONEX_DEMAND(root >= 0, "No root clique found.");

  // Compute post-order (children before parents) via DFS.
  std::vector<int> post_order;
  post_order.reserve(num_cliques);
  std::function<void(int)> dfs = [&](int node) {
    for (int child : cliques_[node].children) dfs(child);
    post_order.push_back(node);
  };
  dfs(root);
  CONEX_DEMAND(static_cast<int>(post_order.size()) == num_cliques,
               "Tree is disconnected.");

  // Compute separators bottom-up.
  // separator(c) = all_vars(c) ∩ all_vars(parent(c))
  // supernode(c) = all_vars(c) \ separator(c)
  std::vector<std::vector<int>> supernodes(num_cliques);
  std::vector<std::vector<int>> separators(num_cliques);

  for (int c : post_order) {
    int p = cliques_[c].parent;
    if (p == -1) {
      // Root: all variables are supernodes.
      supernodes[c].assign(cliques_[c].all_vars.begin(),
                           cliques_[c].all_vars.end());
    } else {
      const auto& parent_vars = cliques_[p].all_vars;
      for (int v : cliques_[c].all_vars) {
        if (parent_vars.count(v)) {
          separators[c].push_back(v);
        } else {
          supernodes[c].push_back(v);
        }
      }
    }
  }

  // Optional RIP validation: for every variable v in clique c that also
  // appears in some ancestor a, v must appear in every clique on the
  // path from c to a.  Without this, a variable could land in a supernode
  // when it should be a separator, producing a silently wrong factorization.
  if (check_rip_) {
    for (int c = 0; c < num_cliques; ++c) {
      for (int v : cliques_[c].all_vars) {
        // Walk from c's parent toward the root.  If v reappears in an
        // ancestor, every intermediate clique must also contain v.
        bool found_gap = false;
        int gap_clique = -1;
        for (int a = cliques_[c].parent; a >= 0; a = cliques_[a].parent) {
          if (cliques_[a].all_vars.count(v)) {
            // v is in ancestor a.  If we had a gap, that's a violation.
            CONEX_DEMAND(!found_gap,
                         "Running intersection property violated: variable "
                         "appears in a child and ancestor but is missing "
                         "from an intermediate clique.");
            break;
          }
          // v is NOT in this intermediate clique — record the gap.
          found_gap = true;
          gap_clique = a;
        }
      }
    }
  }

  // Compute total variable count.
  std::set<int> all_vars;
  for (const auto& ci : cliques_) {
    all_vars.insert(ci.all_vars.begin(), ci.all_vars.end());
  }
  int num_variables = all_vars.empty() ? 0 : (*all_vars.rbegin() + 1);

  // Build CliqueTree.
  CliqueTree tree;
  tree.supernodes = supernodes;
  tree.separators = separators;
  tree.node_to_parent.resize(num_cliques);
  for (int i = 0; i < num_cliques; ++i) {
    tree.node_to_parent[i] = cliques_[i].parent;
  }
  tree.post_order_position_to_clique = post_order;

  // Build solver.
  auto solver = std::make_unique<SymmetricLinearSystemTreeSolver>();
  for (auto& pa : pending_) {
    auto adapter =
        std::make_unique<KKTAssemblerToSubsystemAdapter>(pa.assembler);
    adapter->set_contribution_type(pa.type);
    solver->push_back(std::move(adapter));
  }

  solver->SetUseGenericFactorization(false);
  solver->SetUseLUForIndefinite(false);
  solver->Finalize(tree);
  solver->SetFactorizationMode(true);
  solver->EnableAutoUpdateAtAssemble(true);

  Result result;
  result.solver = std::move(solver);
  result.num_variables = num_variables;
  return result;
}

struct TreeSolverBuilder::Result::Storage {
  Eigen::SparseMatrix<double> Q;
  ConstraintManager cm;
};

TreeSolverBuilder::Result::Result() = default;
TreeSolverBuilder::Result::~Result() = default;
TreeSolverBuilder::Result::Result(Result&&) noexcept = default;
TreeSolverBuilder::Result& TreeSolverBuilder::Result::operator=(Result&&) noexcept = default;

TreeSolverBuilder::Result TreeSolverBuilder::BuildFromSparseMatrices(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& C,
    const Eigen::VectorXd& d) {
  auto storage = std::make_unique<Result::Storage>();
  storage->Q = Q;  // own a copy
  const int n_primal = Q.cols();
  const int n_eq = C.rows();

  std::set<int> q_var_set;
  for (int k = 0; k < storage->Q.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(storage->Q, k); it;
         ++it) {
      q_var_set.insert(it.row());
      q_var_set.insert(it.col());
    }
  std::vector<int> q_vars(q_var_set.begin(), q_var_set.end());

  storage->cm = ConstraintManager(n_primal);
  auto q_asm =
      std::make_unique<SparseQuadraticTermAssembler>(storage->Q, q_vars);
  storage->cm.AddCustomAssembler(std::move(q_asm));

  auto sec = std::make_unique<SparseEqualityConstraint>(C, d);
  std::set<int> eq_set;
  for (const auto& s : sec->row_supports())
    eq_set.insert(s.begin(), s.end());
  std::vector<int> eq_primal(eq_set.begin(), eq_set.end());
  auto dual_vars = storage->cm.AllocateDualVariables(n_eq);
  auto eq_asm = std::make_unique<SparseEqualityConstraintAssembler>(
      std::move(sec), eq_primal, dual_vars);
  storage->cm.AddCustomAssembler(std::move(eq_asm));

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&storage->cm, config);
  int n_vars = storage->cm.SizeOfKKTSystem();

  Result result;
  result.solver = std::move(solver);
  result.num_variables = n_vars;
  result.storage_ = std::move(storage);
  return result;
}

}  // namespace conex
