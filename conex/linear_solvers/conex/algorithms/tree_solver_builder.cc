#include "conex/algorithms/tree_solver_builder.h"

#include <algorithm>
#include <functional>
#include <sstream>

#include "conex/common/clique_tree.h"
#include "conex/common/error_checking_macros.h"
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
      // Validate RIP: separator vars must be in parent's variable set.
      // (This is guaranteed by construction above, but check explicitly.)
      for (int v : separators[c]) {
        CONEX_DEMAND(parent_vars.count(v),
                     "RIP violation: separator variable not in parent.");
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

  return {std::move(solver), num_variables};
}

}  // namespace conex
