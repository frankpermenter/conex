#include "conex/algorithms/tree_solver_builder.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <sstream>

#include "conex/common/clique_ordering.h"
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

void TreeSolverBuilder::ComputeEliminationTree() {
  const int C = static_cast<int>(cliques_.size());

  // Build quotient graph: edge weight = number of shared variables.
  // Use var_to_cliques map to only compare clique pairs sharing variables.
  std::unordered_map<int, std::vector<int>> var_to_cliques;
  for (int i = 0; i < C; ++i)
    for (int v : cliques_[i].all_vars)
      var_to_cliques[v].push_back(i);

  // Adjacency: adj[i] = {(j, weight)}. Use map for easy updates.
  std::vector<std::unordered_map<int, int>> adj(C);
  for (auto& [var, clist] : var_to_cliques) {
    for (int a = 0; a < static_cast<int>(clist.size()); ++a)
      for (int b = a + 1; b < static_cast<int>(clist.size()); ++b) {
        adj[clist[a]][clist[b]]++;
        adj[clist[b]][clist[a]]++;
      }
  }

  // Weighted min-degree elimination.
  std::vector<bool> eliminated(C, false);
  std::vector<int> elim_order;
  elim_order.reserve(C);
  std::vector<int> parent(C, -1);

  // Maintain working copies of variable sets for fill propagation.
  std::vector<std::set<int>> work_vars(C);
  for (int i = 0; i < C; ++i) work_vars[i] = cliques_[i].all_vars;

  for (int step = 0; step < C; ++step) {
    // Find clique with minimum weighted degree.
    int best = -1;
    int best_deg = std::numeric_limits<int>::max();
    for (int i = 0; i < C; ++i) {
      if (eliminated[i]) continue;
      int deg = 0;
      for (auto& [nb, w] : adj[i])
        if (!eliminated[nb]) deg += w;
      if (deg < best_deg) {
        best_deg = deg;
        best = i;
      }
    }
    CONEX_DEMAND(best >= 0, "No clique to eliminate.");
    eliminated[best] = true;
    elim_order.push_back(best);

    // Collect uneliminated neighbors.
    std::vector<int> nbrs;
    for (auto& [nb, w] : adj[best])
      if (!eliminated[nb]) nbrs.push_back(nb);

    // Parent = the uneliminated neighbor that will be eliminated next
    // (lowest future weighted degree). Use current degree as heuristic.
    if (!nbrs.empty()) {
      int best_parent = nbrs[0];
      int best_parent_deg = std::numeric_limits<int>::max();
      for (int nb : nbrs) {
        int deg = 0;
        for (auto& [nn, w] : adj[nb])
          if (!eliminated[nn]) deg += w;
        if (deg < best_parent_deg) {
          best_parent_deg = deg;
          best_parent = nb;
        }
      }
      parent[best] = best_parent;
    }

    // Separator of best = variables shared with remaining neighbors.
    std::set<int> separator;
    for (int nb : nbrs) {
      for (int v : work_vars[best]) {
        if (work_vars[nb].count(v)) separator.insert(v);
      }
    }

    // Propagate separator variables to parent for RIP.
    if (parent[best] >= 0) {
      for (int v : separator)
        work_vars[parent[best]].insert(v);
    }

    // Fill: connect all pairs of uneliminated neighbors.
    for (int a = 0; a < static_cast<int>(nbrs.size()); ++a) {
      for (int b = a + 1; b < static_cast<int>(nbrs.size()); ++b) {
        int u = nbrs[a], v = nbrs[b];
        if (adj[u].count(v) == 0) {
          // Fill edge: weight = shared variables through eliminated clique.
          int fill_weight = 0;
          for (int var : work_vars[u])
            if (work_vars[v].count(var)) fill_weight++;
          if (fill_weight > 0) {
            adj[u][v] = fill_weight;
            adj[v][u] = fill_weight;
          }
        }
      }
    }
  }

  // Handle disconnected components: cliques with parent=-1 after AMD
  // (more than one root). Connect extra roots to the last-eliminated root.
  int last_root = elim_order.back();
  for (int i = 0; i < C; ++i) {
    if (i != last_root && parent[i] == -1) {
      parent[i] = last_root;
    }
  }

  // Set parents and rebuild children lists.
  for (int i = 0; i < C; ++i) {
    cliques_[i].parent = parent[i];
    cliques_[i].children.clear();
  }
  for (int i = 0; i < C; ++i) {
    if (parent[i] >= 0)
      cliques_[parent[i]].children.push_back(i);
  }

  // Update all_vars from the fill-propagated working sets.
  // The AMD process propagates separator variables through the tree;
  // Build() needs these in all_vars to compute separators correctly.
  for (int i = 0; i < C; ++i) {
    cliques_[i].all_vars = work_vars[i];
  }
}

TreeSolverBuilder::Result TreeSolverBuilder::Build() {
  const int num_cliques = static_cast<int>(cliques_.size());
  CONEX_DEMAND(num_cliques > 0, "No cliques added.");

  // Count roots.  If all parents are -1, run weighted AMD to compute tree.
  int num_roots = 0;
  for (int i = 0; i < num_cliques; ++i)
    if (cliques_[i].parent == -1) num_roots++;

  if (num_roots == num_cliques && num_cliques > 1) {
    ComputeEliminationTree();
  }

  // Validate: exactly one root after possible AMD.
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

  // Compute fill statistics.
  long long fill = 0;
  int max_cs = 0;
  for (int i = 0; i < num_cliques; ++i) {
    int cs = static_cast<int>(supernodes[i].size() + separators[i].size());
    fill += static_cast<long long>(cs) * cs;
    max_cs = std::max(max_cs, cs);
  }

  Result result;
  result.solver = std::move(solver);
  result.num_variables = num_variables;
  result.num_cliques = num_cliques;
  result.max_clique_size = max_cs;
  result.fill = fill;
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

  // Collect cliques and dual vars for fill measurement.
  std::vector<std::vector<int>> all_cliques;
  std::vector<int> dual_flat;
  for (auto* asm_ptr : storage->cm.clique_assemblers()) {
    auto c = asm_ptr->get_cliques();
    all_cliques.insert(all_cliques.end(), c.begin(), c.end());
    auto dv = asm_ptr->dual_variables();
    dual_flat.insert(dual_flat.end(), dv.begin(), dv.end());
  }

  // Build clique tree for fill statistics (cheap relative to solver build).
  std::vector<std::vector<int>> maximal_cliques;
  auto ctree = MakeCliqueTreeMinDegreeFromRowSupports(
      all_cliques, &maximal_cliques, 0, 0, dual_flat);

  long long fill = 0;
  int max_cs = 0;
  int nc = static_cast<int>(ctree.supernodes.size());
  for (int i = 0; i < nc; ++i) {
    int cs = static_cast<int>(ctree.supernodes[i].size() +
                              ctree.separators[i].size());
    fill += static_cast<long long>(cs) * cs;
    max_cs = std::max(max_cs, cs);
  }

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&storage->cm, config);
  int n_vars = storage->cm.SizeOfKKTSystem();

  Result result;
  result.solver = std::move(solver);
  result.num_variables = n_vars;
  result.num_cliques = nc;
  result.max_clique_size = max_cs;
  result.fill = fill;
  result.storage_ = std::move(storage);
  return result;
}

}  // namespace conex
