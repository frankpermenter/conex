#include "conex/algorithms/tree_solver_builder.h"

#include <algorithm>
#include <functional>
#include <numeric>
#include <sstream>
#include <unordered_set>

#include "conex/common/clique_ordering.h"
#include "conex/common/clique_tree.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/linear_solvers/kkt_solver_factory.h"
#include "conex/linear_solvers/assembler_adapter.h"

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

void TreeSolverBuilder::AddLinearConstraint(
    int clique, const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
    const std::vector<int>& vars) {
  CONEX_DEMAND(clique >= 0 && clique < static_cast<int>(cliques_.size()),
               "Invalid clique id.");
  CONEX_DEMAND(A.cols() == static_cast<int>(vars.size()),
               "A cols must match vars size.");
  CONEX_DEMAND(A.rows() == b.rows(), "A rows must match b size.");
  cliques_[clique].all_vars.insert(vars.begin(), vars.end());
  linear_assemblers_.emplace_back(A, b);
  linear_assemblers_.back().SetPrimalVariables(vars);
  // Workspace is arena-allocated in Build().
  pending_.push_back(
      {&linear_assemblers_.back(), clique, ContributionType::kPositiveDefinite});
}

void TreeSolverBuilder::AddPSDConstraint(
    int clique, std::unique_ptr<ConeConstraint> constraint,
    const std::vector<int>& vars) {
  CONEX_DEMAND(clique >= 0 && clique < static_cast<int>(cliques_.size()),
               "Invalid clique id.");
  cliques_[clique].all_vars.insert(vars.begin(), vars.end());
  constraint->SetPrimalVariables(vars);
  pending_.push_back(
      {constraint.get(), clique, ContributionType::kPositiveDefinite});
  owned_constraints_.push_back(std::move(constraint));
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

EliminationOrdering TreeSolverBuilder::ComputeQuotientAMDOrdering() {
  const int NC = static_cast<int>(cliques_.size());

  // --- Step 1: Weighted AMD on the clique quotient graph ---

  // Build quotient graph edges from shared variables.
  std::unordered_map<int, std::vector<int>> var_to_cliques;
  for (int i = 0; i < NC; ++i)
    for (int v : cliques_[i].all_vars)
      var_to_cliques[v].push_back(i);

  std::vector<std::unordered_map<int, int>> adj(NC);
  for (auto& [var, clist] : var_to_cliques)
    for (int a = 0; a < static_cast<int>(clist.size()); ++a)
      for (int b = a + 1; b < static_cast<int>(clist.size()); ++b) {
        adj[clist[a]][clist[b]]++;
        adj[clist[b]][clist[a]]++;
      }

  // Weighted min-degree with exclusive-var tie-breaking.
  std::vector<bool> eliminated(NC, false);
  std::vector<int> clique_order;
  clique_order.reserve(NC);

  for (int step = 0; step < NC; ++step) {
    int best = -1, best_deg = std::numeric_limits<int>::max();
    int best_exclusive = -1;
    for (int i = 0; i < NC; ++i) {
      if (eliminated[i]) continue;
      int deg = 0;
      for (auto& [nb, w] : adj[i])
        if (!eliminated[nb]) deg += w;
      int exclusive = 0;
      for (int v : cliques_[i].all_vars) {
        bool shared = false;
        for (auto& [nb, w] : adj[i])
          if (!eliminated[nb] && cliques_[nb].all_vars.count(v)) {
            shared = true; break;
          }
        if (!shared) exclusive++;
      }
      if (deg < best_deg || (deg == best_deg && exclusive > best_exclusive)) {
        best_deg = deg; best_exclusive = exclusive; best = i;
      }
    }
    eliminated[best] = true;
    clique_order.push_back(best);

    // Fill: connect uneliminated neighbors.
    std::vector<int> nbrs;
    for (auto& [nb, w] : adj[best])
      if (!eliminated[nb]) nbrs.push_back(nb);
    for (int a = 0; a < static_cast<int>(nbrs.size()); ++a)
      for (int b = a + 1; b < static_cast<int>(nbrs.size()); ++b) {
        int u = nbrs[a], v = nbrs[b];
        if (!adj[u].count(v)) {
          int fw = 0;
          for (int var : cliques_[u].all_vars)
            if (cliques_[v].all_vars.count(var)) fw++;
          if (fw > 0) { adj[u][v] = fw; adj[v][u] = fw; }
        }
      }
  }

  // --- Step 2: Convert clique order to variable elimination order ---
  // Within each clique, eliminate exclusive vars first (supernode candidates),
  // then shared vars (separator candidates, will be re-eliminated in parent).
  // Only emit each variable once (first occurrence).

  // Collect variables that participate in PD blocks (cost or A'A).
  std::unordered_set<int> has_pd;
  for (const auto& pa : pending_) {
    if (pa.type == ContributionType::kPositiveDefinite) {
      for (int v : pa.assembler->primal_variables())
        has_pd.insert(v);
    }
  }

  // Build clique step map for "is later" checks.
  std::vector<int> clique_step(NC);
  for (int s = 0; s < NC; ++s) clique_step[clique_order[s]] = s;

  std::set<int> emitted;
  std::vector<int> var_order;
  std::set<int> all_var_set;
  for (const auto& ci : cliques_) all_var_set.insert(ci.all_vars.begin(), ci.all_vars.end());
  var_order.reserve(all_var_set.size());

  for (int ci : clique_order) {
    // Partition this clique's unemitted variables into:
    //   pd_exclusive: PD vars not in any later clique (eliminated here)
    //   pd_shared:    PD vars in some later clique (separator candidates)
    //   non_pd:       vars without PD contribution (duals, uncovered primals)
    // Order: pd_exclusive, pd_shared, non_pd.
    // This ensures PD vars appear before non-PD vars in the elimination
    // order, so indefinite blocks see nonzero pivots from PD neighbors.
    std::vector<int> pd_exclusive, pd_shared, non_pd;
    for (int v : cliques_[ci].all_vars) {
      if (emitted.count(v)) continue;
      if (!has_pd.count(v)) {
        non_pd.push_back(v);
        continue;
      }
      bool in_later = false;
      for (int j = 0; j < NC; ++j) {
        if (j == ci && clique_step[j] > clique_step[ci] &&
            cliques_[j].all_vars.count(v)) {
          in_later = true; break;
        }
        if (j != ci && clique_step[j] > clique_step[ci] &&
            cliques_[j].all_vars.count(v)) {
          in_later = true; break;
        }
      }
      if (in_later) pd_shared.push_back(v);
      else pd_exclusive.push_back(v);
    }
    for (int v : pd_exclusive) { var_order.push_back(v); emitted.insert(v); }
    for (int v : pd_shared) { var_order.push_back(v); emitted.insert(v); }
    for (int v : non_pd) { var_order.push_back(v); emitted.insert(v); }
  }
  for (int v : all_var_set)
    if (!emitted.count(v)) { var_order.push_back(v); emitted.insert(v); }

  // --- Step 3: Build variable adjacency graph ---
  // Each clique's all_vars forms a clique in the variable graph.
  const int n = static_cast<int>(var_order.size());

  // Compact variable indices.
  std::unordered_map<int, int> to_compact;
  to_compact.reserve(n);
  std::vector<int> unique_vars(n);
  for (int i = 0; i < n; ++i) {
    to_compact[var_order[i]] = i;  // but we need elimination order, not original
    unique_vars[i] = var_order[i];
  }

  // Actually, the compact space should be indexed by elimination position.
  // var_order[k] = original var eliminated at position k.
  // to_compact[orig_var] = elimination position.
  // But EliminationOrdering expects compact indices 0..n-1 with order[k] = compact var at position k.
  // And unique_vars[compact] = original var.

  // Let me redo: use the var_order as the elimination order.
  // compact index i corresponds to unique_vars[i] = var_order[i].
  // order[k] = k (identity, since we've already sorted by elimination position).
  // No — order[k] should be the compact index of the k-th eliminated var.
  // Since compact index = position in unique_vars = position in var_order = k,
  // order = {0, 1, 2, ..., n-1}.

  // Build row supports in compact space (each clique = one row support).
  std::vector<std::vector<int>> supports_compact;
  for (const auto& ci : cliques_) {
    std::vector<int> sup;
    for (int v : ci.all_vars) {
      auto it = to_compact.find(v);
      if (it != to_compact.end()) sup.push_back(it->second);
    }
    std::sort(sup.begin(), sup.end());
    if (!sup.empty()) supports_compact.push_back(std::move(sup));
  }

  // --- Step 4: Symbolic elimination to produce later sets ---
  // Use bitset adjacency like Phase 1 of clique_ordering_md.cc.
  const int words = (n + 63) / 64;
  std::vector<uint64_t> var_adj(static_cast<size_t>(n) * words, 0);
  auto row = [&](int i) -> uint64_t* {
    return &var_adj[static_cast<size_t>(i) * words];
  };

  // Populate adjacency from row supports.
  for (const auto& sup : supports_compact)
    for (size_t i = 0; i < sup.size(); ++i)
      for (size_t j = i + 1; j < sup.size(); ++j) {
        int u = sup[i], v = sup[j];
        row(u)[v >> 6] |= (1ULL << (v & 63));
        row(v)[u >> 6] |= (1ULL << (u & 63));
      }

  // Eliminate in the var_order (which is 0, 1, ..., n-1 in compact space).
  EliminationOrdering elim;
  elim.order.resize(n);
  elim.later.resize(n);
  elim.parent_col.assign(n, -1);
  elim.unique_vars = unique_vars;

  std::vector<int> nbrs_buf;
  std::vector<uint64_t> clique_mask(words);

  for (int k = 0; k < n; ++k) {
    int v = k;  // compact var at elimination position k
    elim.order[k] = v;

    // Get living neighbors.
    auto* rv = row(v);
    nbrs_buf.clear();
    for (int w = 0; w < words; ++w) {
      uint64_t b = rv[w];
      while (b) {
        int bit = __builtin_ctzll(b);
        int u = (w << 6) + bit;
        if (u < n) nbrs_buf.push_back(u);
        b &= b - 1;
      }
    }
    elim.later[v] = nbrs_buf;

    // Make neighbors a clique (fill).
    std::memset(clique_mask.data(), 0, words * sizeof(uint64_t));
    for (int u : nbrs_buf) clique_mask[u >> 6] |= (1ULL << (u & 63));
    for (int u : nbrs_buf) {
      auto* ru = row(u);
      const uint64_t self_bit = 1ULL << (u & 63);
      const int self_word = u >> 6;
      for (int w = 0; w < words; ++w) {
        uint64_t target = clique_mask[w];
        if (w == self_word) target &= ~self_bit;
        ru[w] |= target;
      }
    }

    // Remove v from graph.
    const uint64_t v_bit = 1ULL << (v & 63);
    const int v_word = v >> 6;
    for (int u : nbrs_buf) row(u)[v_word] &= ~v_bit;
    std::memset(rv, 0, words * sizeof(uint64_t));
  }

  return elim;
}

TreeSolverBuilder::Result TreeSolverBuilder::Build() {
  int num_cliques = static_cast<int>(cliques_.size());
  CONEX_DEMAND(num_cliques > 0, "No cliques added.");

  // Count roots.  If all parents are -1, run weighted AMD to compute tree.
  int num_roots = 0;
  for (int i = 0; i < num_cliques; ++i)
    if (cliques_[i].parent == -1) num_roots++;

  // Two paths: quotient AMD (auto tree) or explicit tree.
  bool use_quotient_amd = (num_roots == num_cliques && num_cliques > 1);

  CliqueTree tree;
  std::vector<std::vector<int>> maximal_cliques;
  int num_variables = 0;

  if (use_quotient_amd) {
    // Quotient AMD: compute variable ordering, then use Phase 2 to get
    // proper maximal cliques with correct fill handling.
    auto elim = ComputeQuotientAMDOrdering();
    tree = MakeCliqueTreeFromEliminationOrdering(
        elim, &maximal_cliques, 0, SUPERNODE_REORDER_BFS_GREEDY);
    num_variables = elim.unique_vars.empty()
        ? 0 : (*std::max_element(elim.unique_vars.begin(),
                                  elim.unique_vars.end()) + 1);
  } else {
    // Explicit tree: validate and compute separators directly.
    CONEX_DEMAND(num_roots == 1, "Multiple roots found (use all parent=-1 for auto tree).");
    int root = -1;
    for (int i = 0; i < num_cliques; ++i)
      if (cliques_[i].parent == -1) root = i;

    std::vector<int> post_order;
    post_order.reserve(num_cliques);
    std::function<void(int)> dfs = [&](int node) {
      for (int child : cliques_[node].children) dfs(child);
      post_order.push_back(node);
    };
    dfs(root);
    CONEX_DEMAND(static_cast<int>(post_order.size()) == num_cliques,
                 "Tree is disconnected.");

    std::vector<std::vector<int>> supernodes(num_cliques);
    std::vector<std::vector<int>> separators(num_cliques);
    for (int c : post_order) {
      int p = cliques_[c].parent;
      if (p == -1) {
        supernodes[c].assign(cliques_[c].all_vars.begin(),
                             cliques_[c].all_vars.end());
      } else {
        const auto& pv = cliques_[p].all_vars;
        for (int v : cliques_[c].all_vars) {
          if (pv.count(v)) separators[c].push_back(v);
          else supernodes[c].push_back(v);
        }
      }
    }

    if (check_rip_) {
      CONEX_DEMAND(tree.CheckRunningIntersectionProperty(),
                   "Running intersection property violated.");
    }

    std::set<int> all_vars;
    for (const auto& ci : cliques_)
      all_vars.insert(ci.all_vars.begin(), ci.all_vars.end());
    num_variables = all_vars.empty() ? 0 : (*all_vars.rbegin() + 1);

    tree.supernodes = supernodes;
    tree.separators = separators;
    tree.node_to_parent.resize(num_cliques);
    for (int i = 0; i < num_cliques; ++i)
      tree.node_to_parent[i] = cliques_[i].parent;
    tree.post_order_position_to_clique = post_order;
  }

  // Arena-allocate LinearConstraint workspaces.
  {
    size_t total = 0;
    for (auto& lc : linear_assemblers_)
      total += lc.RequiredArenaBytes();
    for (auto& lc : owned_constraints_)
      total += lc->RequiredArenaBytes();
    if (total > 0) {
      workspace_arena_.resize(total / sizeof(double) + 1);
      double* cursor = workspace_arena_.data();
      for (auto& lc : linear_assemblers_) {
        size_t bytes = lc.RequiredArenaBytes();
        lc.BindArenaMemory(cursor, bytes);
        cursor += bytes / sizeof(double);
      }
      for (auto& lc : owned_constraints_) {
        size_t bytes = lc->RequiredArenaBytes();
        lc->BindArenaMemory(cursor, bytes);
        cursor += bytes / sizeof(double);
      }
    }
  }

  // Build solver.
  auto solver = std::make_unique<SymmetricLinearSystemTreeSolver>();
  for (auto& pa : pending_) {
    auto adapter =
        std::make_unique<AssemblerAdapter>(pa.assembler);
    adapter->set_contribution_type(pa.type);
    solver->push_back(std::move(adapter));
  }

  solver->SetUseGenericFactorization(false);
  solver->SetUseLUForIndefinite(false);
  solver->FinalizeStructure(tree);
  solver->SetFactorizationMode(true);
  solver->EnableAutoUpdateAtAssemble(true);

  // Compute fill statistics from the final clique tree.
  long long fill = 0;
  int max_cs = 0;
  int final_num_cliques = static_cast<int>(tree.supernodes.size());
  for (int i = 0; i < final_num_cliques; ++i) {
    int cs = static_cast<int>(tree.supernodes[i].size() +
                              tree.separators[i].size());
    fill += static_cast<long long>(cs) * cs;
    max_cs = std::max(max_cs, cs);
  }

  Result result;
  result.solver = std::move(solver);
  result.clique_tree = tree;
  result.num_variables = num_variables;
  result.num_cliques = final_num_cliques;
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

}  // namespace conex
