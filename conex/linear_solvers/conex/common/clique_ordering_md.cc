#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Sparse>


#include "conex/common/clique_ordering.h"
#include "conex/common/pq_tree.h"

namespace conex {
namespace {

// Scan set bits out of a bitset row into a sorted vector.
void BitsToVec(const uint64_t* bits, int words, int n, std::vector<int>* out) {
  out->clear();
  for (int w = 0; w < words; w++) {
    uint64_t b = bits[w];
    while (b) {
      int bit = __builtin_ctzll(b);
      int v = (w << 6) + bit;
      if (v < n) out->push_back(v);
      b &= b - 1;
    }
  }
}

// popcount of a bitset row.
int Popcount(const uint64_t* bits, int words) {
  int c = 0;
  for (int w = 0; w < words; w++) c += __builtin_popcountll(bits[w]);
  return c;
}


struct TreeAndCliques {
  CliqueTree tree;
  std::vector<std::vector<int>> cliques;
};

#if CONEX_MD_HAS_CHOLMOD
SuiteSparse_long ReadCholmodIndex(const void* base, size_t idx, int itype) {
  if (itype == CHOLMOD_LONG) {
    return static_cast<const SuiteSparse_long*>(base)[idx];
  }
  if (itype == CHOLMOD_INT) {
    return static_cast<SuiteSparse_long>(static_cast<const int*>(base)[idx]);
  }
  return -1;
}

TreeAndCliques BuildFromCholmodReference(
    const std::vector<std::vector<int>>& supports_compact,
    const std::vector<int>& unique_vars, int n) {
  using ColSparse = Eigen::SparseMatrix<double, Eigen::ColMajor>;
  std::vector<Eigen::Triplet<double>> triplets;
  for (const auto& sup : supports_compact) {
    for (size_t i = 0; i < sup.size(); ++i) {
      for (size_t j = i; j < sup.size(); ++j) {
        const int a = sup[i];
        const int b = sup[j];
        const int r = std::max(a, b);
        const int c = std::min(a, b);
        triplets.emplace_back(r, c, 1.0);
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    triplets.emplace_back(i, i, 1e-8);
  }
  ColSparse ata_lower(n, n);
  ata_lower.setFromTriplets(triplets.begin(), triplets.end());
  ata_lower.makeCompressed();

  struct CholmodSession {
    cholmod_common common;
    cholmod_sparse* M = nullptr;
    cholmod_factor* L = nullptr;
    CholmodSession() { cholmod_l_start(&common); }
    ~CholmodSession() {
      if (L != nullptr) cholmod_l_free_factor(&L, &common);
      if (M != nullptr) cholmod_l_free_sparse(&M, &common);
      cholmod_l_finish(&common);
    }
  } session;
  session.common.supernodal = 2;

  const SuiteSparse_long nnz = static_cast<SuiteSparse_long>(ata_lower.nonZeros());
  session.M = cholmod_l_allocate_sparse(static_cast<size_t>(n), static_cast<size_t>(n),
                                        static_cast<size_t>(nnz), 1, 1, -1,
                                        CHOLMOD_REAL, &session.common);
  if (session.M == nullptr) return {};

  std::vector<SuiteSparse_long> p(static_cast<size_t>(n) + 1);
  std::vector<SuiteSparse_long> i(static_cast<size_t>(nnz));
  for (int col = 0; col <= n; ++col) {
    p[static_cast<size_t>(col)] =
        static_cast<SuiteSparse_long>(ata_lower.outerIndexPtr()[col]);
  }
  for (SuiteSparse_long k = 0; k < nnz; ++k) {
    i[static_cast<size_t>(k)] =
        static_cast<SuiteSparse_long>(ata_lower.innerIndexPtr()[k]);
  }
  std::memcpy(session.M->p, p.data(), static_cast<size_t>(n + 1) * sizeof(SuiteSparse_long));
  std::memcpy(session.M->i, i.data(), static_cast<size_t>(nnz) * sizeof(SuiteSparse_long));
  std::memcpy(session.M->x, ata_lower.valuePtr(), static_cast<size_t>(nnz) * sizeof(double));

  session.L = cholmod_l_analyze(session.M, &session.common);
  if (session.L == nullptr) return {};
  double beta[2] = {0.0, 0.0};
  if (!cholmod_l_factorize_p(session.M, beta, nullptr, 0, session.L, &session.common)) {
    return {};
  }
  if (!session.L->is_super) return {};

  const int nsuper = static_cast<int>(session.L->nsuper);
  std::vector<SuiteSparse_long> perm(static_cast<size_t>(n));
  std::vector<SuiteSparse_long> super(static_cast<size_t>(nsuper) + 1);
  std::vector<SuiteSparse_long> pi(static_cast<size_t>(nsuper) + 1);
  std::vector<SuiteSparse_long> s(static_cast<size_t>(session.L->ssize));
  for (int k = 0; k < n; ++k) {
    perm[static_cast<size_t>(k)] =
        ReadCholmodIndex(session.L->Perm, static_cast<size_t>(k), session.L->itype);
  }
  for (int k = 0; k <= nsuper; ++k) {
    super[static_cast<size_t>(k)] =
        ReadCholmodIndex(session.L->super, static_cast<size_t>(k), session.L->itype);
    pi[static_cast<size_t>(k)] =
        ReadCholmodIndex(session.L->pi, static_cast<size_t>(k), session.L->itype);
  }
  for (size_t k = 0; k < s.size(); ++k) {
    s[k] = ReadCholmodIndex(session.L->s, k, session.L->itype);
  }

  std::vector<int> col_to_super(static_cast<size_t>(n), -1);
  for (int sn = 0; sn < nsuper; ++sn) {
    for (SuiteSparse_long c = super[static_cast<size_t>(sn)];
         c < super[static_cast<size_t>(sn + 1)]; ++c) {
      if (c >= 0 && c < n) col_to_super[static_cast<size_t>(c)] = sn;
    }
  }

  TreeAndCliques out;
  out.cliques.resize(static_cast<size_t>(nsuper));
  out.tree.node_to_parent.assign(static_cast<size_t>(nsuper), -1);
  out.tree.supernodes.resize(static_cast<size_t>(nsuper));
  out.tree.separators.resize(static_cast<size_t>(nsuper));

  for (int sn = 0; sn < nsuper; ++sn) {
    const SuiteSparse_long p0 = pi[static_cast<size_t>(sn)];
    const SuiteSparse_long p1 = pi[static_cast<size_t>(sn + 1)];
    const SuiteSparse_long nscol =
        super[static_cast<size_t>(sn + 1)] - super[static_cast<size_t>(sn)];
    int parent = -1;
    if (p0 + nscol < p1) {
      const SuiteSparse_long parent_col = s[static_cast<size_t>(p0 + nscol)];
      if (parent_col >= 0 && parent_col < n) {
        parent = col_to_super[static_cast<size_t>(parent_col)];
      }
    }
    out.tree.node_to_parent[static_cast<size_t>(sn)] = parent;

    auto& bag = out.cliques[static_cast<size_t>(sn)];
    bag.reserve(static_cast<size_t>(p1 - p0));
    for (SuiteSparse_long t = p0; t < p1; ++t) {
      const SuiteSparse_long c = s[static_cast<size_t>(t)];
      if (c < 0 || c >= n) continue;
      const SuiteSparse_long v = perm[static_cast<size_t>(c)];
      if (v < 0 || v >= n) continue;
      bag.push_back(unique_vars[static_cast<size_t>(v)]);
    }
    std::sort(bag.begin(), bag.end());
    bag.erase(std::unique(bag.begin(), bag.end()), bag.end());
  }

  for (int sn = 0; sn < nsuper; ++sn) {
    const int pnode = out.tree.node_to_parent[static_cast<size_t>(sn)];
    if (pnode >= 0) {
      std::set_intersection(
          out.cliques[static_cast<size_t>(sn)].begin(),
          out.cliques[static_cast<size_t>(sn)].end(),
          out.cliques[static_cast<size_t>(pnode)].begin(),
          out.cliques[static_cast<size_t>(pnode)].end(),
          std::back_inserter(out.tree.separators[static_cast<size_t>(sn)]));
    }
    std::set_difference(
        out.cliques[static_cast<size_t>(sn)].begin(),
        out.cliques[static_cast<size_t>(sn)].end(),
        out.tree.separators[static_cast<size_t>(sn)].begin(),
        out.tree.separators[static_cast<size_t>(sn)].end(),
        std::back_inserter(out.tree.supernodes[static_cast<size_t>(sn)]));
  }

  std::vector<std::vector<int>> children(static_cast<size_t>(nsuper));
  std::vector<int> roots;
  for (int i = 0; i < nsuper; ++i) {
    const int pnode = out.tree.node_to_parent[static_cast<size_t>(i)];
    if (pnode >= 0) children[static_cast<size_t>(pnode)].push_back(i);
    else roots.push_back(i);
  }
  for (auto& c : children) std::sort(c.begin(), c.end());
  std::sort(roots.begin(), roots.end());
  out.tree.post_order_position_to_clique.clear();
  out.tree.post_order_position_to_clique.reserve(static_cast<size_t>(nsuper));
  std::vector<int> stk;
  std::vector<char> expanded(static_cast<size_t>(nsuper), 0);
  for (int root : roots) {
    stk.clear();
    stk.push_back(root);
    while (!stk.empty()) {
      const int node = stk.back();
      if (!expanded[static_cast<size_t>(node)]) {
        expanded[static_cast<size_t>(node)] = 1;
        for (int c : children[static_cast<size_t>(node)]) stk.push_back(c);
      } else {
        stk.pop_back();
        out.tree.post_order_position_to_clique.push_back(node);
      }
    }
  }
  return out;
}
#endif

// Reorder supernodes/separators to maximize scatter block alignment.
// Top-down: for each parent, reorder its supernode so that variables
// appearing in each child's separator form contiguous blocks.  Then
// reorder each child's separator to list parent-supernode variables first
// (in parent's supernode order), then parent-separator variables (in
// parent's separator order).  This guarantees at most 2 scatter blocks
// per parent-child edge (1 for supernode, 1 for separator).
void ReorderSupernodes(CliqueTree& ct, int supernode_reorder_method) {
  if (supernode_reorder_method == SUPERNODE_REORDER_NONE) return;

  const int nk = static_cast<int>(ct.supernodes.size());
  std::vector<std::vector<int>> ch(nk);
  std::vector<int> roots;
  for (int i = 0; i < nk; ++i) {
    if (ct.node_to_parent[i] >= 0)
      ch[ct.node_to_parent[i]].push_back(i);
    else
      roots.push_back(i);
  }

  // BFS top-down.
  std::vector<int> bfs;
  bfs.reserve(nk);
  for (int r : roots) bfs.push_back(r);
  for (size_t qi = 0; qi < bfs.size(); ++qi) {
    int p = bfs[qi];
    for (int c : ch[p]) bfs.push_back(c);
  }

  for (int p : bfs) {
    auto& sn = ct.supernodes[p];

    if (supernode_reorder_method == SUPERNODE_REORDER_PQ_TREE) {
      // PQ-tree reorder: build constraints from child separators,
      // find a permutation where all children's subsets are contiguous.
      std::map<int, int> var_to_idx;
      for (int i = 0; i < static_cast<int>(sn.size()); ++i) {
        var_to_idx[sn[i]] = i;
      }
      std::vector<std::vector<int>> constraints;
      for (int c : ch[p]) {
        std::vector<int> constraint;
        for (int v : ct.separators[c]) {
          auto it = var_to_idx.find(v);
          if (it != var_to_idx.end()) constraint.push_back(it->second);
        }
        if (!constraint.empty()) constraints.push_back(std::move(constraint));
      }
      if (!constraints.empty()) {
        PQTree pqt(static_cast<int>(sn.size()));
        pqt.AddConstraintsBestEffort(constraints);
        auto perm = pqt.GetPermutation();
        std::vector<int> new_sn(sn.size());
        for (size_t i = 0; i < perm.size(); ++i) {
          new_sn[i] = sn[perm[i]];
        }
        sn = std::move(new_sn);
      }
    } else {
      // BFS-greedy reorder: place variables requested by children first,
      // grouped per child.
      std::set<int> sn_set(sn.begin(), sn.end());

      // Sort children by intersection size.
      std::vector<int> ordered_ch(ch[p].begin(), ch[p].end());
      std::sort(ordered_ch.begin(), ordered_ch.end(),
                [&](int a, int b) {
        int ca = 0, cb = 0;
        for (int v : ct.separators[a]) ca += sn_set.count(v);
        for (int v : ct.separators[b]) cb += sn_set.count(v);
        return (supernode_reorder_method == SUPERNODE_REORDER_BFS_GREEDY_LARGEST)
                   ? ca > cb   // largest first
                   : ca < cb;  // smallest first
      });

      std::vector<int> new_sn;
      new_sn.reserve(sn.size());
      std::set<int> placed;

      for (int c : ordered_ch) {
        for (int v : ct.separators[c]) {
          if (sn_set.count(v) && !placed.count(v)) {
            new_sn.push_back(v);
            placed.insert(v);
          }
        }
      }
      for (int v : sn) {
        if (!placed.count(v)) new_sn.push_back(v);
      }
      sn = std::move(new_sn);
    }

    // Reorder each child's separator: parent-supernode vars in parent's
    // supernode order, then parent-separator vars in parent's separator
    // order.
    const auto& sep = ct.separators[p];
    for (int c : ch[p]) {
      std::set<int> csep_set(ct.separators[c].begin(),
                             ct.separators[c].end());
      std::vector<int> new_csep;
      new_csep.reserve(ct.separators[c].size());
      for (int v : sn) {
        if (csep_set.count(v)) new_csep.push_back(v);
      }
      for (int v : sep) {
        if (csep_set.count(v)) new_csep.push_back(v);
      }
      ct.separators[c] = std::move(new_csep);
    }
  }
}
}  // namespace

namespace {
// Internal implementation that optionally accepts a sparse matrix for
// pairwise edges (from Q's sparsity pattern).
CliqueTree MakeCliqueTreeImpl(
    const std::vector<std::vector<int>>& row_supports,
    const Eigen::SparseMatrix<double>* Q_sparsity,
    std::vector<std::vector<int>>* maximal_cliques_out,
    int max_merge_supernode_size,
    int supernode_reorder_method,
    const std::vector<int>& delayed_variables) {
  // --- Compact variable indices to [0, n) ---
  std::vector<int> unique_vars;
  for (const auto& row : row_supports) {
    for (int v : row) {
      if (v >= 0) unique_vars.push_back(v);
    }
  }
  // Also collect variables from Q's sparsity pattern.
  if (Q_sparsity) {
    for (int k = 0; k < Q_sparsity->outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(*Q_sparsity, k);
           it; ++it) {
        if (it.row() >= 0) unique_vars.push_back(it.row());
        if (it.col() >= 0) unique_vars.push_back(it.col());
      }
    }
  }
  std::sort(unique_vars.begin(), unique_vars.end());
  unique_vars.erase(std::unique(unique_vars.begin(), unique_vars.end()),
                    unique_vars.end());
  const int n = static_cast<int>(unique_vars.size());

  if (n == 0) {
    if (maximal_cliques_out) maximal_cliques_out->clear();
    return CliqueTree{};
  }

  std::unordered_map<int, int> to_compact;
  to_compact.reserve(n);
  for (int i = 0; i < n; i++) to_compact[unique_vars[i]] = i;

  std::vector<std::vector<int>> supports_compact;
  supports_compact.reserve(row_supports.size());
  for (const auto& row : row_supports) {
    std::vector<int> sup;
    sup.reserve(row.size());
    for (int v : row) {
      auto it = to_compact.find(v);
      if (it != to_compact.end()) sup.push_back(it->second);
    }
    std::sort(sup.begin(), sup.end());
    sup.erase(std::unique(sup.begin(), sup.end()), sup.end());
    if (!sup.empty()) supports_compact.push_back(std::move(sup));
  }

  // Map dual variables to compact indices.  Use an unordered_set for O(1)
  // lookup — efficient when few variables are dual relative to n.
  std::unordered_set<int> is_delayed;
  is_delayed.reserve(delayed_variables.size());
  for (int v : delayed_variables) {
    auto it = to_compact.find(v);
    if (it != to_compact.end()) is_delayed.insert(it->second);
  }

  const int words = (n + 63) / 64;

  // ===================================================================
  // Phase 1: Bitset min-degree elimination
  // ===================================================================

  // Working adjacency (shrinks as vertices are eliminated)
  std::vector<uint64_t> adj(static_cast<size_t>(n) * words, 0);
  auto row = [&](int i) -> uint64_t* {
    return &adj[static_cast<size_t>(i) * words];
  };

  for (const auto& sup : supports_compact) {
    for (size_t i = 0; i < sup.size(); i++)
      for (size_t j = i + 1; j < sup.size(); j++) {
        int u = sup[i], v = sup[j];
        row(u)[v >> 6] |= (1ULL << (v & 63));
        row(v)[u >> 6] |= (1ULL << (u & 63));
      }
  }

  // Add individual edges from Q's sparsity pattern.
  if (Q_sparsity) {
    for (int k = 0; k < Q_sparsity->outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(*Q_sparsity, k);
           it; ++it) {
        auto it_u = to_compact.find(it.row());
        auto it_v = to_compact.find(it.col());
        if (it_u != to_compact.end() && it_v != to_compact.end()) {
          int u = it_u->second, v = it_v->second;
          if (u != v) {
            row(u)[v >> 6] |= (1ULL << (v & 63));
            row(v)[u >> 6] |= (1ULL << (u & 63));
          }
        }
      }
    }
  }

  std::vector<int> deg(n);
  for (int v = 0; v < n; v++) deg[v] = Popcount(row(v), words);

  std::vector<int> order;
  order.reserve(n);

  // Build "later" sets directly during elimination instead of from bitsets.
  std::vector<std::vector<int>> later(static_cast<size_t>(n));
  std::vector<int> parent_col(static_cast<size_t>(n), -1);
  std::vector<int> child_count(static_cast<size_t>(n), 0);

  std::vector<uint64_t> clique_mask(words);
  std::vector<int> nbrs;

  // Track whether each vertex has at least one eliminated neighbor.
  // Updated incrementally: when vertex `best` is eliminated, all its
  // living neighbors get their flag set.
  std::vector<char> has_eliminated_neighbor(n, 0);

  using Clock = std::chrono::high_resolution_clock;
  double t_scan = 0, t_bits = 0, t_fillin = 0, t_remove = 0;

  for (int step = 0; step < n; step++) {
    auto t0 = Clock::now();
    int best = -1, best_deg = std::numeric_limits<int>::max();
    for (int v = 0; v < n; v++) {
      if (deg[v] < 0) continue;
      // A dual variable must have a neighbor eliminated first.
      if (is_delayed.count(v) && !has_eliminated_neighbor[v]) continue;
      if (deg[v] < best_deg) {
        best_deg = deg[v];
        best = v;
      }
    }
    auto t1 = Clock::now();

    order.push_back(best);

    auto* rb = row(best);
    BitsToVec(rb, words, n, &nbrs);
    auto t2 = Clock::now();

    // Mark living neighbors as having an eliminated neighbor.
    if (!is_delayed.empty()) {
      for (int u : nbrs) has_eliminated_neighbor[u] = 1;
    }

    // Build later[best] = living neighbors (exactly those eliminated after best).
    later[static_cast<size_t>(best)] = nbrs;

    // Build clique mask from neighbors
    std::memset(clique_mask.data(), 0, words * sizeof(uint64_t));
    for (int u : nbrs) clique_mask[u >> 6] |= (1ULL << (u & 63));

    // Make neighbors a clique via word-level OR
    for (int u : nbrs) {
      auto* ru = row(u);
      const uint64_t self_bit = 1ULL << (u & 63);
      const int self_word = u >> 6;
      for (int w = 0; w < words; w++) {
        uint64_t target = clique_mask[w];
        if (w == self_word) target &= ~self_bit;
        uint64_t new_bits = target & ~ru[w];
        if (new_bits) {
          deg[u] += __builtin_popcountll(new_bits);
          ru[w] |= new_bits;
        }
      }
    }
    auto t3 = Clock::now();

    // Remove best from working graph
    const uint64_t best_bit = 1ULL << (best & 63);
    const int best_word = best >> 6;
    for (int u : nbrs) {
      row(u)[best_word] &= ~best_bit;
      deg[u]--;
    }
    std::memset(rb, 0, words * sizeof(uint64_t));
    deg[best] = -1;  // mark eliminated
    auto t4 = Clock::now();

    t_scan += std::chrono::duration<double, std::micro>(t1 - t0).count();
    t_bits += std::chrono::duration<double, std::micro>(t2 - t1).count();
    t_fillin += std::chrono::duration<double, std::micro>(t3 - t2).count();
    t_remove += std::chrono::duration<double, std::micro>(t4 - t3).count();
  }

  fprintf(stderr, "  MakeCliqueTreeImpl phase1 (n=%d): scan=%.0fus bits=%.0fus "
          "fillin=%.0fus remove=%.0fus total=%.0fus\n",
          n, t_scan, t_bits, t_fillin, t_remove,
          t_scan + t_bits + t_fillin + t_remove);

  // Package Phase 1 results for Phase 2.
  EliminationOrdering elim;
  elim.order = std::move(order);
  elim.later = std::move(later);
  elim.parent_col = std::move(parent_col);
  elim.unique_vars = std::move(unique_vars);

  return MakeCliqueTreeFromEliminationOrdering(
      elim, maximal_cliques_out, max_merge_supernode_size,
      supernode_reorder_method);
}

}  // anonymous namespace

CliqueTree MakeCliqueTreeFromEliminationOrdering(
    const EliminationOrdering& elim,
    std::vector<std::vector<int>>* maximal_cliques_out,
    int max_merge_supernode_size,
    int supernode_reorder_method) {
  const auto& order = elim.order;
  const auto& unique_vars = elim.unique_vars;
  const int n = static_cast<int>(order.size());

  if (n == 0) {
    if (maximal_cliques_out) maximal_cliques_out->clear();
    return CliqueTree{};
  }

  // Sort later sets by elimination position and compute parent_col / child_count.
  std::vector<std::vector<int>> later = elim.later;
  std::vector<int> parent_col = elim.parent_col;
  std::vector<int> child_count(static_cast<size_t>(n), 0);

  std::vector<int> pos(static_cast<size_t>(n), -1);
  for (int k = 0; k < n; ++k)
    pos[static_cast<size_t>(order[static_cast<size_t>(k)])] = k;

  for (int k = 0; k < n; ++k) {
    const int v = order[static_cast<size_t>(k)];
    auto& lv = later[static_cast<size_t>(v)];
    std::sort(lv.begin(), lv.end(),
              [&](int a, int b) { return pos[static_cast<size_t>(a)] < pos[static_cast<size_t>(b)]; });
    if (!lv.empty()) {
      parent_col[static_cast<size_t>(v)] = lv.front();
      child_count[static_cast<size_t>(lv.front())]++;
    }
  }

  // Supernode detection.
  std::vector<int> col_to_super(static_cast<size_t>(n), -1);
  std::vector<std::vector<int>> super_cols;
  super_cols.reserve(static_cast<size_t>(n));
  for (int k = 0; k < n; ++k) {
    const int start = order[static_cast<size_t>(k)];
    if (col_to_super[static_cast<size_t>(start)] >= 0) continue;
    std::vector<int> cols;
    cols.push_back(start);
    int cur = start;
    while (true) {
      const int p = parent_col[static_cast<size_t>(cur)];
      if (p < 0) break;
      if (pos[static_cast<size_t>(p)] != pos[static_cast<size_t>(cur)] + 1) break;
      const auto& lc = later[static_cast<size_t>(cur)];
      const auto& lp = later[static_cast<size_t>(p)];
      bool same = lc.size() == lp.size() + 1 && !lc.empty() && lc.front() == p;
      if (same) {
        for (size_t t = 1; t < lc.size(); ++t) {
          if (lc[t] != lp[t - 1]) { same = false; break; }
        }
      }
      if (!same) break;
      cols.push_back(p);
      cur = p;
    }
    const int sid = static_cast<int>(super_cols.size());
    for (int c : cols) col_to_super[static_cast<size_t>(c)] = sid;
    super_cols.push_back(std::move(cols));
  }

  // Build cliques (supernode ∪ separator) and parent pointers.
  const int num_supers = static_cast<int>(super_cols.size());
  std::vector<std::vector<int>> cliques(static_cast<size_t>(num_supers));
  std::vector<int> tree_parent(static_cast<size_t>(num_supers), -1);
  for (int si = 0; si < num_supers; ++si) {
    const auto& cols = super_cols[static_cast<size_t>(si)];
    const int first = cols.front();
    std::vector<int> merged;
    merged.reserve(cols.size() + later[static_cast<size_t>(first)].size());
    merged.insert(merged.end(), cols.begin(), cols.end());
    merged.insert(merged.end(), later[static_cast<size_t>(first)].begin(),
                  later[static_cast<size_t>(first)].end());
    std::sort(merged.begin(), merged.end());
    merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
    auto& bag = cliques[static_cast<size_t>(si)];
    bag.reserve(merged.size());
    for (int v : merged)
      bag.push_back(unique_vars[static_cast<size_t>(v)]);
    std::sort(bag.begin(), bag.end());

    const int top = cols.back();
    const int pcol = parent_col[static_cast<size_t>(top)];
    if (pcol >= 0)
      tree_parent[static_cast<size_t>(si)] = col_to_super[static_cast<size_t>(pcol)];
  }

  // Build CliqueTree with supernodes/separators.
  CliqueTree ct;
  ct.node_to_parent = tree_parent;
  ct.supernodes.resize(static_cast<size_t>(num_supers));
  ct.separators.resize(static_cast<size_t>(num_supers));
  for (int i = 0; i < num_supers; ++i) {
    const int pidx = tree_parent[static_cast<size_t>(i)];
    if (pidx >= 0) {
      std::set_intersection(
          cliques[static_cast<size_t>(i)].begin(), cliques[static_cast<size_t>(i)].end(),
          cliques[static_cast<size_t>(pidx)].begin(), cliques[static_cast<size_t>(pidx)].end(),
          std::back_inserter(ct.separators[static_cast<size_t>(i)]));
    }
    std::set_difference(
        cliques[static_cast<size_t>(i)].begin(), cliques[static_cast<size_t>(i)].end(),
        ct.separators[static_cast<size_t>(i)].begin(), ct.separators[static_cast<size_t>(i)].end(),
        std::back_inserter(ct.supernodes[static_cast<size_t>(i)]));
  }

  // Merge small supernodes into parents.
  {
    int k = num_supers;
    bool merged_any = true;
    while (merged_any) {
      merged_any = false;
      for (int i = 0; i < k; ++i) {
        if (static_cast<int>(ct.supernodes[i].size()) > max_merge_supernode_size)
          continue;
        const int p = ct.node_to_parent[i];
        if (p < 0) continue;
        auto& psn = ct.supernodes[p];
        for (int v : ct.supernodes[i]) psn.push_back(v);
        std::sort(psn.begin(), psn.end());
        for (int j = 0; j < k; ++j)
          if (ct.node_to_parent[j] == i) ct.node_to_parent[j] = p;
        ct.supernodes[i].clear();
        ct.separators[i].clear();
        ct.node_to_parent[i] = -2;
        merged_any = true;
      }
    }

    // Compact.
    std::vector<int> old_to_new(k, -1);
    int new_k = 0;
    for (int i = 0; i < k; ++i)
      if (ct.node_to_parent[i] != -2) old_to_new[i] = new_k++;

    CliqueTree ct2;
    ct2.supernodes.resize(new_k);
    ct2.separators.resize(new_k);
    ct2.node_to_parent.resize(new_k);
    std::vector<std::vector<int>> new_cliques(new_k);
    for (int i = 0; i < k; ++i) {
      if (old_to_new[i] < 0) continue;
      const int ni = old_to_new[i];
      ct2.supernodes[ni] = std::move(ct.supernodes[i]);
      ct2.separators[ni] = std::move(ct.separators[i]);
      const int p = ct.node_to_parent[i];
      ct2.node_to_parent[ni] = (p >= 0) ? old_to_new[p] : -1;
      new_cliques[ni] = ct2.supernodes[ni];
      new_cliques[ni].insert(new_cliques[ni].end(),
                             ct2.separators[ni].begin(),
                             ct2.separators[ni].end());
      std::sort(new_cliques[ni].begin(), new_cliques[ni].end());
    }
    ct = std::move(ct2);
    cliques = std::move(new_cliques);
  }

  ReorderSupernodes(ct, supernode_reorder_method);

  // Post-order traversal.
  {
    const int nk = static_cast<int>(ct.supernodes.size());
    std::vector<std::vector<int>> children(nk);
    std::vector<int> roots;
    for (int i = 0; i < nk; ++i) {
      if (ct.node_to_parent[i] >= 0)
        children[ct.node_to_parent[i]].push_back(i);
      else
        roots.push_back(i);
    }
    for (auto& c : children) std::sort(c.begin(), c.end());
    std::sort(roots.begin(), roots.end());
    ct.post_order_position_to_clique.clear();
    ct.post_order_position_to_clique.reserve(nk);
    std::vector<int> stk;
    std::vector<char> expanded(nk, 0);
    for (int root : roots) {
      stk.clear();
      stk.push_back(root);
      while (!stk.empty()) {
        const int node = stk.back();
        if (!expanded[node]) {
          expanded[node] = 1;
          for (int c : children[node]) stk.push_back(c);
        } else {
          stk.pop_back();
          ct.post_order_position_to_clique.push_back(node);
        }
      }
    }
  }

  if (maximal_cliques_out) *maximal_cliques_out = cliques;
  return ct;
}

CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out,
    int max_merge_supernode_size,
    int supernode_reorder_method,
    const std::vector<int>& delayed_variables) {
  return MakeCliqueTreeImpl(row_supports, nullptr, maximal_cliques_out,
                            max_merge_supernode_size, supernode_reorder_method,
                            delayed_variables);
}

CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    const Eigen::SparseMatrix<double>& Q_sparsity,
    std::vector<std::vector<int>>* maximal_cliques_out,
    int max_merge_supernode_size,
    int supernode_reorder_method,
    const std::vector<int>& delayed_variables) {
  return MakeCliqueTreeImpl(row_supports, &Q_sparsity, maximal_cliques_out,
                            max_merge_supernode_size, supernode_reorder_method,
                            delayed_variables);
}

}  // namespace conex
