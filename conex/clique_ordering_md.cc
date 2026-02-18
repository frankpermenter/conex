#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <vector>

#include "conex/clique_ordering.h"

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

// popcount(a & b)
int PopcountAnd(const uint64_t* a, const uint64_t* b, int words) {
  int c = 0;
  for (int w = 0; w < words; w++) c += __builtin_popcountll(a[w] & b[w]);
  return c;
}

// Is a a subset of b?  (a & b) == a
bool IsSubset(const uint64_t* a, const uint64_t* b, int words) {
  for (int w = 0; w < words; w++)
    if (a[w] & ~b[w]) return false;
  return true;
}


}  // namespace

CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out) {
  // --- Compact variable indices to [0, n) ---
  std::vector<int> unique_vars;
  for (const auto& row : row_supports) {
    for (int v : row) {
      if (v >= 0) unique_vars.push_back(v);
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

  std::map<int, int> to_compact;
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

  std::vector<int> deg(n);
  for (int v = 0; v < n; v++) deg[v] = Popcount(row(v), words);

  std::vector<int> order;
  order.reserve(n);

  // Store elimination cliques as bitsets (each n/64 words).
  std::vector<uint64_t> elim_bits(static_cast<size_t>(n) * words, 0);
  auto elim_row = [&](int i) -> uint64_t* {
    return &elim_bits[static_cast<size_t>(i) * words];
  };
  // Also track their sizes for later sorting.
  std::vector<int> elim_size(n, 0);

  std::vector<uint64_t> clique_mask(words);
  std::vector<int> nbrs;

  for (int step = 0; step < n; step++) {
    int best = -1, best_deg = std::numeric_limits<int>::max();
    for (int v = 0; v < n; v++) {
      if (deg[v] >= 0 && deg[v] < best_deg) {
        best_deg = deg[v];
        best = v;
      }
    }

    order.push_back(best);

    auto* rb = row(best);
    BitsToVec(rb, words, n, &nbrs);

    // Record elimination clique bitset = {best} ∪ nbrs
    auto* eb = elim_row(best);
    std::memcpy(eb, rb, words * sizeof(uint64_t));
    eb[best >> 6] |= (1ULL << (best & 63));  // include self
    elim_size[best] = static_cast<int>(nbrs.size()) + 1;

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

    // Remove best from working graph
    const uint64_t best_bit = 1ULL << (best & 63);
    const int best_word = best >> 6;
    for (int u : nbrs) {
      row(u)[best_word] &= ~best_bit;
      deg[u]--;
    }
    std::memset(rb, 0, words * sizeof(uint64_t));
    deg[best] = -1;  // mark eliminated
  }

  // ===================================================================
  // Phase 2: Extract maximal cliques (bitset subset checks)
  // ===================================================================
  // Sort elimination clique indices by size descending.
  // A candidate is non-maximal if it is a subset of any larger one.

  std::vector<int> idx(n);
  for (int i = 0; i < n; i++) idx[i] = order[i];  // vertex ids in elim order
  std::sort(idx.begin(), idx.end(),
            [&](int a, int b) { return elim_size[a] > elim_size[b]; });

  std::vector<bool> is_maximal(n, true);  // indexed by vertex
  for (int ii = 0; ii < n; ii++) {
    int i = idx[ii];
    if (!is_maximal[i]) continue;
    const auto* ei = elim_row(i);
    for (int jj = ii + 1; jj < n; jj++) {
      int j = idx[jj];
      if (!is_maximal[j]) continue;
      if (elim_size[j] >= elim_size[i]) continue;
      if (IsSubset(elim_row(j), ei, words)) {
        is_maximal[j] = false;
      }
    }
  }

  // Collect maximal clique bitsets + build sorted vector cliques for output.
  // mc_bits[ci] points to the bitset for maximal clique ci.
  std::vector<int> mc_vertex;  // which original vertex owns each maximal clique
  for (int i = 0; i < n; i++) {
    int v = order[i];
    if (is_maximal[v]) mc_vertex.push_back(v);
  }
  const int k = static_cast<int>(mc_vertex.size());

  // Pointers to the bitset rows for each maximal clique
  std::vector<uint64_t*> mc_bits(k);
  for (int ci = 0; ci < k; ci++) mc_bits[ci] = elim_row(mc_vertex[ci]);

  // Build sorted-vector cliques (in global indices) for output
  std::vector<std::vector<int>> cliques(k);
  {
    std::vector<int> tmp;
    for (int ci = 0; ci < k; ci++) {
      BitsToVec(mc_bits[ci], words, n, &tmp);
      cliques[ci].reserve(tmp.size());
      for (int v : tmp) cliques[ci].push_back(unique_vars[v]);
      // Already sorted since unique_vars is sorted and bits scan low-to-high
    }
  }

  if (maximal_cliques_out) *maximal_cliques_out = cliques;

  if (k == 0) return CliqueTree{};

  // ===================================================================
  // Phase 3: Prim's max-weight spanning tree (bitset AND + popcount)
  // ===================================================================
  int root = 0;
  for (int i = 1; i < k; i++)
    if (elim_size[mc_vertex[i]] > elim_size[mc_vertex[root]]) root = i;

  std::vector<bool> in_tree(k, false);
  std::vector<int> parent(k, -1);
  std::vector<int> best_weight(k, -1);
  std::vector<int> best_neighbor(k, -1);

  in_tree[root] = true;
  for (int j = 0; j < k; j++) {
    if (j == root) continue;
    best_weight[j] = PopcountAnd(mc_bits[root], mc_bits[j], words);
    best_neighbor[j] = root;
  }

  for (int step = 1; step < k; step++) {
    int best = -1, bw = -1;
    for (int j = 0; j < k; j++) {
      if (!in_tree[j] && best_weight[j] > bw) {
        bw = best_weight[j];
        best = j;
      }
    }
    in_tree[best] = true;
    parent[best] = best_neighbor[best];

    const auto* bb = mc_bits[best];
    for (int j = 0; j < k; j++) {
      if (!in_tree[j]) {
        int w = PopcountAnd(bb, mc_bits[j], words);
        if (w > best_weight[j]) {
          best_weight[j] = w;
          best_neighbor[j] = best;
        }
      }
    }
  }

  // ===================================================================
  // Phase 4: Build conex::CliqueTree (post-order, supernodes, separators)
  // ===================================================================
  std::vector<std::vector<int>> children(k);
  for (int i = 0; i < k; i++)
    if (parent[i] != -1) children[parent[i]].push_back(i);

  // Post-order via iterative DFS
  std::vector<int> post_order;
  post_order.reserve(k);
  {
    std::vector<int> stk = {root};
    std::vector<bool> expanded(k, false);
    while (!stk.empty()) {
      int node = stk.back();
      if (!expanded[node]) {
        expanded[node] = true;
        for (int c : children[node]) stk.push_back(c);
      } else {
        stk.pop_back();
        post_order.push_back(node);
      }
    }
  }

  // Supernodes and separators via bitwise AND / AND-NOT
  CliqueTree ct;
  ct.node_to_parent = std::move(parent);
  ct.post_order_position_to_clique = std::move(post_order);
  ct.supernodes.resize(k);
  ct.separators.resize(k);

  // We need separators in global indices.  Reuse the compact→global mapping.
  // mc_bits are in compact space; convert via unique_vars.
  auto bits_to_global = [&](const uint64_t* bits, int nw, int nn,
                            std::vector<int>* out) {
    out->clear();
    for (int w = 0; w < nw; w++) {
      uint64_t b = bits[w];
      while (b) {
        int bit = __builtin_ctzll(b);
        int v = (w << 6) + bit;
        if (v < nn) out->push_back(unique_vars[v]);
        b &= b - 1;
      }
    }
  };

  // Temp buffer for AND / DIFF results
  std::vector<uint64_t> tmp_bits(words);

  for (int i = 0; i < k; i++) {
    if (ct.node_to_parent[i] == -1) {
      bits_to_global(mc_bits[i], words, n, &ct.supernodes[i]);
    } else {
      const auto* ci = mc_bits[i];
      const auto* pi = mc_bits[ct.node_to_parent[i]];
      // separator = C_i AND C_parent
      for (int w = 0; w < words; w++) tmp_bits[w] = ci[w] & pi[w];
      bits_to_global(tmp_bits.data(), words, n, &ct.separators[i]);
      // supernode = C_i AND NOT C_parent
      for (int w = 0; w < words; w++) tmp_bits[w] = ci[w] & ~pi[w];
      bits_to_global(tmp_bits.data(), words, n, &ct.supernodes[i]);
    }
  }

  return ct;
}

}  // namespace conex
