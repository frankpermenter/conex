#include "conex/clique_ordering.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <vector>

namespace conex {
namespace {

// Bitset-based min-degree triangulation.  Returns maximal cliques of the
// triangulated graph, with vertex labels in the compact [0, n) space.
std::vector<std::vector<int>> MinDegreeCliques(int n,
    const std::vector<std::vector<int>>& supports_compact) {

  if (n == 0) return {};

  const int words = (n + 63) / 64;

  // Working adjacency (shrinks as vertices are eliminated)
  std::vector<uint64_t> adj(static_cast<size_t>(n) * words, 0);
  auto row = [&](int i) -> uint64_t* {
    return &adj[static_cast<size_t>(i) * words];
  };

  // Build graph: make each support a clique
  for (const auto& sup : supports_compact) {
    for (size_t i = 0; i < sup.size(); i++) {
      for (size_t j = i + 1; j < sup.size(); j++) {
        int u = sup[i], v = sup[j];
        row(u)[v >> 6] |= (1ULL << (v & 63));
        row(v)[u >> 6] |= (1ULL << (u & 63));
      }
    }
  }

  // Initial degrees
  std::vector<int> deg(n);
  for (int v = 0; v < n; v++) {
    const auto* r = row(v);
    int d = 0;
    for (int w = 0; w < words; w++)
      d += __builtin_popcountll(r[w]);
    deg[v] = d;
  }

  std::vector<bool> eliminated(n, false);
  std::vector<int> order;
  order.reserve(n);

  // For each eliminated vertex, store its clique (itself + later neighbors).
  // We extract maximal cliques from these afterward.
  std::vector<std::vector<int>> elim_cliques(n);

  std::vector<uint64_t> clique_mask(words);

  for (int step = 0; step < n; step++) {
    // Find non-eliminated vertex with min degree
    int best = -1, best_deg = std::numeric_limits<int>::max();
    for (int v = 0; v < n; v++) {
      if (!eliminated[v] && deg[v] < best_deg) {
        best_deg = deg[v];
        best = v;
      }
    }

    order.push_back(best);
    eliminated[best] = true;

    // Collect neighbors
    std::vector<int> nbrs;
    auto* rb = row(best);
    for (int w = 0; w < words; w++) {
      uint64_t bits = rb[w];
      while (bits) {
        int b = __builtin_ctzll(bits);
        int u = (w << 6) + b;
        if (u < n) nbrs.push_back(u);
        bits &= bits - 1;
      }
    }

    // Record elimination clique = {best} ∪ nbrs
    elim_cliques[best].reserve(nbrs.size() + 1);
    elim_cliques[best].push_back(best);
    for (int u : nbrs) elim_cliques[best].push_back(u);
    std::sort(elim_cliques[best].begin(), elim_cliques[best].end());

    // Build clique mask
    std::memset(clique_mask.data(), 0, words * sizeof(uint64_t));
    for (int u : nbrs)
      clique_mask[u >> 6] |= (1ULL << (u & 63));

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

    // Remove best from neighbors
    const uint64_t best_bit = 1ULL << (best & 63);
    const int best_word = best >> 6;
    for (int u : nbrs) {
      row(u)[best_word] &= ~best_bit;
      deg[u]--;
    }
    std::memset(rb, 0, words * sizeof(uint64_t));
    deg[best] = 0;
  }

  // Extract maximal cliques from elimination cliques.
  // Sort candidates by size descending; a candidate is non-maximal if it
  // is a subset of any larger (already accepted) candidate.
  std::vector<std::vector<int>>& candidates = elim_cliques;
  // Collect into a flat vector (elim_cliques is indexed by vertex, repack by order)
  std::vector<std::vector<int>> cands;
  cands.reserve(n);
  for (int i = 0; i < n; i++)
    cands.push_back(std::move(candidates[order[i]]));

  // Sort indices by size descending
  std::vector<int> idx(n);
  for (int i = 0; i < n; i++) idx[i] = i;
  std::sort(idx.begin(), idx.end(), [&](int a, int b) {
    return cands[a].size() > cands[b].size();
  });

  std::vector<bool> is_maximal(n, true);
  for (int ii = 0; ii < n; ii++) {
    int i = idx[ii];
    if (!is_maximal[i]) continue;
    for (int jj = ii + 1; jj < n; jj++) {
      int j = idx[jj];
      if (!is_maximal[j]) continue;
      if (cands[j].size() >= cands[i].size()) continue;
      if (std::includes(cands[i].begin(), cands[i].end(),
                        cands[j].begin(), cands[j].end())) {
        is_maximal[j] = false;
      }
    }
  }

  std::vector<std::vector<int>> maximal;
  for (int i = 0; i < n; i++) {
    if (is_maximal[i])
      maximal.push_back(std::move(cands[i]));
  }

  return maximal;
}

}  // namespace

CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out) {

  // Compact variable indices to [0, n)
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

  // Run bitset min-degree and extract maximal cliques (in compact space)
  auto cliques = MinDegreeCliques(n, supports_compact);

  // Remap to global indices
  for (auto& c : cliques) {
    for (int& v : c) v = unique_vars[v];
    std::sort(c.begin(), c.end());
  }

  if (maximal_cliques_out) *maximal_cliques_out = cliques;

  if (cliques.empty()) return CliqueTree{};

  // Delegate tree construction to conex's existing builder
  return MakeCliqueTree(cliques, {}, CLIQUE_TREE_METHOD_WEIGHTED_DFS);
}

}  // namespace conex
