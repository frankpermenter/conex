#include "conex/common/structural_rank.h"

#include <algorithm>
#include <vector>

namespace conex {
namespace {

// Hopcroft-Karp maximum bipartite matching.
// Graph: rows on the left, columns on the right.
// adj[row] = list of columns with nonzeros in that row.
//
// Returns the size of a maximum matching and fills match_row[col] = row
// (or -1 if unmatched).

struct HopcroftKarp {
  int num_rows, num_cols;
  std::vector<std::vector<int>> adj;  // row -> list of cols
  std::vector<int> match_col;  // match_col[row] = col matched to row (-1 if free)
  std::vector<int> match_row;  // match_row[col] = row matched to col (-1 if free)
  std::vector<int> dist;       // BFS distance for rows

  static constexpr int INF = 1 << 30;

  HopcroftKarp(int nr, int nc)
      : num_rows(nr), num_cols(nc), adj(nr),
        match_col(nr, -1), match_row(nc, -1), dist(nr) {}

  // BFS: find shortest augmenting path length from free rows.
  bool bfs() {
    std::vector<int> queue;
    for (int u = 0; u < num_rows; u++) {
      if (match_col[u] == -1) {
        dist[u] = 0;
        queue.push_back(u);
      } else {
        dist[u] = INF;
      }
    }
    bool found = false;
    size_t qi = 0;
    while (qi < queue.size()) {
      int u = queue[qi++];
      for (int v : adj[u]) {
        int w = match_row[v];  // row matched to col v
        if (w == -1) {
          found = true;  // augmenting path found
        } else if (dist[w] == INF) {
          dist[w] = dist[u] + 1;
          queue.push_back(w);
        }
      }
    }
    return found;
  }

  // DFS: try to augment along shortest path from row u.
  bool dfs(int u) {
    for (int v : adj[u]) {
      int w = match_row[v];
      if (w == -1 || (dist[w] == dist[u] + 1 && dfs(w))) {
        match_col[u] = v;
        match_row[v] = u;
        return true;
      }
    }
    dist[u] = INF;
    return false;
  }

  int solve() {
    int matching = 0;
    while (bfs()) {
      for (int u = 0; u < num_rows; u++) {
        if (match_col[u] == -1) {
          if (dfs(u)) matching++;
        }
      }
    }
    return matching;
  }
};

}  // namespace

int StructuralRank(const Eigen::SparseMatrix<double>& A) {
  const int nr = static_cast<int>(A.rows());
  const int nc = static_cast<int>(A.cols());

  HopcroftKarp hk(nr, nc);
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      hk.adj[it.row()].push_back(it.col());
    }
  }

  return hk.solve();
}

Eigen::SparseMatrix<double> DropStructurallyDependentColumns(
    const Eigen::SparseMatrix<double>& A,
    std::vector<int>* col_map_out) {
  const int nr = static_cast<int>(A.rows());
  const int nc = static_cast<int>(A.cols());

  HopcroftKarp hk(nr, nc);
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      hk.adj[it.row()].push_back(it.col());
    }
  }
  hk.solve();

  // Matched columns: those where match_row[col] >= 0.
  std::vector<int> kept_cols;
  for (int c = 0; c < nc; c++) {
    if (hk.match_row[c] >= 0) {
      kept_cols.push_back(c);
    }
  }
  std::sort(kept_cols.begin(), kept_cols.end());

  if (col_map_out) *col_map_out = kept_cols;

  // Build reduced matrix.
  int new_nc = static_cast<int>(kept_cols.size());
  std::vector<int> inv(nc, -1);
  for (int i = 0; i < new_nc; i++) inv[kept_cols[i]] = i;

  std::vector<Eigen::Triplet<double>> trips;
  for (int k = 0; k < A.outerSize(); ++k) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
      if (inv[it.col()] >= 0) {
        trips.emplace_back(it.row(), inv[it.col()], it.value());
      }
    }
  }

  Eigen::SparseMatrix<double> B(nr, new_nc);
  B.setFromTriplets(trips.begin(), trips.end());
  return B;
}

Eigen::SparseMatrix<double> DropStructurallyDependentRows(
    const Eigen::SparseMatrix<double>& A,
    std::vector<int>* row_map_out) {
  // Rows of A are columns of A^T; drop dependent columns of A^T,
  // then transpose back.
  Eigen::SparseMatrix<double> At = A.transpose();
  std::vector<int> col_map;
  Eigen::SparseMatrix<double> At_reduced =
      DropStructurallyDependentColumns(At, &col_map);
  if (row_map_out) *row_map_out = col_map;
  Eigen::SparseMatrix<double> result = At_reduced.transpose();
  result.makeCompressed();
  return result;
}

}  // namespace conex
