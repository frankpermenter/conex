#pragma once
#include <vector>

#include "conex/tree_solver/tree_utils.h"
#include <Eigen/Sparse>

namespace conex {

enum : int {
  CLIQUE_TREE_METHOD_WEIGHTED_DFS = 0,
  CLIQUE_TREE_METHOD_AMD = 1,
};

enum : int {
  SUPERNODE_REORDER_BFS_GREEDY = 0,
  SUPERNODE_REORDER_PQ_TREE = 1,
  SUPERNODE_REORDER_NONE = 2,
  SUPERNODE_REORDER_BFS_GREEDY_LARGEST = 3,
};

// Build a clique tree using minimum-degree elimination on row supports.
// Uses bitset-based min-degree ordering, then extracts a supernodal tree.
// Falls back to CHOLMOD supernodal factorization when available.
CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr,
    int max_merge_supernode_size = 0,
    int supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY,
    const std::vector<int>& dual_variables = {});

// Overload that also accepts pairwise edges from a sparse symmetric matrix.
// Row supports (from A) add all-pairs edges within each support (cliques).
// The sparse matrix Q adds individual edges {i,j} for each Q(i,j)!=0.
// This correctly handles Q's sparsity without creating false cliques.
CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    const Eigen::SparseMatrix<double>& Q_sparsity,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr,
    int max_merge_supernode_size = 0,
    int supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY,
    const std::vector<int>& dual_variables = {});

}  // namespace conex
