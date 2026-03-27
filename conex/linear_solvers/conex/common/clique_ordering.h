#pragma once
#include <vector>

#include "conex/common/clique_tree.h"
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
//
// delayed_variables: variables that must not be eliminated until at least
// one of their neighbors has been eliminated first.  This includes dual
// variables in KKT systems and any variable without a positive-definite
// diagonal contribution.  (Previously called "dual_variables".)
CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr,
    int max_merge_supernode_size = 0,
    int supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY,
    const std::vector<int>& delayed_variables = {});

// Overload that also accepts pairwise edges from a sparse symmetric matrix.
CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    const Eigen::SparseMatrix<double>& Q_sparsity,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr,
    int max_merge_supernode_size = 0,
    int supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY,
    const std::vector<int>& delayed_variables = {});

// Elimination result from Phase 1 (AMD or user-provided ordering).
// Contains everything Phase 2 needs to build the clique tree.
struct EliminationOrdering {
  std::vector<int> order;       // elimination order (variable indices in compact space)
  std::vector<std::vector<int>> later;  // later[v] = living neighbors when v was eliminated
  std::vector<int> parent_col;  // parent_col[v] = first neighbor eliminated after v
  std::vector<int> unique_vars; // compact → original variable mapping
};

// Phase 2: Build a clique tree from a pre-computed elimination ordering.
// Performs supernode detection, tree construction, supernode merging,
// reordering, and post-order computation.
CliqueTree MakeCliqueTreeFromEliminationOrdering(
    const EliminationOrdering& elim,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr,
    int max_merge_supernode_size = 0,
    int supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY);

}  // namespace conex
