#pragma once
#include <vector>

#include "conex/tree_utils.h"

namespace conex {

enum : int {
  CLIQUE_TREE_METHOD_WEIGHTED_DFS = 0,
  CLIQUE_TREE_METHOD_AMD = 1,
};

CliqueTree MakeCliqueTree(const std::vector<std::vector<int>>& cliques,
                          const std::vector<int>& clique_is_valid_leaf = {},
                          int method = CLIQUE_TREE_METHOD_AMD);

CliqueTree MakePrimalDualCliqueTree(
    const std::vector<std::vector<int>>& cliques,
    const std::vector<std::vector<int>>& dual_variables,
    int method = CLIQUE_TREE_METHOD_AMD);

// Build a clique tree using the implicit construction path from row supports.
// Each entry in `row_supports` is treated as the support of one equation row.
// If `maximal_cliques_out` is provided, it is filled with the maximal cliques
// used to construct the returned tree.
CliqueTree MakeCliqueTreeImplicitFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr);

// Build a clique tree using bitset-based minimum-degree elimination to
// triangulate, then extract maximal cliques and construct the tree.
// Same interface as MakeCliqueTreeImplicitFromRowSupports.
CliqueTree MakeCliqueTreeMinDegreeFromRowSupports(
    const std::vector<std::vector<int>>& row_supports,
    std::vector<std::vector<int>>* maximal_cliques_out = nullptr);

size_t FillIn(const RootedTree& tree, int num_variables,
              const std::vector<int>& order,
              std::vector<std::vector<int>>* supernodes,
              std::vector<std::vector<int>>* separators);

size_t CountCliqueTreeFillIn(const std::vector<std::vector<int>>& cliques,
                             const std::vector<int>& clique_is_valid_leaf = {},
                             int method = CLIQUE_TREE_METHOD_AMD);

void PickCliqueOrder(
    const std::vector<std::vector<int>>& cliques_sorted,
    const std::vector<int>& valid_leaf, int root, std::vector<int>* order,
    std::vector<std::vector<int>>* supernodes,
    std::vector<std::vector<int>>* separators,
    std::vector<std::vector<std::vector<int>>>* post_ordering = NULL);

void PickCliqueOrder(
    const std::vector<std::vector<int>>& cliques_sorted, int root,
    std::vector<int>* order, std::vector<std::vector<int>>* supernodes,
    std::vector<std::vector<int>>* separators,
    std::vector<std::vector<std::vector<int>>>* post_ordering = NULL);

}  // namespace conex
