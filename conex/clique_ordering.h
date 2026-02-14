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
void FillIn(const RootedTree& tree, int num_variables,
            const std::vector<int>& order, vector<std::vector<int>>* supernodes,
            vector<std::vector<int>>* separators);

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
