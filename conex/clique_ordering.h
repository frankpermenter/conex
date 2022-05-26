#pragma once
#include <vector>

#include "conex/tree_utils.h"

namespace conex {

CliqueTree MakeCliqueTree(const std::vector<std::vector<int>>& cliques,
                          const std::vector<bool>& clique_is_valid_leaf = {});


void FillIn(const RootedTree& tree, int num_variables,
            const std::vector<int>& order, vector<std::vector<int>>* supernodes,
            vector<std::vector<int>>* separators);

void PickCliqueOrder(const std::vector<std::vector<int>>& cliques_sorted,
                     const std::vector<bool>& valid_leaf, int root,
                     std::vector<int>* order, std::vector<int>* parent_in_tree,
                     std::vector<std::vector<int>>* supernodes,
                     std::vector<std::vector<int>>* separators);

void PickCliqueOrder(const std::vector<std::vector<int>>& cliques_sorted,
                     int root, std::vector<int>* order,
                     std::vector<std::vector<int>>* supernodes,
                     std::vector<std::vector<int>>* separators);

void PickCliqueOrder(const std::vector<std::vector<int>>& cliques_sorted,
                     int root, std::vector<int>* order,
                     std::vector<int>* parent_in_tree,
                     std::vector<std::vector<int>>* supernodes,
                     std::vector<std::vector<int>>* separators);

void PickCliqueOrder(const std::vector<std::vector<int>>& cliques_sorted,
                     int root, RootedTree* order,
                     std::vector<std::vector<int>>* supernodes,
                     std::vector<std::vector<int>>* separators);

}  // namespace conex
