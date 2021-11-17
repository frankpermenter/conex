#pragma once
#include <array>
#include <stack>
#include <vector>
#include "debug_macros.h"

using std::array;
using std::vector;
constexpr int N = 5;

namespace conex {

std::vector<int> UnionOfSorted(const std::vector<int>& x1,
                               const std::vector<int>& x2);
void Sort(std::vector<std::vector<int>>* path);

void IntersectionOfSorted(const std::vector<int>& v1,
                          const std::vector<int>& v2, std::vector<int>* v3);

struct RootedTree {
  RootedTree(int number_of_nodes)
      : parent(number_of_nodes), height(number_of_nodes) {}

  RootedTree() : parent({}), height({}) {}
  std::vector<int> parent;
  std::vector<int> height;
  int NumberOfNodes() const { return parent.size(); };
};

vector<int> PathInTree(int x, int y, const std::vector<int>& tree,
                       const std::vector<int>& height);


std::vector<int> NumberOfChildren(const RootedTree& tree);

std::vector<int> GetUnvisitedRootNodes(const RootedTree& d,
                                       const vector<int>& visited);

vector<int> GetUnvisitedLeafNode(const std::vector<int>& num_children,
                                 const std::vector<int>& visited);



}  // namespace conex
