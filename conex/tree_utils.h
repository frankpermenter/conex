#pragma once
#include <array>
#include <stack>
#include <vector>
#include "debug_macros.h"

using std::array;
using std::vector;
constexpr int N = 5;

namespace conex {

struct RootedTree {
  RootedTree(int number_of_nodes)
      : parent(number_of_nodes), height(number_of_nodes) {}
  std::vector<int> parent;
  std::vector<int> height;
};

/* Given a union of disjoint rooted trees (forest), we return the list of nodes
 * on the unique path between x and y. Throws an exception if no path exists.
 * The last element of the list is the node closest to the root. */
vector<int> PathInForest(int x, int y, const std::vector<int>& tree,
                         const std::vector<int>& height);

}  // namespace conex
