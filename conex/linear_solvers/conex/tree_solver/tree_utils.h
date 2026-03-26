#pragma once
#include <array>
#include <stack>
#include <vector>

#include "conex/common/clique_tree.h"
#include "conex/common/debug_macros.h"

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

// Merge a child node into its parent.  The child's supernodes are absorbed
// into the parent; children of the child become children of the parent.
// The child node is removed and all indices are compacted.
void MergeChildIntoParent(CliqueTree& tree, int child_index);

}  // namespace conex
