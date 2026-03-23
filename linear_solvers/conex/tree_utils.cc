#include "conex/tree_utils.h"

#include <algorithm>
#include <stack>
#include <vector>

#include "conex/error_checking_macros.h"

using std::array;
using std::vector;

namespace conex {

vector<int> PathInForest(int x, int y, const std::vector<int>& parent,
                         const std::vector<int>& distance_from_root) {
  std::vector<int> path;
  while (x != y) {
    if (distance_from_root[x] + distance_from_root[y] == 0) {
      throw std::runtime_error(
          "Path does not exist. Points lie in disjoint trees.");
    }

    if (distance_from_root[x] < distance_from_root[y]) {
      path.push_back(y);
      y = parent.at(y);
    } else {
      path.push_back(x);
      x = parent.at(x);
    }
  }
  path.push_back(x);
  return path;
}

void MergeChildIntoParent(CliqueTree& tree, int child) {
  const int n = static_cast<int>(tree.node_to_parent.size());
  CONEX_DEMAND(child >= 0 && child < n, "child index out of range");
  int parent = tree.node_to_parent[child];
  CONEX_DEMAND(parent >= 0, "cannot merge a root node (no parent)");

  // Absorb child's supernodes into parent's supernodes.
  auto& parent_sn = tree.supernodes[parent];
  parent_sn.insert(parent_sn.end(), tree.supernodes[child].begin(),
                   tree.supernodes[child].end());
  std::sort(parent_sn.begin(), parent_sn.end());

  // Reparent grandchildren: any node whose parent was child now points to parent.
  for (int i = 0; i < n; ++i) {
    if (tree.node_to_parent[i] == child) {
      tree.node_to_parent[i] = parent;
    }
  }

  // Erase the child from all arrays.
  tree.supernodes.erase(tree.supernodes.begin() + child);
  tree.separators.erase(tree.separators.begin() + child);
  tree.node_to_parent.erase(tree.node_to_parent.begin() + child);

  // Compact parent indices (everything above child shifts down by 1).
  for (auto& p : tree.node_to_parent) {
    if (p == child) {
      // Should not happen — we already reparented.
      p = parent > child ? parent - 1 : parent;
    }
    if (p > child) --p;
  }

  // Also compact the post_order_position_to_clique mapping if present.
  if (!tree.post_order_position_to_clique.empty()) {
    tree.post_order_position_to_clique.clear();  // invalidated by merge
  }
}

}  // namespace conex
