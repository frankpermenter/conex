#include "conex/tree_utils.h"

#include "assert.h"

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

void RootedTree::SwapPositions(int node1, int node2) {
  CONEX_ASSERT(node1 < static_cast<int>(parent.size()),
               "Specified index is out of bounds.");
  CONEX_ASSERT(node2 < static_cast<int>(parent.size()),
               "Specified index is out of bounds.");

  if (node1 == node2) {
    return;
  }

  // Case One: one node is a parent of the other:
  // swap(B, C)
  //  A      A
  //  B      C
  //  C      B
  if (parent[node1] == node2) {
    int grand_parent = parent[node2];
    parent[node2] = node1;
    parent[node1] = grand_parent;
    return;
  }
  if (parent[node2] == node1) {
    int grand_parent = parent[node1];
    parent[node1] = node2;
    parent[node2] = grand_parent;
    return;
  }

  // Case Two:  !(Case One):
  //
  // swap(B, D):
  //
  //  A      A
  //  B      D
  //  C      C
  //  D      B
  //
  // The parent of B becomes the parent of D,
  // the parent of D becomes the parent of B.
  std::swap(parent[node1], parent[node2]);
}

}  // namespace conex
