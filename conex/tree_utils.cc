#include "conex/tree_utils.h"

#include "assert.h"

#include <stack>
#include <vector>

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

}  // namespace conex
