#pragma once
#include <array>
#include <stack>
#include <vector>

#include "debug_macros.h"

using std::array;
using std::vector;
constexpr int N = 5;

namespace conex {

struct CliqueTree {
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> node_to_parent;
  std::vector<int> post_order_position_to_clique;
};

struct PrimalDualCliqueTree {
  std::vector<std::vector<int>> primal_supernodes;
  std::vector<std::vector<int>> primal_separators;
  std::vector<std::vector<int>> dual_supernodes;
  std::vector<std::vector<int>> dual_separators;
  std::vector<int> clique_id_to_parent;
  std::vector<int> post_order_position_to_clique;
  PrimalDualCliqueTree(int n)
      : primal_supernodes(n),
        primal_separators(n),
        dual_supernodes(n),
        dual_separators(n),
        clique_id_to_parent(n),
        post_order_position_to_clique(n) {}
};

struct RootedTree {
  RootedTree(int number_of_nodes)
      : parent(number_of_nodes), height_(number_of_nodes) {}
  void SwapPositions(int node1, int node2);
  std::vector<int> parent;
  int& height(int i) { return height_.at(i); }
  int height(int i) const { return height_.at(i); }
  std::vector<int> PathInForest(int x, int y) const;

 private:
  mutable std::vector<int> height_;
};

/* Given a union of disjoint rooted trees (forest), we return the list of nodes
 * on the unique path between x and y. Throws an exception if no path exists.
 * The last element of the list is the node closest to the root. */
vector<int> PathInForest(int x, int y, const std::vector<int>& tree,
                         const std::vector<int>& height);

}  // namespace conex
