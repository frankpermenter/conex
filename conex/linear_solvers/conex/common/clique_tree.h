#pragma once
#include <vector>

namespace conex {

struct CliqueTree {
  std::vector<std::vector<int>> supernodes;
  std::vector<std::vector<int>> separators;
  std::vector<int> node_to_parent;
  std::vector<int> post_order_position_to_clique;
};

}  // namespace conex
