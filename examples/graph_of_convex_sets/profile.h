#pragma once
#include <vector>

#include "directed_graph.h"
namespace conex {

struct Time {
  int factor_time;
  int solve_time;
  int factor_time_dense;
  int solve_time_dense;
  int factor_time_natural;
  int factor_time_amd;
  int factor_time_left_looking;
  int solve_time_sparse;
  int non_zeros_amd;
  int non_zeros_natural;
  int non_zeros_lower_tri;
};
struct GraphData {
  std::vector<Node> nodes;
  std::vector<Edge> edges;
};


Time Verify(const GraphData& data,
  std::vector<int> node_to_parent_in_spanning_tree_reference,
            int spatial_dim);

Time Profile(const GraphData& data,
  std::vector<int> node_to_parent_in_spanning_tree_reference);
} // namespace conex
