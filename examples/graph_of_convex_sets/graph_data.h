#pragma once
#include <vector>
namespace conex {

  struct Node {
    std::vector<int> incoming_edges;
    std::vector<int> outgoing_edges;
    int spatial_dimension;
  };

  struct Edge {
    int source;
    int sink;
  };

  // A list of edges
  struct GraphData {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
  };



enum class VariablePartition {
  flow_variable, /* phi_e */
  incoming_spatial_variable /*z_e*/,
  outgoing_spatial_variable /*y_e*/,
};

}
