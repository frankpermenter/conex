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
  struct GraphData {
    std::vector<Node> nodes;
    std::vector<Edge> edges;
  };
}
