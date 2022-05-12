#pragma once
#include <vector>
#include <stack>
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"
#include "graph_data.h"

namespace conex {

struct Variables {
  std::vector<int> edge_to_flow_variable;
  // Map incoming edge label to spatial variable
  std::vector<std::vector<int>> edge_to_incoming_spatial_flow_variable;
  std::vector<std::vector<int>> edge_to_outgoing_spatial_flow_variable;
  std::vector<std::vector<int>> node_to_conversation_of_spatial_flow_multiplier;
  std::vector<int> node_to_conversation_of_flow_multiplier;
};


#define CONEX_NO_COPY_NO_MOVE(T)\
T(const T&) = delete;\
T(T&&) = delete;\
T& operator=(const T&) = delete;\
T& operator=(T&&) = delete;\

class Graph {
  CONEX_NO_COPY_NO_MOVE(Graph)
 public:
  Graph(std::vector<Node> nodes, std::vector<Edge> edges);

  std::vector<int> ComputeTopologicalOrdering();
  bool IsTopologicalOrderingValid(const std::vector<int>& order) const;
  void AssignEliminationOrder() {
    int offset = 0;
    for (auto& root : roots_) {
      offset = AssignEliminationOrderHelper(root, offset);
    }
  }

  void IdentifyFillInEdges();

  // Sort the edge list of each node in reverse topological order:
  // apply this order to the sink node of the edge, and break
  // ties by applying this order to the source node.
  void SortEdgeListInReverseTopologicalOrder();

  void SortEdgeListInReverseTopologicalOrder(std::vector<int>*) const;

  void BuildSpanningTree(const std::vector<int>& topological_ordering = {});
  int edge_id_to_sink_node(int e) { return edges_.at(e).sink; }

  // All edges >
  std::vector<int> node_to_fill_in_edges(int i) const { return node_to_fill_in_edges_.at(i); }
  std::vector<int> node_to_parent_in_spanning_tree() const { return node_to_parent_in_spanning_tree_; }

  const Node& node(int i) const { return nodes_.at(i); }; 

  std::vector<int> roots_;
  std::vector<Node> nodes_;
  std::vector<Edge> edges_;
  Variables ids_;

 private:

  bool Edge1LessThanEdge2(int edge1, int edge2) const {
    if (edges_.at(edge1).sink != 
        edges_.at(edge2).sink)  {
      return Node1LessThanNode2(edges_.at(edge1).sink,
                         edges_.at(edge2).sink);
    } else {
      return Node1LessThanNode2(edges_.at(edge1).source,
                                edges_.at(edge2).source);
    }
  }

  bool Node1LessThanNode2(int node1, int node2) const {
    // Use reverse topological ordering.
    return node_to_topological_order_position_.at(node1) >
           node_to_topological_order_position_.at(node2);
  }
  int source_node_ = -1;
  std::vector<int> node_to_parent_in_spanning_tree_{};
  std::vector<int> node_to_topological_order_position_{};
  std::vector<std::vector<int>> node_to_fill_in_edges_{};
  std::vector<std::vector<int>> node_to_children_in_spanning_tree_{};
  int AssignEliminationOrderHelper(int node_index, int offset);
};


} // namespace
