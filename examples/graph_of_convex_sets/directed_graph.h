#pragma once
#include <vector>
#include <stack>
#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"

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

struct Variables {
  std::vector<int> edge_to_flow_variable;
  // Map incoming edge label to spatial variable
  std::vector<std::vector<int>> edge_to_incoming_spatial_flow_variable;
  std::vector<std::vector<int>> edge_to_outgoing_spatial_flow_variable;
  std::vector<std::vector<int>> node_to_conversation_of_spatial_flow_multiplier;
  std::vector<int> node_to_conversation_of_flow_multiplier;
};

class Graph {
 public:
  Graph(std::vector<Node> nodes, std::vector<Edge> edges)
      : nodes_(std::move(nodes)), edges_(std::move(edges)) {
    int num_nodes = nodes_.size();
    int num_edges = edges_.size();
    ids_.edge_to_outgoing_spatial_flow_variable.resize(num_edges);
    ids_.edge_to_incoming_spatial_flow_variable.resize(num_edges);
    ids_.node_to_conversation_of_spatial_flow_multiplier.resize(num_nodes);
    ids_.node_to_conversation_of_flow_multiplier.resize(num_nodes);
    ids_.edge_to_flow_variable.resize(num_edges);
    node_to_children_in_spanning_tree_.resize(num_nodes);
    node_to_parent_in_spanning_tree_.resize(num_nodes);

    int i = 0;
    for (auto& e : edges_) {
      if (e.source > -1) {
        nodes_.at(e.source).outgoing_edges.push_back(i);
      } else {
      CONEX_DEMAND(source_node_ == -1, 
      "Source node already specified.");
        source_node_ = e.sink;
      }
      nodes_.at(e.sink).incoming_edges.push_back(i);
      i++;
    }
  }

  std::vector<int> ComputeTopologicalOrdering();
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

  void BuildSpanningTree();

  // All edges >
  std::vector<int> node_to_fill_in_edges(int i) const { return node_to_fill_in_edges_.at(i); }
  std::vector<int> node_to_parent_in_spanning_tree() { return node_to_parent_in_spanning_tree_; }

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
