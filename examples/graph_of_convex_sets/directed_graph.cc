#include "directed_graph.h"

#include <algorithm>
namespace conex {


using T = Graph;

namespace {

class RecursiveTopologicalSort {
  public:
   RecursiveTopologicalSort(const std::vector<Node>& nodes, const std::vector<Edge>& edges) : 
   nodes_(nodes), edges_(edges), 
   permanent_mark_(nodes_.size(), 0), 
   temporary_mark_(nodes_.size(), 0),
   position_to_node(nodes.size()) {}

   void visit(int n) {
     if (permanent_mark_.at(n)) {
       return;
     }
     if (temporary_mark_.at(n)) {
       throw std::runtime_error("Not a DAG");
     }
     temporary_mark_.at(n) = 1;

    srand(time(0));
    std::vector<int> edges = nodes_.at(n).outgoing_edges;
    std::random_shuffle ( edges.begin(), edges.end() ); 

     for (auto& e : edges) {
       visit(edges_.at(e).sink);
     }
     temporary_mark_.at(n) = 0;
     permanent_mark_.at(n) = 1;
     position_to_node.at(num_ordered) = n;
     num_ordered++;
   }

   std::vector<int> Compute(int source_node) {
     visit(source_node);
     if (num_ordered != nodes_.size()) {
      throw std::runtime_error("Graph has multiple source nodes.");
     }
     return position_to_node;
   }

 private:
  const std::vector<Node>& nodes_;
  const std::vector<Edge>& edges_;
  std::vector<int> permanent_mark_;
  std::vector<int> temporary_mark_;
  std::vector<int> position_to_node;
  int num_ordered = 0;
};

} // namespace 

std::vector<int> T::ComputeTopologicalOrdering() {
  CONEX_DEMAND(source_node_ != -1, "Source node not specified.");
  return RecursiveTopologicalSort(nodes_, edges_).Compute(source_node_);
}

// Do a pass to assign elimination position to each variable
int T::AssignEliminationOrderHelper(int node_index, int offset) {
  CONEX_CHECK(node_index < static_cast<int>(nodes_.size()));

  for (auto& child : node_to_children_in_spanning_tree_.at(node_index)) {
    offset = AssignEliminationOrderHelper(child, offset);
  }

  for (auto e : nodes_.at(node_index).incoming_edges) {
    // Spatial z_e
    for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
      ids_.edge_to_incoming_spatial_flow_variable.at(e).push_back(offset);
      offset++;
    }
  }

  // Assign variable
  for (auto e : nodes_.at(node_index).incoming_edges) {
    // Spatial y_e
    for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
      ids_.edge_to_outgoing_spatial_flow_variable.at(e).push_back(offset);
      offset++;
    }
    ids_.edge_to_flow_variable.at(e) = offset;
    offset++;  // phi_e
  }

  for (int i = 0; i < nodes_.at(node_index).spatial_dimension; ++i) {
    ids_.node_to_conversation_of_spatial_flow_multiplier.at(node_index)
        .push_back(offset);
    offset++;
  }
  ids_.node_to_conversation_of_flow_multiplier.at(node_index) = offset;
  offset++;
  return offset;
}

void T::SortEdgeListInReverseTopologicalOrder() {
  int num_nodes = nodes_.size();
  auto EdgeOrder = [this] (int x, int y) -> bool { return this->Edge1LessThanEdge2(x, y); };
  for (int i = 0; i < num_nodes; i++) {
    std::sort(nodes_.at(i).incoming_edges.begin(),  
              nodes_.at(i).incoming_edges.end(), EdgeOrder);
    std::sort(nodes_.at(i).outgoing_edges.begin(),  
              nodes_.at(i).outgoing_edges.end(), EdgeOrder);
  }
};

void T::SortEdgeListInReverseTopologicalOrder(std::vector<int>* edge_list) const {
  auto EdgeOrder = [this] (int x, int y) -> bool { return this->Edge1LessThanEdge2(x, y); };
  std::sort(edge_list->begin(), edge_list->end(), EdgeOrder);
};


void T::BuildSpanningTree() {
  std::vector<int> position_to_node = ComputeTopologicalOrdering();
  std::vector<int> node_to_parent(nodes_.size());
  int root = position_to_node.at(0);
  node_to_parent_in_spanning_tree_.at(root) = -1;
  roots_.push_back(root);
  node_to_topological_order_position_.resize(position_to_node.size());
  node_to_topological_order_position_.at(root) = 0;

  for (size_t i = 1; i < position_to_node.size(); i++) {
    const int parent = position_to_node.at(i-1);
    const int child = position_to_node.at(i);
    node_to_parent_in_spanning_tree_.at(child) = parent;
    node_to_children_in_spanning_tree_.at(parent).push_back(child);
    node_to_topological_order_position_.at(child) = i;
  }


  //std::vector<int> visited(nodes_.size(), 0);

  //int parent = root;
  //roots_.push_back(root);
  //visited.at(parent) = 1;
  //node_to_parent_in_spanning_tree_[parent] = -1;

  //std::stack<int> nodes_to_visit;
  //nodes_to_visit.push(parent);
  //while (nodes_to_visit.size() > 0) {
  //  parent = nodes_to_visit.top();
  //  nodes_to_visit.pop();
  //  for (auto& e : nodes_.at(parent).incoming_edges) {
  //    CONEX_CHECK(edges_.at(e).sink == parent);
  //    int child = edges_.at(e).source;
  //    if (child >= 0 && visited.at(child) == 0) {
  //      visited.at(child) = 1;
  //      node_to_parent_in_spanning_tree_.at(child) = parent;
  //      node_to_children_in_spanning_tree_.at(parent).push_back(child);
  //      nodes_to_visit.push(child);
  //    }
  //  }
  //}
  //for (auto node_visited : visited) {
  //  CONEX_CHECK(node_visited == 1);
  //}
}

void T::IdentifyFillInEdges() {
  int child = source_node_;
  auto& node_to_fill_in_edge_indices = node_to_fill_in_edges_;
  node_to_fill_in_edge_indices.resize(nodes_.size());
  int num_fill_in = 0;

  while (child != -1 /*root node*/) { 
    int parent = node_to_parent_in_spanning_tree_[child];


   // Create new fill-in
    for (auto& f : nodes_.at(child).outgoing_edges) {
      if (edges_[f].sink != parent) {
        node_to_fill_in_edge_indices.at(parent).push_back(f);
        num_fill_in++;
      }
    }

   // Propogate fill-in 
    for (auto& f : node_to_fill_in_edge_indices[child]) {
      if (edges_[f].sink != parent) {
        node_to_fill_in_edge_indices.at(parent).push_back(f);
        num_fill_in++;
      }
    }
    child = parent;
  }
}



}
