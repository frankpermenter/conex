#include "directed_graph.h"

#include <algorithm>
#include <numeric>
namespace conex {

using T = Graph;

namespace {

class RecursiveTopologicalSort {
 public:
  RecursiveTopologicalSort(const std::vector<Node>& nodes,
                           const std::vector<Edge>& edges)
      : nodes_(nodes),
        edges_(edges),
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
    std::random_shuffle(edges.begin(), edges.end());

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
  size_t num_ordered = 0;
  bool order_unique_ = true;
};

}  // namespace

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
  auto EdgeOrder = [this](int x, int y) -> bool {
    return this->Edge1LessThanEdge2(x, y);
  };
  for (int i = 0; i < num_nodes; i++) {
    std::sort(nodes_.at(i).incoming_edges.begin(),
              nodes_.at(i).incoming_edges.end(), EdgeOrder);
    std::sort(nodes_.at(i).outgoing_edges.begin(),
              nodes_.at(i).outgoing_edges.end(), EdgeOrder);
  }
};

void T::SortEdgeListInReverseTopologicalOrder(
    std::vector<int>* edge_list) const {
  CONEX_CHECK(node_to_topological_order_position_.size() > 0);
  auto EdgeOrder = [this](int x, int y) -> bool {
    return this->Edge1LessThanEdge2(x, y);
  };
  std::sort(edge_list->begin(), edge_list->end(), EdgeOrder);
};

bool T::IsTopologicalOrderingValid(
    const std::vector<int>& order_position_to_node) const {
  if (order_position_to_node.size() != nodes_.size()) {
    return false;
  }
  std::vector<int> node_to_order_position(order_position_to_node.size());
  int i = 0;
  for (auto p : order_position_to_node) {
    node_to_order_position.at(p) = i;
  }

  for (auto& e : edges_) {
    if (e.source > -1) {
      if (node_to_order_position.at(e.sink) >
          node_to_order_position.at(e.source)) {
        return false;
      }
    }
  }
  return true;
}

void T::BuildSpanningTree(const std::vector<int>& order_position_to_node) {
  topological_order_position_to_node_ = order_position_to_node;
  if (order_position_to_node.size() == 0) {
    topological_order_position_to_node_ = ComputeTopologicalOrdering();
  }
  CONEX_CHECK(IsTopologicalOrderingValid(topological_order_position_to_node_));
  std::vector<int> node_to_parent(nodes_.size());
  int root = topological_order_position_to_node_.at(0);
  node_to_parent_in_spanning_tree_.at(root) = -1;
  roots_.push_back(root);
  node_to_topological_order_position_.resize(
      topological_order_position_to_node_.size());
  node_to_topological_order_position_.at(root) = 0;

  for (size_t i = 1; i < topological_order_position_to_node_.size(); i++) {
    const int parent = topological_order_position_to_node_.at(i - 1);
    const int child = topological_order_position_to_node_.at(i);
    node_to_parent_in_spanning_tree_.at(child) = parent;
    node_to_children_in_spanning_tree_.at(parent).push_back(child);
    node_to_topological_order_position_.at(child) = i;
  }
}

void T::IdentifyFillInEdges() {
  int child = source_node_;
  auto& node_to_fill_in_edge_indices = node_to_fill_in_edges_;
  node_to_fill_in_edge_indices.resize(nodes_.size());
  int num_fill_in = 0;

  while (child != -1 /*root node*/) {
    int parent = node_to_parent_in_spanning_tree_[child];

    // Add out-going edges of children that are not incoming.
    // Fill-in interpretation:  |neighbors of children that are > in the
    // topological ordering|.
    for (auto& f : nodes_.at(child).outgoing_edges) {
      if (edges_[f].sink != parent) {
        node_to_fill_in_edge_indices.at(parent).push_back(f);
        num_fill_in++;
      }
    }

    // Propagate fill-in
    if (parent != -1) {
      for (auto& f : node_to_fill_in_edge_indices[child]) {
        if (edges_[f].sink != parent) {
          node_to_fill_in_edge_indices.at(parent).push_back(f);
          num_fill_in++;
        }
      }
    }
    child = parent;
  }
}

T::Graph(std::vector<Node> nodes, std::vector<Edge> edges)
    : nodes_(std::move(nodes)), edges_(std::move(edges)) {
  int i = 0;
  for (auto& e : edges_) {
    if (e.source > -1) {
      nodes_.at(e.source).outgoing_edges.push_back(i);
      nodes_.at(e.sink).incoming_edges.push_back(i);
    } else {
      throw std::runtime_error("Remove trivial source edges.");
    }
    i++;
  }

  i = 0;
  for (auto& n : nodes_) {
    if (n.incoming_edges.size() == 0) {
      CONEX_DEMAND(source_node_ == -1, "Source node already specified.");
      source_node_ = i;
      Edge e;
      e.source = -1;
      e.sink = source_node_;
      edges_.push_back(e);
      nodes_.at(e.sink).incoming_edges.push_back(edges_.size() - 1);
    }
    i++;
  }

  int num_nodes = nodes_.size();
  int num_edges = edges_.size();
  ids_.edge_to_outgoing_spatial_flow_variable.resize(num_edges);
  ids_.edge_to_incoming_spatial_flow_variable.resize(num_edges);
  ids_.node_to_conversation_of_spatial_flow_multiplier.resize(num_nodes);
  ids_.node_to_conversation_of_flow_multiplier.resize(num_nodes);
  ids_.edge_to_flow_variable.resize(num_edges);
  node_to_children_in_spanning_tree_.resize(num_nodes);
  node_to_parent_in_spanning_tree_.resize(num_nodes);
}

std::vector<int> T::topological_order_position_to_edge() const {
  std::vector<int> order_position_to_edge(edges_.size());
  std::iota(order_position_to_edge.begin(), order_position_to_edge.end(), 0);
  SortEdgeListInReverseTopologicalOrder(&order_position_to_edge);
  return order_position_to_edge;
}

std::vector<int> T::edge_to_topological_order_position() const {
  auto order_position_to_edge = topological_order_position_to_edge();
  std::vector<int> edge_to_order_position(order_position_to_edge.size());
  for (size_t i = 0; i < order_position_to_edge.size(); i++) {
    edge_to_order_position.at(order_position_to_edge.at(i)) = i;
  }
  return edge_to_order_position;
}

struct Clique {
  int edge_id = 0;
  std::vector<int> nodes;
};

std::vector<int> T::primal_dual_to_interleaved_topological_order() const {
  std::vector<Clique> edge_to_supernodes;
  std::vector<int> node_to_edge_elimination(nodes_.size());

  std::vector<int> edge_to_order_position =
      edge_to_topological_order_position();
  std::vector<int> order_position_to_edge =
      topological_order_position_to_edge();

  // Put edges in order and assign nodes
  for (size_t i = 0; i < edges_.size(); ++i) {
    Clique c;
    c.edge_id = order_position_to_edge.at(i);

    if (edges_.at(c.edge_id).source != -1) {
      if (LastEdgeInReverseTopologicalOrder(edges_.at(c.edge_id).source) ==
          c.edge_id) {
        c.nodes.push_back(edges_.at(c.edge_id).source);
        node_to_edge_elimination.at(edges_.at(c.edge_id).source) = i;
      }
    }
    if (LastEdgeInReverseTopologicalOrder(edges_.at(c.edge_id).sink) ==
        c.edge_id) {
      c.nodes.push_back(edges_.at(c.edge_id).sink);
      node_to_edge_elimination.at(edges_.at(c.edge_id).sink) = i;
    }
    edge_to_supernodes.push_back(c);
  }

  std::vector<int> primal_dual_order_to_edge_topological;
  int spatial_dim = nodes_.at(0).spatial_dimension;

  for (auto& e : edge_to_supernodes) {
    for (auto n : e.nodes) {
      int offset_node =
          edges_.size() * (2 * spatial_dim + 1) + n * (spatial_dim + 1);
      for (int i = 0; i < spatial_dim + 1; i++) {
        primal_dual_order_to_edge_topological.push_back(i + offset_node);
      }
    }
    int offset_edge = (e.edge_id) * (2 * spatial_dim + 1);
    for (int i = 0; i < 2 * spatial_dim + 1; i++) {
      primal_dual_order_to_edge_topological.push_back(i + offset_edge);
    }
  }
  return primal_dual_order_to_edge_topological;
}

std::vector<int> T::primal_dual_to_node_edge_order() const {
  std::vector<int> order_position_to_edge =
      topological_order_position_to_edge();

  std::vector<int> primal_dual_order_to_edge_node;
  int spatial_dim = nodes_.at(0).spatial_dimension;

  std::reverse(order_position_to_edge.begin(), order_position_to_edge.end());
  for (auto edge_id : order_position_to_edge) {
    int offset_edge = (edge_id) * (2 * spatial_dim + 1);
    for (int i = 0; i < 2 * spatial_dim + 1; i++) {
      primal_dual_order_to_edge_node.push_back(i + offset_edge);
    }
  }

  auto order_position_to_node = topological_order_position_to_node_;
  // std::reverse(order_position_to_node.begin(), order_position_to_node.end());
  for (auto& node_id : order_position_to_node) {
    int offset_node =
        edges_.size() * (2 * spatial_dim + 1) + node_id * (spatial_dim + 1);
    for (int i = 0; i < spatial_dim + 1; i++) {
      primal_dual_order_to_edge_node.push_back(i + offset_node);
    }
  }

  return primal_dual_order_to_edge_node;
}

}  // namespace conex
