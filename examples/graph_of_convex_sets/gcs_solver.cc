#include "gcs_solver.h"
#include "directed_graph.h"
#include "convex_set_node_factory.h"

namespace conex {

GraphSolver::GraphSolver(const GraphData& data, 
                         const std::vector<int>& topological_order_position_to_node) :
  nodes_(data.nodes.size()), graph_(data.nodes, data.edges) {
  graph_.BuildSpanningTree(topological_order_position_to_node);
  graph_.SortEdgeListInReverseTopologicalOrder();
  graph_.AssignEliminationOrder();
  graph_.IdentifyFillInEdges();

  int num_nodes = graph_.nodes_.size();

  for (int i = 0; i < num_nodes; i++) {
    nodes_.at(i) = MakeConvexSetNode(graph_, i);
  }

  for (auto& n : nodes_) {
    tree_solver_.AddSubsystem(n.get());
  }

  tree_solver_.SetEliminationTree(graph_.node_to_parent_in_spanning_tree());

}


void GraphSolver::SetFactorizationMode(const GraphSolver::FactorizationMode& mode) {
  for (auto& node: nodes_) {
    node->UseCustomInverse(mode.custom_block_inverse);
  }
  tree_solver_.SetFactorizationMode(mode.left_looking);
}

Eigen::Ref<Eigen::MatrixXd> GraphSolver::quadratic_edge_cost_mutable(int edge_number, VariablePartition row_block, VariablePartition col_block) {
  int node = graph_.edge_id_to_sink_node(edge_number);
  return nodes_.at(node)->quadratic_cost_mutable(edge_number, row_block, col_block);
}

Eigen::PermutationMatrix<-1> GraphSolver::variable_to_primal_dual_order_position() const {
  int i = 0;
  std::vector<int> y(tree_solver_.number_of_variables());
  auto positions = graph_.elimination_positions();
  for (size_t e = 0; e < graph_.edges().size(); ++e) {
    for (auto v : positions.edge_to_incoming_spatial_flow_variable.at(e)) {
      y.at(i++) = v;
    }
    for (auto v : positions.edge_to_outgoing_spatial_flow_variable.at(e)) {
      y.at(i++) = v;
    }
    y.at(i++) = positions.edge_to_flow_variable.at(e);
  }
  for (size_t n = 0; n < graph_.nodes().size(); ++n) {
    for (auto v : positions.node_to_conversation_of_spatial_flow_multiplier.at(n)) {
      y.at(i++) = v;
    }
    y.at(i++) = positions.node_to_conversation_of_flow_multiplier.at(n);
  }
  Eigen::PermutationMatrix<-1> P(y.size());
  P.indices() = Eigen::Map<Eigen::VectorXi>(y.data(), y.size());
  return P;
}

} // namespace conex
