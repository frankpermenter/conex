#include "gcs_solver.h"
#include "directed_graph.h"
#include "convex_set_node_factory.h"

namespace conex {

GraphSolver::GraphSolver(const GraphData& data) :
  nodes_(data.nodes.size()), graph_(data.nodes, data.edges) {
  graph_.BuildSpanningTree();
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

} // namespace conex
