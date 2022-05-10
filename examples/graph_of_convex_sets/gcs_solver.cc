#include "gcs_solver.h"
#include "directed_graph.h"
#include "convex_set_node_factory.h"

namespace conex {

GraphSolver::GraphSolver(const GraphData& data) :
  nodes_(data.nodes.size()) {

  Graph graph(data.nodes, data.edges);

  graph.BuildSpanningTree();
  graph.SortEdgeListInReverseTopologicalOrder();
  graph.AssignEliminationOrder();
  graph.IdentifyFillInEdges();

  int num_nodes = graph.nodes_.size();

  for (int i = 0; i < num_nodes; i++) {
    nodes_.at(i) = MakeConvexSetNode(graph, i);
  }

  for (auto& n : nodes_) {
    tree_solver_.AddSubsystem(n.get());
  }

  tree_solver_.SetEliminationTree(graph.node_to_parent_in_spanning_tree());
}

void GraphSolver::SetFactorizationMode(const GraphSolver::FactorizationMode& mode) {
  for (auto& node: nodes_) {
    node->UseCustomInverse(mode.custom_block_inverse);
  }
  tree_solver_.SetFactorizationMode(mode.left_looking);
}

} // namespace conex
