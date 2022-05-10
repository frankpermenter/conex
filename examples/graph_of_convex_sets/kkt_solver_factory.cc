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
  std::vector<std::unique_ptr<ConvexSetNode>> nodes(num_nodes);

  for (int i = 0; i < num_nodes; i++) {
    nodes_.at(i) = MakeConvexSetNode(graph, i);
  }

  for (auto& n : nodes) {
    tree_solver_.AddSubsystem(n.get());
  }
}

} // namespace conex
