#include "kkt_subsystem.h"
#include "conex/debug_macros.h"
#define CONEX_ENABLE_TIMER 1

namespace conex {
#if 0
std::vector<int> ConcatenateVariablesInLocalOrdering(
    const Graph& graph, const int node_index) {
  auto ids = graph.ids_;
  std::vector<int> variables;
  auto& node = graph.nodes_.at(node_index);
  for (auto e : node.incoming_edges) {
    auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
    variables.insert(variables.end(), y_e.begin(), y_e.end());
    auto& z_e = ids.edge_to_incoming_spatial_flow_variable.at(e);
    variables.insert(variables.end(), z_e.begin(), z_e.end());
    variables.push_back(ids.edge_to_flow_variable.at(e));
  }
  auto& lam_1 =
      ids.node_to_conversation_of_spatial_flow_multiplier.at(node_index);
  variables.insert(variables.end(), lam_1.begin(), lam_1.end());
  variables.push_back(
      ids.node_to_conversation_of_flow_multiplier.at(node_index));

  std::vector<int> separator_edges = graph.node_to_fill_in_edges(node_index);
  separator_edges.insert(separator_edges.end(), 
                        node.outgoing_edges.begin(),
                        node.outgoing_edges.end());
  graph.SortEdgeListInReverseTopologicalOrder(&separator_edges);

  for (auto e : separator_edges) {
    auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
    variables.insert(variables.end(), y_e.begin(), y_e.end());
    variables.push_back(ids.edge_to_flow_variable.at(e));
  }
  return variables;
}
#endif
using T = ConvexSetNode;

T::ConvexSetNode(const std::vector<int>& variables, 
                 const ConvexSetNodeParameters& params) : 
                 KKTSubsystem(variables, 0), params_(params) {
  int num_supernodes =
      params.num_incoming * (2 * params.spatial_dimension + 1) +
      params.spatial_dimension + 1;

  std::vector<int> supernodes;
  supernodes.insert(supernodes.begin(), variables.begin(),
                    variables.begin() + num_supernodes);

  std::vector<int> separators;
  separators.insert(separators.begin(), variables.begin() + num_supernodes,
                    variables.end());
  SetSupernodes(supernodes);
  SetSeparators(separators);
  num_incoming = params.num_incoming; 
  num_outgoing = params.num_outgoing;
  spatial_dim = params.spatial_dimension;
}

} // namespace conex
