#include "convex_set_node_factory.h"

namespace conex {

namespace {

void PrepareInputs(
    const Graph& graph, const int node_index, 
    std::vector<int>* variables,
    ConvexSetNodeParameters* params) {
  auto ids = graph.ids_;

  auto& node = graph.nodes_.at(node_index);

  for (auto e : node.incoming_edges) {
    params->incoming_spatial_flow_start_positions.push_back(variables->size());
    auto& z_e = ids.edge_to_incoming_spatial_flow_variable.at(e);
    variables->insert(variables->end(), z_e.begin(), z_e.end());
  }

  for (auto e : node.incoming_edges) {
    params->outgoing_spatial_flow_of_incoming_edge_start_positions.push_back(variables->size());
    auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
    variables->insert(variables->end(), y_e.begin(), y_e.end());
    
    params->incoming_flow_start_positions.push_back(variables->size());
    variables->push_back(ids.edge_to_flow_variable.at(e));
  }

  auto& lam_1 =
      ids.node_to_conversation_of_spatial_flow_multiplier.at(node_index);
  params->conservation_of_spatial_flow_multiplier_position = variables->size();
  variables->insert(variables->end(), lam_1.begin(), lam_1.end());

  params->conservation_of_flow_multiplier_position = variables->size();
  variables->push_back(
      ids.node_to_conversation_of_flow_multiplier.at(node_index));

  std::vector<int> separator_edges = graph.node_to_fill_in_edges(node_index);
  separator_edges.insert(separator_edges.end(), 
                        node.outgoing_edges.begin(),
                        node.outgoing_edges.end());
  graph.SortEdgeListInReverseTopologicalOrder(&separator_edges);

  int offset = 0;
  for (auto e : separator_edges) {
    auto& y_e = ids.edge_to_outgoing_spatial_flow_variable.at(e);
    variables->insert(variables->end(), y_e.begin(), y_e.end());
    variables->push_back(ids.edge_to_flow_variable.at(e));
    if (std::find(node.outgoing_edges.begin(), node.outgoing_edges.end(), e) !=
        node.outgoing_edges.end()) {
      params->outgoing_spatial_flow_start_positions.push_back(offset);
      params->outgoing_flow_start_positions.push_back(offset + y_e.size());
    }
    offset += y_e.size() + 1;
  }
}

} // namespace 

std::unique_ptr<ConvexSetNode> MakeConvexSetNode(const Graph& graph, int node_index) {
  ConvexSetNodeParameters params;
  std::vector<int> variables;
  PrepareInputs(graph, node_index, &variables, &params);
  auto& node = graph.nodes_.at(node_index);
  params.num_incoming = node.incoming_edges.size();
  params.num_outgoing = node.outgoing_edges.size();
  params.spatial_dimension = node.spatial_dimension;
  return std::make_unique<ConvexSetNode>(variables, params);
}

} // namespace conex
