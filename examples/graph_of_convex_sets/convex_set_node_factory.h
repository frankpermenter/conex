#pragma once
#include "kkt_subsystem.h"
#include "directed_graph.h"

namespace conex {

std::unique_ptr<ConvexSetNode> MakeConvexSetNode(const Graph& graph, int node_index);

}
