#pragma once
#include "directed_graph.h"
#include "kkt_subsystem.h"

namespace conex {

std::unique_ptr<ConvexSetNode> MakeConvexSetNode(const Graph& graph,
                                                 int node_index);

}
