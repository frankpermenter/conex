#pragma once

#include "conex/kkt_tree_solver.h"
#include "kkt_subsystem.h"

#include "graph_data.h"

namespace conex {

class GraphSolver {
 public:
  GraphSolver(const GraphData& graph_data);
 private:
  std::vector<std::unique_ptr<ConvexSetNode>> nodes_;
  SymmetricLinearSystemTreeSolver tree_solver_;
};

}
