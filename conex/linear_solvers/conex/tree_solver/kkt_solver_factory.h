#pragma once
#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config);

}  // namespace conex
