#pragma once
#include "conex/conex.h"
#include "conex/constraint_manager.h"
#include "conex/kkt_tree_solver.h"

namespace conex {

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config);

}  // namespace conex
