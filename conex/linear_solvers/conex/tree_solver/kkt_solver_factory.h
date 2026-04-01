#pragma once
#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    const std::vector<SupernodalAssemblerBase*>& assemblers,
    int num_primal_variables,
    const SolverConfiguration& config);

// Legacy: extract assemblers from ConstraintManager.
inline std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  return MakeTreeSolver(c->clique_assemblers(),
                        c->GetNumberOfVariables(), config);
}

}  // namespace conex
