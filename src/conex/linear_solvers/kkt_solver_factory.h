#pragma once
#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    const std::vector<CliqueProvider*>& assemblers,
    int num_primal_variables,
    const SolverConfiguration& config,
    Arena* arena = nullptr);

inline std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config,
    Arena* arena = nullptr) {
  return MakeTreeSolver(c->clique_assemblers(),
                        c->GetNumberOfVariables(), config, arena);
}

}  // namespace conex
