#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "conex/common/model.h"
#include "conex/common/solve_result.h"

namespace conex {

// Compute DIMACS-like normalized errors from a Model and a candidate solution.
//
// When infeasible=false (optimal solution):
//   dual_err  = ||c + Qx - A'λ - C'ν|| / max(1, ||c|| + ||Qx||)
//   eq_err    = max ||Cx - d|| / max(1, max ||d||)
//   compl_err = |<s,λ>| / max(1, |objective|)    where s = Ax + b
//   prim_err  = min(s) / max(1, ||s||)
//   min_dual  = min(λ) / max(1, ||λ||)
//
// When infeasible=true (certificate ray, not divided by tau):
//   dual_err  = ||Qx - A'λ - C'ν|| / max(1, ||Qx||)   (no c)
//   eq_err    = max ||Cx|| / max(1, ||x||)              (no d)
//   compl_err = |<Ax, λ>| / max(1, ||x|| * ||λ||)      (no b)
//   prim_err  = min(Ax) / max(1, ||Ax||)                (no b)
//   min_dual  = min(λ) / max(1, ||λ||)
//
DimacsErrors ComputeDimacsErrors(
    const Model& model,
    const Eigen::VectorXd& x,
    const std::vector<Eigen::VectorXd>& lambda,
    const std::vector<Eigen::VectorXd>& nu,
    const std::vector<Eigen::MatrixXd>& psd_lambda,
    bool infeasible = false);

// Convenience: extract all duals from a SolveResult.
DimacsErrors ComputeDimacsErrors(const Model& model,
                                 const SolveResult& result);

}  // namespace conex
