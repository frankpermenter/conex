// Self-dual homogeneous embedding for conic optimization.
//
// Adds a scalar variable τ (and dual κ) to the original problem,
// creating a system that is always strictly feasible from (W=I, τ=1).
// Eliminates the need for a Phase-I feasibility step.
//
// The embedding equations:
//   τc - Ay = s + μ(c - e)
//   A^T x = τb + μ(Ae - b)
//   <x, s> + κτ = μ(rank + 1)
//   b^T y - c^T x = κ
//
// where x = P(W^{1/2})(e + d), s = P(W^{-1/2})(e - d).

#pragma once
#include <Eigen/Core>
#include "conex/common/kkt_solver_interface.h"

namespace conex {

struct HSDResult {
  int iterations = 0;
  double mu = 0;          // final barrier parameter
  double tau = 0;         // final τ
  double kappa = 0;       // final κ
  double primal_obj = 0;  // b^T y / τ
  double dual_obj = 0;    // c^T x / τ
  double d_inf = 0;       // final ||d||_inf
  Eigen::VectorXd y;      // primal variables (scaled by τ)
  bool solved = false;
};

HSDResult SolveHSD(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 100,
    double tol = 1e-8,
    bool verbose = false);

}  // namespace conex
