#pragma once
// Unified geodesic IPM with per-component centering parameter r
// and theta-continuation.
//
// Parameterization:
//   s = P(W^{-1/2})(r - delta)    (primal slack)
//   lambda = P(W^{1/2})(r + delta) (dual multiplier)
//   s .* lambda = r^2 - delta^2    (complementarity = gap)
//
// The data is blended via theta:
//   b_theta = theta * e + (1 - theta) * b
//
// Direction at (W, r, theta):
//   Solve [A'W²A + Q] y = -(c + A^T P(W)(b_theta)) + 2*A^T P(W^{1/2})(r) + d_eq
//   delta = r - P(W^{1/2})(b_theta + A*y)
//   d = solve_lyapunov(r, delta)   [r*d + d*r = 2*delta]
//
// Only W-updates require refactorization.  r-updates and theta-updates
// are free back-solves.

#include <functional>
#include <Eigen/Dense>
#include "conex/common/kkt_solver_interface.h"

namespace conex {

struct GeodesicResult;

// Direction result at (W, r, theta).
struct HybridRDirection {
  double gap;        // |r|^2 - |delta|^2
  double d_inf;      // ||d||_inf
  double d_sq;       // ||d||^2
  double min_slack;  // min eigenvalue of (r - |delta|)
};

// Compute the Newton direction at (W, r, theta).
// b_theta = theta * e + (1 - theta) * b.
// Assumes the KKT system is already factored with weights W².
// Returns d, delta (via output params), and derived quantities.
HybridRDirection ComputeHybridRDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta);

// Verify the Newton direction satisfies its defining equations:
//   Primal: delta = r - P(W^{1/2})(b_theta + A*y)
//   Dual:   A^T P(W^{1/2})(r + delta) + Q*y = c + A^T P(W)(b_theta) - 2*A^T P(W^{1/2})(r)
// Returns (primal_residual, dual_residual) norms.
std::pair<double, double> VerifyHybridREquations(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    const RowSpace& d,
    const RowSpace& delta,
    const Eigen::VectorXd& y);

// ThetaContinuation with r-updates interleaved.
//
// Uses ThetaContinuation's tau selection (duality identity V(tau)=0,
// binary search over theta) at each W-update.  Between W-updates,
// does free r-updates (shrinkR) with theta frozen.  W-updates triggered
// when gap < 0.
GeodesicResult SolveGeodesicThetaContinuationR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false);

// Original HybridR: theta = |gap|/m heuristic.
GeodesicResult SolveGeodesicHybridR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
