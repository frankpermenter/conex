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
    RowSpace& delta,
    Eigen::VectorXd* y_out = nullptr);

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

// Three-solve decomposition for the r-parameterization.
// x(tau,theta) = x0 + tau*x1 + theta*x_theta
// lambda(tau,theta) = lam0 + tau*lam1 + theta*lam_theta
//
// For direction evaluation at fixed theta:
//   y_center = x0 + theta*x_theta  (f, tau-free)
//   y_cost   = x1                   (g, tau-proportional)
//   delta(tau) = delta_center + tau*delta_cost
struct HybridRDecomposition {
  // Combined two-term for direction evaluation.
  Eigen::VectorXd y_center, y_cost;  // x(tau) = y_center + tau*y_cost
  RowSpace delta_center, delta_cost;  // delta(tau) = delta_center + tau*delta_cost

  // Raw three-solve components for joint (tau, theta) selection.
  Eigen::VectorXd x0, x1, x_theta;
  RowSpace lam0, lam1, lam_theta;
};

// Compute the three-solve decomposition at (W, r).
// Requires one factorization (already done) and three back-solves.
// The result contains raw components (x0, x1, x_theta, lam0, lam1, lam_theta).
// Call SetTheta() to form the two-term combination at a specific theta.
HybridRDecomposition ComputeHybridRDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r);

// Update the two-term combination (y_center, y_cost, delta_center, delta_cost)
// at a given theta.  Uses the raw three-solve components.
void SetTheta(HybridRDecomposition& decomp,
              KKTSolverBase& kkt,
              const RowSpace& b,
              const RowSpace& W,
              const RowSpace& r,
              double theta);

// Evaluate the direction at a specific tau from the decomposition.
// Returns (d, delta, gap, d_inf).
HybridRDirection EvalHybridRAtTau(
    KKTSolverBase& kkt,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta);

// ThetaContinuation with r-updates interleaved.
//
// Uses ThetaContinuation's tau selection (duality identity V(tau)=0,
// binary search over theta) at each W-update.  Between W-updates,
// does free r-updates (shrinkR) with theta frozen.  W-updates triggered
// when gap < 0.
// Switching policy for ThetaContR: returns true to center (W-update).
// Arguments: (gap, d_inf, r_updates_since_last_center).
// Default: center if d_inf > 1.
using ThetaContRSwitchPolicy = std::function<bool(double, double, int)>;
inline bool DefaultThetaContRPolicy(double, double d_inf, int) {
  return d_inf > 1.0;
}

GeodesicResult SolveGeodesicThetaContinuationR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false,
    ThetaContRSwitchPolicy policy = DefaultThetaContRPolicy,
    double compl_tol = 1e-12);

// Original HybridR: theta = |gap|/m heuristic.
GeodesicResult SolveGeodesicHybridR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
