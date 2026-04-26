#pragma once
// ThetaContR: geodesic IPM with r-parameterization, joint (tau, theta)
// selection, and tau treated as a cone variable.
//
// See doc/hybrid_theta_continuation.tex for the full derivation.
//
// Parameterization:
//   s = P(W^{-1/2})(r - delta),  lambda = P(W^{1/2})(r + delta)
//   tau = w_tau * r_tau * (1 + d_tau),  kappa = w_tau^{-1} * r_tau * (1 - d_tau)
//
// Three-solve decomposition (x0, x1, x_theta) at fixed (W, r):
//   x(tau, theta) = x0 + tau*x1 + theta*x_theta
//
// Joint (tau, theta) selection from gap + normalization equations.
// Theta substituted before Q expansion to avoid cancellation.
//
// Switching: W-update if gap < 0 or theta progress stalls.
// Complementarity freeze: stop W-updates when gap + tau*kappa != theta*alpha.

#include <functional>
#include <Eigen/Dense>
#include "conex/common/compiled_model.h"

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
    CompiledModel& model,
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
    CompiledModel& model,
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

  // Cached A*x products (avoid recomputing in SetTheta).
  RowSpace ax0, ax1, ax_theta;
};

// Compute the three-solve decomposition at (W, r).
// Requires one factorization (already done) and three back-solves.
// The result contains raw components (x0, x1, x_theta, lam0, lam1, lam_theta).
// Call SetTheta() to form the two-term combination at a specific theta.
HybridRDecomposition ComputeHybridRDecomposition(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r);

// Re-solve only x0 after an r-update (x1 and x_theta are unchanged).
// Returns 1 (number of solves performed).
int UpdateX0(HybridRDecomposition& decomp,
             CompiledModel& model,
             const RowSpace& W,
             const RowSpace& r);

// Update the two-term combination (y_center, y_cost, delta_center, delta_cost)
// at a given theta.  Uses cached ax0, ax_theta — no MultiplyA needed.
void SetTheta(HybridRDecomposition& decomp,
              CompiledModel& model,
              const RowSpace& b,
              const RowSpace& W,
              const RowSpace& r,
              double theta);

// Evaluate the direction at a specific tau from the decomposition.
// Returns (d, delta, gap, d_inf).
HybridRDirection EvalHybridRAtTau(
    CompiledModel& model,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta);

// ThetaContR: joint (tau, theta) selection with r-updates.
//
// Three-solve decomposition + joint quadratic for (tau, theta).
// tau treated as cone variable with (w_tau, r_tau, d_tau).
// Complementarity freeze prevents W-updates when numerics degrade.
// Theta-rate trigger forces W-updates when r-updates stall.
// Switching policy for ThetaContR: returns true to center (W-update).
// Arguments: (gap, d_inf, r_updates_since_last_center).
// Default: center if gap < 0 (complementarity violation).
using ThetaContRSwitchPolicy = std::function<bool(double, double, int)>;
inline bool DefaultThetaContRPolicy(double gap, double, int) {
  return gap < 0;
}


GeodesicResult SolveGeodesicThetaContinuationR(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false,
    ThetaContRSwitchPolicy policy = DefaultThetaContRPolicy,
    double compl_tol = 1e-12,
    double theta_rate = 0.1);  // center if theta hasn't decreased by this
                                // factor since last W-update (0 = disabled)

// Original HybridR: theta = |gap|/m heuristic.
GeodesicResult SolveGeodesicHybridR(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
