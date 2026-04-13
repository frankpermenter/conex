// Geodesic interior-point method for linear programs.
//
// Parameterization
// ----------------
// The primal slack s and dual variable lambda are parameterized on the
// central path via a weight vector W = exp(v):
//
//   s(mu)      = sqrt(mu) * exp(-v)
//   lambda(mu) = sqrt(mu) * exp(v)
//
// so that s * lambda = mu identically.  The weight W_i = exp(v_i) is the
// sole state variable; mu = 1/k^2 is the barrier parameter.
//
// Newton step
// -----------
// A Newton step computes a direction d in v-space.  The linearizations
//
//   exp(v - d) ≈ exp(v) (1 - d)      (primal slack update)
//   exp(v + d) ≈ exp(v) (1 + d)      (dual variable update)
//
// lead to the KKT system (A^T diag(W^2) A) y = RHS, from which d is
// recovered as d = 1 + W .* (k*b - A*y).  The centering term "1"
// pulls s*lambda toward mu; the A*y term moves toward optimality.
//
// The geodesic update W *= exp(alpha * d) preserves positivity and
// corresponds to a step along the geodesic on the manifold of positive
// diagonal scalings.
//
// Convergence
// -----------
// At the central path, d = 0 and complementarity = mu * m.  The step
// size alpha = min(1, 2/||d||^2_inf) ensures convergence.  The line
// search for k exploits d(k) = d0 + k * d1 (affine in k) to find the
// largest k with ||d||_inf <= 1 analytically.

#pragma once
#include <vector>
#include <Eigen/Dense>
#include "conex/common/kkt_solver_interface.h"

namespace conex {

struct GeodesicIterStats {
  double mu;
  double d_inf;
  double d_sqr;
  double complementarity;  // mu * (rank - d_sqr)
  int r_updates = 0;       // r-updates since last factorization (hybrid)
  double min_slack = 0;    // min(r_i - |r_i * d_i|)
};

struct OptimalityReport {
  double primal_residual = 0;   // ||s - (Ax + b)||
  double dual_residual = 0;     // ||A^T λ - Qx - c||
  double complementarity = 0;   // <s, λ>
  double min_slack = 0;          // min eigenvalue of s
  double min_dual = 0;           // min eigenvalue of λ
  double mu = 0;
};

struct GeodesicResult {
  int iterations;
  double d_inf_norm;   // final ||d||_inf
  double d_sq_norm;    // final ||d||^2
  double mu;           // barrier parameter 1/k^2
  double complementarity;  // mu * (rank - ||d||^2)
  int total_factorizations = 0;
  int total_solves = 0;
  Eigen::VectorXd x;  // primal variable: x = y/k from last Newton solve
  std::vector<GeodesicIterStats> iter_stats;
  OptimalityReport optimality;
};

// Verify the Newton direction satisfies its defining equations:
//   Primal: P(W^{-1/2})(I - d) = k*(Ay + b)   (slack condition)
//   Dual:   A^T P(W^{1/2})(I + d) / k = c      (cost condition)
// Returns (primal_residual, dual_residual) norms.
std::pair<double, double> VerifyNewtonEquations(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& d,
    const Eigen::VectorXd& y,
    double k);

// Run the geodesic centering iteration with fixed barrier parameter k = 1/sqrt(mu).
// Maintains weight vector W as the sole state variable, updated via W *= exp(alpha * d).
//
// cost_rhs: the cost vector c in SolverRHS format (for min c^T x s.t. Ax <= b).
// W: initial weight vector (m-dimensional, positive). Modified in-place.
// k: barrier parameter 1/sqrt(mu), held fixed.
// max_iterations: iteration limit.
// tolerance: stop when ||d||_inf < tolerance.
GeodesicResult GeodesicCenter(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose = false);

// Run the full geodesic IPM: repeated line-search for k then center.
// Returns per-outer-iteration stats for comparison with barrier method.
GeodesicResult SolveGeodesicLP(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_outer_iterations = 30,
    int max_centering_steps = 1,
    double tolerance = 1e-8,
    bool verbose = false);

// Find the largest k such that ||d(k)||_inf <= 1, where d(k) = d0 + k * d1.
// Requires W to be centered (d ≈ 0 at the current k).  Uses one factorization
// and two back-solves.  Returns the new k (>= current k).
double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W);

// =====================================================================
// Generalized geodesic IPM with per-component centering vector r.
//
// Parameterization:
//   s_i = r_i * exp(-v_i),   lambda_i = r_i * exp(v_i)
//   s_i * lambda_i = r_i^2
//
// When r = sqrt(mu) * ones, this reduces to the scalar-mu version above.
// =====================================================================

// Centering with per-component r (fixed k and r).
GeodesicResult GeodesicCenterR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    const RowSpace& r,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose = false);

// Line search for k with per-component r.
double GeodesicLineSearchR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    const RowSpace& r);

// Full solve with per-component r.
GeodesicResult SolveGeodesicLPR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    const RowSpace& r,
    int max_outer_iterations = 30,
    int max_centering_steps = 1,
    double tolerance = 1e-8,
    bool verbose = false);

// Result of a single hybrid direction computation.
struct HybridDirection {
  double gap;
  double d_inf;
  double d_sq;
  double min_slack;
};

// Check optimality conditions given primal x (as SolverRHS) and dual λ (as RowSpace).
// Computes s = Ax + b, dual residual A^T λ - Qx - c, complementarity <s, λ>.
OptimalityReport CheckOptimality(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const SolverRHS& x_rhs,
    const RowSpace& lambda);

// Compute the hybrid Newton direction at (W, r).
// Assumes the KKT system is already factored with weights W².
// Returns d, delta (via output params), and derived quantities.
// Performs one back-solve (no factorization).
HybridDirection ComputeHybridDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta);

// Single centering step: factor with W², compute direction, take
// geodesic/automorphism step.  Modifies W and r in place.
// Returns the direction info (gap, d_inf, etc.).
HybridDirection HybridCenteringStep(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    RowSpace& r);

// Hybrid geodesic IPM: alternates between centering (when gap < 0)
// and shrinking per-component centering targets r (when gap >= 0).
// gap(r, d) = <r.*(1+d), r.*(1-d)> = sum(r_i^2 * (1 - d_i^2)).
GeodesicResult SolveGeodesicHybrid(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations = 50,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
