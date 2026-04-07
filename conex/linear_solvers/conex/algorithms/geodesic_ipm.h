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
};

struct GeodesicResult {
  int iterations;
  double d_inf_norm;   // final ||d||_inf
  double d_sq_norm;    // final ||d||^2
  double mu;           // barrier parameter 1/k^2
  double complementarity;  // mu * (rank - ||d||^2)
  std::vector<GeodesicIterStats> iter_stats;
};

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
    Eigen::VectorXd& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose = false);

// Run the full geodesic IPM: repeated line-search for k then center.
// Returns per-outer-iteration stats for comparison with barrier method.
GeodesicResult SolveGeodesicLP(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
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
    const Eigen::VectorXd& W);

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
    Eigen::VectorXd& W,
    const Eigen::VectorXd& r,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose = false);

// Line search for k with per-component r.
double GeodesicLineSearchR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W,
    const Eigen::VectorXd& r);

// Full solve with per-component r.
GeodesicResult SolveGeodesicLPR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    const Eigen::VectorXd& r,
    int max_outer_iterations = 30,
    int max_centering_steps = 1,
    double tolerance = 1e-8,
    bool verbose = false);

// Geodesic IPM with Mehrotra predictor-corrector.
// One factorization + two back-solves per outer iteration.
GeodesicResult SolveGeodesicMehrotra(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    int max_iterations = 50,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
