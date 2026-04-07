#pragma once
#include <Eigen/Dense>
#include "conex/common/kkt_solver_interface.h"

namespace conex {

struct GeodesicResult {
  int iterations;
  double d_inf_norm;   // final ||d||_inf
  double d_sq_norm;    // final ||d||^2
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
    double tolerance);

// Find the largest k such that ||d(k)||_inf <= 1, where d(k) = d0 + k * d1.
// Requires W to be centered (d ≈ 0 at the current k).  Uses one factorization
// and two back-solves.  Returns the new k (>= current k).
double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W);

}  // namespace conex
