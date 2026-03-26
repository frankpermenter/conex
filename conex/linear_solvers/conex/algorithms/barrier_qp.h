#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Barrier method for linearly-constrained convex QP.
//
// Solves: min  0.5 x^T Q x + c^T x
//         s.t. A x <= b
//
// via the log-barrier:
//   min_x  t (0.5 x^T Q x + c^T x) - sum_i log(b_i - a_i^T x)
//
// The Newton step at each iteration solves:
//   (Q + A^T W A) dx = -(Q x + c + A^T d)
// where s = b - A x (slacks), W = diag(1/(t * s_i^2)), d = -1/(t * s_i).
//
// This maps directly to our reweighting framework:
// - Build solver once from Q and A (fixes the clique tree)
// - Each Newton step: update W via SetWeights, re-factor, solve
// - Increase t by factor mu each outer iteration
struct BarrierQPResult {
  Eigen::VectorXd x;
  int outer_iterations;
  int total_newton_steps;
  double objective;
  double duality_gap;
  double solve_time_us;
};

BarrierQPResult SolveBarrierQP(
    const Eigen::SparseMatrix<double>& Q,  // n x n, PSD
    const Eigen::VectorXd& c,              // n
    const Eigen::SparseMatrix<double>& A,  // m x n, inequality constraints
    const Eigen::VectorXd& b,              // m, A x <= b
    const Eigen::VectorXd& x0,            // n, strictly feasible start (A x0 < b)
    int max_outer_iterations = 30,
    int max_newton_steps = 50,
    double mu = 10.0,
    double tolerance = 1e-8);

}  // namespace conex
