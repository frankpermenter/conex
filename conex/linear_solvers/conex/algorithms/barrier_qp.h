#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Barrier method for box-constrained quadratic programming.
//
// Solves: min  0.5 x^T Q x + c^T x
//         s.t. lb <= x <= ub
//
// via iterating on the barrier subproblem:
//   min  0.5 x^T Q x + c^T x - (1/t) sum log(x_i - lb_i)
//                               - (1/t) sum log(ub_i - x_i)
//
// The Newton step solves:
//   (Q + D(x,t)) dx = -(Q x + c - (1/t)(d_lower - d_upper))
// where D(x,t) = (1/t)(diag(1/(x-lb)^2) + diag(1/(ub-x)^2))
// is a diagonal Hessian of the barrier terms.
//
// This maps to our reweighting framework: D is a diagonal matrix that
// changes each iteration. We solve (Q + A^T W A + D) dx = rhs
// where A^T W A is the original quadratic term (if any) and D comes
// from the barrier.
//
// For pure QP with Q sparse, we use SparseQuadraticTermLeastSquares
// with Q_iter = Q + D(x,t) at each Newton step.
struct BarrierQPResult {
  Eigen::VectorXd x;
  int outer_iterations;   // barrier parameter updates
  int total_newton_steps;
  double objective;
  double solve_time_us;
};

BarrierQPResult SolveBarrierQP(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    int max_outer_iterations = 20,
    int max_newton_steps_per_outer = 20,
    double mu = 10.0,       // barrier parameter growth rate
    double tolerance = 1e-8);

}  // namespace conex
