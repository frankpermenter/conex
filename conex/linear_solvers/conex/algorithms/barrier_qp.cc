#include "conex/algorithms/barrier_qp.h"

#include <chrono>
#include <cmath>
#include <set>
#include <vector>

#include "conex/common/constraint_manager.h"
#include "conex/common/conex.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_solver_factory.h"

namespace conex {

BarrierQPResult SolveBarrierQP(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    int max_outer_iterations,
    int max_newton_steps_per_outer,
    double mu,
    double tolerance) {
  using clock = std::chrono::high_resolution_clock;
  BarrierQPResult result;
  const int n = Q.cols();
  result.total_newton_steps = 0;

  // Starting point: center of the box.
  Eigen::VectorXd x = 0.5 * (lb + ub);

  // Number of inequality constraints (2n for box constraints).
  const int num_ineq = 2 * n;
  double t = 1.0;  // barrier parameter

  auto t0 = clock::now();

  // For each barrier subproblem, we solve:
  //   (Q + D) dx = -grad
  // where D = (1/t)(diag(1/(x-lb)^2) + diag(1/(ub-x)^2))
  //       grad = Q*x + c - (1/t)(1/(x-lb) - 1/(ub-x))
  //
  // We build Q + D as a sparse matrix and use SparseQuadraticTermLeastSquares
  // with A = 0 (pure quadratic, no least-squares term).
  Eigen::SparseMatrix<double> A_empty(0, n);

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    // Check duality gap: num_ineq / t.
    double gap = static_cast<double>(num_ineq) / t;
    if (gap < tolerance) break;
    result.outer_iterations = outer + 1;

    // Newton's method for the barrier subproblem.
    for (int newton = 0; newton < max_newton_steps_per_outer; ++newton) {
      result.total_newton_steps++;

      // Compute barrier Hessian diagonal.
      Eigen::VectorXd sl = x - lb;  // slack lower
      Eigen::VectorXd su = ub - x;  // slack upper

      // Clamp to avoid division by zero.
      for (int i = 0; i < n; ++i) {
        sl(i) = std::max(sl(i), 1e-15);
        su(i) = std::max(su(i), 1e-15);
      }

      Eigen::VectorXd d_barrier(n);
      for (int i = 0; i < n; ++i) {
        d_barrier(i) = (1.0 / t) * (1.0 / (sl(i) * sl(i)) +
                                     1.0 / (su(i) * su(i)));
      }

      // Build Q + D as sparse.
      std::vector<Eigen::Triplet<double>> trips;
      for (int k = 0; k < Q.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it)
          trips.emplace_back(it.row(), it.col(), it.value());
      for (int i = 0; i < n; ++i)
        trips.emplace_back(i, i, d_barrier(i));
      Eigen::SparseMatrix<double> H(n, n);
      H.setFromTriplets(trips.begin(), trips.end());

      // Gradient: Q*x + c - (1/t)(1/sl - 1/su).
      Eigen::VectorXd grad = Q * x + c;
      for (int i = 0; i < n; ++i) {
        grad(i) -= (1.0 / t) * (1.0 / sl(i) - 1.0 / su(i));
      }

      // Solve H dx = -grad.
      Eigen::VectorXd neg_grad = -grad;
      auto step_result = SparseQuadraticTermLeastSquares(H, A_empty, neg_grad);
      Eigen::VectorXd dx = step_result.x;

      // Line search: ensure x + alpha*dx stays in (lb, ub).
      double alpha = 1.0;
      for (int i = 0; i < n; ++i) {
        if (dx(i) < 0) {
          double max_step = -(x(i) - lb(i)) / dx(i);
          alpha = std::min(alpha, 0.99 * max_step);
        } else if (dx(i) > 0) {
          double max_step = (ub(i) - x(i)) / dx(i);
          alpha = std::min(alpha, 0.99 * max_step);
        }
      }

      x += alpha * dx;

      // Check Newton decrement.
      double newton_decrement = dx.dot(-grad);
      if (newton_decrement / 2.0 < tolerance * 0.01) break;
    }

    // Increase barrier parameter.
    t *= mu;
  }

  auto t1 = clock::now();
  result.x = x;
  result.objective = 0.5 * x.dot(Q * x) + c.dot(x);
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
