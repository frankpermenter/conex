#include "conex/algorithms/barrier_qp.h"

#include <chrono>
#include <cmath>
#include <numeric>

#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"

namespace conex {

BarrierQPResult SolveBarrierQP(
    KKTSolverBase& kkt,
    const SolverRHS& c_rhs,
    SolverRHS& x,
    int max_outer_iterations,
    int max_newton_steps,
    double mu,
    double tolerance) {
  using clock = std::chrono::high_resolution_clock;
  BarrierQPResult result;
  const int nr = kkt.number_of_variables();

  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();
  result.total_newton_steps = 0;
  result.outer_iterations = 0;

  kkt.AssembleAndFactor();

  auto dx = kkt.MakeSolverRHS();
  auto grad = kkt.MakeSolverRHS();
  auto qx = kkt.MakeSolverRHS();       // reusable Q*x storage
  auto row = kkt.MakeRowSpace();
  auto row_trial = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();      // #5: allocate once
  RowSpace scaled_inv_s = kkt.MakeRowSpace(); // #5: allocate once
  auto x_trial = kkt.MakeSolverRHS();

  auto t_start = clock::now();

  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    int newton_this_outer = 0;
    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;
      newton_this_outer++;

      // Slacks: s = A*x + b  (Ax + b >= 0 convention).
      kkt.MultiplyA(x, row);
      row.col() = row.col() + b;  // row now holds s
      const auto& s = row.col();
      if (s.minCoeff() <= 0) break;

      // Barrier weights: w_i = 1/(t * s_i^2).
      weights.col() = (t * s.cwiseProduct(s)).cwiseInverse();
      kkt.SetWeights(weights);

      // A^T term: -(1/t) * A^T * (1/s)  (barrier gradient for s = Ax + b).
      scaled_inv_s.col() = -(t * s).cwiseInverse();

      // Compute Q*x once (ungathered), reuse for gradient and objective.
      qx.SetZero();
      kkt.AccumulateQx(x, qx);

      // Build gradient: grad = Q*x + c + (1/t) A^T(1/s).
      // qx has pending separators — += propagates them into grad.
      grad = c_rhs;
      grad += qx;
      kkt.AccumulateAtranspose(scaled_inv_s, grad);
      // grad has pending separators — solve and dot handle lazily.

      // Solve (Q + A^T W A) dx = -grad.
      if (!kkt.AssembleAndFactor()) break;
      dx = grad;
      dx *= -1.0;
      kkt.SolveSolverRHS(dx);

      // Newton decrement (lazy gather on grad).
      double lambda_sq = kkt.dot(grad, dx);
      if (-lambda_sq / 2.0 < tolerance * 0.01) break;

      // Max step for feasibility: s + α*A*dx > 0.
      double alpha = 1.0;
      kkt.MultiplyA(dx, row_trial);
      for (int i = 0; i < m; ++i) {
        if (row_trial.col()(i) < 0)
          alpha = std::min(alpha, -0.99 * s(i) / row_trial.col()(i));
      }

      // Objective at current point (lazy gather on qx).
      double f0 = 0.5 * kkt.dot(x, qx) + x.dot(c_rhs);
      f0 -= (1.0 / t) * s.array().log().sum();

      // Backtracking line search.
      const double beta = 0.5;
      const double armijo = 0.01;
      for (int ls = 0; ls < 20; ++ls) {
        x_trial = x;
        x_trial.AddScaled(alpha, dx);

        kkt.MultiplyA(x_trial, row_trial);
        row_trial.col() = row_trial.col() + b;  // s_trial = A*x_trial + b
        if (row_trial.col().minCoeff() <= 0) { alpha *= beta; continue; }

        qx.SetZero();
        kkt.AccumulateQx(x_trial, qx);
        double f_new = 0.5 * kkt.dot(x_trial, qx) + x_trial.dot(c_rhs);
        f_new -= (1.0 / t) * row_trial.col().array().log().sum();

        if (f_new <= f0 + armijo * alpha * (-lambda_sq)) break;
        alpha *= beta;
      }

      x.AddScaled(alpha, dx);
    }

    result.iter_stats.push_back({newton_this_outer, gap, 1.0 / t});
    t *= mu;
  }

  auto t_end = clock::now();
  Eigen::VectorXd x_final(nr);
  x.supernodes->GatherInto(x_final);
  result.x = x_final;

  // Objective: 0.5 x^T Q x + c^T x.
  qx.SetZero();
  kkt.AccumulateQx(x, qx);
  result.objective = 0.5 * kkt.dot(x, qx) + x.dot(c_rhs);

  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

// Convenience wrapper: builds Model + Solver from raw matrices.
BarrierQPResult SolveBarrierQP(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& x0,
    int max_outer_iterations,
    int max_newton_steps,
    double mu,
    double tolerance) {
  const int n = Q.cols();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.AddQuadraticCost(Q, vars);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto c_rhs = solver.MakeCostRHS();
  auto x = kkt->MakeSolverRHS();
  x = kkt->MakeBlockVariable(solver.ReduceVector(x0));

  auto result = SolveBarrierQP(*kkt, c_rhs, x,
                                max_outer_iterations, max_newton_steps,
                                mu, tolerance);

  // Expand back to original space.
  result.x = solver.ExpandSolution(result.x);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  return result;
}

}  // namespace conex
