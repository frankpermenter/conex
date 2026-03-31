#include "conex/algorithms/barrier_qp.h"

#include <chrono>
#include <cmath>
#include <numeric>

#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {

BarrierQPResult SolveBarrierQP(
    KKTSolverBase& kkt,
    const TreeRHS& c_rhs,
    TreeRHS& x,
    int max_outer_iterations,
    int max_newton_steps,
    double mu,
    double tolerance) {
  using clock = std::chrono::high_resolution_clock;
  BarrierQPResult result;
  const int nr = kkt.number_of_variables();

  RowSpace b_row = kkt.GetAffineTerm();
  const Eigen::VectorXd& b = b_row.data;
  const int m = b.size();
  result.total_newton_steps = 0;
  result.outer_iterations = 0;

  kkt.AssembleAndFactor();

  auto dx = kkt.MakeTreeRHS();
  auto grad = kkt.MakeTreeRHS();
  auto row = kkt.MakeRowSpace();
  auto x_trial = kkt.MakeTreeRHS();
  auto qx_trial = kkt.MakeTreeRHS();

  auto t_start = clock::now();

  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x.
      kkt.MultiplyA(x, row);
      Eigen::VectorXd s = b - row.data;
      if (s.minCoeff() <= 0) break;

      // Barrier weights: w_i = 1/(t * s_i^2).
      RowSpace weights = kkt.MakeRowSpace();
      for (int i = 0; i < m; ++i)
        weights.data(i) = 1.0 / (t * s(i) * s(i));
      kkt.SetWeights(weights);

      // A^T term: (1/t) * A^T * (1/s).
      RowSpace scaled_inv_s = kkt.MakeRowSpace();
      for (int i = 0; i < m; ++i)
        scaled_inv_s.data(i) = 1.0 / (t * s(i));

      // Build gradient: grad = Q*x + c + (1/t) A^T(1/s).
      grad = c_rhs;
      kkt.AccumulateQx(x, grad);
      kkt.AccumulateAtranspose(scaled_inv_s, grad);
      kkt.GatherSeparators(grad);

      // Solve (Q + A^T W A) dx = -grad.
      if (!kkt.AssembleAndFactor()) break;
      dx = grad;
      dx *= -1.0;
      kkt.SolveTreeRHS(dx);

      // Newton decrement.
      double lambda_sq = grad.dot(dx);
      if (-lambda_sq / 2.0 < tolerance * 0.01) break;

      // Max step for feasibility.
      double alpha = 1.0;
      kkt.MultiplyA(dx, row);
      for (int i = 0; i < m; ++i) {
        if (row.data(i) > 0)
          alpha = std::min(alpha, 0.99 * s(i) / row.data(i));
      }

      // Objective at current point.
      qx_trial.SetZero();
      kkt.AccumulateQx(x, qx_trial);
      kkt.GatherSeparators(qx_trial);
      double f0 = 0.5 * x.dot(qx_trial) + x.dot(c_rhs);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      // Backtracking line search.
      const double beta = 0.5;
      const double armijo = 0.01;
      for (int ls = 0; ls < 20; ++ls) {
        x_trial = x;
        x_trial.AddScaled(alpha, dx);

        kkt.MultiplyA(x_trial, row);
        Eigen::VectorXd s_new = b - row.data;
        if (s_new.minCoeff() <= 0) { alpha *= beta; continue; }

        qx_trial.SetZero();
        kkt.AccumulateQx(x_trial, qx_trial);
        kkt.GatherSeparators(qx_trial);
        double f_new = 0.5 * x_trial.dot(qx_trial) + x_trial.dot(c_rhs);
        for (int i = 0; i < m; ++i)
          f_new -= (1.0 / t) * std::log(s_new(i));

        if (f_new <= f0 + armijo * alpha * (-lambda_sq)) break;
        alpha *= beta;
      }

      x.AddScaled(alpha, dx);
    }

    t *= mu;
  }

  auto t_end = clock::now();
  Eigen::VectorXd x_final(nr);
  x.supernodes->GatherInto(x_final);
  result.x = x_final;

  // Objective: 0.5 x^T Q x + c^T x.
  qx_trial.SetZero();
  kkt.AccumulateQx(x, qx_trial);
  kkt.GatherSeparators(qx_trial);
  result.objective = 0.5 * x.dot(qx_trial) + x.dot(c_rhs);

  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

// Convenience wrapper: builds Problem + Solver from raw matrices.
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
  const int m = A.rows();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, b, vars);
  problem.AddQuadraticCost(Q, vars);

  auto consolidated = problem.Consolidate();
  auto [reduced, expansion] = Preprocess(consolidated);

  Eigen::VectorXd c_r = expansion.Reduce(c);
  Eigen::VectorXd x0_r = expansion.Reduce(x0);

  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto c_rhs = kkt->MakeTreeRHS();
  c_rhs = kkt->MakeBlockVariable(c_r);
  auto x = kkt->MakeTreeRHS();
  x = kkt->MakeBlockVariable(x0_r);

  auto result = SolveBarrierQP(*kkt, c_rhs, x,
                                max_outer_iterations, max_newton_steps,
                                mu, tolerance);

  // Expand back to original space.
  result.x = expansion.Expand(result.x);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  return result;
}

}  // namespace conex
