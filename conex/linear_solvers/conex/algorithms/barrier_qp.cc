#include "conex/algorithms/barrier_qp.h"

#include <chrono>
#include <cmath>
#include <numeric>

#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {

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
  using clock = std::chrono::high_resolution_clock;
  BarrierQPResult result;
  const int n = Q.cols();
  const int m = A.rows();
  result.total_newton_steps = 0;
  result.outer_iterations = 0;

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  auto [reduced, expansion] = Preprocess(problem);
  const int nr = reduced.num_variables();

  Eigen::VectorXd c_r = expansion.Reduce(c);

  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();
  kkt->AssembleAndFactor();

  auto x = kkt->MakeTreeRHS();
  x = solver.MakeBlockVariable(expansion.Reduce(x0));
  auto dx = kkt->MakeTreeRHS();
  auto grad = kkt->MakeTreeRHS();
  auto row = kkt->MakeRowSpace();
  auto c_bv = solver.MakeBlockVariable(c_r);
  auto x_trial = kkt->MakeTreeRHS();
  auto qx_trial = kkt->MakeTreeRHS();

  auto t_start = clock::now();

  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x.
      kkt->MultiplyA(x, row);
      Eigen::VectorXd s = b - row.data;
      if (s.minCoeff() <= 0) break;

      // Barrier weights: w_i = 1/(t * s_i^2).
      RowSpace weights = kkt->MakeRowSpace();
      for (int i = 0; i < m; ++i)
        weights.data(i) = 1.0 / (t * s(i) * s(i));
      kkt->SetWeights(weights);

      // A^T term: (1/t) * A^T * (1/s).
      RowSpace scaled_inv_s = kkt->MakeRowSpace();
      for (int i = 0; i < m; ++i)
        scaled_inv_s.data(i) = 1.0 / (t * s(i));

      // Build gradient: grad = Q*x + c + (1/t) A^T(1/s).
      grad = c_bv;
      kkt->AccumulateQx(x, grad);
      kkt->AccumulateAtranspose(scaled_inv_s, grad);
      kkt->GatherSeparators(grad);

      // Solve (Q + A^T W A) dx = -grad.
      if (!kkt->AssembleAndFactor()) break;
      dx = grad;
      dx *= -1.0;
      kkt->SolveTreeRHS(dx);

      // Newton decrement: lambda^2 = grad^T * dx (block-wise dot).
      double lambda_sq = grad.dot(dx);
      if (-lambda_sq / 2.0 < tolerance * 0.01) break;

      // Backtracking line search.
      double alpha = 1.0;
      kkt->MultiplyA(dx, row);
      for (int i = 0; i < m; ++i) {
        if (row.data(i) > 0)
          alpha = std::min(alpha, 0.99 * s(i) / row.data(i));
      }

      // Objective at current point: f = 0.5 x^T Q x + c^T x - (1/t) sum log(s).
      // x^T Q x = (grad - c - A^T scaled_inv_s)^T x = grad^T x - c^T x - scaled_inv_s^T (Ax)
      // But simpler: use Q*x already in grad.
      // grad = Qx + c + Atv, so Qx = grad - c - Atv.
      // x^T Qx via: qx = grad - c_bv - atv.  Then x.dot(qx).
      // Actually, just compute f0 = 0.5 * (grad - c_bv).dot(x) + c_bv.dot(x) - barrier
      //                            = 0.5 * grad.dot(x) + 0.5 * c_bv.dot(x) - barrier
      // No — that includes the A^T term.  Simpler to compute Qx directly.
      qx_trial.SetZero();
      kkt->AccumulateQx(x, qx_trial);
      kkt->GatherSeparators(qx_trial);
      double f0 = 0.5 * x.dot(qx_trial) + x.dot(c_bv);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      const double beta = 0.5;
      const double armijo = 0.01;
      double grad_dot_dx = lambda_sq;  // = grad^T dx

      for (int ls = 0; ls < 20; ++ls) {
        // x_trial = x + alpha * dx.
        x_trial = x;
        x_trial.AddScaled(alpha, dx);

        kkt->MultiplyA(x_trial, row);
        Eigen::VectorXd s_new = b - row.data;
        if (s_new.minCoeff() <= 0) { alpha *= beta; continue; }

        qx_trial.SetZero();
        kkt->AccumulateQx(x_trial, qx_trial);
        kkt->GatherSeparators(qx_trial);
        double f_new = 0.5 * x_trial.dot(qx_trial) + x_trial.dot(c_bv);
        for (int i = 0; i < m; ++i)
          f_new -= (1.0 / t) * std::log(s_new(i));

        if (f_new <= f0 + armijo * alpha * (-grad_dot_dx)) break;
        alpha *= beta;
      }

      // Update x += alpha * dx.
      x.AddScaled(alpha, dx);
    }

    t *= mu;
  }

  auto t_end = clock::now();
  Eigen::VectorXd x_final(nr);
  x.supernodes->GatherInto(x_final);
  result.x = expansion.Expand(x_final);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

}  // namespace conex
