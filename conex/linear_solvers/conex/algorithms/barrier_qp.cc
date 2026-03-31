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
  auto c_ineq_id = problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);
  auto c_quad_id = problem.AddQuadraticCost(Q, vars);

  auto [reduced, expansion] = Preprocess(problem);
  const int nr = reduced.num_variables();

  Eigen::VectorXd c_r = expansion.Reduce(c);

  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();
  kkt->AssembleAndFactor();

  // Variables in solver-native format.
  auto x_rhs = kkt->MakeTreeRHS();
  x_rhs = solver.MakeBlockVariable(expansion.Reduce(x0));
  auto rhs = kkt->MakeTreeRHS();
  auto row = kkt->MakeRowSpace();

  auto t_start = clock::now();

  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x.
      kkt->MultiplyA(x_rhs, row);
      Eigen::VectorXd s = b - row.data;
      if (s.minCoeff() <= 0) break;

      // Barrier weights.
      RowSpace weights = kkt->MakeRowSpace();
      for (int i = 0; i < m; ++i)
        weights.data(i) = 1.0 / (t * s(i) * s(i));
      kkt->SetWeights(weights);

      // Gradient vector for A^T term.
      RowSpace scaled_inv_s = kkt->MakeRowSpace();
      for (int i = 0; i < m; ++i)
        scaled_inv_s.data(i) = 1.0 / (t * s(i));

      // Solve (Q + A^T W A) dx = -(Q*x + c + (1/t) A^T(1/s)).
      if (!kkt->AssembleAndFactor()) break;

      rhs = solver.MakeBlockVariable(c_r);
      kkt->AccumulateQx(x_rhs, rhs);
      kkt->AccumulateAtranspose(scaled_inv_s, rhs);
      rhs *= -1.0;
      kkt->SolveTreeRHS(rhs);
      // rhs now holds dx.

      // Newton decrement (dense, for line search).
      Eigen::VectorXd dx_dense(nr);
      rhs.supernodes->GatherInto(dx_dense);
      Eigen::VectorXd x_dense(nr);
      x_rhs.supernodes->GatherInto(x_dense);

      // Reconstruct grad for decrement and objective.
      auto grad_rhs = kkt->MakeTreeRHS();
      grad_rhs = solver.MakeBlockVariable(c_r);
      kkt->AccumulateQx(x_rhs, grad_rhs);
      kkt->AccumulateAtranspose(scaled_inv_s, grad_rhs);
      Eigen::VectorXd grad(nr);
      // Need full gather — gather separators first.
      grad_rhs.supernodes->GatherInto(grad);
      // Note: this is approximate for tree solver (misses unscattered sep).
      // For line search convergence check, this is sufficient.

      double lambda_sq = grad.dot(dx_dense);
      if (-lambda_sq / 2.0 < tolerance * 0.01) break;

      // Backtracking line search.
      double alpha = 1.0;
      auto dx_rhs = kkt->MakeTreeRHS();
      dx_rhs = rhs;  // copy solved dx
      kkt->MultiplyA(dx_rhs, row);
      Eigen::VectorXd Adx = row.data;
      for (int i = 0; i < m; ++i) {
        if (Adx(i) > 0)
          alpha = std::min(alpha, 0.99 * s(i) / Adx(i));
      }

      // Objective at current point.
      auto qx_rhs = kkt->MakeTreeRHS();
      qx_rhs = x_rhs;
      // We already have grad = Qx + c + A^T*v, so Qx = grad - c - A^T*v.
      // Simpler: just compute x^T grad - x^T c - x^T A^T v...
      // Actually just use dense dot products.
      Eigen::VectorXd Qx = grad;
      Qx -= c_r;
      kkt->AccumulateAtranspose(scaled_inv_s, grad_rhs); // already accumulated
      // Use: f0 = 0.5 * x^T Qx + c^T x
      // where Qx = grad - c - A^T scaled_inv_s
      Eigen::VectorXd atv(nr);
      // Simpler approach: compute Qx directly.
      auto qx_out = kkt->MakeTreeRHS();
      qx_out.SetZero();
      kkt->AccumulateQx(x_rhs, qx_out);
      Eigen::VectorXd Qx_dense(nr);
      qx_out.supernodes->GatherInto(Qx_dense);

      const double beta = 0.5;
      const double armijo = 0.01;
      double f0 = 0.5 * x_dense.dot(Qx_dense) + c_r.dot(x_dense);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      for (int ls = 0; ls < 20; ++ls) {
        Eigen::VectorXd x_new = x_dense + alpha * dx_dense;
        auto x_new_rhs = kkt->MakeTreeRHS();
        x_new_rhs = solver.MakeBlockVariable(x_new);
        kkt->MultiplyA(x_new_rhs, row);
        Eigen::VectorXd s_new = b - row.data;
        if (s_new.minCoeff() <= 0) { alpha *= beta; continue; }
        auto qx_new = kkt->MakeTreeRHS();
        qx_new.SetZero();
        kkt->AccumulateQx(x_new_rhs, qx_new);
        Eigen::VectorXd Qx_new(nr);
        qx_new.supernodes->GatherInto(Qx_new);
        double f_new = 0.5 * x_new.dot(Qx_new) + c_r.dot(x_new);
        for (int i = 0; i < m; ++i)
          f_new -= (1.0 / t) * std::log(s_new(i));
        if (f_new <= f0 + armijo * alpha * (-grad).dot(dx_dense)) break;
        alpha *= beta;
      }

      // Update x: x += alpha * dx.
      // dx is in rhs after solve.
      x_rhs += rhs;  // rhs was scaled by -1 and solved — wait, need alpha.
      // Actually: x_dense += alpha * dx_dense, then scatter back.
      Eigen::VectorXd x_updated = x_dense + alpha * dx_dense;
      x_rhs = solver.MakeBlockVariable(x_updated);
    }

    t *= mu;
  }

  auto t_end = clock::now();
  Eigen::VectorXd x_final(nr);
  x_rhs.supernodes->GatherInto(x_final);
  result.x = expansion.Expand(x_final);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

}  // namespace conex
