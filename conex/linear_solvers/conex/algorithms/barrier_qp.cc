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

  // Build problem: A'WA (reweighted) + Q.
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  auto c_ineq = problem.AddLinearConstraint(A, Eigen::VectorXd::Zero(m), vars);
  problem.AddQuadraticCost(Q, vars);

  // Preprocess (drop structurally dependent columns).
  auto [reduced, expansion] = Preprocess(problem);
  const int nr = reduced.num_variables();

  // Reduce Q, c, x0, A to the reduced space.
  Eigen::VectorXd c_r = expansion.Reduce(c);
  Eigen::VectorXd x0_r = expansion.Reduce(x0);

  Eigen::MatrixXd A_r(A);
  if (expansion.was_reduced()) {
    A_r.resize(m, nr);
    for (int j = 0; j < nr; ++j)
      A_r.col(j) = Eigen::MatrixXd(A).col(expansion.col_map[j]);
  }
  Eigen::SparseMatrix<double> Q_r;
  if (expansion.was_reduced()) {
    std::vector<Eigen::Triplet<double>> qt;
    std::vector<int> inv(n, -1);
    for (int i = 0; i < nr; ++i) inv[expansion.col_map[i]] = i;
    for (int k = 0; k < Q.outerSize(); ++k)
      for (Eigen::SparseMatrix<double>::InnerIterator it(Q, k); it; ++it) {
        int ri = inv[it.row()], rj = inv[it.col()];
        if (ri >= 0 && rj >= 0) qt.emplace_back(ri, rj, it.value());
      }
    Q_r.resize(nr, nr);
    Q_r.setFromTriplets(qt.begin(), qt.end());
  } else {
    Q_r = Q;
  }

  auto solver = Solver::Build(reduced);
  solver.AssembleAndFactor();

  auto rhs_bv = solver.MakeBlockVariable();
  auto dx_bv = solver.MakeBlockVariable();

  auto t_start = clock::now();

  Eigen::VectorXd x = x0_r;
  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x.
      Eigen::VectorXd s = b - A_r * x;
      if (s.minCoeff() <= 0) break;

      // Barrier weights: W_ii = 1 / (t * s_i^2).
      Eigen::VectorXd weights(m);
      for (int i = 0; i < m; ++i)
        weights(i) = 1.0 / (t * s(i) * s(i));
      solver.SetWeights(c_ineq, weights);

      // Gradient: grad = Q x + c + (1/t) A^T (1/s).
      Eigen::VectorXd inv_s(m);
      for (int i = 0; i < m; ++i) inv_s(i) = 1.0 / s(i);
      Eigen::VectorXd grad =
          Q_r * x + c_r + (1.0 / t) * (A_r.transpose() * inv_s);

      // Solve (Q + A^T W A) dx = -grad.
      if (!solver.AssembleAndFactor()) break;
      rhs_bv.ScatterFrom(-grad);
      solver.SolveInto(rhs_bv, dx_bv);
      Eigen::VectorXd dx = dx_bv.Gather();

      // Newton decrement.
      double lambda_sq = -grad.dot(dx);
      if (lambda_sq / 2.0 < tolerance * 0.01) break;

      // Backtracking line search.
      double alpha = 1.0;
      Eigen::VectorXd Adx = A_r * dx;
      for (int i = 0; i < m; ++i) {
        if (Adx(i) > 0)
          alpha = std::min(alpha, 0.99 * s(i) / Adx(i));
      }

      const double beta = 0.5;
      const double armijo = 0.01;
      double f0 = 0.5 * x.dot(Q_r * x) + c_r.dot(x);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      for (int ls = 0; ls < 20; ++ls) {
        Eigen::VectorXd x_new = x + alpha * dx;
        Eigen::VectorXd s_new = b - A_r * x_new;
        if (s_new.minCoeff() <= 0) { alpha *= beta; continue; }
        double f_new = 0.5 * x_new.dot(Q_r * x_new) + c_r.dot(x_new);
        for (int i = 0; i < m; ++i)
          f_new -= (1.0 / t) * std::log(s_new(i));
        if (f_new <= f0 + armijo * alpha * grad.dot(dx)) break;
        alpha *= beta;
      }

      x += alpha * dx;
    }

    t *= mu;
  }

  auto t_end = clock::now();
  result.x = expansion.Expand(x);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

}  // namespace conex
