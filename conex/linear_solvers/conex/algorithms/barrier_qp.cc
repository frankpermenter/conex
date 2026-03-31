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
  solver.AssembleAndFactor();

  // BlockVariables for the solve path.
  auto x = solver.MakeBlockVariable(expansion.Reduce(x0));
  auto dx = solver.MakeBlockVariable();
  auto grad_bv = solver.MakeBlockVariable();
  auto qx_bv = solver.MakeBlockVariable();

  auto t_start = clock::now();

  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x (uses per-clique A, reads x's blocks).
      Eigen::VectorXd s = b - solver.MultiplyA(c_ineq_id, x);
      if (s.minCoeff() <= 0) break;

      // Barrier weights.
      Eigen::VectorXd weights(m);
      for (int i = 0; i < m; ++i)
        weights(i) = 1.0 / (t * s(i) * s(i));
      solver.SetWeights(c_ineq_id, weights);

      // Gradient: grad = Q x + c + (1/t) A^T (1/s).
      Eigen::VectorXd inv_s(m);
      for (int i = 0; i < m; ++i) inv_s(i) = 1.0 / s(i);
      Eigen::VectorXd scaled_inv_s = (1.0 / t) * inv_s;

      // Solve (Q + A^T W A) dx = -(Q*x + c + (1/t) A^T(1/s)).
      if (!solver.AssembleAndFactor()) break;

      auto* ts = solver.tree_solver();
      if (ts) {
        // Fused path: build RHS in tree form, then solve.
        ts->ScatterSeparators(x.partition(), ts->sep_scratch_in());
        auto rhs = ts->MakeTreeRHS(dx);
        rhs.SetZero();
        solver.AccumulateQx(c_quad_id, x, rhs);
        solver.AccumulateAtranspose(c_ineq_id, scaled_inv_s, rhs);
        rhs += solver.MakeBlockVariable(c_r);
        rhs *= -1.0;
        ts->SolveBlockedInPlace(rhs);
      } else {
        // Dense fallback.
        solver.MultiplyQ(c_quad_id, x, qx_bv);
        Eigen::VectorXd Qx = qx_bv.Gather().col(0);
        Eigen::VectorXd at_inv_s =
            solver.ComputeTransposeProduct(c_ineq_id, scaled_inv_s);
        Eigen::VectorXd grad = Qx + c_r + at_inv_s;
        grad_bv.ScatterFrom(-grad);
        solver.SolveInto(grad_bv, dx);
      }

      // Newton decrement: lambda^2 = -grad^T dx.
      // Reconstruct grad densely for decrement and line search.
      Eigen::VectorXd dx_dense = dx.Gather().col(0);
      Eigen::VectorXd x_dense = x.Gather().col(0);
      solver.MultiplyQ(c_quad_id, x, qx_bv);
      Eigen::VectorXd Qx = qx_bv.Gather().col(0);
      Eigen::VectorXd grad = Qx + c_r;
      grad += solver.ComputeTransposeProduct(c_ineq_id, scaled_inv_s);

      double lambda_sq = grad.dot(dx_dense);  // -grad^T * dx
      if (-lambda_sq / 2.0 < tolerance * 0.01) break;

      // Backtracking line search.
      double alpha = 1.0;
      Eigen::VectorXd Adx = solver.MultiplyA(c_ineq_id, dx);
      for (int i = 0; i < m; ++i) {
        if (Adx(i) > 0)
          alpha = std::min(alpha, 0.99 * s(i) / Adx(i));
      }

      const double beta = 0.5;
      const double armijo = 0.01;
      double f0 = 0.5 * x_dense.dot(Qx) + c_r.dot(x_dense);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      for (int ls = 0; ls < 20; ++ls) {
        Eigen::VectorXd x_new = x_dense + alpha * dx_dense;
        auto x_new_bv = solver.MakeBlockVariable(x_new);
        Eigen::VectorXd s_new = b - solver.MultiplyA(c_ineq_id, x_new_bv);
        if (s_new.minCoeff() <= 0) { alpha *= beta; continue; }
        solver.MultiplyQ(c_quad_id, x_new_bv, qx_bv);
        Eigen::VectorXd Qx_new = qx_bv.Gather().col(0);
        double f_new = 0.5 * x_new.dot(Qx_new) + c_r.dot(x_new);
        for (int i = 0; i < m; ++i)
          f_new -= (1.0 / t) * std::log(s_new(i));
        if (f_new <= f0 + armijo * alpha * (-grad).dot(dx_dense)) break;
        alpha *= beta;
      }

      x.AddScaled(alpha, dx);
    }

    t *= mu;
  }

  auto t_end = clock::now();
  Eigen::VectorXd x_final = x.Gather().col(0);
  result.x = expansion.Expand(x_final);
  result.objective = 0.5 * result.x.dot(Q * result.x) + c.dot(result.x);
  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

}  // namespace conex
