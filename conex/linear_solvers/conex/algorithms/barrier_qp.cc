#include "conex/algorithms/barrier_qp.h"

#include <chrono>
#include <cmath>
#include <set>

#include "conex/common/constraint_manager.h"
#include "conex/common/conex.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_solver_factory.h"

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

  // Build solver once: the Newton system is (Q + A^T W A) dx = rhs.
  // Q is the quadratic cost, A defines the inequality constraints.
  // W changes each iteration but the sparsity structure is fixed.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(m);
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);
  std::set<int> var_set;
  for (const auto& sup : slc->row_supports())
    var_set.insert(sup.begin(), sup.end());
  for (int i = 0; i < n; ++i) var_set.insert(i);
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(n);
  auto a_assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  auto* a_asm_ptr = a_assembler.get();
  cm.AddCustomAssembler(a_asm_ptr);

  auto q_assembler = std::make_unique<SparseQuadraticTermAssembler>(Q, all_vars);
  cm.AddCustomAssembler(q_assembler.get());

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&cm, config);

  auto t_start = clock::now();

  Eigen::VectorXd x = x0;
  double t = 1.0;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    result.outer_iterations = outer + 1;

    // Duality gap estimate: m / t.
    double gap = static_cast<double>(m) / t;
    if (gap < tolerance) break;

    // Newton's method on the barrier subproblem.
    for (int newton = 0; newton < max_newton_steps; ++newton) {
      result.total_newton_steps++;

      // Slacks: s = b - A x.  ComputeResiduals returns A*x (since
      // the SparseLinearConstraint was built with b_zero).
      Eigen::VectorXd s = b - a_asm_ptr->ComputeResiduals(x);

      // Check feasibility.
      if (s.minCoeff() <= 0) {
        // Infeasible — should not happen with proper line search.
        break;
      }

      // Barrier weights: W_ii = 1 / (t * s_i^2).
      Eigen::VectorXd weights(m);
      for (int i = 0; i < m; ++i) {
        weights(i) = 1.0 / (t * s(i) * s(i));
      }
      a_asm_ptr->SetWeights(weights);

      // Gradient of barrier subproblem:
      //   grad = Q x + c + (1/t) A^T (1/s)
      //        = Q x + c - A^T d   where d = -1/(t * s)
      Eigen::VectorXd inv_s(m);
      for (int i = 0; i < m; ++i) inv_s(i) = 1.0 / s(i);
      Eigen::VectorXd grad = Q * x + c + (1.0 / t) * A.transpose() * inv_s;

      // Solve (Q + A^T W A) dx = -grad.
      bool ok = solver->AssembleAndFactor();
      if (!ok) break;
      Eigen::VectorXd dx = solver->Solve(-grad);

      // Newton decrement: lambda^2 = -grad^T dx.
      double lambda_sq = -grad.dot(dx);
      if (lambda_sq / 2.0 < tolerance * 0.01) break;

      // Backtracking line search to maintain feasibility.
      double alpha = 1.0;

      // Max step to stay feasible: s - alpha * A dx > 0.
      Eigen::VectorXd Adx = a_asm_ptr->ComputeResiduals(dx);
      for (int i = 0; i < m; ++i) {
        if (Adx(i) > 0) {
          // s(i) - alpha * Adx(i) > 0  =>  alpha < s(i) / Adx(i)
          alpha = std::min(alpha, 0.99 * s(i) / Adx(i));
        }
      }

      // Backtracking on barrier objective.
      const double beta = 0.5;
      const double armijo = 0.01;
      double f0 = 0.5 * x.dot(Q * x) + c.dot(x);
      for (int i = 0; i < m; ++i) f0 -= (1.0 / t) * std::log(s(i));

      for (int ls = 0; ls < 20; ++ls) {
        Eigen::VectorXd x_new = x + alpha * dx;
        Eigen::VectorXd s_new = b - a_asm_ptr->ComputeResiduals(x_new);
        if (s_new.minCoeff() <= 0) {
          alpha *= beta;
          continue;
        }
        double f_new = 0.5 * x_new.dot(Q * x_new) + c.dot(x_new);
        for (int i = 0; i < m; ++i) f_new -= (1.0 / t) * std::log(s_new(i));
        if (f_new <= f0 + armijo * alpha * grad.dot(dx)) break;
        alpha *= beta;
      }

      x += alpha * dx;
    }

    // Increase barrier parameter.
    t *= mu;
  }

  auto t_end = clock::now();
  result.x = x;
  result.objective = 0.5 * x.dot(Q * x) + c.dot(x);
  result.duality_gap = static_cast<double>(m) / t;
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t_end - t_start).count();
  return result;
}

}  // namespace conex
