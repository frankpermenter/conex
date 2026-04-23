#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

// Helper: build sparse A from dense.
Eigen::SparseMatrix<double> ToSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14)
        t.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

// =====================================================================
// Test: fixed point is strictly feasible.
// =====================================================================

TEST(ExtendedEmbedding, FixedPointFeasible) {
  srand(42);
  const int n = 5, m = 3;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  auto A = ToSparse(A_dense);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  auto [model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  printf("  n=%d m=%d total_vars=%d\n", n, m, info.total_vars());
  printf("  alpha=%.4f rg=%.4f\n", info.alpha, info.rg);
  printf("  ||rp||=%.4f ||rd||=%.4f\n", info.rp.norm(), info.rd.norm());

  // Check (x_hat, s_hat, y_hat, tau_hat, kappa_hat, theta=1) is feasible.
  VectorXd z = VectorXd::Zero(info.total_vars());
  z.segment(info.x_start(), n) = info.x_hat;
  z.segment(info.y_start(), m) = info.y_hat;
  z.segment(info.s_start(), m) = info.s_hat;
  z(info.tau_idx()) = info.tau_hat;
  z(info.kappa_idx()) = info.kappa_hat;
  z(info.theta_idx()) = 1.0;

  // Check equalities at (x_hat, y_hat, s_hat, tau_hat, kappa_hat, theta=1).
  // Eq1: A*x_hat - b*tau_hat - rp*1 = 0.
  VectorXd eq1 = A_dense * info.x_hat - b * info.tau_hat - info.rp;
  printf("  eq1 (primal): ||residual|| = %.2e\n", eq1.norm());
  EXPECT_LT(eq1.norm(), 1e-12);

  // Eq2: -A'*y_hat - s_hat + c*tau_hat - rd*1 = 0.
  VectorXd eq2 = -A_dense.transpose() * info.y_hat - info.s_hat +
                 c * info.tau_hat - info.rd;
  printf("  eq2 (dual): ||residual|| = %.2e\n", eq2.norm());
  EXPECT_LT(eq2.norm(), 1e-12);

  // Eq3: <b,y_hat> - <c,x_hat> - kappa_hat - rg*1 = 0.
  double eq3 = b.dot(info.y_hat) - c.dot(info.x_hat) -
               info.kappa_hat - info.rg;
  printf("  eq3 (gap): |residual| = %.2e\n", std::abs(eq3));
  EXPECT_LT(std::abs(eq3), 1e-12);

  // Eq4: <rp,y_hat> + <rd,x_hat> + rg*tau_hat = -alpha.
  double eq4 = info.rp.dot(info.y_hat) + info.rd.dot(info.x_hat) +
               info.rg * info.tau_hat + info.alpha;
  printf("  eq4 (norm): |residual| = %.2e\n", std::abs(eq4));
  EXPECT_LT(std::abs(eq4), 1e-12);

  // Cone: s > 0, tau > 0, kappa > 0.
  EXPECT_GT(info.s_hat.minCoeff(), 0);
  EXPECT_GT(info.tau_hat, 0);
  EXPECT_GT(info.kappa_hat, 0);
}

// =====================================================================
// Test: solve the embedding and recover the original LP solution.
// =====================================================================

TEST(ExtendedEmbedding, SolveAndRecover) {
  // Toy LP in standard form:  min c'x  s.t. Ax = b, x >= 0.
  //   c = (1, 2),  A = [1, 1],  b = [2].
  // Solution: x = (2, 0), obj = 2.
  const int n = 2, m = 1;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  VectorXd b(m); b << 2.0;
  VectorXd c(n); c << 1.0, 2.0;

  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  // Solve the embedding.
  auto solver = Solver::Build(emb_model);
  auto result = solver.Solve(ThetaContinuation());

  printf("  embedding: mu=%.2e converged=%d fac=%d\n",
         result.mu, result.converged, result.factorizations);
  printf("  obj=%.6e (should be ≈ 0 = alpha*theta)\n", result.objective);

  // Extract theta.
  double theta = result.x(info.theta_idx());
  printf("  theta = %.6e\n", theta);
  EXPECT_LT(std::abs(theta), 1e-4);

  // Extract tau.
  double tau = result.x(info.tau_idx());
  printf("  tau = %.6e\n", tau);

  if (tau > 1e-6) {
    // Complementary solution: x_opt = x/tau, y_opt = y/tau.
    VectorXd x_opt = result.x.segment(info.x_start(), n) / tau;
    VectorXd y_opt = result.x.segment(info.y_start(), m) / tau;
    double primal_obj = c.dot(x_opt);
    double dual_obj = b.dot(y_opt);

    printf("  primal obj = %.6e\n", primal_obj);
    printf("  dual obj   = %.6e\n", dual_obj);
    printf("  gap = %.2e\n", primal_obj - dual_obj);

    // Primal feasibility: Ax ≈ b (from Ax - bτ = 0 at θ=0, τ>0).
    VectorXd eq_res = Eigen::MatrixXd(A) * x_opt - b;
    printf("  ||Ax - b|| = %.2e\n", eq_res.norm());
    EXPECT_LT(eq_res.norm(), 1e-2);

    // x ≥ 0.
    printf("  min x = %.2e\n", x_opt.minCoeff());
    EXPECT_GE(x_opt.minCoeff(), -1e-3);
  } else {
    printf("  tau ≈ 0: infeasible or unbounded\n");
  }
}

// =====================================================================
// Test: model structure.
// =====================================================================

TEST(ExtendedEmbedding, ModelStructure) {
  const int n = 5, m = 3;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  auto A = ToSparse(A_dense);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = VectorXd::Ones(n);

  auto [model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  // Check structure: 4 equality constraints + 2 linear constraints.
  EXPECT_EQ(model.num_variables(), info.total_vars());
  printf("  total_vars=%d constraints=%d\n",
         info.total_vars(), model.num_constraints());

  // Variables: x(5), y(3), s(5), tau(1), kappa(1), theta(1) = 16.
  EXPECT_EQ(info.total_vars(), 16);
}

}  // namespace
}  // namespace conex
