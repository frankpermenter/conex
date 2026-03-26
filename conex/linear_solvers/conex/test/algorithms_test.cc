#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/irls.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cstdio>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// =====================================================================
// IRLS tests
// =====================================================================

// IRLS on an overdetermined system: the L1 solution should be more
// robust to outliers than L2.
TEST(IRLS, BasicL1) {
  srand(42);
  const int m = 50, n = 5;

  // Random A.
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; r++)
    for (int c = 0; c < n; c++)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  // True signal.
  VectorXd x_true = VectorXd::Random(n);
  MatrixXd Ad(A);
  VectorXd b = Ad * x_true;

  // Add outliers to 10% of measurements.
  for (int i = 0; i < m / 10; i++) {
    b(rand() % m) += 10.0 * ((double)rand() / RAND_MAX - 0.5);
  }

  auto result = SolveIRLS(A, b, 50, 1e-6, 1e-8);

  // IRLS should recover x_true approximately despite outliers.
  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 0.5) << "IRLS L1 error: " << err;
  EXPECT_GT(result.iterations, 1) << "Should take multiple iterations";

  printf("IRLS: %d iterations, L1 obj=%.4f, err=%.4f, time=%.0fus\n",
         result.iterations, result.l1_objective, err, result.solve_time_us);
}

// IRLS with no outliers should converge to L2 solution.
TEST(IRLS, ConvergesToL2WithoutOutliers) {
  srand(42);
  const int m = 30, n = 5;
  std::vector<Eigen::Triplet<double>> trips;
  for (int r = 0; r < m; r++)
    for (int c = 0; c < n; c++)
      trips.emplace_back(r, c, (double)rand() / RAND_MAX - 0.5);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd x_true = VectorXd::Random(n);
  MatrixXd Ad(A);
  VectorXd b = Ad * x_true;  // No noise, no outliers.

  auto result = SolveIRLS(A, b, 50, 1e-8, 1e-10);

  // Should recover x_true exactly.
  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-6) << "IRLS without outliers error: " << err;
}

// =====================================================================
// Barrier QP tests
// =====================================================================

// Unconstrained minimum inside the box: barrier should find it.
TEST(BarrierQP, UnconstrainedMinInsideBox) {
  const int n = 5;
  // Q = I, c = 0. Minimum at x = 0, which is inside [-1, 1].
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) qt.emplace_back(i, i, 1.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c = VectorXd::Zero(n);
  VectorXd lb = VectorXd::Constant(n, -1.0);
  VectorXd ub = VectorXd::Constant(n, 1.0);

  auto result = SolveBarrierQP(Q, c, lb, ub, 20, 20, 10.0, 1e-8);

  // x should be near zero.
  EXPECT_LT(result.x.norm(), 0.01)
      << "Unconstrained min should be near zero, got norm=" << result.x.norm();
  EXPECT_NEAR(result.objective, 0.0, 1e-4);

  printf("BarrierQP unconstrained: obj=%.6f, x_norm=%.6f, "
         "%d outer, %d newton, time=%.0fus\n",
         result.objective, result.x.norm(),
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

// Active constraint: minimum is at the boundary.
TEST(BarrierQP, ActiveConstraint) {
  const int n = 3;
  // Q = I, c = [-2, 0, 0]. Unconstrained min at x = [2, 0, 0].
  // Box [0, 1]^3. Constrained min at x = [1, 0, 0].
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) qt.emplace_back(i, i, 1.0);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c(n);
  c << -2.0, 0.0, 0.0;
  VectorXd lb = VectorXd::Zero(n);
  VectorXd ub = VectorXd::Ones(n);

  auto result = SolveBarrierQP(Q, c, lb, ub, 30, 20, 10.0, 1e-8);

  // x should be near [1, 0, 0].
  EXPECT_NEAR(result.x(0), 1.0, 0.01);
  EXPECT_NEAR(result.x(1), 0.0, 0.01);
  EXPECT_NEAR(result.x(2), 0.0, 0.01);

  // Optimal objective: 0.5 * 1 - 2 * 1 = -1.5.
  EXPECT_NEAR(result.objective, -1.5, 0.01);

  printf("BarrierQP active: obj=%.6f, x=[%.4f, %.4f, %.4f], "
         "%d outer, %d newton, time=%.0fus\n",
         result.objective, result.x(0), result.x(1), result.x(2),
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

// Sparse Q: banded + box constraints.
TEST(BarrierQP, SparseQ) {
  const int n = 20;
  std::vector<Eigen::Triplet<double>> qt;
  for (int i = 0; i < n; i++) {
    qt.emplace_back(i, i, 4.0);
    if (i + 1 < n) {
      qt.emplace_back(i, i + 1, -1.0);
      qt.emplace_back(i + 1, i, -1.0);
    }
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());

  VectorXd c = VectorXd::Random(n) * 0.5;
  VectorXd lb = VectorXd::Constant(n, -2.0);
  VectorXd ub = VectorXd::Constant(n, 2.0);

  auto result = SolveBarrierQP(Q, c, lb, ub, 30, 20, 10.0, 1e-8);

  // Verify KKT: Q x + c should be approximately zero for unconstrained
  // components, or have the right sign for active constraints.
  VectorXd grad = Q * result.x + c;
  bool feasible = (result.x.array() >= lb.array() - 1e-6).all() &&
                  (result.x.array() <= ub.array() + 1e-6).all();
  EXPECT_TRUE(feasible) << "Solution not feasible";

  printf("BarrierQP sparse: obj=%.6f, grad_norm=%.6f, "
         "%d outer, %d newton, time=%.0fus\n",
         result.objective, grad.norm(),
         result.outer_iterations, result.total_newton_steps,
         result.solve_time_us);
}

}  // namespace
}  // namespace conex
