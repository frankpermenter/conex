// Test RelEntropyConeOps against autodiff reference.
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "derivatives.h"
#include "barrier_functions.h"
#include "conex/common/rel_entropy_cone_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/solve_strategies.h"

using namespace Eigen;
using conex::EuclideanJordanAlgebra::RelEntropyConeOps;

TEST(RelEntropyConeOps, Gradient) {
  RelEntropyConeOps ops;
  // dim = 5: (u, v1, v2, w1, w2), d=2
  VectorXd z(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;

  double grad_hand[5];
  ops.computeGradient(grad_hand, z.data(), 5);

  auto grad_ad = derivatives::gradient(barriers::rel_entropy<derivatives::AD1>, z);

  for (int i = 0; i < 5; ++i)
    EXPECT_NEAR(grad_hand[i], grad_ad(i), 1e-10) << "component " << i;
}

TEST(RelEntropyConeOps, HessianProduct) {
  RelEntropyConeOps ops;
  VectorXd z(5), v(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;
  v << 0.1, -0.2, 0.15, 0.3, -0.1;

  double Hv_hand[5];
  ops.hessianProduct(Hv_hand, z.data(), v.data(), 5);

  auto H_ad = derivatives::hessian(barriers::rel_entropy<derivatives::AD2>, z);
  VectorXd Hv_ad = H_ad * v;

  for (int i = 0; i < 5; ++i)
    EXPECT_NEAR(Hv_hand[i], Hv_ad(i), 1e-8) << "component " << i;
}

TEST(RelEntropyConeOps, HessianSymmetric) {
  RelEntropyConeOps ops;
  VectorXd z(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;

  MatrixXd H(5, 5);
  for (int j = 0; j < 5; ++j) {
    VectorXd ej = VectorXd::Unit(5, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 5);
  }

  EXPECT_NEAR((H - H.transpose()).norm(), 0.0, 1e-10) << "Hessian not symmetric";
}

TEST(RelEntropyConeOps, LogHomogeneity) {
  RelEntropyConeOps ops;
  VectorXd z(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;

  double grad[5];
  ops.computeGradient(grad, z.data(), 5);

  double Hz[5];
  ops.hessianProduct(Hz, z.data(), z.data(), 5);

  for (int i = 0; i < 5; ++i)
    EXPECT_NEAR(Hz[i], -grad[i], 1e-10) << "H(z)*z != -grad(z), component " << i;

  double d = 0;
  for (int i = 0; i < 5; ++i) d += z[i] * grad[i];
  EXPECT_NEAR(d, -5.0, 1e-10);  // nu = dim = 5
}

TEST(RelEntropyConeOps, HessianPD) {
  RelEntropyConeOps ops;
  VectorXd z(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;

  MatrixXd H(5, 5);
  for (int j = 0; j < 5; ++j) {
    VectorXd ej = VectorXd::Unit(5, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 5);
  }

  SelfAdjointEigenSolver<MatrixXd> eig(H);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0) << "Hessian not positive definite";
}

TEST(RelEntropyConeOps, InteriorPoint) {
  RelEntropyConeOps ops;
  double z[5];
  ops.getInteriorPoint(z, 5);
  // u=1, v=w=1, so s = 1 - 0 = 1 > 0.
  EXPECT_GT(z[0], 0);
  for (int i = 1; i < 5; ++i) EXPECT_GT(z[i], 0);
}

TEST(RelEntropyConeOps, LargerDimension) {
  // d=4, dim=9: (u, v1..v4, w1..w4)
  RelEntropyConeOps ops;
  srand(42);
  VectorXd z(9);
  z(0) = 5.0;  // u large enough
  for (int i = 1; i <= 4; ++i) z(i) = 0.5 + 0.5 * (rand() % 100) / 100.0;  // v
  for (int i = 5; i <= 8; ++i) z(i) = 0.5 + 0.5 * (rand() % 100) / 100.0;  // w

  double grad[9];
  ops.computeGradient(grad, z.data(), 9);
  double Hz[9];
  ops.hessianProduct(Hz, z.data(), z.data(), 9);

  for (int i = 0; i < 9; ++i)
    EXPECT_NEAR(Hz[i], -grad[i], 1e-8) << "log-homogeneity at dim=9, component " << i;

  double d = 0;
  for (int i = 0; i < 9; ++i) d += z[i] * grad[i];
  EXPECT_NEAR(d, -9.0, 1e-8);
}

TEST(RelEntropyConeOps, SolverIntegration_BarrierLP) {
  srand(42);
  const int n = 3, cone_dim = 5;  // d=2
  RelEntropyConeOps ops;

  MatrixXd A_dense = MatrixXd::Random(cone_dim, n) * 0.3;
  VectorXd b(cone_dim);
  ops.getInteriorPoint(b.data(), cone_dim);

  double grad_b[5];
  ops.computeGradient(grad_b, b.data(), cone_dim);
  VectorXd c = -A_dense.transpose() * Map<VectorXd>(grad_b, cone_dim);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd z0(cone_dim);
  ops.getInteriorPoint(z0.data(), cone_dim);

  conex::Model model;
  model.AddBarrierConstraint(A_dense.sparseView(), b, vars, &ops);
  model.SetLinearCost(c);

  auto solver = conex::Solver::Build(model);
  auto cm = solver.MakeCompiledModel();
  auto result = conex::GeodesicBarrierLP{1e-4, 30, 0, false, z0}.Run(cm);

  printf("\n=== RelEntropy BarrierLP ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);

  EXPECT_LT(result.mu, 1e-2);
  EXPECT_GT(result.iterations, 0);
}

TEST(RelEntropyConeOps, SolverIntegration_ThetaCont) {
  srand(42);
  const int n = 3, cone_dim = 5;
  RelEntropyConeOps ops;

  MatrixXd A_dense = MatrixXd::Random(cone_dim, n) * 0.3;
  VectorXd b(cone_dim);
  ops.getInteriorPoint(b.data(), cone_dim);

  double grad_b[5];
  ops.computeGradient(grad_b, b.data(), cone_dim);
  VectorXd c = -A_dense.transpose() * Map<VectorXd>(grad_b, cone_dim);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  VectorXd z0(cone_dim);
  ops.getInteriorPoint(z0.data(), cone_dim);

  conex::Model model;
  model.AddBarrierConstraint(A_dense.sparseView(), b, vars, &ops);
  model.SetLinearCost(c);

  auto solver = conex::Solver::Build(model);
  auto cm = solver.MakeCompiledModel();
  auto result = conex::GeodesicBarrierThetaContinuation{
      1e-4, 30, 0, false, z0}.Run(cm);

  printf("\n=== RelEntropy ThetaCont ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);

  EXPECT_LT(result.mu, 1e-2);
}
