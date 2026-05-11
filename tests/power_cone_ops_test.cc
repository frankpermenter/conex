// Test PowerConeOps against autodiff reference.
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "derivatives.h"
#include "barrier_functions.h"
#include "conex/common/power_cone_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/solve_strategies.h"

using namespace Eigen;
using conex::EuclideanJordanAlgebra::PowerConeOps;

class PowerConeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    alpha.resize(2);
    alpha << 0.4, 0.6;
    barriers::g_power_alpha = &alpha;
  }
  VectorXd alpha;
};

TEST_F(PowerConeTest, Gradient) {
  PowerConeOps ops(alpha);
  VectorXd z(4);
  z << 2.0, 1.5, 0.3, 0.2;

  double grad_hand[4];
  ops.computeGradient(grad_hand, z.data(), 4);

  auto grad_ad = derivatives::gradient(barriers::power_cone<derivatives::AD1>, z);

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(grad_hand[i], grad_ad(i), 1e-10) << "component " << i;
}

TEST_F(PowerConeTest, HessianProduct) {
  PowerConeOps ops(alpha);
  VectorXd z(4), v(4);
  z << 2.0, 1.5, 0.3, 0.2;
  v << 0.1, -0.2, 0.3, -0.1;

  double Hv_hand[4];
  ops.hessianProduct(Hv_hand, z.data(), v.data(), 4);

  auto H_ad = derivatives::hessian(barriers::power_cone<derivatives::AD2>, z);
  VectorXd Hv_ad = H_ad * v;

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hv_hand[i], Hv_ad(i), 1e-8) << "component " << i;
}

TEST_F(PowerConeTest, HessianSymmetric) {
  PowerConeOps ops(alpha);
  VectorXd z(4);
  z << 2.0, 1.5, 0.3, 0.2;

  // Build full Hessian from hessianProduct.
  MatrixXd H(4, 4);
  for (int j = 0; j < 4; ++j) {
    VectorXd ej = VectorXd::Unit(4, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 4);
  }

  EXPECT_NEAR((H - H.transpose()).norm(), 0.0, 1e-10) << "Hessian not symmetric";
}

TEST_F(PowerConeTest, LogHomogeneity) {
  PowerConeOps ops(alpha);
  VectorXd z(4);
  z << 2.0, 1.5, 0.3, 0.2;

  double grad[4];
  ops.computeGradient(grad, z.data(), 4);

  // H(z)*z should equal -grad(z).
  double Hz[4];
  ops.hessianProduct(Hz, z.data(), z.data(), 4);

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hz[i], -grad[i], 1e-10) << "H(z)*z != -grad(z), component " << i;

  // <z, grad(z)> should equal -nu.
  double dot = 0;
  for (int i = 0; i < 4; ++i) dot += z[i] * grad[i];
  EXPECT_NEAR(dot, -(alpha.size() + 2.0), 1e-10);
}

TEST_F(PowerConeTest, HessianPD) {
  PowerConeOps ops(alpha);
  VectorXd z(4);
  z << 2.0, 1.5, 0.3, 0.2;

  MatrixXd H(4, 4);
  for (int j = 0; j < 4; ++j) {
    VectorXd ej = VectorXd::Unit(4, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 4);
  }

  SelfAdjointEigenSolver<MatrixXd> eig(H);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0) << "Hessian not positive definite";
}

TEST_F(PowerConeTest, InteriorPoint) {
  PowerConeOps ops(alpha);
  double z[4];
  ops.getInteriorPoint(z, 4);

  // Should be interior: phi > ||w||^2.
  double log_prod = 0;
  for (int i = 0; i < 2; ++i) log_prod += alpha(i) * std::log(z[i]);
  double phi = std::exp(2 * log_prod);
  double w_sq = z[2]*z[2] + z[3]*z[3];
  EXPECT_GT(phi - w_sq, 0);
}

TEST_F(PowerConeTest, DefaultHessianNormSquared) {
  // Test that the base-class default hessianNormSquared agrees with
  // the autodiff Hessian.
  PowerConeOps ops(alpha);
  VectorXd z(4), target(4);
  z << 2.0, 1.5, 0.3, 0.2;
  target << 2.1, 1.4, 0.25, 0.15;

  double d_sq = ops.hessianNormSquared(z.data(), target.data(), 4);

  auto H = derivatives::hessian(barriers::power_cone<derivatives::AD2>, z);
  VectorXd d = target - z;
  double d_sq_ad = d.dot(H * d);

  EXPECT_NEAR(d_sq, d_sq_ad, 1e-8);
}

TEST_F(PowerConeTest, DefaultLineSearch) {
  PowerConeOps ops(alpha);
  VectorXd z(4), t0(4), t1(4);
  z << 2.0, 1.5, 0.3, 0.2;
  t0 << 2.1, 1.4, 0.25, 0.15;
  t1 << 0.1, 0.2, -0.05, 0.03;

  double k = ops.lineSearchTarget(z.data(), t0.data(), t1.data(), 4);

  // Verify: ||t0 + k*t1 - z||^2_H should be approximately 1.
  VectorXd target_k = t0 + k * t1;
  double d_sq = ops.hessianNormSquared(z.data(), target_k.data(), 4);
  EXPECT_NEAR(d_sq, 1.0, 1e-6);
}

TEST_F(PowerConeTest, SolverIntegration_BarrierLP) {
  // Small problem: min c^T x s.t. power_cone(A*x + b).
  // Use m=2 alpha=(0.4,0.6), dim=4 (u1,u2,w1,w2), n=3 variables.
  srand(42);
  const int n = 3, cone_dim = 4;
  PowerConeOps ops(alpha);

  MatrixXd A_dense = MatrixXd::Random(cone_dim, n) * 0.3;
  VectorXd b(cone_dim);
  ops.getInteriorPoint(b.data(), cone_dim);  // b is interior

  // Cost: c = -A^T grad(b) so x=0 is on central path.
  double grad_b[4];
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
  auto result = conex::GeodesicBarrierLP{1e-4, 30, 0, true, z0}.Run(cm);

  printf("\n=== PowerCone BarrierLP ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);

  EXPECT_LT(result.mu, 1e-2) << "mu should decrease";
  EXPECT_GT(result.iterations, 0);
}

TEST_F(PowerConeTest, SolverIntegration_ThetaCont) {
  srand(42);
  const int n = 3, cone_dim = 4;
  PowerConeOps ops(alpha);

  MatrixXd A_dense = MatrixXd::Random(cone_dim, n) * 0.3;
  VectorXd b(cone_dim);
  ops.getInteriorPoint(b.data(), cone_dim);

  double grad_b[4];
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
      1e-4, 20, 0, false, z0}.Run(cm);

  printf("\n=== PowerCone ThetaCont ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);

  EXPECT_LT(result.mu, 1e-2) << "mu should decrease";
}
