#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "derivatives.h"
#include "barrier_functions.h"
#include "conex/common/hypo_geomean_cone_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/solve_strategies.h"

using namespace Eigen;
using conex::EuclideanJordanAlgebra::HypoGeoMeanConeOps;

TEST(HypoGeoMeanConeOps, Gradient) {
  HypoGeoMeanConeOps ops;
  VectorXd z(4);  // (u, w1, w2, w3), d=3
  z << 0.5, 2.0, 1.5, 1.8;

  double grad_hand[4];
  ops.computeGradient(grad_hand, z.data(), 4);

  auto grad_ad = derivatives::gradient(barriers::hypo_geomean<derivatives::AD1>, z);

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(grad_hand[i], grad_ad(i), 1e-10) << "component " << i;
}

TEST(HypoGeoMeanConeOps, HessianProduct) {
  HypoGeoMeanConeOps ops;
  VectorXd z(4), v(4);
  z << 0.5, 2.0, 1.5, 1.8;
  v << 0.1, -0.2, 0.15, -0.1;

  double Hv_hand[4];
  ops.hessianProduct(Hv_hand, z.data(), v.data(), 4);

  auto H_ad = derivatives::hessian(barriers::hypo_geomean<derivatives::AD2>, z);
  VectorXd Hv_ad = H_ad * v;

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hv_hand[i], Hv_ad(i), 1e-8) << "component " << i;
}

TEST(HypoGeoMeanConeOps, HessianSymmetric) {
  HypoGeoMeanConeOps ops;
  VectorXd z(4);
  z << 0.5, 2.0, 1.5, 1.8;

  MatrixXd H(4, 4);
  for (int j = 0; j < 4; ++j) {
    VectorXd ej = VectorXd::Unit(4, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 4);
  }
  EXPECT_NEAR((H - H.transpose()).norm(), 0.0, 1e-10);
}

TEST(HypoGeoMeanConeOps, LogHomogeneity) {
  HypoGeoMeanConeOps ops;
  VectorXd z(4);
  z << 0.5, 2.0, 1.5, 1.8;

  double grad[4], Hz[4];
  ops.computeGradient(grad, z.data(), 4);
  ops.hessianProduct(Hz, z.data(), z.data(), 4);

  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hz[i], -grad[i], 1e-10) << "component " << i;

  double d = 0;
  for (int i = 0; i < 4; ++i) d += z[i] * grad[i];
  EXPECT_NEAR(d, -4.0, 1e-10);  // nu = dim = 4
}

TEST(HypoGeoMeanConeOps, HessianPD) {
  HypoGeoMeanConeOps ops;
  VectorXd z(4);
  z << 0.5, 2.0, 1.5, 1.8;

  MatrixXd H(4, 4);
  for (int j = 0; j < 4; ++j) {
    VectorXd ej = VectorXd::Unit(4, j);
    ops.hessianProduct(H.col(j).data(), z.data(), ej.data(), 4);
  }
  SelfAdjointEigenSolver<MatrixXd> eig(H);
  EXPECT_GT(eig.eigenvalues().minCoeff(), 0);
}

TEST(HypoGeoMeanConeOps, LargerDimension) {
  HypoGeoMeanConeOps ops;
  VectorXd z(6);  // d=5
  z << 0.3, 1.2, 1.5, 0.8, 2.0, 1.1;

  double grad[6], Hz[6];
  ops.computeGradient(grad, z.data(), 6);
  ops.hessianProduct(Hz, z.data(), z.data(), 6);

  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(Hz[i], -grad[i], 1e-8) << "component " << i;

  double d = 0;
  for (int i = 0; i < 6; ++i) d += z[i] * grad[i];
  EXPECT_NEAR(d, -6.0, 1e-8);
}

TEST(HypoGeoMeanConeOps, SolverIntegration_BarrierLP) {
  srand(42);
  const int n = 3, cone_dim = 4;
  HypoGeoMeanConeOps ops;

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
  auto result = conex::GeodesicBarrierLP{1e-4, 30, 0, false, z0}.Run(cm);

  printf("\n=== HypoGeoMean BarrierLP ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);
  EXPECT_LT(result.mu, 1e-2);
}

TEST(HypoGeoMeanConeOps, SolverIntegration_ThetaCont) {
  srand(42);
  const int n = 3, cone_dim = 4;
  HypoGeoMeanConeOps ops;

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
      1e-4, 30, 0, false, z0}.Run(cm);

  printf("\n=== HypoGeoMean ThetaCont ===\n");
  printf("  iters=%d, fac=%d, gap=%.2e, mu=%.2e\n",
         result.iterations, result.total_factorizations,
         result.complementarity, result.mu);
  EXPECT_LT(result.mu, 1e-2);
}
