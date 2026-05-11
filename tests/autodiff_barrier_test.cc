// Tests: validate hand-coded cone derivatives against autodiff.
//
// This file instantiates barrier templates with AD types (via function
// pointers) and calls the pre-compiled derivatives.cc machinery.
// Adding a new barrier = add a test function here. No recompilation
// of derivatives.cc needed.

#include <gtest/gtest.h>
#include "derivatives.h"
#include "barrier_functions.h"
#include "conex/common/exp_cone_ops.h"

using namespace derivatives;
using conex::EuclideanJordanAlgebra::ExpConeOps;

// ---- Exp cone: validate hand-coded ops against autodiff ----

TEST(Derivatives, ExpCone_Gradient) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  auto ad_grad = gradient(barriers::exp_cone<AD1>, z);

  double g[3];
  ExpConeOps::BarrierGrad(z(0), z(1), z(2), g);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(ad_grad(i), g[i], 1e-10) << "component " << i;
}

TEST(Derivatives, ExpCone_Hessian) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  auto ad_H = hessian(barriers::exp_cone<AD2>, z);

  double H[9];
  ExpConeOps::BarrierHessian(z(0), z(1), z(2), H);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(ad_H(i, j), H[3*i + j], 1e-10)
          << "H[" << i << "," << j << "]";
}

TEST(Derivatives, ExpCone_ThirdDeriv) {
  Eigen::VectorXd z(3), v(3);
  z << 0.3, 1.2, 3.0;
  v << 0.5, -0.3, 0.2;

  auto ad_T = third_deriv_contract(barriers::exp_cone<AD3>, z, v);

  double T[3];
  ExpConeOps::ThirdDerivContract(z(0), z(1), z(2), v.data(), T);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(ad_T(i), T[i], 1e-10) << "T[" << i << "]";
}

// ---- Log-homogeneity: H(z)*z = -grad(z), <z,grad> = -nu ----

TEST(Derivatives, ExpCone_LogHomogeneity) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  auto grad = gradient(barriers::exp_cone<AD1>, z);
  auto H = hessian(barriers::exp_cone<AD2>, z);

  Eigen::VectorXd Hz = H * z;
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(Hz(i), -grad(i), 1e-10);
  EXPECT_NEAR(z.dot(grad), -2.0, 1e-10);  // nu = 2
}

TEST(Derivatives, PowerCone_LogHomogeneity) {
  Eigen::VectorXd alpha(2);
  alpha << 0.4, 0.6;
  barriers::g_power_alpha = &alpha;
  double nu = alpha.size() + 2;

  Eigen::VectorXd z(4);
  z << 2.0, 1.5, 0.3, 0.2;

  auto grad = gradient(barriers::power_cone<AD1>, z);
  auto H = hessian(barriers::power_cone<AD2>, z);

  Eigen::VectorXd Hz = H * z;
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hz(i), -grad(i), 1e-8) << "component " << i;
  EXPECT_NEAR(z.dot(grad), -nu, 1e-8);
}

TEST(Derivatives, RelEntropy_LogHomogeneity) {
  Eigen::VectorXd z(5);
  z << 3.0, 1.0, 2.0, 0.5, 0.8;
  double nu = 5.0;

  auto grad = gradient(barriers::rel_entropy<AD1>, z);
  auto H = hessian(barriers::rel_entropy<AD2>, z);

  Eigen::VectorXd Hz = H * z;
  for (int i = 0; i < 5; ++i)
    EXPECT_NEAR(Hz(i), -grad(i), 1e-8) << "component " << i;
  EXPECT_NEAR(z.dot(grad), -nu, 1e-8);
}

TEST(Derivatives, HypoGeoMean_LogHomogeneity) {
  Eigen::VectorXd z(4);
  z << 0.5, 2.0, 1.5, 1.8;
  double nu = 4.0;

  auto grad = gradient(barriers::hypo_geomean<AD1>, z);
  auto H = hessian(barriers::hypo_geomean<AD2>, z);

  Eigen::VectorXd Hz = H * z;
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(Hz(i), -grad(i), 1e-8) << "component " << i;
  EXPECT_NEAR(z.dot(grad), -nu, 1e-8);
}
