// Verify the autodiff barrier oracle against hand-coded exp cone ops.
#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Dense>
#include "autodiff_barrier.h"
#include "conex/common/exp_cone_ops.h"

using namespace conex::testing;
using conex::EuclideanJordanAlgebra::ExpConeOps;

TEST(AutodiffBarrier, ExpCone_Gradient) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  // Autodiff gradient.
  Eigen::VectorXd ad_grad = exp_cone_gradient(z);

  // Hand-coded gradient.
  double g[3];
  ExpConeOps::BarrierGrad(z(0), z(1), z(2), g);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(ad_grad(i), g[i], 1e-8)
        << "gradient component " << i;
  }
}

TEST(AutodiffBarrier, ExpCone_Hessian) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  // Autodiff Hessian.
  Eigen::MatrixXd ad_H = exp_cone_hessian(z);

  // Hand-coded Hessian.
  double H[9];
  ExpConeOps::BarrierHessian(z(0), z(1), z(2), H);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(ad_H(i, j), H[3*i + j], 1e-7)
          << "Hessian[" << i << "," << j << "]";
    }
}

TEST(AutodiffBarrier, ExpCone_ThirdDeriv) {
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;
  Eigen::VectorXd v(3);
  v << 0.5, -0.3, 0.2;

  // Autodiff third derivative.
  Eigen::VectorXd ad_T = exp_cone_third_deriv(z, v);

  // Hand-coded third derivative.
  double T[3];
  ExpConeOps::ThirdDerivContract(z(0), z(1), z(2), v.data(), T);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(ad_T(i), T[i], 1e-10)
        << "ThirdDeriv[" << i << "]";
  }
}

TEST(AutodiffBarrier, ExpCone_LogHomogeneity) {
  // H(z)*z = -grad(z) for log-homogeneous barriers.
  Eigen::VectorXd z(3);
  z << 0.3, 1.2, 3.0;

  Eigen::VectorXd grad = exp_cone_gradient(z);
  Eigen::MatrixXd H = exp_cone_hessian(z);
  Eigen::VectorXd Hz = H * z;

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(Hz(i), -grad(i), 1e-8)
        << "log-homogeneity H(z)*z = -grad(z) component " << i;
  }

  // <z, grad(z)> = -nu
  double nu = 2.0;  // exp cone barrier parameter
  EXPECT_NEAR(z.dot(grad), -nu, 1e-8);
}

TEST(AutodiffBarrier, PowerCone_Gradient) {
  Eigen::VectorXd alpha(2);
  alpha << 0.3, 0.7;

  // z = (u1, u2, w1) with u1^0.3 * u2^0.7 > |w1|
  Eigen::VectorXd z(3);
  z << 2.0, 1.5, 0.5;

  Eigen::VectorXd ad_grad = power_cone_gradient(z, alpha);

  // Verify via barrier value finite diff.
  double h = 1e-7;
  for (int i = 0; i < 3; ++i) {
    Eigen::VectorXd zp = z, zm = z;
    zp(i) += h;
    zm(i) -= h;
    double fd = (power_cone_barrier(zp, alpha) - power_cone_barrier(zm, alpha))
                / (2.0 * h);
    EXPECT_NEAR(ad_grad(i), fd, 1e-5)
        << "power cone gradient component " << i;
  }
}

TEST(AutodiffBarrier, PowerCone_LogHomogeneity) {
  Eigen::VectorXd alpha(2);
  alpha << 0.4, 0.6;
  double nu = alpha.size() + 2;  // m + 2 (from -log(prod^2 - ||w||^2) - sum log(u_i))

  Eigen::VectorXd z(4);  // (u1, u2, w1, w2)
  z << 2.0, 1.5, 0.3, 0.2;

  Eigen::VectorXd grad = power_cone_gradient(z, alpha);
  Eigen::MatrixXd H = power_cone_hessian(z, alpha);
  Eigen::VectorXd Hz = H * z;

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(Hz(i), -grad(i), 1e-6)
        << "power cone log-homogeneity component " << i;
  }

  EXPECT_NEAR(z.dot(grad), -nu, 1e-6);
}

TEST(AutodiffBarrier, RelEntropy_LogHomogeneity) {
  // dim = 1 + 2*d, use d=2 -> dim=5
  Eigen::VectorXd z(5);
  // z = (u, v1, v2, w1, w2) with u > w1*log(w1/v1) + w2*log(w2/v2)
  z << 3.0, 1.0, 2.0, 0.5, 0.8;

  double nu = 5.0;  // dim for relative entropy

  Eigen::VectorXd grad = rel_entropy_gradient(z);
  Eigen::MatrixXd H = rel_entropy_hessian(z);
  Eigen::VectorXd Hz = H * z;

  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(Hz(i), -grad(i), 1e-6)
        << "rel entropy log-homogeneity component " << i;
  }

  EXPECT_NEAR(z.dot(grad), -nu, 1e-6);
}

TEST(AutodiffBarrier, HypoGeoMean_LogHomogeneity) {
  // dim = 1 + d, use d=3 -> dim=4
  Eigen::VectorXd z(4);
  // z = (u, w1, w2, w3) with u < geomean(w)
  z << 0.5, 2.0, 1.5, 1.8;

  double nu = 4.0;  // dim for hypo geomean

  Eigen::VectorXd grad = hypo_geomean_gradient(z);
  Eigen::MatrixXd H = hypo_geomean_hessian(z);
  Eigen::VectorXd Hz = H * z;

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(Hz(i), -grad(i), 1e-6)
        << "hypo geomean log-homogeneity component " << i;
  }

  EXPECT_NEAR(z.dot(grad), -nu, 1e-6);
}
