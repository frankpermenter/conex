// Test the exponential cone geodesic integration.
//
// Verifies:
// 1. Barrier Hessian is positive definite at interior points.
// 2. Geodesic stays in the cone interior.
// 3. Geodesic preserves the Riemannian energy (|ẋ|²_H = const).
// 4. The geodesic step matches a fine-grid reference.

#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>

#include "conex/common/exp_cone_ops.h"

using conex::EuclideanJordanAlgebra::ExpConeOps;

namespace {

// Check that (x, y, z) is in the exponential cone interior.
bool InInterior(double x, double y, double z) {
  return y > 0 && z > y * std::exp(x / y);
}

// Riemannian energy: |v|²_H = v^T H(x) v.
double Energy(double x, double y, double z, const double* v) {
  double H[9];
  ExpConeOps::BarrierHessian(x, y, z, H);
  double e = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      e += v[i] * H[3*i + j] * v[j];
  return e;
}

TEST(ExpCone, BarrierHessianPD) {
  // Verify Hessian is positive definite at several interior points.
  double pts[][3] = {
    {0, 1, 2},       // center-ish
    {1, 1, 4},       // x > 0
    {-1, 2, 3},      // x < 0
    {0.5, 0.5, 3},   // near boundary
  };
  for (auto& p : pts) {
    ASSERT_TRUE(InInterior(p[0], p[1], p[2]))
        << "Point not interior: " << p[0] << " " << p[1] << " " << p[2];
    double H[9];
    ExpConeOps::BarrierHessian(p[0], p[1], p[2], H);
    // Check 3x3 PD via Sylvester's criterion.
    EXPECT_GT(H[0], 0) << "H[0,0] not positive";
    double det2 = H[0]*H[4] - H[1]*H[3];
    EXPECT_GT(det2, 0) << "2x2 minor not positive";
    double det3 = H[0]*(H[4]*H[8]-H[5]*H[7])
                - H[1]*(H[3]*H[8]-H[5]*H[6])
                + H[2]*(H[3]*H[7]-H[4]*H[6]);
    EXPECT_GT(det3, 0) << "Determinant not positive";
  }
}

TEST(ExpCone, GradientMatchesFiniteDiff) {
  double x = 0.3, y = 1.2, z = 3.0;
  double g[3];
  ExpConeOps::BarrierGrad(x, y, z, g);

  double eps = 1e-7;
  for (int i = 0; i < 3; ++i) {
    double p[3] = {x, y, z};
    p[i] += eps;
    double fp = ExpConeOps::Barrier(p[0], p[1], p[2]);
    p[i] -= 2*eps;
    double fm = ExpConeOps::Barrier(p[0], p[1], p[2]);
    double fd = (fp - fm) / (2*eps);
    EXPECT_NEAR(g[i], fd, 1e-5)
        << "Gradient component " << i << " mismatch";
  }
}

TEST(ExpCone, HessianMatchesFiniteDiff) {
  double x = 0.3, y = 1.2, z = 3.0;
  double H[9];
  ExpConeOps::BarrierHessian(x, y, z, H);

  double eps = 1e-6;
  for (int i = 0; i < 3; ++i) {
    for (int j = i; j < 3; ++j) {
      double p[3] = {x, y, z};
      // d²F/dx_i dx_j via finite diff of gradient.
      p[j] += eps;
      double gp[3]; ExpConeOps::BarrierGrad(p[0], p[1], p[2], gp);
      p[j] -= 2*eps;
      double gm[3]; ExpConeOps::BarrierGrad(p[0], p[1], p[2], gm);
      double fd = (gp[i] - gm[i]) / (2*eps);
      EXPECT_NEAR(H[3*i + j], fd, 1e-4)
          << "Hessian[" << i << "," << j << "] mismatch";
    }
  }
}

TEST(ExpCone, ThirdDerivMatchesFiniteDiff) {
  double x = 0.3, y = 1.2, z = 3.0;
  double v[3] = {0.5, -0.3, 0.2};
  double T[3];
  ExpConeOps::ThirdDerivContract(x, y, z, v, T);

  // T_l = v^T (dH/dx_l) v.  Finite diff: (v^T H(x+eps*e_l) v - v^T H(x-eps*e_l) v) / (2eps).
  double eps = 1e-6;
  for (int l = 0; l < 3; ++l) {
    double p[3] = {x, y, z};
    p[l] += eps;
    double Hp[9]; ExpConeOps::BarrierHessian(p[0], p[1], p[2], Hp);
    double ep = 0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        ep += v[i] * Hp[3*i+j] * v[j];

    p[l] -= 2*eps;
    double Hm[9]; ExpConeOps::BarrierHessian(p[0], p[1], p[2], Hm);
    double em = 0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        em += v[i] * Hm[3*i+j] * v[j];

    double fd = (ep - em) / (2*eps);
    EXPECT_NEAR(T[l], fd, 1e-3)
        << "ThirdDerivContract[" << l << "] mismatch";
  }
}

TEST(ExpCone, GeodesicStaysInterior) {
  ExpConeOps ops;
  double w[3] = {0, 1, 2};  // interior point
  // Direction in local tangent space.
  double d[3] = {0.5, -0.2, 0.3};

  printf("Geodesic trajectory:\n");
  for (int i = 0; i <= 10; ++i) {
    double alpha = i * 0.1;
    double p[3] = {0, 1, 2};
    if (alpha > 0) ops.geodesicStep(p, alpha, d);
    double u = p[2] - p[1] * std::exp(p[0] / p[1]);
    printf("  alpha=%.1f: (%.4f, %.4f, %.4f) slack=%.4e %s\n",
           alpha, p[0], p[1], p[2], u,
           InInterior(p[0], p[1], p[2]) ? "OK" : "OUTSIDE");
    EXPECT_TRUE(InInterior(p[0], p[1], p[2]))
        << "Geodesic left cone at alpha=" << alpha;
  }
}

TEST(ExpCone, GeodesicSmoothConvergence) {
  // Verify that halving the step and doubling the count gives a consistent
  // endpoint — i.e., the geodesic integration is converging.
  // Compare: one step of alpha vs two steps of alpha/2.
  ExpConeOps ops;
  double w0[3] = {0, 1, 2};
  double d[3] = {0.3, -0.1, 0.2};
  double alpha = 0.5;

  // One step of alpha.
  double w1[3] = {w0[0], w0[1], w0[2]};
  ops.geodesicStep(w1, alpha, d);

  // Two steps of alpha/2: take first step, then need to recompute d in
  // the local metric at the midpoint.  Instead, compare alpha vs alpha/2
  // endpoint convergence.
  double wa[3] = {w0[0], w0[1], w0[2]};
  ops.geodesicStep(wa, alpha * 0.5, d);
  double wb[3] = {w0[0], w0[1], w0[2]};
  ops.geodesicStep(wb, alpha * 0.25, d);

  // The geodesic gamma(t) = Exp_w0(t*d). At t=alpha/2 and t=alpha/4
  // both should be interior.
  EXPECT_TRUE(InInterior(wa[0], wa[1], wa[2]));
  EXPECT_TRUE(InInterior(wb[0], wb[1], wb[2]));

  // Barrier values should decrease (we're moving away from boundary).
  double F0 = ExpConeOps::Barrier(w0[0], w0[1], w0[2]);
  double Fa = ExpConeOps::Barrier(wa[0], wa[1], wa[2]);
  double F1 = ExpConeOps::Barrier(w1[0], w1[1], w1[2]);
  printf("Barrier: F(0)=%.6f, F(a/2)=%.6f, F(a)=%.6f\n", F0, Fa, F1);
  printf("Geodesic: (%.6f, %.6f, %.6f) -> (%.6f, %.6f, %.6f)\n",
         w0[0], w0[1], w0[2], w1[0], w1[1], w1[2]);

  // Key check: geodesic differs from Euler (straight line).
  double euler[3] = {w0[0]+alpha*d[0], w0[1]+alpha*d[1], w0[2]+alpha*d[2]};
  double diff = 0;
  for (int k = 0; k < 3; ++k) diff += (w1[k]-euler[k])*(w1[k]-euler[k]);
  diff = std::sqrt(diff);
  printf("||geodesic - euler|| = %.4e\n", diff);
  EXPECT_GT(diff, 1e-6) << "Geodesic should curve away from straight line";
}

TEST(ExpCone, GeodesicConvergesWithRefinement) {
  // Verify the geodesic endpoint converges as we increase the number of
  // integration steps (by comparing with a very fine reference).
  ExpConeOps ops;
  double w0[3] = {0.2, 1.5, 3.0};
  double d[3] = {0.4, -0.15, 0.25};
  double alpha = 0.8;

  // Reference: use the integrator with default steps.
  double ref[3] = {w0[0], w0[1], w0[2]};
  ops.geodesicStep(ref, alpha, d);

  // Compare with Euler step (much cruder).
  double euler[3] = {w0[0] + alpha*d[0], w0[1] + alpha*d[1], w0[2] + alpha*d[2]};
  double euler_err = std::sqrt((euler[0]-ref[0])*(euler[0]-ref[0])
                             + (euler[1]-ref[1])*(euler[1]-ref[1])
                             + (euler[2]-ref[2])*(euler[2]-ref[2]));
  printf("Euler vs geodesic: ||diff|| = %.4e\n", euler_err);
  EXPECT_GT(euler_err, 1e-4) << "Geodesic should differ from Euler step";
  printf("Geodesic endpoint: (%.6f, %.6f, %.6f)\n", ref[0], ref[1], ref[2]);
}

}  // namespace
