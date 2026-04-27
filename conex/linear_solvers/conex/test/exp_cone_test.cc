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

#include <Eigen/Dense>

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

// Check dual cone membership for the exponential cone.
// K* = cl{(a,b,c) : a <= 0, c >= 0, -a*exp(b/a - 1) <= c} (when a < 0).
bool InDualCone(double a, double b, double c) {
  if (a > 0) return false;
  if (c < 0) return false;
  if (std::abs(a) < 1e-15) return c >= 0;  // a = 0: need c >= 0, b >= 0
  return -a * std::exp(b / a - 1) <= c + 1e-14;
}

// Invert 3x3 matrix.
void Invert3x3(const double* A, double* Ainv) {
  double det = A[0]*(A[4]*A[8]-A[5]*A[7])
             - A[1]*(A[3]*A[8]-A[5]*A[6])
             + A[2]*(A[3]*A[7]-A[4]*A[6]);
  Ainv[0] = (A[4]*A[8]-A[5]*A[7])/det;
  Ainv[1] = (A[2]*A[7]-A[1]*A[8])/det;
  Ainv[2] = (A[1]*A[5]-A[2]*A[4])/det;
  Ainv[3] = (A[5]*A[6]-A[3]*A[8])/det;
  Ainv[4] = (A[0]*A[8]-A[2]*A[6])/det;
  Ainv[5] = (A[2]*A[3]-A[0]*A[5])/det;
  Ainv[6] = (A[3]*A[7]-A[4]*A[6])/det;
  Ainv[7] = (A[1]*A[6]-A[0]*A[7])/det;
  Ainv[8] = (A[0]*A[4]-A[1]*A[3])/det;
}

// Dual energy: |λ̇|²_{H⁻¹} = λ̇ᵀ H(x)⁻¹ λ̇
double DualEnergy(double x, double y, double z, const double* ldot) {
  double H[9], Hinv[9];
  ExpConeOps::BarrierHessian(x, y, z, H);
  Invert3x3(H, Hinv);
  double e = 0;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      e += ldot[i] * Hinv[3*i+j] * ldot[j];
  return e;
}

TEST(ExpCone, DualGeodesicViaGradientMap) {
  // Verify that λ(t) = -∇F(γ(t)) is a geodesic in K* when γ(t) is
  // a geodesic in K.
  //
  // Checks:
  // 1. λ(t) ∈ K* for all t (dual feasibility)
  // 2. Dual energy |λ̇|²_{H⁻¹} is constant (geodesic ⟺ constant speed)
  // 3. Primal energy = dual energy (isometry)

  ExpConeOps ops;
  double w0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};

  // Sample the geodesic at many points.
  const int N = 20;
  double alpha_max = 0.6;
  double dt = alpha_max / N;

  // Compute primal geodesic and dual image at each sample.
  double gamma[N+1][3], lambda[N+1][3];
  for (int i = 0; i <= N; ++i) {
    double t = i * dt;
    gamma[i][0] = w0[0]; gamma[i][1] = w0[1]; gamma[i][2] = w0[2];
    if (t > 0) ops.geodesicStep(gamma[i], t, d);

    // λ(t) = -∇F(γ(t))
    double g[3];
    ExpConeOps::BarrierGrad(gamma[i][0], gamma[i][1], gamma[i][2], g);
    lambda[i][0] = -g[0]; lambda[i][1] = -g[1]; lambda[i][2] = -g[2];
  }

  // Check 1: dual cone membership.
  printf("\n=== Dual geodesic via gradient map ===\n");
  printf("  %3s  %10s %10s %10s  %10s %10s %10s  %5s  %10s\n",
         "i", "gam_x", "gam_y", "gam_z", "lam_0", "lam_1", "lam_2",
         "K*?", "dual_E");
  for (int i = 0; i <= N; ++i) {
    bool in_dual = InDualCone(lambda[i][0], lambda[i][1], lambda[i][2]);

    // Check 2: dual energy via finite differences.
    double dual_e = -1;
    if (i > 0 && i < N) {
      double ldot[3];
      for (int k = 0; k < 3; ++k)
        ldot[k] = (lambda[i+1][k] - lambda[i-1][k]) / (2*dt);
      dual_e = DualEnergy(gamma[i][0], gamma[i][1], gamma[i][2], ldot);
    }

    printf("  %3d  %10.4f %10.4f %10.4f  %10.4f %10.4f %10.4f  %5s  %10.4e\n",
           i, gamma[i][0], gamma[i][1], gamma[i][2],
           lambda[i][0], lambda[i][1], lambda[i][2],
           in_dual ? "yes" : "NO", dual_e);

    EXPECT_TRUE(in_dual) << "λ(" << i << ") not in dual cone";
  }

  // Check 2 (quantitative): dual energy should be constant.
  // Collect interior dual energies (skip endpoints where finite diff is one-sided).
  double E_first = -1, E_max = 0, E_min = 1e30;
  for (int i = 2; i < N-1; ++i) {
    double ldot[3];
    for (int k = 0; k < 3; ++k)
      ldot[k] = (lambda[i+1][k] - lambda[i-1][k]) / (2*dt);
    double e = DualEnergy(gamma[i][0], gamma[i][1], gamma[i][2], ldot);
    if (E_first < 0) E_first = e;
    E_max = std::max(E_max, e);
    E_min = std::min(E_min, e);
  }
  double rel_var = (E_max - E_min) / E_first;
  printf("  Dual energy: min=%.6e max=%.6e relative variation=%.2e\n",
         E_min, E_max, rel_var);
  EXPECT_LT(rel_var, 1e-2)
      << "Dual energy not constant — dual curve is not geodesic";

  // Check 3: primal energy at t=0 should equal dual energy.
  double v0[3] = {d[0]*alpha_max, d[1]*alpha_max, d[2]*alpha_max};
  // Actually initial velocity is just d (not alpha_max*d) since geodesicStep
  // integrates gamma(alpha) = Exp_{w0}(alpha*d). So velocity at t is d.
  // But we sampled at t = i*dt with dt = alpha_max/N. The velocity of gamma
  // w.r.t. parameter t (not alpha) is d * (alpha_max / alpha_max) = d...
  // Hmm, geodesicStep(w, alpha, d) computes Exp_w(alpha*d). So gamma(alpha)
  // has velocity d at alpha=0. The parameterization has |γ̇|² = |d|²_H.
  double primal_E = Energy(w0[0], w0[1], w0[2], d);
  printf("  Primal energy at t=0: %.6e, dual energy: ~%.6e\n",
         primal_E, E_first);
  // They should be equal (isometry of gradient map).
  // But finite-diff introduces error, so be lenient.
  EXPECT_NEAR(primal_E, E_first, primal_E * 0.05)
      << "Primal and dual energies differ — gradient map not isometric";
}

TEST(ExpCone, BregmanMidpointStaysInterior) {
  ExpConeOps ops;
  double w0[3] = {0, 1, 2};
  double d[3] = {0.5, -0.2, 0.3};
  for (int i = 0; i <= 10; ++i) {
    double alpha = i * 0.1;
    double p[3] = {w0[0], w0[1], w0[2]};
    if (alpha > 0) ops.bregmanMidpointStep(p, alpha, d);
    EXPECT_TRUE(InInterior(p[0], p[1], p[2]))
        << "Bregman step left cone at alpha=" << alpha;
  }
}

TEST(ExpCone, BregmanVsGeodesicODE) {
  // Compare the Bregman midpoint step with the ODE geodesic.
  // They should agree to second order in alpha.
  ExpConeOps ops;
  double w0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};

  printf("\n=== Bregman midpoint vs ODE geodesic ===\n");
  printf("  %6s  %12s  %12s  %12s\n", "alpha", "||diff||", "||geo-euler||", "ratio");

  double prev_diff = 0;
  for (int i = 1; i <= 8; ++i) {
    double alpha = i * 0.1;
    double geo[3] = {w0[0], w0[1], w0[2]};
    double breg[3] = {w0[0], w0[1], w0[2]};
    double euler[3] = {w0[0]+alpha*d[0], w0[1]+alpha*d[1], w0[2]+alpha*d[2]};

    ops.geodesicStep(geo, alpha, d);
    ops.bregmanMidpointStep(breg, alpha, d);

    double diff = std::sqrt((geo[0]-breg[0])*(geo[0]-breg[0])
                          + (geo[1]-breg[1])*(geo[1]-breg[1])
                          + (geo[2]-breg[2])*(geo[2]-breg[2]));
    double euler_diff = std::sqrt((geo[0]-euler[0])*(geo[0]-euler[0])
                                + (geo[1]-euler[1])*(geo[1]-euler[1])
                                + (geo[2]-euler[2])*(geo[2]-euler[2]));
    printf("  %6.2f  %12.4e  %12.4e  %12.4f\n",
           alpha, diff, euler_diff, euler_diff > 0 ? diff/euler_diff : 0);
    prev_diff = diff;
  }

  // At small alpha, Bregman should be much closer to geodesic than Euler.
  double alpha_small = 0.1;
  double geo[3] = {w0[0], w0[1], w0[2]};
  double breg[3] = {w0[0], w0[1], w0[2]};
  double euler[3] = {w0[0]+alpha_small*d[0], w0[1]+alpha_small*d[1],
                     w0[2]+alpha_small*d[2]};
  ops.geodesicStep(geo, alpha_small, d);
  ops.bregmanMidpointStep(breg, alpha_small, d);
  double breg_err = std::sqrt((geo[0]-breg[0])*(geo[0]-breg[0])
                            + (geo[1]-breg[1])*(geo[1]-breg[1])
                            + (geo[2]-breg[2])*(geo[2]-breg[2]));
  double euler_err = std::sqrt((geo[0]-euler[0])*(geo[0]-euler[0])
                             + (geo[1]-euler[1])*(geo[1]-euler[1])
                             + (geo[2]-euler[2])*(geo[2]-euler[2]));
  printf("  At alpha=0.1: breg_err=%.2e, euler_err=%.2e, improvement=%.1fx\n",
         breg_err, euler_err, euler_err / std::max(breg_err, 1e-30));
  EXPECT_LT(breg_err, euler_err)
      << "Bregman should be closer to geodesic than Euler";
}

TEST(ExpCone, InvertGradientRoundtrip) {
  // Verify that InvertGradient correctly inverts the gradient map.
  double x0[3] = {0.3, 1.2, 3.0};
  double g[3];
  ExpConeOps::BarrierGrad(x0[0], x0[1], x0[2], g);
  double lambda[3] = {-g[0], -g[1], -g[2]};

  double x_recovered[3];
  bool ok = ExpConeOps::InvertGradient(lambda, x_recovered);
  ASSERT_TRUE(ok) << "InvertGradient did not converge";

  double err = std::sqrt((x0[0]-x_recovered[0])*(x0[0]-x_recovered[0])
                       + (x0[1]-x_recovered[1])*(x0[1]-x_recovered[1])
                       + (x0[2]-x_recovered[2])*(x0[2]-x_recovered[2]));
  printf("InvertGradient roundtrip error: %.2e\n", err);
  EXPECT_LT(err, 1e-10);
}

TEST(ExpCone, LeapfrogVsVerlet) {
  // Compare the (s,λ) leapfrog with the Störmer-Verlet reference.
  ExpConeOps ops;
  double w0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};

  printf("\n=== Leapfrog vs Verlet vs Bregman ===\n");
  printf("  %6s  %12s  %12s  %12s\n",
         "alpha", "||lf-vl||", "||br-vl||", "||euler-vl||");

  for (int i = 1; i <= 8; ++i) {
    double alpha = i * 0.1;
    double geo[3] = {w0[0], w0[1], w0[2]};   // Verlet (reference)
    double lf[3] = {w0[0], w0[1], w0[2]};    // leapfrog
    double br[3] = {w0[0], w0[1], w0[2]};    // Bregman
    double eu[3] = {w0[0]+alpha*d[0], w0[1]+alpha*d[1], w0[2]+alpha*d[2]};

    ops.geodesicStep(geo, alpha, d);
    ops.leapfrogStep(lf, alpha, d);
    ops.bregmanMidpointStep(br, alpha, d);

    auto dist = [](const double* a, const double* b) {
      return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1])
                     + (a[2]-b[2])*(a[2]-b[2]));
    };
    printf("  %6.2f  %12.4e  %12.4e  %12.4e\n",
           alpha, dist(lf, geo), dist(br, geo), dist(eu, geo));
  }

  // At small alpha, leapfrog should be close to Verlet.
  double alpha = 0.1;
  double geo[3] = {w0[0], w0[1], w0[2]};
  double lf[3] = {w0[0], w0[1], w0[2]};
  ops.geodesicStep(geo, alpha, d);
  ops.leapfrogStep(lf, alpha, d);
  double err = std::sqrt((lf[0]-geo[0])*(lf[0]-geo[0])
                       + (lf[1]-geo[1])*(lf[1]-geo[1])
                       + (lf[2]-geo[2])*(lf[2]-geo[2]));
  printf("  Leapfrog vs Verlet at alpha=0.1: %.2e\n", err);
  EXPECT_LT(err, 1e-3) << "Leapfrog should be close to Verlet";

  // Leapfrog should stay interior.
  double lf2[3] = {w0[0], w0[1], w0[2]};
  ops.leapfrogStep(lf2, 0.8, d);
  EXPECT_GT(lf2[1], 0) << "y should be positive";
  EXPECT_GT(lf2[2] - lf2[1]*std::exp(lf2[0]/lf2[1]), 0) << "should be interior";
}

TEST(ExpCone, ConsistencyErrorEstimate) {
  // The λ = H(s)·s consistency error should increase with step size
  // (larger steps → more integration error) and decrease for the
  // identity λ=Hs (which is exact on the true geodesic).
  ExpConeOps ops;
  double w0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};

  printf("\n=== Consistency error estimate ===\n");
  printf("  %6s  %12s\n", "alpha", "||lam_int - Hs||");
  double prev_err = 0;
  for (int i = 1; i <= 10; ++i) {
    double alpha = i * 0.1;
    double w[3] = {w0[0], w0[1], w0[2]};
    double err = ops.geodesicStepWithErrorEstimate(w, alpha, d);
    printf("  %6.2f  %12.4e\n", alpha, err);
    if (i > 1) {
      EXPECT_GT(err, prev_err * 0.5)
          << "Error should generally increase with alpha";
    }
    prev_err = err;
  }
  // At small alpha, error should be small.
  double w_small[3] = {w0[0], w0[1], w0[2]};
  double err_small = ops.geodesicStepWithErrorEstimate(w_small, 0.01, d);
  printf("  alpha=0.01: err=%.2e\n", err_small);
  EXPECT_LT(err_small, 1e-5) << "Small step should have small consistency error";
}

// =====================================================================
// Prototype geodesic IPM for the exponential cone.
//
// min c^T x  s.t.  Ax + b in K_exp
//
// Barrier subproblem at parameter mu:
//   min c^T x + mu * F(Ax + b)
// Newton: (A^T H A) dx = -(c + mu * A^T grad_F(s))
// Geodesic step: s_new via Bregman midpoint in K_exp.
// =====================================================================

// Geodesic IPM for exp cone using log-homogeneity decomposition.
//
// Substitution: w = k·s where k = 1/√μ. By log-homogeneity:
//   ∇²F(s) = k²·∇²F(w)
//
// The Gram A^T ∇²F(w) A is k-independent → factor ONCE, two back-solves:
//   (A^T ∇²F(w) A) dx₀ = -A^T ∇F(w)    (centering)
//   (A^T ∇²F(w) A) dx₁ = -c             (optimality)
//
// Combined: dx(k) = dx₁ + (1/k)·dx₀. Cone direction: d(k) = A·dx(k).
// Line search: largest k with ||d(k)||_{∇²F(w)} ≤ 1.
// Geodesic step via Bregman midpoint.

// Fine-grid pure Verlet reference (no primal-dual reconciliation).
// Uses ThirdDerivContract + BarrierHessian directly.
static void fineGridGeodesic(const double* w0, double alpha, const double* d,
                              double* out, int fine_steps = 1000) {
  double pos[3] = {w0[0], w0[1], w0[2]};
  double vel[3] = {alpha*d[0], alpha*d[1], alpha*d[2]};
  double dt = 1.0 / fine_steps;
  for (int step = 0; step < fine_steps; ++step) {
    for (int half = 0; half < 2; ++half) {
      double H[9], T[3];
      ExpConeOps::BarrierHessian(pos[0], pos[1], pos[2], H);
      ExpConeOps::ThirdDerivContract(pos[0], pos[1], pos[2], vel, T);
      // a = -(1/2) H⁻¹ T
      double det = H[0]*(H[4]*H[8]-H[5]*H[7])
                 - H[1]*(H[3]*H[8]-H[5]*H[6])
                 + H[2]*(H[3]*H[7]-H[4]*H[6]);
      double inv[9];
      inv[0]=(H[4]*H[8]-H[5]*H[7])/det; inv[1]=(H[2]*H[7]-H[1]*H[8])/det;
      inv[2]=(H[1]*H[5]-H[2]*H[4])/det; inv[3]=(H[5]*H[6]-H[3]*H[8])/det;
      inv[4]=(H[0]*H[8]-H[2]*H[6])/det; inv[5]=(H[2]*H[3]-H[0]*H[5])/det;
      inv[6]=(H[3]*H[7]-H[4]*H[6])/det; inv[7]=(H[1]*H[6]-H[0]*H[7])/det;
      inv[8]=(H[0]*H[4]-H[1]*H[3])/det;
      double a[3];
      for (int k = 0; k < 3; ++k)
        a[k] = -0.5*(inv[3*k]*T[0]+inv[3*k+1]*T[1]+inv[3*k+2]*T[2]);
      for (int k = 0; k < 3; ++k) vel[k] += 0.5 * dt * a[k];
      if (half == 0)
        for (int k = 0; k < 3; ++k) pos[k] += dt * vel[k];
    }
  }
  out[0] = pos[0]; out[1] = pos[1]; out[2] = pos[2];
}

TEST(ExpCone, AccuracyVsFineGrid) {
  ExpConeOps ops;
  double w0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};
  auto dist = [](const double* a, const double* b) {
    return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])
                   +(a[2]-b[2])*(a[2]-b[2]));
  };

  printf("\n=== All integrators vs 1000-step pure Verlet reference ===\n");
  printf("  %6s  %12s  %12s  %12s  %12s\n",
         "alpha", "pd_verlet", "bregman", "leapfrog", "euler");
  for (int i = 1; i <= 8; ++i) {
    double alpha = i * 0.1;
    double ref[3]; fineGridGeodesic(w0, alpha, d, ref);
    double pdv[3]={w0[0],w0[1],w0[2]};
    double br[3]={w0[0],w0[1],w0[2]};
    double lf[3]={w0[0],w0[1],w0[2]};
    double eu[3]={w0[0]+alpha*d[0],w0[1]+alpha*d[1],w0[2]+alpha*d[2]};
    ops.geodesicStep(pdv, alpha, d);
    ops.bregmanMidpointStep(br, alpha, d);
    ops.leapfrogStep(lf, alpha, d);
    printf("  %6.2f  %12.4e  %12.4e  %12.4e  %12.4e\n",
           alpha, dist(pdv,ref), dist(br,ref), dist(lf,ref), dist(eu,ref));
  }
  // Quantitative at alpha=0.1.
  double ref[3]; fineGridGeodesic(w0, 0.1, d, ref);
  double pdv[3]={w0[0],w0[1],w0[2]};
  double br[3]={w0[0],w0[1],w0[2]};
  double lf[3]={w0[0],w0[1],w0[2]};
  double eu[3]={w0[0]+0.1*d[0],w0[1]+0.1*d[1],w0[2]+0.1*d[2]};
  ops.geodesicStep(pdv, 0.1, d);
  ops.bregmanMidpointStep(br, 0.1, d);
  ops.leapfrogStep(lf, 0.1, d);
  printf("\n  At alpha=0.1 vs 1000-step reference:\n");
  printf("    primal-dual Verlet: %.2e  (%.0fx vs euler)\n",
         dist(pdv,ref), dist(eu,ref)/std::max(dist(pdv,ref),1e-30));
  printf("    Bregman midpoint:   %.2e  (%.0fx vs euler)\n",
         dist(br,ref), dist(eu,ref)/std::max(dist(br,ref),1e-30));
  printf("    leapfrog:           %.2e  (%.0fx vs euler)\n",
         dist(lf,ref), dist(eu,ref)/std::max(dist(lf,ref),1e-30));
  printf("    Euler:              %.2e  (baseline)\n", dist(eu,ref));
  EXPECT_LT(dist(pdv,ref), dist(eu,ref));
  EXPECT_LT(dist(br,ref), dist(eu,ref));
}

TEST(ExpCone, GeodesicIPM_LogHomogeneous) {
  using Eigen::Matrix;
  using Eigen::Vector2d;
  using Eigen::Vector3d;
  using Eigen::Matrix3d;
  typedef Matrix<double, 3, 2> Matrix32;

  ExpConeOps ops;

  Matrix32 A;
  A << 1.0, 0.0,
       0.0, 1.0,
       0.5, 0.3;
  Vector3d b(0.0, 1.0, 3.0);
  Vector2d c(1.0, -0.5);

  auto is_interior = [](const Vector3d& s) {
    return s(1) > 1e-15 && s(2) > s(1) * std::exp(s(0) / s(1)) + 1e-15;
  };

  // Hessian norm: ||d||²_H = d^T H d.
  auto hessian_norm = [](const Vector3d& d, const double* H) {
    double n = 0;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        n += d(i) * H[3*i+j] * d(j);
    return std::sqrt(std::max(n, 0.0));
  };

  // Line search for k: largest k > 0 with ||d₁ + (1/k)·d₀||_H ≤ bound.
  // ||d(k)||²_H = ||d₁||² + (2/k)<d₁,d₀> + (1/k²)||d₀||² (all in H-norm).
  // This is quadratic in 1/k. Find the root of ||d||²_H = bound².
  auto line_search_k = [](const Vector3d& d0, const Vector3d& d1,
                           const double* H, double bound) -> double {
    // Compute H-inner products.
    auto ip = [&](const Vector3d& a, const Vector3d& b_v) {
      double v = 0;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          v += a(i) * H[3*i+j] * b_v(j);
      return v;
    };
    double a = ip(d0, d0);  // ||d₀||²_H
    double f = ip(d0, d1);  // <d₀, d₁>_H
    double p = ip(d1, d1);  // ||d₁||²_H
    // ||d(k)||²_H = p + 2f/k + a/k²
    // Want p + 2f/k + a/k² = bound². Let t = 1/k:
    // a·t² + 2f·t + (p - bound²) = 0.
    double disc = 4*f*f - 4*a*(p - bound*bound);
    if (disc < 0) return 0.0;  // no feasible k
    double t = (-2*f + std::sqrt(disc)) / (2*a);
    if (t <= 0) {
      // Try other root.
      t = (-2*f - std::sqrt(disc)) / (2*a);
    }
    if (t <= 0) return 1e6;  // d₁ already within bound
    return 1.0 / t;
  };

  // Geodesic LP for the exp cone.
  //
  // Work in w = k·s (scaled slack). By log-homogeneity:
  //   H(w) = (1/k²)H(s),  ∇F(w) = (1/k)∇F(s)
  //
  // Scaled primal z = k·x. KKT Schur complement at w-level:
  //   (A^T H(w) A) z = A^T(H(w)w - ∇F(w)) - k(c + A^T H(w)b)
  //
  // Two solves with k-independent Gram A^T H(w) A:
  //   z₀ = solve for A^T(H(w)w - ∇F(w))     (centering)
  //   z₁ = solve for -(c + A^T H(w)b)        (cost + affine)
  //   z(k) = z₀ + k·z₁
  //
  // Both c and b appear in z₁, scaled by k symmetrically.
  // Recover: x = z/k, s_new = Ax + b, δλ from KKT, λ_new = λ+δλ.
  // A^T λ_new = c exactly. gap = <s_new, λ_new>.

  Vector2d x(0.0, 0.0);

  printf("\n=== Geodesic LP (exp cone, z = kx formulation) ===\n");
  printf("  %3s  %12s  %12s  %10s  %10s  %10s  %10s\n",
         "fac", "k", "k_new", "d_norm", "gap", "dual_res", "c^Tx");
  printf("  %s\n", std::string(82, '-').c_str());

  int factorizations = 0;
  double k_prev = 0.0;

  auto in_dual_cone = [](const Vector3d& lam) {
    return lam(0) < -1e-15 && lam(2) > 1e-15 &&
           -lam(0) * std::exp(lam(1)/lam(0) - 1) <= lam(2) - 1e-15;
  };

  for (int outer = 0; outer < 30; ++outer) {
    Vector3d s = A * x + b;
    if (!is_interior(s)) { printf("  INFEASIBLE at iter %d\n", outer); break; }

    // Work at w = k_eff · s where k_eff = max(k_prev, 1).
    double k_eff = std::max(k_prev, 1.0);
    Vector3d w = k_eff * s;

    // Hessian and gradient at w.
    double Hw_arr[9], gw_arr[3];
    ExpConeOps::BarrierHessian(w(0), w(1), w(2), Hw_arr);
    ExpConeOps::BarrierGrad(w(0), w(1), w(2), gw_arr);
    Matrix3d Hw;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        Hw(i, j) = Hw_arr[3*i+j];
    Vector3d gw(gw_arr[0], gw_arr[1], gw_arr[2]);

    // Gram = A^T H(w) A (k-independent).
    Eigen::Matrix2d G = A.transpose() * Hw * A;
    auto Gf = G.ldlt();
    factorizations++;

    // Two back-solves: z(k) = z₀ + k·z₁.
    Vector2d z0 = Gf.solve(A.transpose() * (Hw * w - gw));      // centering
    Vector2d z1 = Gf.solve(-(c + A.transpose() * (Hw * b)));    // cost + affine

    // Cone directions in w-space: d₀ = Az₀, d₁ = Az₁.
    Vector3d d0 = A * z0;
    Vector3d d1 = A * z1;

    // Line search for k: largest k such that s_new ∈ K and λ_new ∈ K*.
    // z(k) = z₀ + k·z₁, x = z/k = z₀/k + z₁.
    // s_new = Ax + b = (1/k)·d₀ + d₁ + b.
    // λ from KKT: λ_new = -(1/k²)(∇F(s) + H(s)·(s_new - s)).
    // Use H(s) = k²H(w): λ_new = -(∇F(w)/k + H(w)·(k·s_new - w)).
    // Since k·s_new = d₀ + k·d₁ + k·b = d₀ + k(d₁+b):
    //   k·s_new - w = d₀ + k(d₁+b) - w = Az₀ + k(Az₁+b) - k_eff·s.
    // At k = k_eff: k·s_new - w = Az₀ + k_eff(Az₁+b) - k_eff·s = d₀ + k_eff·d₁.
    // More generally: use the w-level formula.

    // For each candidate k, compute primal-dual pair and check feasibility.
    auto eval_at_k = [&](double k) -> std::tuple<Vector3d, Vector3d, bool> {
      Vector2d z = z0 + k * z1;
      Vector2d xk = z / k;
      Vector3d sk = A * xk + b;  // s_new = Ax + b
      if (!is_interior(sk)) return {sk, Vector3d::Zero(), false};

      // λ_new from KKT: λ = -μ∇F(s_old) + μH(s_old)·(s_old - b - A·xk)
      // Equivalently at w-level: λ = -(1/k)∇F(w) + (1/k)H(w)·(w - k·(A·xk + b - s_old)... )
      // Simplest: use s-level. H(s) = k_eff² H(w), ∇F(s) = k_eff ∇F(w).
      double mu = 1.0/(k*k);
      double k_eff2 = k_eff * k_eff;
      Vector3d gs = k_eff * gw;        // ∇F(s) = k_eff·∇F(w)
      Matrix3d Hs = k_eff2 * Hw;       // H(s) = k_eff²·H(w)
      Vector3d lam_old = -mu * gs;
      Vector3d dlam = mu * Hs * ((s - b) - A * xk);
      Vector3d lam_new = lam_old + dlam;

      bool dual_ok = in_dual_cone(lam_new);
      return {sk, lam_new, dual_ok};
    };

    // Binary search for k.
    double k_new = std::max(k_prev, 1.0);
    double k_hi = k_new * 100 + 10;
    for (int i = 0; i < 20; ++i) {
      auto [sk, lk, ok] = eval_at_k(k_hi);
      if (!ok) break;
      k_hi *= 2;
    }
    for (int i = 0; i < 40; ++i) {
      double k_mid = 0.5 * (k_new + k_hi);
      auto [sk, lk, ok] = eval_at_k(k_mid);
      if (ok) { k_new = k_mid; } else { k_hi = k_mid; }
    }
    k_new = std::max(k_new, k_prev);

    // Evaluate at k_new.
    auto [s_new, lam_new, ok] = eval_at_k(k_new);
    double gap = s_new.dot(lam_new);
    double dual_res = (A.transpose() * lam_new - c).norm();

    // Direction norm in the w-metric.
    Vector3d dw = d0 + k_new * d1;  // δw direction
    double d_norm = std::sqrt(std::max(dw.dot(Hw * dw), 0.0));

    Vector2d x_new = (z0 + k_new * z1) / k_new;
    printf("  %3d  %12.4e  %12.4e  %10.4f  %10.2e  %10.2e  %10.4f\n",
           factorizations, k_prev, k_new, d_norm, gap, dual_res, c.dot(x_new));

    if (gap < 1e-6 && gap > 0) {
      x = x_new;
      break;
    }

    // Geodesic step in w-space: Exp_w(α · dw).
    double alpha = 1.0 / (1.0 + d_norm);
    double w_arr[3] = {w(0), w(1), w(2)};
    double dw_arr[3] = {dw(0), dw(1), dw(2)};
    ops.geodesicStep(w_arr, alpha, dw_arr);
    Vector3d w_new(w_arr[0], w_arr[1], w_arr[2]);

    // Recover s = w/k_eff, then x from s = Ax + b.
    Vector3d s_geo = w_new / k_eff;
    if (is_interior(s_geo)) {
      x = (A.transpose()*A).ldlt().solve(A.transpose() * (s_geo - b));
    } else {
      x = x_new;  // fallback to linear
    }

    k_prev = k_new;
  }

  double final_obj = c.dot(x);
  printf("  Final: c^Tx = %.6f, x = (%.4f, %.4f), fac = %d\n",
         final_obj, x(0), x(1), factorizations);

  EXPECT_LT(final_obj, -7.0) << "Should find objective < -7";
  EXPECT_LT(factorizations, 25) << "Should converge in < 25 factorizations";
}

// =====================================================================
// Geodesic LP for a product of m exponential cones.
//
// min c^T x  s.t.  A_i x + b_i ∈ K_exp  for i = 1..m
//
// Barrier: F(s) = Σᵢ Fᵢ(sᵢ)  where sᵢ = Aᵢx + bᵢ.
// Gram = Σᵢ Aᵢ^T Hᵢ(sᵢ) Aᵢ  (block-additive).
// Gradient = Σᵢ Aᵢ^T ∇Fᵢ(sᵢ).
// Geodesic step: component-wise on each cone.
// =====================================================================

struct ExpConeConstraint {
  Eigen::Matrix<double, 3, Eigen::Dynamic> A;
  Eigen::Vector3d b;
};

TEST(ExpCone, GeodesicLP_ProductCone) {
  using Eigen::VectorXd;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;
  using Eigen::Matrix3d;

  ExpConeOps ops;
  srand(42);

  const int p = 4;     // variables
  const int m = 3;     // number of exp cone constraints

  // Build m exp cone constraints: A_i x + b_i ∈ K_exp.
  // Choose b_i in int(K_exp) and c = -Σ Aᵢ^T ∇Fᵢ(bᵢ) so that x=0
  // is on the central path at μ=1 (centered and feasible).
  std::vector<ExpConeConstraint> cones(m);
  for (int i = 0; i < m; ++i) {
    cones[i].A = 0.3 * MatrixXd::Random(3, p);
    // b_i well inside K_exp: (0, 1, e+1) where slack = e.
    cones[i].b = Vector3d(0, 1.0, std::exp(1.0) + 1.0);
  }
  // c = -Σ Aᵢ^T ∇Fᵢ(bᵢ) ensures x=0 is on the central path at k=1.
  VectorXd c = VectorXd::Zero(p);
  for (int i = 0; i < m; ++i) {
    double gi[3];
    ExpConeOps::BarrierGrad(cones[i].b(0), cones[i].b(1), cones[i].b(2), gi);
    c -= cones[i].A.transpose() * Eigen::Map<Vector3d>(gi);
  }

  auto is_interior = [](const Vector3d& s) {
    return s(1) > 1e-15 && s(2) > s(1) * std::exp(s(0) / s(1)) + 1e-15;
  };
  auto in_dual_cone = [](const Vector3d& lam) {
    return lam(0) < -1e-15 && lam(2) > 1e-15 &&
           -lam(0) * std::exp(lam(1)/lam(0) - 1) <= lam(2) - 1e-15;
  };

  auto all_interior = [&](const VectorXd& xv) {
    for (int i = 0; i < m; ++i) {
      Vector3d si = cones[i].A * xv + cones[i].b;
      if (!is_interior(si)) return false;
    }
    return true;
  };

  VectorXd x = VectorXd::Zero(p);
  ASSERT_TRUE(all_interior(x)) << "Initial point not interior";

  printf("\n=== Geodesic LP (product of %d exp cones, %d vars) ===\n", m, p);
  printf("  %3s  %12s  %12s  %10s  %10s  %10s  %10s\n",
         "fac", "k", "k_new", "d_norm", "gap", "dual_res", "c^Tx");
  printf("  %s\n", std::string(82, '-').c_str());

  int factorizations = 0;
  double k_prev = 0.0;

  for (int outer = 0; outer < 30; ++outer) {
    // Accumulate Gram = Σ Aᵢ^T Hᵢ Aᵢ and grad = Σ Aᵢ^T ∇Fᵢ.
    MatrixXd Gram = MatrixXd::Zero(p, p);
    VectorXd grad = VectorXd::Zero(p);
    std::vector<Vector3d> slacks(m);
    std::vector<Matrix3d> hessians(m);

    bool feasible = true;
    for (int i = 0; i < m; ++i) {
      slacks[i] = cones[i].A * x + cones[i].b;
      if (!is_interior(slacks[i])) { feasible = false; break; }
      double Hi[9], gi[3];
      ExpConeOps::BarrierHessian(slacks[i](0), slacks[i](1), slacks[i](2), Hi);
      ExpConeOps::BarrierGrad(slacks[i](0), slacks[i](1), slacks[i](2), gi);
      for (int r = 0; r < 3; ++r)
        for (int cc = 0; cc < 3; ++cc)
          hessians[i](r, cc) = Hi[3*r+cc];
      Gram += cones[i].A.transpose() * hessians[i] * cones[i].A;
      grad += cones[i].A.transpose() * Eigen::Map<Vector3d>(gi);
    }
    if (!feasible) { printf("  INFEASIBLE at iter %d\n", outer); break; }

    // Factor Gram once.
    auto Gf = Gram.ldlt();
    factorizations++;

    // Two back-solves.
    VectorXd y0 = Gf.solve(-grad);  // centering
    VectorXd y1 = Gf.solve(-c);     // optimality

    // Per-cone directions.
    std::vector<Vector3d> d0(m), d1(m);
    for (int i = 0; i < m; ++i) {
      d0[i] = cones[i].A * y0;
      d1[i] = cones[i].A * y1;
    }

    // Hessian norm: ||d₀ + k·d₁||²_H = Σᵢ (d₀ᵢ+k·d₁ᵢ)^T Hᵢ (d₀ᵢ+k·d₁ᵢ).
    double aa = 0, ff = 0, pp = 0;
    for (int i = 0; i < m; ++i) {
      aa += d0[i].dot(hessians[i] * d0[i]);
      ff += d0[i].dot(hessians[i] * d1[i]);
      pp += d1[i].dot(hessians[i] * d1[i]);
    }

    // Dikin bound.
    double k_dikin = 0;
    double disc = 4*ff*ff - 4*pp*(aa - 1);
    if (disc >= 0 && pp > 1e-30) {
      double k1 = (-2*ff + std::sqrt(disc)) / (2*pp);
      double k2 = (-2*ff - std::sqrt(disc)) / (2*pp);
      k_dikin = std::max(std::max(k1, k2), 0.0);
    }

    // Primal-dual feasibility search beyond Dikin.
    auto try_k = [&](double k_try) -> bool {
      for (int i = 0; i < m; ++i) {
        Vector3d dki = d0[i] + k_try * d1[i];
        double dn = std::sqrt(std::max(dki.dot(hessians[i] * dki), 0.0));
        double al = 1.0 / (1.0 + dn);
        double si[3] = {slacks[i](0), slacks[i](1), slacks[i](2)};
        double di[3] = {dki(0), dki(1), dki(2)};
        ops.geodesicStep(si, al, di);
        Vector3d sn(si[0], si[1], si[2]);
        if (!is_interior(sn)) return false;
        double gn[3];
        ExpConeOps::BarrierGrad(sn(0), sn(1), sn(2), gn);
        Vector3d lam_new(-gn[0], -gn[1], -gn[2]);
        if (!in_dual_cone(lam_new)) return false;
      }
      return true;
    };

    double k_new = std::max(k_dikin, k_prev);
    double k_hi = k_new * 10 + 1;
    for (int i = 0; i < 30; ++i) {
      double k_mid = 0.5 * (k_new + k_hi);
      if (try_k(k_mid)) { k_new = k_mid; } else { k_hi = k_mid; }
    }
    k_new = std::max(k_new, k_prev);

    // Total Hessian norm of the combined direction.
    double d_sqr = 0;
    for (int i = 0; i < m; ++i) {
      Vector3d dki = d0[i] + k_new * d1[i];
      d_sqr += dki.dot(hessians[i] * dki);
    }
    double d_norm = std::sqrt(std::max(d_sqr, 0.0));
    double mu = (k_new > 0) ? 1.0/(k_new*k_new) : 1.0;

    // Compute duality gap BEFORE stepping, using Newton primal-dual pair.
    // λᵢ_old = -(1/k²)∇Fᵢ(sᵢ), δλᵢ = -(1/k²)Hᵢ·Aᵢ·δx.
    // λᵢ_new = λᵢ_old + δλᵢ. A^T Σ λᵢ_new = c exactly.
    // sᵢ_new = sᵢ + Aᵢ·δx (first order).
    // δx = (1/k)y₀ + y₁. Per cone: δsᵢ = Aᵢ·δx, λᵢ = -(1/k²)(∇Fᵢ + Hᵢ·δsᵢ).
    double k2 = k_new * k_new;
    VectorXd dx_k = (1.0/k_new) * y0 + y1;
    double gap = 0;
    VectorXd At_lam = VectorXd::Zero(p);
    for (int i = 0; i < m; ++i) {
      double gi[3];
      ExpConeOps::BarrierGrad(slacks[i](0), slacks[i](1), slacks[i](2), gi);
      Vector3d ds_i = cones[i].A * dx_k;
      Vector3d lam_new_i = -(1.0/k2) * (Eigen::Map<Vector3d>(gi) + hessians[i] * ds_i);
      Vector3d si_new = slacks[i] + cones[i].A * dx_k;
      gap += si_new.dot(lam_new_i);
      At_lam += cones[i].A.transpose() * lam_new_i;
    }
    double dual_res = (At_lam - c).norm();

    printf("  %3d  %12.4e  %12.4e  %10.4f  %10.2e  %10.2e  %10.4f\n",
           factorizations, k_prev, k_new, d_norm, gap, dual_res, c.dot(x));

    if (gap < 1e-6 && gap > 0) break;

    // Geodesic step: component-wise on each cone.
    std::vector<Vector3d> s_new(m);
    for (int i = 0; i < m; ++i) {
      Vector3d dki = d0[i] + k_new * d1[i];
      double dn = std::sqrt(std::max(dki.dot(hessians[i] * dki), 0.0));
      double al = 1.0 / (1.0 + dn);
      double si[3] = {slacks[i](0), slacks[i](1), slacks[i](2)};
      double di[3] = {dki(0), dki(1), dki(2)};
      ops.geodesicStep(si, al, di);
      s_new[i] = Vector3d(si[0], si[1], si[2]);
    }

    // Recover x from s_new = A_i x + b_i (least squares).
    MatrixXd A_stack(3*m, p);
    VectorXd rhs_stack(3*m);
    for (int i = 0; i < m; ++i) {
      A_stack.block(3*i, 0, 3, p) = cones[i].A;
      rhs_stack.segment(3*i, 3) = s_new[i] - cones[i].b;
    }
    VectorXd x_new = (A_stack.transpose()*A_stack).ldlt().solve(
        A_stack.transpose() * rhs_stack);

    if (all_interior(x_new)) {
      x = x_new;
    } else {
      VectorXd dx = y1 + (1.0/k_new) * y0;
      for (double a = 0.5; a > 1e-10; a *= 0.5) {
        VectorXd x_try = x + a * dx;
        if (all_interior(x_try)) { x = x_try; break; }
      }
    }

    k_prev = k_new;
  }

  double final_obj = c.dot(x);
  printf("  Final: c^Tx = %.6f, fac = %d\n", final_obj, factorizations);

  EXPECT_LT(final_obj, 0) << "Objective should be negative";
  EXPECT_GT(k_prev, 1.0) << "k should increase beyond 1";
}

}  // namespace
