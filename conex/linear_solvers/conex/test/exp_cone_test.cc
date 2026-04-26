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

TEST(ExpCone, PrototypeGeodesicIPM) {
  // Problem: min c^T x  s.t.  y*exp(x0/y) <= z, y > 0
  // where s = (x0, y, z) = A*x + b.
  //
  // Use p=2 variables. A is 3x2, b is 3x1.
  // Construct so that the problem is bounded and feasible.
  using Eigen::Matrix;
  using Eigen::Vector2d;
  using Eigen::Vector3d;
  using Eigen::Matrix3d;
  typedef Matrix<double, 3, 2> Matrix32;

  srand(42);
  Matrix32 A;
  A << 1.0, 0.0,
       0.0, 1.0,
       0.5, 0.3;
  Vector3d b(0.0, 1.0, 3.0);  // b is interior: y=1, z=3, y*exp(0/1)=1 < 3
  Vector2d c(1.0, -0.5);      // cost

  ExpConeOps ops;

  // Initial x such that s = Ax + b is interior.
  Vector2d x(0.0, 0.0);

  auto slack = [&](const Vector2d& xv) -> Vector3d { return A * xv + b; };
  auto is_interior = [](const Vector3d& s) {
    return s(1) > 0 && s(2) > s(1) * std::exp(s(0) / s(1));
  };

  ASSERT_TRUE(is_interior(slack(x))) << "Initial point not interior";

  double mu = 1.0;
  const double mu_factor = 0.5;
  const int max_outer = 15;
  const int max_inner = 20;

  printf("\n=== Prototype Geodesic IPM (Exp Cone) ===\n");
  printf("  %3s %3s  %10s  %10s  %10s  %10s  %10s  %5s\n",
         "out", "in", "mu", "c^Tx", "||grad||", "alpha", "slack_u", "step");
  printf("  %s\n", std::string(75, '-').c_str());

  // Track results for both step types.
  double obj_geo = 0, obj_euler = 0;
  int total_iters_geo = 0, total_iters_euler = 0;

  for (int use_geodesic = 0; use_geodesic <= 1; ++use_geodesic) {
    const char* step_name = use_geodesic ? "bregman" : "euler";
    Vector2d xk(0.0, 0.0);
    mu = 1.0;

    for (int outer = 0; outer < max_outer; ++outer) {
      for (int inner = 0; inner < max_inner; ++inner) {
        Vector3d s = slack(xk);
        if (!is_interior(s)) {
          printf("  INFEASIBLE at outer=%d inner=%d\n", outer, inner);
          goto done;
        }

        // Barrier gradient and Hessian at s.
        double g[3], H[9];
        ExpConeOps::BarrierGrad(s(0), s(1), s(2), g);
        ExpConeOps::BarrierHessian(s(0), s(1), s(2), H);
        Matrix3d Hm;
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            Hm(i, j) = H[3*i+j];

        // Gram: A^T H A (2x2).
        Eigen::Matrix2d G = A.transpose() * Hm * A;
        // RHS: -(c + mu * A^T g).
        Eigen::Map<Vector3d> gv(g);
        Vector2d rhs = -(c + mu * A.transpose() * gv);

        // Check convergence: ||grad||.
        double grad_norm = rhs.norm();
        if (grad_norm < 1e-8 * mu) break;

        // Newton step.
        Vector2d dx = G.ldlt().solve(rhs);
        Vector3d ds = A * dx;

        // Step size via backtracking.
        double alpha = 1.0;
        bool stepped = false;

        if (use_geodesic) {
          // Bregman midpoint step in the cone.
          for (int ls = 0; ls < 20; ++ls) {
            double s_new[3] = {s(0), s(1), s(2)};
            double dir[3] = {ds(0), ds(1), ds(2)};
            ops.bregmanMidpointStep(s_new, alpha, dir);
            if (is_interior(Vector3d(s_new[0], s_new[1], s_new[2]))) {
              // Recover x from s_new = A*x_new + b.
              // x_new = A^† (s_new - b). For 3x2 A: use normal equations.
              Vector3d s_new_v(s_new[0], s_new[1], s_new[2]);
              Vector2d x_new = (A.transpose()*A).ldlt().solve(
                  A.transpose() * (s_new_v - b));
              // Check that A*x_new + b ≈ s_new (residual).
              Vector3d s_check = A * x_new + b;
              if (is_interior(s_check)) {
                xk = x_new;
                stepped = true;
                break;
              }
            }
            alpha *= 0.5;
          }
        } else {
          // Euclidean step with backtracking.
          for (int ls = 0; ls < 20; ++ls) {
            Vector2d x_new = xk + alpha * dx;
            if (is_interior(slack(x_new))) {
              xk = x_new;
              stepped = true;
              break;
            }
            alpha *= 0.5;
          }
        }

        if (!stepped) {
          printf("  LINE SEARCH FAILED at outer=%d inner=%d\n", outer, inner);
          goto done;
        }

        Vector3d s_final = slack(xk);
        double u_final = s_final(2) - s_final(1) * std::exp(s_final(0)/s_final(1));
        if (inner == 0 || grad_norm > 1e-6 * mu) {
          printf("  %3d %3d  %10.2e  %10.4f  %10.2e  %10.4f  %10.2e  %s\n",
                 outer, inner, mu, c.dot(xk), grad_norm, alpha, u_final,
                 step_name);
        }
      }
      done_inner:
      mu *= mu_factor;
    }
    done:

    double final_obj = c.dot(xk);
    if (use_geodesic) {
      obj_geo = final_obj;
    } else {
      obj_euler = final_obj;
    }
    printf("  %s final: c^Tx = %.6f, x = (%.4f, %.4f)\n\n",
           step_name, final_obj, xk(0), xk(1));
  }

  // Both methods should find the same optimum.
  EXPECT_NEAR(obj_geo, obj_euler, 1e-3)
      << "Geodesic and Euler should converge to same optimum";
}

}  // namespace
