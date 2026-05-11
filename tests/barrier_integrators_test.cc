// Test barrier integrators against a fine-grid Verlet reference.
// Verifies: accuracy, convergence order, reversibility, energy conservation.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "conex/common/barrier_integrators.h"
#include "conex/common/exp_cone_ops.h"
#include "conex/common/power_cone_ops.h"
#include "conex/common/rel_entropy_cone_ops.h"
#include "conex/common/hypo_geomean_cone_ops.h"

using Ops = conex::EuclideanJordanAlgebra::BarrierConeOperations;
using conex::EuclideanJordanAlgebra::ExpConeOps;
using conex::EuclideanJordanAlgebra::PowerConeOps;
using conex::EuclideanJordanAlgebra::RelEntropyConeOps;
using conex::EuclideanJordanAlgebra::HypoGeoMeanConeOps;

// Fine-grid reference using repeated Euler steps (simple, reliable).
// The Euler step z += h*vel is first-order but with 10000 steps
// the error is O(1/10000) — accurate enough as a reference.
static void fineGridReference(const Ops* ops, const double* z0,
                               double alpha, const double* d,
                               double* out, int size, int steps = 10000) {
  double z[conex::kMaxBarrierDim];
  double vel[conex::kMaxBarrierDim];
  double grad[conex::kMaxBarrierDim];
  double H_buf[conex::kMaxBarrierDim * conex::kMaxBarrierDim];
  std::memcpy(z, z0, size * sizeof(double));
  for (int i = 0; i < size; ++i) vel[i] = alpha * d[i];

  // Störmer-Verlet with dense Hessian solve for acceleration.
  // a = -½ H⁻¹ D³φ[v,v,·] approximated via finite diff of gradient.
  double dt = 1.0 / steps;
  for (int s = 0; s < steps; ++s) {
    // Compute acceleration via finite diff: a ≈ -H⁻¹ [∇φ(z+ε·v) - ∇φ(z-ε·v)]/(2ε) + v
    // Actually, simpler: use the geodesic equation directly.
    // Half-kick: vel += 0.5*dt*accel, drift: z += dt*vel, half-kick again.
    // Acceleration from: H(z)·a = -½ D³φ[v,v,·].
    // Approximate D³φ[v,v,·] via finite diff of H·v:
    //   D³φ[v,v,e_l] ≈ (H(z+ε·e_l)·v - H(z-ε·e_l)·v) / (2ε)
    // This is expensive for large n but fine for testing (n ≤ 5).
    auto accel = [&](double* a) {
      double eps = 1e-5;
      Eigen::Map<Eigen::VectorXd> av(a, size);
      Eigen::Map<Eigen::VectorXd> zv(z, size);
      double Hv_p[conex::kMaxBarrierDim], Hv_m[conex::kMaxBarrierDim];
      double T[conex::kMaxBarrierDim];
      for (int l = 0; l < size; ++l) {
        z[l] += eps;
        ops->hessianProduct(Hv_p, z, vel, size);
        z[l] -= 2*eps;
        ops->hessianProduct(Hv_m, z, vel, size);
        z[l] += eps;  // restore
        T[l] = 0;
        for (int j = 0; j < size; ++j)
          T[l] += vel[j] * (Hv_p[j] - Hv_m[j]) / (2*eps);
      }
      ops->hessian(H_buf, z, size);
      Eigen::Map<Eigen::MatrixXd> H(H_buf, size, size);
      Eigen::Map<Eigen::VectorXd> Tv(T, size);
      av = H.ldlt().solve(-0.5 * Tv);
    };

    double a[conex::kMaxBarrierDim];
    accel(a);
    for (int i = 0; i < size; ++i) vel[i] += 0.5 * dt * a[i];
    for (int i = 0; i < size; ++i) z[i] += dt * vel[i];
    accel(a);
    for (int i = 0; i < size; ++i) vel[i] += 0.5 * dt * a[i];
  }
  std::memcpy(out, z, size * sizeof(double));
}

static double dist(const double* a, const double* b, int n) {
  double s = 0;
  for (int i = 0; i < n; ++i) s += (a[i]-b[i])*(a[i]-b[i]);
  return std::sqrt(s);
}

// Riemannian energy: v^T H(z) v.
static double energy(const Ops* ops, const double* z, const double* vel, int n) {
  double Hv[conex::kMaxBarrierDim];
  ops->hessianProduct(Hv, z, vel, n);
  double e = 0;
  for (int i = 0; i < n; ++i) e += vel[i] * Hv[i];
  return e;
}

struct ConeTestCase {
  const Ops* ops;
  std::vector<double> z0;
  std::vector<double> d;
  const char* name;
};

static ExpConeOps g_exp_ops;
static Eigen::VectorXd g_power_alpha;
static PowerConeOps* g_power_ops = nullptr;
static RelEntropyConeOps g_relent_ops;
static HypoGeoMeanConeOps g_geomean_ops;

static std::vector<ConeTestCase> makeCases() {
  g_power_alpha.resize(2);
  g_power_alpha << 0.4, 0.6;
  static PowerConeOps power_ops(g_power_alpha);
  g_power_ops = &power_ops;

  return {
    {&g_exp_ops,    {0.1, 1.0, 2.5},           {0.2, -0.1, 0.15},         "ExpCone"},
    {g_power_ops,   {2.0, 1.5, 0.3, 0.2},      {0.1, -0.1, 0.05, -0.03}, "PowerCone"},
    {&g_relent_ops, {3.0, 1.0, 2.0, 0.5, 0.8}, {0.1, -0.1, 0.05, 0.08, -0.05}, "RelEntropy"},
    {&g_geomean_ops,{0.5, 2.0, 1.5, 1.8},      {0.05, -0.1, 0.08, -0.06},"HypoGeoMean"},
  };
}

// ============================================================
// Accuracy vs fine-grid reference
// ============================================================
TEST(BarrierIntegrators, AccuracyVsReference) {
  auto cases = makeCases();

  for (const auto& tc : cases) {
    int n = tc.z0.size();
    double alpha = 0.3;

    double ref[conex::kMaxBarrierDim];
    fineGridReference(tc.ops, tc.z0.data(), alpha, tc.d.data(), ref, n);

    // Symmetric (1 step).
    double z_sym[conex::kMaxBarrierDim], vel_sym[conex::kMaxBarrierDim];
    std::memcpy(z_sym, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel_sym[i] = alpha * tc.d[i];
    conex::symmetricStep(tc.ops, z_sym, vel_sym, 1.0, n);
    double err_sym = dist(z_sym, ref, n);

    // Primal midpoint (1 step).
    double z_pm[conex::kMaxBarrierDim], vel_pm[conex::kMaxBarrierDim];
    std::memcpy(z_pm, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel_pm[i] = alpha * tc.d[i];
    conex::primalMidpointStep(tc.ops, z_pm, vel_pm, 1.0, n);
    double err_pm = dist(z_pm, ref, n);

    // Dual midpoint (1 step).
    double z_dm[conex::kMaxBarrierDim], vel_dm[conex::kMaxBarrierDim];
    std::memcpy(z_dm, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel_dm[i] = alpha * tc.d[i];
    conex::dualMidpointStep(tc.ops, z_dm, vel_dm, 1.0, n);
    double err_dm = dist(z_dm, ref, n);

    // Yoshida-4 variants (1 composite step = 3 sub-steps each).
    auto run_y4 = [&](auto step_fn, double* z_out) {
      double vel[conex::kMaxBarrierDim];
      std::memcpy(z_out, tc.z0.data(), n * sizeof(double));
      for (int i = 0; i < n; ++i) vel[i] = alpha * tc.d[i];
      step_fn(tc.ops, z_out, vel, 1.0, n);
      return dist(z_out, ref, n);
    };
    double z_y4s[conex::kMaxBarrierDim], z_y4p[conex::kMaxBarrierDim], z_y4d[conex::kMaxBarrierDim];
    double err_y4s = run_y4(conex::yoshida4Step, z_y4s);
    double err_y4p = run_y4(conex::yoshida4PrimalStep, z_y4p);
    double err_y4d = run_y4(conex::yoshida4DualStep, z_y4d);

    // Euler.
    double z_eu[conex::kMaxBarrierDim];
    for (int i = 0; i < n; ++i) z_eu[i] = tc.z0[i] + alpha * tc.d[i];
    double err_eu = dist(z_eu, ref, n);

    printf("\n  %s (alpha=%.1f):\n", tc.name, alpha);
    printf("    Symmetric:       %.4e  (%.0fx vs euler)\n",
           err_sym, err_eu / std::max(err_sym, 1e-30));
    printf("    Primal-mid:      %.4e  (%.0fx vs euler)\n",
           err_pm, err_eu / std::max(err_pm, 1e-30));
    printf("    Dual-mid:        %.4e  (%.0fx vs euler)\n",
           err_dm, err_eu / std::max(err_dm, 1e-30));
    printf("    Y4-symmetric:    %.4e  (%.0fx vs euler)\n",
           err_y4s, err_eu / std::max(err_y4s, 1e-30));
    printf("    Y4-primal-mid:   %.4e  (%.0fx vs euler)\n",
           err_y4p, err_eu / std::max(err_y4p, 1e-30));
    printf("    Y4-dual-mid:     %.4e  (%.0fx vs euler)\n",
           err_y4d, err_eu / std::max(err_y4d, 1e-30));
    printf("    Euler:           %.4e  (baseline)\n", err_eu);

    // Also measure dual error: ||∇φ(z) - ∇φ(z_ref)||.
    auto dual_err = [&](const double* z_test) {
      double g_test[conex::kMaxBarrierDim], g_ref[conex::kMaxBarrierDim];
      tc.ops->computeGradient(g_test, z_test, n);
      tc.ops->computeGradient(g_ref, ref, n);
      return dist(g_test, g_ref, n);
    };

    printf("    --- dual space (grad error) ---\n");
    printf("    Symmetric:       %.4e\n", dual_err(z_sym));
    printf("    Primal-mid:      %.4e\n", dual_err(z_pm));
    printf("    Dual-mid:        %.4e\n", dual_err(z_dm));
    printf("    Y4-symmetric:    %.4e\n", dual_err(z_y4s));
    printf("    Y4-primal-mid:   %.4e\n", dual_err(z_y4p));
    printf("    Y4-dual-mid:     %.4e\n", dual_err(z_y4d));
    printf("    Euler:           %.4e\n", dual_err(z_eu));

    // All integrators should beat Euler.
    EXPECT_LT(err_pm, err_eu) << tc.name << ": primal midpoint worse than Euler";
    EXPECT_LT(err_dm, err_eu) << tc.name << ": dual midpoint worse than Euler";
    EXPECT_LT(err_y4s, err_sym) << tc.name << ": Y4-sym worse than symmetric";
    EXPECT_LT(err_y4p, err_pm) << tc.name << ": Y4-primal worse than primal";
    EXPECT_LT(err_y4d, err_dm) << tc.name << ": Y4-dual worse than dual";
  }
}

// ============================================================
// Convergence order: halve h → error should decrease by ~4x (2nd order)
// ============================================================
TEST(BarrierIntegrators, ConvergenceOrder) {
  auto cases = makeCases();

  for (const auto& tc : cases) {
    int n = tc.z0.size();
    double alpha = 0.2;

    double ref[conex::kMaxBarrierDim];
    fineGridReference(tc.ops, tc.z0.data(), alpha, tc.d.data(), ref, n);

    // 1 step of h=1.
    double z1[conex::kMaxBarrierDim], v1[conex::kMaxBarrierDim];
    std::memcpy(z1, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) v1[i] = alpha * tc.d[i];
    conex::primalMidpointStep(tc.ops, z1, v1, 1.0, n);
    double err1 = dist(z1, ref, n);

    // 2 steps of h=0.5.
    double z2[conex::kMaxBarrierDim], v2[conex::kMaxBarrierDim];
    std::memcpy(z2, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) v2[i] = alpha * tc.d[i];
    conex::primalMidpointStep(tc.ops, z2, v2, 0.5, n);
    conex::primalMidpointStep(tc.ops, z2, v2, 0.5, n);
    double err2 = dist(z2, ref, n);

    double ratio = err1 / std::max(err2, 1e-30);
    printf("  %s: err(h=1)=%.2e, err(h=0.5)=%.2e, ratio=%.1f (expect ~4)\n",
           tc.name, err1, err2, ratio);

    // Second-order: ratio should be roughly 4 (= 2²).
    EXPECT_GT(ratio, 2.0) << tc.name << ": convergence ratio too low";
  }
}

// ============================================================
// Time reversibility: forward then backward recovers original
// ============================================================
TEST(BarrierIntegrators, Reversibility) {
  auto cases = makeCases();

  for (const auto& tc : cases) {
    int n = tc.z0.size();
    double alpha = 0.2;

    double z[conex::kMaxBarrierDim], vel[conex::kMaxBarrierDim];
    std::memcpy(z, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel[i] = alpha * tc.d[i];

    // Forward step.
    conex::primalMidpointStep(tc.ops, z, vel, 1.0, n);
    // Negate velocity.
    for (int i = 0; i < n; ++i) vel[i] = -vel[i];
    // Backward step.
    conex::primalMidpointStep(tc.ops, z, vel, 1.0, n);

    double err = dist(z, tc.z0.data(), n);
    printf("  %s: reversibility error = %.2e\n", tc.name, err);
    EXPECT_LT(err, 1e-8) << tc.name << ": not reversible";
  }
}

// ============================================================
// Energy conservation: ||vel||²_H should be approximately constant
// ============================================================
TEST(BarrierIntegrators, EnergyConservation) {
  auto cases = makeCases();

  for (const auto& tc : cases) {
    int n = tc.z0.size();
    double alpha = 0.3;

    double z[conex::kMaxBarrierDim], vel[conex::kMaxBarrierDim];
    std::memcpy(z, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel[i] = alpha * tc.d[i];

    double E0 = energy(tc.ops, z, vel, n);

    // Take 10 steps, track energy.
    double E_max = E0, E_min = E0;
    for (int s = 0; s < 10; ++s) {
      conex::primalMidpointStep(tc.ops, z, vel, 0.1, n);
      double E = energy(tc.ops, z, vel, n);
      E_max = std::max(E_max, E);
      E_min = std::min(E_min, E);
    }

    double rel_var = (E_max - E_min) / E0;
    printf("  %s: E0=%.4e, variation=%.2e\n", tc.name, E0, rel_var);
    EXPECT_LT(rel_var, 0.1) << tc.name << ": energy not conserved";
  }
}

// ============================================================
// Exact reference for exp cone using analytic ThirdDerivContract
// ============================================================

// Fine-grid Störmer-Verlet using ExpConeOps::ThirdDerivContract (exact).
static void expConeExactReference(const double* z0, double alpha,
                                   const double* d, double* out,
                                   int steps = 10000) {
  ExpConeOps ops;
  const int n = 3;
  double pos[3] = {z0[0], z0[1], z0[2]};
  double vel[3] = {alpha * d[0], alpha * d[1], alpha * d[2]};
  double dt = 1.0 / steps;
  for (int s = 0; s < steps; ++s) {
    auto kick = [&]() {
      double T[3], H_buf[9];
      ops.thirdDerivContract(T, pos, vel, n);
      ops.hessian(H_buf, pos, n);
      Eigen::Map<Eigen::MatrixXd> H(H_buf, 3, 3);
      Eigen::Map<Eigen::VectorXd> Tv(T, 3);
      Eigen::Vector3d a = H.ldlt().solve(-0.5 * Tv);
      for (int i = 0; i < 3; ++i) vel[i] += 0.5 * dt * a(i);
    };
    kick();
    for (int i = 0; i < 3; ++i) pos[i] += dt * vel[i];
    kick();
  }
  for (int i = 0; i < 3; ++i) out[i] = pos[i];
}

TEST(BarrierIntegrators, ExpCone_ExactThirdDerivReference) {
  const int n = 3;
  double z0[3] = {0.1, 1.0, 2.5};
  double d[3] = {0.2, -0.1, 0.15};
  double alpha = 0.3;

  // Exact reference: 10000-step Verlet with analytic third derivative.
  double ref_exact[3];
  expConeExactReference(z0, alpha, d, ref_exact);

  // Finite-diff reference (used by other tests).
  double ref_fd[3];
  fineGridReference(&g_exp_ops, z0, alpha, d, ref_fd, n);

  // They should agree closely (finite-diff error ≈ 1e-10 × 10000 steps).
  double ref_diff = dist(ref_exact, ref_fd, n);
  printf("\n  Exact vs finite-diff reference: %.2e\n", ref_diff);
  EXPECT_LT(ref_diff, 1e-6) << "references disagree";

  // Now test each integrator against the exact reference.
  auto test = [&](const char* name, auto step_fn) {
    double z[3], vel[3];
    std::memcpy(z, z0, sizeof(z));
    for (int i = 0; i < n; ++i) vel[i] = alpha * d[i];
    step_fn(z, vel);
    double err = dist(z, ref_exact, n);
    printf("  %s: %.4e\n", name, err);
    return err;
  };

  double err_sym = test("Symmetric", [&](double* z, double* v) {
    conex::symmetricStep(&g_exp_ops, z, v, 1.0, n);
  });
  double err_pm = test("Primal-mid", [&](double* z, double* v) {
    conex::primalMidpointStep(&g_exp_ops, z, v, 1.0, n);
  });
  double err_dm = test("Dual-mid", [&](double* z, double* v) {
    conex::dualMidpointStep(&g_exp_ops, z, v, 1.0, n);
  });
  double err_y4 = test("Yoshida-4", [&](double* z, double* v) {
    conex::yoshida4Step(&g_exp_ops, z, v, 1.0, n);
  });
  double z_eu[3] = {z0[0]+alpha*d[0], z0[1]+alpha*d[1], z0[2]+alpha*d[2]};
  double err_eu = dist(z_eu, ref_exact, n);
  printf("  Euler:       %.4e\n", err_eu);

  // Verify Yoshida-4 is 4th order: compare h=1 vs h=0.5 (ratio ≈ 16).
  double z_h1[3], v_h1[3], z_h05[3], v_h05[3];
  std::memcpy(z_h1, z0, sizeof(z_h1));
  std::memcpy(z_h05, z0, sizeof(z_h05));
  for (int i = 0; i < n; ++i) v_h1[i] = v_h05[i] = alpha * d[i];
  conex::yoshida4Step(&g_exp_ops, z_h1, v_h1, 1.0, n);
  conex::yoshida4Step(&g_exp_ops, z_h05, v_h05, 0.5, n);
  conex::yoshida4Step(&g_exp_ops, z_h05, v_h05, 0.5, n);
  double err_y4_h1 = dist(z_h1, ref_exact, n);
  double err_y4_h05 = dist(z_h05, ref_exact, n);
  double ratio = err_y4_h1 / std::max(err_y4_h05, 1e-30);
  printf("  Yoshida-4 convergence: h=1 err=%.2e, h=0.5 err=%.2e, ratio=%.1f (expect ~16)\n",
         err_y4_h1, err_y4_h05, ratio);
  EXPECT_GT(ratio, 8.0) << "Yoshida-4 should be 4th order (ratio ≈ 16)";

  // All integrators should beat Euler.
  EXPECT_LT(err_sym, err_eu);
  EXPECT_LT(err_pm, err_eu);
  EXPECT_LT(err_dm, err_eu);
  EXPECT_LT(err_y4, err_eu);
  EXPECT_LT(err_y4, err_sym) << "Yoshida-4 should beat symmetric";
}
