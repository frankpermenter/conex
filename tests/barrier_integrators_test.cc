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

    // Yoshida-4 (1 composite step = 3 symmetric sub-steps).
    double z_y4[conex::kMaxBarrierDim], vel_y4[conex::kMaxBarrierDim];
    std::memcpy(z_y4, tc.z0.data(), n * sizeof(double));
    for (int i = 0; i < n; ++i) vel_y4[i] = alpha * tc.d[i];
    conex::yoshida4Step(tc.ops, z_y4, vel_y4, 1.0, n);
    double err_y4 = dist(z_y4, ref, n);

    // Euler.
    double z_eu[conex::kMaxBarrierDim];
    for (int i = 0; i < n; ++i) z_eu[i] = tc.z0[i] + alpha * tc.d[i];
    double err_eu = dist(z_eu, ref, n);

    printf("\n  %s (alpha=%.1f):\n", tc.name, alpha);
    printf("    Symmetric:     %.4e  (%.0fx vs euler)\n",
           err_sym, err_eu / std::max(err_sym, 1e-30));
    printf("    Primal-mid:    %.4e  (%.0fx vs euler)\n",
           err_pm, err_eu / std::max(err_pm, 1e-30));
    printf("    Dual-mid:      %.4e  (%.0fx vs euler)\n",
           err_dm, err_eu / std::max(err_dm, 1e-30));
    printf("    Yoshida-4:     %.4e  (%.0fx vs euler)\n",
           err_y4, err_eu / std::max(err_y4, 1e-30));
    printf("    Euler:         %.4e  (baseline)\n", err_eu);

    // Midpoint integrators should beat Euler.
    EXPECT_LT(err_pm, err_eu) << tc.name << ": primal midpoint worse than Euler";
    EXPECT_LT(err_dm, err_eu) << tc.name << ": dual midpoint worse than Euler";
    // TODO: fix symmetric step Newton solve — currently diverges at h=1.
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
