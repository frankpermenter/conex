// Geodesic integrators for log-homogeneous barrier cones.
// Stack-allocated workspace, max segment size = kMaxBarrierDim.

#include "conex/common/barrier_integrators.h"
#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <cstring>

namespace conex {

using Ops = EuclideanJordanAlgebra::BarrierConeOperations;

// Helper: solve ∇φ(z) = target for z via Newton iteration.
// Uses ops->computeGradient and ops->hessian.
// z is initial guess on input, solution on output.
static void invertGradient(const Ops* ops, double* z,
                           const double* target, int size,
                           double* grad_buf, double* H_buf, double* delta_buf) {
  Eigen::Map<Eigen::VectorXd> zv(z, size);
  Eigen::Map<const Eigen::VectorXd> tv(target, size);
  Eigen::Map<Eigen::VectorXd> gv(grad_buf, size);
  Eigen::Map<Eigen::MatrixXd> Hm(H_buf, size, size);
  Eigen::Map<Eigen::VectorXd> dv(delta_buf, size);

  for (int iter = 0; iter < 20; ++iter) {
    ops->computeGradient(grad_buf, z, size);
    gv -= tv;  // residual = ∇φ(z) - target

    if (gv.norm() < 1e-12 * (1.0 + tv.norm())) break;

    ops->hessian(H_buf, z, size);
    dv = Hm.ldlt().solve(-gv);

    // Backtracking: ensure z + delta is interior.
    double alpha = 1.0;
    double z_save[kMaxBarrierDim];
    std::memcpy(z_save, z, size * sizeof(double));
    for (int bt = 0; bt < 10; ++bt) {
      for (int i = 0; i < size; ++i) z[i] = z_save[i] + alpha * dv(i);
      if (ops->isInterior(z, size)) break;
      alpha *= 0.5;
    }
  }
}

// ============================================================
// Symmetric step (Algorithm 1 in integrator.tex)
// ============================================================
//
// Solve for z₁:
//   ∇φ(z₁) + H(z₀)·z₁ = 2h·H(z₀)·v₀
//
// Equivalently: find z₁ such that F(z₁) = ∇φ(z₁) + H(z₀)·z₁ - rhs = 0
// where rhs = H(z₀)·(z₀ + 2h·v₀)  [using log-homogeneity: ∇φ(z₀) = -H(z₀)·z₀]
//
// Newton: J = H(z₁) + H(z₀), step = -J⁻¹ F(z₁).
//
// Velocity update:
//   w₁ = [(∇φ(z₁) - ∇φ(z₀)) + H(z₁)·(z₁ - z₀)] / (2h)
//   v₁ = H(z₁)⁻¹ · w₁

// Single substep of the symmetric integrator (no adaptive subdivision).
static void symmetricSubstep(const Ops* ops,
                              double* z, double* vel, double h, int size);

void symmetricStep(const Ops* ops,
                   double* z, double* vel, double h, int size) {
  assert(size <= kMaxBarrierDim);

  // Adaptive substeps: subdivide if Riemannian speed is large.
  double Hv[kMaxBarrierDim];
  ops->hessianProduct(Hv, z, vel, size);
  double speed_sq = 0;
  for (int i = 0; i < size; ++i) speed_sq += vel[i] * Hv[i];
  double speed = std::sqrt(std::max(speed_sq, 0.0)) * std::abs(h);
  int steps = std::max(1, (int)std::ceil(speed));
  double sub_h = h / steps;

  for (int s = 0; s < steps; ++s)
    symmetricSubstep(ops, z, vel, sub_h, size);
}

static void symmetricSubstep(const Ops* ops,
                              double* z, double* vel, double h, int size) {
  double grad0_buf[kMaxBarrierDim];
  double grad1_buf[kMaxBarrierDim];
  double H0_buf[kMaxBarrierDim * kMaxBarrierDim];
  double H1_buf[kMaxBarrierDim * kMaxBarrierDim];
  double rhs_buf[kMaxBarrierDim];
  double F_buf[kMaxBarrierDim];
  double z0_buf[kMaxBarrierDim];
  double z1_buf[kMaxBarrierDim];

  const int n = size;
  std::memcpy(z0_buf, z, n * sizeof(double));

  Eigen::Map<Eigen::VectorXd> z0(z0_buf, n);
  Eigen::Map<Eigen::VectorXd> z1(z1_buf, n);
  Eigen::Map<Eigen::VectorXd> v(vel, n);
  Eigen::Map<Eigen::VectorXd> g0(grad0_buf, n);
  Eigen::Map<Eigen::VectorXd> g1(grad1_buf, n);
  Eigen::Map<Eigen::MatrixXd> H0(H0_buf, n, n);
  Eigen::Map<Eigen::MatrixXd> H1(H1_buf, n, n);
  Eigen::Map<Eigen::VectorXd> rhs(rhs_buf, n);
  Eigen::Map<Eigen::VectorXd> F(F_buf, n);

  // Compute H(z₀) and ∇φ(z₀).
  ops->hessian(H0_buf, z0_buf, n);
  ops->computeGradient(grad0_buf, z0_buf, n);

  // From integrator.tex equation (2):
  //   ∇φ(z₁) - ∇φ(z₀) = H₀·(z₁ - z₀) + 2h·w₀
  //
  // Solved as: ∇φ(z₁) + H₀·z₁ = target, where
  //   target = ∇φ(z₀) + H₀·z₀ + 2h·H₀·v₀
  // By log-homogeneity ∇φ(z₀) + H₀·z₀ = 0, so target = 2h·H₀·v₀.
  // Jacobian: H(z₁) + H₀ (positive definite).
  rhs = 2.0 * h * (H0 * v);

  // Initial guess: z_pred = z₀ + h·v₀. If infeasible, backtrack toward z₀.
  z1 = z0 + h * v;
  if (!ops->isInterior(z1_buf, n)) {
    double t = 1.0;
    for (int bt = 0; bt < 30; ++bt) {
      t *= 0.5;
      z1 = z0 + t * h * v;
      if (ops->isInterior(z1_buf, n)) break;
    }
  }

  for (int iter = 0; iter < 30; ++iter) {
    ops->computeGradient(grad1_buf, z1_buf, n);
    F = g1 + H0 * z1 - rhs;

    if (F.norm() < 1e-13 * (1.0 + rhs.norm())) break;

    ops->hessian(H1_buf, z1_buf, n);
    Eigen::MatrixXd J = H1 + H0;

    Eigen::VectorXd delta = J.ldlt().solve(-F);

    // Backtracking with proper feasibility check.
    double alpha = 1.0;
    Eigen::VectorXd z1_save = z1;
    bool found = false;
    for (int bt = 0; bt < 20; ++bt) {
      z1 = z1_save + alpha * delta;
      if (ops->isInterior(z1_buf, n)) { found = true; break; }
      alpha *= 0.5;
    }
    if (!found) {
      // Newton direction doesn't lead to interior. Step toward z₀.
      z1 = 0.5 * (z1_save + z0);
    }
  }

  // Velocity update:
  //   w₁ = [(∇φ(z₁) - ∇φ(z₀)) + H(z₁)·(z₁ - z₀)] / (2h)
  //   v₁ = H(z₁)⁻¹ · w₁
  ops->computeGradient(grad1_buf, z1_buf, n);
  ops->hessian(H1_buf, z1_buf, n);

  Eigen::VectorXd w1 = ((g1 - g0) + H1 * (z1 - z0)) / (2.0 * h);
  v = H1.ldlt().solve(w1);

  // Write output.
  std::memcpy(z, z1_buf, n * sizeof(double));
}

// ============================================================
// Primal midpoint step
// ============================================================
//
// z½ = z₀ + (h/2)·v₀
// λ₁ = 2·∇φ(z½) - ∇φ(z₀)
// z₁ = (∇φ)⁻¹(λ₁)
// v₁ = (2/h)·(z₁ - z½)

void primalMidpointStep(const Ops* ops,
                        double* z, double* vel, double h, int size) {
  assert(size <= kMaxBarrierDim);

  double z0_buf[kMaxBarrierDim];
  double zhalf_buf[kMaxBarrierDim];
  double grad0_buf[kMaxBarrierDim];
  double gradhalf_buf[kMaxBarrierDim];
  double lambda1_buf[kMaxBarrierDim];
  double H_buf[kMaxBarrierDim * kMaxBarrierDim];
  double delta_buf[kMaxBarrierDim];

  const int n = size;
  std::memcpy(z0_buf, z, n * sizeof(double));

  Eigen::Map<Eigen::VectorXd> z0(z0_buf, n);
  Eigen::Map<Eigen::VectorXd> zhalf(zhalf_buf, n);
  Eigen::Map<Eigen::VectorXd> zv(z, n);
  Eigen::Map<Eigen::VectorXd> v(vel, n);
  Eigen::Map<Eigen::VectorXd> g0(grad0_buf, n);
  Eigen::Map<Eigen::VectorXd> ghalf(gradhalf_buf, n);
  Eigen::Map<Eigen::VectorXd> lam1(lambda1_buf, n);

  // z½ = z₀ + (h/2)·v₀.
  zhalf = z0 + (h / 2.0) * v;

  // ∇φ(z₀) and ∇φ(z½).
  ops->computeGradient(grad0_buf, z0_buf, n);
  ops->computeGradient(gradhalf_buf, zhalf_buf, n);

  // λ₁ = 2·∇φ(z½) - ∇φ(z₀).
  lam1 = 2.0 * ghalf - g0;

  // z₁ = (∇φ)⁻¹(λ₁): Newton solve ∇φ(z) = λ₁, starting from z½.
  std::memcpy(z, zhalf_buf, n * sizeof(double));
  invertGradient(ops, z, lambda1_buf, n, gradhalf_buf, H_buf, delta_buf);

  // v₁ = (2/h)·(z₁ - z½).  [from integrator.tex eq. (4)]
  v = (2.0 / h) * (zv - zhalf);
}

// ============================================================
// Dual midpoint step
// ============================================================
//
// w₀ = H(z₀)·v₀
// λ½ = ∇φ(z₀) + (h/2)·w₀
// z₁ = 2·(∇φ)⁻¹(λ½) - z₀
// w₁ = (2/h)·(∇φ(z₁) - λ½)
// v₁ = H(z₁)⁻¹ · w₁

void dualMidpointStep(const Ops* ops,
                      double* z, double* vel, double h, int size) {
  assert(size <= kMaxBarrierDim);

  double z0_buf[kMaxBarrierDim];
  double grad0_buf[kMaxBarrierDim];
  double w0_buf[kMaxBarrierDim];
  double lamhalf_buf[kMaxBarrierDim];
  double zstar_buf[kMaxBarrierDim];
  double grad1_buf[kMaxBarrierDim];
  double H_buf[kMaxBarrierDim * kMaxBarrierDim];
  double delta_buf[kMaxBarrierDim];

  const int n = size;
  std::memcpy(z0_buf, z, n * sizeof(double));

  Eigen::Map<Eigen::VectorXd> z0(z0_buf, n);
  Eigen::Map<Eigen::VectorXd> zv(z, n);
  Eigen::Map<Eigen::VectorXd> v(vel, n);
  Eigen::Map<Eigen::VectorXd> g0(grad0_buf, n);
  Eigen::Map<Eigen::VectorXd> w0(w0_buf, n);
  Eigen::Map<Eigen::VectorXd> lamhalf(lamhalf_buf, n);
  Eigen::Map<Eigen::VectorXd> zstar(zstar_buf, n);
  Eigen::Map<Eigen::VectorXd> g1(grad1_buf, n);
  Eigen::Map<Eigen::MatrixXd> H1(H_buf, n, n);

  // w₀ = H(z₀)·v₀.
  ops->hessianProduct(w0_buf, z0_buf, vel, n);

  // λ½ = ∇φ(z₀) + (h/2)·w₀.
  ops->computeGradient(grad0_buf, z0_buf, n);
  lamhalf = g0 + (h / 2.0) * w0;

  // (∇φ)⁻¹(λ½): Newton solve, starting from z₀.
  std::memcpy(zstar_buf, z0_buf, n * sizeof(double));
  invertGradient(ops, zstar_buf, lamhalf_buf, n, grad1_buf, H_buf, delta_buf);

  // z₁ = 2·(∇φ)⁻¹(λ½) - z₀.
  zv = 2.0 * zstar - z0;

  // w₁ = (2/h)·(∇φ(z₁) - λ½).  [from integrator.tex Definition 3]
  ops->computeGradient(grad1_buf, z, n);
  Eigen::VectorXd w1 = (2.0 / h) * (g1 - lamhalf);

  // v₁ = H(z₁)⁻¹ · w₁.
  ops->hessian(H_buf, z, n);
  v = H1.ldlt().solve(w1);
}

// ============================================================
// Yoshida 4th-order composition
// ============================================================
//
// Ψ_h^(4) = Ψ_{w₁h} ∘ Ψ_{w₀h} ∘ Ψ_{w₁h}
// w₁ = 1/(2 - 2^{1/3}), w₀ = 1 - 2·w₁

void yoshida4Step(const Ops* ops,
                  double* z, double* vel, double h, int size) {
  const double cbrt2 = std::cbrt(2.0);
  const double w1 = 1.0 / (2.0 - cbrt2);
  const double w0 = 1.0 - 2.0 * w1;

  symmetricStep(ops, z, vel, w1 * h, size);
  symmetricStep(ops, z, vel, w0 * h, size);
  symmetricStep(ops, z, vel, w1 * h, size);
}

void yoshida4PrimalStep(const Ops* ops,
                        double* z, double* vel, double h, int size) {
  const double cbrt2 = std::cbrt(2.0);
  const double w1 = 1.0 / (2.0 - cbrt2);
  const double w0 = 1.0 - 2.0 * w1;

  primalMidpointStep(ops, z, vel, w1 * h, size);
  primalMidpointStep(ops, z, vel, w0 * h, size);
  primalMidpointStep(ops, z, vel, w1 * h, size);
}

void yoshida4DualStep(const Ops* ops,
                      double* z, double* vel, double h, int size) {
  const double cbrt2 = std::cbrt(2.0);
  const double w1 = 1.0 / (2.0 - cbrt2);
  const double w0 = 1.0 - 2.0 * w1;

  dualMidpointStep(ops, z, vel, w1 * h, size);
  dualMidpointStep(ops, z, vel, w0 * h, size);
  dualMidpointStep(ops, z, vel, w1 * h, size);
}

}  // namespace conex
