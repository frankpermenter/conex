// Cone operations interfaces for the geodesic IPM.
//
// BarrierConeOperations: lean base class for any cone with a log-homogeneous
//   barrier.  Provides z-space operations (gradient, Hessian, geodesic step,
//   line search) used by SolveGeodesicBarrierLP and ThetaContinuation.
//
// SymmetricConeOperations: extends BarrierConeOperations with Jordan algebra
//   operations (product, sqrt, quadratic representation, automorphisms) used
//   by the W-space algorithms on nonneg, SOC, and PSD cones.

#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace conex {
namespace EuclideanJordanAlgebra {

// Base class for any cone with a computable log-homogeneous barrier.
//
// Required: computeGradient, hessianProduct, barrierParameter.
// Everything else has working defaults built from these primitives.
//
// Hierarchy:
//   BarrierConeOperations           — primitives + Bregman geodesic step
//   ├── BarrierConeOpsThirdDeriv    — adds thirdDerivContract, Verlet step
//   │   └── ExpConeOps, etc.
//   └── SymmetricConeOperations     — Jordan algebra, exact geodesic
//       └── NonnegOrthantOps, PSDConeOps, SOCConeOps
class BarrierConeOperations {
 public:
  virtual ~BarrierConeOperations() = default;

  // --- Required primitives ---

  // Gradient: grad = ∇F(z).
  virtual void computeGradient(double* grad, const double* z,
                               int size) const = 0;

  // Hessian-vector product: out = H(z) · v.
  virtual void hessianProduct(double* out, const double* z,
                              const double* v, int size) const = 0;

  // Barrier parameter ν for a cone of this dimension.
  virtual double barrierParameter(int size) const = 0;

  // An interior point of the cone (e.g., identity element for symmetric
  // cones). Used as z₀ for theta-continuation and initialization.
  virtual void getInteriorPoint(double* out, int size) const = 0;

  // --- Defaults built from primitives (override for performance) ---

  // Full Hessian matrix: out = H(z), column-major, size × size.
  virtual void hessian(double* out, const double* z, int size) const {
    std::vector<double> ei(size, 0.0);
    for (int j = 0; j < size; ++j) {
      ei[j] = 1.0;
      hessianProduct(out + j * size, z, ei.data(), size);
      ei[j] = 0.0;
    }
  }

  // Squared Hessian norm: ||target - z||²_{H(z)}.
  virtual double hessianNormSquared(const double* z, const double* target,
                                    int size) const {
    // d = target - z, return d^T H(z) d.
    std::vector<double> d(size), Hd(size);
    for (int i = 0; i < size; ++i) d[i] = target[i] - z[i];
    hessianProduct(Hd.data(), z, d.data(), size);
    double result = 0;
    for (int i = 0; i < size; ++i) result += d[i] * Hd[i];
    return result;
  }

  // Step size: α = min(1, 2/||target - z||²_{H,∞}).
  // Default uses the Hessian norm squared (conservative).
  virtual double stepSize(const double* z, const double* target,
                          int size) const {
    double d_sq = hessianNormSquared(z, target, size);
    if (d_sq <= 2.0) return 1.0;
    return 2.0 / d_sq;
  }

  // Geodesic step: z ← Exp_z(α(target - z)).
  // Default: primal midpoint (second-order, time-reversible, no third
  // derivatives needed). Override with exact geodesic (symmetric cones)
  // or Verlet (BarrierConeOpsThirdDeriv).
  virtual void geodesicStepTarget(double* z, double alpha,
                                  const double* target, int size) const {
    defaultGeodesicStep(z, alpha, target, size);
  }

  // Non-virtual helper. Defined in barrier_cone_operations.cc.
  void defaultGeodesicStep(double* z, double alpha,
                           const double* target, int size) const;

  // Line search: max k with ||target0 + k·target1 - z||²_{H(z)} ≤ 1.
  // Quadratic in k: a + 2fk + pk² ≤ 1.
  virtual double lineSearchTarget(const double* z, const double* target0,
                                  const double* target1, int size) const {
    // Compute inner products via hessianProduct.
    std::vector<double> d0(size), d1(size), Hd0(size), Hd1(size);
    for (int i = 0; i < size; ++i) {
      d0[i] = target0[i] - z[i];
      d1[i] = target1[i];
    }
    hessianProduct(Hd0.data(), z, d0.data(), size);
    hessianProduct(Hd1.data(), z, d1.data(), size);
    double a = 0, f = 0, p = 0;
    for (int i = 0; i < size; ++i) {
      a += d0[i] * Hd0[i];
      f += d0[i] * Hd1[i];
      p += d1[i] * Hd1[i];
    }
    double disc = 4*f*f - 4*p*(a - 1);
    if (disc < 0 || p < 1e-30) return 0;
    double k1 = (-2*f + std::sqrt(disc)) / (2*p);
    double k2 = (-2*f - std::sqrt(disc)) / (2*p);
    return std::max(std::max(k1, k2), 0.0);
  }

  // Recover the raw cone point z from the stored representation.
  // Identity for barrier cones (stored = raw). Override for symmetric
  // cones where stored = W = -∇F(z), so raw = W^{-1}.
  virtual void getConePoint(double* out, const double* stored,
                            int size) const {
    for (int i = 0; i < size; ++i) out[i] = stored[i];
  }

  // Barrier function value F(z). Required for default isInterior.
  virtual double barrierValue(const double* z, int size) const = 0;

  // Interior check: returns true if z ∈ int(K).
  // Default: barrier value is finite.
  virtual bool isInterior(const double* z, int size) const {
    return std::isfinite(barrierValue(z, size));
  }

  // Inner product <a, b>.  Default: Euclidean dot product.
  // Symmetric cones override with trace inner product.
  virtual double dot(const double* a, const double* b, int size) const {
    double r = 0; for (int i = 0; i < size; ++i) r += a[i]*b[i]; return r;
  }

  // ||a||^2.  Default: Euclidean squared norm.
  virtual double squaredNorm(const double* a, int size) const {
    return dot(a, a, size);
  }

};

// Extended: cones with an analytic third-derivative contraction.
// Default geodesicStepTarget upgrades from Euler to Störmer-Verlet.
class BarrierConeOpsThirdDeriv : public BarrierConeOperations {
 public:
  // Third-derivative contraction: out_l = v^T (∂H/∂z_l) v.
  virtual void thirdDerivContract(double* out, const double* z,
                                  const double* v, int size) const = 0;

  // Default: Störmer-Verlet geodesic integrator using thirdDerivContract.
  void geodesicStepTarget(double* z, double alpha,
                          const double* target, int size) const override;
};

// Extended interface for symmetric cones (nonneg, SOC, PSD).
// Adds Jordan algebra operations, automorphism tracking, and
// eigenvalue-based norms used by the W-space algorithms.
class SymmetricConeOperations : public BarrierConeOperations {
 public:
  // Element-wise product (Jordan product).
  virtual void product(double* out, const double* a, const double* b,
                       int size) const = 0;

  // Geodesic update: out_i = a_i * exp(alpha * d_i).
  virtual void geodesicUpdate(double* out, const double* a, double alpha,
                              const double* d, int size) const = 0;

  // Set to identity element.
  virtual void setIdentity(double* out, int size) const = 0;

  // ||a||_inf (max eigenvalue magnitude).
  virtual double normInf(const double* a, int size) const = 0;

  // ||a||^2 (Frobenius for SDP).
  virtual double squaredNorm(const double* a, int size) const = 0;

  // <a, b> (trace inner product).
  virtual double dot(const double* a, const double* b, int size) const = 0;

  // Symmetric square root.
  virtual void sqrt(double* out, const double* a, int size) const = 0;

  // Jordan algebra inverse: a^{-1} such that a ∘ a^{-1} = e.
  virtual void inverse(double* out, const double* a, int size) const = 0;

  // Quadratic representation: P(a)b.
  virtual void quadraticRepresentation(double* out, const double* a,
                                       const double* b, int size) const = 0;

  // Solve Lyapunov R*D + D*R = 2*Delta for D.
  virtual void solveLyapunovForD(double* out, const double* r,
                                 const double* delta, int size) const = 0;

  // Absolute value in the EJA sense.
  virtual void abs(double* out, const double* a, int size) const = 0;

  // Minimum eigenvalue.
  virtual double minEigenvalue(const double* a, int size) const = 0;

  // Automorphism updates.
  virtual void updateAutomorphism(double* w, double* r, double alpha,
                                  const double* d, int size) const = 0;
  virtual void updateAutomorphismP(double* p, double* r, double alpha,
                                   const double* d, int size) const = 0;
  virtual void updateM(double* m, double* r, double alpha,
                       const double* d, int size) const = 0;
  virtual void applyM(double* out, const double* m,
                      const double* x, int size) const = 0;
  virtual void applyMt(double* out, const double* m,
                       const double* x, int size) const = 0;
  virtual void squareM(double* w, const double* m, int size) const = 0;

  // Line search: largest k > 0 with ||d0 + k*d1||_inf <= 1.
  virtual double lineSearchK(const double* d0, const double* d1,
                             int size) const = 0;

  // Project onto the cone.
  virtual void project(double* out, const double* a, int size) const = 0;

  // Geodesic update from raw slack (sqrt-free).
  virtual void geodesicUpdateFromSlack(double* W_out, const double* W,
                                       double alpha, const double* slack,
                                       int size) const {
    std::vector<double> sqW(size), d(size), ones(size);
    sqrt(sqW.data(), W, size);
    quadraticRepresentation(d.data(), sqW.data(), slack, size);
    setIdentity(ones.data(), size);
    for (int i = 0; i < size; ++i) d[i] += ones[i];
    geodesicUpdate(W_out, W, alpha, d.data(), size);
  }

  // --- Default z-space implementations for symmetric cones ---
  // These use the Jordan algebra: d = e - P(W, target), where W = z.
  // Override in NonnegOrthantOps for elementwise (faster) versions.

  // Stored W = -∇F(z_primal), so z_primal = W^{-1} (Jordan algebra inverse).
  void getConePoint(double* out, const double* stored,
                    int size) const override {
    inverse(out, stored, size);
  }

  // Interior point: the identity element (W = e).
  void getInteriorPoint(double* out, int size) const override {
    setIdentity(out, size);
  }

  // ∇F(z) = -W (since z stores W = -∇F(z_primal)).
  void computeGradient(double* grad, const double* z,
                       int size) const override {
    for (int i = 0; i < size; ++i) grad[i] = -z[i];
  }

  // H(z) v = P(W) v (quadratic representation).
  void hessianProduct(double* out, const double* z,
                      const double* v, int size) const override {
    quadraticRepresentation(out, z, v, size);
  }

  // Convert target to d-space: d = e - P(W^{1/2})(target).
  // For nonneg: d_i = 1 - W_i * target_i (product commutes).
  // For PSD: d = I - W^{1/2} target W^{1/2} (quadratic representation).
  double hessianNormSquared(const double* z, const double* target,
                            int size) const override {
    std::vector<double> sqW(size), d(size), e(size);
    sqrt(sqW.data(), z, size);
    quadraticRepresentation(d.data(), sqW.data(), target, size);
    setIdentity(e.data(), size);
    for (int i = 0; i < size; ++i) d[i] = e[i] - d[i];
    return squaredNorm(d.data(), size);
  }

  double stepSize(const double* z, const double* target,
                  int size) const override {
    std::vector<double> sqW(size), d(size), e(size);
    sqrt(sqW.data(), z, size);
    quadraticRepresentation(d.data(), sqW.data(), target, size);
    setIdentity(e.data(), size);
    for (int i = 0; i < size; ++i) d[i] = e[i] - d[i];
    double d_inf = normInf(d.data(), size);
    return std::min(1.0, 2.0 / (d_inf * d_inf));
  }

  void geodesicStepTarget(double* z, double alpha,
                          const double* target, int size) const override {
    std::vector<double> sqW(size), d(size), e(size);
    sqrt(sqW.data(), z, size);
    quadraticRepresentation(d.data(), sqW.data(), target, size);
    setIdentity(e.data(), size);
    for (int i = 0; i < size; ++i) d[i] = e[i] - d[i];
    geodesicUpdate(z, z, alpha, d.data(), size);
  }

  double lineSearchTarget(const double* z, const double* target0,
                          const double* target1, int size) const override {
    std::vector<double> sqW(size), d0(size), d1(size), e(size);
    sqrt(sqW.data(), z, size);
    quadraticRepresentation(d0.data(), sqW.data(), target0, size);
    setIdentity(e.data(), size);
    for (int i = 0; i < size; ++i) d0[i] = e[i] - d0[i];
    quadraticRepresentation(d1.data(), sqW.data(), target1, size);
    for (int i = 0; i < size; ++i) d1[i] = -d1[i];
    return lineSearchK(d0.data(), d1.data(), size);
  }

  // Barrier parameter = rank of the Jordan algebra.
  // Nonneg: ν = n.  PSD: ν = n (matrix dim).  SOC: ν = 2.
  // Default: ν = n for nonneg/PSD. SOC overrides.
  double barrierParameter(int size) const override {
    return static_cast<double>(size);
  }

  // Barrier value: F(W) = -log det(W) for symmetric cones.
  // For nonneg: -Σ log(W_i).  For PSD: -log det(W).  For SOC: -log(w₀²-||w₁||²).
  // Default implementation uses eigenvalue decomposition via minEigenvalue
  // as a stub — concrete classes should override for efficiency.
  // For nonneg (stored as vector): -Σ log(w_i).
  double barrierValue(const double* z, int size) const override {
    // Generic: -Σ log(w_i) works for nonneg. PSD and SOC override.
    double val = 0;
    for (int i = 0; i < size; ++i) {
      if (z[i] <= 0) return std::numeric_limits<double>::infinity();
      val -= std::log(z[i]);
    }
    return val;
  }
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
