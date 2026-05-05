// Per-segment cone operations interface.
// Each cone type (nonneg orthant, PSD, SOCP, ...) implements this.

#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace conex {
namespace EuclideanJordanAlgebra {

class SymmetricConeOperations {
 public:
  virtual ~SymmetricConeOperations() = default;

  // Element-wise product (Jordan product).
  virtual void product(double* out, const double* a, const double* b,
                       int size) const = 0;

  // Geodesic update: out_i = a_i * exp(alpha * d_i).
  virtual void geodesicUpdate(double* out, const double* a, double alpha,
                              const double* d, int size) const = 0;

  // Set to identity element.
  virtual void setIdentity(double* out, int size) const = 0;

  // ||a||_inf (max eigenvalue magnitude for SDP).
  virtual double normInf(const double* a, int size) const = 0;

  // ||a||^2 (Frobenius for SDP).
  virtual double squaredNorm(const double* a, int size) const = 0;

  // <a, b> (trace inner product for SDP).
  virtual double dot(const double* a, const double* b, int size) const = 0;

  // Symmetric square root (elementwise for nonneg, matrix sqrt for PSD).
  virtual void sqrt(double* out, const double* a, int size) const = 0;

  // Quadratic representation: P(a)b = a * b * a.
  //   Nonneg: out_i = a_i² * b_i.
  //   PSD:    Out = A * B * A.
  virtual void quadraticRepresentation(double* out, const double* a,
                                       const double* b, int size) const = 0;

  // Solve Lyapunov R*D + D*R = 2*Delta for D (given R and Delta).
  //   Nonneg: d_i = delta_i / r_i.
  //   PSD: eigendecompose R, D_ij = 2*Delta_ij / (l_i + l_j) in eigenbasis.
  virtual void solveLyapunovForD(double* out, const double* r,
                                 const double* delta, int size) const = 0;

  // Absolute value in the EJA sense.
  //   Nonneg: out_i = |a_i|.
  //   PSD:    eigendecompose, abs eigenvalues, reconstruct.
  virtual void abs(double* out, const double* a, int size) const = 0;

  // Minimum eigenvalue (min element for nonneg).
  virtual double minEigenvalue(const double* a, int size) const = 0;

  // Update automorphism: T <- T exp(alpha*D/2), then polar decompose
  // to extract W = P^2 and rotate R.
  //   Nonneg: w_i *= exp(alpha * d_i), r unchanged.
  //   PSD:    M = W^{1/2} exp(alpha*D/2), polar M=PT,
  //           W = P^2, R = T^T R T.
  virtual void updateAutomorphism(double* w, double* r, double alpha,
                                  const double* d, int size) const = 0;

  // Same as updateAutomorphism but operates on P = sqrt(W) directly.
  // Input: p = P_old.  Output: p = P_new (not P_new²).
  //   Nonneg: p_i *= exp(alpha * d_i / 2).
  //   PSD:    M = P * exp(alpha*D/2), polar M=P_new*T,
  //           p = P_new, R = T^T R T.
  virtual void updateAutomorphismP(double* p, double* r, double alpha,
                                   const double* d, int size) const = 0;

  // Polar-free automorphism update: M_new = M_old * exp(alpha*D/2).
  // M tracks the full automorphism (no polar split into P*T).
  // For PSD: r is unchanged (rotation absorbed into M).
  // For SOC: polar is done internally (O(n)), r is rotated by T.
  // For nonneg: r is unchanged (T = I).
  virtual void updateM(double* m, double* r, double alpha,
                       const double* d, int size) const = 0;

  // Apply automorphism: out = M * x * M^T.
  //   Nonneg: out_i = m_i^2 * x_i.
  //   PSD:    Out = M * X * M^T  (M general, X symmetric).
  virtual void applyM(double* out, const double* m,
                      const double* x, int size) const = 0;

  // Apply transpose automorphism: out = M^T * x * M.
  //   Nonneg: out_i = m_i^2 * x_i  (same as applyM).
  //   PSD:    Out = M^T * X * M  (M general, X symmetric).
  virtual void applyMt(double* out, const double* m,
                       const double* x, int size) const = 0;

  // Compute W = M * M^T.
  //   Nonneg: w_i = m_i^2.
  //   PSD:    W = M * M^T  (symmetric, positive semidefinite).
  virtual void squareM(double* w, const double* m, int size) const = 0;

  // Line search: largest k > 0 with ||d0 + k*d1||_inf <= 1.
  //   Nonneg: per-element bound |d0_i + k*d1_i| <= 1.
  //   PSD:    GEV on (D1, I ± D0) to find when eigenvalues hit ±1.
  virtual double lineSearchK(const double* d0, const double* d1,
                             int size) const = 0;

  // Project onto the cone: out = argmin ||out - a||  s.t. out in K.
  virtual void project(double* out, const double* a, int size) const = 0;

  // Geodesic update from raw slack (sqrt-free).
  //   W_new = W^{1/2} exp(α·(I + P(W^{1/2})(S))) W^{1/2}
  //         = exp(α·(I + W·S)) · W   (for PSD: avoids eigendecomposition of W)
  //
  // Default: falls back to computing d = I + P(W^{1/2})(S), then geodesicUpdate.
  // PSD override: computes exp(α(I + WS))·W via Padé without W^{1/2}.
  virtual void geodesicUpdateFromSlack(double* W_out, const double* W,
                                       double alpha, const double* slack,
                                       int size) const {
    // Default: d = I + P(sqrt(W))(S), then standard geodesicUpdate.
    std::vector<double> sqW(size), d(size), ones(size);
    sqrt(sqW.data(), W, size);
    quadraticRepresentation(d.data(), sqW.data(), slack, size);
    setIdentity(ones.data(), size);
    for (int i = 0; i < size; ++i) d[i] += ones[i];
    geodesicUpdate(W_out, W, alpha, d.data(), size);
  }
  // --- z-space operations for geodesic IPM on general cones ---
  //
  // These are stateless: the segment data z (first argument) is the
  // internal representation (W = -∇F(z) for symmetric cones, z itself
  // for barrier cones).  The algorithm owns the RowSpace and calls
  // SetScaling to sync with constraint workspaces for Gram assembly.

  // Gradient: grad = ∇F(z).  Nonneg: grad_i = -z_i (since z stores W).
  virtual void computeGradient(double* grad, const double* z,
                               int size) const {
    (void)grad; (void)z; (void)size; std::abort();
  }

  // Hessian-vector product: out = H(z) · v.  Nonneg: out_i = z_i² v_i.
  virtual void hessianProduct(double* out, const double* z,
                              const double* v, int size) const {
    (void)out; (void)z; (void)v; (void)size; std::abort();
  }

  // Full Hessian matrix: out = H(z), column-major, size × size.
  // Used for Gram assembly (chol(H) * A).  Default builds from hessianProduct.
  virtual void hessian(double* out, const double* z, int size) const {
    std::vector<double> ei(size, 0.0);
    for (int j = 0; j < size; ++j) {
      ei[j] = 1.0;
      hessianProduct(out + j * size, z, ei.data(), size);
      ei[j] = 0.0;
    }
  }

  // Squared Hessian norm: ||target - z_primal||²_{H(z)}.
  // Nonneg: Σ (z_i target_i - 1)² (where z stores W, z_primal = 1/W).
  virtual double hessianNormSquared(const double* z, const double* target,
                                    int size) const {
    (void)z; (void)target; (void)size; std::abort(); return 0;
  }

  // Step size from z-space tangent (target - z_primal).
  // Nonneg: min(1, 2/||d||²_∞) where d_i = 1 - z_i target_i.
  virtual double stepSize(const double* z, const double* target,
                          int size) const {
    (void)z; (void)target; (void)size; std::abort(); return 0;
  }

  // Geodesic step: z ← updated state after Exp_z(α(target - z_primal)).
  // Modifies z in place.  Nonneg: d_i = 1 - z_i target_i; z_i *= exp(αd_i).
  virtual void geodesicStepTarget(double* z, double alpha,
                                  const double* target, int size) const {
    (void)z; (void)alpha; (void)target; (void)size; std::abort();
  }

  // Line search: max k with feasibility for ż(k) = target0 + k·target1 - z_primal.
  // Nonneg: d0_i = 1 - z_i target0_i, d1_i = -z_i target1_i, then lineSearchK.
  virtual double lineSearchTarget(const double* z, const double* target0,
                                  const double* target1, int size) const {
    (void)z; (void)target0; (void)target1; (void)size; std::abort(); return 0;
  }

  // Barrier parameter ν for a cone of this dimension.
  virtual double barrierParameter(int size) const {
    (void)size; std::abort(); return 0;
  }
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
