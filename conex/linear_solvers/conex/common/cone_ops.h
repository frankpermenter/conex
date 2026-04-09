// Per-segment cone operations interface.
// Each cone type (nonneg orthant, PSD, SOCP, ...) implements this.

#pragma once
#include <algorithm>
#include <cmath>

namespace conex {
namespace EuclideanJordanAlgebra {

class ConeOps {
 public:
  virtual ~ConeOps() = default;

  // Element-wise product (Jordan product).
  virtual void product(double* out, const double* a, const double* b,
                       int size) const = 0;

  // Element-wise quotient (Jordan division).
  virtual void quotient(double* out, const double* a, const double* b,
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

  // Solve Lyapunov: a*d + d*a = 2*out.
  //   Nonneg: out_i = a_i * d_i.
  //   PSD:    eigendecompose A, Hadamard in eigenbasis.
  virtual void solveLyapunov(double* out, const double* a, const double* d,
                             int size) const = 0;

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

  // Project onto the cone: out = argmin ||out - a||  s.t. out in K.
  virtual void project(double* out, const double* a, int size) const = 0;
};

// Nonneg orthant (linear constraints): all operations are element-wise.
class NonnegOrthantOps : public ConeOps {
 public:
  void product(double* out, const double* a, const double* b,
               int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] * b[i];
  }

  void quotient(double* out, const double* a, const double* b,
                int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] / b[i];
  }

  void geodesicUpdate(double* out, const double* a, double alpha,
                      const double* d, int size) const override {
    for (int i = 0; i < size; ++i)
      out[i] = a[i] * std::exp(alpha * d[i]);
  }

  void setIdentity(double* out, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = 1.0;
  }

  double normInf(const double* a, int size) const override {
    double result = 0;
    for (int i = 0; i < size; ++i)
      result = std::max(result, std::abs(a[i]));
    return result;
  }

  double squaredNorm(const double* a, int size) const override {
    double result = 0;
    for (int i = 0; i < size; ++i) result += a[i] * a[i];
    return result;
  }

  double dot(const double* a, const double* b, int size) const override {
    double result = 0;
    for (int i = 0; i < size; ++i) result += a[i] * b[i];
    return result;
  }

  void sqrt(double* out, const double* a, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = std::sqrt(a[i]);
  }

  void quadraticRepresentation(double* out, const double* a,
                               const double* b, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] * a[i] * b[i];
  }

  void solveLyapunov(double* out, const double* a, const double* d,
                     int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] * d[i];
  }

  void abs(double* out, const double* a, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = std::abs(a[i]);
  }

  double minEigenvalue(const double* a, int size) const override {
    double result = a[0];
    for (int i = 1; i < size; ++i) result = std::min(result, a[i]);
    return result;
  }

  void updateAutomorphism(double* w, double* /*r*/, double alpha,
                          const double* d, int size) const override {
    for (int i = 0; i < size; ++i)
      w[i] *= std::exp(alpha * d[i]);
  }

  void project(double* out, const double* a, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = std::max(a[i], 0.0);
  }
};

// Singleton for the nonneg orthant ops.
inline const NonnegOrthantOps& nonnegOrthantOps() {
  static const NonnegOrthantOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
