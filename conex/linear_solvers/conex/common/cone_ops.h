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

  // Quadratic representation: P(a)b = a * b * a.
  //   Nonneg: out_i = a_i² * b_i.
  //   PSD:    Out = A * B * A.
  virtual void quadraticRepresentation(double* out, const double* a,
                                       const double* b, int size) const = 0;

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

  void quadraticRepresentation(double* out, const double* a,
                               const double* b, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] * a[i] * b[i];
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
