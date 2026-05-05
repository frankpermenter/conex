// Nonneg orthant (linear constraints): all operations are element-wise.

#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class NonnegOrthantOps : public SymmetricConeOperations {
 public:
  void product(double* out, const double* a, const double* b,
               int size) const override {
    for (int i = 0; i < size; ++i) out[i] = a[i] * b[i];
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

  void solveLyapunovForD(double* out, const double* r, const double* delta,
                         int size) const override {
    for (int i = 0; i < size; ++i) out[i] = delta[i] / r[i];
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

  void updateAutomorphismP(double* p, double* /*r*/, double alpha,
                           const double* d, int size) const override {
    for (int i = 0; i < size; ++i)
      p[i] *= std::exp(0.5 * alpha * d[i]);
  }

  void updateM(double* m, double* /*r*/, double alpha,
               const double* d, int size) const override {
    for (int i = 0; i < size; ++i)
      m[i] *= std::exp(0.5 * alpha * d[i]);
  }

  void applyM(double* out, const double* m,
              const double* x, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = m[i] * m[i] * x[i];
  }

  void applyMt(double* out, const double* m,
               const double* x, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = m[i] * m[i] * x[i];
  }

  void squareM(double* w, const double* m, int size) const override {
    for (int i = 0; i < size; ++i) w[i] = m[i] * m[i];
  }

  double lineSearchK(const double* d0, const double* d1,
                     int size) const override {
    double k_max = std::numeric_limits<double>::max();
    for (int i = 0; i < size; ++i) {
      if (d1[i] > 0)
        k_max = std::min(k_max, (1.0 - d0[i]) / d1[i]);
      else if (d1[i] < 0)
        k_max = std::min(k_max, (-1.0 - d0[i]) / d1[i]);
    }
    return k_max;
  }

  void project(double* out, const double* a, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = std::max(a[i], 0.0);
  }

  // Nonneg: d_i = 1 + w_i * s_i, so w_new = w * exp(α(1 + w*s)).
  void geodesicUpdateFromSlack(double* W_out, const double* W,
                               double alpha, const double* slack,
                               int size) const override {
    for (int i = 0; i < size; ++i)
      W_out[i] = W[i] * std::exp(alpha * (1.0 + W[i] * slack[i]));
  }
};

// Singleton for the nonneg orthant ops.
inline const NonnegOrthantOps& nonnegOrthantOps() {
  static const NonnegOrthantOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
