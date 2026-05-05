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
  // --- z-space operations (nonneg: z stores W = 1/z_primal) ---

  void computeGradient(double* grad, const double* z,
                       int size) const override {
    for (int i = 0; i < size; ++i) grad[i] = -z[i];
  }

  void hessianProduct(double* out, const double* z,
                      const double* v, int size) const override {
    for (int i = 0; i < size; ++i) out[i] = z[i] * z[i] * v[i];
  }

  void hessian(double* out, const double* z, int size) const override {
    // H(z) = diag(z²) for nonneg (z stores W = 1/z_primal, H = diag(W²)).
    std::fill(out, out + size * size, 0.0);
    for (int i = 0; i < size; ++i) out[i * size + i] = z[i] * z[i];
  }

  double hessianNormSquared(const double* z, const double* target,
                            int size) const override {
    double result = 0;
    for (int i = 0; i < size; ++i) {
      double di = z[i] * target[i] - 1.0;
      result += di * di;
    }
    return result;
  }

  double stepSize(const double* z, const double* target,
                  int size) const override {
    double d_inf = 0;
    for (int i = 0; i < size; ++i) {
      double di = 1.0 - z[i] * target[i];
      d_inf = std::max(d_inf, std::abs(di));
    }
    return std::min(1.0, 2.0 / (d_inf * d_inf));
  }

  void geodesicStepTarget(double* z, double alpha,
                          const double* target, int size) const override {
    for (int i = 0; i < size; ++i) {
      double di = 1.0 - z[i] * target[i];
      z[i] *= std::exp(alpha * di);
    }
  }

  double lineSearchTarget(const double* z, const double* target0,
                           const double* target1, int size) const override {
    // d0_i = 1 - z_i*target0_i, d1_i = -z_i*target1_i.
    double k_max = std::numeric_limits<double>::max();
    for (int i = 0; i < size; ++i) {
      double d0i = 1.0 - z[i] * target0[i];
      double d1i = -z[i] * target1[i];
      if (d1i > 0)
        k_max = std::min(k_max, (1.0 - d0i) / d1i);
      else if (d1i < 0)
        k_max = std::min(k_max, (-1.0 - d0i) / d1i);
    }
    return k_max;
  }

  double barrierParameter(int size) const override {
    return static_cast<double>(size);
  }
};

// Singleton for the nonneg orthant ops.
inline const NonnegOrthantOps& nonnegOrthantOps() {
  static const NonnegOrthantOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
