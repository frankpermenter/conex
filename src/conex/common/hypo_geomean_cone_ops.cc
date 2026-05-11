// Hypograph of geometric mean cone barrier operations.
//
// F(u,w) = -log(g - u) - sum log(w_i)
// where g = geomean(w) = exp((1/d) sum log(w_i)).
//
// Let s = g - u (the slack).
//
// Gradient:
//   dF/du = 1/s
//   dF/dw_i = -g/(d*s*w_i) - 1/w_i
//
// For Hessian-vector product:
//   dg = g * (1/d) * sum(pw_i / w_i)
//   ds = dg - pu
//
// Then H*p = d/dt grad(z + t*p)|_{t=0}.

#include "conex/common/hypo_geomean_cone_ops.h"
#include <cmath>

namespace conex {
namespace EuclideanJordanAlgebra {

void HypoGeoMeanConeOps::computeGradient(double* grad, const double* z,
                                           int size) const {
  const int d = size - 1;
  const double u = z[0];
  const double di = 1.0 / d;

  // g = exp((1/d) sum log(w_i))
  double log_sum = 0;
  for (int i = 1; i < size; ++i)
    log_sum += std::log(z[i]);
  double g = std::exp(di * log_sum);
  double s = g - u;

  // dF/du = 1/s
  grad[0] = 1.0 / s;

  // dF/dw_i = -g/(d*s*w_i) - 1/w_i
  for (int i = 1; i < size; ++i)
    grad[i] = -g / (d * s * z[i]) - 1.0 / z[i];
}

void HypoGeoMeanConeOps::hessianProduct(double* out, const double* z,
                                          const double* p, int size) const {
  const int d = size - 1;
  const double u = z[0];
  const double di = 1.0 / d;

  // Precompute g, s.
  double log_sum = 0;
  for (int i = 1; i < size; ++i)
    log_sum += std::log(z[i]);
  double g = std::exp(di * log_sum);
  double s = g - u;

  // Directional derivatives:
  // dg = g * (1/d) * sum(p_i / w_i)
  double sum_pw = 0;
  for (int i = 1; i < size; ++i)
    sum_pw += p[i] / z[i];
  double dg = g * di * sum_pw;
  double ds = dg - p[0];

  // H*p for u-component: d/dt [1/s] = -ds / s^2
  out[0] = -ds / (s * s);

  for (int i = 1; i < size; ++i) {
    double wi = z[i];
    double pwi = p[i];

    // H*p for w_i-component: d/dt [-g/(d*s*w_i) - 1/w_i]
    //
    // Let f_i = -g/(d*s*w_i). Then:
    //   df_i = -[dg*s*w_i - g*(ds*w_i + s*pw_i)] / (d * (s*w_i)^2)
    //        = -[dg/(d*s*w_i)] + g*ds/(d*s^2*w_i) + g*pw_i/(d*s*w_i^2)
    //
    // d/dt [-1/w_i] = pw_i / w_i^2
    //
    // Total:
    out[i] = -dg / (d * s * wi)
             + g * ds / (d * s * s * wi)
             + g * pwi / (d * s * wi * wi)
             + pwi / (wi * wi);
  }
}

double HypoGeoMeanConeOps::barrierParameter(int size) const {
  return static_cast<double>(size);  // nu = dim = 1 + d
}

void HypoGeoMeanConeOps::getInteriorPoint(double* out, int size) const {
  // w_i = 1 => geomean = 1, u = 0.5 < 1.
  out[0] = 0.5;
  for (int i = 1; i < size; ++i)
    out[i] = 1.0;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
