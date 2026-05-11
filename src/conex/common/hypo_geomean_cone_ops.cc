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

void HypoGeoMeanConeOps::thirdDerivContract(double* out, const double* z,
                                              const double* v, int size) const {
  // F = -log(s) - sum log(w_i), s = g - u, g = geomean(w).
  // T_l = -d3s_v_l/s + s_l*Q/s^2 + 2*ds_v_l*ds_v/s^2 + T_diag_l
  // where Q = d2s_v - 2*ds_v^2/s.
  //
  // g = exp((1/d) sum log(w_i)), dg/dw_i = g/(d*w_i).
  // s_u = -1, s_{w_i} = g/(d*w_i).
  //
  // dg_v = g/d * sum(v_i/w_i)  (for i >= 1)
  // ds_v = -vu + dg_v
  //
  // d2g_v = dg_v^2/g - g/d * sum(v_i^2/w_i^2)
  // d2s_v = d2g_v  (u terms contribute nothing to second derivatives of s)
  //
  // d3g_v_l for l=w_k:
  //   = d(d2g_v)/dw_k
  //   = 2*dg_v/(g*d) * (dg_v*(-v_k/w_k) ... ) actually derive from d2g_v formula:
  //   d2g_v = dg_v^2/g - g/d * S2  where S2 = sum v_i^2/w_i^2
  //   d(dg_v^2/g)/dw_k = (2*dg_v*d(dg_v)/dw_k*g - dg_v^2*dg/dw_k)/g^2
  //     d(dg_v)/dw_k = dg_v/(d*w_k) - g*v_k/(d*w_k^2)   [same as dphi_v pattern]
  //     dg/dw_k = g/(d*w_k)
  //   = 2*dg_v/g*(dg_v/(d*w_k) - g*v_k/(d*w_k^2)) - dg_v^2/(g^2)*g/(d*w_k)
  //   = 2*dg_v^2/(g*d*w_k) - 2*dg_v*v_k/(d*w_k^2) - dg_v^2/(g*d*w_k)
  //   = dg_v^2/(g*d*w_k) - 2*dg_v*v_k/(d*w_k^2)
  //
  //   d(g/d*S2)/dw_k = g/(d^2*w_k)*S2 - g/d*2*v_k^2/w_k^3 + dg/dw_k ... wait
  //   d(g*S2/d)/dw_k = (dg/dw_k)*S2/d + g/d*d(S2)/dw_k
  //     = g*S2/(d^2*w_k) + g/d*(-2*v_k^2/w_k^3)
  //     = g/(d*w_k)*(S2/d - 2*v_k^2/w_k^2)
  //
  //   d3g_v_{w_k} = dg_v^2/(g*d*w_k) - 2*dg_v*v_k/(d*w_k^2) - g/(d*w_k)*(S2/d - 2*v_k^2/w_k^2)
  //   = 1/(d*w_k) * [dg_v^2/g - 2*dg_v*v_k/w_k - g*S2/d + 2*g*v_k^2/w_k^2]
  //   = 1/(d*w_k) * [d2g_v - 2*dg_v*v_k/w_k + 2*g*v_k^2/w_k^2]
  // Same pattern as power cone!

  const int d = size - 1;
  const double di = 1.0 / d;
  const double u = z[0];

  // g = geomean(w)
  double log_sum = 0;
  for (int i = 1; i < size; ++i) log_sum += std::log(z[i]);
  double g = std::exp(di * log_sum);
  double s = g - u;

  // dg_v = g/d * sum(v_i/w_i)
  double sum_vw = 0;
  for (int i = 1; i < size; ++i) sum_vw += v[i] / z[i];
  double dg_v = g * di * sum_vw;

  double ds_v = -v[0] + dg_v;

  // d2g_v = dg_v^2/g - g/d * sum(v_i^2/w_i^2)
  double S2 = 0;  // sum v_i^2/w_i^2
  for (int i = 1; i < size; ++i) S2 += v[i] * v[i] / (z[i] * z[i]);
  double d2g_v = dg_v * dg_v / g - g * di * S2;
  double d2s_v = d2g_v;

  double Q = d2s_v - 2.0 * ds_v * ds_v / s;

  // T for u-component: s_u = -1, ds_v_u = 0, d3s_v_u = 0.
  // T_diag_u = 0 (no -log(u) term).
  out[0] = (-1.0) * Q / (s * s);  // s_u * Q / s^2

  // T for w_k components:
  for (int k = 1; k < size; ++k) {
    double wk = z[k];
    double vk = v[k];

    double s_l = g * di / wk;
    double ds_v_l = dg_v * di / wk - g * vk * di / (wk * wk);
    // d3s_v_l = 1/(d*w_k) * (d2g_v - 2*dg_v*v_k/w_k + 2*g*v_k^2/w_k^2)
    double d3s_v_l = di / wk *
        (d2g_v - 2.0 * dg_v * vk / wk + 2.0 * g * vk * vk / (wk * wk));
    double T_diag = -2.0 * vk * vk / (wk * wk * wk);  // from -log(w_k)

    out[k] = -d3s_v_l / s + s_l * Q / (s * s)
             + 2.0 * ds_v_l * ds_v / (s * s) + T_diag;
  }
}

double HypoGeoMeanConeOps::barrierParameter(int size) const {
  return static_cast<double>(size);  // nu = dim = 1 + d
}

double HypoGeoMeanConeOps::barrierValue(const double* z, int size) const {
  const int d = size - 1;
  double log_sum = 0;
  for (int i = 1; i < size; ++i) log_sum += std::log(z[i]);
  double g = std::exp(log_sum / d);
  double val = -std::log(g - z[0]);
  for (int i = 1; i < size; ++i) val -= std::log(z[i]);
  return val;
}

void HypoGeoMeanConeOps::getInteriorPoint(double* out, int size) const {
  // w_i = 1 => geomean = 1, u = 0.5 < 1.
  out[0] = 0.5;
  for (int i = 1; i < size; ++i)
    out[i] = 1.0;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
