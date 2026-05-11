// Relative entropy cone barrier operations.
//
// F(u,v,w) = -log(s) - sum log(v_i) - sum log(w_i)
// where s = u - sum w_i * log(w_i / v_i).
//
// Let d = (dim - 1) / 2, l_i = log(w_i / v_i).
//
// Gradient:
//   dF/du   = -1/s
//   dF/dv_i = -w_i/(s*v_i) - 1/v_i
//   dF/dw_i = (l_i + 1)/s - 1/w_i
//
// For Hessian-vector product, we need:
//   ds = du - sum [dw_i * l_i + dw_i - w_i * dv_i / v_i]
//      = du - sum [dw_i * (l_i + 1) - w_i * dv_i / v_i]
//
// Then H*p = d/dt grad(z + t*p)|_{t=0}.

#include "conex/common/rel_entropy_cone_ops.h"
#include <cmath>

namespace conex {
namespace EuclideanJordanAlgebra {

void RelEntropyConeOps::computeGradient(double* grad, const double* z,
                                         int size) const {
  const int d = (size - 1) / 2;
  const double u = z[0];

  // Compute s = u - sum w_i * log(w_i / v_i).
  double s = u;
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    s -= wi * std::log(wi / vi);
  }

  // dF/du = -1/s
  grad[0] = -1.0 / s;

  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];

    // dF/dv_i = -w_i / (s * v_i) - 1 / v_i
    grad[1 + i] = -wi / (s * vi) - 1.0 / vi;

    // dF/dw_i = (log(w_i/v_i) + 1) / s - 1 / w_i
    grad[1 + d + i] = (std::log(wi / vi) + 1.0) / s - 1.0 / wi;
  }
}

void RelEntropyConeOps::hessianProduct(double* out, const double* z,
                                        const double* p, int size) const {
  const int d = (size - 1) / 2;
  const double u = z[0];

  // Precompute s, l_i = log(w_i/v_i).
  double s = u;
  std::vector<double> li(d);
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    li[i] = std::log(wi / vi);
    s -= wi * li[i];
  }

  // Directional derivative of s:
  // ds = pu - sum [pw_i * (l_i + 1) - w_i * pv_i / v_i]
  double pu = p[0];
  double ds = pu;
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    double pvi = p[1 + i];
    double pwi = p[1 + d + i];
    ds -= pwi * (li[i] + 1.0) - wi * pvi / vi;
  }

  // H*p for u-component: d/dt [-1/s] = ds / s^2
  out[0] = ds / (s * s);

  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    double pvi = p[1 + i];
    double pwi = p[1 + d + i];

    // H*p for v_i-component: d/dt [-w_i/(s*v_i) - 1/v_i]
    //   = -[pw_i/(s*v_i) + w_i*(-ds*v_i - s*pv_i)/(s*v_i)^2] + pv_i/v_i^2
    //   = -pw_i/(s*v_i) + w_i*ds/(s^2*v_i) + w_i*pv_i/(s*v_i^2) + pv_i/v_i^2
    out[1 + i] = -pwi / (s * vi)
                 + wi * ds / (s * s * vi)
                 + wi * pvi / (s * vi * vi)
                 + pvi / (vi * vi);

    // H*p for w_i-component: d/dt [(l_i + 1)/s - 1/w_i]
    //   l_i = log(w_i/v_i), so dl_i = pw_i/w_i - pv_i/v_i
    //   d/dt [(l_i+1)/s] = dl_i/s - (l_i+1)*ds/s^2
    //   d/dt [-1/w_i] = pw_i/w_i^2
    double dli = pwi / wi - pvi / vi;
    out[1 + d + i] = dli / s
                     - (li[i] + 1.0) * ds / (s * s)
                     + pwi / (wi * wi);
  }
}

void RelEntropyConeOps::thirdDerivContract(double* out, const double* z,
                                            const double* p, int size) const {
  // F = -log(s) - sum log(v_i) - sum log(w_i)
  // T_l = -d3s_p_l/s + s_l*Q/s^2 + 2*ds_p_l*ds_p/s^2 + T_diag_l
  // where Q = d2s_p - 2*ds_p^2/s.
  //
  // Building blocks for s = u - sum w_i*log(w_i/v_i):
  //   s_u = 1, s_{v_i} = w_i/v_i, s_{w_i} = -(log(w_i/v_i) + 1)
  //   d2s_p = sum_i [-pv_i^2*w_i/v_i^2 + 2*pv_i*pw_i/v_i - pw_i^2/w_i]
  //   d3s_p_u = 0
  //   d3s_p_{v_i} = 2*pv_i*(pv_i*w_i - pw_i*v_i)/v_i^3
  //   d3s_p_{w_i} = pw_i^2/w_i^2 - pv_i^2/v_i^2

  const int d = (size - 1) / 2;
  const double u = z[0];

  // Compute s.
  double s = u;
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    s -= wi * std::log(wi / vi);
  }

  // ds_p = pu + sum [pv_i*w_i/v_i - pw_i*(log(w_i/v_i) + 1)]
  double ds_p = p[0];
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    ds_p += p[1 + i] * wi / vi - p[1 + d + i] * (std::log(wi / vi) + 1.0);
  }

  // d2s_p = sum_i [-pv_i^2*w_i/v_i^2 + 2*pv_i*pw_i/v_i - pw_i^2/w_i]
  double d2s_p = 0;
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    double pvi = p[1 + i];
    double pwi = p[1 + d + i];
    d2s_p += -pvi * pvi * wi / (vi * vi) + 2.0 * pvi * pwi / vi - pwi * pwi / wi;
  }

  double Q = d2s_p - 2.0 * ds_p * ds_p / s;

  // T for u-component (l = 0): d3s_p_u = 0, s_u = 1, ds_p_u = pu... wait.
  // ds_p_l = d(ds_p)/dz_l. For l=u: ds_p doesn't depend on u (s_u = 1,
  // and the only u-dependence in ds_p is through pu*1 = pu, which is constant).
  // Actually ds_p = pu + sum[...] where none of the terms depend on u.
  // So ds_p_u = 0.
  // But s_u = 1, so:
  out[0] = Q / (s * s);  // -0/s + 1*Q/s^2 + 0 + 0

  // T for v_i components (l = 1+i):
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    double pvi = p[1 + i];
    double pwi = p[1 + d + i];

    double s_l = wi / vi;
    // ds_p_l = d(ds_p)/dv_i:
    //   d(pv_i*w_i/v_i)/dv_i = -pv_i*w_i/v_i^2
    //   d(-pw_i*log(w_i/v_i))/dv_i = pw_i/v_i
    double ds_p_l = -pvi * wi / (vi * vi) + pwi / vi;
    double d3s_p_l = 2.0 * pvi * (pvi * wi - pwi * vi) / (vi * vi * vi);
    double T_diag = -2.0 * pvi * pvi / (vi * vi * vi);  // from -log(v_i)

    out[1 + i] = -d3s_p_l / s + s_l * Q / (s * s)
                 + 2.0 * ds_p_l * ds_p / (s * s) + T_diag;
  }

  // T for w_i components (l = 1+d+i):
  for (int i = 0; i < d; ++i) {
    double vi = z[1 + i];
    double wi = z[1 + d + i];
    double pvi = p[1 + i];
    double pwi = p[1 + d + i];

    double s_l = -(std::log(wi / vi) + 1.0);
    // ds_p_l = d(ds_p)/dw_i: ds_p has terms pv_i*w_i/v_i and -pw_i*(log(w_i/v_i)+1).
    // d/dw_i [pv_i*w_i/v_i] = pv_i/v_i
    // d/dw_i [-pw_i*(log(w_i/v_i)+1)] = -pw_i/w_i
    double ds_p_l = pvi / vi - pwi / wi;
    double d3s_p_l = pwi * pwi / (wi * wi) - pvi * pvi / (vi * vi);
    double T_diag = -2.0 * pwi * pwi / (wi * wi * wi);  // from -log(w_i)

    out[1 + d + i] = -d3s_p_l / s + s_l * Q / (s * s)
                     + 2.0 * ds_p_l * ds_p / (s * s) + T_diag;
  }
}

double RelEntropyConeOps::barrierParameter(int size) const {
  return static_cast<double>(size);  // nu = dim = 1 + 2d
}

void RelEntropyConeOps::getInteriorPoint(double* out, int size) const {
  const int d = (size - 1) / 2;
  // v_i = 1, w_i = 1 => log(w_i/v_i) = 0, so u > 0 suffices.
  // Set u = 1, v = 1, w = 1.
  out[0] = 1.0;
  for (int i = 0; i < d; ++i) {
    out[1 + i] = 1.0;      // v_i
    out[1 + d + i] = 1.0;  // w_i
  }
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
