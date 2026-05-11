// Generalized power cone barrier operations.
//
// F(u,w) = -log(phi - ||w||^2) - sum log(u_i)
// where phi = (prod u_i^{alpha_i})^2.
//
// Let s = phi - ||w||^2 (the slack).
//
// Gradient:
//   dF/du_i = -2*alpha_i*phi / (s*u_i) - 1/u_i
//   dF/dw_j = 2*w_j / s
//
// Hessian-vector product H(z)*v:
//   For u-components:
//     (H*v)_i = [2*alpha_i*phi / (s*u_i^2) + 1/u_i^2] * v_i
//             + [2*alpha_i*phi / (s*u_i)] * [sum_k (2*alpha_k*phi/(s*u_k) * v_k)
//                                            - 2*sum_j w_j*v_{m+j}/s]
//             - [2*alpha_i*phi / (s*u_i)] * [2*alpha_i/(u_i) * v_i] / s ... (wrong)
//
// Actually, let's derive this carefully from the gradient.

#include "conex/common/power_cone_ops.h"
#include <cmath>

namespace conex {
namespace EuclideanJordanAlgebra {

std::pair<double, double> PowerConeOps::computePhiS(
    const double* z, int size) const {
  // phi = exp(2 * sum alpha_i * log(u_i))
  double log_prod = 0;
  for (int i = 0; i < m_; ++i)
    log_prod += alpha_(i) * std::log(z[i]);
  double phi = std::exp(2.0 * log_prod);

  // s = phi - ||w||^2
  double w_sq = 0;
  for (int i = m_; i < size; ++i)
    w_sq += z[i] * z[i];

  return {phi, phi - w_sq};
}

void PowerConeOps::computeGradient(double* grad, const double* z,
                                    int size) const {
  auto [phi, s] = computePhiS(z, size);

  // dF/du_i = -2*alpha_i*phi / (s*u_i) - 1/u_i
  for (int i = 0; i < m_; ++i)
    grad[i] = -2.0 * alpha_(i) * phi / (s * z[i]) - 1.0 / z[i];

  // dF/dw_j = 2*w_j / s
  for (int j = m_; j < size; ++j)
    grad[j] = 2.0 * z[j] / s;
}

void PowerConeOps::hessianProduct(double* out, const double* z,
                                   const double* v, int size) const {
  auto [phi, s] = computePhiS(z, size);

  // The gradient is:
  //   g_i = -2*alpha_i*phi / (s*u_i) - 1/u_i    for i < m
  //   g_j = 2*w_j / s                            for j >= m
  //
  // H*v = d/dt grad(z + t*v)|_{t=0}.
  // We need derivatives of phi and s w.r.t. z:
  //   dphi/du_i = 2*alpha_i*phi / u_i
  //   dphi/dw_j = 0
  //   ds/du_i = dphi/du_i = 2*alpha_i*phi / u_i
  //   ds/dw_j = -2*w_j
  //
  // Directional derivatives:
  //   dphi_v = sum_i (2*alpha_i*phi / u_i) * v_i
  //   ds_v = dphi_v - 2*sum_j w_j * v_{m+j}

  double dphi_v = 0;
  for (int i = 0; i < m_; ++i)
    dphi_v += 2.0 * alpha_(i) * phi / z[i] * v[i];

  double dw_dot_v = 0;
  for (int j = m_; j < size; ++j)
    dw_dot_v += z[j] * v[j];
  double ds_v = dphi_v - 2.0 * dw_dot_v;

  // u-components: d/dt [-2*alpha_i*phi / (s*u_i) - 1/u_i]
  //   = -2*alpha_i * [dphi_v * s*u_i - phi*(ds_v*u_i + s*v_i)] / (s*u_i)^2
  //     + v_i / u_i^2
  //   = -2*alpha_i * [dphi_v/(s*u_i) - phi*ds_v/(s^2*u_i) - phi*v_i/(s*u_i^2)]
  //     + v_i / u_i^2
  for (int i = 0; i < m_; ++i) {
    double ai = alpha_(i);
    double ui = z[i];
    double t1 = dphi_v / (s * ui);
    double t2 = phi * ds_v / (s * s * ui);
    double t3 = phi * v[i] / (s * ui * ui);
    out[i] = -2.0 * ai * (t1 - t2 - t3) + v[i] / (ui * ui);
  }

  // w-components: d/dt [2*w_j / s]
  //   = 2*v_j / s - 2*w_j * ds_v / s^2
  for (int j = m_; j < size; ++j)
    out[j] = 2.0 * v[j] / s - 2.0 * z[j] * ds_v / (s * s);
}

double PowerConeOps::barrierParameter(int /*size*/) const {
  return m_ + 2.0;
}

void PowerConeOps::thirdDerivContract(double* out, const double* z,
                                       const double* v, int size) const {
  auto [phi, s] = computePhiS(z, size);
  const int n_w = size - m_;

  // Precompute directional derivatives of phi and s along v.
  // dphi_v = sum_i 2*alpha_i*phi/u_i * v_i
  double dphi_v = 0;
  for (int i = 0; i < m_; ++i)
    dphi_v += 2.0 * alpha_(i) * phi / z[i] * v[i];

  double dw_dot_v = 0;
  for (int j = m_; j < size; ++j)
    dw_dot_v += z[j] * v[j];
  double ds_v = dphi_v - 2.0 * dw_dot_v;

  // T_l = v^T (dH/dz_l) v = d/dz_l [v^T H(z) v].
  // We compute this by differentiating (Hv)^T v = sum_k (Hv)_k v_k
  // where (Hv)_k is the k-th component of the Hessian-vector product.
  //
  // Strategy: compute (Hv) as a function of (phi, s, dphi_v, ds_v, z)
  // and take its derivative w.r.t. z_l, contracted with v.
  //
  // For the u-components of Hv:
  //   (Hv)_i = -2*a_i*(dphi_v/(s*u_i) - phi*ds_v/(s^2*u_i)
  //            - phi*v_i/(s*u_i^2)) + v_i/u_i^2
  //
  // For the w-components of Hv:
  //   (Hv)_j = 2*v_j/s - 2*w_j*ds_v/s^2
  //
  // We need d/dz_l of (Hv)^T v.  Since H is symmetric,
  // d/dz_l [v^T H v] = 2 * v^T (dH/dz_l) v ... no, that's wrong.
  // v^T H v is a scalar, its derivative w.r.t. z_l is the third deriv
  // contraction T_l directly.  But (dH/dz_l) is the derivative of the
  // Hessian matrix, not of the quadratic form.
  //
  // Actually T_l = v^T (dH/dz_l) v, and d/dz_l [v^T H v] = T_l since
  // v doesn't depend on z.
  //
  // So T_l = d/dz_l [(Hv)^T v] = [d(Hv)/dz_l]^T v.
  //
  // Equivalently: T_l = (d/dz_l of each component of Hv) dotted with v.

  // Precompute second-order directional derivatives.
  // For each direction e_l, we need d(dphi_v)/dz_l and d(ds_v)/dz_l.
  //
  // dphi_v = sum_i 2*a_i*phi/u_i * v_i
  // d(dphi_v)/du_l = sum_i 2*a_i*v_i * d(phi/u_i)/du_l
  //   d(phi/u_i)/du_l = (dphi/du_l)/u_i - delta_{il}*phi/u_i^2
  //                   = 2*a_l*phi/(u_l*u_i) - delta_{il}*phi/u_i^2
  //   So d(dphi_v)/du_l = dphi_v * 2*a_l/u_l - 2*a_l*phi/u_l^2 * v_l
  //
  // d(dphi_v)/dw_j = 0  (phi doesn't depend on w)
  //
  // ds_v = dphi_v - 2*sum_j w_j*v_{m+j}
  // d(ds_v)/du_l = d(dphi_v)/du_l
  // d(ds_v)/dw_j = d(dphi_v)/dw_j - 2*v_{m+j} = -2*v_{m+j}

  for (int l = 0; l < size; ++l) {
    // Compute d(dphi_v)/dz_l and d(ds_v)/dz_l.
    double d2phi_v;  // d(dphi_v)/dz_l
    double d2s_v;    // d(ds_v)/dz_l
    double dphi_l;   // dphi/dz_l
    double ds_l;     // ds/dz_l

    if (l < m_) {
      // z_l = u_l
      double al = alpha_(l);
      double ul = z[l];
      dphi_l = 2.0 * al * phi / ul;
      ds_l = dphi_l;
      d2phi_v = dphi_v * 2.0 * al / ul - 2.0 * al * phi / (ul * ul) * v[l];
      d2s_v = d2phi_v;
    } else {
      // z_l = w_{l-m}
      dphi_l = 0;
      ds_l = -2.0 * z[l];
      d2phi_v = 0;
      d2s_v = -2.0 * v[l];
    }

    // Now compute T_l = sum_k [d(Hv)_k/dz_l] * v_k.
    double T_l = 0;

    // u-components: (Hv)_i = -2*a_i*(dphi_v/(s*u_i) - phi*ds_v/(s^2*u_i)
    //                         - phi*v_i/(s*u_i^2)) + v_i/u_i^2
    for (int i = 0; i < m_; ++i) {
      double ai = alpha_(i);
      double ui = z[i];
      // Let A = dphi_v/(s*u_i), B = phi*ds_v/(s^2*u_i), C = phi*v_i/(s*u_i^2)
      // (Hv)_i = -2*a_i*(A - B - C) + v_i/u_i^2
      //
      // d(Hv)_i/dz_l = -2*a_i*(dA - dB - dC) + d(v_i/u_i^2)/dz_l
      // where d means d/dz_l.

      // dA = d[dphi_v/(s*u_i)]/dz_l
      //    = [d2phi_v*s*u_i - dphi_v*(ds_l*u_i + s*delta_{il})] / (s*u_i)^2
      double dA = (d2phi_v * s * ui - dphi_v * (ds_l * ui + (l == i ? s : 0)))
                  / (s * s * ui * ui);

      // dB = d[phi*ds_v/(s^2*u_i)]/dz_l
      //    = [dphi_l*ds_v + phi*d2s_v]/(s^2*u_i)
      //      - phi*ds_v*[2*s*ds_l*u_i + s^2*delta_{il}]/(s^4*u_i^2)
      //    = (dphi_l*ds_v + phi*d2s_v)/(s^2*u_i)
      //      - phi*ds_v*(2*ds_l)/(s^3*u_i)
      //      - phi*ds_v*delta_{il}/(s^2*u_i^2)
      double dB = (dphi_l * ds_v + phi * d2s_v) / (s * s * ui)
                  - phi * ds_v * 2.0 * ds_l / (s * s * s * ui)
                  - (l == i ? phi * ds_v / (s * s * ui * ui) : 0);

      // dC = d[phi*v_i/(s*u_i^2)]/dz_l
      //    = dphi_l*v_i/(s*u_i^2)
      //      - phi*v_i*ds_l/(s^2*u_i^2)
      //      - delta_{il}*2*phi*v_i/(s*u_i^3)
      double dC = dphi_l * v[i] / (s * ui * ui)
                  - phi * v[i] * ds_l / (s * s * ui * ui)
                  - (l == i ? 2.0 * phi * v[i] / (s * ui * ui * ui) : 0);

      // d(v_i/u_i^2)/dz_l = -delta_{il}*2*v_i/u_i^3
      double dD = (l == i ? -2.0 * v[i] / (ui * ui * ui) : 0);

      T_l += (-2.0 * ai * (dA - dB - dC) + dD) * v[i];
    }

    // w-components: (Hv)_j = 2*v_j/s - 2*w_j*ds_v/s^2
    for (int j = m_; j < size; ++j) {
      // d(Hv)_j/dz_l = -2*v_j*ds_l/s^2
      //   - 2*[delta_{jl}*ds_v/s^2 + w_j*d2s_v/s^2 - 2*w_j*ds_v*ds_l/s^3]
      double dHvj = -2.0 * v[j] * ds_l / (s * s)
                    - 2.0 * ((l == j ? 1.0 : 0.0) * ds_v / (s * s)
                             + z[j] * d2s_v / (s * s)
                             - 2.0 * z[j] * ds_v * ds_l / (s * s * s));
      T_l += dHvj * v[j];
    }

    out[l] = T_l;
  }
}

void PowerConeOps::getInteriorPoint(double* out, int size) const {
  // u_i = 1 for all i, w_j = 0 for all j.
  // phi = prod(1^alpha_i)^2 = 1, s = 1 - 0 = 1 > 0.
  for (int i = 0; i < m_; ++i) out[i] = 1.0;
  for (int i = m_; i < size; ++i) out[i] = 0.0;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
