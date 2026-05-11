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

void PowerConeOps::getInteriorPoint(double* out, int size) const {
  // u_i = 1 for all i, w_j = 0 for all j.
  // phi = prod(1^alpha_i)^2 = 1, s = 1 - 0 = 1 > 0.
  for (int i = 0; i < m_; ++i) out[i] = 1.0;
  for (int i = m_; i < size; ++i) out[i] = 0.0;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
