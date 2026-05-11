// Barrier functions for testing.
// Each is a template on the scalar type — works with double, AD1, AD2, AD3.
// No AD headers included here; this file compiles instantly.

#pragma once
#include <cmath>
#include <Eigen/Dense>

// Forward-declare scale from derivatives.h for use in barriers.
// When instantiated with double, this resolves to plain multiplication.
namespace derivatives {
inline double scale(double c, double x);
}

namespace barriers {

// Helper: zero-initialize any scalar type from an existing value.
template <typename S> S zero(const S& x) { return x - x; }

// Exponential cone: F(x,y,z) = -log(z - y*exp(x/y)) - log(y)
// nu = 2
template <typename S>
S exp_cone(const Eigen::Matrix<S, Eigen::Dynamic, 1>& z) {
  using std::exp; using std::log;
  return -log(z(2) - z(1) * exp(z(0) / z(1))) - log(z(1));
}

// Generalized power cone: F(u,w;alpha) = -log((prod u_i^a_i)^2 - ||w||^2) - sum log(u_i)
// nu = m + 2, where m = len(alpha)
// Alpha is passed via a global (set before calling).
inline const Eigen::VectorXd* g_power_alpha = nullptr;

template <typename S>
S power_cone(const Eigen::Matrix<S, Eigen::Dynamic, 1>& p) {
  using std::exp; using std::log;
  using derivatives::scale;
  const auto& alpha = *g_power_alpha;
  const int m = alpha.size();
  const int dim = p.size();

  S log_prod = zero(p(0));
  for (int i = 0; i < m; ++i)
    log_prod = log_prod + scale(alpha(i), log(p(i)));
  S zval = exp(log_prod + log_prod);

  S w_sq = zero(p(0));
  for (int i = m; i < dim; ++i)
    w_sq = w_sq + p(i) * p(i);

  S result = -log(zval - w_sq);
  for (int i = 0; i < m; ++i)
    result = result - log(p(i));
  return result;
}

// Relative entropy: F(u,v,w) = -log(u - sum w_i*log(w_i/v_i)) - sum log(v_i) - sum log(w_i)
// nu = dim
template <typename S>
S rel_entropy(const Eigen::Matrix<S, Eigen::Dynamic, 1>& p) {
  using std::log;
  const int dim = p.size();
  const int d = (dim - 1) / 2;
  S u = p(0);

  S rel_ent = zero(p(0));
  for (int i = 0; i < d; ++i) {
    S vi = p(1 + i);
    S wi = p(1 + d + i);
    rel_ent = rel_ent + wi * log(wi / vi);
  }

  S result = -log(u - rel_ent);
  for (int i = 0; i < d; ++i) {
    result = result - log(p(1 + i));
    result = result - log(p(1 + d + i));
  }
  return result;
}

// Hypograph of geometric mean: F(u,w) = -log(geomean(w) - u) - sum log(w_i)
// nu = dim
template <typename S>
S hypo_geomean(const Eigen::Matrix<S, Eigen::Dynamic, 1>& p) {
  using std::exp; using std::log;
  using derivatives::scale;
  const int dim = p.size();
  const int d = dim - 1;
  S u = p(0);

  S log_sum = zero(p(0));
  for (int i = 1; i < dim; ++i)
    log_sum = log_sum + log(p(i));
  S geomean = exp(scale(1.0 / d, log_sum));

  S result = -log(geomean - u);
  for (int i = 1; i < dim; ++i)
    result = result - log(p(i));
  return result;
}

}  // namespace barriers
