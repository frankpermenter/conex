// Compiler firewall: all Eigen AutoDiff template instantiations live here.
// Tests include only autodiff_barrier.h — never see autodiff headers.

#include "autodiff_barrier.h"
#include <unsupported/Eigen/AutoDiff>
#include <cmath>

namespace conex {
namespace testing {

// --- AD type aliases ---
// First order: value + gradient vector.
using AD1 = Eigen::AutoDiffScalar<Eigen::VectorXd>;
using AV1 = Eigen::Matrix<AD1, Eigen::Dynamic, 1>;

// Second order: value + gradient of (value + gradient).
using Inner = Eigen::Matrix<AD1, Eigen::Dynamic, 1>;
using AD2 = Eigen::AutoDiffScalar<Inner>;
using AV2 = Eigen::Matrix<AD2, Eigen::Dynamic, 1>;

// --- Lift helpers ---

// Lift VectorXd to first-order AD with identity seed.
static AV1 lift1(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV1 az(n);
  for (int i = 0; i < n; ++i) {
    az(i).value() = z(i);
    az(i).derivatives() = Eigen::VectorXd::Unit(n, i);
  }
  return az;
}

// Lift VectorXd to second-order AD with identity seeds.
//
// For independent variables z_i, the seeding is:
//   value:        AD1(z_i, e_i)     -- inner differentiation
//   derivatives:  [AD1(δ_{ij}, 0)]  -- outer differentiation
//
// The inner derivatives of the outer seeds are zero because ∂²z_i/∂z_j∂z_k = 0
// (variables are independent). The Hessian cross-terms arise from the chain
// rule applied to the barrier function, not from the variable seeds.
static AV2 lift2(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az(n);
  for (int i = 0; i < n; ++i) {
    // Inner level: value = z(i), gradient seed = e_i.
    az(i).value().value() = z(i);
    az(i).value().derivatives() = Eigen::VectorXd::Unit(n, i);
    // Outer level: ∂z_i/∂z_j = δ_{ij}, with zero inner derivatives.
    az(i).derivatives().resize(n);
    for (int j = 0; j < n; ++j) {
      az(i).derivatives()(j).value() = (i == j) ? 1.0 : 0.0;
      az(i).derivatives()(j).derivatives() = Eigen::VectorXd::Zero(n);
    }
  }
  return az;
}

// Extract gradient from AD1 result.
static Eigen::VectorXd extract_grad(const AD1& f) {
  return f.derivatives();
}

// Extract Hessian from AD2 result (the outer derivatives of the value's
// derivatives give the second-order cross terms).
static Eigen::MatrixXd extract_hessian(const AD2& f, int n) {
  Eigen::MatrixXd H(n, n);
  for (int j = 0; j < n; ++j) {
    H.col(j) = f.derivatives()(j).derivatives();
  }
  return 0.5 * (H + H.transpose());
}

// Helper: scale an AD scalar by a double. Works at any nesting depth
// by recursively scaling value and derivatives.
inline double scale(double c, double x) { return c * x; }

inline AD1 scale(double c, const AD1& x) {
  return AD1(c * x.value(), c * x.derivatives());
}

inline AD2 scale(double c, const AD2& x) {
  Inner scaled_derivs(x.derivatives().size());
  for (int i = 0; i < x.derivatives().size(); ++i)
    scaled_derivs(i) = scale(c, x.derivatives()(i));
  return AD2(scale(c, x.value()), scaled_derivs);
}

using AD3 = Eigen::AutoDiffScalar<Eigen::Matrix<AD2, Eigen::Dynamic, 1>>;
using AV3 = Eigen::Matrix<AD3, Eigen::Dynamic, 1>;

inline AD3 scale(double c, const AD3& x) {
  Eigen::Matrix<AD2, Eigen::Dynamic, 1> scaled_derivs(x.derivatives().size());
  for (int i = 0; i < x.derivatives().size(); ++i)
    scaled_derivs(i) = scale(c, x.derivatives()(i));
  return AD3(scale(c, x.value()), scaled_derivs);
}

// AD3/AV3 defined above (before scale overloads).

static AV3 lift3(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV3 az(n);
  for (int i = 0; i < n; ++i) {
    // Level 1 (innermost, AD1): value = z_i, deriv = e_i
    az(i).value().value().value() = z(i);
    az(i).value().value().derivatives() = Eigen::VectorXd::Unit(n, i);

    // Level 2 (middle, AD2): deriv(j) = AD1(δ_{ij}, 0)
    az(i).value().derivatives().resize(n);
    for (int j = 0; j < n; ++j) {
      az(i).value().derivatives()(j).value() = (i == j) ? 1.0 : 0.0;
      az(i).value().derivatives()(j).derivatives() = Eigen::VectorXd::Zero(n);
    }

    // Level 3 (outermost, AD3): deriv(l) = AD2(δ_{il}, 0)
    az(i).derivatives().resize(n);
    for (int l = 0; l < n; ++l) {
      az(i).derivatives()(l).value().value() = (i == l) ? 1.0 : 0.0;
      az(i).derivatives()(l).value().derivatives() = Eigen::VectorXd::Zero(n);
      az(i).derivatives()(l).derivatives().resize(n);
      for (int k = 0; k < n; ++k) {
        az(i).derivatives()(l).derivatives()(k).value() = 0.0;
        az(i).derivatives()(l).derivatives()(k).derivatives() = Eigen::VectorXd::Zero(n);
      }
    }
  }
  return az;
}

// Extract T_l = D³F[v, v, e_l] from AD3 result.
// f.derivatives()(l) is AD2: its Hessian gives ∂³F/∂z_l ∂z_i ∂z_j.
// T_l = Σ_{ij} v_i v_j ∂³F/∂z_l ∂z_i ∂z_j.
static Eigen::VectorXd extract_third_contract(const AD3& f, int n,
                                               const Eigen::VectorXd& v) {
  Eigen::VectorXd T(n);
  for (int l = 0; l < n; ++l) {
    // f.derivatives()(l) is AD2.
    // Its Hessian: H3_{ij} = ∂³F/∂z_l ∂z_i ∂z_j
    // = f.derivatives()(l).derivatives()(i).derivatives()(j)
    double tl = 0;
    for (int i = 0; i < n; ++i) {
      const Eigen::VectorXd& row = f.derivatives()(l).derivatives()(i).derivatives();
      for (int j = 0; j < n; ++j) {
        tl += v(i) * v(j) * row(j);
      }
    }
    T(l) = tl;
  }
  return T;
}

// ============================================================
// Exponential cone barrier template
// F(x,y,z) = -log(z - y*exp(x/y)) - log(y)
// ============================================================
template <typename Scalar>
static Scalar exp_cone_barrier_impl(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& p) {
  using std::exp;
  using std::log;
  Scalar x = p(0), y = p(1), z = p(2);
  return -log(z - y * exp(x / y)) - log(y);
}

double exp_cone_barrier(const Eigen::VectorXd& z) {
  return exp_cone_barrier_impl<double>(z);
}

Eigen::VectorXd exp_cone_gradient(const Eigen::VectorXd& z) {
  AV1 az = lift1(z);
  AD1 f = exp_cone_barrier_impl(az);
  return extract_grad(f);
}

Eigen::MatrixXd exp_cone_hessian(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az = lift2(z);
  AD2 f = exp_cone_barrier_impl(az);
  return extract_hessian(f, n);
}

Eigen::VectorXd exp_cone_third_deriv(const Eigen::VectorXd& z,
                                      const Eigen::VectorXd& v) {
  AV3 az = lift3(z);
  AD3 f = exp_cone_barrier_impl(az);
  return extract_third_contract(f, z.size(), v);
}

// ============================================================
// Power cone barrier template
// F(u,w;a) = -log((prod u_i^a_i)^2 - ||w||^2) - sum log(u_i)
// where a = alpha, sum a_i = 1.
// ============================================================

// We store alpha as a static thread_local to pass it through the AD template.
static thread_local const Eigen::VectorXd* g_alpha = nullptr;

template <typename Scalar>
static Scalar power_cone_barrier_impl(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& p) {
  using std::exp;
  using std::log;
  const Eigen::VectorXd& alpha = *g_alpha;
  const int m = alpha.size();
  const int dim = p.size();

  // u = p(0..m-1), w = p(m..dim-1)
  Scalar log_prod = p(0) - p(0);  // zero in any AD type
  for (int i = 0; i < m; ++i)
    log_prod = log_prod + scale(alpha(i), log(p(i)));
  Scalar z = exp(log_prod + log_prod);  // (prod u_i^a_i)^2

  Scalar w_sq = p(0) - p(0);  // zero
  for (int i = m; i < dim; ++i)
    w_sq = w_sq + p(i) * p(i);

  Scalar result = -log(z - w_sq);
  for (int i = 0; i < m; ++i)
    result -= log(p(i));
  return result;
}

double power_cone_barrier(const Eigen::VectorXd& z,
                          const Eigen::VectorXd& alpha) {
  g_alpha = &alpha;
  return power_cone_barrier_impl<double>(z);
}

Eigen::VectorXd power_cone_gradient(const Eigen::VectorXd& z,
                                     const Eigen::VectorXd& alpha) {
  g_alpha = &alpha;
  AV1 az = lift1(z);
  AD1 f = power_cone_barrier_impl(az);
  return extract_grad(f);
}

Eigen::MatrixXd power_cone_hessian(const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& alpha) {
  g_alpha = &alpha;
  const int n = z.size();
  AV2 az = lift2(z);
  AD2 f = power_cone_barrier_impl(az);
  return extract_hessian(f, n);
}

Eigen::VectorXd power_cone_third_deriv(const Eigen::VectorXd& z,
                                        const Eigen::VectorXd& alpha,
                                        const Eigen::VectorXd& v) {
  g_alpha = &alpha;
  AV3 az = lift3(z);
  AD3 f = power_cone_barrier_impl(az);
  return extract_third_contract(f, z.size(), v);
}

// ============================================================
// Relative entropy barrier template
// F(u,v,w) = -log(u - sum w_i*log(w_i/v_i))
//            - sum log(v_i) - sum log(w_i)
// z = (u, v_1..v_d, w_1..w_d), dim = 1 + 2d
// ============================================================
template <typename Scalar>
static Scalar rel_entropy_barrier_impl(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& p) {
  using std::log;
  const int dim = p.size();
  const int d = (dim - 1) / 2;
  Scalar u = p(0);

  Scalar rel_ent = p(0) - p(0);  // zero
  for (int i = 0; i < d; ++i) {
    Scalar vi = p(1 + i);
    Scalar wi = p(1 + d + i);
    rel_ent += wi * log(wi / vi);
  }

  Scalar result = -log(u - rel_ent);
  for (int i = 0; i < d; ++i) {
    result -= log(p(1 + i));      // -log(v_i)
    result -= log(p(1 + d + i));  // -log(w_i)
  }
  return result;
}

double rel_entropy_barrier(const Eigen::VectorXd& z) {
  return rel_entropy_barrier_impl<double>(z);
}

Eigen::VectorXd rel_entropy_gradient(const Eigen::VectorXd& z) {
  AV1 az = lift1(z);
  AD1 f = rel_entropy_barrier_impl(az);
  return extract_grad(f);
}

Eigen::MatrixXd rel_entropy_hessian(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az = lift2(z);
  AD2 f = rel_entropy_barrier_impl(az);
  return extract_hessian(f, n);
}

Eigen::VectorXd rel_entropy_third_deriv(const Eigen::VectorXd& z,
                                         const Eigen::VectorXd& v) {
  AV3 az = lift3(z);
  AD3 f = rel_entropy_barrier_impl(az);
  return extract_third_contract(f, z.size(), v);
}

// ============================================================
// Hypo geometric mean barrier template
// F(u,w) = -log(geomean(w) - u) - sum log(w_i)
// z = (u, w_1..w_{d}), dim = 1 + d
// ============================================================
template <typename Scalar>
static Scalar hypo_geomean_barrier_impl(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& p) {
  using std::exp;
  using std::log;
  const int dim = p.size();
  const int d = dim - 1;
  Scalar u = p(0);

  Scalar log_sum = p(0) - p(0);  // zero
  for (int i = 1; i < dim; ++i)
    log_sum = log_sum + log(p(i));
  Scalar geomean = exp(scale(1.0 / d, log_sum));

  Scalar result = -log(geomean - u);
  for (int i = 1; i < dim; ++i)
    result -= log(p(i));
  return result;
}

double hypo_geomean_barrier(const Eigen::VectorXd& z) {
  return hypo_geomean_barrier_impl<double>(z);
}

Eigen::VectorXd hypo_geomean_gradient(const Eigen::VectorXd& z) {
  AV1 az = lift1(z);
  AD1 f = hypo_geomean_barrier_impl(az);
  return extract_grad(f);
}

Eigen::MatrixXd hypo_geomean_hessian(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az = lift2(z);
  AD2 f = hypo_geomean_barrier_impl(az);
  return extract_hessian(f, n);
}

Eigen::VectorXd hypo_geomean_third_deriv(const Eigen::VectorXd& z,
                                          const Eigen::VectorXd& v) {
  AV3 az = lift3(z);
  AD3 f = hypo_geomean_barrier_impl(az);
  return extract_third_contract(f, z.size(), v);
}

}  // namespace testing
}  // namespace conex
