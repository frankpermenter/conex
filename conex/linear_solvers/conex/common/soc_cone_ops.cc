#include "conex/common/soc_cone_ops.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace conex {
namespace EuclideanJordanAlgebra {

namespace {

// Spectral decomposition: (t, x) = λ₁ c₁ + λ₂ c₂
// where λ₁ = t + ||x||, λ₂ = t - ||x||.
struct SOCSpectral {
  double lam1, lam2;      // eigenvalues
  double nx;              // ||x||
  Eigen::VectorXd xhat;  // x / ||x|| (unit direction, or zero)

  SOCSpectral(const double* a, int size) {
    double t = a[0];
    Eigen::Map<const Eigen::VectorXd> x(a + 1, size - 1);
    nx = x.norm();
    lam1 = t + nx;
    lam2 = t - nx;
    if (nx > 1e-15) {
      xhat = x / nx;
    } else {
      xhat = Eigen::VectorXd::Zero(size - 1);
      if (size > 1) xhat(0) = 1.0;  // arbitrary unit vector
    }
  }

  // Reconstruct (t, x) from eigenvalues and direction.
  void Reconstruct(double* out, int size) const {
    out[0] = 0.5 * (lam1 + lam2);
    double half_diff = 0.5 * (lam1 - lam2);
    for (int i = 0; i < size - 1; ++i)
      out[i + 1] = half_diff * xhat(i);
  }
};

// Apply a scalar function to eigenvalues and reconstruct.
template <typename F>
void SpectralApply(double* out, const double* a, int size, F func) {
  SOCSpectral s(a, size);
  s.lam1 = func(s.lam1);
  s.lam2 = func(s.lam2);
  s.Reconstruct(out, size);
}

}  // namespace

// Jordan product: (t₁t₂ + x₁·x₂, t₁x₂ + t₂x₁).
void SOCConeOps::product(double* out, const double* a, const double* b,
                         int size) const {
  double ta = a[0], tb = b[0];
  Eigen::Map<const Eigen::VectorXd> xa(a + 1, size - 1);
  Eigen::Map<const Eigen::VectorXd> xb(b + 1, size - 1);
  out[0] = ta * tb + xa.dot(xb);
  Eigen::Map<Eigen::VectorXd> xo(out + 1, size - 1);
  xo = ta * xb + tb * xa;
}

// Geodesic: apply exp(α·eigenvalue) to each eigenvalue.
void SOCConeOps::geodesicUpdate(double* out, const double* a, double alpha,
                                const double* d, int size) const {
  SOCSpectral sa(a, size);
  SOCSpectral sd(d, size);
  // In the eigenbasis of a, apply exp(α·λ_d) to each eigenvalue of a.
  // For SOC with rank 2, the eigenbasis of a is {c₁, c₂}.
  // The geodesic is: new eigenvalues = old_λᵢ * exp(α * d_λᵢ).
  // But d is in the tangent space, not in a's eigenbasis.
  // Use spectral: a^{1/2} exp(α d) a^{1/2}.
  double sqrt_lam1 = std::sqrt(std::max(sa.lam1, 0.0));
  double sqrt_lam2 = std::sqrt(std::max(sa.lam2, 0.0));

  // sqrtA
  double sqrtA[size];
  SOCSpectral sa_copy = sa;
  sa_copy.lam1 = sqrt_lam1;
  sa_copy.lam2 = sqrt_lam2;
  sa_copy.Reconstruct(sqrtA, size);

  // expD
  double expD[size];
  SpectralApply(expD, d, size, [alpha](double l) { return std::exp(alpha * l); });

  // sqrtA * expD * sqrtA = P(sqrtA)(expD)
  quadraticRepresentation(out, sqrtA, expD, size);
}

void SOCConeOps::setIdentity(double* out, int size) const {
  out[0] = 1.0;
  for (int i = 1; i < size; ++i) out[i] = 0.0;
}

double SOCConeOps::normInf(const double* a, int size) const {
  SOCSpectral s(a, size);
  return std::max(std::abs(s.lam1), std::abs(s.lam2));
}

// Trace inner product: <a, a> = 2(t² + ||x||²) = λ₁² + λ₂².
double SOCConeOps::squaredNorm(const double* a, int size) const {
  SOCSpectral s(a, size);
  return s.lam1 * s.lam1 + s.lam2 * s.lam2;
}

// Trace inner product: <a, b> = 2(t_a t_b + x_a · x_b) = λ₁(a∘b) + λ₂(a∘b).
double SOCConeOps::dot(const double* a, const double* b, int size) const {
  double ta = a[0], tb = b[0];
  Eigen::Map<const Eigen::VectorXd> xa(a + 1, size - 1);
  Eigen::Map<const Eigen::VectorXd> xb(b + 1, size - 1);
  return 2.0 * (ta * tb + xa.dot(xb));
}

void SOCConeOps::sqrt(double* out, const double* a, int size) const {
  SpectralApply(out, a, size, [](double l) {
    return std::sqrt(std::max(l, 0.0));
  });
}

// P(a)b = 2(a ∘ b) ∘ a - a² ∘ b.
void SOCConeOps::quadraticRepresentation(double* out, const double* a,
                                         const double* b, int size) const {
  // a² in SOC: (t² + ||x||², 2tx).
  double ta = a[0];
  Eigen::Map<const Eigen::VectorXd> xa(a + 1, size - 1);
  double a2_t = ta * ta + xa.squaredNorm();
  // a2_x = 2*ta*xa

  // a ∘ b:
  double tb = b[0];
  Eigen::Map<const Eigen::VectorXd> xb(b + 1, size - 1);
  double ab_t = ta * tb + xa.dot(xb);
  // ab_x = ta*xb + tb*xa

  // 2*(a∘b)∘a:
  // Let c = a∘b = (ab_t, ta*xb + tb*xa).
  // c∘a = (c_t*ta + cx·xa, c_t*xa + ta*cx)
  double cx_dot_xa = (ta * xb + tb * xa).dot(xa);
  double two_ca_t = 2.0 * (ab_t * ta + cx_dot_xa);
  // two_ca_x = 2*(ab_t*xa + ta*(ta*xb + tb*xa))

  // a²∘b:
  // a² = (a2_t, 2*ta*xa). a²∘b = (a2_t*tb + 2*ta*xa·xb, a2_t*xb + tb*2*ta*xa)
  double a2b_t = a2_t * tb + 2.0 * ta * xa.dot(xb);

  out[0] = two_ca_t - a2b_t;
  Eigen::Map<Eigen::VectorXd> xo(out + 1, size - 1);
  xo = 2.0 * (ab_t * xa + ta * (ta * xb + tb * xa))
       - (a2_t * xb + tb * 2.0 * ta * xa);
}

// Solve RD + DR = 2Δ for D, given R and Δ.
// In R's spectral basis: D_ij = 2Δ_ij / (λ_i + λ_j).
// For rank-2 SOC: eigenvalues λ₁, λ₂ of R.
// D_11 = Δ_11/λ₁, D_22 = Δ_22/λ₂, D_12 = 2Δ_12/(λ₁+λ₂).
void SOCConeOps::solveLyapunovForD(double* out, const double* r,
                                   const double* delta, int size) const {
  SOCSpectral sr(r, size);
  // Transform delta to R's eigenbasis.
  // Idempotents: c₁ = ½(1, xhat), c₂ = ½(1, -xhat).
  // Delta in eigenbasis: Δ₁₁ = <Δ, c₁>, Δ₂₂ = <Δ, c₂>, Δ₁₂ from off-diagonal.
  double td = delta[0];
  Eigen::Map<const Eigen::VectorXd> xd(delta + 1, size - 1);
  double xd_dot_xhat = xd.dot(sr.xhat);

  // Eigenvalues of delta: td ± xd·xhat (projection onto R's eigenbasis).
  double d11 = td + xd_dot_xhat;  // <delta, c1> * 2
  double d22 = td - xd_dot_xhat;  // <delta, c2> * 2
  // Off-diagonal component: xd - (xd·xhat)*xhat projected.
  Eigen::VectorXd xd_perp = xd - xd_dot_xhat * sr.xhat;

  // Solve: D eigenvalues = Delta eigenvalues / R eigenvalues.
  double d1 = (std::abs(sr.lam1) > 1e-15) ? d11 / sr.lam1 : 0.0;
  double d2 = (std::abs(sr.lam2) > 1e-15) ? d22 / sr.lam2 : 0.0;

  // Off-diagonal: scale by 2/(λ₁+λ₂).
  double lam_sum = sr.lam1 + sr.lam2;
  double off_scale = (std::abs(lam_sum) > 1e-15) ? 2.0 / lam_sum : 0.0;

  // Reconstruct D.
  out[0] = 0.5 * (d1 + d2);
  Eigen::Map<Eigen::VectorXd> xo(out + 1, size - 1);
  xo = 0.5 * (d1 - d2) * sr.xhat + off_scale * xd_perp;
}

void SOCConeOps::abs(double* out, const double* a, int size) const {
  SpectralApply(out, a, size, [](double l) { return std::abs(l); });
}

double SOCConeOps::minEigenvalue(const double* a, int size) const {
  SOCSpectral s(a, size);
  return std::min(s.lam1, s.lam2);
}

void SOCConeOps::updateAutomorphism(double* w, double* r, double alpha,
                                    const double* d, int size) const {
  // M = W^{1/2} exp(α D / 2).  For SOC, polar decomposition is trivial
  // (rank 2, the "rotation" is just a sign flip on eigenvalues).
  // Compute via spectral: apply to eigenvalues, reconstruct.

  // sqrtW
  double sqrtW[size];
  sqrt(sqrtW, w, size);

  // exp(α/2 * D)
  double expHalfD[size];
  SpectralApply(expHalfD, d, size, [alpha](double l) {
    return std::exp(0.5 * alpha * l);
  });

  // M = sqrtW * expHalfD (Jordan product).
  double M[size];
  product(M, sqrtW, expHalfD, size);

  // Polar: for SOC, M = P * T where P has eigenvalues |λᵢ(M)|.
  SOCSpectral sm(M, size);
  double p_lam1 = std::abs(sm.lam1);
  double p_lam2 = std::abs(sm.lam2);

  // W = P² (eigenvalues squared).
  SOCSpectral sw = sm;
  sw.lam1 = p_lam1 * p_lam1;
  sw.lam2 = p_lam2 * p_lam2;
  sw.Reconstruct(w, size);

  // If an eigenvalue of M was negative, T is a nontrivial reflection.
  // Rotate r: flip the component along M's spectral direction.
  bool need_flip = (sm.lam1 < 0 || sm.lam2 < 0);
  if (need_flip && size > 1) {
    double r_t = r[0];
    Eigen::Map<Eigen::VectorXd> r_x(r + 1, size - 1);
    double r_along = r_x.dot(sm.xhat);
    r_x -= 2.0 * r_along * sm.xhat;
  }
}

void SOCConeOps::updateAutomorphismP(double* p, double* r, double alpha,
                                     const double* d, int size) const {
  // M = P * exp(α D / 2), polar decomposition, store P_new (not P²).

  // exp(α/2 * D)
  double expHalfD[size];
  SpectralApply(expHalfD, d, size, [alpha](double l) {
    return std::exp(0.5 * alpha * l);
  });

  // M = P * expHalfD (Jordan product).
  double M[size];
  product(M, p, expHalfD, size);

  // Polar: M = P_new * T. P_new has eigenvalues |λᵢ(M)|.
  SOCSpectral sm(M, size);
  bool need_flip = (sm.lam1 < 0 || sm.lam2 < 0);

  // P_new: eigenvalues are |λᵢ(M)|.
  SOCSpectral sp = sm;
  sp.lam1 = std::abs(sm.lam1);
  sp.lam2 = std::abs(sm.lam2);
  sp.Reconstruct(p, size);

  // If an eigenvalue was negative, T is a nontrivial reflection in M's
  // eigenbasis.  Rotate r by T: swap r's eigenvalue projections onto
  // M's idempotents c₁ = (1, x̂)/2, c₂ = (1, -x̂)/2.
  if (need_flip && size > 1) {
    // Decompose r in M's eigenbasis.
    double r_t = r[0];
    Eigen::Map<Eigen::VectorXd> r_x(r + 1, size - 1);
    double r_along = r_x.dot(sm.xhat);       // projection onto x̂
    // Eigenvalue projections: μ₁ = r_t + r_along, μ₂ = r_t - r_along.
    // T swaps μ₁ ↔ μ₂: (μ₁, μ₂) → (μ₂, μ₁).
    // New r_t = (μ₂ + μ₁)/2 = r_t (unchanged).
    // New r_along = (μ₂ - μ₁)/2 = -r_along.
    // Perpendicular component: unchanged by T.
    r_x -= 2.0 * r_along * sm.xhat;  // flip along-component sign
  }
}

// SOC: polar is O(n), so do it internally and rotate r.
// M output equals P (rotation absorbed into r).
void SOCConeOps::updateM(double* m, double* r, double alpha,
                         const double* d, int size) const {
  updateAutomorphismP(m, r, alpha, d, size);
}

void SOCConeOps::applyM(double* out, const double* m,
                         const double* x, int size) const {
  quadraticRepresentation(out, m, x, size);
}

void SOCConeOps::applyMt(double* out, const double* m,
                          const double* x, int size) const {
  quadraticRepresentation(out, m, x, size);
}

void SOCConeOps::squareM(double* w, const double* m, int size) const {
  double ones[size];
  setIdentity(ones, size);
  quadraticRepresentation(w, m, ones, size);
}

// Find largest k > 0 with ||d0 + k*d1||_inf ≤ 1.
// Eigenvalues: λ(k) = (t₀+k*t₁) ± ||x₀+k*x₁||.
// Need |λ₁(k)| ≤ 1 and |λ₂(k)| ≤ 1.
double SOCConeOps::lineSearchK(const double* d0, const double* d1,
                               int size) const {
  double t0 = d0[0], t1 = d1[0];
  Eigen::Map<const Eigen::VectorXd> x0(d0 + 1, size - 1);
  Eigen::Map<const Eigen::VectorXd> x1(d1 + 1, size - 1);

  // ||x0 + k*x1||² = ||x0||² + 2k*(x0·x1) + k²*||x1||²
  double a_coeff = x1.squaredNorm();
  double b_coeff = 2.0 * x0.dot(x1);
  double c_coeff = x0.squaredNorm();

  // We need: |(t0 + k*t1) + sqrt(a*k² + b*k + c)| ≤ 1
  //      and |(t0 + k*t1) - sqrt(a*k² + b*k + c)| ≤ 1
  //
  // Use nonneg elementwise bound as seed, then bisect with eigenvalue eval.
  // (The quadratic structure makes closed-form messy with the sqrt.)

  auto eval_norm = [&](double k) -> double {
    double tk = t0 + k * t1;
    double nxk = std::sqrt(std::max(a_coeff * k * k + b_coeff * k + c_coeff, 0.0));
    return std::max(std::abs(tk + nxk), std::abs(tk - nxk));
  };

  // Nonneg elementwise bound (upper bound for SOC).
  double k_nn = std::numeric_limits<double>::max();
  for (int i = 0; i < size; ++i) {
    double ai = d0[i], bi = d1[i];
    if (bi > 1e-14)
      k_nn = std::min(k_nn, (1.0 - ai) / bi);
    else if (bi < -1e-14)
      k_nn = std::min(k_nn, (-1.0 - ai) / bi);
  }
  if (k_nn < 0) k_nn = 0;
  if (k_nn > 1e15) return k_nn;

  // Halve from k_nn until feasible.
  double k0 = k_nn;
  while (eval_norm(k0) > 1.0 - 1e-10 && k0 > 1e-15)
    k0 *= 0.5;
  if (eval_norm(k0) > 1.0) return 0;

  // Bisect rightward.
  double lo = k0, hi = std::max(lo + 1.0, lo * 2);
  while (eval_norm(hi) <= 1.0) {
    lo = hi;
    hi *= 2;
    if (hi > 1e15) return hi;
  }
  for (int iter = 0; iter < 60; ++iter) {
    double mid = 0.5 * (lo + hi);
    if (eval_norm(mid) <= 1.0) lo = mid; else hi = mid;
  }
  return lo;
}

void SOCConeOps::project(double* out, const double* a, int size) const {
  double t = a[0];
  Eigen::Map<const Eigen::VectorXd> x(a + 1, size - 1);
  double nx = x.norm();

  if (t >= nx) {
    // Already in cone.
    for (int i = 0; i < size; ++i) out[i] = a[i];
  } else if (t <= -nx) {
    // Projection is zero.
    for (int i = 0; i < size; ++i) out[i] = 0.0;
  } else {
    // Project onto boundary: ((t + ||x||) / 2, ((t + ||x||) / (2||x||)) * x).
    double s = 0.5 * (t + nx);
    out[0] = s;
    double scale = s / nx;
    Eigen::Map<Eigen::VectorXd> xo(out + 1, size - 1);
    xo = scale * x;
  }
}

const SOCConeOps& socConeOps() {
  static const SOCConeOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
