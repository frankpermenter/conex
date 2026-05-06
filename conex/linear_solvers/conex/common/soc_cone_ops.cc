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

void SOCConeOps::inverse(double* out, const double* a, int size) const {
  // a^{-1} = (t, -x) / (t² - ||x||²) for SOC element (t, x).
  SpectralApply(out, a, size, [](double l) { return 1.0 / l; });
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

// Build the (1+n)×(1+n) matrix form of the quadratic representation P(a)
// for SOC element a = (a_t, a_x).
// P(a) = [a_t²+||a_x||²,     2·a_t·a_x^T                    ]
//        [2·a_t·a_x,          (a_t²-||a_x||²)·I + 2·a_x·a_x^T]
static Eigen::MatrixXd QuadRepMatrix(const double* a, int size) {
  int n = size - 1;
  double a_t = a[0];
  Eigen::Map<const Eigen::VectorXd> a_x(a + 1, n);
  double nx2 = a_x.squaredNorm();

  Eigen::MatrixXd P(size, size);
  P(0, 0) = a_t * a_t + nx2;
  P.block(0, 1, 1, n) = 2.0 * a_t * a_x.transpose();
  P.block(1, 0, n, 1) = 2.0 * a_t * a_x;
  P.bottomRightCorner(n, n) =
      (a_t * a_t - nx2) * Eigen::MatrixXd::Identity(n, n)
      + 2.0 * a_x * a_x.transpose();
  return P;
}

void SOCConeOps::updateAutomorphism(double* w, double* r, double alpha,
                                    const double* d, int size) const {
  // O(n) implementation exploiting SOC rank-2 structure.
  //
  // The composed automorphism g = P(sqrt(W)) · P(exp(αd/2)) acts as a scalar
  // (det(a)·det(b)) on the (n-2)-dimensional subspace perpendicular to both
  // spectral directions. The nontrivial part is a 3×3 matrix in
  // span{e_t, â_x, ê_perp} where â_x = a_x/||a_x|| and ê_perp is the
  // component of b̂_x perpendicular to â_x.

  double a[size];  // a = sqrt(W)
  sqrt(a, w, size);
  double b[size];  // b = exp(αd/2)
  SpectralApply(b, d, size, [alpha](double l) {
    return std::exp(0.5 * alpha * l);
  });

  int n = size - 1;  // x-dimension
  double a_t = a[0], b_t = b[0];
  Eigen::Map<const Eigen::VectorXd> a_x(a + 1, n);
  Eigen::Map<const Eigen::VectorXd> b_x(b + 1, n);
  double na = a_x.norm(), nb = b_x.norm();

  // W_new = P(a)(b²) = geodesicUpdate(W, alpha, d).  O(n).
  geodesicUpdate(w, w, alpha, d, size);

  // Special case: if either direction is zero, P(a) or P(b) is scalar and T = I.
  if (na < 1e-15 || nb < 1e-15) return;

  Eigen::VectorXd ahat = a_x / na;
  Eigen::VectorXd bhat = b_x / nb;
  double cos_ab = ahat.dot(bhat);
  double sin_ab = std::sqrt(std::max(0.0, 1.0 - cos_ab * cos_ab));

  // If spectral directions are parallel, P(a) and P(b) share eigenbasis and T = I.
  if (sin_ab < 1e-14) return;

  // Build orthonormal ê_perp: component of b̂ perpendicular to â.
  Eigen::VectorXd eperp = bhat - cos_ab * ahat;
  eperp /= sin_ab;

  // P(a) in the 3D basis {e_t, â, ê_perp}:
  //   [a_t²+na², 2*a_t*na, 0; 2*a_t*na, a_t²+na², 0; 0, 0, a_t²-na²]
  Eigen::Matrix3d Pa;
  Pa << a_t*a_t + na*na, 2*a_t*na, 0,
        2*a_t*na, a_t*a_t + na*na, 0,
        0, 0, a_t*a_t - na*na;

  // P(b) in the 3D basis {e_t, â, ê_perp}:
  // b_x in this basis: along â = nb*cos_ab, along ê_perp = nb*sin_ab.
  double bc = nb * cos_ab, bs = nb * sin_ab;
  Eigen::Matrix3d Pb;
  Pb << b_t*b_t + nb*nb, 2*b_t*bc, 2*b_t*bs,
        2*b_t*bc, b_t*b_t + 2*bc*bc - nb*nb, 2*bc*bs,
        2*b_t*bs, 2*bc*bs, b_t*b_t + 2*bs*bs - nb*nb;

  // Composed automorphism in 3D subspace.
  Eigen::Matrix3d G = Pa * Pb;

  // Polar decomposition of G: G = P3 * T3.
  Eigen::Matrix3d GGt = G * G.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(GGt);
  Eigen::Matrix3d P3 = eig.eigenvectors() *
      eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eig.eigenvectors().transpose();
  Eigen::Matrix3d T3 = P3.inverse() * G;

  // Apply T3^T to r's projection onto the 3D subspace.
  // r = r_t * e_t + (r·â) * â + (r·ê_perp) * ê_perp + r_rest
  // Only the 3D part rotates; r_rest is unchanged (T = I on perpendicular).
  Eigen::Map<Eigen::VectorXd> r_x(r + 1, n);
  double r_along_a = r_x.dot(ahat);
  double r_along_e = r_x.dot(eperp);
  Eigen::Vector3d r3(r[0], r_along_a, r_along_e);
  Eigen::Vector3d r3_new = T3.transpose() * r3;

  // Reconstruct r: remove old 3D components, add new ones.
  r_x -= r_along_a * ahat + r_along_e * eperp;
  r[0] = r3_new(0);
  r_x += r3_new(1) * ahat + r3_new(2) * eperp;
}

void SOCConeOps::updateAutomorphismP(double* p, double* r, double alpha,
                                     const double* d, int size) const {
  // Compute W = P², update W and r, recover P = sqrt(W_new).
  double w[size];
  // W = P²: use quadraticRepresentation(P, e).
  double e_vec[size];
  setIdentity(e_vec, size);
  quadraticRepresentation(w, p, e_vec, size);
  updateAutomorphism(w, r, alpha, d, size);
  sqrt(p, w, size);
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
