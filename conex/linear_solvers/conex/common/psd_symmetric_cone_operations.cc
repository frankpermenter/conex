#include "conex/common/psd_cone_ops.h"

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace conex {
namespace EuclideanJordanAlgebra {

namespace {
int MatrixDim(int size) {
  int n = static_cast<int>(std::round(std::sqrt(static_cast<double>(size))));
  return n;
}
void Symmetrize(Eigen::Ref<Eigen::MatrixXd> M) {
  M = 0.5 * (M + M.transpose().eval());
}
}  // namespace

void PSDConeOps::product(double* out, const double* a, const double* b,
                         int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  // Jordan product: (AB + BA) / 2.
  Out = 0.5 * (A * B + B * A);
  Symmetrize(Out);
}

// Padé [6/6] (order 13) approximation of expm(M) with scaling and squaring.
// Based on Higham (2005). Accurate to double precision for ||A|| ≤ 5.4.
Eigen::MatrixXd ExpmPade(const Eigen::MatrixXd& M) {
  const int n = M.rows();
  // Pade coefficients for p=q=6 (order 13).
  static const double b[] = {
    64764752532480000.0, 32382376266240000.0, 7771770303897600.0,
    1187353796428800.0,  129060195264000.0,   10559470521600.0,
    670442572800.0,      33522128640.0,       1323241920.0,
    40840800.0,          960960.0,            16380.0,
    182.0,               1.0
  };

  // Scaling: find s such that ||M/2^s||_1 ≤ 5.4.
  double norm1 = M.colwise().template lpNorm<1>().maxCoeff();
  int s = 0;
  const double theta13 = 5.4;
  if (norm1 > theta13) {
    s = static_cast<int>(std::ceil(std::log2(norm1 / theta13)));
  }
  double scale = std::ldexp(1.0, -s);
  Eigen::MatrixXd A = scale * M;

  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
  Eigen::MatrixXd A2 = A * A;
  Eigen::MatrixXd A4 = A2 * A2;
  Eigen::MatrixXd A6 = A2 * A4;

  Eigen::MatrixXd U = A * (A6 * (b[13] * A6 + b[11] * A4 + b[9] * A2) +
                            b[7] * A6 + b[5] * A4 + b[3] * A2 + b[1] * I);
  Eigen::MatrixXd V = A6 * (b[12] * A6 + b[10] * A4 + b[8] * A2) +
                       b[6] * A6 + b[4] * A4 + b[2] * A2 + b[0] * I;

  Eigen::MatrixXd result = (V - U).partialPivLu().solve(V + U);

  for (int i = 0; i < s; ++i) {
    result = result * result;
  }
  return result;
}

void PSDConeOps::geodesicUpdate(double* out, const double* a, double alpha,
                                const double* d, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);

  // Geodesic: A^{1/2} expm(alpha * D) A^{1/2}.
  // Compute A^{1/2} via eigendecomposition.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigA(A);
  Eigen::MatrixXd sqrtA = eigA.eigenvectors() *
      eigA.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigA.eigenvectors().transpose();

  // Padé [3/3] with scaling-and-squaring for expm(alpha * D).
  Eigen::MatrixXd expD = ExpmPade(alpha * D);

  Out = sqrtA * expD * sqrtA;
  Symmetrize(Out);
}

void PSDConeOps::setIdentity(double* out, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out.setIdentity();
}

double PSDConeOps::normInf(const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::MatrixXd Sym = 0.5 * (A + A.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Sym,
      Eigen::EigenvaluesOnly);
  return eig.eigenvalues().cwiseAbs().maxCoeff();
}

double PSDConeOps::squaredNorm(const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::MatrixXd Sym = 0.5 * (A + A.transpose());
  return Sym.squaredNorm();
}

double PSDConeOps::dot(const double* a, const double* b, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  return (A.cwiseProduct(B)).sum();  // trace(A^T B).
}

void PSDConeOps::sqrt(double* out, const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A);
  Out = eig.eigenvectors() *
      eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eig.eigenvectors().transpose();
  Symmetrize(Out);
}

void PSDConeOps::quadraticRepresentation(double* out, const double* a,
                                         const double* b, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out.noalias() = A * B * A;
  Symmetrize(Out);
}

void PSDConeOps::solveLyapunovForD(double* out, const double* r,
                                   const double* delta, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> R(r, n, n);
  Eigen::Map<const Eigen::MatrixXd> Delta(delta, n, n);
  Eigen::Map<Eigen::MatrixXd> D(out, n, n);
  // RD + DR = 2*Delta.  In R's eigenbasis: D_ij = 2*Delta_ij / (l_i + l_j).
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(R);
  const auto& V = eig.eigenvectors();
  const auto& lam = eig.eigenvalues();
  Eigen::MatrixXd Delta_eig = V.transpose() * Delta * V;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      double denom = lam(i) + lam(j);
      Delta_eig(i, j) = (std::abs(denom) > 1e-14)
          ? 2.0 * Delta_eig(i, j) / denom : 0.0;
    }
  D = V * Delta_eig * V.transpose();
  Symmetrize(D);
}

void PSDConeOps::abs(double* out, const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A);
  Out = eig.eigenvectors() *
      eig.eigenvalues().cwiseAbs().asDiagonal() *
      eig.eigenvectors().transpose();
  Symmetrize(Out);
}

double PSDConeOps::minEigenvalue(const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A, Eigen::EigenvaluesOnly);
  return eig.eigenvalues().minCoeff();
}

void PSDConeOps::updateAutomorphism(double* w, double* r, double alpha,
                              const double* d, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> W(w, n, n);
  Eigen::Map<Eigen::MatrixXd> R(r, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);

  // M = W^{1/2} exp(alpha * D / 2).
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigW(W);
  Eigen::MatrixXd sqrtW = eigW.eigenvectors() *
      eigW.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigW.eigenvectors().transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigD(alpha * 0.5 * D);
  Eigen::MatrixXd expHalfD = eigD.eigenvectors() *
      eigD.eigenvalues().array().exp().matrix().asDiagonal() *
      eigD.eigenvectors().transpose();

  Eigen::MatrixXd M = sqrtW * expHalfD;

  // Polar decomposition: M = P * T, P = (M M^T)^{1/2}, T = P^{-1} M.
  Eigen::MatrixXd MMt = M * M.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigP(MMt);
  Eigen::MatrixXd P = eigP.eigenvectors() *
      eigP.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigP.eigenvectors().transpose();
  Eigen::MatrixXd T = P.inverse() * M;

  // W = P^2, R = T^T R T.
  W = P * P;
  Symmetrize(W);
  R = T.transpose() * R * T;
  Symmetrize(R);
}

void PSDConeOps::updateAutomorphismP(double* p, double* r, double alpha,
                                     const double* d, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> P(p, n, n);
  Eigen::Map<Eigen::MatrixXd> R(r, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);

  // M = P * exp(alpha * D / 2).  No eigendecomp of W needed.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigD(alpha * 0.5 * D);
  Eigen::MatrixXd expHalfD = eigD.eigenvectors() *
      eigD.eigenvalues().array().exp().matrix().asDiagonal() *
      eigD.eigenvectors().transpose();

  Eigen::MatrixXd M = P * expHalfD;

  // Polar decomposition: M = P_new * T, P_new = (M M^T)^{1/2}, T = P_new^{-1} M.
  Eigen::MatrixXd MMt = M * M.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigP(MMt);
  Eigen::MatrixXd P_new = eigP.eigenvectors() *
      eigP.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigP.eigenvectors().transpose();
  Eigen::MatrixXd T = P_new.inverse() * M;

  // P = P_new, R = T^T R T.
  P = P_new;
  Symmetrize(P);
  R = T.transpose() * R * T;
  Symmetrize(R);
}

void PSDConeOps::updateM(double* m, double* /*r*/, double alpha,
                         const double* d, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> M(m, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);

  // M_new = M_old * exp(alpha * D / 2).  Padé approximant, no eigendecomp.
  M = M * ExpmPade(alpha * 0.5 * D);
}

void PSDConeOps::applyM(double* out, const double* m,
                         const double* x, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Eigen::Map<const Eigen::MatrixXd> M(m, n, n);
  Eigen::Map<const Eigen::MatrixXd> X(x, n, n);
  Out = M * X * M.transpose();
  Symmetrize(Out);
}

void PSDConeOps::applyMt(double* out, const double* m,
                          const double* x, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Eigen::Map<const Eigen::MatrixXd> M(m, n, n);
  Eigen::Map<const Eigen::MatrixXd> X(x, n, n);
  Out = M.transpose() * X * M;
  Symmetrize(Out);
}

void PSDConeOps::squareM(double* w, const double* m, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> W(w, n, n);
  Eigen::Map<const Eigen::MatrixXd> M(m, n, n);
  W = M * M.transpose();
  Symmetrize(W);
}

double PSDConeOps::lineSearchK(const double* d0, const double* d1,
                               int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> D0(d0, n, n);
  Eigen::Map<const Eigen::MatrixXd> D1(d1, n, n);

  // Find largest k > 0 with ||D0 + k*D1||_inf <= 1.
  // Symmetrize inputs for consistent eigenvalue computation.
  Eigen::MatrixXd D0s = 0.5 * (D0 + D0.transpose());
  Eigen::MatrixXd D1s = 0.5 * (D1 + D1.transpose());

  auto eval_norm = [&](double k) -> double {
    Eigen::MatrixXd Dk = D0s + k * D1s;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Dk,
        Eigen::EigenvaluesOnly);
    return eig.eigenvalues().cwiseAbs().maxCoeff();
  };

  // The feasible set {k >= 0 : ||d0 + k*d1|| <= 1} is an interval
  // [k_low, k_high].  We want k_high.
  //
  // Strategy: find a feasible k by sampling, then bisect rightward.

  // Find the largest k where ||D0 + k*D1||_inf <= 1.
  //
  // GEV: at a feasible k0 (||D0 + k0*D1|| < 1), the boundary is
  //   D1 v = lambda (I - D0') v  and  D1 v = lambda (I + D0') v
  // where D0' = D0 + k0*D1, giving dk = min positive 1/lambda.
  //
  // The nonneg elementwise bound is an UPPER bound (entries ≤ 1
  // does not imply eigenvalues ≤ 1).  Scale it down by 1/n to get
  // a feasible starting point, then refine with GEV.

  // Nonneg bound (upper bound for PSD).
  double k_nn = std::numeric_limits<double>::max();
  for (int i = 0; i < size; ++i) {
    double a = D0(i), b = D1(i);
    if (b > 1e-14)
      k_nn = std::min(k_nn, (1.0 - a) / b);
    else if (b < -1e-14)
      k_nn = std::min(k_nn, (-1.0 - a) / b);
  }
  if (k_nn < 0) k_nn = 0;
  if (k_nn > 1e15) return k_nn;

  // Scale down to get a feasible k0 where ||D0'|| < 1.
  // Factor of 1/n is conservative: ||M||_2 <= sqrt(n) * max|M_ij|
  // and nonneg ensures max|D0'_ij| <= 1, so ||D0'||_2 <= sqrt(n).
  // Scaling k by 1/sqrt(n) from the centered point isn't quite right
  // either. Just bisect down from k_nn until feasible.
  // Use symmetrized D0s, D1s throughout (D0, D1 may have slight asymmetry
  // from numerical operations; SelfAdjointEigenSolver requires exact symmetry).
  double k0 = k_nn;
  {
    // Find a feasible k0 by halving from k_nn toward 0.
    while (eval_norm(k0) > 1.0 - 1e-10 && k0 > 1e-15)
      k0 *= 0.5;
    if (eval_norm(k0) > 1.0) return 0;
  }

  Eigen::MatrixXd D0p = D0s + k0 * D1s;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

  double dk = std::numeric_limits<double>::max();
  {
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> gev(D1s, I - D0p);
    const auto& eigs = gev.eigenvalues();
    for (int i = 0; i < n; ++i) {
      if (eigs(i) > 1e-14)
        dk = std::min(dk, 1.0 / eigs(i));
    }
  }
  {
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> gev(D1s, I + D0p);
    const auto& eigs = gev.eigenvalues();
    for (int i = 0; i < n; ++i) {
      if (eigs(i) < -1e-14)
        dk = std::min(dk, -1.0 / eigs(i));
    }
  }

  double k_result = k0 + dk;
  // Verify: check that ||D0s + k_result * D1s|| <= 1.
  double norm_check = eval_norm(k_result);
  if (norm_check > 1.0 + 1e-6) {
    fprintf(stderr, "PSD lineSearchK BUG: k=%.6e but norm=%.6e (n=%d, "
                    "k_nn=%.4e, k0=%.4e, dk=%.4e)\n",
            k_result, norm_check, n, k_nn, k0, dk);
    // Fall back to bisection.
    double lo = k0, hi = k_result;
    for (int b = 0; b < 60; ++b) {
      double mid = 0.5 * (lo + hi);
      if (eval_norm(mid) <= 1.0) lo = mid; else hi = mid;
    }
    k_result = lo;
  }
  return k_result;
}

void PSDConeOps::project(double* out, const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);

  // Symmetrize (in case of numerical asymmetry).
  Eigen::MatrixXd Sym = 0.5 * (A + A.transpose());

  // Eigendecompose and clamp negative eigenvalues to zero.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Sym);
  Eigen::VectorXd lambdas = eig.eigenvalues().cwiseMax(0.0);
  Out = eig.eigenvectors() * lambdas.asDiagonal() *
        eig.eigenvectors().transpose();
  Symmetrize(Out);
}

// Sqrt-free geodesic update for PSD cone.
// W_new = W^{1/2} exp(α D) W^{1/2} where D = I + W^{1/2} S W^{1/2}.
// Rewrite: W_new = exp(α(I + WS)) · W  (avoids eigendecomposition of W).
// Proof: W^{1/2} exp(W^{1/2} M W^{1/2}) W^{1/2} = exp(WM)·W for any M.
void PSDConeOps::geodesicUpdateFromSlack(double* W_out, const double* W,
                                          double alpha, const double* slack,
                                          int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> Wm(W, n, n);
  Eigen::Map<const Eigen::MatrixXd> S(slack, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(W_out, n, n);

  // WS = W · S.
  Eigen::MatrixXd WS = Wm * S;

  // Argument to expm: α(I + WS).
  WS.diagonal().array() += 1.0;
  WS *= alpha;

  // W_new = expm(α(I + WS)) · W.
  Out = ExpmPade(WS) * Wm;
  Symmetrize(Out);
}

const PSDConeOps& psdConeOps() {
  static const PSDConeOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
