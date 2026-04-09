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
}  // namespace

void PSDConeOps::product(double* out, const double* a, const double* b,
                         int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  // Jordan product: (AB + BA) / 2.
  Out = 0.5 * (A * B + B * A);
}

void PSDConeOps::quotient(double* out, const double* a, const double* b,
                          int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out = A * B.inverse();
}

void PSDConeOps::geodesicUpdate(double* out, const double* a, double alpha,
                                const double* d, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);

  // Geodesic: A^{1/2} expm(alpha * D) A^{1/2}.
  // Compute A^{1/2} via eigendecomposition of A.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigA(A);
  Eigen::MatrixXd sqrtA = eigA.eigenvectors() *
      eigA.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eigA.eigenvectors().transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigD(alpha * D);
  Eigen::MatrixXd expD = eigD.eigenvectors() *
      eigD.eigenvalues().array().exp().matrix().asDiagonal() *
      eigD.eigenvectors().transpose();

  Out = sqrtA * expD * sqrtA;
}

void PSDConeOps::setIdentity(double* out, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out.setIdentity();
}

double PSDConeOps::normInf(const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A,
      Eigen::EigenvaluesOnly);
  return eig.eigenvalues().cwiseAbs().maxCoeff();
}

double PSDConeOps::squaredNorm(const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  return A.squaredNorm();  // Frobenius squared = trace(A^T A).
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
}

void PSDConeOps::quadraticRepresentation(double* out, const double* a,
                                         const double* b, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out.noalias() = A * B * A;
}

void PSDConeOps::solveLyapunov(double* out, const double* a, const double* d,
                               int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> R(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> D(d, n, n);
  Eigen::Map<Eigen::MatrixXd> Delta(out, n, n);
  // RD + DR = 2*Delta.  In R's eigenbasis: Delta_ij = (l_i + l_j)/2 * D_ij.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(R);
  const auto& V = eig.eigenvectors();
  const auto& lam = eig.eigenvalues();
  Eigen::MatrixXd D_eig = V.transpose() * D * V;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      D_eig(i, j) *= 0.5 * (lam(i) + lam(j));
  Delta = V * D_eig * V.transpose();
}

void PSDConeOps::abs(double* out, const double* a, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A);
  Out = eig.eigenvectors() *
      eig.eigenvalues().cwiseAbs().asDiagonal() *
      eig.eigenvectors().transpose();
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
  R = T.transpose() * R * T;
}

double PSDConeOps::lineSearchK(const double* d0, const double* d1,
                               int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> D0(d0, n, n);
  Eigen::Map<const Eigen::MatrixXd> D1(d1, n, n);

  // Find largest k > 0 with ||D0 + k*D1||_inf <= 1, where ||.||_inf
  // is the max absolute eigenvalue.  Bisect on k.
  // Use SelfAdjointEigenSolver (reads lower triangle) — same convention
  // as normInf.  Do NOT symmetrize, as d may have slight asymmetry.
  auto eval_norm = [&](double k) -> double {
    Eigen::MatrixXd Dk = D0 + k * D1;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Dk,
        Eigen::EigenvaluesOnly);
    return eig.eigenvalues().cwiseAbs().maxCoeff();
  };

  // The feasible set {k >= 0 : ||d0 + k*d1|| <= 1} is an interval
  // [k_low, k_high].  We want k_high.
  //
  // Strategy: find a feasible k by sampling, then bisect rightward.

  // Find the feasible interval {k >= 0 : ||d(k)|| <= 1} and return
  // the upper boundary.  Use the nonneg (elementwise) line search
  // as a cheap initial estimate, then refine with eigenvalue checks.
  Eigen::Map<const Eigen::VectorXd> d0_vec(d0, size);
  Eigen::Map<const Eigen::VectorXd> d1_vec(d1, size);

  // Nonneg bound: largest k with |d0_i + k*d1_i| <= 1 for all i.
  double k_nn = std::numeric_limits<double>::max();
  for (int i = 0; i < size; ++i) {
    double a = d0_vec(i), b = d1_vec(i);
    if (b > 1e-14)
      k_nn = std::min(k_nn, (1.0 - a) / b);
    else if (b < -1e-14)
      k_nn = std::min(k_nn, (-1.0 - a) / b);
  }
  if (k_nn <= 0) k_nn = 0;

  // The nonneg bound is a lower bound on the PSD answer (elementwise
  // feasibility implies eigenvalue feasibility for diagonal matrices,
  // and is generally conservative).  Start bisection from there.
  double lo = k_nn;
  if (eval_norm(lo) > 1.0) {
    // Nonneg estimate infeasible for eigenvalues — bisect down.
    double hi = lo;
    lo = 0;
    for (int iter = 0; iter < 60; ++iter) {
      double mid = 0.5 * (lo + hi);
      if (eval_norm(mid) <= 1.0) lo = mid; else hi = mid;
    }
  }

  // Bisect rightward from lo to find the upper boundary.
  double hi = std::max(lo + 1.0, lo * 2);
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
}

const PSDConeOps& psdConeOps() {
  static const PSDConeOps instance;
  return instance;
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
