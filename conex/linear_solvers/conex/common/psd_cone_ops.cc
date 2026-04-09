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
  // Strategy: first find any k_feas where ||d|| <= 1 (the minimum-norm
  // point), then search rightward for the upper boundary.

  // Find the minimum of ||d(k)|| via golden section on [0, k_big].
  // This gives a k_feas inside the feasible interval.
  double k_big = 1;
  while (eval_norm(k_big) < eval_norm(k_big / 2) && k_big < 1e15)
    k_big *= 2;

  // Golden section search for the minimizer.
  double a = 0, b = k_big;
  const double phi = 0.5 * (std::sqrt(5.0) - 1.0);
  double x1 = b - phi * (b - a), x2 = a + phi * (b - a);
  double f1 = eval_norm(x1), f2 = eval_norm(x2);
  for (int iter = 0; iter < 60; ++iter) {
    if (f1 < f2) {
      b = x2; x2 = x1; f2 = f1;
      x1 = b - phi * (b - a); f1 = eval_norm(x1);
    } else {
      a = x1; x1 = x2; f1 = f2;
      x2 = a + phi * (b - a); f2 = eval_norm(x2);
    }
  }
  double k_min = 0.5 * (a + b);
  if (eval_norm(k_min) > 1.0) return 0;  // no feasible k

  // Bisect rightward from k_min to find k_high.
  double lo = k_min, hi = std::max(k_min * 2, 1.0);
  while (eval_norm(hi) <= 1.0) {
    lo = hi;
    hi *= 2;
    if (hi > 1e15) return hi;
  }
  for (int iter = 0; iter < 60; ++iter) {
    double mid = 0.5 * (lo + hi);
    if (eval_norm(mid) <= 1.0)
      lo = mid;
    else
      hi = mid;
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
