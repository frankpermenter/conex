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
  double k0 = k_nn;
  {
    auto eval_norm = [&](double k) -> double {
      Eigen::MatrixXd Dk = D0 + k * D1;
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(Dk,
          Eigen::EigenvaluesOnly);
      return eig.eigenvalues().cwiseAbs().maxCoeff();
    };
    // Find a feasible k0 by halving from k_nn toward 0.
    while (eval_norm(k0) > 1.0 - 1e-10 && k0 > 1e-15)
      k0 *= 0.5;
    if (eval_norm(k0) > 1.0) return 0;
  }

  Eigen::MatrixXd D0p = D0 + k0 * D1;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

  double dk = std::numeric_limits<double>::max();
  {
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> gev(D1, I - D0p);
    const auto& eigs = gev.eigenvalues();
    for (int i = 0; i < n; ++i) {
      if (eigs(i) > 1e-14)
        dk = std::min(dk, 1.0 / eigs(i));
    }
  }
  {
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> gev(D1, I + D0p);
    const auto& eigs = gev.eigenvalues();
    for (int i = 0; i < n; ++i) {
      if (eigs(i) < -1e-14)
        dk = std::min(dk, -1.0 / eigs(i));
    }
  }

  return k0 + dk;
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
