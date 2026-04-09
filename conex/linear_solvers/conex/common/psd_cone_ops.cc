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

void PSDConeOps::quadraticRepresentation(double* out, const double* a,
                                         const double* b, int size) const {
  int n = MatrixDim(size);
  Eigen::Map<const Eigen::MatrixXd> A(a, n, n);
  Eigen::Map<const Eigen::MatrixXd> B(b, n, n);
  Eigen::Map<Eigen::MatrixXd> Out(out, n, n);
  Out.noalias() = A * B * A;
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
