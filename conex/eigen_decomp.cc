#include "eigen_decomp.h"

namespace conex {
namespace jordan_algebra {


EigenvalueDecomposition eig(const Eigen::MatrixXd& x) {
  //conex::jordan_algebra::SpectralDecompSymmetricMatrices<d> spec;
  //spec.Compute(x);
  Eigen::EigenSolver<Eigen::MatrixXd> spec;
  spec.compute(x);
  EigenvalueDecomposition output;
  output.eigenvalues = spec.eigenvalues().array().real();
  // output.eigenvectors = spec.eigenvectors();
  return output;
}


Eigen::MatrixXd Log(const Eigen::MatrixXd& X) {
  auto d = eig(X);
  Eigen::MatrixXd Y(X.rows(), X.rows());
  Y.setZero();
  for (int i = 0; i < d.eigenvectors.cols(); i++) {
    Y += log(d.eigenvalues(i)) *  d.eigenvectors.col(i) *  d.eigenvectors.col(i).transpose();
  }
  return Y;
}

Eigen::MatrixXd ExpMap(const Eigen::MatrixXd& X) {
  auto d = eig(X);
  Eigen::MatrixXd Y(X.rows(), X.rows());
  Y.setZero();
  for (int i = 0; i < d.eigenvectors.cols(); i++) {
    Y += exp(d.eigenvalues(i)) *  d.eigenvectors.col(i) *  d.eigenvectors.col(i).transpose();
  }
  return Y;
}

Eigen::MatrixXd Sqrt(const Eigen::MatrixXd& X) {
  auto d = eig(X);
  Eigen::MatrixXd Y(X.rows(), X.rows());
  Y.setZero();
  for (int i = 0; i < d.eigenvectors.cols(); i++) {
    Y += sqrt(d.eigenvalues(i)) *  d.eigenvectors.col(i) *  d.eigenvectors.col(i).transpose();
  }
  return Y;
}

Eigen::MatrixXd Mean(const Eigen::MatrixXd& S, const Eigen::MatrixXd& Z) {
  auto Zsqrt = Sqrt(Z);
  Eigen::MatrixXd Zsqrtinv = Zsqrt.inverse();
  return Zsqrt * Sqrt(Zsqrtinv*S*Zsqrtinv) *Zsqrt;
}

std::pair<double, double> SpectralRadius(const Eigen::MatrixXd& X) {
  std::pair<double, double> y;
  y.first = eig(X).eigenvalues.maxCoeff();
  y.second = eig(X).eigenvalues.minCoeff();
  return y;
}

double NormInf(const Eigen::MatrixXd& X) {
//  Eigen::MatrixXd b = Eigen::MatrixXd::Random(X.cols(), 1);
//  b.normalize();
//  DUMP(b);
//  double lambda = 0;
//  for (int i = 0; i < 10; i++) {
//    b = X*b;
//    double lamb = b.norm();
//    b.normalize();
//    DUMP(lamb);
//  }

  



  auto r = SpectralRadius(X);
  // return r.first; 
  double n = std::fabs(r.first);
  double n2 = std::fabs(r.second);
  if (n2 > n) {
    n = n2;
  }
  return n;
}

double NormInfPowerMethod(Ref* X, Ref* temp1) {
  return NormInf(*X);
  auto temp2 = X;

  temp1->noalias() = (*temp2)*(*temp2);
  double norm_x_sqr = temp1->trace();

  temp2->noalias() = (*temp1)*(*temp1);
  double norm_xn_sqr = temp2->trace(); 
  // x^4 / x^2
  return sqrt(norm_xn_sqr /  norm_x_sqr);
}





}  // namespace jordan_algebra
}  // namespace conex