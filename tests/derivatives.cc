// Compiled once — contains the AD lift/extract machinery.
// Barrier templates are NOT instantiated here.

#include "derivatives.h"

namespace derivatives {

// --- Lift helpers (create AD variables with identity seeds) ---

static AV1 lift1(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV1 az(n);
  for (int i = 0; i < n; ++i) {
    az(i).value() = z(i);
    az(i).derivatives() = Eigen::VectorXd::Unit(n, i);
  }
  return az;
}

static AV2 lift2(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az(n);
  for (int i = 0; i < n; ++i) {
    az(i).value().value() = z(i);
    az(i).value().derivatives() = Eigen::VectorXd::Unit(n, i);
    az(i).derivatives().resize(n);
    for (int j = 0; j < n; ++j) {
      az(i).derivatives()(j).value() = (i == j) ? 1.0 : 0.0;
      az(i).derivatives()(j).derivatives() = Eigen::VectorXd::Zero(n);
    }
  }
  return az;
}

static AV3 lift3(const Eigen::VectorXd& z) {
  const int n = z.size();
  AV3 az(n);
  for (int i = 0; i < n; ++i) {
    az(i).value().value().value() = z(i);
    az(i).value().value().derivatives() = Eigen::VectorXd::Unit(n, i);
    az(i).value().derivatives().resize(n);
    for (int j = 0; j < n; ++j) {
      az(i).value().derivatives()(j).value() = (i == j) ? 1.0 : 0.0;
      az(i).value().derivatives()(j).derivatives() = Eigen::VectorXd::Zero(n);
    }
    az(i).derivatives().resize(n);
    for (int l = 0; l < n; ++l) {
      az(i).derivatives()(l).value().value() = (i == l) ? 1.0 : 0.0;
      az(i).derivatives()(l).value().derivatives() = Eigen::VectorXd::Zero(n);
      az(i).derivatives()(l).derivatives().resize(n);
      for (int k = 0; k < n; ++k) {
        az(i).derivatives()(l).derivatives()(k).value() = 0.0;
        az(i).derivatives()(l).derivatives()(k).derivatives() =
            Eigen::VectorXd::Zero(n);
      }
    }
  }
  return az;
}

// --- Public API ---

Eigen::VectorXd gradient(GradFn f, const Eigen::VectorXd& z) {
  AV1 az = lift1(z);
  AD1 result = f(az);
  return result.derivatives();
}

Eigen::MatrixXd hessian(HessFn f, const Eigen::VectorXd& z) {
  const int n = z.size();
  AV2 az = lift2(z);
  AD2 result = f(az);
  Eigen::MatrixXd H(n, n);
  for (int j = 0; j < n; ++j)
    H.col(j) = result.derivatives()(j).derivatives();
  return 0.5 * (H + H.transpose());
}

Eigen::VectorXd third_deriv_contract(ThirdFn f, const Eigen::VectorXd& z,
                                      const Eigen::VectorXd& v) {
  const int n = z.size();
  AV3 az = lift3(z);
  AD3 result = f(az);
  Eigen::VectorXd T(n);
  for (int l = 0; l < n; ++l) {
    double tl = 0;
    for (int i = 0; i < n; ++i) {
      const auto& row = result.derivatives()(l).derivatives()(i).derivatives();
      for (int j = 0; j < n; ++j)
        tl += v(i) * v(j) * row(j);
    }
    T(l) = tl;
  }
  return T;
}

}  // namespace derivatives
