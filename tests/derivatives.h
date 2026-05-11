// Generic derivative computation using Eigen AutoDiffScalar.
//
// The heavy AD machinery (lift, extract) is compiled once in derivatives.cc.
// Barrier functions are templates on the scalar type — instantiated at
// the call site (test file), not here.
//
// Usage:
//   // In barrier_functions.h (pure math, no AD headers needed):
//   template <typename S>
//   S my_barrier(const Eigen::Matrix<S, Eigen::Dynamic, 1>& z) {
//     return -log(z(0));
//   }
//
//   // In test:
//   auto grad = derivatives::gradient(my_barrier<AD1>, z);
//   auto H    = derivatives::hessian(my_barrier<AD2>, z);
//   auto T    = derivatives::third_deriv(my_barrier<AD3>, z, v);

#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

namespace derivatives {

// --- AD type aliases (public so barriers can use them) ---
using AD1 = Eigen::AutoDiffScalar<Eigen::VectorXd>;
using AV1 = Eigen::Matrix<AD1, Eigen::Dynamic, 1>;

using AD2 = Eigen::AutoDiffScalar<Eigen::Matrix<AD1, Eigen::Dynamic, 1>>;
using AV2 = Eigen::Matrix<AD2, Eigen::Dynamic, 1>;

using AD3 = Eigen::AutoDiffScalar<Eigen::Matrix<AD2, Eigen::Dynamic, 1>>;
using AV3 = Eigen::Matrix<AD3, Eigen::Dynamic, 1>;

// --- Function pointer types for each derivative level ---
using GradFn = AD1 (*)(const AV1&);
using HessFn = AD2 (*)(const AV2&);
using ThirdFn = AD3 (*)(const AV3&);

// --- Derivative computation (compiled once in derivatives.cc) ---
Eigen::VectorXd gradient(GradFn f, const Eigen::VectorXd& z);
Eigen::MatrixXd hessian(HessFn f, const Eigen::VectorXd& z);
Eigen::VectorXd third_deriv_contract(ThirdFn f, const Eigen::VectorXd& z,
                                      const Eigen::VectorXd& v);

// --- Helper: scale AD scalar by double (works at any nesting depth) ---
inline double scale(double c, double x) { return c * x; }
inline AD1 scale(double c, const AD1& x) {
  return AD1(c * x.value(), (c * x.derivatives()).eval());
}
inline AD2 scale(double c, const AD2& x) {
  Eigen::Matrix<AD1, Eigen::Dynamic, 1> sd(x.derivatives().size());
  for (int i = 0; i < sd.size(); ++i) sd(i) = scale(c, x.derivatives()(i));
  return AD2(scale(c, x.value()), sd);
}
inline AD3 scale(double c, const AD3& x) {
  Eigen::Matrix<AD2, Eigen::Dynamic, 1> sd(x.derivatives().size());
  for (int i = 0; i < sd.size(); ++i) sd(i) = scale(c, x.derivatives()(i));
  return AD3(scale(c, x.value()), sd);
}

}  // namespace derivatives
