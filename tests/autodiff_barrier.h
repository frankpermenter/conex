// Autodiff barrier oracle for testing cone implementations.
//
// Each cone provides a barrier function template parameterized on the
// scalar type. The .cc file instantiates with Eigen's AutoDiffScalar
// to get exact gradient, Hessian, and third derivatives. Tests include
// only this header (no autodiff templates).
//
// To add a new cone barrier:
//   1. Declare it here (e.g., autodiff_exp_cone_gradient).
//   2. Implement the barrier template and AD wrappers in the .cc file.

#pragma once
#include <Eigen/Dense>

namespace conex {
namespace testing {

// ============================================================
// Exponential cone: F(x,y,z) = -log(z - y*exp(x/y)) - log(y)
// ============================================================
double exp_cone_barrier(const Eigen::VectorXd& z);
Eigen::VectorXd exp_cone_gradient(const Eigen::VectorXd& z);
Eigen::MatrixXd exp_cone_hessian(const Eigen::VectorXd& z);
Eigen::VectorXd exp_cone_third_deriv(const Eigen::VectorXd& z,
                                      const Eigen::VectorXd& v);

// ============================================================
// Power cone: F(u,w) = -log((prod u_i^a_i)^2 - ||w||^2) - sum log(u_i)
// ============================================================
double power_cone_barrier(const Eigen::VectorXd& z,
                          const Eigen::VectorXd& alpha);
Eigen::VectorXd power_cone_gradient(const Eigen::VectorXd& z,
                                     const Eigen::VectorXd& alpha);
Eigen::MatrixXd power_cone_hessian(const Eigen::VectorXd& z,
                                    const Eigen::VectorXd& alpha);
Eigen::VectorXd power_cone_third_deriv(const Eigen::VectorXd& z,
                                        const Eigen::VectorXd& alpha,
                                        const Eigen::VectorXd& v);

// ============================================================
// Relative entropy: F(u,v,w) = -log(u - sum w_i*log(w_i/v_i))
//                               - sum log(v_i) - sum log(w_i)
// ============================================================
double rel_entropy_barrier(const Eigen::VectorXd& z);
Eigen::VectorXd rel_entropy_gradient(const Eigen::VectorXd& z);
Eigen::MatrixXd rel_entropy_hessian(const Eigen::VectorXd& z);
Eigen::VectorXd rel_entropy_third_deriv(const Eigen::VectorXd& z,
                                         const Eigen::VectorXd& v);

// ============================================================
// Hypo geometric mean: F(u,w) = -log(geomean(w) - u) - sum log(w_i)
// ============================================================
double hypo_geomean_barrier(const Eigen::VectorXd& z);
Eigen::VectorXd hypo_geomean_gradient(const Eigen::VectorXd& z);
Eigen::MatrixXd hypo_geomean_hessian(const Eigen::VectorXd& z);
Eigen::VectorXd hypo_geomean_third_deriv(const Eigen::VectorXd& z,
                                          const Eigen::VectorXd& v);

}  // namespace testing
}  // namespace conex
