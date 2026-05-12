// Barrier operations for box constraints: l ≤ x ≤ u.
//
// Barrier: F(x) = -Σ log(x_i - l_i) - Σ log(u_i - x_i)
// Barrier parameter: ν = 2n (n lower + n upper bounds).
// Hessian: diagonal, H_ii = 1/(x_i - l_i)² + 1/(u_i - x_i)².
//
// Storage: z = x directly. Dimension: size = n.
// Bounds are stored in the ops instance (not in z).

#pragma once

#include <Eigen/Dense>
#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class BoxConeOps : public BarrierConeOperations {
 public:
  BoxConeOps(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
      : lower_(lower), upper_(upper) {}

  void computeGradient(double* grad, const double* z,
                       int size) const override;
  void hessianProduct(double* out, const double* z, const double* v,
                      int size) const override;
  double barrierParameter(int size) const override;
  void getInteriorPoint(double* out, int size) const override;
  double barrierValue(const double* z, int size) const override;

  // Performance overrides: exploit diagonal Hessian.
  double hessianNormSquared(const double* z, const double* target,
                            int size) const override;
  double stepSize(const double* z, const double* target,
                  int size) const override;
  double lineSearchTarget(const double* z, const double* target0,
                           const double* target1, int size) const override;

  const Eigen::VectorXd& lower() const { return lower_; }
  const Eigen::VectorXd& upper() const { return upper_; }

 private:
  Eigen::VectorXd lower_, upper_;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
