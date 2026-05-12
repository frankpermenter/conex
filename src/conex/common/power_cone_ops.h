// Barrier operations for the generalized power cone.
//
// K = { (u, w) : prod(u_i^{alpha_i}) >= ||w||, u >= 0 }
// where alpha_i > 0, sum(alpha_i) = 1.
//
// Barrier: F(u,w) = -log((prod u_i^{alpha_i})^2 - ||w||^2) - sum log(u_i)
// Barrier parameter: nu = m + 2, where m = len(alpha).
//
// Storage: z = (u_1, ..., u_m, w_1, ..., w_n) directly (no reparameterization).
// Dimension: size = m + n.

#pragma once

#include <Eigen/Core>
#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class PowerConeOps : public BarrierConeOpsThirdDeriv {
 public:
  explicit PowerConeOps(const Eigen::VectorXd& alpha)
      : alpha_(alpha), m_(alpha.size()) {}

  void computeGradient(double* grad, const double* z,
                       int size) const override;
  void hessianProduct(double* out, const double* z, const double* v,
                      int size) const override;
  double barrierParameter(int size) const override;
  void getInteriorPoint(double* out, int size) const override;
  double barrierValue(const double* z, int size) const override;
  void thirdDerivContract(double* out, const double* z,
                          const double* v, int size) const override;

  const Eigen::VectorXd& alpha() const { return alpha_; }
  int m() const { return m_; }

 private:
  Eigen::VectorXd alpha_;
  int m_;  // number of u-variables

  // Helper: compute phi = (prod u_i^{alpha_i})^2 and s = phi - ||w||^2.
  // Returns {phi, s}. Assumes u_i > 0 and s > 0 (interior point).
  std::pair<double, double> computePhiS(const double* z, int size) const;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
