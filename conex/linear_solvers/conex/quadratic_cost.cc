#include "conex/quadratic_cost.h"
namespace conex {
double SupernodalAssemblerQuadratic::EvaluateQuadraticCost(
    const Eigen::Ref<const Eigen::MatrixXd> x) const {
  const Eigen::VectorXd xtemp = PrimalSubvector(x);
  return xtemp.dot(A_ * xtemp);
}
}  // namespace conex
