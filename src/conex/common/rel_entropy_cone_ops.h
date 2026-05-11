// Barrier operations for the relative entropy cone.
//
// K = { (u, v, w) : u >= sum w_i * log(w_i / v_i), v > 0, w > 0 }
// where v, w in R^d and u in R.
//
// Barrier: F(u,v,w) = -log(u - sum w_i*log(w_i/v_i)) - sum log(v_i) - sum log(w_i)
// Barrier parameter: nu = 1 + 2d = dim.
//
// Storage: z = (u, v_1..v_d, w_1..w_d) directly.
// Dimension: size = 1 + 2*d.

#pragma once

#include <Eigen/Dense>
#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class RelEntropyConeOps : public BarrierConeOperations {
 public:
  void computeGradient(double* grad, const double* z,
                       int size) const override;
  void hessianProduct(double* out, const double* z, const double* v,
                      int size) const override;
  double barrierParameter(int size) const override;
  void getInteriorPoint(double* out, int size) const override;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
