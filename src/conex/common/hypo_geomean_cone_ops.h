// Barrier operations for the hypograph of geometric mean cone.
//
// K = { (u, w) : u <= (prod w_i)^{1/d}, w > 0 }
// where w in R^d and u in R.
//
// Barrier: F(u,w) = -log(geomean(w) - u) - sum log(w_i)
//        = -log(exp((1/d) sum log(w_i)) - u) - sum log(w_i)
// Barrier parameter: nu = dim = 1 + d.
//
// Storage: z = (u, w_1..w_d) directly.
// Dimension: size = 1 + d.

#pragma once

#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class HypoGeoMeanConeOps : public BarrierConeOpsThirdDeriv {
 public:
  void computeGradient(double* grad, const double* z,
                       int size) const override;
  void hessianProduct(double* out, const double* z, const double* v,
                      int size) const override;
  double barrierParameter(int size) const override;
  void getInteriorPoint(double* out, int size) const override;
  double barrierValue(const double* z, int size) const override;
  void thirdDerivContract(double* out, const double* z,
                          const double* v, int size) const override;
};

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
