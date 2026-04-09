// Cone operations for the positive semidefinite cone.
// Segments store n*n doubles as a column-major n×n matrix.

#pragma once
#include "conex/common/cone_ops.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class PSDConeOps : public ConeOps {
 public:
  void product(double* out, const double* a, const double* b,
               int size) const override;
  void quotient(double* out, const double* a, const double* b,
                int size) const override;
  void geodesicUpdate(double* out, const double* a, double alpha,
                      const double* d, int size) const override;
  void setIdentity(double* out, int size) const override;
  double normInf(const double* a, int size) const override;
  double squaredNorm(const double* a, int size) const override;
  double dot(const double* a, const double* b, int size) const override;
  void project(double* out, const double* a, int size) const override;
};

const PSDConeOps& psdConeOps();

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
