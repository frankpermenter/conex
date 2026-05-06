// Cone operations for the second-order (Lorentz) cone.
// Segments store (t, x₁, ..., xₙ) where t is scalar, x is n-vector.
// The cone is {(t,x) : ||x|| ≤ t}.
// Eigenvalues: λ₁ = t + ||x||, λ₂ = t - ||x||.

#pragma once
#include "conex/common/cone_ops.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class SOCConeOps : public ConeOps {
 public:
  void product(double* out, const double* a, const double* b,
               int size) const override;
  void geodesicUpdate(double* out, const double* a, double alpha,
                      const double* d, int size) const override;
  void setIdentity(double* out, int size) const override;
  double normInf(const double* a, int size) const override;
  double squaredNorm(const double* a, int size) const override;
  double dot(const double* a, const double* b, int size) const override;
  void sqrt(double* out, const double* a, int size) const override;
  void quadraticRepresentation(double* out, const double* a,
                               const double* b, int size) const override;
  void solveLyapunovForD(double* out, const double* r,
                         const double* delta, int size) const override;
  void abs(double* out, const double* a, int size) const override;
  double minEigenvalue(const double* a, int size) const override;
  void updateAutomorphism(double* w, double* r, double alpha,
                          const double* d, int size) const override;
  void updateAutomorphismP(double* p, double* r, double alpha,
                           const double* d, int size) const override;
  void updateM(double* m, double* r, double alpha,
               const double* d, int size) const override;
  void applyM(double* out, const double* m,
              const double* x, int size) const override;
  void applyMt(double* out, const double* m,
               const double* x, int size) const override;
  void squareM(double* w, const double* m, int size) const override;
  double lineSearchK(const double* d0, const double* d1,
                     int size) const override;
  void project(double* out, const double* a, int size) const override;
};

const SOCConeOps& socConeOps();

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
