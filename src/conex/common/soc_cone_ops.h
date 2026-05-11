// Cone operations for the second-order (Lorentz) cone.
// Segments store (t, x₁, ..., xₙ) where t is scalar, x is n-vector.
// The cone is {(t,x) : ||x|| ≤ t}.
// Eigenvalues: λ₁ = t + ||x||, λ₂ = t - ||x||.

#pragma once
#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class SOCConeOps : public SymmetricConeOperations {
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
  void inverse(double* out, const double* a, int size) const override;
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

  // SOC barrier parameter is always 2 (rank of the Jordan algebra).
  double barrierParameter(int /*size*/) const override { return 2.0; }

  // SOC barrier value: -log(w₀² - ||w₁||²).
  double barrierValue(const double* z, int size) const override {
    double det = z[0] * z[0];
    for (int i = 1; i < size; ++i) det -= z[i] * z[i];
    return -std::log(det);
  }
};

const SOCConeOps& socConeOps();

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
