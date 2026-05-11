// Cone operations for the positive semidefinite cone.
// Segments store n*n doubles as a column-major n×n matrix.

#pragma once
#include <cmath>
#include "conex/common/symmetric_cone_operations.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class PSDConeOps : public SymmetricConeOperations {
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
  void solveLyapunovForD(double* out, const double* r, const double* delta,
                         int size) const override;
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

  // Sqrt-free geodesic: W_new = exp(α(I + WS)) · W via Padé.
  void geodesicUpdateFromSlack(double* W_out, const double* W,
                               double alpha, const double* slack,
                               int size) const override;

  // PSD barrier parameter: ν = n (matrix dimension), size = n².
  double barrierParameter(int size) const override {
    int n = static_cast<int>(std::round(std::sqrt(size)));
    return static_cast<double>(n);
  }
};

const PSDConeOps& psdConeOps();

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
