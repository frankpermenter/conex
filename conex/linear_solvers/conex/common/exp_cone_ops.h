// Cone operations for the exponential cone.
// The exponential cone is K = cl{(x, y, z) : y*exp(x/y) <= z, y > 0}.
// Barrier: F(x,y,z) = -log(z - y*exp(x/y)) - log(y).
// Segments store 3 doubles: (x, y, z).

#pragma once
#include "conex/common/cone_ops.h"

namespace conex {
namespace EuclideanJordanAlgebra {

class ExpConeOps : public ConeOps {
 public:
  // Barrier function and derivatives.
  static double Barrier(double x, double y, double z);
  static void BarrierGrad(double x, double y, double z, double* g);
  static void BarrierHessian(double x, double y, double z, double* H);
  // Third derivative contraction: T_l = v^T (dH/dx_l) v, l=0,1,2.
  static void ThirdDerivContract(double x, double y, double z,
                                 const double* v, double* T);

  // Geodesic step: integrate geodesic ODE for the Hessian metric.
  void geodesicStep(double* w, double alpha, const double* d) const;

  // Bregman midpoint step: second-order approximation to the Levi-Civita
  // geodesic using primal and dual flat structures.  No third derivatives.
  void bregmanMidpointStep(double* w, double alpha, const double* d) const;

  // (s, λ) leapfrog: symplectic geodesic integrator using only H and H⁻¹.
  // Exploits log-homogeneity identities: λ = Hs, λ̇ = -Hṡ, λ̈ = Hs̈.
  // No third derivatives, no Newton solve.
  void leapfrogStep(double* w, double alpha, const double* d) const;

  // Invert the gradient map: given lambda, find x such that -grad F(x) = lambda.
  // Uses Newton's method (3x3 system, typically 3-5 iterations).
  static bool InvertGradient(const double* lambda, double* x, int max_iter = 20);

  // ConeOps interface — most are not meaningful for exp cone
  // since it's not a symmetric cone. Stubs for compilation.
  void product(double*, const double*, const double*, int) const override {}
  void geodesicUpdate(double*, const double*, double, const double*, int) const override;
  void setIdentity(double* out, int size) const override;
  double normInf(const double* a, int size) const override;
  double squaredNorm(const double* a, int size) const override;
  double dot(const double* a, const double* b, int size) const override;
  void sqrt(double*, const double*, int) const override {}
  void quadraticRepresentation(double*, const double*, const double*, int) const override {}
  void solveLyapunovForD(double*, const double*, const double*, int) const override {}
  void abs(double*, const double*, int) const override {}
  double minEigenvalue(const double* a, int size) const override;
  void updateAutomorphism(double*, double*, double, const double*, int) const override {}
  void updateAutomorphismP(double*, double*, double, const double*, int) const override {}
  void updateM(double*, double*, double, const double*, int) const override {}
  void applyM(double*, const double*, const double*, int) const override {}
  void applyMt(double*, const double*, const double*, int) const override {}
  void squareM(double*, const double*, int) const override {}
  double lineSearchK(const double*, const double*, int) const override { return 0; }
  void project(double* out, const double* a, int size) const override;
};

const ExpConeOps& expConeOps();

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
