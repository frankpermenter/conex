// Geodesic integrators for log-homogeneous barrier cones.
//
// Three second-order, time-reversible integrators from integrator.tex:
//   1. Symmetric: implicit solve using H(z₀) and ∇φ
//   2. Primal midpoint: explicit half-step, gradient extrapolation
//   3. Dual midpoint: dual half-step, inverse gradient map
//
// Plus Yoshida 4th-order composition of the symmetric step.
//
// All use only BarrierConeOperations primitives (computeGradient,
// hessianProduct, hessian). No third derivatives needed.
//
// Current limitation: stack-allocated workspace, max segment size = 32.

#pragma once
#include "conex/common/symmetric_cone_operations.h"

namespace conex {

// Maximum segment dimension for stack-allocated workspace.
// 5*N + N*N doubles at N=32 = 1184 doubles ≈ 9.5KB.
constexpr int kMaxBarrierDim = 32;

// Symmetric geodesic step (Algorithm 1 in integrator.tex).
// Solves: ∇φ(z₁) + H(z₀)·z₁ = 2h·H(z₀)·v₀
// Updates z and vel in-place.
void symmetricStep(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                   double* z, double* vel, double h, int size);

// Primal midpoint step.
// z½ = z₀ + (h/2)·v₀, λ₁ = 2∇φ(z½) - ∇φ(z₀), z₁ = (∇φ)⁻¹(λ₁).
void primalMidpointStep(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                        double* z, double* vel, double h, int size);

// Dual midpoint step.
// λ½ = ∇φ(z₀) + (h/2)·w₀, z₁ = 2(∇φ)⁻¹(λ½) - z₀.
void dualMidpointStep(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                      double* z, double* vel, double h, int size);

// Yoshida 4th-order compositions of each base integrator.
void yoshida4Step(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                  double* z, double* vel, double h, int size);
void yoshida4PrimalStep(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                        double* z, double* vel, double h, int size);
void yoshida4DualStep(const EuclideanJordanAlgebra::BarrierConeOperations* ops,
                      double* z, double* vel, double h, int size);

}  // namespace conex
