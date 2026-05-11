// Default implementations for BarrierConeOperations and BarrierConeOpsThirdDeriv.

#include "conex/common/symmetric_cone_operations.h"
#include "conex/common/barrier_integrators.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace conex {
namespace EuclideanJordanAlgebra {

// Default isInterior: check that gradient is finite.
// For most barriers, ∇φ(z) → ∞ as z → ∂K, so isfinite(grad) implies interior.
// Cones where this is insufficient (e.g., exp cone) should override.
bool BarrierConeOperations::isInterior(const double* z, int size) const {
  std::vector<double> grad(size);
  computeGradient(grad.data(), z, size);
  for (int i = 0; i < size; ++i)
    if (!std::isfinite(grad[i])) return false;
  return true;
}

// Generic Störmer-Verlet geodesic integrator.
// Uses thirdDerivContract for the geodesic acceleration:
//   z̈ = -½ H(z)⁻¹ D³F[ż, ż, ·]
//
// The direction is d = target - z (in z-space). We integrate the
// geodesic ODE for parameter t ∈ [0, α] with velocity ż(0) = d.
void BarrierConeOpsThirdDeriv::geodesicStepTarget(
    double* z, double alpha, const double* target, int size) const {
  std::vector<double> vel(size), pos(size), T(size), Hv(size);

  // Initial conditions: pos = z, vel = alpha * (target - z).
  for (int i = 0; i < size; ++i) {
    pos[i] = z[i];
    vel[i] = alpha * (target[i] - z[i]);
  }

  // Adaptive substeps based on Riemannian speed ||vel||_H.
  hessianProduct(Hv.data(), pos.data(), vel.data(), size);
  double speed_sq = 0;
  for (int i = 0; i < size; ++i) speed_sq += vel[i] * Hv[i];
  double speed = std::sqrt(std::max(speed_sq, 0.0));
  int steps = std::max(1, (int)std::ceil(speed));
  double dt = 1.0 / steps;

  // Scale velocity for substeps.
  for (int i = 0; i < size; ++i) vel[i] *= dt;

  for (int step = 0; step < steps; ++step) {
    // Compute acceleration: a = -½ H(z)⁻¹ T where T_l = v^T (∂H/∂z_l) v.
    // We solve H(z) a = -½ T via a dense Hessian factorization.
    thirdDerivContract(T.data(), pos.data(), vel.data(), size);

    // Form H(pos) as a dense matrix and solve H*a = -½T.
    std::vector<double> H_dense(size * size);
    hessian(H_dense.data(), pos.data(), size);

    // Solve via Eigen (small n for barrier cones).
    Eigen::Map<Eigen::MatrixXd> H(H_dense.data(), size, size);
    Eigen::Map<Eigen::VectorXd> Tv(T.data(), size);
    Eigen::VectorXd accel = H.ldlt().solve(-0.5 * Tv);

    // Störmer-Verlet: half-kick, drift, half-kick.
    for (int i = 0; i < size; ++i) vel[i] += 0.5 * accel(i);
    for (int i = 0; i < size; ++i) pos[i] += vel[i];

    // Second half-kick with updated position.
    thirdDerivContract(T.data(), pos.data(), vel.data(), size);
    hessian(H_dense.data(), pos.data(), size);
    Eigen::Map<Eigen::MatrixXd> H2(H_dense.data(), size, size);
    accel = H2.ldlt().solve(-0.5 * Tv);
    for (int i = 0; i < size; ++i) vel[i] += 0.5 * accel(i);
  }

  // Write result.
  for (int i = 0; i < size; ++i) z[i] = pos[i];
}

}  // namespace EuclideanJordanAlgebra
}  // namespace conex
