#pragma once
#include "conex/common/compiled_model.h"
#include "conex/algorithms/geodesic_ipm.h"

namespace conex {

// GeodesicHSDE: Homogeneous Self-Dual Embedding with geodesic updates.
//
// Like ThetaContR but with scalar k instead of per-component r:
//   s = (1/k) P(W^{-1/2})(e - d)
//   lambda = (1/k) P(W^{1/2})(e + d)
//   tau*kappa = r_tau^2*(1 - d_tau^2)
//
// Joint (tau, theta) from gap + normalization equations (same as ThetaContR).
// Theta = complementarity gap, not tied to k.
// k advanced via line search ||d0 + k*d1||_inf <= 1 (no refactor).
// W updated via geodesic step when gap < 0 (refactor).
GeodesicResult SolveGeodesicHSDE(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 500,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
