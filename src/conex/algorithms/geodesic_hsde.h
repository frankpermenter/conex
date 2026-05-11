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
// Joint (tau, theta) from gap + normalization equations (2x2 linear solve).
// k advanced via affine line search (d(k) is affine in k).
// W updated via geodesic step (refactor every outer iteration).
// Optional frozen-Jacobian inner steps advance k without refactoring.
GeodesicResult SolveGeodesicHSDE(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 500,
    int max_frozen_steps = 0,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
