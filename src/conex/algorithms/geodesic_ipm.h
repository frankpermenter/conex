// Geodesic interior-point method for conic programs.
//
// Public API: result types and solver functions.
// Internal utilities (decomposition, direction evaluation, etc.) are in
// geodesic_ipm_helpers.h — include that header for unit-testing internals.

#pragma once
#include <functional>
#include <vector>
#include "conex/common/compiled_model.h"
#include "conex/common/solve_stats.h"

namespace conex {

// =====================================================================
// Result types
// =====================================================================

struct GeodesicIterStats {
  double mu;
  double d_inf;
  double d_sqr;
  double complementarity;  // mu * (rank - d_sqr)
  int r_updates = 0;       // r-updates since last factorization (hybrid)
  double min_slack = 0;    // min(r_i - |r_i * d_i|)
  double theta = 0;        // homotopy parameter (ThetaContR)
  int factorizations = 0;  // cumulative factorizations at this iteration
  double gap_error = 0;    // |mu*(nu-d_sqr) - (b'lam + c'x + x'Qx + d'nu)|
};

struct OptimalityReport {
  double primal_residual = 0;   // ||s - (Ax + b)||
  double dual_residual = 0;     // ||A^T λ - Qx - c||
  double complementarity = 0;   // <s, λ>
  double min_slack = 0;          // min eigenvalue of s
  double min_dual = 0;           // min eigenvalue of λ
  double mu = 0;
};

struct GeodesicResult {
  int iterations;
  double d_inf_norm;   // final ||d||_inf
  double d_sq_norm;    // final ||d||^2
  double mu;           // barrier parameter 1/k^2
  double tau = 1.0;    // homogenization parameter at termination
  double kappa = 0;    // theta/tau at termination (infeasibility certificate when large)
  double complementarity;  // mu * (rank - ||d||^2)
  bool infeasible = false; // true if tau→0 with finite kappa (infeasibility certificate)
  int total_factorizations = 0;
  int total_solves = 0;
  std::vector<double> x;  // primal variable: x = y/k from last Newton solve
  RowSpace lambda;     // dual variable (cone multipliers), reduced space
  std::vector<GeodesicIterStats> iter_stats;
  OptimalityReport optimality;
};

// =====================================================================
// Solver functions
// =====================================================================

// Geodesic centering at fixed barrier parameter k.
GeodesicResult GeodesicCenter(
    CompiledModel& model,
    RowSpace& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose = false);

// Full geodesic IPM: repeated line-search for k then center.
GeodesicResult SolveGeodesicLP(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations = 30,
    int max_centering_steps = 1,
    double tolerance = 1e-8,
    bool verbose = false,
    bool mehrotra_correction = false,
    SolveStats* stats = nullptr);

// Line search for k at current W.
double GeodesicLineSearch(
    CompiledModel& model,
    const RowSpace& W);

// Geodesic HSD: joint (τ, θ) selection via gap + normalization.
GeodesicResult SolveGeodesicHSD(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 30,
    double tolerance = 1e-8,
    bool verbose = false);

// θ-continuation geodesic IPM.
GeodesicResult SolveGeodesicThetaContinuation(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations = 50,
    int max_centering_steps = 10,
    double tolerance = 1e-8,
    bool verbose = false,
    SolveStats* stats = nullptr);

// Aggressive θ→0 / increase-k (phase-one then phase-two).
GeodesicResult SolveGeodesicPhaseOne(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations = 50,
    int max_centering_steps = 10,
    double tolerance = 1e-8,
    bool verbose = false,
    bool phase1_only = false);

// z-space geodesic LP for general log-homogeneous barriers.
GeodesicResult SolveGeodesicBarrierLP(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations = 30,
    int max_frozen_steps = 0,
    double tolerance = 1e-8,
    bool verbose = false,
    SolveStats* stats = nullptr);

// z-space θ-continuation for general log-homogeneous barriers.
GeodesicResult SolveGeodesicBarrierThetaContinuation(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations = 500,
    int max_centering_steps = 1,
    double tolerance = 1e-8,
    bool verbose = false,
    SolveStats* stats = nullptr);

// Switching policy for the hybrid algorithm.
using HybridSwitchPolicy = std::function<bool(double, double, int)>;
inline bool DefaultHybridPolicy(double gap, double, int) {
  return gap < 0;
}

// Hybrid geodesic IPM: alternates centering and r-shrinking.
GeodesicResult SolveGeodesicHybrid(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations = 50,
    double tolerance = 1e-8,
    bool verbose = false,
    double initial_k = -1,
    double tau = 1.0,
    HybridSwitchPolicy policy = DefaultHybridPolicy);

}  // namespace conex
