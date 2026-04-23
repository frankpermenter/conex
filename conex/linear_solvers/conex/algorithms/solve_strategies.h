#pragma once
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"

namespace conex {

// Strategy: geodesic HSD (joint tau/theta selection).
struct GeodesicHSD {
  double tolerance = 1e-8;
  int max_iterations = 30;
  bool verbose = false;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHSD(
        kkt, cost_rhs, W, max_iterations, tolerance, verbose);
  }
};

// Strategy: θ-continuation geodesic IPM.
struct ThetaContinuation {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicThetaContinuation(
        kkt, cost_rhs, W, max_iterations, max_centering_steps,
        tolerance, verbose);
  }
};

// Strategy: aggressive θ→0 / increase-k method.
struct PhaseOne {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicPhaseOne(
        kkt, cost_rhs, W, max_iterations, max_centering_steps,
        tolerance, verbose);
  }
};

// Strategy: PhaseOne to θ=0, then hybrid r-update.
// The switching policy controls when to center (W-update) vs shrink (r-update).
struct PhaseOneHybrid {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;
  HybridSwitchPolicy policy = DefaultHybridPolicy;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    auto p1 = SolveGeodesicPhaseOne(
        kkt, cost_rhs, W, max_iterations, max_centering_steps,
        tolerance, verbose, /*phase1_only=*/true);
    double k_init = (p1.mu > 0) ? 1.0 / std::sqrt(p1.mu) : -1;
    auto result = SolveGeodesicHybrid(
        kkt, cost_rhs, W, max_iterations, tolerance, verbose,
        k_init, p1.tau, policy);
    result.total_factorizations += p1.total_factorizations;
    result.total_solves += p1.total_solves;
    return result;
  }
};

// Strategy: Hybrid only (no PhaseOne warmup). Uses the hybrid's own
// initial k line search. Starts from W=I.
struct HybridOnly {
  double tolerance = 1e-8;
  int max_iterations = 500;
  bool verbose = false;
  HybridSwitchPolicy policy = DefaultHybridPolicy;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHybrid(
        kkt, cost_rhs, W, max_iterations, tolerance, verbose,
        /*initial_k=*/-1, /*tau=*/1.0, policy);
  }
};

// Strategy: geodesic LP (line-search for k, then center).
struct GeodesicLP {
  double tolerance = 1e-8;
  int max_iterations = 30;
  int max_centering_steps = 0;
  bool verbose = false;

  GeodesicResult Run(KKTSolverBase& kkt,
                     const SolverRHS& cost_rhs) const {
    RowSpace W = kkt.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicLP(
        kkt, cost_rhs, W, max_iterations, max_centering_steps,
        tolerance, verbose);
  }
};

}  // namespace conex
