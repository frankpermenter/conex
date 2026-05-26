#pragma once
#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_hsde.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"

namespace conex {

// Strategy: theta-continuation with r-updates and tau from duality identity.
struct ThetaContinuationR {
  double tolerance = 1e-8;
  int max_iterations = 500;
  bool verbose = false;
  ThetaContRSwitchPolicy policy = DefaultThetaContRPolicy;
  double compl_tol = 1e-12;
  double theta_rate = 0.1;  // center if theta hasn't decreased by this factor
  int max_r_updates = 0;    // force W-update after this many r-updates (0 = disabled)
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicThetaContinuationR(
        model, W, max_iterations, tolerance, verbose, policy,
        compl_tol, theta_rate, &stats, max_r_updates);
  }
};

// Strategy: hybrid r-updates with theta = |gap|/m.
struct HybridR {
  double tolerance = 1e-8;
  int max_iterations = 500;
  bool verbose = false;
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHybridR(
        model, W, max_iterations, tolerance, verbose, &stats);
  }
};

// Strategy: geodesic HSD (joint tau/theta selection).
struct GeodesicHSD {
  double tolerance = 1e-8;
  int max_iterations = 30;
  bool verbose = false;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHSD(
        model, W, max_iterations, tolerance, verbose);
  }
};

// Strategy: θ-continuation geodesic IPM.
struct ThetaContinuation {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicThetaContinuation(
        model, W, max_iterations, max_centering_steps,
        tolerance, verbose, &stats);
  }
};

// Strategy: aggressive θ→0 / increase-k method.
struct PhaseOne {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicPhaseOne(
        model, W, max_iterations, max_centering_steps,
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

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    auto p1 = SolveGeodesicPhaseOne(
        model, W, max_iterations, max_centering_steps,
        tolerance, verbose, /*phase1_only=*/true);
    double k_init = (p1.mu > 0) ? 1.0 / std::sqrt(p1.mu) : -1;
    auto result = SolveGeodesicHybrid(
        model, W, max_iterations, tolerance, verbose,
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

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHybrid(
        model, W, max_iterations, tolerance, verbose,
        /*initial_k=*/-1, /*tau=*/1.0, policy);
  }
};

// Strategy: geodesic LP (line-search for k, then center).
struct GeodesicLP {
  double tolerance = 1e-8;
  int max_iterations = 30;
  int max_centering_steps = 0;
  bool verbose = false;
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicLP(
        model, W, max_iterations, max_centering_steps,
        tolerance, verbose, false, &stats);
  }
};

// Strategy: z-space geodesic LP for general log-homogeneous barriers.
// Bit-identical to GeodesicLP for symmetric cones when initial_z is empty.
struct GeodesicBarrierLP {
  double tolerance = 1e-8;
  int max_iterations = 30;
  int max_frozen_steps = 0;
  bool verbose = false;
  Eigen::VectorXd initial_z;  // empty → setOnes (symmetric cone default)
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace z = model.MakeRowSpace();
    if (initial_z.size() > 0) {
      z.col() = initial_z;
    } else {
      setOnes(z);
    }
    return SolveGeodesicBarrierLP(
        model, z, max_iterations, max_frozen_steps, tolerance, verbose,
        &stats);
  }
};

// Strategy: z-space θ-continuation for general log-homogeneous barriers.
struct GeodesicBarrierThetaContinuation {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_centering_steps = 1;
  bool verbose = false;
  Eigen::VectorXd initial_z;  // empty → setOnes
  mutable SolveStats stats;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace z = model.MakeRowSpace();
    if (initial_z.size() > 0) {
      z.col() = initial_z;
    } else {
      setOnes(z);
    }
    return SolveGeodesicBarrierThetaContinuation(
        model, z, max_iterations, max_centering_steps,
        tolerance, verbose, &stats);
  }
};

// Strategy: HSDE with scalar k and line search.
// Theta from gap+normalization, tau as cone variable.
struct GeodesicHSDE {
  double tolerance = 1e-8;
  int max_iterations = 500;
  int max_frozen_steps = 0;
  bool verbose = false;

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.MakeRowSpace();
    setOnes(W);
    return SolveGeodesicHSDE(
        model, W, max_iterations, max_frozen_steps, tolerance, verbose);
  }
};

// Strategy: direct linear solve (one factorization, one back-solve).
// Solves (A'WA + Q)x = rhs at W = identity. No iteration.
// rhs is in reduced variable space.
struct DirectSolve {
  std::vector<double> rhs;  // right-hand side in reduced space

  GeodesicResult Run(CompiledModel& model) const {
    RowSpace W = model.AllocRowSpace();
    setOnes(W);
    model.SetScaling(W);

    auto rhs_blk = model.AllocSolverRHS();
    rhs_blk.ScatterFrom(rhs.data(), rhs.size());
    model.SolveSolverRHS(rhs_blk);

    GeodesicResult result{};
    int nr = model.number_of_variables();
    result.x.resize(nr);
    Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr);
    rhs_blk.supernodes->GatherInto(xm);
    result.iterations = 1;
    result.total_factorizations = 1;
    result.total_solves = 1;
    return result;
  }
};

}  // namespace conex
