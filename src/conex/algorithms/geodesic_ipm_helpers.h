#pragma once

#include <utility>
#include <Eigen/Core>
#include "conex/algorithms/geodesic_ipm.h"

namespace conex {

class Arena;

// =====================================================================
// Internal types (used by algorithm implementations and tests)
// =====================================================================

// Decomposition of the Newton direction into components that are
// independent of k and theta:
//   d(k, theta) = d0 + k * (d1_0 + theta * d1_theta)
//   y(k, theta) = y0 + k * (y1_0 + theta * y1_theta)
struct NewtonDecomposition {
  RowSpace d0;
  RowSpace d1_0;       // standard optimality direction
  RowSpace d1_theta;   // theta correction direction
  SolverRHS y0, y1_0, y1_theta;  // primal variables in block form
};

// Six inner products that determine ||d||^2 as a function of (k, tau, theta).
struct DecompInnerProducts {
  double a;  // ||d0||^2
  double f;  // <d0, d1_0>
  double g;  // <d0, d1_theta>
  double p;  // ||d1_0||^2
  double q;  // <d1_0, d1_theta>
  double r;  // ||d1_theta||^2
};

struct KTauResult {
  double k;
  double tau;
  double theta;   // = 1/k^2
  double d_sq;    // ||d||^2 at the optimum
};

struct DualityCoeffs {
  double sigma1;
  double gamma1;
  double q11;
};

struct HybridDirection {
  double gap;
  double d_inf;
  double d_sq;
  double min_slack;
};

// =====================================================================
// Internal utility functions
// =====================================================================

// Verify the Newton direction satisfies its defining equations.
std::pair<double, double> VerifyNewtonEquations(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& d,
    const std::vector<double>& y,
    double k,
    double theta = 0.0);

// Check optimality conditions given primal x and dual λ.
OptimalityReport CheckOptimality(
    CompiledModel& model,
    const SolverRHS& x_rhs,
    const RowSpace& lambda);

// Factor the Gram system and compute the three-term decomposition.
void ComputeFullDecomposition(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    NewtonDecomposition& decomp);

// Evaluate d(k, tau, theta) = d0 + k * (tau * d1_0 + theta * d1_theta).
void EvaluateDirection(RowSpace& out, const NewtonDecomposition& decomp,
                       double k, double tau, double theta);
RowSpace EvaluateDirection(const NewtonDecomposition& decomp,
                           double k, double tau, double theta);

// Find k that minimizes ||d(k, tau, theta)||^2.
double MinNormK(const NewtonDecomposition& decomp, double tau, double theta);

DecompInnerProducts ComputeInnerProducts(const NewtonDecomposition& decomp);
KTauResult SelectKTau(const DecompInnerProducts& ip);

// Tau-independent duality coefficients.
DualityCoeffs ComputeDualityCoeffs(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp);

// Hybrid direction computation.
HybridDirection ComputeHybridDirection(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta,
    double tau_scale = 1.0);

HybridDirection HybridCenteringStep(
    CompiledModel& model,
    RowSpace& W,
    RowSpace& r);

// Frozen-Jacobian d0 refresh.
void RefreshD0Frozen(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    RowSpace& d0_out,
    SolverRHS& y0_out);

inline void RefreshD0Frozen(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    RowSpace& d0_out,
    SolverRHS& y0_out) {
  RefreshD0Frozen(model, model.arena(), b, W0, Wi, d0_out, y0_out);
}

// =====================================================================
// Theta-continuation internal helpers
// =====================================================================

// Precomputed theta-independent quantities for EvalThetaCandidate.
struct ThetaCandidateCoeffs {
  double beta;         // sigma1 + gamma1 + q11
  double bT_P_ed0;    // dot(b, P(sqrtW, e+d0))
  double bT_P_d1t;    // dot(b, P(sqrtW, d1_theta))
  double cT_y0, cT_yt;
  double bT_ones;
  // Q dot products: f = y0/k + theta*y_theta, need q00, q0t, qtt, q01, qt1
  double q00, q0t, qtt, q01, qt1;
  bool has_Q;
  // ||d||^2 inner products for root selection.
  DecompInnerProducts ip;
};

// Binary search result for smallest theta with d_inf <= beta_target.
struct ThetaSearchResult { double theta, k, tau; };

// Build the variable-space RHS for cost + equality terms, scaled.
SolverRHS MakeCostVarRHS(
    CompiledModel& model,
    const SolverRHS& cost_rhs,
    double cost_scale,
    double eq_scale);
SolverRHS MakeCostVarRHS(
    CompiledModel& model,
    const SolverRHS& cost_rhs,
    double scale);

// Build the "duality cost" for the V(tau)=0 identity.
SolverRHS MakeDualityCost(CompiledModel& model);

// Core solve pattern shared by all Newton-step variants.
RowSpace SolveConeSystem(
    CompiledModel& model,
    SolverRHS& var_rhs,
    const RowSpace& cone_rhs,
    Eigen::VectorXd* x_out = nullptr);

// Factor, two back-solves, 2-column MultiplyA -> compute d0, d1.
void ComputeDecomposition(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    RowSpace& d0,
    RowSpace& d1,
    SolverRHS* y0_out = nullptr,
    SolverRHS* y1_out = nullptr);

// Fast theta evaluation using precomputed coefficients (no eigendecomps).
std::pair<double, double> EvalThetaFast(
    const ThetaCandidateCoeffs& c,
    const DecompInnerProducts& ip,
    double theta_cand);

// Full evaluation (with exact d_inf) for the final chosen theta.
std::pair<double, double> EvalThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const ThetaCandidateCoeffs& c,
    const NewtonDecomposition& decomp,
    double theta_cand);

// Frozen-Jacobian variant of EvalThetaCandidate.
std::pair<double, double> FrozenEvalThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& Wi,
    const RowSpace& sqrtW0,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double beta,
    double theta_cand);

// Precompute theta-independent coefficients.
ThetaCandidateCoeffs PrecomputeThetaCoeffs(
    CompiledModel& model, Arena& arena,
    const SolverRHS& duality_cost, const RowSpace& b,
    const RowSpace& W, const NewtonDecomposition& decomp,
    double bT_ones);

// Frozen-J variant: recompute only the d0-dependent terms.
ThetaCandidateCoeffs RefreshFrozenThetaCoeffs(
    CompiledModel& model, Arena& arena,
    const SolverRHS& duality_cost, const RowSpace& b,
    const RowSpace& Wi, const RowSpace& sqrtW0,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double frozen_bT_P_d1t,
    double frozen_beta,
    const ThetaCandidateCoeffs& frozen_base);

// Binary search for smallest theta with d_inf <= beta_target.
ThetaSearchResult BisectTheta(
    CompiledModel& model, Arena& arena,
    const ThetaCandidateCoeffs& tc,
    const NewtonDecomposition& decomp,
    double theta_lo, double theta_hi, double beta_target);

// Verbose stats for ThetaCont iterations.
double PrintThetaContStats(
    CompiledModel& model,
    const NewtonDecomposition& decomp,
    const SolverRHS& cost_rhs,
    const SolverRHS& duality_cost,
    double bT_ones, double nu,
    int outer, int inner,
    double k, double tau, double theta,
    double d_inf, double d_sq, double gap);

// Frozen-J d0 refresh (NewtonDecomposition overload).
void RefreshD0Frozen(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    NewtonDecomposition& decomp);

// Direct Newton step: factor, single RHS, solve, compute d and y.
void ComputeDirectNewtonStep(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W, double k,
    RowSpace& d_out, Eigen::VectorXd& y_out,
    RowSpace* slack_out = nullptr);

// Helper: evaluate the V(tau)=0 quadratic for the z-space theta-continuation.
std::pair<double, double> EvalBarrierThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& z,
    const RowSpace& z_hess,
    const RowSpace& b,
    const RowSpace& grad_z,
    const RowSpace& ay0,
    const RowSpace& t1_tau,
    const RowSpace& t1_th,
    const SolverRHS& y0_rhs,
    const SolverRHS& y1_0_rhs,
    const SolverRHS& y1_theta_rhs,
    double nu, double R_theta1, double theta_cand);

}  // namespace conex
