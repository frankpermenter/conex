#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::dot;
using EuclideanJordanAlgebra::gap;
using EuclideanJordanAlgebra::minEigenvalue;
using EuclideanJordanAlgebra::minSlack;
using EuclideanJordanAlgebra::normInf;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::shrinkR;
using EuclideanJordanAlgebra::solveLyapunovForD;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::square;
using EuclideanJordanAlgebra::squareM;
using EuclideanJordanAlgebra::applyM;
using EuclideanJordanAlgebra::applyMt;
using EuclideanJordanAlgebra::updateAutomorphism;
using EuclideanJordanAlgebra::updateAutomorphismP;
using EuclideanJordanAlgebra::updateM;
using EuclideanJordanAlgebra::sqrt;

// Compute b_theta = theta * e + (1 - theta) * b.
static RowSpace BlendAffine(CompiledModel& model, Arena& arena,
                            const RowSpace& b, double theta) {
  RowSpace b_theta = model.AllocRowSpace(arena);
  if (theta == 0.0) {
    b_theta = b;
  } else if (theta == 1.0) {
    setOnes(b_theta);
  } else {
    RowSpace ones = model.AllocRowSpace(arena);
    setOnes(ones);
    addScaled(b_theta, ones, b, theta, 1.0 - theta);
  }
  return b_theta;
}

HybridRDirection ComputeHybridRDirection(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta,
    std::vector<double>* y_out) {
  auto mark = arena.SaveCursor();
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b_theta = BlendAffine(model, arena, b, theta);
  RowSpace sqrtW = model.AllocRowSpace(arena);
  sqrt(sqrtW, W);

  // RHS = -(c + A^T P(W)(b_theta)) + 2*A^T P(W^{1/2})(r) + d_eq
  auto y = model.AllocSolverRHS();
  y = cost_rhs;
  RowSpace v = model.AllocRowSpace(arena);
  quadraticRepresentation(v, W, b_theta);
  model.AccumulateAtranspose(v, y);
  y *= -1;
  // + 2*A^T P(W^{1/2})(r)
  quadraticRepresentation(v, sqrtW, r);
  v *= 2.0;
  model.AccumulateAtranspose(v, y);
  // + d_eq (equality RHS).
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    y += d_rhs;
  }
  model.SolveSolverRHS(y);

  // Optionally return the solve vector.
  if (y_out) {
    int nv = model.number_of_variables();
    y_out->resize(nv);
    { Eigen::Map<Eigen::VectorXd> ym(y_out->data(), y_out->size()); y.supernodes->GatherInto(ym); }
  }

  // delta = r - P(W^{1/2})(b_theta + A*y)
  RowSpace row = model.AllocRowSpace(arena);
  model.MultiplyA(y, row);
  RowSpace slack_dir = model.AllocRowSpace(arena);
  addScaled(slack_dir, b_theta, row, 1.0, 1.0);
  RowSpace qr_tmp = model.AllocRowSpace(arena);
  quadraticRepresentation(qr_tmp, sqrtW, slack_dir);
  addScaled(delta, r, qr_tmp, 1.0, -1.0);
  solveLyapunovForD(d, r, delta);

  auto result = HybridRDirection{gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
  arena.RestoreCursor(mark);
  return result;
}

// Backward-compat wrapper (no arena).
HybridRDirection ComputeHybridRDirection(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta,
    std::vector<double>* y_out) {
  return ComputeHybridRDirection(model, model.arena(), b, W, r, theta, d, delta, y_out);
}

// M-based variant: uses M (full automorphism, no polar split) instead of W.
// W = M*M^T is passed pre-computed. r is in the M-frame.
// Replaces P(sqrt(W))(x) with applyM(M, x) and P(sqrt(W))^T(x) with applyMt(M, x).
static HybridRDirection ComputeHybridRDirectionM(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& M,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta) {
  auto mark = arena.SaveCursor();
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b_theta = BlendAffine(model, arena, b, theta);

  // RHS = -(c + A^T P(W)(b_theta)) + 2*A^T applyM(M, r) + d_eq
  auto y = model.AllocSolverRHS();
  y = cost_rhs;
  RowSpace v = model.AllocRowSpace(arena);
  quadraticRepresentation(v, W, b_theta);
  model.AccumulateAtranspose(v, y);
  y *= -1;
  // + 2*A^T M*r*M^T  (replaces P(sqrt(W))(r) since r is in M-frame)
  applyM(v, M, r);
  v *= 2.0;
  model.AccumulateAtranspose(v, y);
  // + d_eq (equality RHS).
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    y += d_rhs;
  }
  model.SolveSolverRHS(y);

  // delta = r - M^T*(b_theta + A*y)*M  (in M-frame)
  // Physical-frame slack = b_theta + A*y.
  // M-frame delta: r - applyMt(M, slack) = r - M^T*slack*M.
  RowSpace row = model.AllocRowSpace(arena);
  model.MultiplyA(y, row);
  RowSpace slack_dir = model.AllocRowSpace(arena);
  addScaled(slack_dir, b_theta, row, 1.0, 1.0);
  RowSpace mt_tmp = model.AllocRowSpace(arena);
  applyMt(mt_tmp, M, slack_dir);
  addScaled(delta, r, mt_tmp, 1.0, -1.0);
  solveLyapunovForD(d, r, delta);

  auto result = HybridRDirection{gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
  arena.RestoreCursor(mark);
  return result;
}

std::pair<double, double> VerifyHybridREquations(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    const RowSpace& d,
    const RowSpace& delta,
    const std::vector<double>& y) {
  auto mark = arena.SaveCursor();
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b_theta = BlendAffine(model, arena, b, theta);
  RowSpace sqrtW = model.AllocRowSpace(arena);
  sqrt(sqrtW, W);

  // --- Primal check ---
  // delta should equal r - P(W^{1/2})(b_theta + A*y).
  auto y_rhs = model.AllocSolverRHS();
  y_rhs.ScatterFrom(y.data(), y.size());
  RowSpace Ay = model.AllocRowSpace(arena);
  model.MultiplyA(y_rhs, Ay);
  RowSpace slack = model.AllocRowSpace(arena);
  addScaled(slack, b_theta, Ay, 1.0, 1.0);
  RowSpace qr_tmp = model.AllocRowSpace(arena);
  quadraticRepresentation(qr_tmp, sqrtW, slack);
  RowSpace delta_expected = model.AllocRowSpace(arena);
  addScaled(delta_expected, r, qr_tmp, 1.0, -1.0);
  RowSpace diff_tmp = model.AllocRowSpace(arena);
  addScaled(diff_tmp, delta, delta_expected, 1.0, -1.0);
  double primal_res = normInf(diff_tmp);

  // --- Dual check ---
  RowSpace r_plus_delta = model.AllocRowSpace(arena);
  addScaled(r_plus_delta, r, delta, 1.0, 1.0);
  RowSpace lambda = model.AllocRowSpace(arena);
  quadraticRepresentation(lambda, sqrtW, r_plus_delta);
  auto at_lambda = model.AllocSolverRHS();
  at_lambda.SetZero();
  model.AccumulateAtranspose(lambda, at_lambda);

  // - Qy
  auto qy = model.AllocSolverRHS();
  qy.SetZero();
  model.AccumulateQx(y_rhs, qy);
  at_lambda -= qy;

  // - c
  at_lambda -= cost_rhs;

  int n = model.number_of_variables();
  Eigen::VectorXd dual_err(n);
  at_lambda.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

  arena.RestoreCursor(mark);
  return {primal_res, dual_res};
}

// Backward-compat wrapper (no arena).
std::pair<double, double> VerifyHybridREquations(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    const RowSpace& d,
    const RowSpace& delta,
    const std::vector<double>& y) {
  return VerifyHybridREquations(model, model.arena(), b, W, r, theta, d, delta, y);
}

HybridRDecomposition ComputeHybridRDecomposition(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& M,
    const RowSpace& W,
    const RowSpace& r) {
  // Temporaries use SaveCursor/RestoreCursor; output members are allocated
  // from the arena BEFORE the temp mark so they survive.
  const auto& cost_rhs = model.cost_rhs();

  HybridRDecomposition decomp;
  // Allocate output RowSpace members from the arena (these must survive).
  decomp.ax0 = model.AllocRowSpace(arena);
  decomp.ax1 = model.AllocRowSpace(arena);
  decomp.ax_theta = model.AllocRowSpace(arena);
  decomp.lam0 = model.AllocRowSpace(arena);
  decomp.lam1 = model.AllocRowSpace(arena);
  decomp.lam_theta = model.AllocRowSpace(arena);
  decomp.delta_cost = model.AllocRowSpace(arena);
  decomp.delta_center = model.AllocRowSpace(arena);
  // Allocate SolverRHS members (heap, survive arena restore).
  auto initRHS = [&](SolverRHS& dest) {
    SolverRHS tmp = model.MakeSolverRHS();
    dest.supernodes = tmp.supernodes;
    dest.separators = tmp.separators;
    dest.blocks_fully_gathered = tmp.blocks_fully_gathered;
  };
  initRHS(decomp.x0);
  initRHS(decomp.x1);
  initRHS(decomp.x_theta);
  initRHS(decomp.y_center);
  initRHS(decomp.y_cost);

  auto mark = arena.SaveCursor();  // temps below this mark get freed

  RowSpace v = model.AllocRowSpace(arena);

  // Three-solve decomposition (hybrid_theta_continuation.tex S2).
  RowSpace ones = model.AllocRowSpace(arena);
  setOnes(ones);

  // rhs0 = 2*A' * applyM(M, r)
  auto rhs0 = model.AllocSolverRHS();
  rhs0.SetZero();
  applyM(v, M, r);
  v *= 2.0;
  model.AccumulateAtranspose(v, rhs0);

  // rhs1 = -(c + A'P(W)(b)) + d_eq
  auto rhs1 = model.AllocSolverRHS();
  rhs1 = cost_rhs;
  quadraticRepresentation(v, W, b);
  model.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    rhs1 += ts->EqualityAffineTermRHS();
  }

  // rhs2 = -rhs1 - A'(e + P(W)(e))
  auto rhs2 = model.AllocSolverRHS();
  rhs2 = rhs1;
  rhs2 *= -1;  // (c + A'P(W)(b)) - d_eq
  RowSpace qr_ones = model.AllocRowSpace(arena);
  quadraticRepresentation(qr_ones, W, ones);
  addScaled(v, ones, qr_ones, 1.0, 1.0);
  v *= -1.0;   // -(e + P(W)(e))
  model.AccumulateAtranspose(v, rhs2);

  // Solve all three with one factorization.
  auto y = model.AllocSolverRHS(3);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  y.SetColumn(2, rhs2);
  model.SolveSolverRHS(y);

  // Multiply A * [x0, x1, x_theta].
  auto row = model.AllocRowSpace(arena, 3);
  model.MultiplyA(y, row);

  decomp.ax0.col() = row.col(0);
  decomp.ax1.col() = row.col(1);
  decomp.ax_theta.col() = row.col(2);

  // Extract columns block-by-block into SolverRHS members.
  int nb = y.num_blocks();
  for (int k = 0; k < nb; ++k) {
    decomp.x0.supernodes->block(k).col(0) = y.supernodes->block(k).col(0);
    decomp.x1.supernodes->block(k).col(0) = y.supernodes->block(k).col(1);
    decomp.x_theta.supernodes->block(k).col(0) = y.supernodes->block(k).col(2);
  }

  // lambda(tau,theta) = lam0 + tau*lam1 + theta*lam_theta
  RowSpace e_minus_b = model.AllocRowSpace(arena);
  addScaled(e_minus_b, ones, b, 1.0, -1.0);
  // lam0 = applyM(M, 2r) - P(W)(A*x0)  (physical frame)
  RowSpace Mr2 = model.AllocRowSpace(arena);
  applyM(Mr2, M, r);
  Mr2 *= 2.0;
  RowSpace qr_ax0 = model.AllocRowSpace(arena);
  quadraticRepresentation(qr_ax0, W, decomp.ax0);
  addScaled(decomp.lam0, Mr2, qr_ax0, 1.0, -1.0);

  RowSpace arg1 = model.AllocRowSpace(arena);
  addScaled(arg1, decomp.ax1, b, 1.0, 1.0);
  quadraticRepresentation(decomp.lam1, W, arg1);
  decomp.lam1 *= -1.0;

  RowSpace arg_th = model.AllocRowSpace(arena);
  addScaled(arg_th, decomp.ax_theta, e_minus_b, 1.0, 1.0);
  quadraticRepresentation(decomp.lam_theta, W, arg_th);
  decomp.lam_theta *= -1.0;

  // delta_cost = -applyMt(M, b + A*x1) -- independent of theta, in M-frame.
  applyMt(decomp.delta_cost, M, arg1);
  decomp.delta_cost *= -1.0;

  arena.RestoreCursor(mark);  // free temporaries, output members survive

  // y_center, y_cost, delta_center are set by SetTheta().
  return decomp;
}

HybridRDecomposition ComputeHybridRDecomposition(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& M,
    const RowSpace& W,
    const RowSpace& r) {
  return ComputeHybridRDecomposition(model, model.arena(), b, M, W, r);
}

int UpdateX0(HybridRDecomposition& decomp,
             CompiledModel& model,
             Arena& arena,
             const RowSpace& M,
             const RowSpace& W,
             const RowSpace& r) {
  auto mark = arena.SaveCursor();
  // rhs0 = 2*A' * applyM(M, r)
  RowSpace v = model.AllocRowSpace(arena);
  applyM(v, M, r);
  v *= 2.0;
  auto rhs0 = model.AllocSolverRHS();
  rhs0.SetZero();
  model.AccumulateAtranspose(v, rhs0);
  model.SolveSolverRHS(rhs0);

  int nb = rhs0.num_blocks();
  for (int k = 0; k < nb; ++k)
    decomp.x0.supernodes->block(k) = rhs0.supernodes->block(k);

  model.MultiplyA(rhs0, decomp.ax0);

  // Update lam0 = applyM(M, 2r) - P(W)(A*x0).
  RowSpace Mr2 = model.AllocRowSpace(arena);
  applyM(Mr2, M, r);
  Mr2 *= 2.0;
  RowSpace qr_ax0 = model.AllocRowSpace(arena);
  quadraticRepresentation(qr_ax0, W, decomp.ax0);
  addScaled(decomp.lam0, Mr2, qr_ax0, 1.0, -1.0);

  arena.RestoreCursor(mark);
  return 1;  // 1 solve
}

// Backward-compat wrapper (no arena).
int UpdateX0(HybridRDecomposition& decomp,
             CompiledModel& model,
             const RowSpace& M,
             const RowSpace& W,
             const RowSpace& r) {
  return UpdateX0(decomp, model, model.arena(), M, W, r);
}

void SetTheta(HybridRDecomposition& decomp,
              CompiledModel& model,
              Arena& arena,
              const RowSpace& b,
              const RowSpace& M,
              const RowSpace& r,
              double theta) {
  auto mark = arena.SaveCursor();
  decomp.y_center = decomp.x0;
  decomp.y_center.AddScaled(theta, decomp.x_theta);
  decomp.y_cost = decomp.x1;

  // delta_center = r - applyMt(M, A*f + theta*(e-b))  (M-frame)
  // Use cached A*x0, A*x_theta: A*f = A*x0 + theta*A*x_theta.
  RowSpace ones_v = model.AllocRowSpace(arena); setOnes(ones_v);
  RowSpace e_minus_b = model.AllocRowSpace(arena);
  addScaled(e_minus_b, ones_v, b, 1.0, -1.0);
  RowSpace Af = model.AllocRowSpace(arena);
  addScaled(Af, decomp.ax0, decomp.ax_theta, 1.0, theta);
  RowSpace dc_arg = model.AllocRowSpace(arena);
  addScaled(dc_arg, Af, e_minus_b, 1.0, theta);
  RowSpace mt_tmp = model.AllocRowSpace(arena);
  applyMt(mt_tmp, M, dc_arg);
  addScaled(decomp.delta_center, r, mt_tmp, 1.0, -1.0);
  arena.RestoreCursor(mark);
}

// Backward-compat wrapper (no arena).
void SetTheta(HybridRDecomposition& decomp,
              CompiledModel& model,
              const RowSpace& b,
              const RowSpace& M,
              const RowSpace& r,
              double theta) {
  SetTheta(decomp, model, model.arena(), b, M, r, theta);
}

HybridRDirection EvalHybridRAtTau(
    CompiledModel& model,
    Arena& arena,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta) {
  // delta(tau) = delta_center + tau * delta_cost
  addScaled(delta, decomp.delta_center, decomp.delta_cost, 1.0, tau);
  solveLyapunovForD(d, r, delta);
  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

// Backward-compat wrapper (no arena).
HybridRDirection EvalHybridRAtTau(
    CompiledModel& model,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta) {
  return EvalHybridRAtTau(model, model.arena(), decomp, r, tau, d, delta);
}

GeodesicResult SolveGeodesicThetaContinuationR(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose,
    ThetaContRSwitchPolicy policy,
    double compl_tol,
    double theta_rate) {
  Arena& arena = model.arena();
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b = model.GetAffineTerm();

  RowSpace ones = model.AllocRowSpace(arena);
  setOnes(ones);
  const double cone_rank = dot(ones, ones);  // <e, e> = trace(I), correct for PSD
  const double bT_ones = dot(b, ones);
  const double R = bT_ones + 1.0;

  // Duality cost (cost_rhs + equality dual correction).
  auto duality_cost = model.AllocSolverRHS();
  duality_cost = cost_rhs;
  auto* ts_init = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts_init && !ts_init->equality_sub_assemblers().empty()) {
    duality_cost += ts_init->EqualityAffineTermRHS();
  }

  RowSpace r_var = model.AllocRowSpace(arena);
  setOnes(r_var);
  double theta = 1.0;
  double tau = 1.0;
  double w_tau = 1.0;
  double r_tau = 1.0;
  const double alpha_norm = cone_rank + 1.0;
  const bool skip_Q = !model.has_quadratic_cost();
  // Store M (full automorphism, no polar split) as primary state.
  // W = M*M^T. r stays in M-frame (not rotated during W-updates).
  RowSpace M = model.AllocRowSpace(arena);
  setOnes(M);  // M = I initially
  squareM(W, M);  // W = I initially
  model.SetScaling(W);
  model.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  if (verbose) {
    printf("  %3s  %10s  %10s  %8s  %8s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s  %8s  %10s  %10s  %10s  %3s\n",
           "out", "theta", "tau", "w_tau", "r_tau", "d_inf", "d_tau",
           "gap", "dual", "primal", "mu/tau", "eq_err", "norm_err",
           "compl", "th*alpha", "cpl_err", "st");
    printf("  %s\n", std::string(195, '-').c_str());
  }

  RowSpace last_delta = model.AllocRowSpace(arena);
  RowSpace d_vec = model.AllocRowSpace(arena);      // pre-allocated direction
  RowSpace delta_vec = model.AllocRowSpace(arena);   // pre-allocated delta
  bool need_decomp = true;
  bool full_decomp = true;  // true = all 3 solves, false = only x0
  HybridRDecomposition decomp;
  double theta_at_last_w = 1.0;
  double d_tau = 0;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // No SaveCursor/RestoreCursor: decomp members persist across iterations.

    if (need_decomp) {
      if (full_decomp) {
        decomp = ComputeHybridRDecomposition(model, arena, b, M, W, r_var);
        total_sol += 3;
      } else {
        // Only r changed -- re-solve x0, keep x1 and x_theta.
        total_sol += UpdateX0(decomp, model, arena, M, W, r_var);
      }
      need_decomp = false;
      full_decomp = false;

      // Joint (tau, theta) selection from gap + normalization.
      RowSpace rp = model.AllocRowSpace(arena);
      addScaled(rp, b, ones, 1.0, -1.0);  // b - e

      double bTl0 = dot(b, decomp.lam0);
      double bTl1 = dot(b, decomp.lam1);
      double bTlth = dot(b, decomp.lam_theta);
      double cTx0 = duality_cost.dot(decomp.x0);
      double cTx1 = duality_cost.dot(decomp.x1);
      double cTxth = duality_cost.dot(decomp.x_theta);

      RowSpace Ax0_v = model.AllocRowSpace(arena); model.MultiplyA(decomp.x0, Ax0_v);
      RowSpace Ax1_v = model.AllocRowSpace(arena); model.MultiplyA(decomp.x1, Ax1_v);
      RowSpace Axth_v = model.AllocRowSpace(arena); model.MultiplyA(decomp.x_theta, Axth_v);

      double rpTl0 = dot(rp, decomp.lam0);
      double rpTl1 = dot(rp, decomp.lam1);
      double rpTlth = dot(rp, decomp.lam_theta);
      double rdTx0 = cTx0 - dot(ones, Ax0_v);
      double rdTx1 = cTx1 - dot(ones, Ax1_v);
      double rdTxth = cTxth - dot(ones, Axth_v);
      double rg = -(bT_ones + 1.0);

      double wt = w_tau, rt = r_tau;

      // Normalization coefficients (same for both paths).
      double N0 = rpTl0 + rdTx0;
      double N1 = rpTl1 + rdTx1 + rg;
      double Nth = rpTlth + rdTxth;

      if (skip_Q) {
        // Q=0: 2x2 linear solve in (d_tau, theta).
        double G_dtau = wt * rt * (bTl1 + cTx1) - rt / wt;
        double G_theta = bTlth + cTxth - R;
        double G_0 = bTl0 + cTx0 + wt * rt * (bTl1 + cTx1) + rt / wt;
        double N_dtau = wt * rt * (rpTl1 + rdTx1 + rg);
        double N_theta = rpTlth + rdTxth;
        double N_0_lin = rpTl0 + rdTx0 + wt * rt * (rpTl1 + rdTx1 + rg) + alpha_norm;
        double det_val = G_dtau * N_theta - G_theta * N_dtau;
        if (std::abs(det_val) > 1e-30) {
          d_tau = (N_theta * (-G_0) - G_theta * (-N_0_lin)) / det_val;
          theta = (G_dtau * (-N_0_lin) - N_dtau * (-G_0)) / det_val;
          tau = wt * rt * (1.0 + d_tau);
        }
      } else {
        // Q!=0: eliminate theta via normalization, solve quadratic in tau.
        double eta = alpha_norm + N0;
        double Nth_threshold = 1e-12 * (std::abs(N0) + std::abs(N1) + 1.0);

        if (std::abs(Nth) > Nth_threshold) {
          // Normal case: eliminate theta, quadratic in tau.
          double n1 = N1 / Nth;
          double e1 = eta / Nth;

          auto f0_rhs = model.AllocSolverRHS();
          f0_rhs = decomp.x0;
          f0_rhs.AddScaled(-e1, decomp.x_theta);
          auto h_rhs = model.AllocSolverRHS();
          h_rhs = decomp.x1;
          h_rhs.AddScaled(-n1, decomp.x_theta);

          double qff = 0, qfh = 0, qhh = 0;
          {
            auto Qf0 = model.AllocSolverRHS(); Qf0.SetZero(); model.AccumulateQx(f0_rhs, Qf0);
            auto Qh = model.AllocSolverRHS(); Qh.SetZero(); model.AccumulateQx(h_rhs, Qh);
            qff = Qf0.dot(f0_rhs);
            qfh = Qf0.dot(h_rhs);
            qhh = Qh.dot(h_rhs);
          }

          double bTl0_sub = bTl0 - e1 * bTlth;
          double bTl1_sub = bTl1 - n1 * bTlth;
          double cTf0 = cTx0 - e1 * cTxth;
          double cTh = cTx1 - n1 * cTxth;

          double inv_wt = 1.0 / (w_tau > 1e-30 ? w_tau : 1e-30);
          double A_coeff = bTl1_sub + cTh + qhh + R*n1 - inv_wt*inv_wt;
          double B_coeff = bTl0_sub + cTf0 + 2*qfh + R*e1 + 2*r_tau*inv_wt;
          double C_coeff = qff;

          double tau_new = tau;
          double discr = B_coeff * B_coeff - 4.0 * A_coeff * C_coeff;
          if (discr >= 0 && std::abs(A_coeff) > 1e-30) {
            double sq = std::sqrt(discr);
            double t1 = (-B_coeff + sq) / (2.0 * A_coeff);
            double t2 = (-B_coeff - sq) / (2.0 * A_coeff);
            double wtr = w_tau * r_tau;
            double d1v = (std::abs(wtr) > 1e-30) ? t1 / wtr - 1.0 : 1e30;
            double d2v = (std::abs(wtr) > 1e-30) ? t2 / wtr - 1.0 : 1e30;
            if (t1 > 0 && !(t2 > 0))
              tau_new = t1;
            else if (t2 > 0 && !(t1 > 0))
              tau_new = t2;
            else
              tau_new = (std::abs(d1v) < std::abs(d2v)) ? t1 : t2;
          }
          tau = tau_new;
          double wtr = w_tau * r_tau;
          d_tau = (std::abs(wtr) > 1e-30) ? tau / wtr - 1.0 : 0.0;
          theta = (-alpha_norm - N0 - N1 * tau) / Nth;
        } else {
          // Nth ~ 0: normalization is N0 + N1*tau = -alpha (no theta).
          if (std::abs(N1) > 1e-30) {
            tau = (-alpha_norm - N0) / N1;
          }
          double wtr = w_tau * r_tau;
          d_tau = (std::abs(wtr) > 1e-30) ? tau / wtr - 1.0 : 0.0;

          double kappa_val = r_tau * (1.0 - d_tau) / (w_tau > 1e-30 ? w_tau : 1e-30);
          double gap_linear = bTl0 + tau * bTl1 + cTx0 + tau * cTx1 + kappa_val;

          auto x_rhs = model.AllocSolverRHS();
          x_rhs = decomp.x0;
          x_rhs.AddScaled(tau, decomp.x1);
          x_rhs.AddScaled(theta, decomp.x_theta);
          auto qx = model.AllocSolverRHS(); qx.SetZero();
          model.AccumulateQx(x_rhs, qx);
          double xQx = qx.dot(x_rhs);
          double xQx_tau = (std::abs(tau) > 1e-30) ? xQx / tau : 0.0;

          double G_theta = bTlth + cTxth - R;
          if (std::abs(G_theta) > 1e-30) {
            theta = -(gap_linear + xQx_tau) / G_theta;
          }
          if (verbose) {
            printf("  [Nth~0] tau=%.4e theta=%.4e G_theta=%.4e N1=%.4e\n",
                   tau, theta, G_theta, N1);
          }
        }
      }
      SetTheta(decomp, model, arena, b, M, r_var, theta);
    }

    // Evaluate cone direction at current (tau, r, theta).
    auto info = EvalHybridRAtTau(model, arena, decomp, r_var, tau, d_vec, delta_vec);
    last_delta = delta_vec;
    g = info.gap;
    d_inf = std::max(info.d_inf, std::abs(d_tau));

    // Record per-iteration stats.
    result.iter_stats.push_back({squaredNorm(r_var) / cone_rank, d_inf, info.d_sq,
        g, r_updates_since_fac, info.min_slack, theta, total_fac});

    if (verbose) {
      // Diagnostics: gap equation, normalization, dual residual, complementarity.
      RowSpace r_plus_delta = model.AllocRowSpace(arena);
      addScaled(r_plus_delta, r_var, delta_vec, 1.0, 1.0);
      RowSpace lam_v = model.AllocRowSpace(arena);
      applyM(lam_v, M, r_plus_delta);
      auto x_rhs = model.AllocSolverRHS();
      x_rhs = decomp.y_center;
      x_rhs.AddScaled(tau, decomp.y_cost);
      auto qx = model.AllocSolverRHS(); qx.SetZero();
      model.AccumulateQx(x_rhs, qx);

      double bTl = dot(b, lam_v);
      double cTx = duality_cost.dot(x_rhs);
      double xQx = qx.dot(x_rhs);
      double mu_v = squaredNorm(r_var) / cone_rank;
      double kappa_v = r_tau * (1.0 - d_tau) / std::max(w_tau, 1e-30);
      double xQx_over_tau = (std::abs(tau) > 1e-30) ? xQx / tau : 0.0;
      double eq_err = std::abs(bTl + cTx + xQx_over_tau + kappa_v - theta * R);
      double half_xQx_phys = (std::abs(tau) > 1e-30) ? 0.5 * xQx / (tau*tau) : 0.0;
      double primal_phys = (std::abs(tau) > 1e-30) ? cTx/tau + half_xQx_phys : 0.0;
      double dual_phys = (std::abs(tau) > 1e-30) ? -(bTl/tau + half_xQx_phys) : 0.0;

      // Normalization: rp'lambda + rd'x + rg*tau vs -alpha.
      double eTl = dot(ones, lam_v);
      RowSpace Ax_v = model.AllocRowSpace(arena); model.MultiplyA(x_rhs, Ax_v);
      double norm_val = (bTl - eTl) + (cTx - dot(ones, Ax_v))
                        + (-(bT_ones + 1.0)) * tau;
      double alpha_v = dot(ones, ones) + 1.0;
      double norm_err = norm_val - (-alpha_v);

      // Complementarity: gap + tau*kappa vs theta*alpha.
      double tau_kappa = r_tau*r_tau*(1.0 - d_tau*d_tau);
      double compl_err_v = (info.gap + tau_kappa) - theta * alpha_v;

      printf("  %3d  %10.2e  %10.2e  %8.4f  %8.4f  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %12.2e  %8.2e  %10.4e  %10.4e  %10.2e  %3d\n",
             iter, theta, tau, w_tau, r_tau, d_inf, d_tau, g,
             dual_phys, primal_phys, mu_v/std::max(std::abs(tau), 1e-30),
             eq_err, norm_err, info.gap + tau_kappa, theta * alpha_v,
             compl_err_v, r_updates_since_fac);
    }

    if (!std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: nan (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);

      break;
    }

    result.iterations = iter + 1;
    if (std::abs(theta) < tolerance && std::abs(g) < tolerance && d_inf <= 1.001) {

      break;
    }

    // Complementarity check: gap + r_tau^2 should = theta * alpha.
    double alpha_check = alpha_norm;
    double tau_kappa_check = r_tau*r_tau*(1.0 - d_tau*d_tau);
    double compl_err = std::abs(info.gap + tau_kappa_check - theta * alpha_check);
    bool w_frozen = (compl_err > compl_tol);

    // Theta-rate check.
    bool theta_stalled = false;
    if (theta_rate > 0 && r_updates_since_fac >= 2) {
      theta_stalled = (std::abs(theta) > theta_rate * std::abs(theta_at_last_w));
    }

    bool do_center = !w_frozen && (policy(g, d_inf, r_updates_since_fac)
                                    || theta_stalled);
    if (do_center) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateM(M, r_var, alpha, d_vec);
      squareM(W, M);
      w_tau *= std::exp(d_tau * alpha);
      model.SetScaling(W);
      if (!model.AssembleAndFactor()) {
  
        break;
      }
      total_fac++;
      r_updates_since_fac = 0;
      theta_at_last_w = theta;
      need_decomp = true;
      full_decomp = true;  // W changed, need all 3 solves
    } else {
      shrinkR(r_var, delta_vec);
      r_tau = 0.5 * r_tau * (1.0 + std::abs(d_tau));
      r_updates_since_fac++;
      need_decomp = true;
    }
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r_var) / cone_rank;
  result.complementarity = g;
  result.tau = tau;
  result.kappa = (tau > 1e-30) ? theta / tau : std::numeric_limits<double>::infinity();
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x (de-homogenized by tau).
  {
    auto x_rhs = model.AllocSolverRHS();
    x_rhs = decomp.y_center;
    x_rhs.AddScaled(tau, decomp.y_cost);
    x_rhs *= (1.0 / tau);
    result.x.resize(model.number_of_variables());
    result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs.supernodes->GatherInto(xm); }
  }

  // Lambda and optimality.
  {
    auto x_rhs = model.AllocSolverRHS();
    x_rhs.ScatterFrom(result.x.data(), result.x.size());
    // result.lambda must outlive the arena -> heap allocation.
    RowSpace r_plus_delta = model.AllocRowSpace(arena);
    addScaled(r_plus_delta, r_var, last_delta, 1.0, 1.0);
    result.lambda = model.MakeRowSpace();  // heap
    applyM(result.lambda, M, r_plus_delta);
    if (tau > 0 && tau != 1.0) result.lambda *= (1.0 / tau);
    result.optimality = CheckOptimality(model, x_rhs, result.lambda);
    result.optimality.mu = result.mu;
  }

  return result;
}

GeodesicResult SolveGeodesicHybridR(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  Arena& arena = model.arena();
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b = model.GetAffineTerm();
  const int m = b.total_rows();

  RowSpace r = model.AllocRowSpace(arena);
  setOnes(r);
  double theta = 1.0;

  // Store M (full automorphism, no polar split) as primary state.
  // W = M*M^T. r stays in M-frame (not rotated during W-updates).
  RowSpace M = model.AllocRowSpace(arena);
  setOnes(M);  // M = I initially
  squareM(W, M);
  model.SetScaling(W);
  model.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  if (verbose) {
    printf("  %3s %6s  %12s  %10s %10s  %12s  %6s  %6s\n",
           "it", "theta", "gap", "d_pre", "d_post", "|r|^2/m",
           "r_upd", "step");
    printf("  %s\n", std::string(90, '-').c_str());
  }

  RowSpace last_delta = model.AllocRowSpace(arena);

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = model.AllocRowSpace(arena);
    RowSpace delta = model.AllocRowSpace(arena);
    auto info = ComputeHybridRDirectionM(model, arena, b, M, W, r, theta,
                                          d, delta);
    last_delta = delta;
    total_sol++;
    double d_inf_pre = info.d_inf;
    g = info.gap;
    d_inf = info.d_inf;

    if (d_inf > 10 || !std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: diverging (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);

      break;
    }

    bool do_center = (g < 0);

    if (do_center) {
      // W-update: M_new = M * exp(aD/2). No polar decomposition.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateM(M, r, alpha, d);
      squareM(W, M);
      model.SetScaling(W);
      if (!model.AssembleAndFactor()) {
  
        break;
      }
      total_fac++;
      r_updates_since_fac = 0;
    } else {
      // r-update + theta update.
      shrinkR(r, delta);
      r_updates_since_fac++;
      theta = std::abs(g) / m;
    }

    // Recompute direction at updated state.
    {
      RowSpace d2 = model.AllocRowSpace(arena);
      RowSpace delta2 = model.AllocRowSpace(arena);
      auto info2 = ComputeHybridRDirectionM(model, arena, b, M, W, r, theta,
                                             d2, delta2);
      last_delta = delta2;
      g = info2.gap;
      d_inf = info2.d_inf;
      total_sol++;
    }

    if (verbose) {
      printf("  %3d %6.4f  %12.4e  %10.4e %10.4e  %12.4e  %6d  %s\n",
             iter, theta, g, d_inf_pre, d_inf,
             squaredNorm(r) / m, r_updates_since_fac,
             do_center ? "center" : "r+theta");
    }

    if (theta < tolerance && std::abs(g) < tolerance && d_inf <= 1.001) {
      break;
    }
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x.
  {
    model.SetScaling(W);
    model.AssembleAndFactor();
    RowSpace b_theta = BlendAffine(model, arena, b, theta);

    auto y = model.AllocSolverRHS();
    y = cost_rhs;
    RowSpace v = model.AllocRowSpace(arena);
    quadraticRepresentation(v, W, b_theta);
    model.AccumulateAtranspose(v, y);
    y *= -1;
    // 2*A^T * M*r*M^T  (applyM since r is in M-frame)
    applyM(v, M, r);
    v *= 2.0;
    model.AccumulateAtranspose(v, y);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      y += ts->EqualityAffineTermRHS();
    }
    model.SolveSolverRHS(y);
    int nr = model.number_of_variables();
    result.x.resize(nr);
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); y.supernodes->GatherInto(xm); }
  }

  // Lambda and optimality.
  // lambda = M*(r+delta)*M^T  (in physical frame, from M-frame r and delta)
  {
    auto x_rhs = model.AllocSolverRHS();
    x_rhs.ScatterFrom(result.x.data(), result.x.size());
    RowSpace r_plus_delta = model.AllocRowSpace(arena);
    addScaled(r_plus_delta, r, last_delta, 1.0, 1.0);
    // result.lambda must outlive the arena -> heap allocation.
    result.lambda = model.MakeRowSpace();  // heap
    applyM(result.lambda, M, r_plus_delta);
    result.optimality = CheckOptimality(model, x_rhs, result.lambda);
    result.optimality.mu = result.mu;
  }

  return result;
}

}  // namespace conex
