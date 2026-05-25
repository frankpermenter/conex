#include "conex/algorithms/geodesic_ipm_helpers.h"

#include "conex/algorithms/geodesic_ipm.h"

#include <cmath>
#include <cstdio>

#include "conex/common/equality_constraint.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

#include "conex/common/eja_ops.h"

namespace conex {

std::pair<double, double> VerifyNewtonEquations(
    CompiledModel& model,
    const RowSpace& b_data,
    const RowSpace& W,
    const RowSpace& d,
    const std::vector<double>& y,
    double k,
    double theta) {
  const auto& cost_rhs_data = model.cost_rhs();
  Arena& arena = model.arena();

  RowSpace ones = model.AllocRowSpace();
  setOnes(ones);

  // Blended b and c.
  RowSpace b = model.AllocRowSpace();
  addScaled(b, ones, b_data, theta, 1.0 - theta);

  auto cost_rhs = model.AllocSolverRHS();
  cost_rhs.SetZero();
  model.AccumulateAtranspose(ones, cost_rhs);
  cost_rhs *= theta;
  cost_rhs.AddScaled(1.0 - theta, cost_rhs_data);

  const int n = model.number_of_variables();

  // --- Primal check: d should equal e + P(W^{1/2})(-k*b - A*y) ---
  RowSpace sqrtW = model.AllocRowSpace();
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace I_minus_d = ones - d;
  // W^{-1/2} = P(W^{-1/4})... actually for the check we use:
  // d = I + P(W^{1/2})(S) where S = -k*b - A*y.
  // So I - d = -P(W^{1/2})(S) and P(W^{-1/2})(I - d) = -S = k*b + A*y.
  // Instead of computing W^{-1/2}, verify the equivalent:
  //   S_computed = P(W^{-1/2})(d - I) should equal -k*b - A*y.
  // Or simpler: verify d - I = P(W^{1/2})(-k*b - A*y).
  auto y_rhs = model.AllocSolverRHS();
  y_rhs.ScatterFrom(y.data(), y.size());
  RowSpace Ay = model.AllocRowSpace();
  model.MultiplyA(y_rhs, Ay);
  RowSpace slack = model.AllocRowSpace();
  addScaled(slack, b, Ay, -k, -1.0);  // -k*b - Ay
  RowSpace d_expected = model.AllocRowSpace();
  quadraticRepresentation(d_expected, sqrtW, slack);
  d_expected += ones;  // I + P(W^{1/2})(S)

  RowSpace primal_err = d - d_expected;
  double primal_res = normInf(primal_err);

  // --- Dual check: A'λ + (1/k)*Q*y = c ---
  // where λ = (1/k) * P(W^{1/2})(e + d).
  RowSpace I_plus_d = model.AllocRowSpace();
  addScaled(I_plus_d, ones, d, 1.0, 1.0);
  RowSpace lambda = model.AllocRowSpace();
  quadraticRepresentation(lambda, sqrtW, I_plus_d);
  lambda *= (1.0 / k);

  auto dual_rhs = model.AllocSolverRHS();
  dual_rhs.SetZero();
  model.AccumulateAtranspose(lambda, dual_rhs);
  if (model.has_quadratic_cost()) {
    auto qy = model.AllocSolverRHS();
    qy.SetZero();
    model.AccumulateQx(y_rhs, qy);
    dual_rhs.AddScaled(1.0 / k, qy);
  }
  dual_rhs -= cost_rhs;
  Eigen::VectorXd dual_err(n);
  dual_rhs.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

  return {primal_res, dual_res};
}

KKTResidual VerifyKKT(
    CompiledModel& model,
    const RowSpace& W,
    const RowSpace& d,
    const std::vector<double>& x,
    double k) {
  const int n = model.number_of_variables();
  const auto& cost_rhs = model.cost_rhs();

  RowSpace ones = model.AllocRowSpace();
  setOnes(ones);
  RowSpace sqrtW = model.AllocRowSpace();
  EuclideanJordanAlgebra::sqrt(sqrtW, W);

  // λ(k) = (1/k) * P(W^{1/2})(e + d(k))
  RowSpace e_plus_d = model.AllocRowSpace();
  addScaled(e_plus_d, ones, d, 1.0, 1.0);
  RowSpace lambda = model.AllocRowSpace();
  quadraticRepresentation(lambda, sqrtW, e_plus_d);
  lambda *= (1.0 / k);

  // s(k) = (1/k) * P(W^{-1/2})(e - d(k))
  RowSpace e_minus_d = model.AllocRowSpace();
  addScaled(e_minus_d, ones, d, 1.0, -1.0);
  RowSpace sqrtWinv = model.AllocRowSpace();
  EuclideanJordanAlgebra::inverse(sqrtWinv, sqrtW);
  RowSpace s = model.AllocRowSpace();
  quadraticRepresentation(s, sqrtWinv, e_minus_d);
  s *= (1.0 / k);

  auto x_rhs = model.AllocSolverRHS();
  x_rhs.ScatterFrom(x.data(), x.size());

  // Primal: A*x + b - s = 0
  RowSpace Ax = model.AllocRowSpace();
  model.MultiplyA(x_rhs, Ax);
  Ax += model.GetAffineTerm();
  Ax -= s;
  double primal_res = normInf(Ax);

  // Dual: A'λ + C'ν - Q*x - c = 0
  // (ν is the equality dual embedded in x at saddle-point positions;
  //  C'ν comes from AccumulateCtranspose.)
  auto dual_rhs = model.AllocSolverRHS();
  dual_rhs.SetZero();
  model.AccumulateAtranspose(lambda, dual_rhs);
  // Add C'ν via saddle-point: AccumulateCtranspose(x) gives [C'ν; Cx].
  // Subtract because saddle-point ν has opposite sign to Lagrangian ν.
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto ct = model.kkt().MakeSolverRHS();
    ct.SetZero();
    ts->AccumulateCtranspose(x_rhs, ct);
    dual_rhs -= ct;
  }
  if (model.has_quadratic_cost()) {
    auto qx = model.AllocSolverRHS();
    qx.SetZero();
    model.AccumulateQx(x_rhs, qx);
    dual_rhs -= qx;
  }
  dual_rhs -= cost_rhs;
  // Subtract equality affine term (d at dual positions).
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    dual_rhs += d_rhs;
  }
  Eigen::VectorXd dual_err(n);
  model.kkt().GatherInto(dual_rhs, dual_err);
  double dual_res = dual_err.cwiseAbs().maxCoeff();

  return {primal_res, dual_res};
}

OptimalityReport CheckOptimality(
    CompiledModel& model,
    const SolverRHS& x_rhs,
    const RowSpace& lambda) {
  OptimalityReport report;
  Arena& arena = model.arena();
  int n = model.number_of_variables();

  // s = Ax + b.
  RowSpace s = model.AllocRowSpace();
  model.MultiplyA(x_rhs, s);
  s += model.GetAffineTerm();

  // Cone membership.
  report.min_slack = minEigenvalue(s);
  report.min_dual = minEigenvalue(lambda);

  // Complementarity: <s, λ>.
  report.complementarity = dot(s, lambda);

  return report;
}

// Core solve pattern shared by all Newton-step variants.
// Adds A^T(cone_rhs) to var_rhs, solves with the current factorization,
// and returns A*x (for direction recovery).  Optionally outputs x.
RowSpace SolveConeSystem(
    CompiledModel& model,
    SolverRHS& var_rhs,               // variable-space RHS (modified: += A^T cone_rhs, then solved)
    const RowSpace& cone_rhs,         // cone-space RHS (accumulated via A^T)
    Eigen::VectorXd* x_out) {
  model.AccumulateAtranspose(cone_rhs, var_rhs);
  model.SolveSolverRHS(var_rhs);
  if (x_out) {
    int nr = model.number_of_variables();
    x_out->resize(nr);
    var_rhs.supernodes->GatherInto(*x_out);
  }
  Arena& arena = model.arena();
  RowSpace Ax = model.AllocRowSpace();
  model.MultiplyA(var_rhs, Ax);
  return Ax;
}

// Build the cost (variable-space) RHS: -cost_scale*(c) + eq_scale*d_eq.
// cost_scale and eq_scale are usually the same (k for GeodesicLP, 1 for
// decomposition) but differ for the Hybrid where cost is pre-scaled by tau
// but equality RHS needs explicit tau scaling.
SolverRHS MakeCostVarRHS(
    CompiledModel& model,
    const SolverRHS& cost_rhs,
    double cost_scale,
    double eq_scale) {
  auto rhs = model.AllocSolverRHS();
  rhs = cost_rhs;
  rhs *= -cost_scale;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    d_rhs *= eq_scale;
    rhs += d_rhs;
  }
  return rhs;
}

// Convenience: same scale for cost and equality.
SolverRHS MakeCostVarRHS(
    CompiledModel& model,
    const SolverRHS& cost_rhs,
    double scale) {
  return MakeCostVarRHS(model, cost_rhs, scale, scale);
}

// Direct Newton step: factor, single RHS, solve, compute d and y.
// Matches what GeodesicCenter does per iteration.
void ComputeDirectNewtonStep(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    double k,
    RowSpace& d_out,
    Eigen::VectorXd& y_out,
    RowSpace* slack_out) {
  Arena& arena = model.arena();
  model.SetScaling(W);

  // Combined RHS: centering (2W) + cost (-k·P(W)(b)) in cone space,
  //               -k·c + k·d_eq in variable space.
  auto var_rhs = MakeCostVarRHS(model, model.cost_rhs(), k);
  RowSpace tmp_qr = model.AllocRowSpace();
  quadraticRepresentation(tmp_qr, W, b);
  RowSpace cone_rhs = model.AllocRowSpace();
  addScaled(cone_rhs, tmp_qr, W, -k, 2.0);
  RowSpace Ax = SolveConeSystem(model, var_rhs, cone_rhs);
  RowSpace slack = model.AllocRowSpace();
  addScaled(slack, b, Ax, -k, -1.0);
  if (slack_out) *slack_out = slack;

  RowSpace sqrtW = model.AllocRowSpace();
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  quadraticRepresentation(d_out, sqrtW, slack);
  RowSpace ones = model.AllocRowSpace();
  setOnes(ones);
  d_out += ones;

  int nr = model.number_of_variables();
  y_out.resize(nr);
  var_rhs.supernodes->GatherInto(y_out);
}

// Factor, two back-solves, 2-column MultiplyA → compute d0, d1.
void ComputeDecomposition(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    RowSpace& d0,
    RowSpace& d1,
    SolverRHS* y0_out,
    SolverRHS* y1_out) {
  // Allocate outputs as heap-backed SolverRHS so they survive
  // ArenaFrame (arena allocations would be freed).
  // Copy pointers directly (operator= does deep copy which needs non-null supernodes).
  auto initRHS = [&](SolverRHS& dest) {
    SolverRHS tmp = model.MakeSolverRHS();
    dest.supernodes = tmp.supernodes;
    dest.separators = tmp.separators;
    dest.blocks_fully_gathered = tmp.blocks_fully_gathered;
  };
  if (y0_out) initRHS(*y0_out);
  if (y1_out) initRHS(*y1_out);
  ArenaFrame frame(arena);
  model.SetScaling(W);

  RowSpace v = model.AllocRowSpace(arena);

  auto rhs0 = model.AllocSolverRHS();
  rhs0.SetZero();
  v += W;  // per-segment copy
  v *= 2.0;
  model.AccumulateAtranspose(v, rhs0);

  auto rhs1 = model.AllocSolverRHS();
  rhs1 = cost_rhs;
  quadraticRepresentation(v, W, b);
  model.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;
  // Inject equality RHS: +d at dual positions.
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    rhs1 += d_rhs;
  }

  auto y = model.AllocSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  model.SolveSolverRHS(y);

  if (y0_out || y1_out) {
    int nb = y.num_blocks();
    if (y0_out) {
      for (int k = 0; k < nb; ++k)
        y0_out->supernodes->block(k).col(0) = y.supernodes->block(k).col(0);
    }
    if (y1_out) {
      for (int k = 0; k < nb; ++k)
        y1_out->supernodes->block(k).col(0) = y.supernodes->block(k).col(1);
    }
  }

  auto row = model.AllocRowSpace(arena, 2);
  model.MultiplyA(y, row);

  RowSpace ay0 = model.AllocRowSpace(arena);
  RowSpace ay1 = model.AllocRowSpace(arena);
  ay0.col() = row.col(0);
  ay1.col() = row.col(1);

  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  setOnes(d0);
  RowSpace tmp = model.AllocRowSpace(arena);
  quadraticRepresentation(tmp, sqrtW, ay0);
  d0 -= tmp;

  RowSpace neg_b_ay1 = model.AllocRowSpace(arena);
  addScaled(neg_b_ay1, b, ay1, -1.0, -1.0);
  quadraticRepresentation(d1, sqrtW, neg_b_ay1);
}

// Build the "duality cost" for the V(tau)=0 identity:
//   b^T lambda + c^T x + d^T nu + mu/tau = theta * R
// cost_rhs has [c; 0] (primal cost, zeros at dual positions).
// duality_cost adds +d at dual positions so that
// duality_cost.dot(y) = c^T x + d^T nu.
SolverRHS MakeDualityCost(CompiledModel& model) {
  const auto& cost_rhs = model.cost_rhs();
  auto duality_cost = model.AllocSolverRHS();
  duality_cost = cost_rhs;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    duality_cost += d_rhs;
  }
  return duality_cost;
}

void ComputeFullDecomposition(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    NewtonDecomposition& decomp) {
  Arena& arena = model.arena();
  // Allocate outputs as heap-backed SolverRHS so they survive
  // ArenaFrame (arena allocations would be freed).
  // Heap-allocate outputs so they survive ArenaFrame.
  // Copy pointers directly (operator= does deep copy which requires
  // supernodes != null, but default-constructed SolverRHS has null).
  auto initRHS = [&](SolverRHS& dest) {
    SolverRHS tmp = model.MakeSolverRHS();
    dest.supernodes = tmp.supernodes;
    dest.separators = tmp.separators;
    dest.blocks_fully_gathered = tmp.blocks_fully_gathered;
  };
  initRHS(decomp.y0);
  initRHS(decomp.y1_0);
  initRHS(decomp.y1_theta);
  ArenaFrame frame(arena);
  const auto& cost_rhs = model.cost_rhs();
  model.SetScaling(W);

  RowSpace ones = model.AllocRowSpace(arena);
  setOnes(ones);
  RowSpace v = model.AllocRowSpace(arena);

  auto rhs0 = model.AllocSolverRHS();
  rhs0.SetZero();
  v += W;
  v *= 2.0;
  model.AccumulateAtranspose(v, rhs0);

  auto rhs1 = model.AllocSolverRHS();
  rhs1 = cost_rhs;
  quadraticRepresentation(v, W, b);
  model.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts && !ts->equality_sub_assemblers().empty()) {
    rhs1 += ts->EqualityAffineTermRHS();
  }

  auto rhs2 = model.AllocSolverRHS();
  rhs2 = rhs1;
  rhs2 *= -1;
  quadraticRepresentation(v, W, ones);
  v += ones;
  v *= -1.0;
  model.AccumulateAtranspose(v, rhs2);

  auto y = model.AllocSolverRHS(3);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  y.SetColumn(2, rhs2);
  model.SolveSolverRHS(y);

  {
    int nb = y.num_blocks();
    for (int k = 0; k < nb; ++k) {
      decomp.y0.supernodes->block(k).col(0) = y.supernodes->block(k).col(0);
      decomp.y1_0.supernodes->block(k).col(0) = y.supernodes->block(k).col(1);
      decomp.y1_theta.supernodes->block(k).col(0) = y.supernodes->block(k).col(2);
    }
  }

  auto row = model.AllocRowSpace(arena, 3);
  model.MultiplyA(y, row);

  RowSpace ay0 = model.AllocRowSpace(arena);
  RowSpace ay1_0 = model.AllocRowSpace(arena);
  RowSpace ay1_theta = model.AllocRowSpace(arena);
  ay0.col() = row.col(0);
  ay1_0.col() = row.col(1);
  ay1_theta.col() = row.col(2);

  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);

  // d0 = e - P(W^{1/2})(A y0)
  setOnes(decomp.d0);
  RowSpace tmp = model.AllocRowSpace(arena);
  quadraticRepresentation(tmp, sqrtW, ay0);
  decomp.d0 -= tmp;

  // d1_0 = P(W^{1/2})(-b_0 - A y1_0)
  addScaled(tmp, b, ay1_0, -1.0, -1.0);
  quadraticRepresentation(decomp.d1_0, sqrtW, tmp);

  // d1_theta = P(W^{1/2})(b_0 - e - A y1_theta)
  addScaled(v, b, ones, 1.0, -1.0);
  addScaled(tmp, v, ay1_theta, 1.0, -1.0);
  quadraticRepresentation(decomp.d1_theta, sqrtW, tmp);
}

void EvaluateDirection(RowSpace& out, const NewtonDecomposition& decomp,
                       double k, double tau, double theta) {
  // d(k, tau, theta) = d0 + k * (tau * d1_0 + theta * d1_theta)
  addScaled(out, decomp.d1_0, decomp.d1_theta, tau, theta);
  addScaled(out, decomp.d0, out, 1.0, k);
}

// Allocating convenience wrapper.
RowSpace EvaluateDirection(const NewtonDecomposition& decomp,
                           double k, double tau, double theta) {
  RowSpace out = like(decomp.d0);
  EvaluateDirection(out, decomp, k, tau, theta);
  return out;
}

double MinNormK(const NewtonDecomposition& decomp, double tau, double theta) {
  RowSpace d1 = addScaled(decomp.d1_0, decomp.d1_theta, tau, theta);
  double d1_sq = squaredNorm(d1);
  double d0_d1 = dot(decomp.d0, d1);
  if (d1_sq < 1e-30) return 0.0;
  return -d0_d1 / d1_sq;
}

DecompInnerProducts ComputeInnerProducts(const NewtonDecomposition& decomp) {
  return {
    squaredNorm(decomp.d0),
    dot(decomp.d0, decomp.d1_0),
    dot(decomp.d0, decomp.d1_theta),
    squaredNorm(decomp.d1_0),
    dot(decomp.d1_0, decomp.d1_theta),
    squaredNorm(decomp.d1_theta)
  };
}

// ||d||^2 as a function of k, with tau = tau*(k) and theta = 1/k^2.
static double ReducedDSq(const DecompInnerProducts& ip, double k) {
  double k2 = k * k;
  double num = k * ip.f + ip.q;
  // a + 2c/k + r/k^2 - (kb + q)^2 / (k^2 p)
  return ip.a + 2.0 * ip.g / k + ip.r / k2 - (num * num) / (k2 * ip.p);
}

KTauResult SelectKTau(const DecompInnerProducts& ip) {
  // Golden-section search over k > 0 to minimize ReducedDSq.
  // Bracket: start from k=1, expand to find a bracket.
  double k_lo = 1e-6, k_hi = 1e6;

  // Refine with golden section.
  const double gr = (std::sqrt(5.0) + 1.0) / 2.0;
  for (int i = 0; i < 100; ++i) {
    double k1 = k_hi - (k_hi - k_lo) / gr;
    double k2 = k_lo + (k_hi - k_lo) / gr;
    if (ReducedDSq(ip, k1) < ReducedDSq(ip, k2)) {
      k_hi = k2;
    } else {
      k_lo = k1;
    }
  }

  double k = 0.5 * (k_lo + k_hi);
  double tau = -(k * ip.f + ip.q) / (k * k * ip.p);
  double theta = 1.0 / (k * k);
  double d_sq = ReducedDSq(ip, k);

  return {k, tau, theta, d_sq};
}

DualityCoeffs ComputeDualityCoeffs(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp) {
  ArenaFrame frame(arena);
  // sigma1 = <b0, P(W^{1/2})(d1_0)>
  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace Pw_d1_0 = model.AllocRowSpace(arena);
  quadraticRepresentation(Pw_d1_0, sqrtW, decomp.d1_0);
  double sigma1 = dot(b, Pw_d1_0);

  // gamma1 = duality_cost^T y1_0  (uses +d at dual positions, not -d)
  double gamma1 = duality_cost.dot(decomp.y1_0);

  // q11 = y1_0' Q y1_0  (quadratic cost contribution to beta)
  auto qy1 = model.AllocSolverRHS();
  qy1.SetZero();
  model.AccumulateQx(decomp.y1_0, qy1);
  double q11 = qy1.dot(decomp.y1_0);

  return {sigma1, gamma1, q11};
}

ThetaCandidateCoeffs PrecomputeThetaCoeffs(
    CompiledModel& model, Arena& arena,
    const SolverRHS& duality_cost, const RowSpace& b,
    const RowSpace& W, const NewtonDecomposition& decomp,
    double bT_ones) {
  ArenaFrame frame(arena);
  ThetaCandidateCoeffs c;
  c.bT_ones = bT_ones;

  auto dc = ComputeDualityCoeffs(model, arena, duality_cost, b, W, decomp);
  c.beta = dc.sigma1 + dc.gamma1 + dc.q11;

  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace ones_v = model.AllocRowSpace(arena);
  setOnes(ones_v);
  RowSpace e_plus_d0 = model.AllocRowSpace(arena);
  addScaled(e_plus_d0, ones_v, decomp.d0, 1.0, 1.0);

  RowSpace P_ed0 = model.AllocRowSpace(arena);
  quadraticRepresentation(P_ed0, sqrtW, e_plus_d0);
  c.bT_P_ed0 = dot(b, P_ed0);

  RowSpace P_d1t = model.AllocRowSpace(arena);
  quadraticRepresentation(P_d1t, sqrtW, decomp.d1_theta);
  c.bT_P_d1t = dot(b, P_d1t);

  c.cT_y0 = duality_cost.dot(decomp.y0);
  c.cT_yt = duality_cost.dot(decomp.y1_theta);

  // Q dot products.
  c.has_Q = model.has_quadratic_cost();
  c.q00 = c.q0t = c.qtt = c.q01 = c.qt1 = 0;
  if (c.has_Q) {
    auto Qy0 = model.AllocSolverRHS(); Qy0.SetZero();
    model.AccumulateQx(decomp.y0, Qy0);
    auto Qyt = model.AllocSolverRHS(); Qyt.SetZero();
    model.AccumulateQx(decomp.y1_theta, Qyt);
    c.q00 = Qy0.dot(decomp.y0);
    c.q0t = Qy0.dot(decomp.y1_theta);
    c.qtt = Qyt.dot(decomp.y1_theta);
    c.q01 = Qy0.dot(decomp.y1_0);
    c.qt1 = Qyt.dot(decomp.y1_0);
  }

  c.ip = ComputeInnerProducts(decomp);

  return c;
}

// Fast theta evaluation using precomputed coefficients (no eigendecomps).
std::pair<double, double> EvalThetaFast(
    const ThetaCandidateCoeffs& c,
    const DecompInnerProducts& ip,
    double theta_cand) {
  if (theta_cand <= 0) return {-1, 1e30};
  double k = 1.0 / std::sqrt(theta_cand);
  double mu = theta_cand;

  // sigma0 = dot(b, P(sqrtW, e+d0 + k*theta*d1_theta)) / k
  //        = (bT_P_ed0 + k*theta*bT_P_d1t) / k
  //        = bT_P_ed0/k + theta*bT_P_d1t
  double sigma0 = c.bT_P_ed0 / k + theta_cand * c.bT_P_d1t;

  double gamma0 = c.cT_y0 / k + theta_cand * c.cT_yt;

  // q_ff = (y0/k + theta*yt)' Q (y0/k + theta*yt)
  double q_ff = c.q00 / (k * k) + 2 * theta_cand * c.q0t / k
              + theta_cand * theta_cand * c.qtt;
  // q_f1 = (y0/k + theta*yt)' Q y1_0
  double q_f1 = c.q01 / k + theta_cand * c.qt1;

  double alpha = sigma0 + gamma0 + 2.0 * q_f1;
  double R = theta_cand * (c.bT_ones + 1.0);
  double mu_eff = mu + q_ff;

  double B = alpha - R;
  double disc = B * B - 4.0 * c.beta * mu_eff;
  if (disc < 0) return {-1, 1e30};

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * c.beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * c.beta);

  // ||d||^2 from inner products: d = d0 + k*(tau*d1_0 + theta*d1_theta)
  auto dsq = [&](double tau) -> double {
    if (tau <= 0) return 1e30;
    double t = tau, th = theta_cand;
    // ||d0 + k*(t*d1_0 + th*d1_theta)||^2
    return ip.a + 2*k*(t*ip.f + th*ip.g) + k*k*(t*t*ip.p + 2*t*th*ip.q + th*th*ip.r);
  };

  double dsq1 = dsq(tau1);
  double dsq2 = dsq(tau2);

  double tau_out;
  if (tau1 > 0 && (tau2 <= 0 || dsq1 <= dsq2)) {
    tau_out = tau1;
  } else if (tau2 > 0) {
    tau_out = tau2;
  } else {
    return {-1, 1e30};
  }

  return {tau_out, 0.0};  // d_inf requires normInf; use EvalThetaCandidate for exact check
}

// Frozen-J variant: recompute only the d0-dependent terms.
// sqrtW0 and bT_P_d1t are frozen; d0, y0, Wi change each inner iter.
ThetaCandidateCoeffs RefreshFrozenThetaCoeffs(
    CompiledModel& model, Arena& arena,
    const SolverRHS& duality_cost, const RowSpace& b,
    const RowSpace& Wi, const RowSpace& sqrtW0,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double frozen_bT_P_d1t,   // dot(b, P(sqrtW0, d1_theta)) — frozen
    double frozen_beta,       // sigma1 + gamma1 + q11 — frozen
    const ThetaCandidateCoeffs& frozen_base) {
  ArenaFrame frame(arena);
  ThetaCandidateCoeffs c = frozen_base;  // copy frozen Q products etc.
  c.beta = frozen_beta;
  c.bT_P_d1t = frozen_bT_P_d1t;

  // Recompute d0-dependent terms.
  RowSpace P_d0 = model.AllocRowSpace(arena);
  quadraticRepresentation(P_d0, sqrtW0, decomp.d0);
  double bT_P_d0 = dot(b, P_d0);
  double bT_Wi = dot(b, Wi);
  // sigma0_frozen = bT_Wi/k + (bT_P_d0 + k*theta*bT_P_d1t)/k
  //              = (bT_Wi + bT_P_d0)/k + theta*bT_P_d1t
  // Match the outer formula: sigma0 = bT_P_ed0/k + theta*bT_P_d1t
  // So bT_P_ed0 for frozen = bT_Wi + bT_P_d0.
  c.bT_P_ed0 = bT_Wi + bT_P_d0;

  c.cT_y0 = duality_cost.dot(decomp.y0);
  // cT_yt is frozen (y1_theta doesn't change).

  // Refresh Q products involving y0 (y1_theta and y1_0 are frozen).
  if (c.has_Q) {
    auto Qy0 = model.AllocSolverRHS(); Qy0.SetZero();
    model.AccumulateQx(decomp.y0, Qy0);
    c.q00 = Qy0.dot(decomp.y0);
    c.q0t = Qy0.dot(decomp.y1_theta);
    c.q01 = Qy0.dot(decomp.y1_0);
    // qtt and qt1 are frozen.
  }

  c.ip = ComputeInnerProducts(decomp);

  return c;
}

// Full evaluation (with exact d_inf) for the final chosen theta.
std::pair<double, double> EvalThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const ThetaCandidateCoeffs& c,
    const NewtonDecomposition& decomp,
    double theta_cand) {
  auto [tau, d_inf_approx] = EvalThetaFast(c, c.ip, theta_cand);
  if (tau <= 0) return {-1, 1e30};

  // Compute exact d_inf via normInf.
  ArenaFrame frame(arena);
  double k = 1.0 / std::sqrt(theta_cand);
  RowSpace d = model.AllocRowSpace(arena);
  EvaluateDirection(d, decomp, k, tau, theta_cand);
  double dinf = normInf(d);
  return {tau, dinf};
}

// Frozen-Jacobian variant of EvalThetaCandidate.
// Uses W0 for the Jacobian (Gram, sqrt, P(W)) and Wi for the value
// (the b^T Wi/k term in sigma0).  decomp contains d0 refreshed at Wi,
// d1_0 and d1_theta frozen at W0.
std::pair<double, double> FrozenEvalThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& Wi,       // current iterate
    const RowSpace& sqrtW0,   // precomputed sqrt(W0)
    const NewtonDecomposition& decomp,
    double bT_ones,
    double beta,              // precomputed sigma1 + gamma1 + q11
    double theta_cand) {
  if (theta_cand <= 0) return {-1, 1e30};
  ArenaFrame frame(arena);
  double k = 1.0 / std::sqrt(theta_cand);
  double mu = theta_cand;
  RowSpace arg = model.AllocRowSpace(arena);
  addScaled(arg, decomp.d0, decomp.d1_theta, 1.0, k * theta_cand);
  RowSpace Parg = model.AllocRowSpace(arena);
  quadraticRepresentation(Parg, sqrtW0, arg);
  double sigma0 = dot(b, Wi) / k + dot(b, Parg) / k;

  double cT_y0 = duality_cost.dot(decomp.y0);
  double cT_yt = duality_cost.dot(decomp.y1_theta);
  double gamma0 = cT_y0 / k + theta_cand * cT_yt;

  auto f_rhs = model.AllocSolverRHS();
  f_rhs.SetZero();
  f_rhs.AddScaled(1.0 / k, decomp.y0);
  f_rhs.AddScaled(theta_cand, decomp.y1_theta);
  auto qf = model.AllocSolverRHS();
  qf.SetZero();
  model.AccumulateQx(f_rhs, qf);
  double q_ff = qf.dot(f_rhs);
  double q_f1 = qf.dot(decomp.y1_0);

  double alpha = sigma0 + gamma0 + 2.0 * q_f1;
  double R = theta_cand * (bT_ones + 1.0);
  double mu_eff = mu + q_ff;

  double B = alpha - R;
  double disc = B * B - 4.0 * beta * mu_eff;
  if (disc < 0) { return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  auto eval_dsq = [&](double tau) -> double {
    if (tau <= 0) return 1e30;
    ArenaFrame eval_frame(arena);
    RowSpace d = model.AllocRowSpace(arena);
    EvaluateDirection(d, decomp, k, tau, theta_cand);
    double r = squaredNorm(d);
    return r;
  };

  double dsq1 = eval_dsq(tau1);
  double dsq2 = eval_dsq(tau2);

  double tau_out;
  if (tau1 > 0 && (tau2 <= 0 || dsq1 <= dsq2)) {
    tau_out = tau1;
  } else if (tau2 > 0) {
    tau_out = tau2;
  } else {
    return {-1, 1e30};
  }

  RowSpace d = model.AllocRowSpace(arena);
  EvaluateDirection(d, decomp, k, tau_out, theta_cand);
  double dinf = normInf(d);
  return {tau_out, dinf};
}

// Binary search for smallest theta with d_inf <= beta_target.
// Returns (theta, k, tau). Sets tau <= 0 if no feasible theta found.
ThetaSearchResult BisectTheta(
    CompiledModel& model, Arena& arena,
    const ThetaCandidateCoeffs& tc,
    const NewtonDecomposition& decomp,
    double theta_lo, double theta_hi, double beta_target) {
  for (int bisect = 0; bisect < 30; ++bisect) {
    double theta_mid = 0.5 * (theta_lo + theta_hi);
    auto [tau_try, d_inf_try] = EvalThetaCandidate(model, arena, tc, decomp, theta_mid);
    if (tau_try > 0 && d_inf_try <= beta_target)
      theta_hi = theta_mid;
    else
      theta_lo = theta_mid;
  }
  double theta = theta_hi;
  double k = 1.0 / std::sqrt(theta);
  auto [tau, d_inf] = EvalThetaCandidate(model, arena, tc, decomp, theta);
  return {theta, k, tau};
}

// Verbose stats for ThetaCont iterations.  Computes objectives, equation error,
// and prints a single line.  Used by both outer and frozen-J paths.
double PrintThetaContStats(
    CompiledModel& model,
    const NewtonDecomposition& decomp,
    const SolverRHS& cost_rhs,
    const SolverRHS& duality_cost,
    double bT_ones, double nu,
    int outer, int inner,  // inner < 0 for outer step
    double k, double tau, double theta,
    double d_inf, double d_sq, double gap) {
  auto x_rhs = model.AllocSolverRHS();
  x_rhs.SetZero();
  x_rhs.AddScaled(1.0 / k, decomp.y0);
  x_rhs.AddScaled(tau, decomp.y1_0);
  x_rhs.AddScaled(theta, decomp.y1_theta);
  double cTx = duality_cost.dot(x_rhs);
  auto qx = model.AllocSolverRHS(); qx.SetZero();
  model.AccumulateQx(x_rhs, qx);
  double xQx = qx.dot(x_rhs);
  double mu = 1.0 / (k * k);
  double mu_tau = (tau > 1e-30) ? mu / tau : 0.0;
  double xQx_tau = (tau > 1e-30) ? xQx / tau : 0.0;
  double R = theta * (bT_ones + 1.0);
  // Equation error: V(tau) = bTl + cTx + xQx/tau + mu/tau - theta*R.
  // We don't have lambda here; approximate eq_err from the duality identity.
  double eq_err = 0;  // filled by caller if lambda available

  double cTx_cost = cost_rhs.dot(x_rhs);
  double half_xQx = (tau > 1e-30) ? 0.5 * xQx / (tau * tau) : 0.0;
  double primal = (tau > 1e-30) ? cTx_cost / tau + half_xQx : 0.0;
  double dual = primal + gap / std::max(std::abs(tau), 1e-30);

  if (inner < 0) {
    printf("  %3d    %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
           "  %12.4e  %12.4e  %12.4e  %12.2e\n",
           outer, theta, tau, mu_tau, k, d_inf, d_sq, gap,
           dual, primal, mu_tau, eq_err);
  } else {
    printf("  %3d.%d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
           "  %12.4e  %12.4e  %12.4e  %12.2e  (frozen-J)\n",
           outer, inner + 1, theta, tau, mu_tau, k, d_inf, d_sq, gap,
           dual, primal, mu_tau, eq_err);
  }
  return eq_err;
}

// Core frozen-Jacobian d0 refresh: 1 back-solve with stale Gram.
// Computes d0 = P(sqrt(W0))(Wi^{-1} - Ay0) and optionally outputs y0.
void RefreshD0Frozen(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    RowSpace& d0_out,
    SolverRHS& y0_out) {
  ArenaFrame frame(arena);
  RowSpace Wi_inv = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::inverse(Wi_inv, Wi);
  RowSpace sqrtW0 = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW0, W0);

  RowSpace PW0_Winv = model.AllocRowSpace(arena);
  quadraticRepresentation(PW0_Winv, W0, Wi_inv);
  RowSpace center = model.AllocRowSpace(arena);
  addScaled(center, Wi, PW0_Winv, 1.0, 1.0);
  auto var_rhs = model.AllocSolverRHS();
  var_rhs.SetZero();
  RowSpace Ax = SolveConeSystem(model, var_rhs, center);
  // Copy solved y0 into the caller's pre-allocated buffer.
  {
    int nb = var_rhs.num_blocks();
    for (int k = 0; k < nb; ++k)
      y0_out.supernodes->block(k).col(0) = var_rhs.supernodes->block(k).col(0);
  }

  RowSpace tmp = model.AllocRowSpace(arena);
  addScaled(tmp, Wi_inv, Ax, 1.0, -1.0);
  quadraticRepresentation(d0_out, sqrtW0, tmp);
}

// Overload for NewtonDecomposition (ThetaCont frozen-J).
void RefreshD0Frozen(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    NewtonDecomposition& decomp) {
  RefreshD0Frozen(model, arena, b, W0, Wi, decomp.d0, decomp.y0);
}

// Helper: evaluate the V(τ)=0 quadratic for the z-space θ-continuation.
// Returns (τ, stepSize_alpha) or (-1, 1e30) if no positive root.
//
// z_hess: point at which Hessian products are evaluated (= z for standard,
//         = z₀ for frozen-J where the Gram was factored at z₀).
// z:      current iterate (used for lineSearchTarget / hessianNormSquared
//         feasibility checks).
// grad_z: ∇F(z) at the current iterate (not z_hess).
std::pair<double, double> EvalBarrierThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& z,
    const RowSpace& z_hess,
    const RowSpace& b,
    const RowSpace& grad_z,       // ∇F(z) at current z
    const RowSpace& ay0,          // A y0
    const RowSpace& t1_tau,       // Ay1_0 + b
    const RowSpace& t1_th,        // Ay1_theta + z0
    const SolverRHS& y0_rhs,
    const SolverRHS& y1_0_rhs,
    const SolverRHS& y1_theta_rhs,
    double nu, double R_theta1, double theta_cand) {
  if (theta_cand <= 0) return {-1, 1e30};
  ArenaFrame frame(arena);

  double k = 1.0 / std::sqrt(theta_cand);
  double k2 = k * k;

  // σ₁ = -b^T H(z_hess)·t1_tau  (coefficient of τ in b^T λ)
  RowSpace h_t1_tau = model.AllocRowSpace(arena);
  hessianProduct(z_hess, t1_tau, h_t1_tau);
  double sigma1 = -dot(b, h_t1_tau);

  // γ₁ = duality_cost^T y1_0
  double gamma1 = duality_cost.dot(y1_0_rhs);

  // q_gg = y1_0^T Q y1_0
  auto qg = model.AllocSolverRHS();
  qg.SetZero();
  model.AccumulateQx(y1_0_rhs, qg);
  double q_gg = qg.dot(y1_0_rhs);

  double beta = sigma1 + gamma1 + q_gg;

  // σ₀ = (1/k) b^T (-2∇F(z) - H(z_hess)·(Ay₀ + kθ·t1_th))
  RowSpace target0_part = model.AllocRowSpace(arena);
  addScaled(target0_part, ay0, t1_th, 1.0, k * theta_cand);
  RowSpace h_target0 = model.AllocRowSpace(arena);
  hessianProduct(z_hess, target0_part, h_target0);
  RowSpace lam0_unscaled = model.AllocRowSpace(arena);
  addScaled(lam0_unscaled, grad_z, h_target0, -2.0, -1.0);
  double sigma0 = dot(b, lam0_unscaled) / k;

  // γ₀ = duality_cost^T (y0/k + θ·y1_theta)
  // f = y0/k + θ·y1_theta
  auto f_rhs = model.AllocSolverRHS();
  f_rhs.SetZero();
  f_rhs.AddScaled(1.0 / k, y0_rhs);
  f_rhs.AddScaled(theta_cand, y1_theta_rhs);
  double gamma0 = duality_cost.dot(f_rhs);

  // q_fg = f^T Q g, q_ff = f^T Q f
  auto qf = model.AllocSolverRHS();
  qf.SetZero();
  model.AccumulateQx(f_rhs, qf);
  double q_ff = qf.dot(f_rhs);
  double q_fg = qf.dot(y1_0_rhs);

  double B = sigma0 + gamma0 + 2.0 * q_fg - theta_cand * R_theta1;
  double mu_eff = q_ff + theta_cand;  // μ = θ = 1/k²

  // Solve: β·τ² + B·τ + μ_eff = 0.
  double disc = B * B - 4.0 * beta * mu_eff;
  if (disc < 0) { return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  // Pick the positive root with smaller ||d||_H².
  auto eval = [&](double tau) -> std::pair<double, double> {
    if (tau <= 0) return {1e30, 1e30};
    ArenaFrame eval_frame(arena);
    RowSpace t1_combined = model.AllocRowSpace(arena);
    addScaled(t1_combined, t1_tau, t1_th, tau, theta_cand);
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, ay0, t1_combined, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double k_max = lineSearchTarget(z, ay0, t1_combined);
    double d_inf = (k_max >= k) ? 0.0 : 2.0;  // feasible or not
    return {d_sq, d_inf};
  };

  auto [dsq1, dinf1] = eval(tau1);
  auto [dsq2, dinf2] = eval(tau2);

  double tau_out, d_inf_out;
  if (tau1 > 0 && (tau2 <= 0 || dsq1 <= dsq2)) {
    tau_out = tau1; d_inf_out = dinf1;
  } else if (tau2 > 0) {
    tau_out = tau2; d_inf_out = dinf2;
  } else {
    return {-1, 1e30};
  }

  return {tau_out, d_inf_out};
}

}  // namespace conex
