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

  // --- Dual check: A^T lambda = k*c where lambda = P(W^{1/2})(I + d) / k ---
  RowSpace I_plus_d = model.AllocRowSpace();
  addScaled(I_plus_d, ones, d, 1.0, 1.0);
  RowSpace lambda = model.AllocRowSpace();
  quadraticRepresentation(lambda, sqrtW, I_plus_d);
  lambda *= (1.0 / k);

  auto at_lambda = model.AllocSolverRHS();
  at_lambda.SetZero();
  model.AccumulateAtranspose(lambda, at_lambda);
  // Should equal c (the cost_rhs).
  at_lambda -= cost_rhs;
  Eigen::VectorXd dual_err(n);
  at_lambda.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

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
static RowSpace SolveConeSystem(
    CompiledModel& model,
    SolverRHS& var_rhs,               // variable-space RHS (modified: += A^T cone_rhs, then solved)
    const RowSpace& cone_rhs,         // cone-space RHS (accumulated via A^T)
    Eigen::VectorXd* x_out = nullptr) {
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
static SolverRHS MakeCostVarRHS(
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
static SolverRHS MakeCostVarRHS(
    CompiledModel& model,
    const SolverRHS& cost_rhs,
    double scale) {
  return MakeCostVarRHS(model, cost_rhs, scale, scale);
}

static void ComputeDirectNewtonStep(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W, double k,
    RowSpace& d_out, Eigen::VectorXd& y_out,
    RowSpace* slack_out = nullptr);

GeodesicResult GeodesicCenter(
    CompiledModel& model,
    RowSpace& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  const int m = W.total_rows();
  const double nu = barrierParameter(W);
  const double mu = 1.0 / (k * k);
  const RowSpace b = model.GetAffineTerm();
  Arena& arena = model.arena();

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = model.AllocRowSpace();
    RowSpace slack = model.AllocRowSpace();
    Eigen::VectorXd y_direct;
    ComputeDirectNewtonStep(model, b, W, k, d, y_direct, &slack);

    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));

    double s_dot_x = mu * (nu - d_sq);

    result.iterations = iter + 1;
    result.total_factorizations = iter + 1;
    result.total_solves = iter + 1;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;

    if (verbose) {
      auto [p_res, d_res] = VerifyNewtonEquations(
          model, b, W, d, std::vector<double>(y_direct.data(), y_direct.data() + y_direct.size()), k, 0.0);
      printf("  i=%2d  mu=%.2e  d_sqr=%.2e  d_inf=%.2e  "
             "s_dot_x=%.2e  alpha=%.4f  newton_err=(%.1e, %.1e)\n",
             iter, mu, d_sq, d_inf, s_dot_x, alpha, p_res, d_res);
    }

    if (d_inf < tolerance) {
      { Eigen::VectorXd tmp = y_direct / k; result.x.assign(tmp.data(), tmp.data() + tmp.size()); }
      break;
    }

    geodesicUpdateFromSlack(W, alpha, slack);
  }

  return result;
}

// Direct Newton step: factor, single RHS, solve, compute d and y.
// Matches what GeodesicCenter does per iteration.
static void ComputeDirectNewtonStep(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    double k,
    RowSpace& d_out,
    Eigen::VectorXd& y_out,
    RowSpace* slack_out) {
  Arena& arena = model.arena();
  model.SetScaling(W);
  model.AssembleAndFactor();

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
static void ComputeDecomposition(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    RowSpace& d0,
    RowSpace& d1,
    SolverRHS* y0_out = nullptr,
    SolverRHS* y1_out = nullptr) {
  // Allocate outputs as heap-backed SolverRHS so they survive
  // RestoreCursor (arena allocations would be freed).
  // Copy pointers directly (operator= does deep copy which needs non-null supernodes).
  auto initRHS = [&](SolverRHS& dest) {
    SolverRHS tmp = model.MakeSolverRHS();
    dest.supernodes = tmp.supernodes;
    dest.separators = tmp.separators;
    dest.blocks_fully_gathered = tmp.blocks_fully_gathered;
  };
  if (y0_out) initRHS(*y0_out);
  if (y1_out) initRHS(*y1_out);
  char* mark = arena.SaveCursor();
  model.SetScaling(W);
  model.AssembleAndFactor();

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
  arena.RestoreCursor(mark);
}

// Build the "duality cost" for the V(tau)=0 identity:
//   b^T lambda + c^T x + d^T nu + mu/tau = theta * R
// cost_rhs has [c; 0] (primal cost, zeros at dual positions).
// duality_cost adds +d at dual positions so that
// duality_cost.dot(y) = c^T x + d^T nu.
static SolverRHS MakeDualityCost(CompiledModel& model) {
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
  // RestoreCursor (arena allocations would be freed).
  // Heap-allocate outputs so they survive RestoreCursor.
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
  char* mark = arena.SaveCursor();
  const auto& cost_rhs = model.cost_rhs();
  model.SetScaling(W);
  model.AssembleAndFactor();

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

  arena.RestoreCursor(mark);
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
  char* mark = arena.SaveCursor();
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

  arena.RestoreCursor(mark);
  return {sigma1, gamma1, q11};
}

// Given a decomposition and candidate theta, solve the hard-constraint
// violation quadratic V(tau)=0, evaluate d, return (tau, d_inf).
// Returns tau = -1 if no positive root exists.
static std::pair<double, double> EvalThetaCandidate(
    CompiledModel& model,
    Arena& arena,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double theta_cand) {
  if (theta_cand <= 0) return {-1, 1e30};
  char* mark = arena.SaveCursor();
  double k = 1.0 / std::sqrt(theta_cand);
  double mu = theta_cand;

  auto dc = ComputeDualityCoeffs(model, arena, duality_cost, b, W, decomp);
  double beta = dc.sigma1 + dc.gamma1 + dc.q11;

  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace ones_v = model.AllocRowSpace(arena);
  setOnes(ones_v);
  RowSpace e_plus_d0 = model.AllocRowSpace(arena);
  addScaled(e_plus_d0, ones_v, decomp.d0, 1.0, 1.0);
  RowSpace arg = model.AllocRowSpace(arena);
  addScaled(arg, e_plus_d0, decomp.d1_theta, 1.0, k * theta_cand);
  RowSpace Parg = model.AllocRowSpace(arena);
  quadraticRepresentation(Parg, sqrtW, arg);
  double sigma0 = dot(b, Parg) / k;

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
  if (disc < 0) { arena.RestoreCursor(mark); return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  auto eval_dsq = [&](double tau) -> double {
    if (tau <= 0) return 1e30;
    char* em = arena.SaveCursor();
    RowSpace d = model.AllocRowSpace(arena);
    EvaluateDirection(d, decomp, k, tau, theta_cand);
    double r = squaredNorm(d);
    arena.RestoreCursor(em);
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
    arena.RestoreCursor(mark);
    return {-1, 1e30};
  }

  RowSpace d = model.AllocRowSpace(arena);
  EvaluateDirection(d, decomp, k, tau_out, theta_cand);
  double dinf = normInf(d);
  arena.RestoreCursor(mark);
  return {tau_out, dinf};
}

// Frozen-Jacobian variant of EvalThetaCandidate.
// Uses W0 for the Jacobian (Gram, sqrt, P(W)) and Wi for the value
// (the b^T Wi/k term in sigma0).  decomp contains d0 refreshed at Wi,
// d1_0 and d1_theta frozen at W0.
static std::pair<double, double> FrozenEvalThetaCandidate(
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
  char* mark = arena.SaveCursor();
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
  if (disc < 0) { arena.RestoreCursor(mark); return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  auto eval_dsq = [&](double tau) -> double {
    if (tau <= 0) return 1e30;
    char* em = arena.SaveCursor();
    RowSpace d = model.AllocRowSpace(arena);
    EvaluateDirection(d, decomp, k, tau, theta_cand);
    double r = squaredNorm(d);
    arena.RestoreCursor(em);
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
    arena.RestoreCursor(mark);
    return {-1, 1e30};
  }

  RowSpace d = model.AllocRowSpace(arena);
  EvaluateDirection(d, decomp, k, tau_out, theta_cand);
  double dinf = normInf(d);
  arena.RestoreCursor(mark);
  return {tau_out, dinf};
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
  char* mark = arena.SaveCursor();
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
  arena.RestoreCursor(mark);
}

// Overload for NewtonDecomposition (ThetaCont frozen-J).
static void RefreshD0Frozen(
    CompiledModel& model,
    Arena& arena,
    const RowSpace& b,
    const RowSpace& W0,
    const RowSpace& Wi,
    NewtonDecomposition& decomp) {
  RefreshD0Frozen(model, arena, b, W0, Wi, decomp.d0, decomp.y0);
}

// Forward declaration (defined later in this file).
static std::pair<double, double> EvalKCandidate(
    CompiledModel& model, const SolverRHS& duality_cost,
    const RowSpace& b, const RowSpace& W,
    const NewtonDecomposition& decomp, double bT_ones,
    double theta_val, double k_cand);

// =====================================================================
// Geodesic HSD: joint (tau, theta) via gap + normalization.
// =====================================================================

GeodesicResult SolveGeodesicHSD(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = W.total_rows();
  const double nu = barrierParameter(W);
  Arena& arena = model.arena();

  RowSpace ones = model.AllocRowSpace();
  setOnes(ones);
  const double bT_ones = dot(b, ones);

  auto duality_cost = MakeDualityCost(model);

  // Residuals at the fixed point (x_hat=0, lambda_hat=e, tau_hat=1, kappa_hat=1).
  // rp = A*0 + b*1 - e = b - e  (slack residual at identity)
  // But actually: rp is computed from the perturbed system.  For the standard
  // model Ax + b >= 0: at x_hat=0, slack_hat = b, lambda_hat = e.
  //   rp_i = b_i - e_i  (slack minus identity)
  //   rd = c - A'e      (cost minus dual at identity)
  // These are data-dependent; we compute them via inner products.
  //
  // Normalization: rp'*lambda + rd'*x + rg*tau = -alpha
  // Gap: b'*lambda + c'*x + d'*nu + x'Qx/tau + mu/tau = theta*R
  //
  // After eliminating theta via normalization, we get a quadratic in tau.
  // The normalization coefficients (N_tau, N_theta) and gap coefficients
  // (G_tau, G_theta) are inner products with the decomposition vectors.

  const double R = bT_ones + 1.0;
  const double alpha_norm = static_cast<double>(m) + 1.0;

  // rp = b - e (RowSpace): slack residual at the fixed point.
  // At (x=0, tau=1, theta=1): slack = e, and rp'e + rg = -(m+1).
  RowSpace rp = model.AllocRowSpace();
  rp = b;
  rp -= ones;
  // rd = A'e - c (SolverRHS): dual residual at the fixed point.
  auto rd_rhs = model.AllocSolverRHS();
  rd_rhs.SetZero();
  model.AccumulateAtranspose(ones, rd_rhs);
  rd_rhs -= cost_rhs;
  // rg = -(b'e + 1).
  const double rg = -(bT_ones + 1.0);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;
  double k = 0;

  if (verbose) {
    printf("  %3s  %10s  %10s  %10s  %10s  %10s  %10s\n",
           "it", "k", "tau", "theta", "d_inf", "d_sqr", "gap");
    printf("  %s\n", std::string(76, '-').c_str());
  }

  // Fixed k for centering test.
  const double k_fixed = 1.1;
  k = k_fixed;

  for (int iter = 0; iter < max_iterations; ++iter) {
    model.SetScaling(W);
    if (!model.AssembleAndFactor()) break;
    NewtonDecomposition decomp;
    decomp.d0 = model.AllocRowSpace();
    decomp.d1_0 = model.AllocRowSpace();
    decomp.d1_theta = model.AllocRowSpace();
    ComputeFullDecomposition(model, b, W, decomp);
    total_fac++;
    total_sol += 3;

    RowSpace sqrtW = model.AllocRowSpace();
    EuclideanJordanAlgebra::sqrt(sqrtW, W);
    RowSpace pwd1_0 = model.AllocRowSpace();
    quadraticRepresentation(pwd1_0, sqrtW, decomp.d1_0);
    RowSpace pwd1_t = model.AllocRowSpace();
    quadraticRepresentation(pwd1_t, sqrtW, decomp.d1_theta);
    RowSpace e_plus_d0 = model.AllocRowSpace();
    addScaled(e_plus_d0, ones, decomp.d0, 1.0, 1.0);
    RowSpace pwed0 = model.AllocRowSpace();
    quadraticRepresentation(pwed0, sqrtW, e_plus_d0);

    double rp_pwd10 = dot(rp, pwd1_0);
    double rp_pwd1t = dot(rp, pwd1_t);
    double rp_pwed0 = dot(rp, pwed0);

    double rd_y10 = rd_rhs.dot(decomp.y1_0);
    double rd_y1t = rd_rhs.dot(decomp.y1_theta);
    double rd_y0 = rd_rhs.dot(decomp.y0);

    double b_pwd10 = dot(b, pwd1_0);
    double b_pwd1t = dot(b, pwd1_t);
    double b_pwed0 = dot(b, pwed0);
    double dc_y10 = duality_cost.dot(decomp.y1_0);
    double dc_y1t = duality_cost.dot(decomp.y1_theta);
    double dc_y0 = duality_cost.dot(decomp.y0);

    // Joint (tau, theta) at fixed k.
    double mu = 1.0 / (k * k);

    // lambda_lifted = W*(e+d0)/k + tau*W*d1_0 + theta*W*d1_theta
    // x_lifted = y0/k + tau*y1_0 + theta*y1_theta
    // So: rp'*lambda_tau = rp'*(W*d1_0) = rp_pwd10  (no k factor)
    //     rp'*lambda_0 = rp'*(W*(e+d0))/k = rp_pwed0/k
    double N_tau = rp_pwd10 + rd_y10 + rg;
    double N_theta = rp_pwd1t + rd_y1t;
    double rhs_norm = -alpha_norm - rp_pwed0 / k - rd_y0 / k;

    double a0 = rhs_norm / N_theta;
    double a1 = -N_tau / N_theta;

    double G_tau = b_pwd10 + dc_y10;
    double G_theta = b_pwd1t + dc_y1t;
    double G_0 = b_pwed0 / k + dc_y0 / k;

    double beta = G_tau + (G_theta - R) * a1;
    double gamma_q = (G_theta - R) * a0 + G_0;
    double discr = gamma_q * gamma_q - 4.0 * beta * mu;

    double tau = 1.0, theta = 1.0;
    if (discr >= 0) {
      double sq = std::sqrt(discr);
      double t1 = (-gamma_q + sq) / (2.0 * beta);
      double t2 = (-gamma_q - sq) / (2.0 * beta);
      // Pick the root with tau > 0 and theta in [0,1].
      for (double tc : {t1, t2}) {
        if (tc <= 0) continue;
        double thc = a0 + a1 * tc;
        if (thc >= 0 && thc <= 1.5) { tau = tc; theta = thc; break; }
      }
    }

    RowSpace d = model.AllocRowSpace();
    EvaluateDirection(d, decomp, k, tau, theta);
    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double gap = mu * (nu - d_sq);

    // Evaluate normalization equation at this (k, tau, theta).
    // lambda = k * P(W^{1/2})(e + d)
    // x = y0/k + tau*y1_0 + theta*y1_theta
    // Norm: rp'*lambda + rd'*x + rg*tau should = -alpha
    {
      RowSpace ones_plus_d = model.AllocRowSpace();
      addScaled(ones_plus_d, ones, d, 1.0, 1.0);
      RowSpace lam = model.AllocRowSpace();
      quadraticRepresentation(lam, sqrtW, ones_plus_d);
      lam *= (1.0 / k);  // lambda_phys = lambda_lifted / k ... wait
      // Actually lambda = k * P(W^{1/2})(e+d), not divided by k.
      // Normalization in LIFTED space:
      //   rp'*lambda_lifted + rd'*x_lifted + rg*tau = -alpha
      // lambda_lifted = (1/k) * P(W^{1/2})(e+d)
      // x_lifted = y0/k + tau*y1_0 + theta*y1_theta
      RowSpace lam_lifted = model.AllocRowSpace();
      quadraticRepresentation(lam_lifted, sqrtW, ones_plus_d);
      lam_lifted *= (1.0 / k);
      double rp_lam = dot(rp, lam_lifted);

      auto x_rhs_v = model.AllocSolverRHS();
      x_rhs_v.SetZero();
      x_rhs_v.AddScaled(1.0 / k, decomp.y0);
      x_rhs_v.AddScaled(tau, decomp.y1_0);
      x_rhs_v.AddScaled(theta, decomp.y1_theta);
      double rd_x = rd_rhs.dot(x_rhs_v);

      double norm_val = rp_lam + rd_x + rg * tau;
      double norm_err = norm_val + alpha_norm;

      // Verify normalization directly: N_tau*tau + N_theta*theta should = rhs_norm
      double N_tau_v = rp_pwd10 + rd_y10 + rg;
      double N_theta_v = rp_pwd1t + rd_y1t;
      double rhs_norm_v = -alpha_norm - rp_pwed0 / k - rd_y0 / k;
      double norm_from_coeffs = N_tau_v * tau + N_theta_v * theta - rhs_norm_v;

      // Also evaluate gap equation: b'*lambda + c'*x + mu/tau = theta*R
      double b_lam = dot(b, lam_lifted);
      double c_x = duality_cost.dot(x_rhs_v);
      double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;
      double gap_lhs = b_lam + c_x + mu_over_tau;
      double gap_rhs = theta * R;
      double gap_err = gap_lhs - gap_rhs;

      if (verbose) {
        printf("  %3d  k=%10.4e tau=%10.4e theta=%10.4e dinf=%8.4e "
               "norm_err=%8.2e gap_err=%8.2e coeff_err=%8.2e\n",
               iter, k, tau, theta, d_inf, norm_err, gap_err, norm_from_coeffs);
      }
    }

    if (!verbose) {
      // Keep original verbose format for non-diagnostic mode.
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = iter + 1;

    if (d_inf < tolerance) {
      result.mu = mu;
      result.tau = tau;
      result.d_inf_norm = d_inf;
      result.d_sq_norm = d_sq;
      result.complementarity = gap;
      result.total_factorizations = total_fac;
      result.total_solves = total_sol;
      // De-homogenize: x_phys = x_lifted / tau.
      {
        auto x_rhs = model.AllocSolverRHS();
        x_rhs.SetZero();
        x_rhs.AddScaled(1.0 / k, decomp.y0);
        x_rhs.AddScaled(tau, decomp.y1_0);
        x_rhs.AddScaled(theta, decomp.y1_theta);
        x_rhs *= (1.0 / tau);
        int nr = model.number_of_variables();
        result.x.resize(nr);
        result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs.supernodes->GatherInto(xm); }
      }

      // lambda_phys = lambda_lifted / tau.
      RowSpace sqrtW_c = model.AllocRowSpace();
      EuclideanJordanAlgebra::sqrt(sqrtW_c, W);
      RowSpace ones_v = model.AllocRowSpace();
      setOnes(ones_v);
      RowSpace ones_v_plus_d = model.AllocRowSpace();
      addScaled(ones_v_plus_d, ones_v, d, 1.0, 1.0);
      result.lambda = model.MakeRowSpace();
      quadraticRepresentation(result.lambda, sqrtW_c, ones_v_plus_d);
      result.lambda *= (1.0 / (k * tau));

      {
        auto x_rhs = model.AllocSolverRHS();
        x_rhs.ScatterFrom(result.x.data(), result.x.size());
        result.optimality = CheckOptimality(model, x_rhs, result.lambda);
      }
      result.optimality.mu = mu;

      if (verbose) {
        printf("  Optimality: compl=%.2e, min_s=%.2e, min_lam=%.2e\n",
               result.optimality.complementarity,
               result.optimality.min_slack,
               result.optimality.min_dual);
      }
      break;
    }

    // Geodesic step.
    if (d_inf > 1e-14) {
      double step = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, step, d);
    }
  }

  // Non-convergence fallback.
  if (result.x.size() == 0) {
    result.x.assign(model.number_of_variables(), 0.0);
    result.mu = (k > 0) ? 1.0 / (k * k) : 1.0;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;
  }

  return result;
}

GeodesicResult SolveGeodesicThetaContinuation(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = W.total_rows();
  const double nu = barrierParameter(W);
  Arena& arena = model.arena();

  RowSpace ones_bTe = model.AllocRowSpace();
  setOnes(ones_bTe);
  const double bT_ones = dot(b, ones_bTe);

  // Duality cost: cost_rhs with +d at equality dual positions (instead of -d).
  auto duality_cost = MakeDualityCost(model);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  double k = 0, tau = 0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %8s  %10s  %12s  %12s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s\n",
           "out", "theta", "tau", "kappa", "k", "d_inf", "d_sqr",
           "gap", "dual", "primal", "mu/tau", "eq_err");
    printf("  %s\n", std::string(149, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    char* outer_mark = arena.SaveCursor();
    // Decompose at current W.
    NewtonDecomposition decomp;
    decomp.d0 = model.AllocRowSpace();
    decomp.d1_0 = model.AllocRowSpace();
    decomp.d1_theta = model.AllocRowSpace();
    { CONEX_TIMER(stats, factor_us);
      ComputeFullDecomposition(model, b, W, decomp);
    }
    if (stats) { stats->factor_count++; stats->solve_count += 3; }
    total_fac++;
    total_sol += 3;

    // Binary search for the smallest theta with ||d||_inf <= beta,
    // using the hard-constraint (V(tau)=0) tau selection.
    constexpr double beta_target = 1.0;
    double theta_prev = theta;
    {
      double theta_lo = 0.0;     // allow theta to reach 0
      double theta_hi = theta;   // current (centered) theta
      for (int bisect = 0; bisect < 30; ++bisect) {
        // Use arithmetic mean since theta_lo can be 0 (geometric mean→0).
        double theta_mid = 0.5 * (theta_lo + theta_hi);
        auto [tau_try, d_inf_try] = EvalThetaCandidate(
            model, arena, duality_cost, b, W, decomp, bT_ones, theta_mid);
        if (tau_try > 0 && d_inf_try <= beta_target) {
          theta_hi = theta_mid;  // can go lower
        } else {
          theta_lo = theta_mid;  // too aggressive
        }
      }
      theta = theta_hi;
    }
    k = 1.0 / std::sqrt(theta);
    // Evaluate tau at the chosen theta via hard constraint (V(tau)=0).
    auto [tau_sel, d_inf_sel] = EvalThetaCandidate(
        model, arena, duality_cost, b, W, decomp, bT_ones, theta);
    if (tau_sel <= 0) {
      // No positive root — abort this outer iteration.
      result.iterations = outer + 1;
      break;
    }
    tau = tau_sel;

    // Evaluate direction for the first step.
    RowSpace d_step = model.AllocRowSpace();
    EvaluateDirection(d_step, decomp, k, tau, theta);
    double d_inf = normInf(d_step);
    double d_sq = squaredNorm(d_step);
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    // Take geodesic step.
    RowSpace W0 = W;  // save frozen Jacobian point
    { CONEX_TIMER(stats, cone_us);
      if (d_inf > 1e-14) {
        double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
        geodesicUpdate(W, alpha, d_step);
      }
    }

    // Frozen-Jacobian steps: refresh d0 (1 solve), redo theta binary
    // search with frozen d1_0, d1_theta, then step.
    //
    // Debug: set refactor_inner=true to refactor at Wi and use the
    // standard (non-frozen) path.  This should produce the same
    // trajectory as baseline ThetaCont.  Any difference is a bug.
    constexpr bool refactor_inner = false;

    RowSpace sqrtW0_f = model.AllocRowSpace();
    EuclideanJordanAlgebra::sqrt(sqrtW0_f, W0);
    auto dc_f = ComputeDualityCoeffs(model, arena, duality_cost, b, W0, decomp);
    double beta_f = dc_f.sigma1 + dc_f.gamma1 + dc_f.q11;

    for (int inner = 0; inner < max_centering_steps; ++inner) {
      if (refactor_inner) {
        // Full refactor: makes inner iteration identical to outer.
        W0 = W;
        EuclideanJordanAlgebra::sqrt(sqrtW0_f, W0);
        dc_f = ComputeDualityCoeffs(model, arena, duality_cost, b, W0, decomp);
        beta_f = dc_f.sigma1 + dc_f.gamma1 + dc_f.q11;
        ComputeFullDecomposition(model, b, W, decomp);
        total_fac++;
        total_sol += 3;
      } else {
        RefreshD0Frozen(model, arena, b, W0, W, decomp);
        total_sol += 1;
      }

      // Binary search for smallest theta.
      double theta_lo_f, theta_hi_f;
      if (refactor_inner) {
        theta_lo_f = 0.0;  // standard: allow theta to reach 0
      } else {
        theta_lo_f = theta * 0.1;  // frozen-J: limit reduction
      }
      theta_hi_f = theta;

      for (int bisect = 0; bisect < 30; ++bisect) {
        double theta_mid = 0.5 * (theta_lo_f + theta_hi_f);
        std::pair<double, double> result_try;
        if (refactor_inner) {
          result_try = EvalThetaCandidate(
              model, arena, duality_cost, b, W, decomp, bT_ones, theta_mid);
        } else {
          result_try = FrozenEvalThetaCandidate(
              model, arena, duality_cost, b, W, sqrtW0_f, decomp,
              bT_ones, beta_f, theta_mid);
        }
        auto [tau_try, d_inf_try] = result_try;
        if (tau_try > 0 && d_inf_try <= beta_target) {
          theta_hi_f = theta_mid;
        } else {
          theta_lo_f = theta_mid;
        }
      }
      double theta_f = theta_hi_f;
      double k_f = 1.0 / std::sqrt(theta_f);
      std::pair<double, double> result_sel;
      if (refactor_inner) {
        result_sel = EvalThetaCandidate(
            model, arena, duality_cost, b, W, decomp, bT_ones, theta_f);
      } else {
        result_sel = FrozenEvalThetaCandidate(
            model, arena, duality_cost, b, W, sqrtW0_f, decomp,
            bT_ones, beta_f, theta_f);
      }
      auto [tau_f, d_inf_f] = result_sel;
      if (tau_f <= 0) break;

      RowSpace d_f = model.AllocRowSpace();
      EvaluateDirection(d_f, decomp, k_f, tau_f, theta_f);
      double d_inf_fv = normInf(d_f);
      if (verbose) {
        double mu_f = 1.0 / (k_f * k_f);
        double gap_f = mu_f * (nu - squaredNorm(d_f));

        // Check V(tau)=0: b^T lambda + c^T x + xQx/tau + mu/tau = theta*R.
        // Frozen-J lambda: Wi/k + P(sqrtW0)(d)/k.
        RowSpace lam_f = model.AllocRowSpace();
        lam_f += W;  // Wi term (per-segment copy)
        lam_f *= (1.0 / k_f);
        RowSpace Pd = model.AllocRowSpace();
        quadraticRepresentation(Pd, sqrtW0_f, d_f);
        Pd *= (1.0 / k_f);
        lam_f += Pd;  // + P(sqrtW0)(d)/k

        double bTl_f = dot(b, lam_f);
        auto x_rhs_f = model.AllocSolverRHS();
        x_rhs_f.SetZero();
        x_rhs_f.AddScaled(1.0 / k_f, decomp.y0);
        x_rhs_f.AddScaled(tau_f, decomp.y1_0);
        x_rhs_f.AddScaled(theta_f, decomp.y1_theta);
        double cTx_f = duality_cost.dot(x_rhs_f);
        auto qx_f = model.AllocSolverRHS(); qx_f.SetZero();
        model.AccumulateQx(x_rhs_f, qx_f);
        double xQx_f = qx_f.dot(x_rhs_f);
        double mu_tau_f = (tau_f > 1e-30) ? mu_f / tau_f : 0;
        double xQx_tau_f = (tau_f > 1e-30) ? xQx_f / tau_f : 0;
        double R_f = theta_f * (bT_ones + 1.0);
        double eq_err_f = std::abs(bTl_f + cTx_f + xQx_tau_f + mu_tau_f - R_f);

        printf("  %3d.%d  %10.2e  %10.2e  %12s  %12.4e  %12.4e  %12s  %12.4e"
               "  %12s  %12s  %12s  eq=%.2e  (frozen-J)\n",
               outer, inner + 1, theta_f, tau_f, "", k_f, d_inf_fv, "",
               gap_f, "", "", "", eq_err_f);
      }

      if (d_inf_fv > 1e-14) {
        double alpha_f = std::min(1.0, 2.0 / (d_inf_fv * d_inf_fv));
        geodesicUpdate(W, alpha_f, d_f);
      }
      theta = theta_f;
      k = k_f;
      tau = tau_f;
    }
    // Report final state. Use saved values from last step (outer or frozen-J).
    mu = 1.0 / (k * k);
    // d_inf, d_sq, gap are from the outer step; update if frozen-J ran.
    if (max_centering_steps > 0) {
      RowSpace d_final = model.AllocRowSpace();
      EvaluateDirection(d_final, decomp, k, tau, theta);
      d_inf = normInf(d_final);
      d_sq = squaredNorm(d_final);
      gap = mu * (nu - d_sq);
    }
    if (verbose) {
      // x = y0/k + tau*y1_0 + theta*y1_theta
      auto x_rhs_v = model.AllocSolverRHS();
      x_rhs_v.SetZero();
      x_rhs_v.AddScaled(1.0 / k, decomp.y0);
      x_rhs_v.AddScaled(tau, decomp.y1_0);
      x_rhs_v.AddScaled(theta, decomp.y1_theta);
      double cT_x = cost_rhs.dot(x_rhs_v);
      double dT_nu = duality_cost.dot(x_rhs_v) - cT_x;
      auto qx_rhs = model.AllocSolverRHS(); qx_rhs.SetZero();
      model.AccumulateQx(x_rhs_v, qx_rhs);
      double xQx = qx_rhs.dot(x_rhs_v);
      double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;

      // Physical objectives: de-homogenize by tau.
      double half_xQx_phys = (tau > 1e-30) ? 0.5 * xQx / (tau * tau) : 0.0;
      double primal_phys = (tau > 1e-30) ? cT_x / tau + half_xQx_phys : 0.0;

      // Dual objective: b'lambda where lambda is recovered from the last
      // consistent (W, d) pair. Since W has been updated past the last
      // direction, we report the primal objective only.
      // dual = -(primal + gap/tau) by weak duality.
      double dual_phys = primal_phys + gap / std::max(std::abs(tau), 1e-30);

      printf("  %3d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %3d\n",
             outer, theta, tau, mu_over_tau, k, d_inf, d_sq, gap,
             dual_phys, primal_phys, mu_over_tau,
             max_centering_steps);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.tau = tau;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Termination: mu below tolerance with d_inf small.  Same criterion
    // for both algorithms so iteration counts are directly comparable.
    if (mu < tolerance && d_inf <= 1.001) {
      if (verbose) printf("  TERMINATED: mu = %.2e < tolerance, d_inf = %.2e\n",
                          mu, d_inf);
      break;
    }
    if (outer + 1 == max_outer_iterations) {
      if (verbose) printf("  TERMINATED: reached max_outer_iterations = %d\n",
                          max_outer_iterations);
    }
    arena.RestoreCursor(outer_mark);
  }

  // Recover primal x.  De-homogenize: x_phys = x_lifted / tau.
  if (k > 0 && tau > 0) {
    NewtonDecomposition decomp;
    decomp.d0 = model.AllocRowSpace();
    decomp.d1_0 = model.AllocRowSpace();
    decomp.d1_theta = model.AllocRowSpace();
    ComputeFullDecomposition(model, b, W, decomp);
    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.SetZero();
      x_rhs.AddScaled(1.0 / k, decomp.y0);
      x_rhs.AddScaled(tau, decomp.y1_0);
      x_rhs.AddScaled(theta, decomp.y1_theta);
      x_rhs *= (1.0 / tau);
      int nr = model.number_of_variables();
      result.x.resize(nr);
      result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs.supernodes->GatherInto(xm); }
    }

    // Compute lambda at the CURRENT (k, tau, theta) — consistent with x.
    // lambda_lifted = (1/k) P(W^{1/2})(e + d(k,tau,theta));
    // lambda_phys   = lambda_lifted / tau.
    RowSpace d_cur = model.AllocRowSpace();
    EvaluateDirection(d_cur, decomp, k, tau, theta);
    RowSpace sqrtW_r = model.AllocRowSpace();
    EuclideanJordanAlgebra::sqrt(sqrtW_r, W);
    RowSpace ones_r = model.AllocRowSpace();
    setOnes(ones_r);
    RowSpace ones_plus_dcur = model.AllocRowSpace();
    addScaled(ones_plus_dcur, ones_r, d_cur, 1.0, 1.0);
    result.lambda = model.MakeRowSpace();
    quadraticRepresentation(result.lambda, sqrtW_r, ones_plus_dcur);
    result.lambda *= (1.0 / (k * tau));

    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.ScatterFrom(result.x.data(), result.x.size());
      result.optimality = CheckOptimality(model, x_rhs, result.lambda);
      result.optimality.mu = result.mu;
    }

    if (verbose) {
      printf("  Optimality: compl=%.2e, "
             "min_s=%.2e, min_lam=%.2e\n",
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  return result;
}

double GeodesicLineSearch(
    CompiledModel& model,
    const RowSpace& W) {
  Arena& arena = model.arena();
  const RowSpace b = model.GetAffineTerm();
  RowSpace d0 = model.AllocRowSpace();
  RowSpace d1 = model.AllocRowSpace();
  ComputeDecomposition(model, arena, model.cost_rhs(), b, W, d0, d1);

  return lineSearchK(d0, d1);
}

// Solve V(τ) = 0 for τ at fixed (theta, k); returns (tau, d_inf).
// Same algebra as EvalThetaCandidate but with k passed in independently
// of theta (so we can hold k = 1/sqrt(theta_mid) during a binary search).
static std::pair<double, double> EvalKCandidate(
    CompiledModel& model,
    const SolverRHS& duality_cost,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double theta_val,
    double k_cand) {
  if (k_cand <= 0) return {-1, 1e30};
  Arena& arena = model.arena();
  double mu = 1.0 / (k_cand * k_cand);

  auto dc = ComputeDualityCoeffs(model, arena, duality_cost, b, W, decomp);
  double beta_coeff = dc.sigma1 + dc.gamma1 + dc.q11;

  RowSpace sqrtW = model.AllocRowSpace();
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace ones_v = model.AllocRowSpace();
  setOnes(ones_v);
  RowSpace e_plus_d0 = model.AllocRowSpace();
  addScaled(e_plus_d0, ones_v, decomp.d0, 1.0, 1.0);
  RowSpace arg = model.AllocRowSpace();
  addScaled(arg, e_plus_d0, decomp.d1_theta, 1.0, k_cand * theta_val);
  RowSpace Parg = model.AllocRowSpace();
  quadraticRepresentation(Parg, sqrtW, arg);
  double sigma0 = dot(b, Parg) / k_cand;

  double cT_y0 = duality_cost.dot(decomp.y0);
  double cT_yt = duality_cost.dot(decomp.y1_theta);
  double gamma0 = cT_y0 / k_cand + theta_val * cT_yt;

  // Quadratic cost: f = y0/k + theta*y1_theta.
  auto f_rhs = model.AllocSolverRHS();
  f_rhs.SetZero();
  f_rhs.AddScaled(1.0 / k_cand, decomp.y0);
  f_rhs.AddScaled(theta_val, decomp.y1_theta);
  auto qf = model.AllocSolverRHS();
  qf.SetZero();
  model.AccumulateQx(f_rhs, qf);
  double q_ff = qf.dot(f_rhs);
  double q_f1 = qf.dot(decomp.y1_0);

  double alpha_coeff = sigma0 + gamma0 + 2.0 * q_f1;
  double R = theta_val * (bT_ones + 1.0);
  double mu_eff = mu + q_ff;

  double B = alpha_coeff - R;
  double disc = B * B - 4.0 * beta_coeff * mu_eff;
  if (disc < 0) return {-1, 1e30};

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta_coeff);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta_coeff);

  auto eval_dinf = [&](double tau) -> std::pair<double, double> {
    if (tau <= 0) return {-1, 1e30};
    RowSpace d = model.AllocRowSpace();
    EvaluateDirection(d, decomp, k_cand, tau, theta_val);
    return {tau, normInf(d)};
  };

  auto [t1, dinf1] = eval_dinf(tau1);
  auto [t2, dinf2] = eval_dinf(tau2);

  if (t1 > 0 && (t2 <= 0 || dinf1 <= dinf2)) return {t1, dinf1};
  if (t2 > 0) return {t2, dinf2};
  return {-1, 1e30};
}

GeodesicResult SolveGeodesicPhaseOne(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations,
    int /*max_centering_steps*/,
    double tolerance,
    bool verbose,
    bool phase1_only) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = W.total_rows();
  const double nu = barrierParameter(W);
  Arena& arena = model.arena();

  RowSpace ones_bTe = model.AllocRowSpace();
  setOnes(ones_bTe);
  const double bT_ones = dot(b, ones_bTe);

  auto duality_cost = MakeDualityCost(model);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  double k = 1.0, tau = 1.0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %12s  %10s  %12s  %12s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %4s\n",
           "out", "theta", "tau", "kappa", "mu", "d_inf", "d_sqr",
           "gap", "dual", "primal", "eq_err", "ph");
    printf("  %s\n", std::string(149, '-').c_str());
  }

  bool theta_zero = false;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    NewtonDecomposition decomp;
    decomp.d0 = model.AllocRowSpace();
    decomp.d1_0 = model.AllocRowSpace();
    decomp.d1_theta = model.AllocRowSpace();
    ComputeFullDecomposition(model, b, W, decomp);
    total_fac++;
    total_sol += 3;

    int phase = theta_zero ? 2 : 1;

    if (!theta_zero) {
      // Phase 1: try theta=0 first via lineSearchK with bound 1.1.
      // d at theta=0 is d0 + k * tau * d1_0; we hold tau at its current
      // value (last selected by V(tau)=0).  If a positive k exists,
      // commit to phase 2.
      RowSpace tau_d1_0 = decomp.d1_0; tau_d1_0 *= tau;
      double k_zero = lineSearchK(decomp.d0, tau_d1_0, 1.1);
      if (k_zero > 0) {
        theta = 0.0;
        theta_zero = true;
        k = k_zero;
        phase = 2;
        if (phase1_only) {
          result.mu = 1.0 / (k * k);
          result.tau = tau;
          result.iterations = outer + 1;
          result.total_factorizations = total_fac;
          result.total_solves = total_sol;
          if (verbose) {
            RowSpace d_p1 = model.AllocRowSpace();
            EvaluateDirection(d_p1, decomp, k, tau, 0.0);
            double dinf_p1 = normInf(d_p1);

            RowSpace b_sc = model.GetAffineTerm(); b_sc *= tau;
            auto c_sc = model.AllocSolverRHS(); c_sc = cost_rhs; c_sc *= tau;
            RowSpace r_chk = model.AllocRowSpace();
            setOnes(r_chk); r_chk *= (1.0 / k);
            RowSpace d_hyb = model.AllocRowSpace();
            RowSpace delta_hyb = model.AllocRowSpace();
            ComputeHybridDirection(model, b_sc, W, r_chk,
                                    d_hyb, delta_hyb, tau);
            double dinf_hyb = normInf(d_hyb);

            printf("  PHASE1 DONE: theta=0 at iter %d"
                   " (k=%.2e, tau=%.2e)\n"
                   "    d_inf phase1=%.6e  hybrid=%.6e  diff=%.2e\n",
                   outer, k, tau, dinf_p1, dinf_hyb,
                   std::abs(dinf_p1 - dinf_hyb));
          }
          break;
        }
      } else {
        // Binary search for smallest theta admitting V(τ)=0 with d_inf<=1,
        // with k tied to theta as k = 1/sqrt(theta).  Arithmetic mean
        // (matches phase_one_debug branch).
        double theta_lo = 0.0;
        double theta_hi = theta;
        for (int bisect = 0; bisect < 30; ++bisect) {
          double theta_mid = 0.5 * (theta_lo + theta_hi);
          double k_mid = 1.0 / std::sqrt(theta_mid);
          auto [tau_try, d_inf_try] = EvalKCandidate(
              model, duality_cost, b, W, decomp, bT_ones, theta_mid, k_mid);
          if (tau_try > 0 && d_inf_try <= 1.0) {
            theta_hi = theta_mid;
          } else {
            theta_lo = theta_mid;
          }
        }
        theta = theta_hi;
        k = 1.0 / std::sqrt(theta);
        auto [tau_sel, d_inf_sel] = EvalKCandidate(
            model, duality_cost, b, W, decomp, bT_ones, theta, k);
        if (tau_sel <= 0) {
          if (verbose) printf("  TERMINATED: phase 1 V(τ)=0 has no positive root\n");
          break;
        }
        tau = tau_sel;
      }
    } else {
      // Phase 2 (theta=0): tau frozen, find largest k with d_inf <= 1.
      RowSpace tau_d1_0 = decomp.d1_0; tau_d1_0 *= tau;
      double k_new = lineSearchK(decomp.d0, tau_d1_0);
      if (k_new > 0) k = k_new;
    }

    // Evaluate quantities at the consistent (W, decomp, tau, k, theta)
    // BEFORE the geodesic step, so reported values are coherent.
    RowSpace d_step = model.AllocRowSpace();
    EvaluateDirection(d_step, decomp, k, tau, theta);
    double d_inf = normInf(d_step);
    double d_sq = squaredNorm(d_step);
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    RowSpace sqrtW_step = model.AllocRowSpace();
    EuclideanJordanAlgebra::sqrt(sqrtW_step, W);
    RowSpace ones_step = model.AllocRowSpace();
    setOnes(ones_step);
    RowSpace ones_step_plus_d = model.AllocRowSpace();
    addScaled(ones_step_plus_d, ones_step, d_step, 1.0, 1.0);
    RowSpace lam_step = model.AllocRowSpace();
    quadraticRepresentation(lam_step, sqrtW_step, ones_step_plus_d);
    lam_step *= (1.0 / k);
    double bT_lambda = dot(b, lam_step);
    auto x_rhs_step = model.AllocSolverRHS();
    x_rhs_step.SetZero();
    x_rhs_step.AddScaled(1.0 / k, decomp.y0);
    x_rhs_step.AddScaled(tau, decomp.y1_0);
    x_rhs_step.AddScaled(theta, decomp.y1_theta);
    double cT_x = cost_rhs.dot(x_rhs_step);
    double dT_nu = duality_cost.dot(x_rhs_step) - cT_x;
    auto qx_rhs = model.AllocSolverRHS();
    qx_rhs.SetZero();
    model.AccumulateQx(x_rhs_step, qx_rhs);
    double xQx_over_tau = (tau > 1e-30) ? qx_rhs.dot(x_rhs_step) / tau : 0.0;
    double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;
    double eq_err = std::abs(bT_lambda + cT_x + dT_nu + xQx_over_tau
                             + mu_over_tau - theta * (bT_ones + 1.0));

    if (d_inf > 1e-14) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d_step);
    }

    if (verbose) {
      double half_xQx_phys = (tau > 1e-30)
          ? 0.5 * qx_rhs.dot(x_rhs_step) / (tau * tau) : 0.0;
      double primal_phys = (tau > 1e-30) ? cT_x / tau + half_xQx_phys : 0.0;
      double dual_phys = (tau > 1e-30)
          ? -((bT_lambda + dT_nu) / tau + half_xQx_phys) : 0.0;
      printf("  %3d  %12.4e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.2e  %4d\n",
             outer, theta, tau, mu_over_tau, mu, d_inf, d_sq, gap,
             dual_phys, primal_phys, eq_err, phase);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.tau = tau;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Termination: mu below tolerance with d_inf small.
    // When phase1_only, keep going until theta=0 is reached.
    if (!phase1_only && mu < tolerance && d_inf <= 1.001) {
      if (verbose) printf("  TERMINATED: mu = %.2e < tolerance, d_inf = %.2e\n",
                          mu, d_inf);
      break;
    }
    // Phase 2 divergence guard.
    if (theta_zero && d_inf > 10.0) {
      if (verbose) printf("  TERMINATED: phase 2 diverging (d_inf = %.2e)\n",
                          d_inf);
      break;
    }
  }

  // Recover primal x.  De-homogenize: x_phys = x_lifted / tau.
  if (k > 0 && tau > 0) {
    NewtonDecomposition decomp;
    decomp.d0 = model.AllocRowSpace();
    decomp.d1_0 = model.AllocRowSpace();
    decomp.d1_theta = model.AllocRowSpace();
    ComputeFullDecomposition(model, b, W, decomp);
    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.SetZero();
      x_rhs.AddScaled(1.0 / k, decomp.y0);
      x_rhs.AddScaled(tau, decomp.y1_0);
      x_rhs.AddScaled(theta, decomp.y1_theta);
      x_rhs *= (1.0 / tau);
      int nr = model.number_of_variables();
      result.x.resize(nr);
      result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs.supernodes->GatherInto(xm); }
    }
    RowSpace d_cur = model.AllocRowSpace();
    EvaluateDirection(d_cur, decomp, k, tau, theta);
    RowSpace sqrtW_p = model.AllocRowSpace();
    EuclideanJordanAlgebra::sqrt(sqrtW_p, W);
    RowSpace ones_p = model.AllocRowSpace();
    setOnes(ones_p);
    RowSpace ones_plus_dcur = model.AllocRowSpace();
    addScaled(ones_plus_dcur, ones_p, d_cur, 1.0, 1.0);
    result.lambda = model.MakeRowSpace();
    quadraticRepresentation(result.lambda, sqrtW_p, ones_plus_dcur);
    result.lambda *= (1.0 / (k * tau));
    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.ScatterFrom(result.x.data(), result.x.size());
      result.optimality = CheckOptimality(model, x_rhs, result.lambda);
    }
    result.optimality.mu = result.mu;
  }
  return result;
}


GeodesicResult SolveGeodesicLP(
    CompiledModel& model,
    RowSpace& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose,
    bool mehrotra_correction,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  double k = 0.0;
  const int m = W.total_rows();
  const double nu = barrierParameter(W);
  Arena& arena = model.arena();
  constexpr double theta = 0.0;
  RowSpace ones_b = model.AllocRowSpace(arena);
  setOnes(ones_b);

  RowSpace b = model.AllocRowSpace(arena);
  auto cost_rhs_blend = model.AllocSolverRHS();
  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  if (verbose) {
    printf("  %3s  %12s  %12s  %12s  %12s  %12s\n",
           "fac", "k", "k_new", "d_inf", "d_sqr", "gap");
    printf("  %s\n", std::string(72, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ) {
    char* iter_mark = arena.SaveCursor();

    addScaled(b, ones_b, model.GetAffineTerm(), theta, 1.0 - theta);
    cost_rhs_blend.SetZero();
    model.AccumulateAtranspose(ones_b, cost_rhs_blend);
    cost_rhs_blend *= theta;
    cost_rhs_blend.AddScaled(1.0 - theta, cost_rhs);

    // Factor + decompose.  d0, d1 are arena-backed (from ComputeDecomposition).
    RowSpace d0 = model.AllocRowSpace(arena);
    RowSpace d1 = model.AllocRowSpace(arena);
    SolverRHS y0{}, y1{};
    { CONEX_TIMER(stats, factor_us);
      ComputeDecomposition(model, arena, cost_rhs_blend, b, W, d0, d1, &y0, &y1);
    }
    if (stats) { stats->factor_count++; stats->solve_count++; }
    total_fac += 1;
    total_sol += 2;

    // Line search for k (fresh factorization).
    double k_new;
    { CONEX_TIMER(stats, cone_us);
      k_new = lineSearchK(d0, d1);
    }
    double k_prev = k;
    if (k_new > k) {
      k = k_new;
    } else if (outer == 0 || k_new == 0) {
      double d0d1 = dot(d0, d1);
      double d1sq = squaredNorm(d1);
      if (d1sq > 1e-30) {
        double k_min_norm = std::max(0.0, -d0d1 / d1sq);
        if (k_min_norm > k) k = k_min_norm;
      }
    }

    // Evaluate direction at current k.
    RowSpace d = model.AllocRowSpace(arena);
    addScaled(d, d0, d1, 1.0, k);
    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double mu = 1.0 / (k * k);
    double s_dot_x = mu * (nu - d_sq);

    if (verbose) {
      printf("  %3d  %10.4e  %10.4e  %10.4e  %10.4e  %10.4e"
             "  d0=%.2e d1=%.2e\n",
             outer, k_prev, k, d_inf, d_sq, s_dot_x,
             normInf(d0), normInf(d1));
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, s_dot_x});
    result.iterations = outer + 1;

    bool converged = (s_dot_x < tolerance && d_inf < 1.01);
    bool last_iter = (outer + 1 >= max_outer_iterations);

    if (converged || last_iter) {
      result.mu = mu;
      result.d_inf_norm = d_inf;
      result.d_sq_norm = d_sq;
      result.complementarity = s_dot_x;
      result.total_factorizations = total_fac;
      result.total_solves = total_sol;
      {
        auto x_rhs_conv = model.AllocSolverRHS();
        x_rhs_conv.SetZero();
        x_rhs_conv.AddScaled(1.0 / k, y0);
        x_rhs_conv += y1;
        int nr = model.number_of_variables();
        result.x.resize(nr);
        result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs_conv.supernodes->GatherInto(xm); }
      }
      // Lambda recovery (heap — outlives arena).
      RowSpace sqrtW = model.AllocRowSpace(arena);
      EuclideanJordanAlgebra::sqrt(sqrtW, W);
      RowSpace ones_d = model.AllocRowSpace(arena);
      setOnes(ones_d);
      ones_d += d;  // e + d
      RowSpace lambda = model.MakeRowSpace();
      quadraticRepresentation(lambda, sqrtW, ones_d);
      lambda *= (1.0 / k);
      result.lambda = std::move(lambda);
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.ScatterFrom(result.x.data(), result.x.size());
      result.optimality = CheckOptimality(model, x_rhs, result.lambda);
      result.optimality.mu = result.mu;
      if (verbose) {
        printf("  Optimality: compl=%.2e, min_s=%.2e, min_lam=%.2e\n",
               result.optimality.complementarity,
               result.optimality.min_slack,
               result.optimality.min_dual);
      }
      arena.RestoreCursor(iter_mark);
      break;
    }

    // Save W₀ (frozen Jacobian point) before stepping.
    RowSpace W0 = W;  // heap deep copy (W is heap-backed)

    // Geodesic step.
    { CONEX_TIMER(stats, cone_us);
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d);
    }

    // Frozen-Jacobian iterations.
    if (max_centering_steps > 0) {
      RowSpace d0_f = model.AllocRowSpace(arena);
      RowSpace d1_f = d1;  // frozen (arena shallow copy — lives until iter_mark)
      SolverRHS y0_f = model.MakeSolverRHS();  // heap-backed
      RefreshD0Frozen(model, arena, model.GetAffineTerm(), W0, W, d0_f, y0_f);
      total_sol += 1;

      for (int inner = 0; inner < max_centering_steps; ++inner) {
        char* inner_mark = arena.SaveCursor();

        double k_new_f = lineSearchK(d0_f, d1_f);
        if (k_new_f > k) k = k_new_f;

        RowSpace d_f = model.AllocRowSpace(arena);
        addScaled(d_f, d0_f, d1_f, 1.0, k);
        double d_inf_f = normInf(d_f);
        double d_sq_f = squaredNorm(d_f);
        double mu_f = 1.0 / (k * k);
        double s_dot_x_f = mu_f * (nu - d_sq_f);

        if (verbose) {
          printf("  %3d.%d  %10s  %10.4e  %10.4e  %10.4e  %10.4e"
                 "  d0=%.2e d1=%.2e  (frozen-J)\n",
                 outer, inner + 1, "", k, d_inf_f, d_sq_f, s_dot_x_f,
                 normInf(d0_f), normInf(d1_f));
        }

        double alpha_f = std::min(1.0, 2.0 / (d_inf_f * d_inf_f));
        geodesicUpdate(W, alpha_f, d_f);
        arena.RestoreCursor(inner_mark);

        if (inner + 1 < max_centering_steps) {
          RefreshD0Frozen(model, arena, model.GetAffineTerm(), W0, W, d0_f, y0_f);
          total_sol += 1;
        }
      }
    }

    arena.RestoreCursor(iter_mark);
    ++outer;
  }

  // If the loop exited without converging, populate result with last known
  // state so that Solver::Solve doesn't crash on an empty x vector.
  if (result.x.size() == 0) {
    result.x.assign(model.number_of_variables(), 0.0);
    result.mu = (k > 0) ? 1.0 / (k * k) : 1.0;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;
  }

  return result;
}

GeodesicResult SolveGeodesicBarrierLP(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations,
    int max_frozen_steps,
    double tolerance,
    bool verbose,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = z.total_rows();
  const double nu = barrierParameter(z);
  double k = 0.0;

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  // Arena for temporary RowSpaces (zero heap allocation in the loop).
  Arena& arena = model.arena();

  if (verbose) {
    printf("  %3s  %12s  %12s  %12s  %12s  %12s\n",
           "fac", "k", "k_new", "d_inf", "d_sqr", "gap");
    printf("  %s\n", std::string(72, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    char* iter_mark = arena.SaveCursor();

    // 0. Sync z to constraint workspaces and factor Gram = A^T H(z) A.
    bool factor_ok;
    { CONEX_TIMER(stats, factor_us);
      model.SetScaling(z);
      factor_ok = model.AssembleAndFactor();
    }
    if (!factor_ok) { arena.RestoreCursor(iter_mark); break; }
    if (stats) stats->factor_count++;

    // 1. Centering RHS: A^T(-2∇F(z)).
    RowSpace grad = model.AllocRowSpace(arena);
    computeGradient(z, grad);
    grad *= -2.0;
    auto rhs0 = model.AllocSolverRHS();
    rhs0.SetZero();
    model.AccumulateAtranspose(grad, rhs0);

    // 2. Optimality RHS: -(c + A^T H(z) b).
    RowSpace hb = model.AllocRowSpace(arena);
    hessianProduct(z, b, hb);
    auto rhs1 = model.AllocSolverRHS();
    rhs1 = cost_rhs;
    model.AccumulateAtranspose(hb, rhs1);
    rhs1 *= -1;
    // Inject equality RHS if present.
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      rhs1 += d_rhs;
    }

    // 3. Solve both RHS.
    auto y = model.AllocSolverRHS(2);
    y.SetColumn(0, rhs0);
    y.SetColumn(1, rhs1);
    { CONEX_TIMER(stats, solve_us);
      model.SolveSolverRHS(y);
    }
    if (stats) stats->solve_count += 2;

    total_fac += 1;
    total_sol += 2;

    // 4. Recover y0, y1.
    SolverRHS y0 = model.MakeSolverRHS();
    SolverRHS y1 = model.MakeSolverRHS();
    {
      int nb = y.num_blocks();
      for (int bk = 0; bk < nb; ++bk) {
        y0.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(0);
        y1.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(1);
      }
    }

    // 5. Compute targets: target0 = Ay0, target1 = Ay1 + b.
    auto row = model.AllocRowSpace(arena, 2);
    model.MultiplyA(y, row);
    RowSpace target0 = model.AllocRowSpace(arena);
    RowSpace target1 = model.AllocRowSpace(arena);
    target0.col() = row.col(0);
    target1.col() = row.col(1);
    target1 += b;

    // 6. Line search for k.
    double k_new;
    { CONEX_TIMER(stats, cone_us);
      k_new = lineSearchTarget(z, target0, target1);
    }
    double k_prev = k;

    // Minimum-norm fallback (same logic as SolveGeodesicLP).
    if (k_new > k) {
      k = k_new;
    } else if (outer == 0 || k_new == 0) {
      // ||target0 + k*target1 - z_primal||²_H = a + 2fk + pk²
      // Minimizer: k* = -f/p.  Compute via polarization.
      double a_coeff = hessianNormSquared(z, target0);
      RowSpace t01 = model.AllocRowSpace(arena);
      addScaled(t01, target0, target1, 1.0, 1.0);
      double ap = hessianNormSquared(z, t01);
      double p_coeff = hessianNormSquared(z, target1);
      double f_coeff = 0.5 * (ap - a_coeff - p_coeff);
      if (p_coeff > 1e-30) {
        double k_min_norm = std::max(0.0, -f_coeff / p_coeff);
        if (k_min_norm > k) k = k_min_norm;
      }
    }

    // 7. Assemble target_k, compute step size and norms.
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, target0, target1, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double alpha = stepSize(z, target_k);

    // For reporting: d_inf from alpha.  alpha = min(1, 2/d_inf²)
    // => d_inf = sqrt(2/alpha) if alpha < 1, else d_inf ≈ 0.
    double d_inf = (alpha < 1.0) ? std::sqrt(2.0 / alpha) : 0.0;

    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    if (verbose) {
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e\n",
             outer, k_prev, k, d_inf, d_sq, gap);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;

    bool converged = (gap < tolerance && d_inf < 1.01);
    bool last_iter = (outer + 1 == max_outer_iterations);

    if (converged || last_iter) {
      result.mu = mu;
      result.d_inf_norm = d_inf;
      result.d_sq_norm = d_sq;
      result.complementarity = gap;
      result.total_factorizations = total_fac;
      result.total_solves = total_sol;
      {
        auto x_rhs_conv = model.AllocSolverRHS();
        x_rhs_conv.SetZero();
        x_rhs_conv.AddScaled(1.0 / k, y0);
        x_rhs_conv += y1;
        int nr = model.number_of_variables();
        result.x.resize(nr);
        result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs_conv.supernodes->GatherInto(xm); }
      }

      // Lambda recovery: λ = (1/k)(-2∇F(z) - H(z)·target_k).
      // Use heap for lambda since it outlives the arena.
      RowSpace lambda = model.MakeRowSpace();
      computeGradient(z, lambda);
      lambda *= -2.0;
      RowSpace h_target = model.AllocRowSpace(arena);
      hessianProduct(z, target_k, h_target);
      lambda -= h_target;  // safe: per-segment ops handle mixed layouts
      lambda *= (1.0 / k);
      result.lambda = std::move(lambda);

      result.optimality.mu = result.mu;
      if (verbose) {
        printf("  Optimality: mu=%.2e\n", result.mu);
      }
      arena.RestoreCursor(iter_mark);
      break;
    } else {
      // Save z₀ for frozen-J, then step.
      RowSpace z0 = z;
      { CONEX_TIMER(stats, cone_us);
        geodesicStepTarget(z, alpha, target_k);
      }

      // Frozen-Jacobian inner steps: use stale Gram (H(z₀)) with
      // frozen centering = -∇F(z_i) + H(z₀)·z_i.
      // Cost RHS (rhs1) is frozen (depends only on z₀ through H(z₀)·b).
      // d1 from the standard decomposition is reused.
      //
      // Debug: set refactor_inner=true to refactor at z_i and use the
      // standard (non-frozen) path.  This should produce the same
      // trajectory as baseline (max_frozen_steps=0).  Any difference
      // is a bug in the frozen-J logic.
      constexpr bool refactor_inner = false;

      for (int inner = 0; inner < max_frozen_steps; ++inner) {
        char* inner_mark = arena.SaveCursor();

        RowSpace target0_f = model.AllocRowSpace(arena);
        RowSpace target1_f = target1;  // default: frozen

        if (refactor_inner) {
          // Full refactor: makes inner iteration identical to outer.
          model.SetScaling(z);
          model.AssembleAndFactor();
          total_fac++;

          // Recompute centering RHS: A^T(-2∇F(z_i)).
          RowSpace grad_r = model.AllocRowSpace(arena);
          computeGradient(z, grad_r);
          grad_r *= -2.0;
          auto rhs0_r = model.AllocSolverRHS();
          rhs0_r.SetZero();
          model.AccumulateAtranspose(grad_r, rhs0_r);

          // Recompute optimality RHS: -(c + A^T H(z_i) b).
          RowSpace hb_r = model.AllocRowSpace(arena);
          hessianProduct(z, b, hb_r);
          auto rhs1_r = model.AllocSolverRHS();
          rhs1_r = cost_rhs;
          model.AccumulateAtranspose(hb_r, rhs1_r);
          rhs1_r *= -1;
          if (ts && !ts->equality_sub_assemblers().empty()) {
            rhs1_r += ts->EqualityAffineTermRHS();
          }

          // Solve both.
          auto y_r = model.AllocSolverRHS(2);
          y_r.SetColumn(0, rhs0_r);
          y_r.SetColumn(1, rhs1_r);
          model.SolveSolverRHS(y_r);
          total_sol += 2;

          // Recover targets.
          auto row_r = model.AllocRowSpace(arena, 2);
          model.MultiplyA(y_r, row_r);
          target0_f.col() = row_r.col(0);
          target1_f.col() = row_r.col(1);
          target1_f += b;
        } else {
          // Frozen centering: -∇F(z_i) + H(z₀)·z_i_raw
          // getConePoint recovers the raw cone point from stored representation
          // (identity for exp cone, inverse for symmetric cones).
          RowSpace grad_i = model.AllocRowSpace(arena);
          computeGradient(z, grad_i);  // ∇F(z_i)
          RowSpace z_raw_i = model.AllocRowSpace(arena);
          getConePoint(z_raw_i, z);
          RowSpace hz0_zi = model.AllocRowSpace(arena);
          hessianProduct(z0, z_raw_i, hz0_zi);  // H(z₀)·z_i_raw
          RowSpace centering = model.AllocRowSpace(arena);
          addScaled(centering, grad_i, hz0_zi, -1.0, 1.0);

          // Centering-only solve with stale Gram (1 back-solve).
          auto rhs0_f = model.AllocSolverRHS();
          rhs0_f.SetZero();
          model.AccumulateAtranspose(centering, rhs0_f);
          model.SolveSolverRHS(rhs0_f);
          total_sol += 1;

          // Recover target0 from solve.
          model.MultiplyA(rhs0_f, target0_f);
        }

        // Line search for k.
        double k_new_f = lineSearchTarget(z, target0_f, target1_f);
        if (k_new_f > k) k = k_new_f;

        // Assemble target_k, step.
        RowSpace target_k_f = model.AllocRowSpace(arena);
        addScaled(target_k_f, target0_f, target1_f, 1.0, k);
        double alpha_f = stepSize(z, target_k_f);

        if (verbose) {
          double d_sq_f = hessianNormSquared(z, target_k_f);
          double d_inf_f = (alpha_f < 1.0) ? std::sqrt(2.0 / alpha_f) : 0.0;
          double mu_f = 1.0 / (k * k);
          double gap_f = mu_f * (nu - d_sq_f);
          printf("  %3d.%d  %12s  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                 outer, inner + 1, "", k, d_inf_f, d_sq_f, gap_f);
        }

        geodesicStepTarget(z, alpha_f, target_k_f);
        arena.RestoreCursor(inner_mark);
      }
    }
    arena.RestoreCursor(iter_mark);
  }

  return result;
}

// Helper: evaluate the V(τ)=0 quadratic for the z-space θ-continuation.
// Returns (τ, stepSize_alpha) or (-1, 1e30) if no positive root.
//
// z_hess: point at which Hessian products are evaluated (= z for standard,
//         = z₀ for frozen-J where the Gram was factored at z₀).
// z:      current iterate (used for lineSearchTarget / hessianNormSquared
//         feasibility checks).
// grad_z: ∇F(z) at the current iterate (not z_hess).
static std::pair<double, double> EvalBarrierThetaCandidate(
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
  char* mark = arena.SaveCursor();

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
  if (disc < 0) { arena.RestoreCursor(mark); return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  // Pick the positive root with smaller ||d||_H².
  auto eval = [&](double tau) -> std::pair<double, double> {
    if (tau <= 0) return {1e30, 1e30};
    char* eval_mark = arena.SaveCursor();
    RowSpace t1_combined = model.AllocRowSpace(arena);
    addScaled(t1_combined, t1_tau, t1_th, tau, theta_cand);
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, ay0, t1_combined, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double k_max = lineSearchTarget(z, ay0, t1_combined);
    double d_inf = (k_max >= k) ? 0.0 : 2.0;  // feasible or not
    arena.RestoreCursor(eval_mark);
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
    arena.RestoreCursor(mark);
    return {-1, 1e30};
  }

  arena.RestoreCursor(mark);
  return {tau_out, d_inf_out};
}

GeodesicResult SolveGeodesicBarrierThetaContinuation(
    CompiledModel& model,
    RowSpace& z,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose,
    SolveStats* stats) {
  const auto& cost_rhs = model.cost_rhs();
  const RowSpace b = model.GetAffineTerm();
  const int m = z.total_rows();
  const double nu = barrierParameter(z);

  // Arena for temporary RowSpaces (zero heap allocation in the loop).
  Arena& arena = model.arena();

  // Starting point z_0 = z (the initial interior point).
  RowSpace z0 = model.AllocRowSpace(arena);
  z0 += z;  // per-segment copy from heap z

  // Precompute ∇F(z_0) (constant throughout).
  RowSpace grad_z0 = model.AllocRowSpace(arena);
  computeGradient(z0, grad_z0);

  RowSpace neg_grad_z0 = model.AllocRowSpace(arena);
  computeGradient(z0, neg_grad_z0);
  neg_grad_z0 *= -1.0;
  const double R_theta1 = dot(b, neg_grad_z0) + 1.0;

  // Duality cost (with +d at equality dual positions).
  auto duality_cost = MakeDualityCost(model);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;
  double k = 0, tau = 0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %8s  %10s  %12s  %12s  %12s  %12s\n",
           "out", "theta", "tau", "k", "d_sq", "gap", "mu");
    printf("  %s\n", std::string(85, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    char* iter_mark = arena.SaveCursor();

    // 0. Factor Gram = A^T H(z) A.
    bool factor_ok;
    { CONEX_TIMER(stats, factor_us);
      model.SetScaling(z);
      factor_ok = model.AssembleAndFactor();
    }
    if (!factor_ok) { arena.RestoreCursor(iter_mark); break; }
    if (stats) stats->factor_count++;

    // 1. Build 3 RHS vectors.
    RowSpace grad = model.AllocRowSpace(arena);
    computeGradient(z, grad);

    // rhs0: centering = -2 A^T ∇F(z).
    auto rhs0 = model.AllocSolverRHS();
    rhs0.SetZero();
    RowSpace neg2grad = model.AllocRowSpace(arena);
    neg2grad += grad;  // per-segment copy
    neg2grad *= -2.0;
    model.AccumulateAtranspose(neg2grad, rhs0);

    // rhs1: optimality (θ=0) = -(c + A^T H(z)b) + d_eq.
    RowSpace hb = model.AllocRowSpace(arena);
    hessianProduct(z, b, hb);
    auto rhs1 = model.AllocSolverRHS();
    rhs1 = cost_rhs;
    model.AccumulateAtranspose(hb, rhs1);
    rhs1 *= -1;
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      rhs1 += d_rhs;
    }

    // rhs2: θ=1 correction = A^T(∇F(z_0) - H(z)z_0) - d_eq.
    RowSpace hz0 = model.AllocRowSpace(arena);
    hessianProduct(z, z0, hz0);
    RowSpace theta1_vec = model.AllocRowSpace(arena);
    addScaled(theta1_vec, grad_z0, hz0, 1.0, -1.0);  // ∇F(z_0) - H(z)z_0
    auto rhs2 = model.AllocSolverRHS();
    rhs2 = rhs1;
    rhs2 *= -1;
    model.AccumulateAtranspose(theta1_vec, rhs2);

    // 2. Solve all 3.
    auto y = model.AllocSolverRHS(3);
    y.SetColumn(0, rhs0);
    y.SetColumn(1, rhs1);
    y.SetColumn(2, rhs2);
    { CONEX_TIMER(stats, solve_us);
      model.SolveSolverRHS(y);
    }
    if (stats) stats->solve_count += 3;
    total_fac++;
    total_sol += 3;

    SolverRHS y0_vec = model.MakeSolverRHS();
    SolverRHS y1_0_vec = model.MakeSolverRHS();
    SolverRHS y1_theta_vec = model.MakeSolverRHS();
    {
      int nb = y.num_blocks();
      for (int bk = 0; bk < nb; ++bk) {
        y0_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(0);
        y1_0_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(1);
        y1_theta_vec.supernodes->block(bk).col(0) = y.supernodes->block(bk).col(2);
      }
    }

    // 3. Compute A*y for all 3.
    auto row = model.AllocRowSpace(arena, 3);
    model.MultiplyA(y, row);
    RowSpace ay0 = model.AllocRowSpace(arena);
    RowSpace ay1_0 = model.AllocRowSpace(arena);
    RowSpace ay1_theta = model.AllocRowSpace(arena);
    ay0.col() = row.col(0);
    ay1_0.col() = row.col(1);
    ay1_theta.col() = row.col(2);

    // 4. Precompute target building blocks.
    RowSpace t1_tau = model.AllocRowSpace(arena);
    addScaled(t1_tau, ay1_0, b, 1.0, 1.0);     // Ay1_0 + b
    RowSpace z0_minus_b = model.AllocRowSpace(arena);
    addScaled(z0_minus_b, z0, b, 1.0, -1.0);
    RowSpace t1_th = model.AllocRowSpace(arena);
    addScaled(t1_th, ay1_theta, z0_minus_b, 1.0, 1.0);  // Ay1_theta + z_0 - b

    // 5. Binary search for θ: find smallest θ with V(τ)=0 feasible.
    constexpr double beta_target = 1.0;
    double theta_prev = theta;
    {
      double theta_lo = 0.0;
      double theta_hi = theta;
      for (int bisect = 0; bisect < 30; ++bisect) {
        double theta_mid = 0.5 * (theta_lo + theta_hi);
        auto [tau_try, d_inf_try] = EvalBarrierThetaCandidate(
            model, arena, duality_cost, z, z, b, grad, ay0, t1_tau, t1_th,
            y0_vec, y1_0_vec, y1_theta_vec, nu, R_theta1, theta_mid);
        if (tau_try > 0 && d_inf_try <= beta_target) {
          theta_hi = theta_mid;
        } else {
          theta_lo = theta_mid;
        }
      }
      theta = theta_hi;
    }
    k = 1.0 / std::sqrt(theta);


    // Evaluate at chosen θ.
    auto [tau_sel, d_inf_sel] = EvalBarrierThetaCandidate(
        model, arena, duality_cost, z, z, b, grad, ay0, t1_tau, t1_th,
        y0_vec, y1_0_vec, y1_theta_vec, nu, R_theta1, theta);
    if (tau_sel <= 0) {
      result.iterations = outer + 1;
      arena.RestoreCursor(iter_mark);
      break;
    }
    tau = tau_sel;

    // Final target and metrics.
    RowSpace t1_combined = model.AllocRowSpace(arena);
    addScaled(t1_combined, t1_tau, t1_th, tau, theta);
    RowSpace target_k = model.AllocRowSpace(arena);
    addScaled(target_k, ay0, t1_combined, 1.0, k);
    double d_sq = hessianNormSquared(z, target_k);
    double alpha = stepSize(z, target_k);
    double d_inf = (alpha < 1.0) ? std::sqrt(2.0 / alpha) : 0.0;
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    if (verbose) {
      printf("  %3d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e\n",
             outer, theta, tau, k, d_sq, gap, mu);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;

    bool converged = (mu < tolerance);
    bool last_iter = (outer + 1 == max_outer_iterations);

    if (converged || last_iter) {
      result.mu = mu;
      result.tau = tau;
      result.d_inf_norm = d_inf;
      result.d_sq_norm = d_sq;
      result.complementarity = gap;
      result.total_factorizations = total_fac;
      result.total_solves = total_sol;

      {
        auto x_rhs = model.AllocSolverRHS();
        x_rhs.SetZero();
        x_rhs.AddScaled(1.0 / k, y0_vec);
        x_rhs.AddScaled(tau, y1_0_vec);
        x_rhs.AddScaled(theta, y1_theta_vec);
        x_rhs *= (1.0 / tau);
        int nr = model.number_of_variables();
        result.x.resize(nr);
        result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); x_rhs.supernodes->GatherInto(xm); }
      }

      // Lambda recovery (heap — outlives arena).
      RowSpace lambda = model.MakeRowSpace();
      computeGradient(z, lambda);
      lambda *= -2.0;
      RowSpace h_target = model.AllocRowSpace(arena);
      hessianProduct(z, target_k, h_target);
      lambda -= h_target;
      lambda *= (1.0 / (k * tau));
      result.lambda = std::move(lambda);

      result.optimality.mu = result.mu;
      if (verbose) {
        printf("  Optimality: mu=%.2e\n", result.mu);
      }
      arena.RestoreCursor(iter_mark);
      if (converged) break;
    } else {
      // Geodesic step (always take — d_inf may be ≤ 1 by design in ThetaCont).
      RowSpace z_outer = z;  // heap deep copy (z is heap-backed)
      { CONEX_TIMER(stats, cone_us);
        geodesicStepTarget(z, alpha, target_k);
      }

      // Frozen-Jacobian inner steps: use stale Gram (H(z_outer)) with
      // frozen centering = -∇F(z_i) + H(z_outer)·z_i.
      // rhs1 (optimality) and rhs2 (theta correction) are frozen.
      // y1_0 and y1_theta are frozen. Only y0 (centering) is refreshed.
      //
      // Debug: set refactor_inner=true to refactor at z_i and use the
      // standard (non-frozen) path.  This should produce the same
      // trajectory as baseline (max_centering_steps=0).
      constexpr bool refactor_inner = false;

      for (int inner = 0; inner < max_centering_steps; ++inner) {
        char* inner_mark = arena.SaveCursor();

        RowSpace ay0_f = model.AllocRowSpace(arena);
        SolverRHS y0_f{}, y1_0_f = y1_0_vec, y1_theta_f = y1_theta_vec;
        RowSpace z_hess_f = z_outer;
        RowSpace grad_f = model.AllocRowSpace(arena);

        if (refactor_inner) {
          // Allocate fresh t1 buffers (refactor overwrites them).
          RowSpace t1_tau_f = model.AllocRowSpace(arena);
          RowSpace t1_th_f = model.AllocRowSpace(arena);
          // Full refactor: makes inner iteration identical to outer.
          z_outer = z;
          z_hess_f = z;
          model.SetScaling(z);
          model.AssembleAndFactor();
          total_fac++;

          // Recompute all 3 RHS.
          computeGradient(z, grad_f);

          auto rhs0_r = model.AllocSolverRHS();
          rhs0_r.SetZero();
          RowSpace neg2grad_r = model.AllocRowSpace(arena);
          neg2grad_r += grad_f;
          neg2grad_r *= -2.0;
          model.AccumulateAtranspose(neg2grad_r, rhs0_r);

          RowSpace hb_r = model.AllocRowSpace(arena);
          hessianProduct(z, b, hb_r);
          auto rhs1_r = model.AllocSolverRHS();
          rhs1_r = cost_rhs;
          model.AccumulateAtranspose(hb_r, rhs1_r);
          rhs1_r *= -1;
          if (ts && !ts->equality_sub_assemblers().empty()) {
            rhs1_r += ts->EqualityAffineTermRHS();
          }

          RowSpace hz0_r = model.AllocRowSpace(arena);
          hessianProduct(z, z0, hz0_r);
          RowSpace theta1_vec_r = model.AllocRowSpace(arena);
          addScaled(theta1_vec_r, grad_z0, hz0_r, 1.0, -1.0);
          auto rhs2_r = model.AllocSolverRHS();
          rhs2_r = rhs1_r;
          rhs2_r *= -1;
          model.AccumulateAtranspose(theta1_vec_r, rhs2_r);

          auto y_r = model.AllocSolverRHS(3);
          y_r.SetColumn(0, rhs0_r);
          y_r.SetColumn(1, rhs1_r);
          y_r.SetColumn(2, rhs2_r);
          model.SolveSolverRHS(y_r);
          total_sol += 3;

          y0_f = model.AllocSolverRHS();
          y1_0_f = model.AllocSolverRHS();
          y1_theta_f = model.AllocSolverRHS();
          {
            int nb_r = y_r.num_blocks();
            for (int bk = 0; bk < nb_r; ++bk) {
              y0_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(0);
              y1_0_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(1);
              y1_theta_f.supernodes->block(bk).col(0) = y_r.supernodes->block(bk).col(2);
            }
          }

          auto row_r = model.AllocRowSpace(arena, 3);
          model.MultiplyA(y_r, row_r);
          ay0_f.col() = row_r.col(0);
          RowSpace ay1_0_r = model.AllocRowSpace(arena);
          RowSpace ay1_theta_r = model.AllocRowSpace(arena);
          ay1_0_r.col() = row_r.col(1);
          ay1_theta_r.col() = row_r.col(2);
          addScaled(t1_tau_f, ay1_0_r, b, 1.0, 1.0);
          RowSpace z0_minus_b_r = model.AllocRowSpace(arena);
          addScaled(z0_minus_b_r, z0, b, 1.0, -1.0);
          addScaled(t1_th_f, ay1_theta_r, z0_minus_b_r, 1.0, 1.0);

          // Full theta binary search (same as outer).
          double theta_lo_r = 0.0;
          double theta_hi_r = theta;
          for (int bisect = 0; bisect < 30; ++bisect) {
            double theta_mid = 0.5 * (theta_lo_r + theta_hi_r);
            auto [tau_try, d_inf_try] = EvalBarrierThetaCandidate(
                model, arena, duality_cost, z, z, b, grad_f,
                ay0_f, t1_tau_f, t1_th_f,
                y0_f, y1_0_f, y1_theta_f,
                nu, R_theta1, theta_mid);
            if (tau_try > 0 && d_inf_try <= beta_target) {
              theta_hi_r = theta_mid;
            } else {
              theta_lo_r = theta_mid;
            }
          }
          theta = theta_hi_r;
          k = 1.0 / std::sqrt(theta);

          auto [tau_r, d_inf_r] = EvalBarrierThetaCandidate(
              model, arena, duality_cost, z, z, b, grad_f,
              ay0_f, t1_tau_f, t1_th_f,
              y0_f, y1_0_f, y1_theta_f,
              nu, R_theta1, theta);
          if (tau_r <= 0) { arena.RestoreCursor(inner_mark); break; }
          tau = tau_r;

          RowSpace t1_comb_r = model.AllocRowSpace(arena);
          addScaled(t1_comb_r, t1_tau_f, t1_th_f, tau, theta);
          RowSpace target_k_r = model.AllocRowSpace(arena);
          addScaled(target_k_r, ay0_f, t1_comb_r, 1.0, k);
          double alpha_r = stepSize(z, target_k_r);

          if (verbose) {
            double d_sq_r = hessianNormSquared(z, target_k_r);
            double mu_r = theta;
            double gap_r = mu_r * (nu - d_sq_r);
            printf("  %3d.%d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                   outer, inner + 1, theta, tau, k, d_sq_r, gap_r, mu_r);
          }
          geodesicStepTarget(z, alpha_r, target_k_r);
        } else {
          // Frozen centering: -∇F(z_i) + H(z_outer)·z_i.
          // Keep theta fixed, refresh centering, line search for k.
          computeGradient(z, grad_f);
          RowSpace z_raw_i = model.AllocRowSpace(arena);
          getConePoint(z_raw_i, z);
          RowSpace hz0_zi = model.AllocRowSpace(arena);
          hessianProduct(z_outer, z_raw_i, hz0_zi);
          RowSpace centering_f = model.AllocRowSpace(arena);
          addScaled(centering_f, grad_f, hz0_zi, -1.0, 1.0);

          auto rhs0_f = model.AllocSolverRHS();
          rhs0_f.SetZero();
          model.AccumulateAtranspose(centering_f, rhs0_f);
          model.SolveSolverRHS(rhs0_f);
          total_sol += 1;

          // Recover target0.
          model.MultiplyA(rhs0_f, ay0_f);

          // Frozen inner step: treat tau·t1_tau + theta·t1_th as
          // a single frozen "d1" direction. Line search for k using
          // the refreshed centering (ay0_f) and frozen d1.
          RowSpace d1_frozen = model.AllocRowSpace(arena);
          addScaled(d1_frozen, t1_tau, t1_th, tau, theta);
          double k_new_f = lineSearchTarget(z, ay0_f, d1_frozen);
          if (k_new_f > k) {
            k = k_new_f;
            theta = 1.0 / (k * k);
          }
          RowSpace target_k_f = model.AllocRowSpace(arena);
          addScaled(target_k_f, ay0_f, d1_frozen, 1.0, k);

          double alpha_f = stepSize(z, target_k_f);

          if (verbose) {
            double d_sq_f = hessianNormSquared(z, target_k_f);
            double mu_f = theta;
            double gap_f = mu_f * (nu - d_sq_f);
            printf("  %3d.%d  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  (frozen-J)\n",
                   outer, inner + 1, theta, tau, k, d_sq_f, gap_f, mu_f);
          }

          geodesicStepTarget(z, alpha_f, target_k_f);
        }
        arena.RestoreCursor(inner_mark);
      }
    }
    arena.RestoreCursor(iter_mark);
  }

  if (result.x.size() == 0) {
    result.x.assign(model.number_of_variables(), 0.0);
    result.mu = (k > 0) ? 1.0 / (k * k) : 1.0;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;
  }

  return result;
}

HybridDirection ComputeHybridDirection(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta,
    double tau_scale) {
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // Combined solve: centering 2·P(√W)(r) + cost -P(W)(b) in cone space.
  // cost_scale=1 because cost_rhs is unscaled; eq_scale=tau_scale for d_eq.
  auto var_rhs = MakeCostVarRHS(model, model.cost_rhs(), 1.0, tau_scale);
  RowSpace cone_rhs = addScaled(quadraticRepresentation(W, b),
                                quadraticRepresentation(sqrtW, r), -1, 2.0);
  RowSpace Ax = SolveConeSystem(model, var_rhs, cone_rhs);

  RowSpace slack_dir = addScaled(b, Ax, 1.0, 1.0);
  delta = addScaled(r,
      quadraticRepresentation(sqrtW, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);

  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

// M-based variant: uses applyM/applyMt instead of sqrt(W).
static HybridDirection ComputeHybridDirectionM(
    CompiledModel& model,
    const RowSpace& b,
    const RowSpace& M,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta,
    double tau_scale = 1.0) {
  // Combined solve: centering 2·M·r·M^T + cost -P(W)(b) in cone space.
  // cost_scale=1 because cost_rhs is unscaled; eq_scale=tau_scale for d_eq.
  auto var_rhs = MakeCostVarRHS(model, model.cost_rhs(), 1.0, tau_scale);
  RowSpace cone_rhs = addScaled(quadraticRepresentation(W, b),
                                applyM(M, r), -1, 2.0);
  RowSpace Ax = SolveConeSystem(model, var_rhs, cone_rhs);

  RowSpace slack_dir = addScaled(b, Ax, 1.0, 1.0);
  delta = addScaled(r, applyMt(M, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);

  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

HybridDirection HybridCenteringStep(
    CompiledModel& model,
    RowSpace& W,
    RowSpace& r) {
  Arena& arena = model.arena();
  const RowSpace b = model.GetAffineTerm();
  // HybridCenteringStep is a single-step helper used by tests.
  // Use M-based approach: initialize M = sqrt(W), update, recover W.
  RowSpace M = model.AllocRowSpace();
  EuclideanJordanAlgebra::sqrt(M, W);
  model.SetScaling(W);
  model.AssembleAndFactor();

  RowSpace d = model.AllocRowSpace();
  RowSpace delta = model.AllocRowSpace();
  auto info = ComputeHybridDirectionM(model, b, M, W, r, d, delta);

  double alpha = std::min(1.0, 2.0 / (info.d_inf * info.d_inf));
  updateM(M, r, alpha, d);
  W = squareM(M);
  return info;
}

GeodesicResult SolveGeodesicHybrid(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose,
    double initial_k,
    double tau,
    HybridSwitchPolicy policy) {
  const auto& cost_rhs = model.cost_rhs();
  Arena& arena = model.arena();
  RowSpace b = model.GetAffineTerm();
  const int m = b.total_rows();

  // Scale problem data by tau: b_scaled = tau*b, c_scaled = tau*c.
  // This puts the hybrid on the same central path as the HSD model
  // at the given tau.
  auto cost_scaled = model.AllocSolverRHS(); cost_scaled = cost_rhs;
  if (tau != 1.0) {
    b *= tau;
    cost_scaled *= tau;
  }

  RowSpace r = model.AllocRowSpace();
  setOnes(r);

  // M-based automorphism: M tracks the full automorphism, r stays in M-frame.
  RowSpace M = model.AllocRowSpace();
  setOnes(M);  // M = I initially

  // Initial scaling: use caller-supplied k if positive, otherwise
  // decompose at W to find the minimum-norm k.
  if (initial_k <= 0) {
    model.SetScaling(W);
    model.AssembleAndFactor();
  }
  // When initial_k > 0, reuse the existing factorization from the caller.
  if (initial_k > 0) {
    r *= (1.0 / initial_k);
  } else {
    RowSpace d0 = model.AllocRowSpace();
    RowSpace d1 = model.AllocRowSpace();
    ComputeDecomposition(model, arena, cost_scaled, b, W, d0, d1);
    double d0d1 = dot(d0, d1);
    double d1sq = squaredNorm(d1);
    if (d1sq > 1e-30) {
      double k_init = std::max(1e-6, -d0d1 / d1sq);
      r *= (1.0 / k_init);
    }
  }

  int total_fac = 0;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates = 0;

  RowSpace ones = model.AllocRowSpace();
  setOnes(ones);

  int r_updates_this_fac = 0;
  double g = 0, d_inf = 0, d_sq = 0, mslack = 0;

  // Unscaled b for bTl computation (b was scaled by tau above).
  RowSpace b_unscaled = model.GetAffineTerm();

  // Duality cost: cost_rhs + d_eq at dual positions.
  // duality_cost.dot(y) = c'x + d'nu (full objective with equality terms).
  auto duality_cost = MakeDualityCost(model);

  if (verbose) {
    printf("  %3s  %12s  %10s %10s  %12s  %12s  %12s  %6s  %6s\n",
           "it", "gap", "d_pre", "d_post", "|r|^2/m", "bTl", "cTx",
           "r_upd", "step");
    printf("  %s\n", std::string(100, '-').c_str());
  }

  RowSpace last_delta = model.AllocRowSpace();

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = model.AllocRowSpace();
    RowSpace delta = model.AllocRowSpace();
    auto info = ComputeHybridDirectionM(model, b, M, W, r, d, delta, tau);
    last_delta = delta;
    total_sol++;

    double d_inf_pre = info.d_inf;
    g = info.gap;
    d_inf = info.d_inf;
    d_sq = info.d_sq;
    mslack = info.min_slack;

    // Divergence guard: abort before taking a step with corrupt data.
    if (d_inf > 10 || !std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: hybrid diverging (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);
      break;
    }

    bool do_center = policy(g, d_inf, r_updates_this_fac);
    if (do_center) {
      // Centering step: update M (and r for SOC), then refactor.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateM(M, r, alpha, d);
      W = squareM(M);
      model.SetScaling(W);
      if (!model.AssembleAndFactor()) break;
      total_fac++;
      result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                                   r_updates_this_fac, mslack});
      r_updates_this_fac = 0;
    } else {
      // Update r using delta (shrinks when gap > 0, may grow when gap < 0).
      // W is unchanged so the Gram matrix A'W²A is the same — no
      // refactorization needed, only a back-solve with the new RHS.
      shrinkR(r, delta);
      r_updates_this_fac++;
      r_updates++;
    }
    // Recompute direction at current (W, r).  After a W-update this uses
    // the fresh factorization; after an r-update it reuses the existing one.
    {
      RowSpace d2 = model.AllocRowSpace();
      RowSpace delta2 = model.AllocRowSpace();
      auto info2 = ComputeHybridDirectionM(model, b, M, W, r, d2, delta2, tau);
      total_sol++;
      g = info2.gap;
      d_inf = info2.d_inf;
      d_sq = info2.d_sq;
      mslack = info2.min_slack;
      last_delta = delta2;
    }
    if (verbose) {
      double mu_r = squaredNorm(r) / m;
      // Compute physical bTl and cTx (divided by tau).
      // lambda_lifted = applyM(M, r + delta2).
      RowSpace r_plus_delta = model.AllocRowSpace();
      addScaled(r_plus_delta, r, last_delta, 1.0, 1.0);
      RowSpace lam_v = model.AllocRowSpace();
      applyM(lam_v, M, r_plus_delta);
      double bTl_phys = dot(b_unscaled, lam_v) / tau;
      // cTx + dTnu: re-solve and dot with duality_cost (includes d_eq).
      auto y_v = model.AllocSolverRHS();
      y_v = cost_scaled;
      y_v *= -1;
      RowSpace Mr = model.AllocRowSpace();
      applyM(Mr, M, r);
      RowSpace PWb = model.AllocRowSpace();
      quadraticRepresentation(PWb, W, b);
      RowSpace v_v = model.AllocRowSpace();
      addScaled(v_v, PWb, Mr, -1, 2.0);
      model.AccumulateAtranspose(v_v, y_v);
      auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
      if (ts && !ts->equality_sub_assemblers().empty()) {
        auto d_rhs = ts->EqualityAffineTermRHS();
        if (tau != 1.0) d_rhs *= tau;
        y_v += d_rhs;
      }
      model.SolveSolverRHS(y_v);
      if (ts && !ts->equality_sub_assemblers().empty()) {
        auto d_rhs2 = ts->EqualityAffineTermRHS();
        bTl_phys += model.dot(d_rhs2, y_v) / tau;
      }
      auto dc_scaled = model.AllocSolverRHS(); dc_scaled = duality_cost;
      dc_scaled *= tau;
      double cTx_phys = model.dot(dc_scaled, y_v) / (tau * tau);
      printf("  %3d  %12.4e  %10.4e %10.4e  %12.4e  %12.4e  %12.4e  %6d  %s\n",
             iter, g, d_inf_pre, d_inf, mu_r, bTl_phys, cTx_phys,
             r_updates_this_fac, do_center ? "center" : "shrink");
    }
    if (std::abs(g) < tolerance && d_inf <= 1.001) break;
  }

  result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                               r_updates_this_fac, mslack});
  result.d_inf_norm = d_inf;
  result.d_sq_norm = d_sq;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x: solve Gram * y = RHS at the final (W, r).
  {
    model.SetScaling(W);
    model.AssembleAndFactor();
    auto y = model.AllocSolverRHS();
    y = cost_scaled;
    y *= -1;
    RowSpace PWb_r = model.AllocRowSpace();
    quadraticRepresentation(PWb_r, W, b);
    RowSpace Mr_r = model.AllocRowSpace();
    applyM(Mr_r, M, r);
    RowSpace v = model.AllocRowSpace();
    addScaled(v, PWb_r, Mr_r, -1, 2.0);
    model.AccumulateAtranspose(v, y);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      if (tau != 1.0) d_rhs *= tau;
      y += d_rhs;
    }
    model.SolveSolverRHS(y);
    int nr = model.number_of_variables();
    result.x.resize(nr);
    result.x.resize(model.number_of_variables());
    { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), result.x.size()); y.supernodes->GatherInto(xm); }
    if (tau != 1.0 && tau > 0) for (auto& v : result.x) v /= tau;
  }

  // Optimality check against the UNSCALED problem.
  {
    auto x_rhs = model.AllocSolverRHS();
    x_rhs.ScatterFrom(result.x.data(), result.x.size());
    RowSpace r_plus_delta_o = model.AllocRowSpace();
    addScaled(r_plus_delta_o, r, last_delta, 1.0, 1.0);
    RowSpace lambda = model.MakeRowSpace();
    applyM(lambda, M, r_plus_delta_o);
    if (tau != 1.0 && tau > 0) lambda *= (1.0 / tau);
    result.optimality = CheckOptimality(model, x_rhs, lambda);
    result.optimality.mu = result.mu;
    result.lambda = lambda;
  }

  if (verbose) {
    printf("  Optimality: compl=%.2e, "
           "min_s=%.2e, min_lam=%.2e\n",
           result.optimality.complementarity,
           result.optimality.min_slack,
           result.optimality.min_dual);
  }

  return result;
}

}  // namespace conex
