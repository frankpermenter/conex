#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/tree_solver/kkt_tree_solver.h"

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
using EuclideanJordanAlgebra::updateAutomorphism;

// Compute b_theta = theta * e + (1 - theta) * b.
static RowSpace BlendAffine(KKTSolverBase& kkt, const RowSpace& b,
                            double theta) {
  RowSpace b_theta = kkt.MakeRowSpace();
  if (theta == 0.0) {
    b_theta = b;
  } else if (theta == 1.0) {
    setOnes(b_theta);
  } else {
    RowSpace ones = kkt.MakeRowSpace();
    setOnes(ones);
    b_theta = addScaled(ones, b, theta, 1.0 - theta);
  }
  return b_theta;
}

HybridRDirection ComputeHybridRDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta,
    Eigen::VectorXd* y_out) {
  RowSpace b_theta = BlendAffine(kkt, b, theta);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // RHS = -(c + A^T P(W)(b_theta)) + 2*A^T P(W^{1/2})(r) + d_eq
  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  RowSpace v = quadraticRepresentation(W, b_theta);
  kkt.AccumulateAtranspose(v, y);
  y *= -1;
  // + 2*A^T P(W^{1/2})(r)
  v = quadraticRepresentation(sqrtW, r);
  v *= 2.0;
  kkt.AccumulateAtranspose(v, y);
  // + d_eq (equality RHS).
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    y += d_rhs;
  }
  kkt.SolveSolverRHS(y);

  // Optionally return the solve vector.
  if (y_out) {
    int nv = kkt.number_of_variables();
    y_out->resize(nv);
    y.supernodes->GatherInto(*y_out);
  }

  // delta = r - P(W^{1/2})(b_theta + A*y)
  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  RowSpace slack_dir = addScaled(b_theta, row, 1.0, 1.0);
  delta = addScaled(r, quadraticRepresentation(sqrtW, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);

  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

std::pair<double, double> VerifyHybridREquations(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    const RowSpace& d,
    const RowSpace& delta,
    const Eigen::VectorXd& y) {
  RowSpace b_theta = BlendAffine(kkt, b, theta);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // --- Primal check ---
  // delta should equal r - P(W^{1/2})(b_theta + A*y).
  auto y_rhs = kkt.MakeSolverRHS();
  y_rhs = kkt.MakeBlockVariable(y);
  RowSpace Ay = kkt.MakeRowSpace();
  kkt.MultiplyA(y_rhs, Ay);
  RowSpace slack = addScaled(b_theta, Ay, 1.0, 1.0);
  RowSpace delta_expected = addScaled(r,
      quadraticRepresentation(sqrtW, slack), 1.0, -1.0);
  double primal_res = normInf(addScaled(delta, delta_expected, 1.0, -1.0));

  // --- Dual check ---
  // From the KKT solve: (A'W²A + Q)y = RHS.
  // Lambda = P(W^{1/2})(r + delta) = 2*P(W^{1/2})(r) - P(W)(b_theta + Ay).
  // Therefore: A^T lambda - Qy = c  (stationarity, ignoring d_eq).
  // With d_eq: A^T lambda - Qy + d_eq = c.
  // Check: A^T lambda - Qy - c = 0  (d_eq enters the KKT dual block).
  RowSpace lambda = quadraticRepresentation(sqrtW,
      addScaled(r, delta, 1.0, 1.0));
  auto at_lambda = kkt.MakeSolverRHS();
  at_lambda.SetZero();
  kkt.AccumulateAtranspose(lambda, at_lambda);

  // - Qy
  auto qy = kkt.MakeSolverRHS();
  qy.SetZero();
  kkt.AccumulateQx(y_rhs, qy);
  at_lambda -= qy;

  // - c
  at_lambda -= cost_rhs;

  int n = kkt.number_of_variables();
  Eigen::VectorXd dual_err(n);
  at_lambda.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

  return {primal_res, dual_res};
}

HybridRDecomposition ComputeHybridRDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta) {
  RowSpace b_theta = BlendAffine(kkt, b, theta);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  int nv = kkt.number_of_variables();

  // Solve 1 (centering, tau-independent):
  //   RHS_center = 2*A'P(W^{1/2})(r)
  auto y_c = kkt.MakeSolverRHS();
  y_c.SetZero();
  RowSpace v = quadraticRepresentation(sqrtW, r);
  v *= 2.0;
  kkt.AccumulateAtranspose(v, y_c);
  kkt.SolveSolverRHS(y_c);

  // Solve 2 (cost, coefficient of tau):
  //   RHS_cost = -(c + A'P(W)(b_theta)) + d_eq
  //   d_eq scales with tau (following ThetaContinuation's convention).
  auto y_t = kkt.MakeSolverRHS();
  y_t = cost_rhs;
  v = quadraticRepresentation(W, b_theta);
  kkt.AccumulateAtranspose(v, y_t);
  y_t *= -1;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts && !ts->equality_sub_assemblers().empty()) {
    y_t += ts->EqualityAffineTermRHS();
  }
  kkt.SolveSolverRHS(y_t);

  // delta_center = r - P(W^{1/2})(A*y_center)
  // delta_cost = -P(W^{1/2})(b_theta + A*y_cost)
  HybridRDecomposition decomp;
  decomp.y_center.resize(nv);
  y_c.supernodes->GatherInto(decomp.y_center);
  decomp.y_cost.resize(nv);
  y_t.supernodes->GatherInto(decomp.y_cost);

  RowSpace Ay_c = kkt.MakeRowSpace();
  kkt.MultiplyA(y_c, Ay_c);
  decomp.delta_center = addScaled(r,
      quadraticRepresentation(sqrtW, Ay_c), 1.0, -1.0);

  RowSpace Ay_t = kkt.MakeRowSpace();
  kkt.MultiplyA(y_t, Ay_t);
  RowSpace b_plus_Ayt = addScaled(b_theta, Ay_t, 1.0, 1.0);
  decomp.delta_cost = quadraticRepresentation(sqrtW, b_plus_Ayt);
  decomp.delta_cost *= -1.0;

  return decomp;
}

HybridRDirection EvalHybridRAtTau(
    KKTSolverBase& kkt,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta) {
  // delta(tau) = delta_center + tau * delta_cost
  delta = addScaled(decomp.delta_center, decomp.delta_cost, 1.0, tau);
  d = solveLyapunovForD(r, delta);
  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

GeodesicResult SolveGeodesicThetaContinuationR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);
  const double bT_ones = dot(b, ones);
  const double R = bT_ones + 1.0;

  // Duality cost (cost_rhs + equality dual correction).
  auto duality_cost = kkt.MakeSolverRHS();
  duality_cost = cost_rhs;
  auto* ts_init = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts_init && !ts_init->equality_sub_assemblers().empty()) {
    duality_cost += ts_init->EqualityAffineTermRHS();
  }

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);
  double theta = 1.0;
  double tau = 1.0;

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  if (verbose) {
    printf("  %3s %6s %8s  %12s  %10s %10s  %12s  %6s  %6s\n",
           "it", "theta", "tau", "gap", "d_pre", "d_post", "|r|^2/m",
           "r_upd", "step");
    printf("  %s\n", std::string(100, '-').c_str());
  }

  RowSpace last_delta = kkt.MakeRowSpace();
  bool need_decomp = true;
  HybridRDecomposition decomp;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute the two-solve decomposition when needed (after W-updates).
    if (need_decomp) {
      decomp = ComputeHybridRDecomposition(kkt, cost_rhs, b, W, r, theta);
      total_sol += 2;
      need_decomp = false;

      // Compute duality coefficients for tau selection.
      // lambda(tau) = P(W^{1/2})(r + delta_center + tau*delta_cost)
      // x(tau) = y_center + tau*y_cost
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam_c = quadraticRepresentation(sqrtW,
          addScaled(r, decomp.delta_center, 1.0, 1.0));
      RowSpace lam_t = quadraticRepresentation(sqrtW, decomp.delta_cost);

      double sigma_0 = dot(b, lam_c);
      double sigma_1 = dot(b, lam_t);

      auto yc_rhs = kkt.MakeSolverRHS();
      yc_rhs = kkt.MakeBlockVariable(decomp.y_center);
      auto yt_rhs = kkt.MakeSolverRHS();
      yt_rhs = kkt.MakeBlockVariable(decomp.y_cost);

      double gamma_0 = duality_cost.dot(yc_rhs);
      double gamma_1 = duality_cost.dot(yt_rhs);

      // QP terms.
      auto qyc = kkt.MakeSolverRHS(); qyc.SetZero();
      kkt.AccumulateQx(yc_rhs, qyc);
      auto qyt = kkt.MakeSolverRHS(); qyt.SetZero();
      kkt.AccumulateQx(yt_rhs, qyt);
      double q00 = qyc.dot(yc_rhs);
      double q01 = qyc.dot(yt_rhs);
      double q11 = qyt.dot(yt_rhs);

      // Inner products for gap(tau).
      double r_sq = squaredNorm(r);
      double dc_sq = squaredNorm(decomp.delta_center);
      double dt_sq = squaredNorm(decomp.delta_cost);
      double dc_dt = dot(decomp.delta_center, decomp.delta_cost);
      double mu_0 = r_sq - dc_sq;

      // Quadratic: beta*tau^2 + alpha_q*tau + mu_eff = 0
      double beta = sigma_1 + gamma_1 + q11 - dt_sq;
      double alpha_q = sigma_0 + gamma_0 + 2*q01 - 2*dc_dt - theta*R;
      double mu_eff = q00 + mu_0;

      double discr = alpha_q * alpha_q - 4.0 * beta * mu_eff;
      if (discr >= 0 && std::abs(beta) > 1e-30) {
        double sq = std::sqrt(discr);
        double t1 = (-alpha_q + sq) / (2.0 * beta);
        double t2 = (-alpha_q - sq) / (2.0 * beta);
        // Pick the positive root with smaller ||d||^2 (= smaller gap violation).
        // ||delta(t)||^2 = dc_sq + 2*t*dc_dt + t^2*dt_sq, and
        // ||d||^2 ~ ||delta||^2, so pick the root with smaller ||delta||^2.
        auto dsq = [&](double t) {
          return dc_sq + 2*t*dc_dt + t*t*dt_sq;
        };
        if (t1 > 0 && t2 > 0)
          tau = (dsq(t1) < dsq(t2)) ? t1 : t2;
        else if (t1 > 0) tau = t1;
        else if (t2 > 0) tau = t2;
      }
    }

    // Evaluate direction at current (tau, r, theta).
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = EvalHybridRAtTau(kkt, decomp, r, tau, d, delta);
    last_delta = delta;
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
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;
      need_decomp = true;  // recompute decomposition at new W
    } else {
      shrinkR(r, delta);
      r_updates_since_fac++;
      theta = std::abs(g) / m;
      // r and theta changed — need new decomposition for the direction,
      // but keep tau fixed until next W-update.
      need_decomp = true;
    }

    // Recompute decomposition and tau at updated state.
    if (need_decomp) {
      decomp = ComputeHybridRDecomposition(kkt, cost_rhs, b, W, r, theta);
      total_sol += 2;
      need_decomp = false;

      // Recompute tau at updated state (same quadratic).
      RowSpace sqrtW2 = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam_c2 = quadraticRepresentation(sqrtW2,
          addScaled(r, decomp.delta_center, 1.0, 1.0));
      RowSpace lam_t2 = quadraticRepresentation(sqrtW2, decomp.delta_cost);
      double s0 = dot(b, lam_c2), s1 = dot(b, lam_t2);
      auto yc2 = kkt.MakeSolverRHS();
      yc2 = kkt.MakeBlockVariable(decomp.y_center);
      auto yt2 = kkt.MakeSolverRHS();
      yt2 = kkt.MakeBlockVariable(decomp.y_cost);
      double g0 = duality_cost.dot(yc2), g1 = duality_cost.dot(yt2);
      auto qyc2 = kkt.MakeSolverRHS(); qyc2.SetZero();
      kkt.AccumulateQx(yc2, qyc2);
      auto qyt2 = kkt.MakeSolverRHS(); qyt2.SetZero();
      kkt.AccumulateQx(yt2, qyt2);
      double qq00 = qyc2.dot(yc2), qq01 = qyc2.dot(yt2), qq11 = qyt2.dot(yt2);
      double rsq = squaredNorm(r), dcsq = squaredNorm(decomp.delta_center);
      double dtsq = squaredNorm(decomp.delta_cost);
      double dcdt = dot(decomp.delta_center, decomp.delta_cost);
      double bt = s1+g1+qq11-dtsq, aq = s0+g0+2*qq01-2*dcdt-theta*R;
      double me = qq00+(rsq-dcsq);
      double disc = aq*aq - 4*bt*me;
      if (disc >= 0 && std::abs(bt) > 1e-30) {
        double sq = std::sqrt(disc);
        double t1 = (-aq+sq)/(2*bt), t2 = (-aq-sq)/(2*bt);
        if (t1 > 0 && t2 > 0) tau = (std::abs(t1-tau)<std::abs(t2-tau))?t1:t2;
        else if (t1 > 0) tau = t1;
        else if (t2 > 0) tau = t2;
      }

      // Evaluate at new tau for post-step diagnostics.
      RowSpace d2 = kkt.MakeRowSpace();
      RowSpace delta2 = kkt.MakeRowSpace();
      auto info2 = EvalHybridRAtTau(kkt, decomp, r, tau, d2, delta2);
      last_delta = delta2;
      g = info2.gap;
      d_inf = info2.d_inf;
    }

    if (verbose) {
      printf("  %3d %6.4f %8.4f  %12.4e  %10.4e %10.4e  %12.4e  %6d  %s\n",
             iter, theta, tau, g, d_inf_pre, d_inf,
             squaredNorm(r) / m, r_updates_since_fac,
             do_center ? "center" : "r+theta");
    }

    if (theta < tolerance && std::abs(g) < tolerance && d_inf <= 1.001)
      break;
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x (de-homogenized by tau).
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    // x = y_center + tau * y_cost, then divide by tau.
    Eigen::VectorXd x_lifted = decomp.y_center + tau * decomp.y_cost;
    result.x = x_lifted / tau;
  }

  // Lambda and optimality.
  {
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lambda = quadraticRepresentation(sqrtW,
        addScaled(r, last_delta, 1.0, 1.0));
    if (tau > 0 && tau != 1.0) lambda *= (1.0 / tau);
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
    result.lambda = lambda;
  }

  return result;
}

GeodesicResult SolveGeodesicHybridR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);
  double theta = 1.0;

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();
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

  RowSpace last_delta = kkt.MakeRowSpace();

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = ComputeHybridRDirection(kkt, cost_rhs, b, W, r, theta,
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
      // W-update: centering step. Keep r and theta frozen.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
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
      RowSpace d2 = kkt.MakeRowSpace();
      RowSpace delta2 = kkt.MakeRowSpace();
      auto info2 = ComputeHybridRDirection(kkt, cost_rhs, b, W, r, theta,
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

    if (theta < tolerance && std::abs(g) < tolerance && d_inf <= 1.001)
      break;
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x.
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    RowSpace b_theta = BlendAffine(kkt, b, theta);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    auto y = kkt.MakeSolverRHS();
    y = cost_rhs;
    RowSpace v = quadraticRepresentation(W, b_theta);
    kkt.AccumulateAtranspose(v, y);
    y *= -1;
    v = quadraticRepresentation(sqrtW, r);
    v *= 2.0;
    kkt.AccumulateAtranspose(v, y);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
    if (ts && !ts->equality_sub_assemblers().empty()) {
      y += ts->EqualityAffineTermRHS();
    }
    kkt.SolveSolverRHS(y);
    int nr = kkt.number_of_variables();
    result.x.resize(nr);
    y.supernodes->GatherInto(result.x);
  }

  // Lambda and optimality.
  {
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lambda = quadraticRepresentation(sqrtW,
        addScaled(r, last_delta, 1.0, 1.0));
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
    result.lambda = lambda;
  }

  return result;
}

}  // namespace conex
