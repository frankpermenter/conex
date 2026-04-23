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
    RowSpace& delta) {
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

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Scale b and c by tau.
    RowSpace b_tau = b;
    b_tau *= tau;
    auto cost_tau = kkt.MakeSolverRHS();
    cost_tau = cost_rhs;
    cost_tau *= tau;

    // Compute direction at (W, r, theta, tau).
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = ComputeHybridRDirection(kkt, cost_tau, b_tau, W, r, theta,
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

    // Update tau via duality identity: b'λ + c'x + μ/τ = θ·R.
    // λ = P(W^{1/2})(r + delta), x from the solve vector.
    {
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam = quadraticRepresentation(sqrtW,
          addScaled(r, delta, 1.0, 1.0));
      // b'λ (using unscaled b).
      double bT_lam = dot(b, lam);
      // c'x: recover x from the solve. The solve gave y for the scaled
      // problem (tau*c, tau*b). The primal is x = y (from SolverRHS).
      // For duality: use duality_cost (unscaled) dotted with y/tau.
      // Actually, the solve vector y satisfies the scaled KKT.
      // The physical x = y, and the cost contribution is duality_cost'*y.
      // But the solve was at tau*cost, so y corresponds to the tau-scaled problem.
      // We need x for the UNSCALED problem: x_unscaled = y.
      // c'x = cost_rhs'*y (but y is in SolverRHS format from the solve).
      // We don't have y directly — ComputeHybridRDirection doesn't return it.
      // Use the duality identity differently: at the current tau,
      //   tau*(b'λ_phys + c'x_phys) + μ = θ·R·τ
      // where λ_phys = λ/τ, x_phys = x/τ. But λ and x are at the scaled problem.
      // Simpler: the complementarity is gap = |r|²-|δ|², and
      //   gap + τ·κ = (m+1)·μ where μ = gap/(m)...
      //
      // For now, use the simple relationship:
      //   tau_new = (b'λ + μ_eff) / (θ·R - c_eff)
      // where we approximate from the current iterate.

      // Actually, the simplest correct approach: the duality identity at
      // the tau-scaled problem gives V(tau) = 0 where V is the violation
      // quadratic. We need:
      //   sigma = b'·P(W^{1/2})(r + delta) / tau  (dual objective contribution)
      //   gamma = duality_cost'·x / tau             (primal objective contribution)
      // Then: sigma + gamma + mu/tau = theta*R
      // => tau = (sigma_lifted + gamma_lifted + mu) / (theta*R)
      // where sigma_lifted = sigma*tau, gamma_lifted = gamma*tau.

      double sigma_lifted = bT_lam;  // b' * P(W^{1/2})(r+delta) — already at tau-scale

      // For gamma: we need duality_cost' * y where y is the solve vector.
      // ComputeHybridRDirection doesn't expose y. Recompute it cheaply:
      // delta = r - P(W^{1/2})(b_tau_theta + A*y), so
      // P(W^{1/2})(b_tau_theta + A*y) = r - delta
      // b_tau_theta + A*y = P(W^{-1/2})(r - delta)
      // A*y = P(W^{-1/2})(r - delta) - b_tau_theta
      // But we need y itself, not A*y. Without y, we can't compute c'x.

      // Alternative: compute c'x from the slack/dual relationship.
      // KKT stationarity: tau*c + A'*P(W)(tau*b_theta) = A'*lambda + ...
      // This is messy. Skip the full duality identity for now.
      // Use a simpler tau update based on the gap.

      double mu_r = squaredNorm(r) / m;
      // From the duality identity (approximate):
      //   tau ≈ mu_r / (theta * R - bT_lam/tau_old - ...)
      // This requires too many unknowns. Use the current tau and just
      // verify it's reasonable.
      //
      // For the initial implementation, keep tau=1 (no update).
      // TODO: implement proper tau update from duality identity.
    }

    bool do_center = (g < 0);

    if (do_center) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;
    } else {
      shrinkR(r, delta);
      r_updates_since_fac++;
      theta = std::abs(g) / m;
    }

    // Recompute direction at updated state.
    {
      RowSpace b_tau2 = b;
      b_tau2 *= tau;
      auto cost_tau2 = kkt.MakeSolverRHS();
      cost_tau2 = cost_rhs;
      cost_tau2 *= tau;

      RowSpace d2 = kkt.MakeRowSpace();
      RowSpace delta2 = kkt.MakeRowSpace();
      auto info2 = ComputeHybridRDirection(kkt, cost_tau2, b_tau2, W, r, theta,
                                            d2, delta2);
      last_delta = delta2;
      g = info2.gap;
      d_inf = info2.d_inf;
      total_sol++;
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

  // Recover x (at the final tau).
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    RowSpace b_theta = BlendAffine(kkt, b, theta);
    b_theta *= tau;
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    auto y = kkt.MakeSolverRHS();
    y = cost_rhs;
    y *= tau;
    RowSpace v = quadraticRepresentation(W, b_theta);
    kkt.AccumulateAtranspose(v, y);
    y *= -1;
    v = quadraticRepresentation(sqrtW, r);
    v *= 2.0;
    kkt.AccumulateAtranspose(v, y);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
    if (ts && !ts->equality_sub_assemblers().empty()) {
      auto d_rhs = ts->EqualityAffineTermRHS();
      if (tau != 1.0) d_rhs *= tau;
      y += d_rhs;
    }
    kkt.SolveSolverRHS(y);
    int nr = kkt.number_of_variables();
    result.x.resize(nr);
    y.supernodes->GatherInto(result.x);
    // De-homogenize.
    if (tau > 0 && tau != 1.0) result.x /= tau;
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
