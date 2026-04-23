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

    // Compute direction at (W, r, theta, tau), with solve vector y.
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    Eigen::VectorXd y_vec;
    auto info = ComputeHybridRDirection(kkt, cost_tau, b_tau, W, r, theta,
                                         d, delta, &y_vec);
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

    // Update tau via duality identity.
    //
    // At the tau-scaled problem, the duality identity is:
    //   b'λ + (c + d_eq)'x + x'Qx/τ + μ/τ = θ·R·τ
    // where λ = P(W^{1/2})(r+δ), x = y (solve vector), μ = gap.
    // All quantities are at the tau-scaled level.
    //
    // Rearranging for τ (multiply by τ, collect):
    //   b'λ·τ + (c+d_eq)'x·τ + x'Qx + μ = θ·R·τ²
    //   θ·R·τ² - (b'λ + (c+d_eq)'x)·τ - (x'Qx + μ) = 0
    //
    // But b'λ and c'x are ALREADY at the tau-scaled level (the direction
    // was computed with tau*b, tau*c). So they implicitly include tau.
    // The UNSCALED quantities are: b'λ_unscaled = b'λ (λ doesn't depend on tau
    // scaling — it comes from r and delta which are tau-independent).
    // And c'x_unscaled: the solve vector y satisfies (A'W²A)y = RHS(tau),
    // and x = y. The cost contribution c'x = cost_rhs'·y, but the solve
    // used cost_tau = tau*cost_rhs. So the contribution is already tau-scaled.
    //
    // Cleaner: evaluate everything in UNSCALED terms.
    // λ = P(W^{1/2})(r + δ) — independent of tau (r and δ are tau-independent? NO)
    //
    // Actually δ DOES depend on tau because the direction was computed with
    // tau-scaled data. So λ = P(W^{1/2})(r+δ(tau)) depends on tau.
    //
    // Simplest correct approach: compute the duality identity at the CURRENT
    // tau, then solve for the NEW tau.
    //
    // At current tau:
    //   b'λ + dc'x + xQx/τ + μ/τ = θ·R
    // where dc = duality_cost (includes equality dual correction).
    //
    // This gives τ_new from: τ_new satisfies the identity at the NEXT iterate.
    // But we don't know the next iterate. Use the current iterate's values.
    //
    // From: b'λ + dc'x + μ/τ = θ·R  (LP case, Q=0, dividing scaled eqn by τ)
    //   τ = μ / (θ·R - b'λ/τ - dc'x/τ)
    //
    // The b'λ and dc'x are at the TAU-scaled problem. To get the unscaled:
    //   b'λ_unscaled = b'λ  (λ doesn't change with tau-scaling of data)
    //                   ... actually it does, because δ changes.
    //
    // Let me just compute everything directly.
    {
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam = quadraticRepresentation(sqrtW,
          addScaled(r, delta, 1.0, 1.0));

      // b'λ using UNSCALED b.
      double bT_lam = dot(b, lam);

      // c'x using duality_cost (UNSCALED) dotted with y.
      auto y_rhs = kkt.MakeSolverRHS();
      y_rhs = kkt.MakeBlockVariable(y_vec);
      double dc_x = cost_rhs.dot(y_rhs);

      // x'Qx contribution.
      auto qx_rhs = kkt.MakeSolverRHS();
      qx_rhs.SetZero();
      kkt.AccumulateQx(y_rhs, qx_rhs);
      double xQx = qx_rhs.dot(y_rhs);

      double mu = g;  // gap = |r|² - |δ|² = total complementarity

      // Duality identity (at the tau-scaled problem, divided by τ):
      //   b'λ/τ + dc'x/τ + xQx/τ² + μ/τ = θ·R
      //
      // But b, dc, Q are UNSCALED while the direction used tau-scaled data.
      // The solve vector y satisfies: (A'W²A + Q)y = -τc - A'P(W)(τb_θ) + 2A'P(W^½)r
      // So y (and hence δ, λ) depend on τ. The duality identity at the
      // current iterate gives:
      //   bT_lam + dc_x + xQx/τ + μ/τ = θ·R·τ
      //
      // Note: bT_lam and dc_x are computed from the solve at the current τ,
      // so they implicitly depend on τ. But for the identity, we just
      // evaluate it and solve for the τ that makes it hold.
      //
      // Rearranging: θ·R·τ² - (bT_lam + dc_x)·τ - (xQx + μ) = 0
      double a_coeff = theta * R;
      double b_coeff = -(bT_lam + dc_x);
      double c_coeff = -(xQx + mu);

      if (std::abs(a_coeff) > 1e-30) {
        double discr = b_coeff * b_coeff - 4 * a_coeff * c_coeff;
        if (discr >= 0) {
          double sq = std::sqrt(discr);
          double t1 = (-b_coeff + sq) / (2 * a_coeff);
          double t2 = (-b_coeff - sq) / (2 * a_coeff);
          // Pick positive root closest to current tau.
          double tau_new = tau;
          if (t1 > 0 && t2 > 0)
            tau_new = (std::abs(t1 - tau) < std::abs(t2 - tau)) ? t1 : t2;
          else if (t1 > 0) tau_new = t1;
          else if (t2 > 0) tau_new = t2;
          tau = tau_new;
        }
      }
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
