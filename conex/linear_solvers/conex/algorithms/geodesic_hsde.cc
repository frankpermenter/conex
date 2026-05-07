#include "conex/algorithms/geodesic_hsde.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/tree_solver/kkt_tree_solver.h"

#include <cmath>
#include <cstdio>

namespace conex {

using EuclideanJordanAlgebra::addScaled;
using EuclideanJordanAlgebra::dot;
using EuclideanJordanAlgebra::lineSearchK;
using EuclideanJordanAlgebra::normInf;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::barrierParameter;
using EuclideanJordanAlgebra::geodesicUpdate;

// Precomputed k-independent coefficients for the (tau, theta) selection.
struct HSDECoeffs {
  // Normalization: N0(k) + N1*tau + Nth*theta = -alpha_norm.
  // N0 depends on k: N0 = rpTl0(k) + rdTx0(k).
  double rpTl1, rpTlth, rdTx1, rdTxth, rg;
  double N1, Nth;

  // Gap equation inner products (k-independent parts).
  double bTl1, bTlth;
  double cTx1, cTxth;

  // k-dependent raw dot products (unscaled by 1/k).
  double bTl0_raw;   // b^T P(sqrtW)(e + d0)  (multiply by 1/k to get bTl0)
  double rpTl0_raw;  // rp^T P(sqrtW)(e + d0) (multiply by 1/k)
  double cTx0_raw;   // c^T y0                (multiply by 1/k)
  double rdTx0_raw;  // (c^T y0 - e^T Ax0)    (multiply by 1/k)

  // Q inner products (h = y1_0 - n1*y1_theta is k-independent).
  double qhh;  // h^T Q h (k-independent)

  double alpha_norm, R;
  double w_tau, r_tau;
  bool skip_Q;

  // For Q terms depending on k: f0 = y0/k - e1*y1_theta.
  // We store y0 and y1_theta to recompute f0 at each k.
  Eigen::VectorXd y0, y1_theta, y1_0;
};

// Evaluate (tau, theta, d_inf, d_tau, gap) at a candidate k.
struct HSDEEval {
  double tau, theta, d_inf, d_tau, gap, mu;
};

static HSDEEval EvalAtK(
    const NewtonDecomposition& decomp,
    const HSDECoeffs& c,
    double k) {
  double mu = 1.0 / (k * k);

  // k-dependent normalization coefficient.
  double N0 = c.rpTl0_raw / k + c.rdTx0_raw / k;
  double eta = c.alpha_norm + N0;
  double Nth_threshold = 1e-12 * (std::abs(N0) + std::abs(c.N1) + 1.0);
  double n1 = (std::abs(c.Nth) > Nth_threshold) ? c.N1 / c.Nth : 0.0;
  double e1 = (std::abs(c.Nth) > Nth_threshold) ? eta / c.Nth : 0.0;

  // Gap equation coefficients after theta elimination.
  double bTl0 = c.bTl0_raw / k;
  double cTx0 = c.cTx0_raw / k;
  double bTl0_sub = bTl0 - e1 * c.bTlth;
  double bTl1_sub = c.bTl1 - n1 * c.bTlth;
  double cTf0 = cTx0 - e1 * c.cTxth;
  double cTh = c.cTx1 - n1 * c.cTxth;

  // Q terms.
  double qff = 0, qfh = 0, qhh = c.qhh;
  if (!c.skip_Q) {
    // f0 = y0/k - e1*y1_theta,  h = y1_0 - n1*y1_theta
    // These need full recomputation — expensive for large problems.
    // For now, skip Q (works for LP/SDP without quadratic cost).
    // TODO: cache Q*y0, Q*y1_0, Q*y1_theta and use dot products.
  }

  double inv_wt = 1.0 / (c.w_tau > 1e-30 ? c.w_tau : 1e-30);
  double A_coeff = bTl1_sub + cTh + qhh + c.R * n1 - inv_wt * inv_wt;
  double B_coeff = bTl0_sub + cTf0 + 2 * qfh + c.R * e1 + 2 * c.r_tau * inv_wt;
  double C_coeff = qff;

  // Solve quadratic for tau.
  double tau = -1;
  double discr = B_coeff * B_coeff - 4.0 * A_coeff * C_coeff;
  if (discr >= 0 && std::abs(A_coeff) > 1e-30) {
    double sq = std::sqrt(discr);
    double t1 = (-B_coeff + sq) / (2.0 * A_coeff);
    double t2 = (-B_coeff - sq) / (2.0 * A_coeff);
    double wtr = c.w_tau * c.r_tau;
    double dt1 = (std::abs(wtr) > 1e-30) ? t1 / wtr - 1.0 : 1e30;
    double dt2 = (std::abs(wtr) > 1e-30) ? t2 / wtr - 1.0 : 1e30;
    if (t1 > 0 && !(t2 > 0))
      tau = t1;
    else if (t2 > 0 && !(t1 > 0))
      tau = t2;
    else
      tau = (std::abs(dt1) < std::abs(dt2)) ? t1 : t2;
  }
  if (tau <= 0) return {-1, 0, 1e30, 0, 0, mu};

  double d_tau = (std::abs(c.w_tau * c.r_tau) > 1e-30)
               ? tau / (c.w_tau * c.r_tau) - 1.0 : 0.0;

  // Recover theta from normalization.
  double theta = 0;
  if (std::abs(c.Nth) > Nth_threshold) {
    theta = (-c.alpha_norm - N0 - c.N1 * tau) / c.Nth;
  }

  // Evaluate direction.
  RowSpace d = EvaluateDirection(decomp, k, tau, theta);
  double d_inf = std::max(normInf(d), std::abs(d_tau));
  double d_sq = squaredNorm(d);
  double nu = static_cast<double>(d.total_rows());  // approximate
  double gap = mu * (nu - d_sq);

  return {tau, theta, d_inf, d_tau, gap, mu};
}

GeodesicResult SolveGeodesicHSDE(
    CompiledModel& model,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  const auto& cost_rhs = model.cost_rhs();
  RowSpace b = model.GetAffineTerm();
  RowSpace ones = model.MakeRowSpace();
  setOnes(ones);
  const double nu = barrierParameter(W);
  const double bT_ones = dot(b, ones);
  const double R = bT_ones + 1.0;
  const double alpha_norm = nu + 1.0;
  const bool skip_Q = !model.has_quadratic_cost();

  auto duality_cost = model.MakeSolverRHS();
  duality_cost = cost_rhs;
  auto* ts_init = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts_init && !ts_init->equality_sub_assemblers().empty()) {
    duality_cost += ts_init->EqualityAffineTermRHS();
  }

  double k = 1.0;
  double theta = 1.0;
  double tau = 1.0;
  double w_tau = 1.0;
  double r_tau = 1.0;

  model.SetScaling(W);
  model.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};

  if (verbose) {
    printf("  %3s  %10s  %10s  %10s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s  %3s\n",
           "it", "theta", "tau", "k", "d_inf", "d_tau",
           "gap", "dual", "primal", "mu/tau", "eq_err", "st");
    printf("  %s\n", std::string(140, '-').c_str());
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    auto decomp = ComputeFullDecomposition(model, b, W);
    total_sol += 3;

    // Precompute k-independent coefficients.
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace rp = addScaled(b, ones, 1.0, -1.0);

    // Lambda components (k-independent parts).
    RowSpace pw_ed0 = quadraticRepresentation(sqrtW, ones + decomp.d0);
    RowSpace pw_d1_0 = quadraticRepresentation(sqrtW, decomp.d1_0);
    RowSpace pw_d1_th = quadraticRepresentation(sqrtW, decomp.d1_theta);

    auto x0_rhs = model.MakeSolverRHS();
    x0_rhs = model.MakeBlockVariable(decomp.y0);
    auto x1_rhs = model.MakeSolverRHS();
    x1_rhs = model.MakeBlockVariable(decomp.y1_0);
    auto xth_rhs = model.MakeSolverRHS();
    xth_rhs = model.MakeBlockVariable(decomp.y1_theta);

    RowSpace Ax0_v = model.MakeRowSpace(); model.MultiplyA(x0_rhs, Ax0_v);
    RowSpace Ax1_v = model.MakeRowSpace(); model.MultiplyA(x1_rhs, Ax1_v);
    RowSpace Axth_v = model.MakeRowSpace(); model.MultiplyA(xth_rhs, Axth_v);

    HSDECoeffs coeff;
    coeff.bTl0_raw = dot(b, pw_ed0);
    coeff.bTl1 = dot(b, pw_d1_0);
    coeff.bTlth = dot(b, pw_d1_th);
    coeff.cTx0_raw = duality_cost.dot(x0_rhs);
    coeff.cTx1 = duality_cost.dot(x1_rhs);
    coeff.cTxth = duality_cost.dot(xth_rhs);

    coeff.rpTl0_raw = dot(rp, pw_ed0);
    coeff.rpTl1 = dot(rp, pw_d1_0);
    coeff.rpTlth = dot(rp, pw_d1_th);
    coeff.rdTx0_raw = coeff.cTx0_raw - dot(ones, Ax0_v);
    coeff.rdTx1 = coeff.cTx1 - dot(ones, Ax1_v);
    coeff.rdTxth = coeff.cTxth - dot(ones, Axth_v);
    coeff.rg = -(bT_ones + 1.0);

    coeff.N1 = coeff.rpTl1 + coeff.rdTx1 + coeff.rg;
    coeff.Nth = coeff.rpTlth + coeff.rdTxth;

    coeff.alpha_norm = alpha_norm;
    coeff.R = R;
    coeff.w_tau = w_tau;
    coeff.r_tau = r_tau;
    coeff.skip_Q = skip_Q;
    coeff.qhh = 0;  // TODO: handle Q != 0
    coeff.y0 = decomp.y0;
    coeff.y1_0 = decomp.y1_0;
    coeff.y1_theta = decomp.y1_theta;

    // Evaluate at current k.
    auto ev = EvalAtK(decomp, coeff, k);
    tau = ev.tau;
    theta = ev.theta;
    double d_inf = ev.d_inf;
    double d_tau = ev.d_tau;
    double gap = ev.gap;
    double mu = ev.mu;

    // Override gap with correct nu.
    RowSpace d = EvaluateDirection(decomp, k, tau, theta);
    double d_sq = squaredNorm(d);
    gap = mu * (nu - d_sq);

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = iter + 1;

    if (verbose) {
      RowSpace lam_v = quadraticRepresentation(sqrtW, ones + d);
      lam_v *= (1.0 / k);
      double bTl = dot(b, lam_v);
      Eigen::VectorXd x_vec = decomp.y0 / k + tau * decomp.y1_0
                             + theta * decomp.y1_theta;
      auto x_rhs_v = model.MakeSolverRHS();
      x_rhs_v = model.MakeBlockVariable(x_vec);
      double cTx = duality_cost.dot(x_rhs_v);
      auto qx = model.MakeSolverRHS(); qx.SetZero();
      model.AccumulateQx(x_rhs_v, qx);
      double xQx = qx.dot(x_rhs_v);
      double kappa_v = r_tau * (1.0 - d_tau) / std::max(w_tau, 1e-30);
      double xQx_tau = (std::abs(tau) > 1e-30) ? xQx / tau : 0.0;
      double eq_err = std::abs(bTl + cTx + xQx_tau + kappa_v - theta * R);
      double mu_tau = (std::abs(tau) > 1e-30) ? mu / tau : 0.0;
      double half_xQx_phys = (std::abs(tau) > 1e-30)
          ? 0.5 * xQx / (tau * tau) : 0.0;
      double primal_phys = (std::abs(tau) > 1e-30) ? cTx / tau + half_xQx_phys : 0.0;
      double dual_phys = (std::abs(tau) > 1e-30)
          ? -(bTl / tau + half_xQx_phys) : 0.0;

      printf("  %3d  %10.2e  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %12.2e  %3s\n",
             iter, theta, tau, k, d_inf, d_tau, gap,
             dual_phys, primal_phys, mu_tau, eq_err,
             (gap < 0) ? "W" : "k");
    }

    if (!std::isfinite(d_inf) || !std::isfinite(gap)) {
      if (verbose) printf("  TERMINATED: nan\n");
      break;
    }

    if (std::abs(theta) < tolerance && std::abs(gap) < tolerance && d_inf <= 1.001)
      break;

    if (gap < 0) {
      // W-update: geodesic step + refactor.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d);
      w_tau *= std::exp(d_tau * alpha);
      model.SetScaling(W);
      if (!model.AssembleAndFactor()) break;
      total_fac++;
    } else {
      // k-update: binary search for largest k with max(d_inf, |d_tau|) <= 1.
      double k_lo = k, k_hi = k;
      // First find an upper bound by doubling.
      for (int i = 0; i < 50; ++i) {
        k_hi *= 2.0;
        auto ev_hi = EvalAtK(decomp, coeff, k_hi);
        if (ev_hi.tau <= 0 || ev_hi.d_inf > 1.0) break;
      }
      // Binary search.
      for (int bisect = 0; bisect < 50; ++bisect) {
        double k_mid = 0.5 * (k_lo + k_hi);
        auto ev_mid = EvalAtK(decomp, coeff, k_mid);
        if (ev_mid.tau > 0 && ev_mid.d_inf <= 1.0) {
          k_lo = k_mid;
        } else {
          k_hi = k_mid;
        }
      }
      if (k_lo > k) k = k_lo;
      // Shrink r_tau.
      r_tau = 0.5 * r_tau * (1.0 + std::abs(d_tau));
    }
  }

  result.mu = 1.0 / (k * k);
  result.complementarity = result.iter_stats.empty() ? 0 :
      result.iter_stats.back().complementarity;
  result.tau = tau;
  result.kappa = (tau > 1e-30) ? theta / tau
      : std::numeric_limits<double>::infinity();
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x and lambda.
  {
    auto decomp = ComputeFullDecomposition(model, b, W);
    Eigen::VectorXd x_lifted = decomp.y0 / k + tau * decomp.y1_0
                               + theta * decomp.y1_theta;
    result.x = x_lifted / tau;

    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones_v = model.MakeRowSpace();
    setOnes(ones_v);
    RowSpace d_cur = EvaluateDirection(decomp, k, tau, theta);
    RowSpace lambda = quadraticRepresentation(sqrtW, ones_v + d_cur);
    lambda *= (1.0 / (k * tau));
    result.lambda = lambda;

    auto x_rhs = model.MakeSolverRHS();
    x_rhs = model.MakeBlockVariable(result.x);
    result.optimality = CheckOptimality(model, x_rhs, lambda);
    result.optimality.mu = result.mu;

    if (verbose) {
      printf("  Optimality: compl=%.2e, min_s=%.2e, min_lam=%.2e\n",
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  return result;
}

}  // namespace conex
