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
  const double alpha_norm = nu + 1.0;  // <e,e> + 1
  const bool skip_Q = !model.has_quadratic_cost();

  // Duality cost (cost_rhs + equality dual correction).
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
    // 3-solve decomposition at current W (standard, not r-parameterized).
    auto decomp = ComputeFullDecomposition(model, b, W);
    total_sol += 3;

    // Joint (tau, theta) selection from gap + normalization.
    // Same algebra as ThetaContR but with r = e/k (scalar).
    //
    // lambda = (1/k)*P(W^{1/2})(e + d0 + k*tau*d1_0 + k*theta*d1_theta)
    //        = P(W^{1/2})(e + d0)/k + tau*P(W^{1/2})(d1_0) + theta*P(W^{1/2})(d1_theta)
    //
    // x = y0/k + tau*y1_0 + theta*y1_theta
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    // Lambda components.
    RowSpace lam0 = quadraticRepresentation(sqrtW, ones + decomp.d0);
    lam0 *= (1.0 / k);
    RowSpace lam1 = quadraticRepresentation(sqrtW, decomp.d1_0);
    RowSpace lam_theta = quadraticRepresentation(sqrtW, decomp.d1_theta);

    auto x0_rhs = model.MakeSolverRHS();
    x0_rhs = model.MakeBlockVariable(decomp.y0);
    auto x1_rhs = model.MakeSolverRHS();
    x1_rhs = model.MakeBlockVariable(decomp.y1_0);
    auto xth_rhs = model.MakeSolverRHS();
    xth_rhs = model.MakeBlockVariable(decomp.y1_theta);

    double bTl0 = dot(b, lam0);
    double bTl1 = dot(b, lam1);
    double bTlth = dot(b, lam_theta);
    double cTx0 = duality_cost.dot(x0_rhs) / k;
    double cTx1 = duality_cost.dot(x1_rhs);
    double cTxth = duality_cost.dot(xth_rhs);

    // Normalization coefficients.
    RowSpace rp = addScaled(b, ones, 1.0, -1.0);  // b - e
    RowSpace Ax0_v = model.MakeRowSpace(); model.MultiplyA(x0_rhs, Ax0_v);
    RowSpace Ax1_v = model.MakeRowSpace(); model.MultiplyA(x1_rhs, Ax1_v);
    RowSpace Axth_v = model.MakeRowSpace(); model.MultiplyA(xth_rhs, Axth_v);

    double rpTl0 = dot(rp, lam0);
    double rpTl1 = dot(rp, lam1);
    double rpTlth = dot(rp, lam_theta);
    double rdTx0 = cTx0 - dot(ones, Ax0_v) / k;
    double rdTx1 = cTx1 - dot(ones, Ax1_v);
    double rdTxth = cTxth - dot(ones, Axth_v);
    double rg = -(bT_ones + 1.0);

    double N0 = rpTl0 + rdTx0;
    double N1 = rpTl1 + rdTx1 + rg;
    double Nth = rpTlth + rdTxth;
    double eta = alpha_norm + N0;
    double Nth_threshold = 1e-12 * (std::abs(N0) + std::abs(N1) + 1.0);
    double n1 = (std::abs(Nth) > Nth_threshold) ? N1 / Nth : 0.0;
    double e1 = (std::abs(Nth) > Nth_threshold) ? eta / Nth : 0.0;

    // Substitute theta(tau) into gap equation.
    // x(tau) = (y0/k - e1*y1_theta) + tau*(y1_0 - n1*y1_theta) = f0 + tau*h
    Eigen::VectorXd f0_vec = decomp.y0 / k - e1 * decomp.y1_theta;
    Eigen::VectorXd h_vec = decomp.y1_0 - n1 * decomp.y1_theta;

    double qff = 0, qfh = 0, qhh = 0;
    if (!skip_Q) {
      auto f0_rhs = model.MakeSolverRHS(); f0_rhs = model.MakeBlockVariable(f0_vec);
      auto h_rhs = model.MakeSolverRHS(); h_rhs = model.MakeBlockVariable(h_vec);
      auto Qf0 = model.MakeSolverRHS(); Qf0.SetZero(); model.AccumulateQx(f0_rhs, Qf0);
      auto Qh = model.MakeSolverRHS(); Qh.SetZero(); model.AccumulateQx(h_rhs, Qh);
      qff = Qf0.dot(f0_rhs);
      qfh = Qf0.dot(h_rhs);
      qhh = Qh.dot(h_rhs);
    }

    double bTl0_sub = bTl0 - e1 * bTlth;
    double bTl1_sub = bTl1 - n1 * bTlth;
    double cTf0 = cTx0 - e1 * cTxth;
    double cTh = cTx1 - n1 * cTxth;

    // mu = 1/k^2.  Complementarity: mu*(nu - d_sq) + tau*kappa = theta*alpha.
    double mu = 1.0 / (k * k);

    // tau*kappa = 2*r_tau*tau/w_tau - tau^2/w_tau^2.
    double inv_wt = 1.0 / (w_tau > 1e-30 ? w_tau : 1e-30);
    double A_coeff = bTl1_sub + cTh + qhh + R * n1 - inv_wt * inv_wt;
    double B_coeff = bTl0_sub + cTf0 + 2 * qfh + R * e1 + 2 * r_tau * inv_wt;
    double C_coeff = qff;

    // Solve quadratic for tau.
    double tau_new = tau;
    double discr = B_coeff * B_coeff - 4.0 * A_coeff * C_coeff;
    if (discr >= 0 && std::abs(A_coeff) > 1e-30) {
      double sq = std::sqrt(discr);
      double t1 = (-B_coeff + sq) / (2.0 * A_coeff);
      double t2 = (-B_coeff - sq) / (2.0 * A_coeff);
      double wtr = w_tau * r_tau;
      double d1_t = (std::abs(wtr) > 1e-30) ? t1 / wtr - 1.0 : 1e30;
      double d2_t = (std::abs(wtr) > 1e-30) ? t2 / wtr - 1.0 : 1e30;
      if (t1 > 0 && !( t2 > 0))
        tau_new = t1;
      else if (t2 > 0 && !(t1 > 0))
        tau_new = t2;
      else
        tau_new = (std::abs(d1_t) < std::abs(d2_t)) ? t1 : t2;
    }
    tau = tau_new;

    double d_tau = (std::abs(w_tau * r_tau) > 1e-30)
                 ? tau / (w_tau * r_tau) - 1.0 : 0.0;

    // Recover theta from normalization.
    if (std::abs(Nth) > Nth_threshold) {
      theta = (-alpha_norm - N0 - N1 * tau) / Nth;
    }

    // Evaluate direction at (k, tau, theta).
    RowSpace d = EvaluateDirection(decomp, k, tau, theta);
    double d_inf = std::max(normInf(d), std::abs(d_tau));
    double d_sq = squaredNorm(d);
    double gap = mu * (nu - d_sq);

    // Record stats.
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
      // k-update: line search for largest k with ||d0 + k*(tau*d1_0 + theta*d1_theta)||_inf <= 1.
      RowSpace d1_combined = addScaled(decomp.d1_0, decomp.d1_theta, tau, theta);
      double k_new = lineSearchK(decomp.d0, d1_combined);
      if (k_new > k) k = k_new;
      // Shrink r_tau (same as ThetaContR).
      r_tau = 0.5 * r_tau * (1.0 + std::abs(d_tau));
    }
  }

  result.d_inf_norm = normInf(EvaluateDirection(
      ComputeFullDecomposition(model, b, W), k, tau, theta));
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
