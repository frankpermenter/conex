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

// Precomputed coefficients for the 2x2 (d_tau, theta) linear system.
// All are k-independent except terms marked _raw (divide by k to use).
struct HSDECoeffs {
  // Gap: G_dtau * d_tau + G_theta * theta + G_0(k) = 0
  double G_dtau;   // wt*rt*(bTl1 + cTx1) - rt/wt
  double G_theta;  // bTlth + cTxth - R
  double G_0_raw;  // bTl0_raw + cTx0_raw  (divide by k)
  double G_0_const; // wt*rt*(bTl1 + cTx1) + rt/wt

  // Norm: N_dtau * d_tau + N_theta * theta + N_0(k) = 0
  double N_dtau;   // wt*rt*(rpTl1 + rdTx1 + rg)
  double N_theta;  // rpTlth + rdTxth
  double N_0_raw;  // rpTl0_raw + rdTx0_raw  (divide by k)
  double N_0_const; // wt*rt*(rpTl1 + rdTx1 + rg) + alpha_norm

  double w_tau, r_tau;
  double alpha_norm, R;
  double nu;
};

// Solve the 2x2 system for (d_tau, theta) at fixed k.
// Returns (d_tau, theta, tau).
struct DTauTheta {
  double d_tau, theta, tau;
  bool valid;
};

static DTauTheta SolveDTauTheta(const HSDECoeffs& c, double k) {
  // Gap:  G_dtau * d_tau + G_theta * theta = -(G_0_raw/k + G_0_const)
  // Norm: N_dtau * d_tau + N_theta * theta = -(N_0_raw/k + N_0_const)
  double g_rhs = -(c.G_0_raw / k + c.G_0_const);
  double n_rhs = -(c.N_0_raw / k + c.N_0_const);

  double det = c.G_dtau * c.N_theta - c.G_theta * c.N_dtau;
  if (std::abs(det) < 1e-30) return {0, 0, 0, false};

  double d_tau = (c.N_theta * g_rhs - c.G_theta * n_rhs) / det;
  double theta = (c.G_dtau * n_rhs - c.N_dtau * g_rhs) / det;
  double tau = c.w_tau * c.r_tau * (1.0 + d_tau);

  return {d_tau, theta, tau, tau > 0};
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

  auto duality_cost = model.MakeSolverRHS();
  duality_cost = cost_rhs;
  auto* ts_init = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
  if (ts_init && !ts_init->equality_sub_assemblers().empty()) {
    duality_cost += ts_init->EqualityAffineTermRHS();
  }

  double k = 1.0;
  double w_tau = 1.0;
  double r_tau = 1.0;

  int total_fac = 0;
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
    // ComputeFullDecomposition calls SetScaling + AssembleAndFactor + 3 solves.
    // The end-of-loop also calls SetScaling + AssembleAndFactor after the step,
    // but this is redundant with the next iteration's ComputeFullDecomposition.
    // So we count 1 factorization per iteration here.
    auto decomp = ComputeFullDecomposition(model, b, W);
    total_sol += 3;

    // Precompute k-independent coefficients for the 2x2 system.
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace rp = addScaled(b, ones, 1.0, -1.0);  // b - e

    // Lambda/x components (all k-independent).
    RowSpace pw_d1_0 = quadraticRepresentation(sqrtW, decomp.d1_0);
    RowSpace pw_d1_th = quadraticRepresentation(sqrtW, decomp.d1_theta);
    RowSpace pw_ed0 = quadraticRepresentation(sqrtW, ones + decomp.d0);

    auto x0_rhs = model.MakeSolverRHS();
    x0_rhs = model.MakeBlockVariable(decomp.y0);
    auto x1_rhs = model.MakeSolverRHS();
    x1_rhs = model.MakeBlockVariable(decomp.y1_0);
    auto xth_rhs = model.MakeSolverRHS();
    xth_rhs = model.MakeBlockVariable(decomp.y1_theta);

    RowSpace Ax0 = model.MakeRowSpace(); model.MultiplyA(x0_rhs, Ax0);
    RowSpace Ax1 = model.MakeRowSpace(); model.MultiplyA(x1_rhs, Ax1);
    RowSpace Axth = model.MakeRowSpace(); model.MultiplyA(xth_rhs, Axth);

    double bTl1 = dot(b, pw_d1_0);
    double bTlth = dot(b, pw_d1_th);
    double bTl0_raw = dot(b, pw_ed0);  // multiply by 1/k
    double cTx1 = duality_cost.dot(x1_rhs);
    double cTxth = duality_cost.dot(xth_rhs);
    double cTx0_raw = duality_cost.dot(x0_rhs);  // multiply by 1/k

    double rpTl1 = dot(rp, pw_d1_0);
    double rpTlth = dot(rp, pw_d1_th);
    double rpTl0_raw = dot(rp, pw_ed0);
    double rdTx1 = cTx1 - dot(ones, Ax1);
    double rdTxth = cTxth - dot(ones, Axth);
    double rdTx0_raw = cTx0_raw - dot(ones, Ax0);
    double rg = -(bT_ones + 1.0);

    double wt = w_tau, rt = r_tau;

    // Build 2x2 coefficients.
    // Gap: b'lam + c'x + kappa = theta*R
    // lam = lam0_raw/k + wt*rt*(1+d_tau)*lam1 + theta*lam_theta
    // x = y0/k + wt*rt*(1+d_tau)*y1 + theta*y_theta
    // kappa = rt*(1-d_tau)/wt
    //
    // Collecting d_tau: wt*rt*(bTl1 + cTx1) - rt/wt
    // Collecting theta: bTlth + cTxth - R
    // Constant (k-dep): (bTl0_raw + cTx0_raw)/k + wt*rt*(bTl1+cTx1) + rt/wt
    HSDECoeffs coeff;
    coeff.G_dtau = wt * rt * (bTl1 + cTx1) - rt / wt;
    coeff.G_theta = bTlth + cTxth - R;
    coeff.G_0_raw = bTl0_raw + cTx0_raw;
    coeff.G_0_const = wt * rt * (bTl1 + cTx1) + rt / wt;

    // Norm: rp'lam + rd'x + rg*tau = -alpha
    // tau = wt*rt*(1+d_tau)
    // Collecting d_tau: wt*rt*(rpTl1 + rdTx1 + rg)
    // Collecting theta: rpTlth + rdTxth
    // Constant (k-dep): (rpTl0_raw + rdTx0_raw)/k + wt*rt*(rpTl1+rdTx1+rg) + alpha
    coeff.N_dtau = wt * rt * (rpTl1 + rdTx1 + rg);
    coeff.N_theta = rpTlth + rdTxth;
    coeff.N_0_raw = rpTl0_raw + rdTx0_raw;
    coeff.N_0_const = wt * rt * (rpTl1 + rdTx1 + rg) + alpha_norm;

    coeff.w_tau = wt;
    coeff.r_tau = rt;
    coeff.alpha_norm = alpha_norm;
    coeff.R = R;
    coeff.nu = nu;

    // Solve 2x2 at current k.
    auto sel = SolveDTauTheta(coeff, k);
    double tau = sel.tau;
    double theta = sel.theta;
    double d_tau = sel.d_tau;

    RowSpace d = EvaluateDirection(decomp, k, tau, theta);
    double d_inf = std::max(normInf(d), std::abs(d_tau));
    double d_sq = squaredNorm(d);
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

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
      double kappa_v = rt * (1.0 - d_tau) / std::max(wt, 1e-30);
      double eq_err = std::abs(bTl + cTx + kappa_v - theta * R);
      double mu_tau = (std::abs(tau) > 1e-30) ? mu / tau : 0.0;
      double primal_phys = (std::abs(tau) > 1e-30) ? cTx / tau : 0.0;
      double dual_phys = (std::abs(tau) > 1e-30) ? -bTl / tau : 0.0;

      // Complementarity check: gap + tau*kappa should = theta*alpha.
      // gap = mu*(nu - d_sq), tau*kappa = rt^2*(1 - d_tau^2).
      double tau_kappa = rt * rt * (1.0 - d_tau * d_tau);
      double compl_lhs = gap + tau_kappa;
      double compl_rhs = theta * alpha_norm;
      double compl_err = std::abs(compl_lhs - compl_rhs);

      printf("  %3d  %10.2e  %10.2e  %10.2e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  eq=%.1e  cpl=%.1e (%.2e vs %.2e)\n",
             iter, theta, tau, k, d_inf, d_tau, gap,
             dual_phys, primal_phys, mu_tau, eq_err,
             compl_err, compl_lhs, compl_rhs);
    }

    if (!std::isfinite(d_inf) || !std::isfinite(gap)) {
      if (verbose) printf("  TERMINATED: nan\n");
      break;
    }

    if (std::abs(gap) < tolerance && d_inf <= 1.001)
      break;

    // Line search for k: d(k) is affine in k.
    // Evaluate at two k values, extract D0 + k*D1, use lineSearchK.
    {
      double ka = k, kb = k + 1.0;
      auto sa = SolveDTauTheta(coeff, ka);
      auto sb = SolveDTauTheta(coeff, kb);
      if (sa.valid && sb.valid) {
        RowSpace da = EvaluateDirection(decomp, ka, sa.tau, sa.theta);
        RowSpace db = EvaluateDirection(decomp, kb, sb.tau, sb.theta);
        RowSpace D1 = addScaled(db, da, 1.0, -1.0);
        RowSpace D0 = addScaled(da, D1, 1.0, -ka);
        double k_new = lineSearchK(D0, D1);

        // Clamp for d_tau constraint.
        if (k_new > k) {
          auto ev = SolveDTauTheta(coeff, k_new);
          if (!ev.valid || std::abs(ev.d_tau) > 1.0) {
            double lo = k, hi = k_new;
            for (int bs = 0; bs < 50; ++bs) {
              double mid = 0.5 * (lo + hi);
              auto evm = SolveDTauTheta(coeff, mid);
              if (evm.valid && std::abs(evm.d_tau) <= 1.0)
                lo = mid;
              else
                hi = mid;
            }
            k_new = lo;
          }
        }

        if (k_new > k) {
          k = k_new;
          auto ev = SolveDTauTheta(coeff, k);
          tau = ev.tau;
          theta = ev.theta;
          d_tau = ev.d_tau;
          d = EvaluateDirection(decomp, k, tau, theta);
          d_inf = std::max(normInf(d), std::abs(d_tau));
        }
      }
    }

    // Geodesic step (refactor happens at top of next iteration via
    // ComputeFullDecomposition).
    {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d);
      w_tau *= std::exp(d_tau * alpha);
      r_tau = 0.5 * r_tau * (1.0 + std::abs(d_tau));
      total_fac++;  // count the factorization that will happen next iteration
    }
  }

  result.mu = 1.0 / (k * k);
  result.complementarity = result.iter_stats.empty() ? 0 :
      result.iter_stats.back().complementarity;
  result.tau = w_tau * r_tau;  // approximate
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x and lambda.
  {
    auto sel = SolveDTauTheta(
        HSDECoeffs{}, k);  // need to recompute — use decomp directly
    auto decomp = ComputeFullDecomposition(model, b, W);
    // Re-solve for (tau, theta) at final k — recompute coefficients.
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace pw_d1_0 = quadraticRepresentation(sqrtW, decomp.d1_0);
    RowSpace pw_d1_th = quadraticRepresentation(sqrtW, decomp.d1_theta);
    RowSpace pw_ed0 = quadraticRepresentation(sqrtW, ones + decomp.d0);
    auto x0_rhs = model.MakeSolverRHS();
    x0_rhs = model.MakeBlockVariable(decomp.y0);
    auto x1_rhs = model.MakeSolverRHS();
    x1_rhs = model.MakeBlockVariable(decomp.y1_0);
    auto xth_rhs = model.MakeSolverRHS();
    xth_rhs = model.MakeBlockVariable(decomp.y1_theta);
    RowSpace rp = addScaled(b, ones, 1.0, -1.0);
    RowSpace Ax0 = model.MakeRowSpace(); model.MultiplyA(x0_rhs, Ax0);
    RowSpace Ax1 = model.MakeRowSpace(); model.MultiplyA(x1_rhs, Ax1);
    RowSpace Axth = model.MakeRowSpace(); model.MultiplyA(xth_rhs, Axth);

    double bTl1 = dot(b, pw_d1_0), bTlth = dot(b, pw_d1_th);
    double bTl0_raw = dot(b, pw_ed0);
    double cTx1 = duality_cost.dot(x1_rhs), cTxth = duality_cost.dot(xth_rhs);
    double cTx0_raw = duality_cost.dot(x0_rhs);
    double rpTl1 = dot(rp, pw_d1_0), rpTlth = dot(rp, pw_d1_th);
    double rpTl0_raw = dot(rp, pw_ed0);
    double rdTx1 = cTx1 - dot(ones, Ax1), rdTxth = cTxth - dot(ones, Axth);
    double rdTx0_raw = cTx0_raw - dot(ones, Ax0);
    double rg = -(bT_ones + 1.0);
    double wt = w_tau, rt = r_tau;

    HSDECoeffs coeff;
    coeff.G_dtau = wt*rt*(bTl1+cTx1) - rt/wt;
    coeff.G_theta = bTlth + cTxth - R;
    coeff.G_0_raw = bTl0_raw + cTx0_raw;
    coeff.G_0_const = wt*rt*(bTl1+cTx1) + rt/wt;
    coeff.N_dtau = wt*rt*(rpTl1+rdTx1+rg);
    coeff.N_theta = rpTlth + rdTxth;
    coeff.N_0_raw = rpTl0_raw + rdTx0_raw;
    coeff.N_0_const = wt*rt*(rpTl1+rdTx1+rg) + alpha_norm;
    coeff.w_tau = wt; coeff.r_tau = rt;
    coeff.alpha_norm = alpha_norm; coeff.R = R; coeff.nu = nu;

    auto final_sel = SolveDTauTheta(coeff, k);
    double tau = final_sel.tau;
    double theta = final_sel.theta;

    Eigen::VectorXd x_lifted = decomp.y0 / k + tau * decomp.y1_0
                               + theta * decomp.y1_theta;
    result.x = x_lifted / tau;
    result.tau = tau;

    RowSpace d_cur = EvaluateDirection(decomp, k, tau, theta);
    RowSpace lambda = quadraticRepresentation(sqrtW, ones + d_cur);
    lambda *= (1.0 / (k * tau));
    result.lambda = lambda;

    auto x_rhs_final = model.MakeSolverRHS();
    x_rhs_final = model.MakeBlockVariable(result.x);
    result.optimality = CheckOptimality(model, x_rhs_final, lambda);
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
