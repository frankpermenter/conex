#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/geodesic_ipm_helpers.h"
#include "conex/common/eja_ops.h"
#include "conex/common/solve_stats.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include <cmath>
#include <cstdio>

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

// Forward declaration (defined later in this file).
static std::pair<double, double> EvalKCandidate(
    CompiledModel& model, const SolverRHS& duality_cost,
    const RowSpace& b, const RowSpace& W,
    const NewtonDecomposition& decomp, double bT_ones,
    double theta_val, double k_cand);

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
    ArenaFrame outer_frame(arena);
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

    // Precompute theta-independent quantities (one sqrt(W), one P(W), dot products).
    auto tc = PrecomputeThetaCoeffs(model, arena, duality_cost, b, W, decomp, bT_ones);
    double last_eq_err = 0;

    // Line search for k: d(k) is affine in k (for Q=0).
    // Evaluate at two k values, extract D0 + k*D1, use lineSearchK.
    constexpr double beta_target = 1.0;
    double theta_prev = theta;
    {
      ArenaFrame ls_frame(arena);
      double ka = k > 0 ? k : 1.0;
      double kb = ka + 1.0;
      // Solve for (tau, theta) at each k.
      auto [tau_a, dinf_a] = EvalThetaFast(tc, tc.ip, 1.0 / (ka * ka));
      auto [tau_b, dinf_b] = EvalThetaFast(tc, tc.ip, 1.0 / (kb * kb));
      double theta_a = 1.0 / (ka * ka), theta_b = 1.0 / (kb * kb);
      if (tau_a > 0 && tau_b > 0) {
        RowSpace da = model.AllocRowSpace(arena);
        EvaluateDirection(da, decomp, ka, tau_a, theta_a);
        RowSpace db = model.AllocRowSpace(arena);
        EvaluateDirection(db, decomp, kb, tau_b, theta_b);
        // d(k) = D0 + k*D1, extract: D1 = (db - da)/(kb - ka), D0 = da - ka*D1
        RowSpace D1 = model.AllocRowSpace(arena);
        addScaled(D1, db, da, 1.0, -1.0);  // D1 = db - da (since kb - ka = 1)
        RowSpace D0 = model.AllocRowSpace(arena);
        addScaled(D0, da, D1, 1.0, -ka);   // D0 = da - ka*D1
        double k_new = lineSearchK(D0, D1);
        if (k_new > 0) {
          k = k_new;
          theta = 1.0 / (k * k);
          auto [tau_new, dinf_new] = EvalThetaFast(tc, tc.ip, theta);
          tau = tau_new;
        }
      }
      // Fallback: if affine line search failed, try bisection.
      if (tau <= 0 || k <= 0) {
        auto sr = BisectTheta(model, arena, tc, decomp, 0.0, theta_prev, beta_target);
        theta = sr.theta; k = sr.k; tau = sr.tau;
      }
    }
    if (tau <= 0) {
      if (verbose) printf("  TERMINATED: tau <= 0 at iteration %d\n", outer);
      result.iterations = outer + 1;
      break;
    }

    // Evaluate direction for the first step.
    RowSpace d_step = model.AllocRowSpace(arena);
    EvaluateDirection(d_step, decomp, k, tau, theta);
    double d_inf = normInf(d_step);
    double d_sq = squaredNorm(d_step);
    double mu = 1.0 / (k * k);
    double gap = mu * (nu - d_sq);

    if (verbose) {
      last_eq_err = PrintThetaContStats(model, decomp, cost_rhs, duality_cost,
          bT_ones, nu, outer, -1, k, tau, theta, d_inf, d_sq, gap);
    }

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

    RowSpace sqrtW0_f = model.AllocRowSpace(arena);
    EuclideanJordanAlgebra::sqrt(sqrtW0_f, W0);
    auto dc_f = ComputeDualityCoeffs(model, arena, duality_cost, b, W0, decomp);
    double beta_f = dc_f.sigma1 + dc_f.gamma1 + dc_f.q11;

    // Precompute frozen theta-independent terms (bT_P_d1t, Q products on y1_theta).
    // These don't change across inner iterations.
    double frozen_bT_P_d1t = tc.bT_P_d1t;  // from outer precomputation with sqrtW
    // Recompute bT_P_d1t using sqrtW0 (the frozen Jacobian point).
    {
      ArenaFrame mark2_frame(arena);
      RowSpace P_d1t_f = model.AllocRowSpace(arena);
      quadraticRepresentation(P_d1t_f, sqrtW0_f, decomp.d1_theta);
      frozen_bT_P_d1t = dot(b, P_d1t_f);
    }

    for (int inner = 0; inner < max_centering_steps; ++inner) {
      ArenaFrame inner_frame(arena);
      if (refactor_inner) {
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

      // Precompute coefficients for this inner iteration's bisection.
      ThetaCandidateCoeffs tc_f;
      if (refactor_inner) {
        tc_f = PrecomputeThetaCoeffs(model, arena, duality_cost, b, W, decomp, bT_ones);
      } else {
        tc_f = RefreshFrozenThetaCoeffs(model, arena, duality_cost, b, W,
            sqrtW0_f, decomp, bT_ones, frozen_bT_P_d1t, beta_f, tc);
      }

      // Binary search for smallest theta.
      double theta_lo_f = refactor_inner ? 0.0 : theta * 0.1;
      auto sr_f = BisectTheta(model, arena, tc_f, decomp, theta_lo_f, theta, beta_target);
      double theta_f = sr_f.theta, k_f = sr_f.k, tau_f = sr_f.tau;
      if (tau_f <= 0) { break; }

      RowSpace d_f = model.AllocRowSpace(arena);
      EvaluateDirection(d_f, decomp, k_f, tau_f, theta_f);
      double d_inf_fv = normInf(d_f);
      if (verbose) {
        double mu_f = 1.0 / (k_f * k_f);
        double gap_f = mu_f * (nu - squaredNorm(d_f));
        last_eq_err = PrintThetaContStats(model, decomp, cost_rhs, duality_cost,
            bT_ones, nu, outer, inner, k_f, tau_f, theta_f,
            d_inf_fv, squaredNorm(d_f), gap_f);
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
      RowSpace d_final = model.AllocRowSpace(arena);
      EvaluateDirection(d_final, decomp, k, tau, theta);
      d_inf = normInf(d_final);
      d_sq = squaredNorm(d_final);
      gap = mu * (nu - d_sq);
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

    // Recover x = (y0/k + tau*y1_0 + theta*y1_theta) / tau.
    {
      auto x_rhs = model.AllocSolverRHS();
      x_rhs.SetZero();
      x_rhs.AddScaled(1.0 / k, decomp.y0);
      x_rhs.AddScaled(tau, decomp.y1_0);
      x_rhs.AddScaled(theta, decomp.y1_theta);
      if (tau > 1e-6) x_rhs *= (1.0 / tau);
      int nr = model.number_of_variables();
      result.x.resize(nr);
      { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr); x_rhs.supernodes->GatherInto(xm); }
    }

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
  }

  // Infeasibility detection: tau→0.
  const double tau_tol = 1e-6;
  result.infeasible = (tau < tau_tol);

  // Recover lambda and optimality (requires re-factorization for decomp).
  if (k > 0 && tau > tau_tol && result.x.size() > 0) {
    NewtonDecomposition decomp_r;
    decomp_r.d0 = model.AllocRowSpace(arena);
    decomp_r.d1_0 = model.AllocRowSpace(arena);
    decomp_r.d1_theta = model.AllocRowSpace(arena);
    ComputeFullDecomposition(model, b, W, decomp_r);

    RowSpace d_cur = model.AllocRowSpace(arena);
    EvaluateDirection(d_cur, decomp_r, k, tau, theta);
    RowSpace sqrtW_r = model.AllocRowSpace(arena);
    EuclideanJordanAlgebra::sqrt(sqrtW_r, W);
    RowSpace ones_r = model.AllocRowSpace(arena);
    setOnes(ones_r);
    RowSpace ones_plus_dcur = model.AllocRowSpace(arena);
    addScaled(ones_plus_dcur, ones_r, d_cur, 1.0, 1.0);
    result.lambda = model.MakeRowSpace();
    quadraticRepresentation(result.lambda, sqrtW_r, ones_plus_dcur);
    result.lambda *= (1.0 / (k * tau));

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
  }

  return result;
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
  ArenaFrame frame(arena);
  double mu = 1.0 / (k_cand * k_cand);

  auto dc = ComputeDualityCoeffs(model, arena, duality_cost, b, W, decomp);
  double beta_coeff = dc.sigma1 + dc.gamma1 + dc.q11;

  RowSpace sqrtW = model.AllocRowSpace(arena);
  EuclideanJordanAlgebra::sqrt(sqrtW, W);
  RowSpace ones_v = model.AllocRowSpace(arena);
  setOnes(ones_v);
  RowSpace e_plus_d0 = model.AllocRowSpace(arena);
  addScaled(e_plus_d0, ones_v, decomp.d0, 1.0, 1.0);
  RowSpace arg = model.AllocRowSpace(arena);
  addScaled(arg, e_plus_d0, decomp.d1_theta, 1.0, k_cand * theta_val);
  RowSpace Parg = model.AllocRowSpace(arena);
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
  if (disc < 0) { return {-1, 1e30}; }

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta_coeff);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta_coeff);

  auto eval_dinf = [&](double tau) -> std::pair<double, double> {
    if (tau <= 0) return {-1, 1e30};
    ArenaFrame eval_frame(arena);
    RowSpace d = model.AllocRowSpace(arena);
    EvaluateDirection(d, decomp, k_cand, tau, theta_val);
    double dinf = normInf(d);
    return {tau, dinf};
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
    ArenaFrame outer_frame(arena);
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

  // Infeasibility detection: tau→0.
  result.infeasible = (tau < 1e-6);

  // Recover primal x.  De-homogenize: x_phys = x_lifted / tau.
  if (k > 0 && tau > 1e-6) {
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

}  // namespace conex
