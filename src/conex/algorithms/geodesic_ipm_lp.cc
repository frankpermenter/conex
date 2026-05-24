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
using EuclideanJordanAlgebra::lineSearchK;
using EuclideanJordanAlgebra::normInf;
using EuclideanJordanAlgebra::quadraticRepresentation;
using EuclideanJordanAlgebra::setOnes;
using EuclideanJordanAlgebra::squaredNorm;
using EuclideanJordanAlgebra::barrierParameter;
using EuclideanJordanAlgebra::geodesicUpdate;
using EuclideanJordanAlgebra::geodesicUpdateFromSlack;

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
    ArenaFrame iter_frame(arena);
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
        { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr); x_rhs.supernodes->GatherInto(xm); }
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
    ArenaFrame iter_frame(arena);

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
      // Duality gap check: compare mu*(nu - ||d||^2) with b'lambda + c'x.
      // lambda = (1/k)*P(sqrt(W))(e+d), x = y0/k + y1.
      RowSpace sqrtW_v = model.AllocRowSpace(arena);
      EuclideanJordanAlgebra::sqrt(sqrtW_v, W);
      RowSpace ed = model.AllocRowSpace(arena);
      setOnes(ed);
      ed += d;
      RowSpace lam_v = model.AllocRowSpace(arena);
      quadraticRepresentation(lam_v, sqrtW_v, ed);
      lam_v *= (1.0 / k);
      double b_lam = dot(model.GetAffineTerm(), lam_v);

      auto x_rhs_v = model.AllocSolverRHS();
      x_rhs_v.SetZero();
      x_rhs_v.AddScaled(1.0 / k, y0);
      x_rhs_v += y1;
      double c_x = cost_rhs.dot(x_rhs_v);

      double qx_gap = 0;
      if (model.has_quadratic_cost()) {
        auto qx_v = model.AllocSolverRHS();
        qx_v.SetZero();
        model.AccumulateQx(x_rhs_v, qx_v);
        qx_gap = model.dot(x_rhs_v, qx_v);
      }

      // Equality constraint contribution: d'ν.
      double d_nu = 0;
      auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&model.kkt());
      if (ts && !ts->equality_sub_assemblers().empty()) {
        auto d_rhs = ts->EqualityAffineTermRHS();
        d_nu = model.dot(d_rhs, x_rhs_v);
      }

      // Compute c'x, x'Qx, d'nu via dense vectors to avoid
      // separator double-counting in SolverRHS::dot.
      int nv = model.number_of_variables();
      Eigen::VectorXd x_vec(nv);
      x_rhs_v.supernodes->GatherInto(x_vec);

      Eigen::VectorXd c_vec(nv);
      { auto cc = cost_rhs; cc.supernodes->GatherInto(c_vec); }
      double c_x = c_vec.dot(x_vec);

      double qx_gap = 0;
      if (model.has_quadratic_cost()) {
        auto qx_v = model.AllocSolverRHS();
        qx_v.SetZero();
        model.AccumulateQx(x_rhs_v, qx_v);
        Eigen::VectorXd qx_vec(nv);
        model.kkt().GatherInto(qx_v, qx_vec);
        qx_gap = x_vec.dot(qx_vec);
      }

      double d_nu = 0;
      if (ts && !ts->equality_sub_assemblers().empty()) {
        auto d_rhs = ts->EqualityAffineTermRHS();
        Eigen::VectorXd d_vec(nv);
        d_rhs.supernodes->GatherInto(d_vec);
        d_nu = d_vec.dot(x_vec);
      }

      double gap_primal_dual = b_lam + c_x + qx_gap + d_nu;
      double gap_compl = s_dot_x;
      printf("  %3d  %10.4e  %10.4e  %10.4e  %10.4e"
             "  gap_sl=%.2e  gap_pd=%.2e  err=%.2e"
             "  [b'l=%.2e c'x=%.2e xQx=%.2e d'v=%.2e]\n",
             outer, k_prev, k, d_inf, d_sq,
             gap_compl, gap_primal_dual,
             std::abs(gap_compl - gap_primal_dual),
             b_lam, c_x_dense, qx_dense, d_nu_dense);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, s_dot_x});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Recover x = y0/k + y1.
    {
      auto x_rhs_conv = model.AllocSolverRHS();
      x_rhs_conv.SetZero();
      x_rhs_conv.AddScaled(1.0 / k, y0);
      x_rhs_conv += y1;
      int nr = model.number_of_variables();
      result.x.resize(nr);
      { Eigen::Map<Eigen::VectorXd> xm(result.x.data(), nr); x_rhs_conv.supernodes->GatherInto(xm); }
    }

    bool converged = (s_dot_x < tolerance && d_inf < 1.01);
    bool last_iter = (outer + 1 >= max_outer_iterations);

    if (converged || last_iter) {
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
        ArenaFrame inner_frame(arena);

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

        if (inner + 1 < max_centering_steps) {
          RefreshD0Frozen(model, arena, model.GetAffineTerm(), W0, W, d0_f, y0_f);
          total_sol += 1;
        }
      }
    }

    ++outer;
  }

  return result;
}

}  // namespace conex
