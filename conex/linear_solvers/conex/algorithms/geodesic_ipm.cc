#include "conex/algorithms/geodesic_ipm.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

GeodesicResult GeodesicCenter(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();
  const double mu = 1.0 / (k * k);

  auto y = kkt.MakeSolverRHS();
  RowSpace row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();
  RowSpace d = kkt.MakeRowSpace();

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Set weights W^2 and factor.
    weights = cwiseProduct(W, W);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // 2. RHS = k * cost + A^T (k * W^2 .* b + 2 * W).
    y = cost_rhs;
    y *= k;
    v = addScaled(cwiseProduct(weights, b), W, k, 2.0);
    kkt.AccumulateAtranspose(v, y);

    // 3. Solve.
    kkt.SolveSolverRHS(y);

    // 4. Direction d = 1 + W .* (k*b - A*y).
    kkt.MultiplyA(y, row);
    d = addScaled(b, row, k, -1.0);
    d = cwiseProduct(W, d);
    setOnes(v);
    d += v;

    // 5. Step size.
    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));

    double s_dot_x = mu * (m - d_sq);

    result.iterations = iter + 1;
    result.total_factorizations = iter + 1;
    result.total_solves = iter + 1;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;

    if (verbose) {
      printf("  i=%2d  mu=%.2e  d_sqr=%.2e  d_inf=%.2e  "
             "s_dot_x=%.2e  alpha=%.4f\n",
             iter, mu, d_sq, d_inf, s_dot_x, alpha);
    }

    if (d_inf < tolerance) break;

    // 6. Geodesic update: W *= exp(alpha * d).
    geodesicUpdate(W, alpha, d);
  }

  return result;
}

// Direct Newton step: factor, single RHS, solve, compute d and y.
// Matches what GeodesicCenter does per iteration.
static void ComputeDirectNewtonStep(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    double k,
    RowSpace& d_out,
    Eigen::VectorXd& y_out) {
  RowSpace b = kkt.GetAffineTerm();

  RowSpace weights = cwiseProduct(W, W);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  y *= k;
  RowSpace v = addScaled(cwiseProduct(weights, b), W, k, 2.0);
  kkt.AccumulateAtranspose(v, y);
  kkt.SolveSolverRHS(y);

  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  d_out = addScaled(b, row, k, -1.0);
  d_out = cwiseProduct(W, d_out);
  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);
  d_out += ones;

  int nr = kkt.number_of_variables();
  y_out.resize(nr);
  y.supernodes->GatherInto(y_out);
}

// Factor, two back-solves, 2-column MultiplyA → compute d0, d1.
static void ComputeDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    RowSpace& d0,
    RowSpace& d1) {
  RowSpace b = kkt.GetAffineTerm();

  RowSpace weights = cwiseProduct(W, W);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace v = kkt.MakeRowSpace();

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v = W;
  v *= 2.0;
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v = cwiseProduct(weights, b);
  kkt.AccumulateAtranspose(v, rhs1);

  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  // d0 = 1 - W .* (Ay)_col0.
  // d1 = W .* (b - (Ay)_col1).
  // For now, extract columns into single-col RowSpaces.
  RowSpace ay0 = kkt.MakeRowSpace();
  RowSpace ay1 = kkt.MakeRowSpace();
  ay0.col() = row.col(0);
  ay1.col() = row.col(1);

  d0 = kkt.MakeRowSpace();
  setOnes(d0);
  d0 -= cwiseProduct(W, ay0);

  d1 = cwiseProduct(W, addScaled(b, ay1, 1.0, -1.0));
}

double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W) {
  RowSpace d0 = kkt.MakeRowSpace();
  RowSpace d1 = kkt.MakeRowSpace();
  ComputeDecomposition(kkt, cost_rhs, W, d0, d1);

  return lineSearchK(d0, d1);
}

// =====================================================================
// Generalized versions with per-component centering vector r.
// =====================================================================

static void ComputeDecompositionR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d0,
    RowSpace& d1) {
  RowSpace b = kkt.GetAffineTerm();

  RowSpace weights = cwiseProduct(W, W);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace W_over_r = cwiseQuotient(W, r);
  RowSpace v = kkt.MakeRowSpace();

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v = cwiseProduct(r, W);
  v *= 2.0;
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v = cwiseProduct(weights, b);
  kkt.AccumulateAtranspose(v, rhs1);

  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  RowSpace ay0 = kkt.MakeRowSpace();
  RowSpace ay1 = kkt.MakeRowSpace();
  ay0.col() = row.col(0);
  ay1.col() = row.col(1);

  d0 = kkt.MakeRowSpace();
  setOnes(d0);
  d0 -= cwiseProduct(W_over_r, ay0);

  d1 = cwiseProduct(W_over_r, addScaled(b, ay1, 1.0, -1.0));
}

GeodesicResult GeodesicCenterR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    const RowSpace& r,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();

  auto y = kkt.MakeSolverRHS();
  RowSpace row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();
  RowSpace d = kkt.MakeRowSpace();
  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);

  GeodesicResult result{};

  for (int iter = 0; iter < max_iterations; ++iter) {
    weights = cwiseProduct(W, W);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // RHS = k * cost + A^T (k * W^2 .* b + 2 * r .* W).
    y = cost_rhs;
    y *= k;
    v = addScaled(cwiseProduct(weights, b), cwiseProduct(r, W), k, 2.0);
    kkt.AccumulateAtranspose(v, y);

    kkt.SolveSolverRHS(y);

    // d = 1 + (W./r) .* (k*b - A*y).
    kkt.MultiplyA(y, row);
    RowSpace W_over_r = cwiseQuotient(W, r);
    d = addScaled(b, row, k, -1.0);
    d = cwiseProduct(W_over_r, d);
    d += ones;

    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));

    double s_dot_x = gap(r, d);

    result.iterations = iter + 1;
    result.total_factorizations = iter + 1;
    result.total_solves = iter + 1;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.mu = s_dot_x / m;
    result.complementarity = s_dot_x;

    if (verbose) {
      printf("  i=%2d  s_dot_x=%.2e  d_sqr=%.2e  d_inf=%.2e  alpha=%.4f\n",
             iter, s_dot_x, d_sq, d_inf, alpha);
    }

    if (d_inf < tolerance) break;

    geodesicUpdate(W, alpha, d);
  }

  return result;
}

double GeodesicLineSearchR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    const RowSpace& r) {
  RowSpace d0 = kkt.MakeRowSpace();
  RowSpace d1 = kkt.MakeRowSpace();
  ComputeDecompositionR(kkt, cost_rhs, W, r, d0, d1);

  return lineSearchK(d0, d1);
}

GeodesicResult SolveGeodesicLPR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    const RowSpace& r,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  double k = 1.0;
  const int m = W.total_rows();

  auto result_init = GeodesicCenterR(kkt, cost_rhs, W, r, k, 100, 1e-12);
  GeodesicResult result{};
  result.iter_stats.push_back({result_init.mu, result_init.d_inf_norm,
                               result_init.d_sq_norm,
                               result_init.complementarity});
  int total_fac = result_init.total_factorizations;
  int total_sol = result_init.total_solves;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    double k_new = GeodesicLineSearchR(kkt, cost_rhs, W, r);
    total_fac += 1;
    total_sol += 2;
    if (k_new <= k) break;
    k = k_new;

    auto cr = GeodesicCenterR(kkt, cost_rhs, W, r, k,
                              max_centering_steps, 1e-12, verbose);
    total_fac += cr.total_factorizations;
    total_sol += cr.total_solves;

    result.iter_stats.push_back({cr.mu, cr.d_inf_norm,
                                 cr.d_sq_norm, cr.complementarity});
    result.iterations = outer + 1;
    result.mu = cr.mu;
    result.d_inf_norm = cr.d_inf_norm;
    result.d_sq_norm = cr.d_sq_norm;
    result.complementarity = cr.complementarity;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    if (cr.complementarity < tolerance && cr.complementarity > 0) break;
  }

  return result;
}

GeodesicResult SolveGeodesicLP(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  double k = 1.0;
  const int m = W.total_rows();

  // Initial centering at k = 1.
  auto result = GeodesicCenter(kkt, cost_rhs, W, k, 100, 1e-12);
  result.iter_stats.push_back({result.mu, result.d_inf_norm,
                               result.d_sq_norm, result.complementarity});
  int total_fac = result.total_factorizations;
  int total_sol = result.total_solves;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    // Decompose: 1 factor + 2 back-solves.
    RowSpace d0 = kkt.MakeRowSpace();
    RowSpace d1 = kkt.MakeRowSpace();
    ComputeDecomposition(kkt, cost_rhs, W, d0, d1);
    total_fac += 1;
    total_sol += 2;

    // Line search for k.
    double k_new = lineSearchK(d0, d1);
    if (k_new <= k) break;
    k = k_new;

    // DEBUG: compare decomposed d with direct Newton d at the same k.
    //
    // The Newton direction satisfies:
    //   d = 1 + W*(k*b - A*y)
    //
    // So 1 - d = W*(A*y - k*b), and with x = y/k:
    //   (1/k) * W^{-1} * (1-d)  =  A*x - b
    //
    // This is the slack for A*x >= b.  At convergence d -> 0,
    // slack = (1/k)*W^{-1} > 0, confirming A*x > b.
    {
      RowSpace d_direct = kkt.MakeRowSpace();
      Eigen::VectorXd y_direct;
      ComputeDirectNewtonStep(kkt, cost_rhs, W, k, d_direct, y_direct);
      RowSpace d_decomp = addScaled(d0, d1, 1.0, k);
      double d_err = std::sqrt(squaredNorm(addScaled(d_direct, d_decomp, 1.0, -1.0)));

      // Compute slack = A*x - b where x = y/k.
      // From d: (1/k)*W^{-1}*(1-d) = A*x - b.
      RowSpace ones = kkt.MakeRowSpace();
      setOnes(ones);
      RowSpace one_minus_d = addScaled(ones, d_direct, 1.0, -1.0);
      RowSpace inv_W = cwiseQuotient(ones, W);
      RowSpace slack = cwiseProduct(inv_W, one_minus_d);
      slack *= (1.0 / k);
      double min_slack = slack.col().minCoeff();

      printf("  DEBUG iter=%d k=%.4f: d_err=%.2e  min_slack=%.4e\n",
             outer, k, d_err, min_slack);
      if (d_err > 1e-3) {
        throw std::runtime_error("Decomposition d does not match direct d!");
      }

      // Save x = y/k and slack = (1/k)*W^{-1}*(1-d).
      int nr = kkt.number_of_variables();
      result.x = y_direct / k;
      result.slack.resize(slack.total_rows());
      for (int i = 0; i < slack.total_rows(); ++i)
        result.slack(i) = slack.col()(i);
    }

    // Take one geodesic step at k using d = d0 + k * d1.
    RowSpace d = addScaled(d0, d1, 1.0, k);
    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
    geodesicUpdate(W, alpha, d);

    // Additional centering steps if requested.
    if (max_centering_steps > 0) {
      auto cr = GeodesicCenter(kkt, cost_rhs, W, k,
                               max_centering_steps, 1e-12, verbose);
      total_fac += cr.total_factorizations;
      total_sol += cr.total_solves;
      d_inf = cr.d_inf_norm;
      d_sq = cr.d_sq_norm;
    }

    double mu = 1.0 / (k * k);
    double s_dot_x = mu * (m - d_sq);

    result.iter_stats.push_back({mu, d_inf, d_sq, s_dot_x});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    if (s_dot_x < tolerance) break;
  }

  return result;
}

GeodesicResult SolveGeodesicHybrid(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();
  const double k = 1.0;

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);

  // Initial centering until |d|_inf <= 1.
  auto init = GeodesicCenterR(kkt, cost_rhs, W, r, k, 100, 1.0);
  int total_fac = init.total_factorizations;
  int total_sol = init.total_solves;

  GeodesicResult result{};
  int r_updates = 0;

  RowSpace weights = kkt.MakeRowSpace();
  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);

  int r_updates_this_fac = 0;
  double g = 0, d_inf = 0, d_sq = 0, mslack = 0;

  if (verbose) {
    printf("  %3s  %12s  %12s  %6s  %6s\n",
           "fac", "gap", "d_inf", "r_upd", "solves");
    printf("  %s\n", std::string(48, '-').c_str());
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Solve with current (W, r).
    auto y = kkt.MakeSolverRHS();
    y = cost_rhs;
    y *= k;
    RowSpace v = addScaled(cwiseProduct(cwiseProduct(W, W), b),
                           cwiseProduct(r, W), k, 2.0);
    kkt.AccumulateAtranspose(v, y);
    kkt.SolveSolverRHS(y);
    total_sol++;

    // d = 1 + (W/r) .* (k*b - A*y).
    RowSpace row = kkt.MakeRowSpace();
    kkt.MultiplyA(y, row);
    RowSpace W_over_r = cwiseQuotient(W, r);
    RowSpace d = addScaled(b, row, k, -1.0);
    d = cwiseProduct(W_over_r, d);
    d += ones;

    g = gap(r, d);
    d_inf = normInf(d);
    d_sq = squaredNorm(d);
    mslack = minSlack(r, d);

    if (std::abs(g) < tolerance && mslack > -tolerance) break;

    if (g < 0) {
      // Geodesic step on W, then re-factor.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d);
      weights = cwiseProduct(W, W);
      kkt.SetWeights(weights);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                                   r_updates_this_fac, mslack});
      r_updates_this_fac = 0;
    } else {
      // Shrink r only.
      shrinkR(r, d);
      r_updates_this_fac++;
      r_updates++;
    }
  }

  result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                               r_updates_this_fac, mslack});
  result.d_inf_norm = d_inf;
  result.d_sq_norm = d_sq;
  result.mu = g / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  return result;
}

}  // namespace conex
