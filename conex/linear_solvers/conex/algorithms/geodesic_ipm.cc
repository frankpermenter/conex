#include "conex/algorithms/geodesic_ipm.h"

#include <cmath>
#include <cstdio>

namespace conex {

GeodesicResult GeodesicCenter(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();
  const double mu = 1.0 / (k * k);

  auto y = kkt.MakeSolverRHS();
  auto row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();

  const Eigen::VectorXd W2 = W.cwiseProduct(W);

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Set weights W^2 and factor.
    weights.col() = W.cwiseProduct(W);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // 2. RHS = k * cost + A^T (k * W^2 .* b + 2 * W).
    y = cost_rhs;
    y *= k;
    v.col() = k * W.cwiseProduct(W).cwiseProduct(b) + 2.0 * W;
    kkt.AccumulateAtranspose(v, y);

    // 3. Solve.
    kkt.SolveSolverRHS(y);

    // 4. Direction d = 1 + W .* (k*b - A*y).
    kkt.MultiplyA(y, row);
    Eigen::VectorXd d =
        Eigen::VectorXd::Ones(m) + W.cwiseProduct(k * b - row.col());

    // 5. Step size.
    double d_inf = d.lpNorm<Eigen::Infinity>();
    double d_sq = d.squaredNorm();
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
    W = W.cwiseProduct((alpha * d).array().exp().matrix());
  }

  return result;
}

// Factor, two back-solves, 2-column MultiplyA → compute d0, d1.
// Shared by GeodesicLineSearch and SolveGeodesicMehrotra.
static void ComputeDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W,
    Eigen::VectorXd& d0,
    Eigen::VectorXd& d1) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();

  RowSpace weights = kkt.MakeRowSpace();
  weights.col() = W.cwiseProduct(W);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace v = kkt.MakeRowSpace();

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v.col() = 2.0 * W;
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v.col() = W.cwiseProduct(W).cwiseProduct(b);
  kkt.AccumulateAtranspose(v, rhs1);

  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  d0 = Eigen::VectorXd::Ones(m) - W.cwiseProduct(row.col(0));
  d1 = W.cwiseProduct(b - row.col(1));
}

double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W) {
  Eigen::VectorXd d0, d1;
  ComputeDecomposition(kkt, cost_rhs, W, d0, d1);

  const int m = d0.size();
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < m; ++i) {
    if (d1(i) > 0) {
      k_max = std::min(k_max, (1.0 - d0(i)) / d1(i));
    } else if (d1(i) < 0) {
      k_max = std::min(k_max, (-1.0 - d0(i)) / d1(i));
    }
  }
  return k_max;
}

// =====================================================================
// Generalized versions with per-component centering vector r.
// Changes vs scalar-mu versions:
//   RHS:  2*W  ->  2*r.*W
//   d:    W.*(...)  ->  (W./r).*(...)
// =====================================================================

static void ComputeDecompositionR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W,
    const Eigen::VectorXd& r,
    Eigen::VectorXd& d0,
    Eigen::VectorXd& d1) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();

  RowSpace weights = kkt.MakeRowSpace();
  weights.col() = W.cwiseProduct(W);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace v = kkt.MakeRowSpace();
  Eigen::VectorXd W_over_r = W.cwiseQuotient(r);

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v.col() = 2.0 * r.cwiseProduct(W);
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v.col() = W.cwiseProduct(W).cwiseProduct(b);
  kkt.AccumulateAtranspose(v, rhs1);

  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  d0 = Eigen::VectorXd::Ones(m) - W_over_r.cwiseProduct(row.col(0));
  d1 = W_over_r.cwiseProduct(b - row.col(1));
}

GeodesicResult GeodesicCenterR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    const Eigen::VectorXd& r,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();

  auto y = kkt.MakeSolverRHS();
  auto row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();

  GeodesicResult result{};

  for (int iter = 0; iter < max_iterations; ++iter) {
    weights.col() = W.cwiseProduct(W);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // RHS = k * cost + A^T (k * W^2 .* b + 2 * r .* W).
    y = cost_rhs;
    y *= k;
    v.col() = k * W.cwiseProduct(W).cwiseProduct(b) + 2.0 * r.cwiseProduct(W);
    kkt.AccumulateAtranspose(v, y);

    kkt.SolveSolverRHS(y);

    // d = 1 + (W./r) .* (k*b - A*y).
    kkt.MultiplyA(y, row);
    Eigen::VectorXd W_over_r = W.cwiseQuotient(r);
    Eigen::VectorXd d =
        Eigen::VectorXd::Ones(m) + W_over_r.cwiseProduct(k * b - row.col());

    double d_inf = d.lpNorm<Eigen::Infinity>();
    double d_sq = d.squaredNorm();
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));

    // Complementarity: sum(r_i^2 * (1 - d_i^2)).
    double s_dot_x = r.cwiseProduct(r).dot(
        Eigen::VectorXd::Ones(m) - d.cwiseProduct(d));

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

    W = W.cwiseProduct((alpha * d).array().exp().matrix());
  }

  return result;
}

double GeodesicLineSearchR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W,
    const Eigen::VectorXd& r) {
  Eigen::VectorXd d0, d1;
  ComputeDecompositionR(kkt, cost_rhs, W, r, d0, d1);

  const int m = d0.size();
  double k_max = std::numeric_limits<double>::max();
  for (int i = 0; i < m; ++i) {
    if (d1(i) > 0)
      k_max = std::min(k_max, (1.0 - d0(i)) / d1(i));
    else if (d1(i) < 0)
      k_max = std::min(k_max, (-1.0 - d0(i)) / d1(i));
  }
  return k_max;
}

GeodesicResult SolveGeodesicLPR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    const Eigen::VectorXd& r,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  double k = 1.0;
  const int m = static_cast<int>(W.size());

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
    total_sol += 1;
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
    Eigen::VectorXd& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  double k = 1.0;
  const int m = static_cast<int>(W.size());

  // Initial centering at k = 1.
  auto result = GeodesicCenter(kkt, cost_rhs, W, k, 100, 1e-12);
  result.iter_stats.push_back({result.mu, result.d_inf_norm,
                               result.d_sq_norm, result.complementarity});
  // initial center: N factors + N solves
  int total_fac = result.total_factorizations;
  int total_sol = result.total_solves;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    // Decompose: 1 factor + 1 solve (2-column).
    Eigen::VectorXd d0, d1;
    ComputeDecomposition(kkt, cost_rhs, W, d0, d1);
    total_fac += 1;
    total_sol += 1;

    // Line search for k.
    double k_new = std::numeric_limits<double>::max();
    for (int i = 0; i < m; ++i) {
      if (d1(i) > 0)
        k_new = std::min(k_new, (1.0 - d0(i)) / d1(i));
      else if (d1(i) < 0)
        k_new = std::min(k_new, (-1.0 - d0(i)) / d1(i));
    }
    if (k_new <= k) break;
    k = k_new;

    // Take one geodesic step at k using d = d0 + k * d1.
    Eigen::VectorXd d = d0 + k * d1;
    double d_inf = d.lpNorm<Eigen::Infinity>();
    double d_sq = d.squaredNorm();
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
    W = W.cwiseProduct((alpha * d).array().exp().matrix());

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
    Eigen::VectorXd& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.col();
  const int m = b.size();
  const double k = 1.0;

  Eigen::VectorXd r = Eigen::VectorXd::Ones(m);

  // Initial centering until |d|_inf <= 1.
  auto init = GeodesicCenterR(kkt, cost_rhs, W, r, k, 100, 1.0);
  int total_fac = init.total_factorizations;
  int total_sol = init.total_solves;

  GeodesicResult result{};
  int r_updates = 0;

  RowSpace weights = kkt.MakeRowSpace();

  // R-update loop: solve, check gap, shrink r. Repeat until gap < 0
  // or converged.  Each iteration is one back-solve (reusing factorization).
  int r_updates_this_fac = 0;
  double gap = 0, d_inf = 0, d_sq = 0;

  if (verbose) {
    printf("  %3s  %12s  %12s  %6s  %6s\n",
           "fac", "gap", "d_inf", "r_upd", "solves");
    printf("  %s\n", std::string(48, '-').c_str());
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    auto y = kkt.MakeSolverRHS();
    y = cost_rhs;
    y *= k;
    RowSpace v = kkt.MakeRowSpace();
    v.col() = k * W.cwiseProduct(W).cwiseProduct(b) +
              2.0 * r.cwiseProduct(W);
    kkt.AccumulateAtranspose(v, y);
    kkt.SolveSolverRHS(y);
    total_sol++;

    auto row = kkt.MakeRowSpace();
    kkt.MultiplyA(y, row);
    Eigen::VectorXd d_vec =
        Eigen::VectorXd::Ones(m) +
        W.cwiseQuotient(r).cwiseProduct(k * b - row.col());

    gap = r.cwiseProduct(r).dot(
        Eigen::VectorXd::Ones(m) - d_vec.cwiseProduct(d_vec));
    d_inf = d_vec.lpNorm<Eigen::Infinity>();
    d_sq = d_vec.squaredNorm();

    if (gap >= 0 && gap < tolerance && d_inf <= 1.0) break;

    if (gap < 0) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      W = W.cwiseProduct((alpha * d_vec).array().exp().matrix());
      weights.col() = W.cwiseProduct(W);
      kkt.SetWeights(weights);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      if (verbose) {
        printf("  %3d  %12.4e  %12.4e  %6d  %6d\n",
               total_fac, gap, d_inf, r_updates_this_fac, total_sol);
      }
      r_updates_this_fac = 0;
    } else {
      // Shrink r only.
      r = 0.5 * r.cwiseProduct(
          Eigen::VectorXd::Ones(m) + d_vec.cwiseAbs());
      r_updates_this_fac++;
      r_updates++;
    }
  }

  if (verbose) {
    printf("  %3d  %12.4e  %12.4e  %6d  %6d\n",
           total_fac, gap, d_inf, r_updates_this_fac, total_sol);
  }

  result.iter_stats.push_back({gap / m, d_inf, d_sq, gap});
  result.d_inf_norm = d_inf;
  result.d_sq_norm = d_sq;
  result.mu = gap / m;
  result.complementarity = gap;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  return result;
}

}  // namespace conex
