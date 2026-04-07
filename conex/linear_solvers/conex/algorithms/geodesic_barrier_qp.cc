#include "conex/algorithms/geodesic_barrier_qp.h"

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

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    double k_new = GeodesicLineSearch(kkt, cost_rhs, W);
    if (k_new <= k) break;
    k = k_new;

    auto cr = GeodesicCenter(kkt, cost_rhs, W, k,
                             max_centering_steps, 1e-12, verbose);
    double mu = 1.0 / (k * k);
    double s_dot_x = mu * (m - cr.d_sq_norm);

    result.iter_stats.push_back({mu, cr.d_inf_norm, cr.d_sq_norm, s_dot_x});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = cr.d_inf_norm;
    result.d_sq_norm = cr.d_sq_norm;
    result.complementarity = s_dot_x;

    if (s_dot_x < tolerance) break;
  }

  return result;
}

GeodesicResult SolveGeodesicMehrotra(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    int max_iterations,
    double tolerance,
    bool verbose) {
  const int m = static_cast<int>(W.size());

  // Initialize: center at k=1, line search for first k, then fully center.
  GeodesicCenter(kkt, cost_rhs, W, 1.0, 100, 1e-12);
  double k = GeodesicLineSearch(kkt, cost_rhs, W);
  GeodesicCenter(kkt, cost_rhs, W, k, 100, 1e-12);
  double mu = 1.0 / (k * k);

  GeodesicResult result{};

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Decompose: d(e_weight, k) = e_weight * d0 + k * d1.
    Eigen::VectorXd d0, d1;
    ComputeDecomposition(kkt, cost_rhs, W, d0, d1);

    // 2. Predictor (e_weight = 0): d_pred = k_pred * d1.
    double d1_inf = d1.lpNorm<Eigen::Infinity>();
    if (d1_inf < 1e-15) break;
    double k_pred = 1.0 / d1_inf;

    Eigen::VectorXd d_pred = k_pred * d1;
    double d_pred_inf = d_pred.lpNorm<Eigen::Infinity>();  // = 1.0
    double alpha_aff = std::min(1.0, 2.0 / (d_pred_inf * d_pred_inf));

    // 3. Predicted complementarity.
    double mu_pred = 1.0 / (k_pred * k_pred);
    double d_pred_sq = d_pred.squaredNorm();
    double mu_aff = mu_pred * (m - alpha_aff * alpha_aff * d_pred_sq) / m;

    // 4. Adaptive centering.
    double sigma = std::pow(mu_aff / mu, 3);
    sigma = std::max(sigma, 1e-12);  // safeguard

    // 5. Combined direction: d = d0 + (k_pred / sigma) * d1.
    //    Cap at the line search bound to ensure |d|_inf stays feasible.
    double k_ls = std::numeric_limits<double>::max();
    for (int i = 0; i < m; ++i) {
      if (d1(i) > 0)
        k_ls = std::min(k_ls, (1.0 - d0(i)) / d1(i));
      else if (d1(i) < 0)
        k_ls = std::min(k_ls, (-1.0 - d0(i)) / d1(i));
    }
    double k_eff = std::min(k_pred / sigma, k_ls);
    Eigen::VectorXd d = d0 + k_eff * d1;

    // 6. Step size and update.
    double d_inf = d.lpNorm<Eigen::Infinity>();
    double d_sq = d.squaredNorm();
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));

    double mu_target = 1.0 / (k_eff * k_eff);
    double s_dot_x = mu_target * (m - d_sq);
    mu = s_dot_x / m;  // actual complementarity per constraint

    result.iter_stats.push_back({mu, d_inf, d_sq, s_dot_x});
    result.iterations = iter + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;

    if (verbose) {
      printf("  i=%2d  mu=%.2e  d_inf=%.2e  d_sqr=%.2e  "
             "s_dot_x=%.2e  sigma=%.2e  k_eff=%.2e  alpha=%.4f\n",
             iter, mu, d_inf, d_sq, s_dot_x, sigma, k_eff, alpha);
    }

    if (s_dot_x < tolerance && s_dot_x > 0) break;

    W = W.cwiseProduct((alpha * d).array().exp().matrix());
  }

  return result;
}

}  // namespace conex
