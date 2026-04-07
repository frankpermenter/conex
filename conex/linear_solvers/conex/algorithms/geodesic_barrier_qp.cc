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
  const auto& b = b_row.data.col(0);
  const int m = b.size();
  const double mu = 1.0 / (k * k);

  auto y = kkt.MakeSolverRHS();
  auto row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Set weights W^2 and factor the Gram matrix A^T diag(W^2) A.
    for (int i = 0; i < m; ++i)
      weights.data(i) = W(i) * W(i);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // 2. Build RHS = k * cost + A^T (k * W^2 .* b + 2 * W).
    y = cost_rhs;
    y *= k;
    for (int i = 0; i < m; ++i)
      v.data(i) = k * W(i) * W(i) * b(i) + 2.0 * W(i);
    kkt.AccumulateAtranspose(v, y);

    // 3. Solve for y.
    kkt.SolveSolverRHS(y);

    // 4. Compute direction d = 1 + W .* (k*b - A*y).
    kkt.MultiplyA(y, row);
    Eigen::VectorXd d(m);
    for (int i = 0; i < m; ++i)
      d(i) = 1.0 + W(i) * (k * b(i) - row.data(i));

    // 5. Step size: alpha = min(1, 2 / ||d||_inf^2).
    double d_inf = d.lpNorm<Eigen::Infinity>();
    double d_sq = d.squaredNorm();
    double alpha = 2.0 / (d_inf * d_inf);
    if (alpha > 1.0) alpha = 1.0;

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
    for (int i = 0; i < m; ++i)
      W(i) *= std::exp(alpha * d(i));
  }

  return result;
}

double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const Eigen::VectorXd& W) {
  RowSpace b_row = kkt.GetAffineTerm();
  const auto& b = b_row.data.col(0);
  const int m = b.size();

  // Factor A^T diag(W^2) A (same Gram for all k).
  RowSpace weights = kkt.MakeRowSpace();
  for (int i = 0; i < m; ++i)
    weights.data(i) = W(i) * W(i);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  // Build two single-column RHS, then pack into one 2-column SolverRHS.
  //   col 0: A^T(2W)
  //   col 1: cost + A^T(W^2 .* b)
  RowSpace v = kkt.MakeRowSpace();

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  for (int i = 0; i < m; ++i) v.data(i) = 2.0 * W(i);
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  for (int i = 0; i < m; ++i) v.data(i) = W(i) * W(i) * b(i);
  kkt.AccumulateAtranspose(v, rhs1);

  // Pack into 2-column SolverRHS and solve.
  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  // 2-column MultiplyA.
  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  // d0 = 1 - W .* (Ay)_col0,  d1 = W .* (b - (Ay)_col1).
  Eigen::VectorXd d0(m), d1(m);
  for (int i = 0; i < m; ++i) {
    d0(i) = 1.0 - W(i) * row.data(i, 0);
    d1(i) = W(i) * (b(i) - row.data(i, 1));
  }

  // Find largest k > 0 with |d0_i + k * d1_i| <= 1 for all i.
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

}  // namespace conex
