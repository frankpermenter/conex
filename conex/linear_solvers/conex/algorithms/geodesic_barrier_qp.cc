#include "conex/algorithms/geodesic_barrier_qp.h"

#include <cmath>

namespace conex {

GeodesicResult GeodesicCenter(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    Eigen::VectorXd& W,
    double k,
    int max_iterations,
    double tolerance) {
  RowSpace b_row = kkt.GetAffineTerm();
  const Eigen::VectorXd& b = b_row.data;
  const int m = b.size();

  auto y = kkt.MakeSolverRHS();
  auto row = kkt.MakeRowSpace();
  RowSpace weights = kkt.MakeRowSpace();
  RowSpace v = kkt.MakeRowSpace();

  GeodesicResult result{};

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Set weights W^2 and factor the Gram matrix A^T diag(W^2) A.
    for (int i = 0; i < m; ++i)
      weights.data(i) = W(i) * W(i);
    kkt.SetWeights(weights);
    if (!kkt.AssembleAndFactor()) break;

    // 2. Build RHS = k * cost + A^T (k * W^2 .* b + 2 * W).
    //
    // Derivation (cone_program sign mapping):
    //   cone_program RHS = k*b_cost + k*A_cone^T diag(W^2) c_aff - 2*A_cone^T W
    //   with A_cone = -A, c_aff = -b, b_cost = cost:
    //   = k*cost + k*A^T(W^2.*b) + 2*A^T W
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

    result.iterations = iter + 1;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;

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
  const Eigen::VectorXd& b = b_row.data;
  const int m = b.size();

  // Factor A^T diag(W^2) A (same Gram for all k).
  RowSpace weights = kkt.MakeRowSpace();
  for (int i = 0; i < m; ++i)
    weights.data(i) = W(i) * W(i);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace v = kkt.MakeRowSpace();
  auto row = kkt.MakeRowSpace();

  // Solve for y0: G * y0 = A^T(2W).
  auto y0 = kkt.MakeSolverRHS();
  y0.SetZero();
  for (int i = 0; i < m; ++i)
    v.data(i) = 2.0 * W(i);
  kkt.AccumulateAtranspose(v, y0);
  kkt.SolveSolverRHS(y0);

  // Solve for y1: G * y1 = cost + A^T(W^2 .* b).
  auto y1 = kkt.MakeSolverRHS();
  y1 = cost_rhs;
  for (int i = 0; i < m; ++i)
    v.data(i) = W(i) * W(i) * b(i);
  kkt.AccumulateAtranspose(v, y1);
  kkt.SolveSolverRHS(y1);

  // d0 = 1 - W .* A*y0,  d1 = W .* (b - A*y1).
  kkt.MultiplyA(y0, row);
  Eigen::VectorXd d0(m);
  for (int i = 0; i < m; ++i)
    d0(i) = 1.0 - W(i) * row.data(i);

  kkt.MultiplyA(y1, row);
  Eigen::VectorXd d1(m);
  for (int i = 0; i < m; ++i)
    d1(i) = W(i) * (b(i) - row.data(i));

  // Find largest k > 0 with |d0_i + k * d1_i| <= 1 for all i.
  //   d1_i > 0:  k <= (1 - d0_i) / d1_i
  //   d1_i < 0:  k <= (-1 - d0_i) / d1_i
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
