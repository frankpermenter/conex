#include "conex/algorithms/geodesic_ipm.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

static void ComputeDirectNewtonStep(
    KKTSolverBase& kkt, const SolverRHS& cost_rhs,
    const RowSpace& W, double k,
    RowSpace& d_out, Eigen::VectorXd& y_out);

GeodesicResult GeodesicCenter(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    double k,
    int max_iterations,
    double tolerance,
    bool verbose) {
  const int m = W.total_rows();
  const double mu = 1.0 / (k * k);

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt.MakeRowSpace();
    Eigen::VectorXd y_direct;
    ComputeDirectNewtonStep(kkt, cost_rhs, W, k, d, y_direct);

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

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  // Ax + b >= 0 convention.
  // RHS = -k*cost + A^T(-k*P(W)b + 2*W).
  // d = 1 + P(W^{1/2})(-k*b - A*y).
  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  y *= -k;
  RowSpace v = addScaled(quadraticRepresentation(W, b), W, -k, 2.0);
  kkt.AccumulateAtranspose(v, y);
  kkt.SolveSolverRHS(y);

  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  d_out = addScaled(b, row, -k, -1.0);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  d_out = quadraticRepresentation(sqrtW, d_out);
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
    RowSpace& d1,
    Eigen::VectorXd* y0_out = nullptr,
    Eigen::VectorXd* y1_out = nullptr) {
  RowSpace b = kkt.GetAffineTerm();

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  RowSpace v = kkt.MakeRowSpace();

  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v = W;
  v *= 2.0;
  kkt.AccumulateAtranspose(v, rhs0);

  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v = quadraticRepresentation(W, b);
  kkt.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;

  auto y = kkt.MakeSolverRHS(2);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  kkt.SolveSolverRHS(y);

  if (y0_out || y1_out) {
    int nr = kkt.number_of_variables();
    Eigen::MatrixXd y_dense(nr, 2);
    y.supernodes->GatherInto(y_dense);
    if (y0_out) *y0_out = y_dense.col(0);
    if (y1_out) *y1_out = y_dense.col(1);
  }

  auto row = kkt.MakeRowSpace(2);
  kkt.MultiplyA(y, row);

  RowSpace ay0 = kkt.MakeRowSpace();
  RowSpace ay1 = kkt.MakeRowSpace();
  ay0.col() = row.col(0);
  ay1.col() = row.col(1);

  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  d0 = kkt.MakeRowSpace();
  setOnes(d0);
  d0 -= quadraticRepresentation(sqrtW, ay0);

  d1 = quadraticRepresentation(sqrtW, addScaled(b, ay1, -1.0, -1.0));
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
  //auto result = GeodesicCenter(kkt, cost_rhs, W, k, 100, 1e-12);
  //result.iter_stats.push_back({result.mu, result.d_inf_norm,
  //                             result.d_sq_norm, result.complementarity});
  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;


  if (verbose) {
    printf("  %3s  %12s  %12s  %12s  %12s  %12s\n",
           "fac", "k", "k_new", "d_inf", "d_sqr", "gap");
    printf("  %s\n", std::string(72, '-').c_str());
  }

  kkt.AssembleAndFactor();
  for (int outer = 0; outer < max_outer_iterations; ++outer) {

    // Decompose: 1 factor + 2 back-solves.
    RowSpace d0 = kkt.MakeRowSpace();
    RowSpace d1 = kkt.MakeRowSpace();
    Eigen::VectorXd y0, y1;
    ComputeDecomposition(kkt, cost_rhs, W, d0, d1, &y0, &y1);
    total_fac += 1;
    total_sol += 2;

    // Line search for k.  Accept only if it increases k (except iter 0).
    double k_new = lineSearchK(d0, d1);
    double k_prev = k;
    if (outer == 0 || k_new > k) {
      k = k_new;
    }

    // Take one geodesic step at k using d = d0 + k * d1.
    RowSpace d = addScaled(d0, d1, 1.0, k);
    double d_inf = normInf(d);
    double d_sq = squaredNorm(d);
    double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
    geodesicUpdate(W, alpha, d);

    //// Additional centering steps if requested.
    //if (max_centering_steps > 0) {
    //  auto cr = GeodesicCenter(kkt, cost_rhs, W, k,
    //                           max_centering_steps, 1e-12, verbose);
    //  total_fac += cr.total_factorizations;
    //  total_sol += cr.total_solves;
    //  d_inf = cr.d_inf_norm;
    //  d_sq = cr.d_sq_norm;
    //}

    double mu = 1.0 / (k * k);
    double s_dot_x = mu * (m - d_sq);

    if (verbose) {
      double d0_inf = normInf(d0);
      double d1_inf = normInf(d1);
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e  d0=%.2e d1=%.2e\n",
             outer, k_prev, k, d_inf, d_sq, s_dot_x, d0_inf, d1_inf);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, s_dot_x});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = s_dot_x;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;
    result.x = y0 / k + y1;  // x = y/k = (y0 + k*y1)/k

    if (s_dot_x < tolerance) break;
  }

  return result;
}

HybridDirection ComputeHybridDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta) {
  RowSpace b = kkt.GetAffineTerm();

  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  y *= -1;
  RowSpace sqrtW_rhs = EuclideanJordanAlgebra::sqrt(W);
  RowSpace v = addScaled(quadraticRepresentation(W, b),
                         quadraticRepresentation(sqrtW_rhs, r), -1, 2.0);
  kkt.AccumulateAtranspose(v, y);
  kkt.SolveSolverRHS(y);

  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace slack_dir = addScaled(b, row, 1.0, 1.0);
  delta = addScaled(r,
      quadraticRepresentation(sqrtW, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);
  std::cout << slack_dir;


  auto residual = kkt.MakeSolverRHS();
  residual.SetZero();
  kkt.AccumulateAtranspose(quadraticRepresentation(sqrtW, r + delta), residual);
  std::cout << "RES CHECK";
  std::cout << "Computed\n" << residual << "\n";
  std::cout << "Reference\n" << cost_rhs << "\n";


  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

HybridDirection HybridCenteringStep(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    RowSpace& r) {
  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  RowSpace d = kkt.MakeRowSpace();
  RowSpace delta = kkt.MakeRowSpace();
  auto info = ComputeHybridDirection(kkt, cost_rhs, W, r, d, delta);

  double alpha = std::min(1.0, 2.0 / (info.d_inf * info.d_inf));
  updateAutomorphism(W, r, alpha, d);
  return info;
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

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);

  // Initial centering until |d|_inf <= 1.
  kkt.AssembleAndFactor();
  int total_fac = 0;//init.total_factorizations;
  int total_sol = 0;//init.total_solves;

  GeodesicResult result{};
  int r_updates = 0;

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
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = ComputeHybridDirection(kkt, cost_rhs, W, r, d, delta);
    total_sol++;

    g = info.gap;
    d_inf = info.d_inf;
    d_sq = info.d_sq;
    mslack = info.min_slack;

    if (std::abs(g) < tolerance && mslack > -0.0001) break;
    if (g < 0) {
      // Centering step: update W and r, then refactor.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                                   r_updates_this_fac, mslack});
      r_updates_this_fac = 0;
    } else {
      // Shrink r using Delta.
      shrinkR(r, delta);
      r_updates_this_fac++;
      r_updates++;
    }
    if (iter > 1) break;
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
