#include "conex/algorithms/geodesic_hybrid_r.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/tree_solver/kkt_tree_solver.h"

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
using EuclideanJordanAlgebra::updateAutomorphism;

// Compute b_theta = theta * e + (1 - theta) * b.
static RowSpace BlendAffine(KKTSolverBase& kkt, const RowSpace& b,
                            double theta) {
  RowSpace b_theta = kkt.MakeRowSpace();
  if (theta == 0.0) {
    b_theta = b;
  } else if (theta == 1.0) {
    setOnes(b_theta);
  } else {
    RowSpace ones = kkt.MakeRowSpace();
    setOnes(ones);
    b_theta = addScaled(ones, b, theta, 1.0 - theta);
  }
  return b_theta;
}

HybridRDirection ComputeHybridRDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    RowSpace& d,
    RowSpace& delta) {
  RowSpace b_theta = BlendAffine(kkt, b, theta);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // RHS = -(c + A^T P(W)(b_theta)) + 2*A^T P(W^{1/2})(r) + d_eq
  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  RowSpace v = quadraticRepresentation(W, b_theta);
  kkt.AccumulateAtranspose(v, y);
  y *= -1;
  // + 2*A^T P(W^{1/2})(r)
  v = quadraticRepresentation(sqrtW, r);
  v *= 2.0;
  kkt.AccumulateAtranspose(v, y);
  // + d_eq (equality RHS).
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts && !ts->equality_sub_assemblers().empty()) {
    auto d_rhs = ts->EqualityAffineTermRHS();
    y += d_rhs;
  }
  kkt.SolveSolverRHS(y);

  // delta = r - P(W^{1/2})(b_theta + A*y)
  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  RowSpace slack_dir = addScaled(b_theta, row, 1.0, 1.0);
  delta = addScaled(r, quadraticRepresentation(sqrtW, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);

  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

std::pair<double, double> VerifyHybridREquations(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    double theta,
    const RowSpace& d,
    const RowSpace& delta,
    const Eigen::VectorXd& y) {
  RowSpace b_theta = BlendAffine(kkt, b, theta);
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // --- Primal check ---
  // delta should equal r - P(W^{1/2})(b_theta + A*y).
  auto y_rhs = kkt.MakeSolverRHS();
  y_rhs = kkt.MakeBlockVariable(y);
  RowSpace Ay = kkt.MakeRowSpace();
  kkt.MultiplyA(y_rhs, Ay);
  RowSpace slack = addScaled(b_theta, Ay, 1.0, 1.0);
  RowSpace delta_expected = addScaled(r,
      quadraticRepresentation(sqrtW, slack), 1.0, -1.0);
  double primal_res = normInf(addScaled(delta, delta_expected, 1.0, -1.0));

  // --- Dual check ---
  // From the KKT solve: (A'W²A + Q)y = RHS.
  // Lambda = P(W^{1/2})(r + delta) = 2*P(W^{1/2})(r) - P(W)(b_theta + Ay).
  // Therefore: A^T lambda - Qy = c  (stationarity, ignoring d_eq).
  // With d_eq: A^T lambda - Qy + d_eq = c.
  // Check: A^T lambda - Qy - c = 0  (d_eq enters the KKT dual block).
  RowSpace lambda = quadraticRepresentation(sqrtW,
      addScaled(r, delta, 1.0, 1.0));
  auto at_lambda = kkt.MakeSolverRHS();
  at_lambda.SetZero();
  kkt.AccumulateAtranspose(lambda, at_lambda);

  // - Qy
  auto qy = kkt.MakeSolverRHS();
  qy.SetZero();
  kkt.AccumulateQx(y_rhs, qy);
  at_lambda -= qy;

  // - c
  at_lambda -= cost_rhs;

  int n = kkt.number_of_variables();
  Eigen::VectorXd dual_err(n);
  at_lambda.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

  return {primal_res, dual_res};
}

GeodesicResult SolveGeodesicHybridR(
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
  double theta = 1.0;

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  if (verbose) {
    printf("  %3s %6s  %12s  %10s %10s  %12s  %6s  %6s\n",
           "it", "theta", "gap", "d_pre", "d_post", "|r|^2/m",
           "r_upd", "step");
    printf("  %s\n", std::string(90, '-').c_str());
  }

  RowSpace last_delta = kkt.MakeRowSpace();

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = ComputeHybridRDirection(kkt, cost_rhs, b, W, r, theta,
                                         d, delta);
    last_delta = delta;
    total_sol++;
    double d_inf_pre = info.d_inf;
    g = info.gap;
    d_inf = info.d_inf;

    if (d_inf > 10 || !std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: diverging (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);
      break;
    }

    // r-update allowed only when well-centered:
    //   d_inf <= 1 + theta  (near-feasible for the blended problem)
    //   gap <= theta * m    (complementarity proportional to theta)
    // Otherwise center (W-update).
    bool can_update_r = (d_inf <= 1.0 + theta) && (g >= 0) &&
                        (g <= theta * m);
    bool do_center = !can_update_r;

    if (do_center) {
      // W-update: centering step.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;

      // Decrease theta only when post-center state is well-centered
      // at the current theta (d_inf near 1 after recompute).
      // The recompute below will give us d_inf; we defer theta decrease
      // to after the recompute.
    } else {
      // Free r-update (no theta change).
      shrinkR(r, delta);
      r_updates_since_fac++;
    }

    // Recompute direction at updated state.
    {
      RowSpace d2 = kkt.MakeRowSpace();
      RowSpace delta2 = kkt.MakeRowSpace();
      auto info2 = ComputeHybridRDirection(kkt, cost_rhs, b, W, r, theta,
                                            d2, delta2);
      last_delta = delta2;
      g = info2.gap;
      d_inf = info2.d_inf;
      total_sol++;

      // Decrease theta after centering, but only if well-centered
      // at the current theta: d_inf <= 1 + 0.1*theta.
      if (do_center && d_inf <= 1.0 + 0.1 * theta) {
        double mu_r = squaredNorm(r) / m;
        double theta_new = std::min(theta, mu_r);
        if (theta_new < theta) theta = theta_new;
      }
    }

    if (verbose) {
      printf("  %3d %6.4f  %12.4e  %10.4e %10.4e  %12.4e  %6d  %s\n",
             iter, theta, g, d_inf_pre, d_inf,
             squaredNorm(r) / m, r_updates_since_fac,
             do_center ? "center" : "r+theta");
    }

    if (theta < tolerance && std::abs(g) < tolerance && d_inf <= 1.001)
      break;
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x.
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    RowSpace b_theta = BlendAffine(kkt, b, theta);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

    auto y = kkt.MakeSolverRHS();
    y = cost_rhs;
    RowSpace v = quadraticRepresentation(W, b_theta);
    kkt.AccumulateAtranspose(v, y);
    y *= -1;
    v = quadraticRepresentation(sqrtW, r);
    v *= 2.0;
    kkt.AccumulateAtranspose(v, y);
    auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
    if (ts && !ts->equality_sub_assemblers().empty()) {
      y += ts->EqualityAffineTermRHS();
    }
    kkt.SolveSolverRHS(y);
    int nr = kkt.number_of_variables();
    result.x.resize(nr);
    y.supernodes->GatherInto(result.x);
  }

  // Lambda and optimality.
  {
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lambda = quadraticRepresentation(sqrtW,
        addScaled(r, last_delta, 1.0, 1.0));
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
    result.lambda = lambda;
  }

  return result;
}

}  // namespace conex
