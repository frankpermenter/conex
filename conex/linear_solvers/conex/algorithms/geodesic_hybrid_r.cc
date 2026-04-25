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
    RowSpace& delta,
    Eigen::VectorXd* y_out) {
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

  // Optionally return the solve vector.
  if (y_out) {
    int nv = kkt.number_of_variables();
    y_out->resize(nv);
    y.supernodes->GatherInto(*y_out);
  }

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

HybridRDecomposition ComputeHybridRDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r) {
  double theta = 0;  // theta not used for the three raw solves
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  int nv = kkt.number_of_variables();
  RowSpace v = kkt.MakeRowSpace();

  // Three-solve decomposition (hybrid_theta_continuation.tex §2).
  //
  //   rhs0: 2*A'P(W^{1/2})(r)                               → x0
  //   rhs1: -(c + A'P(W)(b)) + d_eq                          → x1
  //   rhs2: (c + A'P(W)(b)) - d_eq - A'(e + P(W)(e))         → x_theta
  //
  // At fixed point (W=I, r=e): rhs0 + rhs1 + rhs2 = 0.
  //
  // Full direction: x(tau, theta) = x0 + tau*x1 + theta*x_theta
  //
  // Two-term decomposition (§3) with theta fixed:
  //   f = x0 + theta*x_theta   (tau-free)
  //   g = x1                    (tau-proportional)

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);

  // rhs0 = 2*A'P(W^{1/2})(r)
  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v = quadraticRepresentation(sqrtW, r);
  v *= 2.0;
  kkt.AccumulateAtranspose(v, rhs0);

  // rhs1 = -(c + A'P(W)(b)) + d_eq
  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v = quadraticRepresentation(W, b);
  kkt.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts && !ts->equality_sub_assemblers().empty()) {
    rhs1 += ts->EqualityAffineTermRHS();
  }

  // rhs2 = -rhs1 - A'(e + P(W)(e))
  //      = (c + A'P(W)(b)) - d_eq - A'(e + P(W)(e))
  auto rhs2 = kkt.MakeSolverRHS();
  rhs2 = rhs1;
  rhs2 *= -1;  // (c + A'P(W)(b)) - d_eq
  v = addScaled(ones, quadraticRepresentation(W, ones), 1.0, 1.0);
  v *= -1.0;   // -(e + P(W)(e))
  kkt.AccumulateAtranspose(v, rhs2);

  // Solve all three with one factorization.
  auto y = kkt.MakeSolverRHS(3);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  y.SetColumn(2, rhs2);
  kkt.SolveSolverRHS(y);

  Eigen::MatrixXd y_dense(nv, 3);
  y.supernodes->GatherInto(y_dense);

  // Multiply A * [x0, x1, x_theta].
  auto row = kkt.MakeRowSpace(3);
  kkt.MultiplyA(y, row);

  RowSpace ax0 = kkt.MakeRowSpace();
  RowSpace ax1 = kkt.MakeRowSpace();
  RowSpace ax_th = kkt.MakeRowSpace();
  ax0.col() = row.col(0);
  ax1.col() = row.col(1);
  ax_th.col() = row.col(2);

  HybridRDecomposition decomp;

  // Store raw three-solve components.
  decomp.x0 = y_dense.col(0);
  decomp.x1 = y_dense.col(1);
  decomp.x_theta = y_dense.col(2);

  // lambda(tau,theta) = lam0 + tau*lam1 + theta*lam_theta
  // where lam_i = P(W^{1/2})(r + delta_i) for the constant part,
  // and lam_i = -P(W)(stuff) for tau/theta parts.
  //
  // r + delta = 2r - P(W^{1/2})(A*x + tau*b + theta*(e-b))
  // so lambda = P(W^{1/2})(2r) - P(W)(A*x + tau*b + theta*(e-b))
  //           = [P(W^{1/2})(2r) - P(W)(A*x0)]
  //             + tau*[-P(W)(A*x1 + b)]
  //             + theta*[-P(W)(A*x_theta + e - b)]
  RowSpace e_minus_b = addScaled(ones, b, 1.0, -1.0);
  RowSpace Psqrt2r = quadraticRepresentation(sqrtW, r);
  Psqrt2r *= 2.0;
  decomp.lam0 = addScaled(Psqrt2r,
      quadraticRepresentation(W, ax0), 1.0, -1.0);
  RowSpace arg1 = addScaled(ax1, b, 1.0, 1.0);
  decomp.lam1 = quadraticRepresentation(W, arg1);
  decomp.lam1 *= -1.0;
  RowSpace arg_th = addScaled(ax_th, e_minus_b, 1.0, 1.0);
  decomp.lam_theta = quadraticRepresentation(W, arg_th);
  decomp.lam_theta *= -1.0;

  // delta_cost = -P(W^{1/2})(b + A*x1) — independent of theta.
  decomp.delta_cost = quadraticRepresentation(sqrtW, arg1);
  decomp.delta_cost *= -1.0;

  // y_center, y_cost, delta_center are set by SetTheta().
  return decomp;
}

void SetTheta(HybridRDecomposition& decomp,
              KKTSolverBase& kkt,
              const RowSpace& b,
              const RowSpace& W,
              const RowSpace& r,
              double theta) {
  decomp.y_center = decomp.x0 + theta * decomp.x_theta;
  decomp.y_cost = decomp.x1;

  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace ones = kkt.MakeRowSpace(); setOnes(ones);
  RowSpace e_minus_b = addScaled(ones, b, 1.0, -1.0);
  auto f_rhs = kkt.MakeSolverRHS();
  f_rhs = kkt.MakeBlockVariable(decomp.y_center);
  RowSpace Af = kkt.MakeRowSpace();
  kkt.MultiplyA(f_rhs, Af);
  RowSpace dc_arg = addScaled(Af, e_minus_b, 1.0, theta);
  decomp.delta_center = addScaled(r,
      quadraticRepresentation(sqrtW, dc_arg), 1.0, -1.0);
}

HybridRDirection EvalHybridRAtTau(
    KKTSolverBase& kkt,
    const HybridRDecomposition& decomp,
    const RowSpace& r,
    double tau,
    RowSpace& d,
    RowSpace& delta) {
  // delta(tau) = delta_center + tau * delta_cost
  delta = addScaled(decomp.delta_center, decomp.delta_cost, 1.0, tau);
  d = solveLyapunovForD(r, delta);
  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

GeodesicResult SolveGeodesicThetaContinuationR(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tolerance,
    bool verbose,
    ThetaContRSwitchPolicy policy,
    double compl_tol,
    double theta_rate) {
  RowSpace b = kkt.GetAffineTerm();

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);
  const double cone_rank = dot(ones, ones);  // <e, e> = trace(I), correct for PSD
  const double bT_ones = dot(b, ones);
  const double R = bT_ones + 1.0;

  // Duality cost (cost_rhs + equality dual correction).
  auto duality_cost = kkt.MakeSolverRHS();
  duality_cost = cost_rhs;
  auto* ts_init = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (ts_init && !ts_init->equality_sub_assemblers().empty()) {
    duality_cost += ts_init->EqualityAffineTermRHS();
  }

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);
  double theta = 1.0;
  double tau = 1.0;
  double w_tau = 1.0;   // tau scaling (like W for cone variables)
  double r_tau = 1.0;   // tau centering parameter (like r for cone variables)

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  if (verbose) {
    printf("  %3s  %10s  %10s  %8s  %8s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s  %8s  %10s  %10s  %10s  %3s\n",
           "out", "theta", "tau", "w_tau", "r_tau", "d_inf", "d_tau",
           "gap", "dual", "primal", "mu/tau", "eq_err", "norm_err",
           "compl", "th*alpha", "cpl_err", "st");
    printf("  %s\n", std::string(195, '-').c_str());
  }

  RowSpace last_delta = kkt.MakeRowSpace();
  bool need_decomp = true;
  HybridRDecomposition decomp;
  double theta_at_last_w = 1.0;
  double d_tau = 0;  // tau direction, persists across iterations

  for (int iter = 0; iter < max_iterations; ++iter) {
    if (need_decomp) {
      decomp = ComputeHybridRDecomposition(kkt, cost_rhs, b, W, r);
      total_sol += 3;
      need_decomp = false;

      // Joint (tau', theta) selection from gap + normalization.
      // Gap: b'lambda + c'x + x'Qx/tau + kappa = theta*R
      //   where kappa = w_tau^{-1}*r_tau*(1-d_tau).
      // Normalization: N0 + N1*tau + Nth*theta = -alpha.
      // Eliminate theta, substitute into gap, get quadratic in tau.

      RowSpace rp = addScaled(b, ones, 1.0, -1.0);  // b - e

      auto x0_rhs = kkt.MakeSolverRHS();
      x0_rhs = kkt.MakeBlockVariable(decomp.x0);
      auto x1_rhs = kkt.MakeSolverRHS();
      x1_rhs = kkt.MakeBlockVariable(decomp.x1);
      auto xth_rhs = kkt.MakeSolverRHS();
      xth_rhs = kkt.MakeBlockVariable(decomp.x_theta);

      double bTl0 = dot(b, decomp.lam0);
      double bTl1 = dot(b, decomp.lam1);
      double bTlth = dot(b, decomp.lam_theta);
      double cTx0 = duality_cost.dot(x0_rhs);
      double cTx1 = duality_cost.dot(x1_rhs);
      double cTxth = duality_cost.dot(xth_rhs);

      // Normalization coefficients (no Q, always stable).
      RowSpace Ax0_v = kkt.MakeRowSpace(); kkt.MultiplyA(x0_rhs, Ax0_v);
      RowSpace Ax1_v = kkt.MakeRowSpace(); kkt.MultiplyA(x1_rhs, Ax1_v);
      RowSpace Axth_v = kkt.MakeRowSpace(); kkt.MultiplyA(xth_rhs, Axth_v);

      double rpTl0 = dot(rp, decomp.lam0);
      double rpTl1 = dot(rp, decomp.lam1);
      double rpTlth = dot(rp, decomp.lam_theta);
      double rdTx0 = cTx0 - dot(ones, Ax0_v);
      double rdTx1 = cTx1 - dot(ones, Ax1_v);
      double rdTxth = cTxth - dot(ones, Axth_v);
      double rg = -(bT_ones + 1.0);
      // alpha = <e, e> + 1 (trace of identity + 1 for tau/kappa pair).
      double alpha_norm = dot(ones, ones) + 1.0;

      double N0 = rpTl0 + rdTx0;
      double N1 = rpTl1 + rdTx1 + rg;
      double Nth = rpTlth + rdTxth;
      double eta = alpha_norm + N0;
      double Nth_threshold = 1e-12 * (std::abs(N0) + std::abs(N1) + 1.0);
      double n1 = (std::abs(Nth) > Nth_threshold) ? N1 / Nth : 0.0;
      double e1 = (std::abs(Nth) > Nth_threshold) ? eta / Nth : 0.0;

      // Substitute theta(tau) = -(eta + N1*tau)/Nth into the gap equation
      // BEFORE expanding Q terms.  This avoids the six cross-terms
      // (q01, q0th, q1th etc.) that suffer catastrophic cancellation
      // when x1 ≈ -x_theta.
      //
      // With theta(tau), x(tau) = x0 + tau*x1 + theta(tau)*x_theta
      //   = (x0 - e1*x_theta) + tau*(x1 - n1*x_theta)
      //   = f0 + tau*h
      // where f0 = x0 - e1*x_theta, h = x1 - n1*x_theta.
      //
      // Similarly lambda(tau) = lam0 + tau*lam1 + theta(tau)*lam_theta
      //   = (lam0 - e1*lam_theta) + tau*(lam1 - n1*lam_theta)
      //   = l0 + tau*l1
      //
      // Gap × tau = tau*b'lambda + tau*c'x + x'Qx + r_tau^2 - theta*R*tau
      //   = tau^2*(b'l1 + c'h + h'Qh) + tau*(b'l0 + c'f0 + 2*f0'Qh - theta(tau)*R)
      //     + (f0'Qf0 + r_tau^2)
      //
      // But theta(tau)*R*tau = R*(-(eta+N1*tau)/Nth)*tau = -R*(e1*tau + n1*tau^2).
      // So the theta*R*tau term contributes -R*n1 to tau^2 and -R*e1 to tau.
      //
      // Final quadratic: A*tau^2 + B*tau + C = 0 with:
      //   A = b'l1 + c'h + h'Qh + R*n1
      //   B = b'l0 + c'f0 + 2*f0'Qh + R*e1
      //   C = f0'Qf0 + r_tau^2
      double r_tau2 = r_tau * r_tau;

      // Build f0 = x0 - e1*x_theta, h = x1 - n1*x_theta.
      Eigen::VectorXd f0_vec = decomp.x0 - e1 * decomp.x_theta;
      Eigen::VectorXd h_vec = decomp.x1 - n1 * decomp.x_theta;
      auto f0_rhs = kkt.MakeSolverRHS(); f0_rhs = kkt.MakeBlockVariable(f0_vec);
      auto h_rhs = kkt.MakeSolverRHS(); h_rhs = kkt.MakeBlockVariable(h_vec);

      // Q inner products (three terms, all stable).
      auto Qf0 = kkt.MakeSolverRHS(); Qf0.SetZero(); kkt.AccumulateQx(f0_rhs, Qf0);
      auto Qh = kkt.MakeSolverRHS(); Qh.SetZero(); kkt.AccumulateQx(h_rhs, Qh);
      double qff = Qf0.dot(f0_rhs);  // f0'Qf0
      double qfh = Qf0.dot(h_rhs);   // f0'Qh
      double qhh = Qh.dot(h_rhs);    // h'Qh

      // Lambda: l0 = lam0 - e1*lam_theta, l1 = lam1 - n1*lam_theta.
      double bTl0_sub = bTl0 - e1 * bTlth;
      double bTl1_sub = bTl1 - n1 * bTlth;

      // Cost: c'f0 = c'x0 - e1*c'x_theta, c'h = c'x1 - n1*c'x_theta.
      double cTf0 = cTx0 - e1 * cTxth;
      double cTh = cTx1 - n1 * cTxth;

      // tau*kappa = 2*r_tau*tau/w_tau - tau^2/w_tau^2
      // contributes -1/w_tau^2 to A, +2*r_tau/w_tau to B, 0 to C.
      double inv_wt = 1.0 / (w_tau > 1e-30 ? w_tau : 1e-30);
      double A_coeff = bTl1_sub + cTh + qhh + R*n1 - inv_wt*inv_wt;
      double B_coeff = bTl0_sub + cTf0 + 2*qfh + R*e1 + 2*r_tau*inv_wt;
      double C_coeff = qff;

      // Solve for tau'. Pick root that minimizes |d_tau|.
      double tau_new = tau;
      double discr = B_coeff * B_coeff - 4.0 * A_coeff * C_coeff;
      if (discr >= 0 && std::abs(A_coeff) > 1e-30) {
        double sq = std::sqrt(discr);
        double t1 = (-B_coeff + sq) / (2.0 * A_coeff);
        double t2 = (-B_coeff - sq) / (2.0 * A_coeff);
        // Pick root minimizing |d_tau| = |tau'/(w_tau*r_tau) - 1|.
        double wtr = w_tau * r_tau;
        double d1 = (std::abs(wtr) > 1e-30) ? t1 / wtr - 1.0 : 1e30;
        double d2 = (std::abs(wtr) > 1e-30) ? t2 / wtr - 1.0 : 1e30;
        // Prefer positive tau'. Among roots with similar |d_tau|,
        // pick the one giving d_tau > -1 (i.e., tau' > 0).
        bool t1_pos = (t1 > 0);
        bool t2_pos = (t2 > 0);
        if (t1_pos && !t2_pos)
          tau_new = t1;
        else if (t2_pos && !t1_pos)
          tau_new = t2;
        else
          tau_new = (std::abs(d1) < std::abs(d2)) ? t1 : t2;
      }
      tau = tau_new;

      // Compute d_tau.
      double wtr = w_tau * r_tau;
      d_tau = (std::abs(wtr) > 1e-30) ? tau / wtr - 1.0 : 0.0;

      // Recover theta from normalization.
      if (std::abs(Nth) > Nth_threshold) {
        theta = (-alpha_norm - N0 - N1 * tau) / Nth;
      }
      SetTheta(decomp, kkt, b, W, r, theta);
    }

    // Evaluate cone direction at current (tau, r, theta).
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = EvalHybridRAtTau(kkt, decomp, r, tau, d, delta);
    last_delta = delta;
    g = info.gap;
    d_inf = std::max(info.d_inf, std::abs(d_tau));

    // Record per-iteration stats.
    result.iter_stats.push_back({squaredNorm(r) / cone_rank, d_inf, info.d_sq,
        g, r_updates_since_fac, info.min_slack, theta, total_fac});

    if (verbose) {
      // Diagnostics: gap equation, normalization, dual residual, complementarity.
      RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam_v = quadraticRepresentation(sqrtW_v,
          addScaled(r, delta, 1.0, 1.0));
      Eigen::VectorXd x_vec = decomp.y_center + tau * decomp.y_cost;
      auto x_rhs = kkt.MakeSolverRHS();
      x_rhs = kkt.MakeBlockVariable(x_vec);
      auto qx = kkt.MakeSolverRHS(); qx.SetZero();
      kkt.AccumulateQx(x_rhs, qx);

      double bTl = dot(b, lam_v);
      double cTx = duality_cost.dot(x_rhs);
      double xQx = qx.dot(x_rhs);
      double mu_v = squaredNorm(r) / cone_rank;
      double kappa_v = r_tau * (1.0 - d_tau) / std::max(w_tau, 1e-30);
      double xQx_over_tau = (std::abs(tau) > 1e-30) ? xQx / tau : 0.0;
      double eq_err = std::abs(bTl + cTx + xQx_over_tau + kappa_v - theta * R);
      double half_xQx_phys = (std::abs(tau) > 1e-30) ? 0.5 * xQx / (tau*tau) : 0.0;
      double primal_phys = (std::abs(tau) > 1e-30) ? cTx/tau + half_xQx_phys : 0.0;
      double dual_phys = (std::abs(tau) > 1e-30) ? -(bTl/tau + half_xQx_phys) : 0.0;

      // Normalization: rp'lambda + rd'x + rg*tau vs -alpha.
      double eTl = dot(ones, lam_v);
      RowSpace Ax_v = kkt.MakeRowSpace(); kkt.MultiplyA(x_rhs, Ax_v);
      double norm_val = (bTl - eTl) + (cTx - dot(ones, Ax_v))
                        + (-(bT_ones + 1.0)) * tau;
      double alpha_v = dot(ones, ones) + 1.0;
      double norm_err = norm_val - (-alpha_v);

      // Complementarity: gap + tau*kappa vs theta*alpha.
      double tau_kappa = r_tau*r_tau*(1.0 - d_tau*d_tau);
      double compl_err_v = (info.gap + tau_kappa) - theta * alpha_v;

      printf("  %3d  %10.2e  %10.2e  %8.4f  %8.4f  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %12.2e  %8.2e  %10.4e  %10.4e  %10.2e  %3d\n",
             iter, theta, tau, w_tau, r_tau, d_inf, d_tau, g,
             dual_phys, primal_phys, mu_v/std::max(std::abs(tau), 1e-30),
             eq_err, norm_err, info.gap + tau_kappa, theta * alpha_v,
             compl_err_v, r_updates_since_fac);
    }

    if (!std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: nan (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);
      break;
    }

    result.iterations = iter + 1;
    if (std::abs(theta) < tolerance && std::abs(g) < tolerance && d_inf <= 1.001)
      break;

    // Complementarity check: gap + r_tau^2 should = theta * alpha.
    // Once this degrades beyond tolerance, freeze W (only r-updates).
    double alpha_check = dot(ones, ones) + 1.0;
    double tau_kappa_check = r_tau*r_tau*(1.0 - d_tau*d_tau);
    double compl_err = std::abs(info.gap + tau_kappa_check - theta * alpha_check);
    bool w_frozen = (compl_err > compl_tol);

    // Theta-rate check: center if theta hasn't decreased enough since
    // last W-update.  After at least 2 r-updates, if |theta| > theta_rate *
    // |theta_at_last_w|, the r-updates are stalling and a W-update is needed.
    bool theta_stalled = false;
    if (theta_rate > 0 && r_updates_since_fac >= 2) {
      theta_stalled = (std::abs(theta) > theta_rate * std::abs(theta_at_last_w));
    }

    bool do_center = !w_frozen && (policy(g, d_inf, r_updates_since_fac)
                                    || theta_stalled);
    if (do_center) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      w_tau *= std::exp(d_tau * alpha);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;
      theta_at_last_w = theta;
      need_decomp = true;
    } else {
      shrinkR(r, delta);
      // Shrink r_tau: r_tau = r_tau/2 * (1 + |d_tau|).
      // This mirrors shrinkR: r_new = (r + |delta|) / 2 ≈ r*(1+|d|)/2.
      r_tau = 0.5 * r_tau * (1.0 + std::abs(d_tau));
      r_updates_since_fac++;
      need_decomp = true;
    }
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / cone_rank;
  result.complementarity = g;
  result.tau = tau;
  result.kappa = (tau > 1e-30) ? theta / tau : std::numeric_limits<double>::infinity();
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x (de-homogenized by tau).
  // W hasn't changed since last factorization (convergence check is before step).
  {
    Eigen::VectorXd x_lifted = decomp.y_center + tau * decomp.y_cost;
    result.x = x_lifted / tau;
  }

  // Lambda and optimality.
  {
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lambda = quadraticRepresentation(sqrtW,
        addScaled(r, last_delta, 1.0, 1.0));
    if (tau > 0 && tau != 1.0) lambda *= (1.0 / tau);
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
    result.lambda = lambda;
  }

  return result;
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

    bool do_center = (g < 0);

    if (do_center) {
      // W-update: centering step. Keep r and theta frozen.
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;
    } else {
      // r-update + theta update.
      shrinkR(r, delta);
      r_updates_since_fac++;
      theta = std::abs(g) / m;
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
