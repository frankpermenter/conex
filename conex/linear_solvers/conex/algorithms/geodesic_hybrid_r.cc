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
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);
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

  kkt.SetScaling(W);
  kkt.AssembleAndFactor();
  int total_fac = 1;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates_since_fac = 0;
  double g = 0, d_inf = 0;

  // Compute V(tau)/tau = beta*tau + (alpha-R) + mu_eff/tau.
  // V(tau) = beta*tau^2 + (alpha-R)*tau + mu_eff = 0 is the duality identity.
  auto computeV = [&](const HybridRDecomposition& dc, double tau_v, double theta_v) {
    RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lam_c = quadraticRepresentation(sqrtW_v,
        addScaled(r, dc.delta_center, 1.0, 1.0));
    RowSpace lam_t = quadraticRepresentation(sqrtW_v, dc.delta_cost);
    double s0 = dot(b, lam_c), s1 = dot(b, lam_t);
    auto yc = kkt.MakeSolverRHS(); yc = kkt.MakeBlockVariable(dc.y_center);
    auto yt = kkt.MakeSolverRHS(); yt = kkt.MakeBlockVariable(dc.y_cost);
    double g0 = duality_cost.dot(yc), g1 = duality_cost.dot(yt);
    auto qyc = kkt.MakeSolverRHS(); qyc.SetZero(); kkt.AccumulateQx(yc, qyc);
    auto qyt = kkt.MakeSolverRHS(); qyt.SetZero(); kkt.AccumulateQx(yt, qyt);
    double qff = qyc.dot(yc), qfg = qyc.dot(yt), qgg = qyt.dot(yt);
    double bt = s1+g1+qgg;
    double aq = s0+g0+2*qfg-theta_v*R;
    double me = qff+theta_v;
    return bt*tau_v + aq + me/tau_v;
  };

  if (verbose) {
    printf("  %3s  %8s  %10s  %12s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s  %8s  %3s\n",
           "out", "theta", "tau", "kappa", "d_inf", "d_sqr",
           "gap", "dual", "primal", "mu/tau", "eq_err", "norm_err", "st");
    printf("  %s\n", std::string(160, '-').c_str());
  }

  RowSpace last_delta = kkt.MakeRowSpace();
  bool need_decomp = true;
  HybridRDecomposition decomp;

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Compute the two-solve decomposition when needed (after W-updates).
    if (need_decomp) {
      decomp = ComputeHybridRDecomposition(kkt, cost_rhs, b, W, r);
      total_sol += 3;
      need_decomp = false;

      // Joint (tau, theta) selection from gap + normalization equations.
      // lambda(tau,theta) = lam0 + tau*lam1 + theta*lam_theta
      // x(tau,theta) = x0 + tau*x1 + theta*x_theta
      //
      // Gap equation (multiply by tau, Q=0 for now):
      //   G1*tau^2 + G0*tau + G_theta*tau*theta + theta = 0
      // Normalization:
      //   N0 + N1*tau + N_theta*theta = -alpha
      //
      // Eliminate theta from normalization, substitute into gap.

      RowSpace ones_sel = kkt.MakeRowSpace(); setOnes(ones_sel);
      RowSpace rp = addScaled(b, ones_sel, 1.0, -1.0);  // b - e

      // Gap coefficients: G_i = b'lam_i + c'x_i  (for Q=0).
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

      // TODO: add Q terms for QP support.
      double G0 = bTl0 + cTx0;       // constant
      double G1 = bTl1 + cTx1;       // coefficient of tau
      double Gth = bTlth + cTxth - R; // coefficient of tau*theta (includes -R)

      // Normalization coefficients: rp'lam_i + rd'x_i + rg*(tau coefficient).
      // rd = c - A'e, so rd'x = c'x - e'(Ax) = c'x - e'(row from A*x).
      // We compute rp'lam_i directly, rd'x_i via duality_cost - e'(Ax_i).
      RowSpace Ax0_v = kkt.MakeRowSpace(); kkt.MultiplyA(x0_rhs, Ax0_v);
      RowSpace Ax1_v = kkt.MakeRowSpace(); kkt.MultiplyA(x1_rhs, Ax1_v);
      RowSpace Axth_v = kkt.MakeRowSpace(); kkt.MultiplyA(xth_rhs, Axth_v);

      double rpTl0 = dot(rp, decomp.lam0);
      double rpTl1 = dot(rp, decomp.lam1);
      double rpTlth = dot(rp, decomp.lam_theta);
      double rdTx0 = cTx0 - dot(ones_sel, Ax0_v);
      double rdTx1 = cTx1 - dot(ones_sel, Ax1_v);
      double rdTxth = cTxth - dot(ones_sel, Axth_v);
      double rg = -(bT_ones + 1.0);
      double alpha_norm = m + 1.0;

      double N0 = rpTl0 + rdTx0;          // constant
      double N1 = rpTl1 + rdTx1 + rg;     // coefficient of tau
      double Nth = rpTlth + rdTxth;        // coefficient of theta

      // Combined quadratic: eliminate theta = (-alpha_norm - N0 - N1*tau) / Nth.
      // Gap*tau: G1*tau^2 + G0*tau + Gth*tau*theta + theta = 0
      // Multiply by Nth:
      //   G1*Nth*tau^2 + G0*Nth*tau + Gth*tau*(-alpha_norm-N0-N1*tau) + (-alpha_norm-N0-N1*tau) = 0
      //   (G1*Nth - Gth*N1)*tau^2 + (G0*Nth - Gth*(alpha_norm+N0) - N1)*tau - (alpha_norm+N0) = 0
      double A_coeff = G1*Nth - Gth*N1;
      double B_coeff = G0*Nth - Gth*(alpha_norm + N0) - N1;
      double C_coeff = -(alpha_norm + N0);

      double discr = B_coeff * B_coeff - 4.0 * A_coeff * C_coeff;
      if (discr >= 0 && std::abs(A_coeff) > 1e-30) {
        double sq = std::sqrt(discr);
        double t1 = (-B_coeff + sq) / (2.0 * A_coeff);
        double t2 = (-B_coeff - sq) / (2.0 * A_coeff);
        if (t1 > 0 && t2 > 0)
          tau = (std::abs(t1 - tau) < std::abs(t2 - tau)) ? t1 : t2;
        else if (t1 > 0) tau = t1;
        else if (t2 > 0) tau = t2;
      }
      // Recover theta from normalization.
      if (std::abs(Nth) > 1e-30) {
        theta = (-alpha_norm - N0 - N1 * tau) / Nth;
      }
      // Update the two-term decomposition at the new theta.
      SetTheta(decomp, kkt, b, W, r, theta);
    }

    // Evaluate direction at current (tau, r, theta).
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = EvalHybridRAtTau(kkt, decomp, r, tau, d, delta);
    last_delta = delta;
    g = info.gap;
    d_inf = info.d_inf;

    if (verbose) {
      // Print BEFORE the step so we see the state that drives the decision.
      RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam_v = quadraticRepresentation(sqrtW_v,
          addScaled(r, delta, 1.0, 1.0));
      double bTl = dot(b, lam_v);
      Eigen::VectorXd x_vec = decomp.y_center + tau * decomp.y_cost;
      auto x_rhs = kkt.MakeSolverRHS();
      x_rhs = kkt.MakeBlockVariable(x_vec);
      double cTx = duality_cost.dot(x_rhs);
      auto qx = kkt.MakeSolverRHS(); qx.SetZero();
      kkt.AccumulateQx(x_rhs, qx);
      double xQx = qx.dot(x_rhs);
      double mu = squaredNorm(r) / m;
      double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;
      double kappa_v = (tau > 1e-30) ? theta / tau : 1e30;
      double xQx_over_tau = (tau > 1e-30) ? xQx / tau : 0.0;
      double theta_over_tau = (tau > 1e-30) ? theta / tau : 0.0;
      double eq_err = std::abs(bTl + cTx + xQx_over_tau
                                + theta_over_tau - theta * R);
      double half_xQx_phys = (tau > 1e-30) ? 0.5 * xQx / (tau * tau) : 0.0;
      double primal_phys = (tau > 1e-30) ? cTx / tau + half_xQx_phys : 0.0;
      double dual_phys = (tau > 1e-30) ? -(bTl / tau + half_xQx_phys) : 0.0;
      // Normalization check: r_p'λ + r_d'x + r_g·τ should = -(m+1).
      // r_p = b - e, r_d = c - A'e, r_g = -(b'e + 1).
      RowSpace ones_v = kkt.MakeRowSpace(); setOnes(ones_v);
      double eTl = dot(ones_v, lam_v);
      RowSpace Ax_v = kkt.MakeRowSpace();
      kkt.MultiplyA(x_rhs, Ax_v);
      double eTAx = dot(ones_v, Ax_v);

      double rpTl = bTl - eTl;       // (b-e)'λ
      double rdTx = cTx - eTAx;      // (c-A'e)'x
      double r_g = -(bT_ones + 1.0); // -(b'e + 1)
      double norm_target = -(m + 1.0);

      double norm_val = rpTl + rdTx + r_g * tau;

      double d_sq = squaredNorm(delta);
      printf("  %3d  %8.6f  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %12.2e  %8.2e  %3d\n",
             iter, theta, tau, kappa_v, d_inf, d_sq, g,
             dual_phys, primal_phys, mu_over_tau, eq_err,
             norm_val - norm_target, r_updates_since_fac);
    }

    if (d_inf > 10 || !std::isfinite(d_inf) || !std::isfinite(g)) {
      if (verbose) printf("  TERMINATED: diverging (d_inf=%.2e, g=%.2e)\n",
                          d_inf, g);
      break;
    }

    result.iterations = iter + 1;
    if (theta < tolerance && std::abs(g) < tolerance && d_inf <= 1.001)
      break;

    bool do_center = (g < 0);
    if (do_center) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      updateAutomorphism(W, r, alpha, d);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_since_fac = 0;
      need_decomp = true;
    } else {
      shrinkR(r, delta);
      r_updates_since_fac++;
      need_decomp = true;
    }
  }

  result.d_inf_norm = d_inf;
  result.d_sq_norm = 0;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.tau = tau;
  result.kappa = (tau > 1e-30) ? theta / tau : std::numeric_limits<double>::infinity();
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x (de-homogenized by tau).
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    // x = y_center + tau * y_cost, then divide by tau.
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
