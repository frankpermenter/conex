#include "conex/algorithms/self_dual_embedding.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

// Conventions:
//   c = GetAffineTerm(), b = cost vector, A = our constraint matrices.
//   G = A^T Q(W) A  (Gram, what AssembleAndFactor builds).
//
// Embedding: lambda = sqrtmu*Q(W^{1/2})(e+d), s = sqrtmu*Q(W^{-1/2})(e-d),
//            tau = sqrtmu*wt*(1+dt), kappa = sqrtmu/wt*(1-dt).
//
// Eq1: Ay + tau c + s = mu e
// Eq2: A^T lambda = tau b
// Eq4: b^T y - <c, lambda> + kappa = 0
//
// Define ytilde = y / sqrtmu. Then the (n+1) system for (ytilde, dt)
// is AFFINE in sqrtmu:
//   G ytilde + wt(AT_QWc - b) dt = wt(b - AT_QWc) - 2 AT_W + sqrtmu AT_QWe
//   Scalar eq from Eq4 (also affine in sqrtmu after substitution).
//
// So d(sqrtmu) = d0 + sqrtmu * d1 where d0, d1 come from solving at sqrtmu=0,1.

HSDResult SolveHSD(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tol,
    bool verbose) {
  const int m = W.total_rows();
  const int n = kkt.number_of_variables();
  const int rank = m;

  RowSpace c = kkt.GetAffineTerm();
  RowSpace e = kkt.MakeRowSpace();
  setOnes(e);

  Eigen::VectorXd b_vec(n);
  cost_rhs.supernodes->GatherInto(b_vec);
  auto b_rhs = kkt.MakeSolverRHS();
  b_rhs = kkt.MakeBlockVariable(b_vec);

  double wt = 1.0;
  HSDResult result;

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();

    RowSpace QWc = quadraticRepresentation(W, c);
    RowSpace QWe = quadraticRepresentation(W, e);

    auto AT_QWc = kkt.MakeSolverRHS(); AT_QWc.SetZero();
    kkt.AccumulateAtranspose(QWc, AT_QWc);

    auto AT_QWe = kkt.MakeSolverRHS(); AT_QWe.SetZero();
    kkt.AccumulateAtranspose(QWe, AT_QWe);

    auto AT_W = kkt.MakeSolverRHS(); AT_W.SetZero();
    kkt.AccumulateAtranspose(W, AT_W);

    double wc = dot(W, c);
    double cQWc = dot(c, QWc);
    double cQWe = dot(c, QWe);
    double ce = dot(c, e);

    // System for (ytilde, dt):
    //   G ytilde + S12_coeff * dt = rhs_n0 + sqrtmu * rhs_n1
    // where:
    //   rhs_n0 = wt*(b - AT_QWc) - 2*AT_W
    //   rhs_n1 = AT_QWe
    //   S12_coeff = wt*(AT_QWc - b)   [coefficient of dt, independent of sqrtmu]

    auto rhs_n0 = kkt.MakeSolverRHS(); rhs_n0.SetZero();
    { auto tmp = b_rhs; tmp *= wt; rhs_n0 += tmp; }
    { auto tmp = AT_QWc; tmp *= -wt; rhs_n0 += tmp; }
    { auto tmp = AT_W; tmp *= -2; rhs_n0 += tmp; }

    auto rhs_n1 = kkt.MakeSolverRHS(); rhs_n1.SetZero();
    rhs_n1 += AT_QWe;

    auto S12_coeff = kkt.MakeSolverRHS(); S12_coeff.SetZero();
    { auto tmp = AT_QWc; S12_coeff += tmp; }
    { auto tmp = b_rhs; tmp *= -1; S12_coeff += tmp; }
    S12_coeff *= wt;

    auto Ginv_S12 = kkt.MakeSolverRHS(); Ginv_S12 = S12_coeff;
    kkt.SolveSolverRHS(Ginv_S12);

    // Eq4 scalar (in terms of ytilde):
    //   (b^T - AT_QWc^T) ytilde - (wt cQWc + 1/wt) dt
    //     = 2 wc + wt cQWc - sqrtmu cQWe - 1/wt + sqrtmu(ce + 1)  ???
    // Let me re-derive. Eq4: b^T y - <c, lambda> + kappa = 0.
    // y = sqrtmu * ytilde.
    // <c, lambda> = sqrtmu [2 wc + AT_QWc^T ytilde + wt(1+dt) cQWc - sqrtmu cQWe]
    // kappa = sqrtmu/wt (1-dt)
    //
    // Eq4 / sqrtmu:
    //   b^T ytilde - 2 wc - AT_QWc^T ytilde - wt(1+dt) cQWc + sqrtmu cQWe + (1/wt)(1-dt) = 0
    //
    // Group unknowns (ytilde, dt):
    //   (b - AT_QWc)^T ytilde - (wt cQWc + 1/wt) dt = 2 wc + wt cQWc - sqrtmu cQWe - 1/wt
    //
    // This is: S21^T ytilde - S22 dt = rhs_s0 + sqrtmu * rhs_s1
    //   S21 = b_vec - AT_QWc_vec
    //   S22 = wt cQWc + 1/wt
    //   rhs_s0 = 2 wc + wt cQWc - 1/wt
    //   rhs_s1 = -cQWe + (ce + 1)  ... wait, where does (ce+1) come from?
    //
    // Let me redo:
    //   b^T yt - 2wc - AT_QWc^T yt - wt cQWc - wt dt cQWc + sq cQWe + 1/wt - dt/wt = 0
    //   (b - AT_QWc)^T yt - (wt cQWc + 1/wt) dt = 2wc + wt cQWc - sq cQWe - 1/wt
    //   Hmm, no (ce+1) term. Let me recheck <c, lambda>.
    //
    // lambda = sqrtmu Q(W^{1/2})(e + d), d = e + Q(W^{1/2})(slack),
    // slack = A ytilde + wt(1+dt)c - sqrtmu e.
    // e + d = 2e + Q(W^{1/2})(slack).
    // Q(W^{1/2})(e+d) = Q(W^{1/2})(2e) + Q(W^{1/2})(Q(W^{1/2})(slack))
    //                  = 2W + Q(W)(slack)
    // <c, lambda> = sqrtmu <c, 2W + Q(W)(A yt + wt(1+dt)c - sq e)>
    //             = sqrtmu [2wc + <c, Q(W)A> yt + wt(1+dt) cQWc - sq cQWe]
    //             = sqrtmu [2wc + AT_QWc^T yt + wt cQWc + wt dt cQWc - sq cQWe]
    //
    // kappa = sqrtmu/wt (1-dt)
    //
    // Eq4/sqrtmu: b^T yt - 2wc - AT_QWc^T yt - wt cQWc - wt dt cQWc + sq cQWe - 1/wt + dt/wt = 0
    //
    // Hmm, kappa = sq/wt(1-dt), so kappa/sqrtmu = 1/wt - dt/wt. Sign of dt term:
    //   + 1/wt(1-dt) = 1/wt - dt/wt
    //
    // So: (b-AT_QWc)^T yt - (wt cQWc + 1/wt) dt = 2wc + wt cQWc + sq cQWe ... no:
    //   ... = 2wc + wt cQWc - sq cQWe - 1/wt
    //
    // Wait, I have a sign error on cQWe. From <c,lambda>:
    //   sq [... - sq cQWe]
    // Divided by sq: ... - sq cQWe. And Eq4/sq:
    //   b^T yt - [2wc + AT_QWc^T yt + wt cQWc + wt dt cQWc - sq cQWe] + 1/wt - dt/wt = 0
    //   (b-AT_QWc)^T yt + (- wt cQWc - 1/wt) dt = 2wc + wt cQWc - 1/wt - sq cQWe
    //   (b-AT_QWc)^T yt - (wt cQWc + 1/wt) dt = 2wc + wt cQWc - 1/wt - sq cQWe

    Eigen::VectorXd AT_QWc_vec(n);
    AT_QWc.supernodes->GatherInto(AT_QWc_vec);
    Eigen::VectorXd S21_vec = b_vec - AT_QWc_vec;

    double S22 = wt * cQWc + 1.0 / wt;
    double rhs_s0 = 2 * wc + wt * cQWc - 1.0 / wt;
    double rhs_s1 = -cQWe;  // coefficient of sqrtmu

    Eigen::VectorXd Ginv_S12_vec(n);
    Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);
    double S21_Ginv_S12 = S21_vec.dot(Ginv_S12_vec);
    double schur_denom = -S22 - S21_Ginv_S12;  // -(S22 + S21 G^{-1} S12)
    if (std::abs(schur_denom) < 1e-30) schur_denom = -1e-30;

    // Solve at a given sqrtmu for (ytilde, dt).
    auto solve_yt = [&](double sq, Eigen::VectorXd& yt_out, double& dt_out) {
      // RHS for n-eq at dt=0: G yt = rhs_n0 + sq * rhs_n1
      auto rhs = kkt.MakeSolverRHS();
      rhs = rhs_n0;
      { auto tmp = rhs_n1; tmp *= sq; rhs += tmp; }

      auto Ginv_rhs = kkt.MakeSolverRHS(); Ginv_rhs = rhs;
      kkt.SolveSolverRHS(Ginv_rhs);
      Eigen::VectorXd Ginv_rhs_vec(n);
      Ginv_rhs.supernodes->GatherInto(Ginv_rhs_vec);

      double S21_Ginv_rhs = S21_vec.dot(Ginv_rhs_vec);
      double scalar_rhs = rhs_s0 + sq * rhs_s1;

      // Schur complement: (S21 G^{-1} S12 + S22) dt = S21 G^{-1} rhs - scalar_rhs
      // Wait: from the system:
      //   G yt + S12 dt = rhs_n
      //   S21^T yt - S22 dt = scalar_rhs
      // From first: yt = G^{-1}(rhs_n - S12 dt)
      // Sub into second: S21^T G^{-1}(rhs_n - S12 dt) - S22 dt = scalar_rhs
      //   S21^T G^{-1} rhs_n - (S21^T G^{-1} S12 + S22) dt = scalar_rhs
      //   dt = (S21^T G^{-1} rhs_n - scalar_rhs) / (S21^T G^{-1} S12 + S22)
      dt_out = (S21_Ginv_rhs - scalar_rhs) / (S21_Ginv_S12 + S22);

      yt_out = Ginv_rhs_vec - dt_out * Ginv_S12_vec;
    };

    // Compute d from (ytilde, dt, sqrtmu).
    // slack = A ytilde + wt(1+dt)c - sqrtmu e
    // d = e + Q(W^{1/2})(slack)
    auto compute_d_slack = [&](const Eigen::VectorXd& yt, double dt, double sq,
                               RowSpace& d_out, RowSpace& slack_out) {
      RowSpace Ayt = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(yt);
        kkt.MultiplyA(yr, Ayt); }

      slack_out = Ayt;
      { RowSpace tmp = c; tmp *= wt * (1 + dt); slack_out += tmp; }
      { RowSpace tmp = e; tmp *= -sq; slack_out += tmp; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      d_out = quadraticRepresentation(sqrtW, slack_out);
      d_out += e;
    };

    // Solve at sqrtmu=0 and sqrtmu=1.
    Eigen::VectorXd yt0, yt1;
    double dt0, dt1;
    solve_yt(0.0, yt0, dt0);
    solve_yt(1.0, yt1, dt1);

    RowSpace d_at0 = kkt.MakeRowSpace(), d_at1 = kkt.MakeRowSpace();
    RowSpace slack_at0 = kkt.MakeRowSpace(), slack_at1 = kkt.MakeRowSpace();
    compute_d_slack(yt0, dt0, 0.0, d_at0, slack_at0);
    compute_d_slack(yt1, dt1, 1.0, d_at1, slack_at1);

    // Verify decomposition: d(sq) should equal d_at0 + sq*(d_at1 - d_at0).
    // Test at sq=0.5:
    if (verbose && iter == 0) {
      Eigen::VectorXd yt_half; double dt_half;
      solve_yt(0.5, yt_half, dt_half);
      RowSpace d_half_direct = kkt.MakeRowSpace(), s_half = kkt.MakeRowSpace();
      compute_d_slack(yt_half, dt_half, 0.5, d_half_direct, s_half);

      RowSpace d_half_interp = addScaled(d_at0, d_at1 - d_at0, 1.0, 0.5);
      double interp_err = normInf(d_half_direct - d_half_interp);

      double dt_half_interp = dt0 + 0.5 * (dt1 - dt0);
      double dt_interp_err = std::abs(dt_half - dt_half_interp);

      printf("  Decomposition check at sq=0.5: ||d_direct - d_interp||_inf = %.2e, "
             "|dt_direct - dt_interp| = %.2e\n",
             interp_err, dt_interp_err);
    }

    // Line search: d(sq) = d_at0 + sq * (d_at1 - d_at0).
    // dt(sq) = dt0 + sq * (dt1 - dt0).
    // Find smallest sq with ||(d, dt)||_inf <= 1.
    // Reparameterize: t = 1 - sq, d(t) = d_at1 + t*(d_at0 - d_at1).
    // lineSearchK finds largest t with ||d_at1 + t*(d_at0-d_at1)||_inf <= 1.
    RowSpace d_diff = d_at0 - d_at1;
    double t_cone = lineSearchK(d_at1, d_diff);

    double dt_diff = dt0 - dt1;
    double t_tau = 0.0;
    if (std::abs(dt_diff) > 1e-15) {
      double k_lo = (-1.0 - dt1) / dt_diff;
      double k_hi = (1.0 - dt1) / dt_diff;
      if (k_lo > k_hi) std::swap(k_lo, k_hi);
      double lo = std::max(k_lo, 0.0);
      double hi = std::min(k_hi, 1.0);
      t_tau = (lo <= hi) ? hi : 0.0;
    } else {
      t_tau = (std::abs(dt1) <= 1.0) ? 1.0 : 0.0;
    }

    double t_best = std::min(t_cone, t_tau);
    t_best = std::max(t_best, 0.0);
    t_best = std::min(t_best, 1.0);
    double sqrtmu = 1.0 - t_best;

    // Interpolate.
    Eigen::VectorXd yt_final = yt0 + sqrtmu * (yt1 - yt0);
    double d_tau = dt0 + sqrtmu * (dt1 - dt0);
    Eigen::VectorXd y_sol = sqrtmu * yt_final;

    RowSpace d_final = kkt.MakeRowSpace(), slack_final = kkt.MakeRowSpace();
    compute_d_slack(yt_final, d_tau, sqrtmu, d_final, slack_final);
    double dinf = std::max(normInf(d_final), std::abs(d_tau));

    double alpha = std::min(1.0, 2.0 / (dinf * dinf));
    if (dinf < 1e-14) alpha = 1.0;

    double tau = sqrtmu * wt * (1.0 + d_tau);
    double kappa = sqrtmu / wt * (1.0 - d_tau);

    result.iterations = iter + 1;
    result.mu = sqrtmu * sqrtmu;
    result.tau = tau;
    result.kappa = kappa;
    result.d_inf = dinf;
    result.y = y_sol;

    if (verbose) {
      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "d_tau=%.2e  alpha=%.4f  sqrtmu=%.2e  "
             "d0_inf=%.2e  d1_inf=%.2e  dt0=%.2e  dt1=%.2e\n",
             iter, sqrtmu * sqrtmu, tau, kappa, dinf,
             d_tau, alpha, sqrtmu,
             normInf(d_at0), normInf(d_at1), dt0, dt1);
    }

    // Termination.
    if (tau > 0 && kappa > 0) {
      if (tau / kappa > 1e6) {
        result.solved = true;
        result.primal_obj = b_vec.dot(y_sol) / tau;
        return result;
      }
      if (kappa / tau > 1e6) {
        result.solved = false;
        return result;
      }
    }

    geodesicUpdateFromSlack(W, alpha, slack_final);
    wt *= std::exp(alpha * d_tau);
  }

  return result;
}

}  // namespace conex
