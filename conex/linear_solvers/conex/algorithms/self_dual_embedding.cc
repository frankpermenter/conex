#include "conex/algorithms/self_dual_embedding.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

// Our conventions:
//   Constraint: A x + c >= 0  (c = GetAffineTerm)
//   Cost: min b^T x           (b = cost_rhs)
//   Gram: G = A^T Q(W) A      (what AssembleAndFactor builds)
//
// Embedding parameterization:
//   lambda = sqrtmu * Q(W^{1/2})(e + d)   [dual]
//   s      = sqrtmu * Q(W^{-1/2})(e - d)  [slack]
//   tau    = sqrtmu * wt * (1 + dt)
//   kappa  = sqrtmu / wt * (1 - dt)
//
// Embedding equations:
//   Eq1: A y + tau c + s = mu e
//   Eq2: A^T lambda = tau b
//   Eq4: b^T y - <c, lambda> + kappa = 0
//
// From Eq1: slack_raw = (A y)/sqrtmu + wt(1+dt) c - sqrtmu e
//           d = e + Q(W^{1/2})(slack_raw)
//
// Substituting d into Eq2 gives the (n+1) system for (y, dt).
// The n equations (from Eq2):
//   (1/sqrtmu) G y + wt [A^T Q(W) c + b] dt
//       = wt b - wt A^T Q(W) c + sqrtmu A^T Q(W) e - 2 A^T W
//
// The scalar equation (from Eq4):
//   b^T y - sqrtmu <c, Q(W^{1/2})(e+d)> + sqrtmu/wt (1-dt) = 0
//
// Expanding <c, lambda> = sqrtmu <c, 2W + Q(W)(slack)>:
//   = sqrtmu [2<c,W> + <c, Q(W)(Ay/sqrtmu + wt(1+dt)c - sqrtmu e)>]
//   = sqrtmu [2<c,W> + (1/sqrtmu)<c,Q(W)A>y + wt(1+dt)<c,Q(W)c> - sqrtmu<c,Q(W)e>]
//
// So Eq4 becomes:
//   b^T y - [2 sqrtmu <c,W> + <c,Q(W)A>y + sqrtmu wt(1+dt)<c,Q(W)c>
//            - sqrtmu^2 <c,Q(W)e>] + sqrtmu/wt(1-dt) = 0
//
// Group by unknowns:
//   [b^T - <c,Q(W)A>^T] y - [sqrtmu wt <c,Q(W)c> + sqrtmu/wt] dt
//   = 2 sqrtmu <c,W> + sqrtmu wt <c,Q(W)c> - sqrtmu^2 <c,Q(W)e>
//     - sqrtmu/wt + sqrtmu(<c,e> + 1)  ... [need to work this out]
//
// Rather than derive the scalar eq analytically, I'll solve the n-system
// (from Eq2) for y as a function of dt, then use Eq4 as the scalar
// equation for dt (Schur complement).

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

    auto AT_e = kkt.MakeSolverRHS(); AT_e.SetZero();
    kkt.AccumulateAtranspose(e, AT_e);

    double wc = dot(W, c);
    double cQWc = dot(c, QWc);
    double cQWe = dot(c, QWe);
    double ce = dot(c, e);

    // From Eq2: (1/sqrtmu) G y + wt [AT_QWc + b] dt = RHS
    // Multiply through by sqrtmu: G y + sqrtmu wt [AT_QWc + b] dt = sqrtmu RHS
    //
    // But G y is what SolveSolverRHS inverts. The RHS for the n-system:
    //   rhs_n(sqrtmu) = sqrtmu * (wt b - wt AT_QWc + sqrtmu AT_QWe - 2 AT_W)
    //
    // Wait, let me be careful. The Eq2-derived system is:
    //   (1/sqrtmu) G y + wt(AT_QWc + b) dt = wt b - wt AT_QWc + sqrtmu AT_QWe - 2 AT_W
    //
    // Multiply by sqrtmu to get standard form G ỹ = f where ỹ = y:
    //   G y = sqrtmu [wt b - wt AT_QWc + sqrtmu AT_QWe - 2 AT_W]
    //         - sqrtmu wt (AT_QWc + b) dt
    //
    // Hmm, this means the RHS depends on both sqrtmu and dt. The standard
    // approach: solve for y(dt) from the n-eq, substitute into Eq4 for dt.
    //
    // Let rhs0 = the RHS at dt=0:
    //   G y = sqrtmu [wt b - wt AT_QWc + sqrtmu AT_QWe - 2 AT_W]
    // And S12 = -sqrtmu wt (AT_QWc + b)  (coefficient of dt).
    //
    // Then y(dt) = G^{-1}(rhs0 + S12 dt) = G^{-1} rhs0 + dt G^{-1} S12.
    //
    // Actually I want f(sqrtmu) linear in sqrtmu for line search.
    // rhs0(sqrtmu) = sqrtmu wt b - sqrtmu wt AT_QWc + sqrtmu^2 AT_QWe - 2 sqrtmu AT_W
    //
    // Split: rhs_n = sqrtmu * rhs_n1 + sqrtmu^2 * rhs_n2
    //   rhs_n1 = wt b - wt AT_QWc - 2 AT_W
    //   rhs_n2 = AT_QWe
    //   S12(sqrtmu) = -sqrtmu wt (AT_QWc + b)
    //
    // For fixed sqrtmu, the n-system at dt=0:
    //   G y = sqrtmu rhs_n1 + sqrtmu^2 rhs_n2
    // And S12 = -sqrtmu wt (AT_QWc + b).
    //
    // For the Schur complement on dt, I need Eq4.

    // Let me just solve at two values of sqrtmu and use lineSearchK.
    // For each sqrtmu: solve for (y, dt) and compute d.

    auto solve_for = [&](double sq, Eigen::VectorXd& y_out, double& dt_out) {
      // n-system RHS at dt=0: G y0 = sq*(wt b - wt AT_QWc - 2 AT_W) + sq^2 * AT_QWe
      auto rhs0 = kkt.MakeSolverRHS(); rhs0.SetZero();
      { auto tmp = b_rhs; tmp *= sq * wt; rhs0 += tmp; }
      { auto tmp = AT_QWc; tmp *= -sq * wt; rhs0 += tmp; }
      { auto tmp = AT_W; tmp *= -2 * sq; rhs0 += tmp; }
      { auto tmp = AT_QWe; tmp *= sq * sq; rhs0 += tmp; }

      auto Ginv_rhs0 = kkt.MakeSolverRHS(); Ginv_rhs0 = rhs0;
      kkt.SolveSolverRHS(Ginv_rhs0);

      // S12 = sq * wt * (b - AT_QWc)  [coefficient of dt in G y = ...]
      auto S12 = kkt.MakeSolverRHS(); S12.SetZero();
      S12 += b_rhs;
      { auto tmp = AT_QWc; tmp *= -1; S12 += tmp; }
      S12 *= sq * wt;

      auto Ginv_S12 = kkt.MakeSolverRHS(); Ginv_S12 = S12;
      kkt.SolveSolverRHS(Ginv_S12);

      // Now use Eq4 to find dt.
      // Eq4: b^T y - <c, lambda> + kappa = 0
      // lambda = sq * Q(W^{1/2})(e + d), kappa = sq/wt * (1 - dt)
      //
      // d depends on y via: slack = Ay/sq + wt(1+dt)c - sq e
      //                     d = e + Q(W^{1/2})(slack)
      //
      // <c, lambda> = sq <c, Q(W^{1/2})(e+d)>
      //             = sq <c, 2W + Q(W)(slack)>
      //             = sq [2 wc + <c, Q(W)(Ay/sq + wt(1+dt)c - sq e)>]
      //             = sq [2 wc + (1/sq)<c, Q(W) A y> + wt(1+dt) cQWc - sq cQWe]
      //
      // <c, Q(W) A y> = <QWc, A y> = QWc^T · (A y).
      // In variable space: A^T QWc · y = AT_QWc^T y.
      //
      // So: <c, lambda> = 2 sq wc + AT_QWc^T y + sq wt(1+dt) cQWc - sq^2 cQWe
      //
      // kappa = sq/wt (1 - dt)
      //
      // Eq4: b^T y - 2 sq wc - AT_QWc^T y - sq wt(1+dt) cQWc + sq^2 cQWe + sq/wt (1-dt) = 0
      //
      // Group: [b^T - AT_QWc^T] y - [sq wt cQWc + sq/wt] dt
      //   = 2 sq wc + sq wt cQWc - sq^2 cQWe - sq/wt
      //
      // Note: [b^T - AT_QWc^T] y = S21^T y where S21 = b_vec - AT_QWc_vec.
      // And [sq wt cQWc + sq/wt] = S22.

      Eigen::VectorXd AT_QWc_vec(n);
      AT_QWc.supernodes->GatherInto(AT_QWc_vec);
      Eigen::VectorXd S21_vec = b_vec - AT_QWc_vec;

      double S22 = sq * wt * cQWc + sq / wt;

      double rhs_scalar = 2 * sq * wc + sq * wt * cQWc - sq * sq * cQWe - sq / wt;

      // y(dt) = Ginv_rhs0 + dt * Ginv_S12.
      Eigen::VectorXd Ginv_rhs0_vec(n), Ginv_S12_vec(n);
      Ginv_rhs0.supernodes->GatherInto(Ginv_rhs0_vec);
      Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);

      // S21^T y(dt) = S21^T Ginv_rhs0 + dt S21^T Ginv_S12
      double s21_rhs0 = S21_vec.dot(Ginv_rhs0_vec);
      double s21_s12 = S21_vec.dot(Ginv_S12_vec);

      // Eq4 scalar: s21_rhs0 + dt s21_s12 - S22 dt = rhs_scalar
      // dt (s21_s12 - S22) = rhs_scalar - s21_rhs0
      double schur = s21_s12 - S22;
      if (std::abs(schur) < 1e-30) schur = 1e-30;
      dt_out = (rhs_scalar - s21_rhs0) / schur;

      // y = Ginv_rhs0 + dt * Ginv_S12
      y_out = Ginv_rhs0_vec + dt_out * Ginv_S12_vec;
    };

    auto compute_d = [&](const Eigen::VectorXd& y_sol, double dt, double sq,
                         RowSpace& d_out, RowSpace& slack_out) {
      // slack = Ay/sq + wt(1+dt)c - sq e
      RowSpace Ay = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay); }

      slack_out = Ay;
      if (sq > 1e-30) slack_out *= 1.0 / sq;
      { RowSpace tmp = c; tmp *= wt * (1 + dt); slack_out += tmp; }
      { RowSpace tmp = e; tmp *= -sq; slack_out += tmp; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      d_out = quadraticRepresentation(sqrtW, slack_out);
      d_out += e;
    };

    Eigen::VectorXd y0, y1;
    double dt0, dt1;
    solve_for(0.0, y0, dt0);
    solve_for(1.0, y1, dt1);

    RowSpace d0 = kkt.MakeRowSpace(), d1 = kkt.MakeRowSpace();
    RowSpace slack0 = kkt.MakeRowSpace(), slack1 = kkt.MakeRowSpace();
    // At sqrtmu=0 the slack has Ay/0 which is degenerate.
    // Use sqrtmu=eps instead.
    double eps = 1e-6;
    solve_for(eps, y0, dt0);
    compute_d(y0, dt0, eps, d0, slack0);
    compute_d(y1, dt1, 1.0, d1, slack1);

    // Line search via lineSearchK.
    RowSpace d_rev = d0 - d1;
    double t_max = lineSearchK(d1, d_rev);

    double dt_rev = dt0 - dt1;
    double t_tau = 0.0;
    if (std::abs(dt_rev) > 1e-15) {
      double k_lo = (-1.0 - dt1) / dt_rev;
      double k_hi = (1.0 - dt1) / dt_rev;
      if (k_lo > k_hi) std::swap(k_lo, k_hi);
      double lo = std::max(k_lo, 0.0);
      double hi = std::min(k_hi, 1.0);
      t_tau = (lo <= hi) ? hi : 0.0;
    } else {
      t_tau = (std::abs(dt1) <= 1.0) ? 1.0 : 0.0;
    }

    double t_best = std::min(t_max, t_tau);
    t_best = std::max(t_best, 0.0);
    t_best = std::min(t_best, 1.0);
    // sqrtmu = eps + t_best * (1 - eps)  (interpolate between eps and 1)
    double sqrtmu = eps + t_best * (1.0 - eps);
    // Recompute at final sqrtmu.
    Eigen::VectorXd y_sol;
    double d_tau;
    solve_for(sqrtmu, y_sol, d_tau);

    RowSpace d_final = kkt.MakeRowSpace(), slack_final = kkt.MakeRowSpace();
    compute_d(y_sol, d_tau, sqrtmu, d_final, slack_final);
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

    // === Verify equations ===
    if (verbose) {
      double mu_val = sqrtmu * sqrtmu;
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace ones = kkt.MakeRowSpace(); setOnes(ones);

      // lambda = sqrtmu * Q(W^{1/2})(e + d)
      RowSpace lambda = quadraticRepresentation(sqrtW, ones + d_final);
      lambda *= sqrtmu;

      // s via Eq1: s = mu e - A y - tau c
      RowSpace Ay_v = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay_v); }
      RowSpace s_eq1 = e;
      s_eq1 *= mu_val;
      { RowSpace tmp = Ay_v; tmp *= -1; s_eq1 += tmp; }
      { RowSpace tmp = c; tmp *= -tau; s_eq1 += tmp; }

      // s from parameterization: sqrtmu * Q(W^{-1/2})(e-d)
      // Hard to compute. Check Eq1 via: A y + tau c + s = mu e
      // where s = sqrtmu * Q(W^{-1/2})(e-d).
      // Equivalently: Q(W^{1/2})(Eq1/sqrtmu):
      //   Q(W^{1/2})(Ay/sqrtmu + wt(1+dt)c + Q(W^{-1/2})(e-d)) = sqrtmu Q(W^{1/2})(e)
      //   Q(W)(Ay/sqrtmu) + wt(1+dt)Q(W)(c) + (e-d) = sqrtmu W
      // But Q(W)(Ay/sqrtmu) is hard. Just check if s_eq1 is in the cone.
      double s_min = minEigenvalue(s_eq1);

      // Eq2: A^T lambda = tau b
      auto at_lambda = kkt.MakeSolverRHS(); at_lambda.SetZero();
      kkt.AccumulateAtranspose(lambda, at_lambda);
      auto rhs_eq2 = kkt.MakeSolverRHS();
      rhs_eq2 = b_rhs; rhs_eq2 *= tau;
      auto res2 = kkt.MakeSolverRHS();
      res2 = at_lambda;
      { auto tmp = rhs_eq2; tmp *= -1; res2 += tmp; }
      Eigen::VectorXd res2_vec(n);
      res2.supernodes->GatherInto(res2_vec);
      double eq2_err = res2_vec.norm();

      // Eq4: b^T y - <c, lambda> + kappa = 0
      double bty = b_vec.dot(y_sol);
      double c_lam = dot(c, lambda);
      double eq4_err = std::abs(bty - c_lam + kappa);

      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "d_tau=%.2e  alpha=%.4f  sqrtmu=%.2e  "
             "eq2=%.1e eq4=%.1e s_min=%.1e\n",
             iter, mu_val, tau, kappa, dinf,
             d_tau, alpha, sqrtmu, eq2_err, eq4_err, s_min);
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

    // For the geodesic update, we need the slack in the original form:
    // d = e + Q(W^{1/2})(slack), update via geodesicUpdateFromSlack(W, alpha, slack).
    // But our slack_final = Ay/sqrtmu + wt(1+dt)c - sqrtmu e, not the
    // "raw" slack that geodesicUpdateFromSlack expects (which is just S in Ax+c=S).
    // The sqrt-free update wants: W_new = exp(alpha(I + W*S)) * W where S is
    // the raw constraint slack. Here the "slack" for the geodesic is the one
    // that appears inside Q(W^{1/2}): d = e + Q(W^{1/2})(slack_final).
    // geodesicUpdateFromSlack computes: W_new = exp(alpha(I + W*slack)) * W.
    // This requires "slack" such that d = I + Q(W^{1/2})(slack) = I + W^{1/2} slack W^{1/2}.
    // So our slack_final IS the right input.
    geodesicUpdateFromSlack(W, alpha, slack_final);
    wt *= std::exp(alpha * d_tau);
  }

  return result;
}

}  // namespace conex
