#include "conex/algorithms/self_dual_embedding.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

// Conventions (following the embedding equations directly):
//   c = kkt.GetAffineTerm()    (affine term in Ax + c >= 0)
//   b = cost vector            (min b^T y)
//   A = our constraint matrices (kkt.MultiplyA, AccumulateAtranspose)
//
// Embedding variables:
//   s = sqrtmu * P(W^{-1/2})(e - d)
//   x = sqrtmu * P(W^{1/2})(e + d)
//   tau = sqrtmu * wt * (1 + dt)
//   kappa = sqrtmu / wt * (1 - dt)
//
// Embedding equations (from the old conex, adapted to our A,b,c):
//   Eq1: s = tau*c - A*y - mu*(c - e)
//   Eq2: A^T * x = tau*b + mu*(A^T*e - b)
//   Eq4: b^T*y - <c, x> = kappa
//
// Dividing Eq1 by sqrtmu:
//   P(W^{-1/2})(e - d) = wt*(1+dt)*c - (1/sqrtmu)*A*y - sqrtmu*(c - e)
//
// The (n+1) system solves for (y, dt) given sqrtmu, then d is recovered
// from Eq1.

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

  double wt = 1.0;
  HSDResult result;

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();

    // Cone quantities.
    RowSpace QWc = quadraticRepresentation(W, c);
    RowSpace QWe = quadraticRepresentation(W, e);

    // A^T quantities (using our A directly, no sign flip).
    auto AT_QWc = kkt.MakeSolverRHS(); AT_QWc.SetZero();
    kkt.AccumulateAtranspose(QWc, AT_QWc);

    auto AT_QWe = kkt.MakeSolverRHS(); AT_QWe.SetZero();
    kkt.AccumulateAtranspose(QWe, AT_QWe);

    auto AT_e = kkt.MakeSolverRHS(); AT_e.SetZero();
    kkt.AccumulateAtranspose(e, AT_e);

    auto AT_W = kkt.MakeSolverRHS(); AT_W.SetZero();
    kkt.AccumulateAtranspose(W, AT_W);

    auto b_rhs = kkt.MakeSolverRHS();
    b_rhs = kkt.MakeBlockVariable(b_vec);

    double wc = dot(W, c);
    double cQWc = dot(c, QWc);
    double cQWe = dot(c, QWe);
    double ce = dot(c, e);

    // === RHS as f(sqrtmu) = f0 + sqrtmu * f1 ===
    // From the old code BuildRHS, translated with our sign conventions.
    // The old code uses: A_old^T = our -A^T, b_old = our -b.
    // So old "AQc" = A_old^T Q(W) c = -AT_QWc.
    // And old "b" = -b_vec.
    //
    // Old f(1:n) = wt*(b_old + A_old^T Q(W) c) + sqrtmu*(A_old^T Q(W) e - A_old^T Q(W) c + A_old^T e - b_old) - 2*A_old^T W
    // = wt*(-b_rhs - AT_QWc) + sqrtmu*(-AT_QWe + AT_QWc - AT_e + b_rhs) - 2*(-AT_W)
    // = -wt*b_rhs - wt*AT_QWc - sqrtmu*AT_QWe + sqrtmu*AT_QWc - sqrtmu*AT_e + sqrtmu*b_rhs + 2*AT_W
    //
    // f0 = -wt*b_rhs - wt*AT_QWc + 2*AT_W
    // f1 = -AT_QWe + AT_QWc - AT_e + b_rhs

    auto f0 = kkt.MakeSolverRHS(); f0.SetZero();
    { auto tmp = b_rhs; tmp *= -wt; f0 += tmp; }
    { auto tmp = AT_QWc; tmp *= -wt; f0 += tmp; }
    { auto tmp = AT_W; tmp *= 2; f0 += tmp; }

    auto f1 = kkt.MakeSolverRHS(); f1.SetZero();
    { auto tmp = AT_QWe; tmp *= -1; f1 += tmp; }
    f1 += AT_QWc;
    { auto tmp = AT_e; tmp *= -1; f1 += tmp; }
    f1 += b_rhs;

    // Old f(n+1):
    // = 1/wt + 2*<W,c> - wt*<c,Q(W)c> - sqrtmu*(<c,Q(W)e> - <c,Q(W)c>) - sqrtmu*(<c,e> + 1)
    // (no sign issues here — these are inner products in cone space)
    double f0_scalar = 1.0 / wt + 2.0 * wc - wt * cQWc;
    double f1_scalar = -(cQWe - cQWc) - (ce + 1.0);

    // === Schur complement ===
    // Old S12 = -wt*(A_old^T Q(W) c + b_old) = -wt*(-AT_QWc - b_rhs) = wt*(AT_QWc + b_rhs)
    auto S12 = kkt.MakeSolverRHS(); S12.SetZero();
    S12 += AT_QWc;
    S12 += b_rhs;
    S12 *= wt;

    auto Ginv_S12 = kkt.MakeSolverRHS();
    Ginv_S12 = S12;
    kkt.SolveSolverRHS(Ginv_S12);

    // Old S22 = wt*<c,Q(W)c> + 1/wt  (no sign issue)
    double S22 = wt * cQWc + 1.0 / wt;

    // Old S21 = (b_old - A_old^T Q(W) c)^T = (-b_rhs - (-AT_QWc))^T = (AT_QWc - b_rhs)^T
    Eigen::VectorXd AT_QWc_vec(n), Ginv_S12_vec(n);
    AT_QWc.supernodes->GatherInto(AT_QWc_vec);
    Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);
    Eigen::VectorXd S21_vec = AT_QWc_vec - b_vec;

    double schur = S22 - S21_vec.dot(Ginv_S12_vec);
    if (std::abs(schur) < 1e-30) schur = 1e-30;

    // === Solve at sqrtmu=0 and sqrtmu=1 ===
    auto solve_embedding = [&](double sqrtmu_val,
                               Eigen::VectorXd& y_out, double& dt_out) {
      auto f = kkt.MakeSolverRHS();
      f = f0;
      { auto tmp = f1; tmp *= sqrtmu_val; f += tmp; }
      double f_scalar = f0_scalar + sqrtmu_val * f1_scalar;

      auto Ginv_f = kkt.MakeSolverRHS();
      Ginv_f = f;
      kkt.SolveSolverRHS(Ginv_f);
      Eigen::VectorXd Ginv_f_vec(n);
      Ginv_f.supernodes->GatherInto(Ginv_f_vec);

      dt_out = (f_scalar - S21_vec.dot(Ginv_f_vec)) / schur;

      auto y_rhs = kkt.MakeSolverRHS();
      y_rhs = f;
      { auto tmp = S12; tmp *= dt_out; y_rhs -= tmp; }
      kkt.SolveSolverRHS(y_rhs);
      y_out.resize(n);
      y_rhs.supernodes->GatherInto(y_out);
    };

    // Compute d from (y, dt, sqrtmu) via Eq1 (divided by sqrtmu):
    //   P(W^{-1/2})(e-d) = wt*(1+dt)*c - (A*y)/sqrtmu - sqrtmu*(c-e)
    // So: e-d = P(W^{1/2})( wt*(1+dt)*c - (A*y)/sqrtmu - sqrtmu*(c-e) )
    //     d = e - P(W^{1/2})( ... )
    //
    // But for the geodesic update we need slack, not d.
    // Actually the Newton step gives d = e + P(W^{1/2})(slack) where
    // slack is defined by the centering equation.
    //
    // From the old code: c_weight = wt*(1+dt) - sqrtmu, w_weight = sqrtmu.
    // slack_old = A_old*y - c_weight*c - w_weight*e
    //   A_old*y = -A*y (in our primitives).
    // So slack_old = -A*y - c_weight*c - w_weight*e.
    // And d = e + P(W^{1/2})(slack_old).
    auto compute_d = [&](const Eigen::VectorXd& y_sol, double dt,
                         double sqrtmu_val, RowSpace& d_out,
                         RowSpace& slack_out) {
      double c_weight = wt * (1.0 + dt) - sqrtmu_val;
      double w_weight = sqrtmu_val;

      RowSpace Ay = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay); }

      // slack_old = A_old*y - c_weight*c - w_weight*e = -Ay - c_weight*c - w_weight*e
      slack_out = Ay;
      slack_out *= -1;
      { RowSpace tmp = c; tmp *= -c_weight; slack_out += tmp; }
      { RowSpace tmp = e; tmp *= -w_weight; slack_out += tmp; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      d_out = quadraticRepresentation(sqrtW, slack_out);
      d_out += e;
    };

    Eigen::VectorXd y0, y1;
    double dt0, dt1;
    solve_embedding(0.0, y0, dt0);
    solve_embedding(1.0, y1, dt1);

    RowSpace d0 = kkt.MakeRowSpace(), d1 = kkt.MakeRowSpace();
    RowSpace slack0 = kkt.MakeRowSpace(), slack1 = kkt.MakeRowSpace();
    compute_d(y0, dt0, 0.0, d0, slack0);
    compute_d(y1, dt1, 1.0, d1, slack1);

    // Line search: d(sqrtmu) = d1 + t*(d0-d1), t = 1-sqrtmu.
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
    double sqrtmu = 1.0 - t_best;

    Eigen::VectorXd y_sol = y0 + sqrtmu * (y1 - y0);
    double d_tau = dt0 + sqrtmu * (dt1 - dt0);

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

    // === Verify Eq1, Eq2, Eq4 ===
    if (verbose) {
      double mu_val = sqrtmu * sqrtmu;
      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace ones = kkt.MakeRowSpace(); setOnes(ones);

      RowSpace e_plus_d = ones + d_final;
      RowSpace x_v = quadraticRepresentation(sqrtW, e_plus_d);
      x_v *= sqrtmu;

      RowSpace e_minus_d = ones - d_final;
      RowSpace s_v = EuclideanJordanAlgebra::like(W);
      // s = sqrtmu * P(W^{-1/2})(e-d). Hard to compute W^{-1/2}.
      // Instead verify Eq1 as: s = tau*c - A*y - mu*(c-e).
      RowSpace Ay_v = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay_v); }

      // Eq1: s_predicted = tau*c - A*y - mu*(c-e).
      //      s_from_d = sqrtmu * P(W^{-1/2})(e-d).
      // Instead check: P(W^{1/2})(e-d) should equal
      //   (1/sqrtmu) * (tau*c - A*y - mu*(c-e))
      //   = wt*(1+dt)*c - (A*y)/sqrtmu - sqrtmu*(c-e)
      // LHS: P(W^{1/2})(e-d)
      RowSpace lhs1 = quadraticRepresentation(sqrtW, e_minus_d);
      // RHS (if sqrtmu > 0):
      double eq1_err = 0;
      if (sqrtmu > 1e-15) {
        RowSpace rhs1 = c;
        rhs1 *= wt * (1.0 + d_tau);
        { RowSpace tmp = Ay_v; tmp *= -1.0 / sqrtmu; rhs1 += tmp; }
        { RowSpace tmp = c; tmp *= -sqrtmu; rhs1 += tmp; }
        { RowSpace tmp = ones; tmp *= sqrtmu; rhs1 += tmp; }
        RowSpace res1 = lhs1 - rhs1;
        eq1_err = normInf(res1);
      }

      // Eq2: A^T * x = tau*b + mu*(A^T*e - b)
      auto atx = kkt.MakeSolverRHS(); atx.SetZero();
      kkt.AccumulateAtranspose(x_v, atx);
      auto rhs2 = kkt.MakeSolverRHS(); rhs2.SetZero();
      { auto tmp = b_rhs; tmp *= (tau - mu_val); rhs2 += tmp; }
      { auto tmp = AT_e; tmp *= mu_val; rhs2 += tmp; }
      auto res2 = kkt.MakeSolverRHS();
      res2 = atx;
      { auto tmp = rhs2; tmp *= -1; res2 += tmp; }
      Eigen::VectorXd res2_vec(n);
      res2.supernodes->GatherInto(res2_vec);
      double eq2_err = res2_vec.norm();

      // Eq4: b^T*y - <c, x> = kappa
      double bty = b_vec.dot(y_sol);
      double cx = dot(c, x_v);
      double eq4_err = std::abs(bty - cx - kappa);

      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "d_tau=%.2e  alpha=%.4f  sqrtmu=%.2e  "
             "eq1=%.1e eq2=%.1e eq4=%.1e\n",
             iter, mu_val, tau, kappa, dinf,
             d_tau, alpha, sqrtmu, eq1_err, eq2_err, eq4_err);
    }

    // Check termination.
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
