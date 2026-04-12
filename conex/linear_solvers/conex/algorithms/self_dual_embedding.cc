#include "conex/algorithms/self_dual_embedding.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

// Convention mapping:
//
// Old conex:  max b_old^T y   s.t.  A_old y <= c_old
//   Slack: s = c_old - A_old y >= 0
//
// Our framework:  min cost^T x  s.t.  A x + affineTerm >= 0
//   Slack: s = A x + affineTerm >= 0
//
// Mapping:
//   A_old = -A_ours         (sign flip: Ay <= c  →  -Ax + c >= 0  →  Ax + c >= 0 with A=-A_old)
//   c_old = affineTerm      (our GetAffineTerm())
//   b_old = -cost           (max b^T y = min (-b)^T y)
//   y_old = x_ours
//
// So: A_old^T v = -A_ours^T v  (AccumulateAtranspose gives A_ours^T, need to negate)
//     b_old = -cost_vec
//
// The embedding equations (old convention):
//   τ c_old - A_old y = s + μ(c_old - e)
//   A_old^T x = τ b_old + μ(A_old^T e - b_old)
//   <x, s> + κτ = μ(rank + 1)
//   b_old^T y - <c_old, x> = κ
//
// The RHS from BuildRHS (old code):
//   f(1:n) = wt*(b_old + A_old^T Q(W) c_old) + sqrtmu*(A_old^T Q(W) e - A_old^T Q(W) c_old + A_old^T e - b_old) - 2 A_old^T W
//   f(n+1) = 1/wt + 2<W, c_old> - wt*<c_old, Q(W) c_old> - sqrtmu*(<c_old, Q(W) e> - <c_old, Q(W) c_old>) - sqrtmu*(<c_old, e> + 1)
//
// In our primitives (using A_old^T v = -AccAtrans(v) and b_old = -cost):
//   A_old^T Q(W) c = -AccAtrans(Q(W)(affineTerm))
//   A_old^T Q(W) e = -AccAtrans(Q(W)(e))
//   A_old^T e = -AccAtrans(e)
//   A_old^T W = -AccAtrans(W)

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

  RowSpace c_old = kkt.GetAffineTerm();  // c_old = affineTerm
  RowSpace e = kkt.MakeRowSpace();
  setOnes(e);

  // b_old = -cost_vec.
  Eigen::VectorXd cost_vec(n);
  cost_rhs.supernodes->GatherInto(cost_vec);
  Eigen::VectorXd b_old_vec = -cost_vec;

  double wt = 1.0;
  HSDResult result;

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();

    // Cone quantities.
    RowSpace QWc = quadraticRepresentation(W, c_old);  // Q(W)(c_old)
    RowSpace QWe = quadraticRepresentation(W, e);       // Q(W)(e) = W²

    // A_old^T quantities = -A_ours^T quantities.
    // AoldT_QWc = A_old^T Q(W) c_old = -AccAtrans(QWc)
    auto AoldT_QWc = kkt.MakeSolverRHS(); AoldT_QWc.SetZero();
    kkt.AccumulateAtranspose(QWc, AoldT_QWc);
    AoldT_QWc *= -1;  // negate for A_old^T

    auto AoldT_QWe = kkt.MakeSolverRHS(); AoldT_QWe.SetZero();
    kkt.AccumulateAtranspose(QWe, AoldT_QWe);
    AoldT_QWe *= -1;

    auto AoldT_e = kkt.MakeSolverRHS(); AoldT_e.SetZero();
    kkt.AccumulateAtranspose(e, AoldT_e);
    AoldT_e *= -1;

    auto AoldT_W = kkt.MakeSolverRHS(); AoldT_W.SetZero();
    kkt.AccumulateAtranspose(W, AoldT_W);
    AoldT_W *= -1;

    // b_old as SolverRHS.
    auto b_old_rhs = kkt.MakeSolverRHS();
    b_old_rhs = kkt.MakeBlockVariable(b_old_vec);

    // Scalar inner products (these don't involve A, no sign issues).
    double wc = dot(W, c_old);
    double cQWc = dot(c_old, QWc);
    double cQWe = dot(c_old, QWe);
    double ce = dot(c_old, e);

    // === Build f(sqrtmu) = f0 + sqrtmu * f1 ===
    // f0(1:n) = wt*(b_old + AoldT_QWc) - 2*AoldT_W
    auto f0 = kkt.MakeSolverRHS(); f0.SetZero();
    { auto tmp = b_old_rhs; tmp *= wt; f0 += tmp; }
    { auto tmp = AoldT_QWc; tmp *= wt; f0 += tmp; }
    { auto tmp = AoldT_W; tmp *= -2; f0 += tmp; }

    // f1(1:n) = AoldT_QWe - AoldT_QWc + AoldT_e - b_old
    auto f1 = kkt.MakeSolverRHS(); f1.SetZero();
    f1 += AoldT_QWe;
    { auto tmp = AoldT_QWc; tmp *= -1; f1 += tmp; }
    f1 += AoldT_e;
    { auto tmp = b_old_rhs; tmp *= -1; f1 += tmp; }

    // f0(n+1) = 1/wt + 2*wc - wt*cQWc
    double f0_scalar = 1.0 / wt + 2.0 * wc - wt * cQWc;
    // f1(n+1) = -(cQWe - cQWc) - (ce + 1)
    double f1_scalar = -(cQWe - cQWc) - (ce + 1.0);

    // === Schur complement for the extended (n+1)x(n+1) system ===
    // S12 = -wt*(AoldT_QWc + b_old)
    auto S12 = kkt.MakeSolverRHS(); S12.SetZero();
    S12 += AoldT_QWc;
    S12 += b_old_rhs;
    S12 *= -wt;

    // G^{-1} S12.
    auto Ginv_S12 = kkt.MakeSolverRHS();
    Ginv_S12 = S12;
    kkt.SolveSolverRHS(Ginv_S12);

    // S22 = wt*cQWc + 1/wt.
    double S22 = wt * cQWc + 1.0 / wt;

    // S21 = (b_old - AoldT_QWc)^T  (as dot product in variable space).
    Eigen::VectorXd AoldT_QWc_vec(n), Ginv_S12_vec(n);
    AoldT_QWc.supernodes->GatherInto(AoldT_QWc_vec);
    Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);
    Eigen::VectorXd S21_vec = b_old_vec - AoldT_QWc_vec;

    double S21_Ginv_S12 = S21_vec.dot(Ginv_S12_vec);
    double schur = S22 - S21_Ginv_S12;
    if (std::abs(schur) < 1e-30) schur = 1e-30;

    // === Solve at sqrtmu=0 and sqrtmu=1, decompose d(sqrtmu) ===
    auto solve_embedding = [&](double sqrtmu,
                               Eigen::VectorXd& y_out, double& dt_out) {
      auto f = kkt.MakeSolverRHS();
      f = f0;
      { auto tmp = f1; tmp *= sqrtmu; f += tmp; }
      double f_scalar = f0_scalar + sqrtmu * f1_scalar;

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

    // Compute d from (y, dt, sqrtmu) using old-convention slack.
    // In old convention: slack = c_old - A_old * y - c_weight * c_old - w_weight * e
    // With c_weight = wt*(1+dt) - sqrtmu, w_weight = sqrtmu:
    //   Newton direction: d = e + P(W^{1/2})(c_old - A_old*y - c_weight*c_old - w_weight*e)
    // But the Newton system gives: the slack for the geodesic step is
    //   slack = -c_weight * c_old - w_weight * e + A_old * y
    //         = -c_weight * c_old - w_weight * e + (-A_ours * y)
    // Wait — in old convention, the slack involves A_old y, and in our framework
    // A_old y = -A_ours y. So:
    //   slack_old = A_old * y - c_weight * c_old - w_weight * e
    //   In our primitives: A_old * y = -MultiplyA(y).
    // And d = e + P(W^{1/2})(slack_old).
    auto compute_d = [&](const Eigen::VectorXd& y_sol, double dt,
                         double sqrtmu_val, RowSpace& d_out,
                         RowSpace& slack_out) {
      double c_weight = wt * (1.0 + dt) - sqrtmu_val;
      double w_weight = sqrtmu_val;

      // A_old * y = -A_ours * y = -MultiplyA(y_rhs).
      RowSpace Ay_old = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay_old); }
      Ay_old *= -1;  // A_old * y

      slack_out = Ay_old;
      { RowSpace tmp = c_old; tmp *= -c_weight; slack_out += tmp; }
      { RowSpace tmp = e; tmp *= -w_weight; slack_out += tmp; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      d_out = quadraticRepresentation(sqrtW, slack_out);
      d_out += e;
    };

    Eigen::VectorXd y0, y1;
    double dt0, dt1;
    solve_embedding(0.0, y0, dt0);
    solve_embedding(1.0, y1, dt1);

    RowSpace d0_rs = kkt.MakeRowSpace(), d1_rs = kkt.MakeRowSpace();
    RowSpace slack0 = kkt.MakeRowSpace(), slack1 = kkt.MakeRowSpace();
    compute_d(y0, dt0, 0.0, d0_rs, slack0);
    compute_d(y1, dt1, 1.0, d1_rs, slack1);

    // Line search: d(sqrtmu) = d_1 + t*(d_0 - d_1) where t = 1 - sqrtmu.
    // Find largest t with ||d||_inf <= 1 using lineSearchK.
    RowSpace d_rev = d0_rs - d1_rs;
    double t_max = lineSearchK(d1_rs, d_rev);

    // d_tau bound: |dt1 + t*(dt0-dt1)| <= 1.
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

    // Take step.
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

    // === Verify embedding equations ===
    if (verbose) {
      double mu_val = sqrtmu * sqrtmu;
      RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
      RowSpace e_plus_d = e + d_final;
      RowSpace x_v = quadraticRepresentation(sqrtW_v, e_plus_d);
      x_v *= sqrtmu;

      // Eq2: A_old^T x = τ*b_old + μ*(A_old^T e - b_old)
      // A_old^T x = -AccAtrans(x)
      auto AoldT_x = kkt.MakeSolverRHS(); AoldT_x.SetZero();
      kkt.AccumulateAtranspose(x_v, AoldT_x);
      AoldT_x *= -1;
      // RHS: τ*b_old + μ*(AoldT_e - b_old) = (τ-μ)*b_old + μ*AoldT_e
      auto rhs2 = kkt.MakeSolverRHS(); rhs2.SetZero();
      { auto tmp = b_old_rhs; tmp *= (tau - mu_val); rhs2 += tmp; }
      { auto tmp = AoldT_e; tmp *= mu_val; rhs2 += tmp; }
      rhs2 *= -1; rhs2 += AoldT_x;
      Eigen::VectorXd res2(n);
      rhs2.supernodes->GatherInto(res2);
      double eq2_err = res2.norm();

      // Eq3: <x,s> + κτ = μ*(rank+1), where <x,s> = μ*(rank - ||d||²)
      double d_sq = squaredNorm(d_final);
      double xs = mu_val * (rank - d_sq);
      double eq3_err = std::abs(xs + kappa * tau - mu_val * (rank + 1));

      // Eq4: b_old^T y - <c_old, x> = κ
      double bty = b_old_vec.dot(y_sol);
      double cx = dot(c_old, x_v);
      double eq4_err = std::abs(bty - cx - kappa);

      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "d_tau=%.2e  alpha=%.4f  sqrtmu=%.2e  "
             "eq2=%.1e eq3=%.1e eq4=%.1e\n",
             iter, mu_val, tau, kappa, dinf,
             d_tau, alpha, sqrtmu, eq2_err, eq3_err, eq4_err);
    }

    // Check termination.
    if (tau > 0 && kappa > 0) {
      if (tau / kappa > 1e6) {
        result.solved = true;
        result.primal_obj = b_old_vec.dot(y_sol) / tau;
        return result;
      }
      if (kappa / tau > 1e6) {
        result.solved = false;
        return result;
      }
    }

    // Geodesic update for W.
    geodesicUpdateFromSlack(W, alpha, slack_final);

    // Update wt.
    wt *= std::exp(alpha * d_tau);
  }

  return result;
}

}  // namespace conex
