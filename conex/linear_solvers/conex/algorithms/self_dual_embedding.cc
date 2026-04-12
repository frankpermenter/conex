#include "conex/algorithms/self_dual_embedding.h"

#include <cmath>
#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

HSDResult SolveHSD(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_iterations,
    double tol,
    bool verbose) {
  const int m = W.total_rows();
  const int n = kkt.number_of_variables();

  // c = affine term (constraint offset), e = identity element.
  RowSpace c = kkt.GetAffineTerm();
  RowSpace e = kkt.MakeRowSpace();
  setOnes(e);

  // wt = 1/√μ (barrier scaling for τ).
  double wt = 1.0;

  // Gather cost vector b from cost_rhs.
  Eigen::VectorXd b_vec(n);
  cost_rhs.supernodes->GatherInto(b_vec);

  HSDResult result;
  int rank = m;  // Total rank of cone (sum of dimensions).

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();

    // Compute quantities needed for the embedding RHS.
    // Qc = P(W)(c) = quadraticRepresentation(W, c).
    RowSpace Qc = quadraticRepresentation(W, c);
    // Qe = P(W)(e) = W² for nonneg, W·I·W for PSD.
    RowSpace Qe = quadraticRepresentation(W, e);

    // A^T quantities (in SolverRHS format).
    auto AW = kkt.MakeSolverRHS();
    AW.SetZero();
    kkt.AccumulateAtranspose(W, AW);

    auto AQc = kkt.MakeSolverRHS();
    AQc.SetZero();
    kkt.AccumulateAtranspose(Qc, AQc);

    auto AQe = kkt.MakeSolverRHS();
    AQe.SetZero();
    kkt.AccumulateAtranspose(Qe, AQe);

    auto Ae = kkt.MakeSolverRHS();
    Ae.SetZero();
    kkt.AccumulateAtranspose(e, Ae);

    // Scalar inner products.
    double wc = dot(W, c);            // <W, c>
    double cQc = dot(c, Qc);          // <c, P(W)(c)>
    double cQe = dot(c, Qe);          // <c, P(W)(e)>
    double ce = dot(c, e);            // <c, e>

    // === Line search: d(sqrtmu) = d0 + sqrtmu*d1, find largest sqrtmu
    //     with ||d||_inf <= dinf_bound. ===
    double dinf_bound = 0.99;

    // Helper: solve the extended system for a given sqrtmu, return (y, d_tau).
    // The RHS and Schur complement are affine in sqrtmu, so we solve at
    // sqrtmu=0 and sqrtmu=1, then interpolate.

    // Compute S12, S21, S22 (independent of sqrtmu).
    auto S12 = kkt.MakeSolverRHS();
    S12 = AQc;
    S12 *= -wt;
    { auto tmp = cost_rhs; tmp *= wt; S12 += tmp; }

    auto Ginv_S12 = kkt.MakeSolverRHS();
    Ginv_S12 = S12;
    kkt.SolveSolverRHS(Ginv_S12);

    double S22 = wt * cQc + 1.0 / wt;

    Eigen::VectorXd AQc_vec(n), Ginv_S12_vec(n);
    AQc.supernodes->GatherInto(AQc_vec);
    Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);
    Eigen::VectorXd S21_vec = -b_vec - AQc_vec;
    double S21_Ginv_S12 = S21_vec.dot(Ginv_S12_vec);
    double schur = S22 - S21_Ginv_S12;
    if (std::abs(schur) < 1e-30) schur = 1e-30;

    // f1(sqrtmu) = f1_0 + sqrtmu * f1_1.
    // f1_0 = -2*AW + wt*(-cost + AQc)
    auto f1_0 = kkt.MakeSolverRHS();
    f1_0 = AW; f1_0 *= -2.0;
    { auto tmp = cost_rhs; tmp *= -wt; f1_0 += tmp; }
    { auto tmp = AQc; tmp *= wt; f1_0 += tmp; }

    // f1_1 = AQe - AQc + Ae + cost
    auto f1_1 = kkt.MakeSolverRHS();
    f1_1 = AQe;
    { auto tmp = AQc; tmp *= -1; f1_1 += tmp; }
    f1_1 += Ae;
    f1_1 += cost_rhs;

    // f2(sqrtmu) = f2_0 + sqrtmu * f2_1.
    double f2_0 = 1.0 / wt + 2.0 * wc - wt * cQc;
    double f2_1 = -(cQe - cQc) - (ce + 1.0);

    // Solve at sqrtmu=0 and sqrtmu=1.
    auto solve_for_sqrtmu = [&](double sqrtmu,
                                Eigen::VectorXd& y_out, double& d_tau_out) {
      auto f1 = kkt.MakeSolverRHS();
      f1 = f1_0;
      { auto tmp = f1_1; tmp *= sqrtmu; f1 += tmp; }
      double f2 = f2_0 + sqrtmu * f2_1;

      auto Ginv_f1 = kkt.MakeSolverRHS();
      Ginv_f1 = f1;
      kkt.SolveSolverRHS(Ginv_f1);
      Eigen::VectorXd Ginv_f1_vec(n);
      Ginv_f1.supernodes->GatherInto(Ginv_f1_vec);

      double S21_Ginv_f1 = S21_vec.dot(Ginv_f1_vec);
      d_tau_out = (f2 - S21_Ginv_f1) / schur;

      auto y_rhs = kkt.MakeSolverRHS();
      y_rhs = f1;
      { auto tmp = S12; tmp *= d_tau_out; y_rhs -= tmp; }
      kkt.SolveSolverRHS(y_rhs);
      y_out.resize(n);
      y_rhs.supernodes->GatherInto(y_out);
    };

    // Compute d and dinf for a given (y, d_tau, sqrtmu).
    auto compute_dinf = [&](const Eigen::VectorXd& y_sol, double d_tau,
                            double sqrtmu, RowSpace& slack_out) -> double {
      double c_weight = wt * (1.0 + d_tau) - sqrtmu;
      double w_weight = sqrtmu;

      RowSpace Ay = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr = kkt.MakeBlockVariable(y_sol);
        kkt.MultiplyA(yr, Ay); }

      slack_out = Ay;
      { RowSpace tmp = c; tmp *= -c_weight; slack_out += tmp; }
      { RowSpace tmp = e; tmp *= -w_weight; slack_out += tmp; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      RowSpace d = quadraticRepresentation(sqrtW, slack_out);
      d += e;
      double di = normInf(d);
      return std::max(di, std::abs(d_tau));
    };

    // Solve at sqrtmu=0 and sqrtmu=1.
    Eigen::VectorXd y0, y1;
    double dt0, dt1;
    solve_for_sqrtmu(0.0, y0, dt0);
    solve_for_sqrtmu(1.0, y1, dt1);

    // Compute d0 = d(sqrtmu=0) and d1 = d(sqrtmu=1) as RowSpaces.
    RowSpace slack0 = kkt.MakeRowSpace();
    RowSpace slack1 = kkt.MakeRowSpace();
    (void)compute_dinf(y0, dt0, 0.0, slack0);
    (void)compute_dinf(y1, dt1, 1.0, slack1);

    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace d_0 = quadraticRepresentation(sqrtW, slack0);
    d_0 += e;
    RowSpace d_1 = quadraticRepresentation(sqrtW, slack1);
    d_1 += e;

    // d(sqrtmu) = d_1 + (1-sqrtmu) * (d_0 - d_1).
    // Reparametrize: let t = 1 - sqrtmu. Then d(t) = d_1 + t*(d_0 - d_1).
    // lineSearchK finds largest t with ||d_1 + t*(d_0-d_1)||_inf <= 1.
    // Then sqrtmu = 1 - t (smallest sqrtmu with dinf <= 1).
    RowSpace d_rev = d_0 - d_1;
    double t_max = lineSearchK(d_1, d_rev);

    // Also bound t by d_tau: |dt1 + t*(dt0-dt1)| <= 1.
    double dt_rev = dt0 - dt1;
    double t_tau = 0.0;  // default: no progress on tau
    if (std::abs(dt_rev) > 1e-15) {
      double k_lo = (-1.0 - dt1) / dt_rev;
      double k_hi = (1.0 - dt1) / dt_rev;
      if (k_lo > k_hi) std::swap(k_lo, k_hi);
      // Intersect [k_lo, k_hi] with [0, 1].
      double lo = std::max(k_lo, 0.0);
      double hi = std::min(k_hi, 1.0);
      if (lo <= hi) {
        t_tau = hi;  // largest feasible t
      } else {
        t_tau = 0.0;  // no feasible t in [0,1]
      }
    } else {
      // dt_rev ≈ 0: d_tau is constant. Check if |dt1| <= 1.
      t_tau = (std::abs(dt1) <= 1.0) ? 1.0 : 0.0;
    }

    double t_best = std::min(t_max, t_tau);
    t_best = std::max(t_best, 0.0);
    t_best = std::min(t_best, 1.0);
    double sqrtmu = 1.0 - t_best;
    if (verbose) {
      printf("    LS: t_max=%.4f t_tau=%.4f t_best=%.4f sqrtmu=%.4f "
             "dt0=%.4f dt1=%.4f\n",
             t_max, t_tau, t_best, sqrtmu, dt0, dt1);
    }

    // Interpolate y, d_tau, slack at the chosen sqrtmu.
    Eigen::VectorXd y_sol = y0 + sqrtmu * (y1 - y0);
    double d_tau = dt0 + sqrtmu * (dt1 - dt0);

    // Recompute slack at the chosen sqrtmu.
    RowSpace slack_best = kkt.MakeRowSpace();
    double dinf = compute_dinf(y_sol, d_tau, sqrtmu, slack_best);

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
    result.y = y_sol * tau;

    // === Verify embedding equations ===
    // Construct primal/dual variables from (W, d, d_tau):
    //   x = sqrtmu * P(W^{1/2})(e + d)
    //   s = sqrtmu * P(W^{-1/2})(e - d)
    //   λ = s (dual cone variable)
    //   τ = sqrtmu * wt * (1 + d_tau)
    //   κ = sqrtmu * (1/wt) * (1 - d_tau)
    //
    // Embedding equations:
    //   (1) τ*c - A*y = s + μ*(c - e)
    //   (2) A^T*x = τ*b + μ*(A^T*e - b)   [b = cost vector]
    //   (3) <x, s> + κ*τ = μ*(rank + 1)
    //   (4) b^T*y - <c, x> = κ
    if (verbose) {
      double mu_val = sqrtmu * sqrtmu;
      RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
      RowSpace ones_v = kkt.MakeRowSpace();
      setOnes(ones_v);

      // d from slack_best.
      RowSpace d_v = quadraticRepresentation(sqrtW_v, slack_best);
      d_v += ones_v;

      // x = sqrtmu * P(W^{1/2})(e + d)
      RowSpace e_plus_d = ones_v + d_v;
      RowSpace x_v = quadraticRepresentation(sqrtW_v, e_plus_d);
      x_v *= sqrtmu;

      // s = sqrtmu * P(W^{-1/2})(e - d)
      // P(W^{-1/2})(v) = P(W^{-1})(P(W^{1/2})(v))... complex.
      // Instead: s = τ*c - A*y - μ*(c - e)  (from eq 1, if it held).
      // Check eq 1 directly: residual = τ*c - A*y - s - μ*(c - e).
      // But we don't have s independently. Use the Newton equation:
      //   The Newton direction satisfies the linearized system.
      //   Check: d was constructed as d = e + P(W^{1/2})(slack)
      //   where slack = Ay - c_weight*c - w_weight*e.
      //   c_weight = wt*(1+d_tau) - sqrtmu, w_weight = sqrtmu.
      //
      // Equation (2): A^T * x = τ*b + μ*(A^T e - b)
      // A^T x:
      auto atx = kkt.MakeSolverRHS();
      atx.SetZero();
      kkt.AccumulateAtranspose(x_v, atx);
      // RHS: τ*b + μ*(A^T e - b) = (τ-μ)*b + μ*A^T e
      auto rhs2 = kkt.MakeSolverRHS();
      rhs2 = cost_rhs;
      rhs2 *= (tau - mu_val);
      {
        auto tmp = kkt.MakeSolverRHS();
        tmp.SetZero();
        kkt.AccumulateAtranspose(ones_v, tmp);
        tmp *= mu_val;
        rhs2 += tmp;
      }
      // Residual = A^T x - RHS.
      rhs2 *= -1;
      rhs2 += atx;
      Eigen::VectorXd res2(n);
      rhs2.supernodes->GatherInto(res2);
      double eq2_err = res2.norm();

      // Equation (4): b^T y - <c, x> = κ
      double bty = b_vec.dot(y_sol);
      double cx = dot(c, x_v);
      double eq4_err = std::abs(bty - cx - kappa);

      // Equation (3): <x, s> + κτ = μ*(rank+1)
      // We need <x,s>. Since x = sqrtmu*P(W^{1/2})(e+d) and
      // s should equal sqrtmu*P(W^{-1/2})(e-d), we have
      // <x,s> = mu * <e+d, e-d> = mu * (rank - ||d||^2).
      double d_sq = squaredNorm(d_v);
      double xs = mu_val * (rank - d_sq);
      double eq3_err = std::abs(xs + kappa * tau - mu_val * (rank + 1));

      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "d_tau=%.2e  alpha=%.4f  sqrtmu=%.2e  "
             "eq2=%.1e eq3=%.1e eq4=%.1e\n",
             iter, mu_val, tau, kappa, dinf,
             d_tau, alpha, sqrtmu, eq2_err, eq3_err, eq4_err);
    }

    // Check termination (only when tau, kappa are meaningful).
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

    // Geodesic update for W.
    geodesicUpdateFromSlack(W, alpha, slack_best);

    // Update wt.
    wt *= std::exp(alpha * d_tau);
  }

  return result;
}

}  // namespace conex
