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

  RowSpace c = kkt.GetAffineTerm();
  RowSpace e = kkt.MakeRowSpace();
  setOnes(e);

  // Old convention: b_old = -cost, A_old^T v = -AccAtrans(v).
  Eigen::VectorXd cost_vec(n);
  cost_rhs.supernodes->GatherInto(cost_vec);
  Eigen::VectorXd b_old = -cost_vec;

  double wt = 1.0;
  HSDResult result;

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();

    RowSpace QWc = quadraticRepresentation(W, c);
    RowSpace QWe = quadraticRepresentation(W, e);

    // A_old^T quantities = -our A^T.
    auto AQc = kkt.MakeSolverRHS(); AQc.SetZero();
    kkt.AccumulateAtranspose(QWc, AQc); AQc *= -1;

    auto AQe = kkt.MakeSolverRHS(); AQe.SetZero();
    kkt.AccumulateAtranspose(QWe, AQe); AQe *= -1;

    auto Ae = kkt.MakeSolverRHS(); Ae.SetZero();
    kkt.AccumulateAtranspose(e, Ae); Ae *= -1;

    auto AW = kkt.MakeSolverRHS(); AW.SetZero();
    kkt.AccumulateAtranspose(W, AW); AW *= -1;

    auto b_rhs = kkt.MakeSolverRHS();
    b_rhs.ScatterFrom(b_old.data(), b_old.size());

    double wc = dot(W, c);
    double cQc = dot(c, QWc);
    double cQe = dot(c, QWe);
    double ce = dot(c, e);

    // f_n(sq) = f_n0 + sq*f_n1 (from old BuildRHS).
    auto f_n0 = kkt.MakeSolverRHS(); f_n0.SetZero();
    { auto t = b_rhs; t *= wt; f_n0 += t; }
    { auto t = AQc; t *= wt; f_n0 += t; }
    { auto t = AW; t *= -2; f_n0 += t; }

    auto f_n1 = kkt.MakeSolverRHS(); f_n1.SetZero();
    f_n1 += AQe;
    { auto t = AQc; t *= -1; f_n1 += t; }
    f_n1 += Ae;
    { auto t = b_rhs; t *= -1; f_n1 += t; }

    double f_s0 = 1.0 / wt + 2 * wc - wt * cQc;
    double f_s1 = -(cQe - cQc) - (ce + 1.0);

    // Schur complement pieces.
    auto S12 = kkt.MakeSolverRHS(); S12.SetZero();
    S12 += AQc; S12 += b_rhs; S12 *= -wt;

    auto Ginv_S12 = kkt.MakeSolverRHS(); Ginv_S12 = S12;
    kkt.SolveSolverRHS(Ginv_S12);
    Eigen::VectorXd Ginv_S12_vec(n);
    Ginv_S12.supernodes->GatherInto(Ginv_S12_vec);

    Eigen::VectorXd AQc_vec(n);
    AQc.supernodes->GatherInto(AQc_vec);
    Eigen::VectorXd S21_vec = b_old - AQc_vec;

    double S22 = wt * cQc + 1.0 / wt;
    double S21_Ginv_S12 = S21_vec.dot(Ginv_S12_vec);

    if (verbose && iter == 0) {
      auto ff = kkt.MakeSolverRHS(); ff = f_n0; ff += f_n1;
      Eigen::VectorXd fv(n); ff.supernodes->GatherInto(fv);
      printf("  Init: ||f_n(1)||=%.2e, f_s(1)=%.2e\n", fv.norm(), f_s0+f_s1);
    }

    // Solve [G S12; S21 S22] [y; dt] = [f_n(sq); f_s(sq)].
    // y and dt are LINEAR in sq (RHS is linear, system matrix constant).
    auto solve = [&](double sq, Eigen::VectorXd& y_out, double& dt_out) {
      auto fn = kkt.MakeSolverRHS(); fn = f_n0;
      { auto t = f_n1; t *= sq; fn += t; }
      double fs = f_s0 + sq * f_s1;

      auto Ginv_fn = kkt.MakeSolverRHS(); Ginv_fn = fn;
      kkt.SolveSolverRHS(Ginv_fn);
      Eigen::VectorXd gv(n);
      Ginv_fn.supernodes->GatherInto(gv);

      dt_out = (fs - S21_vec.dot(gv)) / (S22 - S21_Ginv_S12);
      y_out = gv - dt_out * Ginv_S12_vec;
    };

    // d from (y, dt, sq). All terms linear in sq since y, dt are linear.
    // minus_s = c_weight*c + A_old*y - sq*e = c_weight*c - A*y - sq*e
    // d = e + Q(W^{1/2})(minus_s)
    auto compute_d = [&](const Eigen::VectorXd& y, double dt, double sq,
                         RowSpace& d_out, RowSpace& slack_out) {
      double cw = wt * (1 + dt) - sq;

      RowSpace Ay = kkt.MakeRowSpace();
      { auto yr = kkt.MakeSolverRHS(); yr.ScatterFrom(y.data(), y.size());
        kkt.MultiplyA(yr, Ay); }

      slack_out = c; slack_out *= cw;
      { RowSpace t = Ay; t *= -1; slack_out += t; }
      { RowSpace t = e; t *= -sq; slack_out += t; }

      RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
      d_out = quadraticRepresentation(sqrtW, slack_out);
      d_out += e;
    };

    Eigen::VectorXd y0, y1; double dt0, dt1;
    solve(0.0, y0, dt0);
    solve(1.0, y1, dt1);

    RowSpace d0 = kkt.MakeRowSpace(), d1 = kkt.MakeRowSpace();
    RowSpace s0 = kkt.MakeRowSpace(), s1 = kkt.MakeRowSpace();
    compute_d(y0, dt0, 0.0, d0, s0);
    compute_d(y1, dt1, 1.0, d1, s1);

    if (verbose && iter == 0) {
      printf("  d(0): inf=%.2e dt=%.4e | d(1): inf=%.2e dt=%.4e\n",
             normInf(d0), dt0, normInf(d1), dt1);
      Eigen::VectorXd yh; double dth;
      solve(0.5, yh, dth);
      RowSpace dh = kkt.MakeRowSpace(), sh = kkt.MakeRowSpace();
      compute_d(yh, dth, 0.5, dh, sh);
      RowSpace di = addScaled(d0, d1 - d0, 1.0, 0.5);
      printf("  Decomp: ||d(.5)-interp||=%.2e |dt(.5)-interp|=%.2e\n",
             normInf(dh - di), std::abs(dth - (dt0 + 0.5*(dt1-dt0))));
    }

    // Line search: d(sq) = d1 + t*(d0-d1) where t=1-sq.
    RowSpace d_rev = d0 - d1;
    double t_cone = lineSearchK(d1, d_rev);

    double dt_diff = dt0 - dt1;
    double t_tau = 0.0;
    if (std::abs(dt_diff) > 1e-15) {
      double klo = (-1.0 - dt1) / dt_diff;
      double khi = (1.0 - dt1) / dt_diff;
      if (klo > khi) std::swap(klo, khi);
      double lo = std::max(klo, 0.0), hi = std::min(khi, 1.0);
      t_tau = (lo <= hi) ? hi : 0.0;
    } else {
      t_tau = (std::abs(dt1) <= 1.0) ? 1.0 : 0.0;
    }

    double t = std::min({t_cone, t_tau, 1.0});
    t = std::max(t, 0.0);
    double sqrtmu = 1.0 - t;

    Eigen::VectorXd yf; double dtf;
    solve(sqrtmu, yf, dtf);
    RowSpace df = kkt.MakeRowSpace(), sf = kkt.MakeRowSpace();
    compute_d(yf, dtf, sqrtmu, df, sf);

    double dinf = std::max(normInf(df), std::abs(dtf));
    double alpha = std::min(1.0, 2.0 / (dinf * dinf));
    if (dinf < 1e-14) alpha = 1.0;

    double tau = sqrtmu * wt * (1.0 + dtf);
    double kappa = sqrtmu / wt * (1.0 - dtf);

    result.iterations = iter + 1;
    result.mu = sqrtmu * sqrtmu;
    result.tau = tau;
    result.kappa = kappa;
    result.d_inf = dinf;
    result.y = yf;

    if (verbose) {
      printf("  %3d  mu=%.2e  tau=%.4e  kap=%.4e  dinf=%.2e  "
             "dt=%.2e  a=%.4f  sq=%.2e\n",
             iter, sqrtmu*sqrtmu, tau, kappa, dinf, dtf, alpha, sqrtmu);
    }

    if (tau > 0 && kappa > 0) {
      if (tau / kappa > 1e6) {
        result.solved = true;
        result.primal_obj = b_old.dot(yf) / tau;
        return result;
      }
      if (kappa / tau > 1e6) { result.solved = false; return result; }
    }

    // minus_s is the negative slack. geodesicUpdateFromSlack expects
    // the raw slack S. The old code does WS = W*minus_s and then
    // GeodesicUpdate adds e_weight to get d = e + WS.
    // geodesicUpdateFromSlack(W, α, S) does W_new = exp(α(e + W*S))*W.
    // So S = minus_s is correct (minus_s IS the "S" in WS = W*S).
    geodesicUpdateFromSlack(W, alpha, sf);
    wt *= std::exp(alpha * dtf);
  }

  return result;
}

}  // namespace conex
