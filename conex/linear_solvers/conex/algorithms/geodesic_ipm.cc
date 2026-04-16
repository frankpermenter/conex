#include "conex/algorithms/geodesic_ipm.h"

#include <cmath>
#include <cstdio>

#include "conex/common/equality_constraint.h"
#include "conex/tree_solver/kkt_tree_solver.h"

#include "conex/common/eja_ops.h"

namespace conex {

std::pair<double, double> VerifyNewtonEquations(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs_data,
    const RowSpace& b_data,
    const RowSpace& W,
    const RowSpace& d,
    const Eigen::VectorXd& y,
    double k,
    double theta) {

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);

  // Blended b and c.
  RowSpace b = addScaled(ones, b_data, theta, 1.0 - theta);

  auto cost_rhs = kkt.MakeSolverRHS();
  cost_rhs.SetZero();
  kkt.AccumulateAtranspose(ones, cost_rhs);
  cost_rhs *= theta;
  cost_rhs.AddScaled(1.0 - theta, cost_rhs_data);

  const int n = kkt.number_of_variables();

  // --- Primal check: d should equal e + P(W^{1/2})(-k*b - A*y) ---
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace I_minus_d = ones - d;
  // W^{-1/2} = P(W^{-1/4})... actually for the check we use:
  // d = I + P(W^{1/2})(S) where S = -k*b - A*y.
  // So I - d = -P(W^{1/2})(S) and P(W^{-1/2})(I - d) = -S = k*b + A*y.
  // Instead of computing W^{-1/2}, verify the equivalent:
  //   S_computed = P(W^{-1/2})(d - I) should equal -k*b - A*y.
  // Or simpler: verify d - I = P(W^{1/2})(-k*b - A*y).
  auto y_rhs = kkt.MakeSolverRHS();
  y_rhs = kkt.MakeBlockVariable(y);
  RowSpace Ay = kkt.MakeRowSpace();
  kkt.MultiplyA(y_rhs, Ay);
  RowSpace slack = addScaled(b, Ay, -k, -1.0);  // -k*b - Ay
  RowSpace d_expected = quadraticRepresentation(sqrtW, slack);
  d_expected += ones;  // I + P(W^{1/2})(S)

  RowSpace primal_err = d - d_expected;
  double primal_res = normInf(primal_err);

  // --- Dual check: A^T lambda = k*c where lambda = P(W^{1/2})(I + d) / k ---
  RowSpace I_plus_d = ones + d;
  RowSpace lambda = quadraticRepresentation(sqrtW, I_plus_d);
  lambda *= (1.0 / k);

  auto at_lambda = kkt.MakeSolverRHS();
  at_lambda.SetZero();
  kkt.AccumulateAtranspose(lambda, at_lambda);
  // Should equal c (the cost_rhs).
  at_lambda -= cost_rhs;
  Eigen::VectorXd dual_err(n);
  at_lambda.supernodes->GatherInto(dual_err);
  double dual_res = dual_err.norm();

  return {primal_res, dual_res};
}

OptimalityReport CheckOptimality(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const SolverRHS& x_rhs,
    const RowSpace& lambda) {
  OptimalityReport report;
  int n = kkt.number_of_variables();

  // s = Ax + b.
  RowSpace s = kkt.MakeRowSpace();
  kkt.MultiplyA(x_rhs, s);
  s += kkt.GetAffineTerm();

  // Cone membership.
  report.min_slack = minEigenvalue(s);
  report.min_dual = minEigenvalue(lambda);

  // Dual residual: ||A^T λ - Qx - c||.
  auto dual_rhs = kkt.MakeSolverRHS();
  dual_rhs.SetZero();
  kkt.AccumulateAtranspose(lambda, dual_rhs);
  auto qx_rhs = kkt.MakeSolverRHS();
  qx_rhs.SetZero();
  kkt.AccumulateQx(x_rhs, qx_rhs);
  dual_rhs -= qx_rhs;
  dual_rhs -= cost_rhs;
  Eigen::VectorXd dual_res(n);
  dual_rhs.supernodes->GatherInto(dual_res);
  report.dual_residual = dual_res.norm();

  // Complementarity: <s, λ>.
  report.complementarity = dot(s, lambda);

  return report;
}

static void ComputeDirectNewtonStep(
    KKTSolverBase& kkt, const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W, double k,
    RowSpace& d_out, Eigen::VectorXd& y_out,
    RowSpace* slack_out = nullptr);

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
  const RowSpace b = kkt.GetAffineTerm();

  GeodesicResult result{};
  result.mu = mu;

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt.MakeRowSpace();
    RowSpace slack = kkt.MakeRowSpace();
    Eigen::VectorXd y_direct;
    ComputeDirectNewtonStep(kkt, cost_rhs, b, W, k, d, y_direct, &slack);

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
      auto [p_res, d_res] = VerifyNewtonEquations(
          kkt, cost_rhs, b, W, d, y_direct, k, 0.0);
      printf("  i=%2d  mu=%.2e  d_sqr=%.2e  d_inf=%.2e  "
             "s_dot_x=%.2e  alpha=%.4f  newton_err=(%.1e, %.1e)\n",
             iter, mu, d_sq, d_inf, s_dot_x, alpha, p_res, d_res);
    }

    if (d_inf < tolerance) break;

    geodesicUpdateFromSlack(W, alpha, slack);
  }

  return result;
}

// Direct Newton step: factor, single RHS, solve, compute d and y.
// Matches what GeodesicCenter does per iteration.
static void ComputeDirectNewtonStep(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    double k,
    RowSpace& d_out,
    Eigen::VectorXd& y_out,
    RowSpace* slack_out) {
  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  y *= -k;
  RowSpace v = addScaled(quadraticRepresentation(W, b), W, -k, 2.0);
  kkt.AccumulateAtranspose(v, y);
  kkt.SolveSolverRHS(y);

  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  RowSpace slack = addScaled(b, row, -k, -1.0);
  if (slack_out) *slack_out = slack;

  d_out = quadraticRepresentation(EuclideanJordanAlgebra::sqrt(W), slack);
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
    const RowSpace& b,
    const RowSpace& W,
    RowSpace& d0,
    RowSpace& d1,
    Eigen::VectorXd* y0_out = nullptr,
    Eigen::VectorXd* y1_out = nullptr) {
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

// Set the equality constraint RHS (d) at dual variable positions in a
// SolverRHS.  This ensures Cy₀ = d so that the parameterized solution
// x(k,τ,θ) = y₀ + k·(τ·y₁₀ + θ·y₁θ) satisfies Cx = d for all (k,τ,θ).
static void SetEqualityRHS(KKTSolverBase& kkt, SolverRHS& rhs) {
  auto* ts = dynamic_cast<SymmetricLinearSystemTreeSolver*>(&kkt);
  if (!ts) return;
  const auto& eq_assemblers = ts->equality_sub_assemblers();
  if (eq_assemblers.empty()) return;
  // Build a dense vector with d at dual positions, scatter into rhs.
  int nv = kkt.number_of_variables();
  Eigen::VectorXd d_vec = Eigen::VectorXd::Zero(nv);
  for (const auto* ec : eq_assemblers) {
    const auto& dv = ec->dual_variables();
    const auto& d = ec->affine_term();
    for (int i = 0; i < static_cast<int>(dv.size()); ++i)
      d_vec(dv[i]) = d(i);
  }
  // Scatter into the supernode partition (additive).
  auto d_rhs = kkt.MakeSolverRHS();
  d_rhs = kkt.MakeBlockVariable(d_vec);
  rhs += d_rhs;
  fprintf(stderr, "SetEqualityRHS: set %d dual entries (|d|=%.4e)\n",
          (int)d_vec.size(), d_vec.norm());
}

NewtonDecomposition ComputeFullDecomposition(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W) {
  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);
  RowSpace v = kkt.MakeRowSpace();

  // rhs0: A^T(2W)  →  y0  (no equality RHS here — it goes in cost_rhs)
  auto rhs0 = kkt.MakeSolverRHS();
  rhs0.SetZero();
  v = W;
  v *= 2.0;
  kkt.AccumulateAtranspose(v, rhs0);

  // rhs1: -(c + A^T P(W) b_0)  →  y1_0
  auto rhs1 = kkt.MakeSolverRHS();
  rhs1 = cost_rhs;
  v = quadraticRepresentation(W, b);
  kkt.AccumulateAtranspose(v, rhs1);
  rhs1 *= -1;

  // rhs2 = (c + A^T P(W) b_0) - A^T(e + P(W)e)  →  y1_theta
  auto rhs2 = kkt.MakeSolverRHS();
  rhs2 = rhs1;
  rhs2 *= -1;  // c + A^T P(W) b_0
  v = addScaled(ones, quadraticRepresentation(W, ones), 1.0, 1.0);
  v *= -1.0;   // -(e + P(W)e)
  kkt.AccumulateAtranspose(v, rhs2);

  // Solve all three.
  auto y = kkt.MakeSolverRHS(3);
  y.SetColumn(0, rhs0);
  y.SetColumn(1, rhs1);
  y.SetColumn(2, rhs2);
  kkt.SolveSolverRHS(y);

  int nr = kkt.number_of_variables();
  Eigen::MatrixXd y_dense(nr, 3);
  y.supernodes->GatherInto(y_dense);

  // Multiply A * [y0, y1_0, y1_theta].
  auto row = kkt.MakeRowSpace(3);
  kkt.MultiplyA(y, row);

  RowSpace ay0 = kkt.MakeRowSpace();
  RowSpace ay1_0 = kkt.MakeRowSpace();
  RowSpace ay1_theta = kkt.MakeRowSpace();
  ay0.col() = row.col(0);
  ay1_0.col() = row.col(1);
  ay1_theta.col() = row.col(2);

  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);

  // d0 = e - P(W^{1/2})(A y0)
  RowSpace d0 = kkt.MakeRowSpace();
  setOnes(d0);
  d0 -= quadraticRepresentation(sqrtW, ay0);

  // d1_0 = P(W^{1/2})(-b_0 - A y1_0)
  RowSpace d1_0 = quadraticRepresentation(sqrtW,
      addScaled(b, ay1_0, -1.0, -1.0));

  // d1_theta = P(W^{1/2})(b_0 - e - A y1_theta)
  RowSpace d1_theta = quadraticRepresentation(sqrtW,
      addScaled(addScaled(b, ones, 1.0, -1.0), ay1_theta, 1.0, -1.0));

  return {d0, d1_0, d1_theta,
          y_dense.col(0), y_dense.col(1), y_dense.col(2)};
}

RowSpace EvaluateDirection(const NewtonDecomposition& decomp,
                           double k, double tau, double theta) {
  // d(k, tau, theta) = d0 + k * (tau * d1_0 + theta * d1_theta)
  RowSpace d1 = addScaled(decomp.d1_0, decomp.d1_theta, tau, theta);
  return addScaled(decomp.d0, d1, 1.0, k);
}

double MinNormK(const NewtonDecomposition& decomp, double tau, double theta) {
  RowSpace d1 = addScaled(decomp.d1_0, decomp.d1_theta, tau, theta);
  double d1_sq = squaredNorm(d1);
  double d0_d1 = dot(decomp.d0, d1);
  if (d1_sq < 1e-30) return 0.0;
  return -d0_d1 / d1_sq;
}

DecompInnerProducts ComputeInnerProducts(const NewtonDecomposition& decomp) {
  return {
    squaredNorm(decomp.d0),
    dot(decomp.d0, decomp.d1_0),
    dot(decomp.d0, decomp.d1_theta),
    squaredNorm(decomp.d1_0),
    dot(decomp.d1_0, decomp.d1_theta),
    squaredNorm(decomp.d1_theta)
  };
}

// ||d||^2 as a function of k, with tau = tau*(k) and theta = 1/k^2.
static double ReducedDSq(const DecompInnerProducts& ip, double k) {
  double k2 = k * k;
  double num = k * ip.f + ip.q;
  // a + 2c/k + r/k^2 - (kb + q)^2 / (k^2 p)
  return ip.a + 2.0 * ip.g / k + ip.r / k2 - (num * num) / (k2 * ip.p);
}

KTauResult SelectKTau(const DecompInnerProducts& ip) {
  // Golden-section search over k > 0 to minimize ReducedDSq.
  // Bracket: start from k=1, expand to find a bracket.
  double k_lo = 1e-6, k_hi = 1e6;

  // Refine with golden section.
  const double gr = (std::sqrt(5.0) + 1.0) / 2.0;
  for (int i = 0; i < 100; ++i) {
    double k1 = k_hi - (k_hi - k_lo) / gr;
    double k2 = k_lo + (k_hi - k_lo) / gr;
    if (ReducedDSq(ip, k1) < ReducedDSq(ip, k2)) {
      k_hi = k2;
    } else {
      k_lo = k1;
    }
  }

  double k = 0.5 * (k_lo + k_hi);
  double tau = -(k * ip.f + ip.q) / (k * k * ip.p);
  double theta = 1.0 / (k * k);
  double d_sq = ReducedDSq(ip, k);

  return {k, tau, theta, d_sq};
}

DualityCoeffs ComputeDualityCoeffs(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp) {
  // sigma1 = <b0, P(W^{1/2})(d1_0)>
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace Pw_d1_0 = quadraticRepresentation(sqrtW, decomp.d1_0);
  double sigma1 = dot(b, Pw_d1_0);

  // gamma1 = c^T y1_0
  auto y1_rhs = kkt.MakeSolverRHS();
  y1_rhs = kkt.MakeBlockVariable(decomp.y1_0);
  double gamma1 = cost_rhs.dot(y1_rhs);

  return {sigma1, gamma1};
}

double SelectTauWeighted(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp,
    const DecompInnerProducts& ip,
    const DualityCoeffs& dc,
    double bT_ones,
    double k, double theta, double w) {
  // --- ||d||^2 as quadratic in tau ---
  // d = (d0 + k*theta*d1_theta) + k*tau*d1_0 = f + k*tau*d1_0
  // ||d||^2 = ||f||^2 + 2*k*tau*<f, d1_0> + k^2*tau^2*||d1_0||^2
  // where <h, d1_0> = <d0, d1_0> + k*theta*<d1_theta, d1_0> = ip.f + k*theta*ip.q
  // and ||h||^2 = ip.a + 2*k*theta*ip.g + k^2*theta^2*ip.r
  double h_dot_d1 = ip.f + k * theta * ip.q;
  double h_sq = ip.a + 2.0 * k * theta * ip.g + k * k * theta * theta * ip.r;
  double d_a = h_sq;            // constant term
  double d_b = k * h_dot_d1;   // linear coeff (half)
  double d_p = k * k * ip.p;   // quadratic coeff
  // ||d||^2 = d_a + 2*d_b*tau + d_p*tau^2

  // --- Violation quadratic: beta*tau^2 + (alpha - R)*tau + mu ---
  // sigma0(k) = (1/k)<b0, P(W^{1/2})(e + d0 + k*theta*d1_theta)>
  // We compute this from the decomposition.
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace ones_v = kkt.MakeRowSpace();
  setOnes(ones_v);
  RowSpace f_vec = addScaled(decomp.d1_theta, ones_v, k * theta, 0.0);
  // f_vec = k*theta*d1_theta; need e + d0 + k*theta*d1_theta
  // Actually let me just compute it directly:
  RowSpace e_plus_d0 = ones_v + decomp.d0;
  RowSpace arg = addScaled(e_plus_d0, decomp.d1_theta, 1.0, k * theta);
  RowSpace Pw_arg = quadraticRepresentation(sqrtW, arg);
  double sigma0 = dot(b, Pw_arg) / k;

  // gamma0(k) = c^T y0 / k + theta * c^T y1_theta
  auto y0_rhs = kkt.MakeSolverRHS();
  y0_rhs = kkt.MakeBlockVariable(decomp.y0);
  double cT_y0 = cost_rhs.dot(y0_rhs);
  auto yt_rhs = kkt.MakeSolverRHS();
  yt_rhs = kkt.MakeBlockVariable(decomp.y1_theta);
  double cT_yt = cost_rhs.dot(yt_rhs);
  double gamma0 = cT_y0 / k + theta * cT_yt;

  double alpha = sigma0 + gamma0;
  double beta = dc.sigma1 + dc.gamma1;
  double R = theta * (bT_ones + 1.0);
  double mu = 1.0 / (k * k);

  // Violation*tau = beta*tau^2 + (alpha - R)*tau + mu
  double v_a = mu;              // constant term
  double v_b = 0.5 * (alpha - R);  // linear coeff (half)
  double v_p = beta;            // quadratic coeff
  // violation = v_a + 2*v_b*tau + v_p*tau^2

  // --- Minimize f(tau) = ||d||^2 + w * violation^2 ---
  // f(tau) = (d_a + 2*d_b*tau + d_p*tau^2) + w*(v_a + 2*v_b*tau + v_p*tau^2)^2
  // f'(tau) = 2*d_b + 2*d_p*tau + 2*w*(v_a + 2*v_b*tau + v_p*tau^2)*(2*v_b + 2*v_p*tau)
  // This is cubic. Solve with golden section over tau > 0.

  // Evaluate objective.
  auto obj = [&](double tau) {
    double dsq = d_a + 2.0 * d_b * tau + d_p * tau * tau;
    double viol = v_a + 2.0 * v_b * tau + v_p * tau * tau;
    return dsq + w * viol * viol;
  };

  // Golden section search over tau in (0, tau_max].
  // Start with tau from min-norm (ignoring violation) as initial guess.
  double tau_minnorm = (d_p > 1e-30) ? -d_b / d_p : 1.0;
  double tau_lo = std::max(1e-8, tau_minnorm * 0.01);
  double tau_hi = std::max(tau_minnorm * 10.0, 10.0);

  const double gr = (std::sqrt(5.0) + 1.0) / 2.0;
  for (int i = 0; i < 100; ++i) {
    double t1 = tau_hi - (tau_hi - tau_lo) / gr;
    double t2 = tau_lo + (tau_hi - tau_lo) / gr;
    if (obj(t1) < obj(t2)) {
      tau_hi = t2;
    } else {
      tau_lo = t1;
    }
  }
  return 0.5 * (tau_lo + tau_hi);
}

// Given a decomposition and candidate theta, solve the hard-constraint
// violation quadratic V(tau)=0, evaluate d, return (tau, d_inf).
// Returns tau = -1 if no positive root exists.
static std::pair<double, double> EvalThetaCandidate(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double theta_cand) {
  if (theta_cand <= 0) return {-1, 1e30};
  double k = 1.0 / std::sqrt(theta_cand);
  double mu = theta_cand;

  // Compute violation quadratic coefficients: beta*tau^2 + (alpha-R)*tau + mu = 0
  auto dc = ComputeDualityCoeffs(kkt, cost_rhs, b, W, decomp);
  double beta = dc.sigma1 + dc.gamma1;

  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace ones_v = kkt.MakeRowSpace();
  setOnes(ones_v);
  RowSpace e_plus_d0 = ones_v + decomp.d0;
  RowSpace arg = addScaled(e_plus_d0, decomp.d1_theta, 1.0, k * theta_cand);
  double sigma0 = dot(b, quadraticRepresentation(sqrtW, arg)) / k;

  auto y0_rhs = kkt.MakeSolverRHS();
  y0_rhs = kkt.MakeBlockVariable(decomp.y0);
  double cT_y0 = cost_rhs.dot(y0_rhs);
  auto yt_rhs = kkt.MakeSolverRHS();
  yt_rhs = kkt.MakeBlockVariable(decomp.y1_theta);
  double cT_yt = cost_rhs.dot(yt_rhs);
  double gamma0 = cT_y0 / k + theta_cand * cT_yt;

  double alpha = sigma0 + gamma0;
  double R = theta_cand * (bT_ones + 1.0);

  // Solve quadratic: beta*tau^2 + (alpha - R)*tau + mu = 0
  double B = alpha - R;
  double disc = B * B - 4.0 * beta * mu;
  if (disc < 0) return {-1, 1e30};

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta);

  // Pick the positive root with smaller ||d||^2.
  auto eval_dsq = [&](double tau) -> double {
    if (tau <= 0) return 1e30;
    RowSpace d = EvaluateDirection(decomp, k, tau, theta_cand);
    return squaredNorm(d);
  };

  double dsq1 = eval_dsq(tau1);
  double dsq2 = eval_dsq(tau2);

  double tau, dsq;
  if (tau1 > 0 && (tau2 <= 0 || dsq1 <= dsq2)) {
    tau = tau1; dsq = dsq1;
  } else if (tau2 > 0) {
    tau = tau2; dsq = dsq2;
  } else {
    return {-1, 1e30};
  }

  RowSpace d = EvaluateDirection(decomp, k, tau, theta_cand);
  return {tau, normInf(d)};
}

GeodesicResult SolveGeodesicThetaContinuation(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  const RowSpace b = kkt.GetAffineTerm();
  const int m = W.total_rows();

  RowSpace ones_bTe = kkt.MakeRowSpace();
  setOnes(ones_bTe);
  const double bT_ones = dot(b, ones_bTe);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  double k = 0, tau = 0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %8s  %10s  %12s  %12s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %12s\n",
           "out", "theta", "tau", "kappa", "k", "d_inf", "d_sqr",
           "gap", "bTl", "cTx", "mu/tau", "eq_err");
    printf("  %s\n", std::string(146, '-').c_str());
  }

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    // Decompose at current W.
    auto decomp = ComputeFullDecomposition(kkt, cost_rhs, b, W);
    total_fac++;
    total_sol += 3;

    // Binary search for the smallest theta with ||d||_inf <= beta,
    // using the hard-constraint (V(tau)=0) tau selection.
    constexpr double beta_target = 1.0;
    double theta_prev = theta;
    {
      double theta_lo = 0.0;     // allow theta to reach 0
      double theta_hi = theta;   // current (centered) theta
      for (int bisect = 0; bisect < 30; ++bisect) {
        // Use arithmetic mean since theta_lo can be 0 (geometric mean→0).
        double theta_mid = 0.5 * (theta_lo + theta_hi);
        auto [tau_try, d_inf_try] = EvalThetaCandidate(
            kkt, cost_rhs, b, W, decomp, bT_ones, theta_mid);
        if (tau_try > 0 && d_inf_try <= beta_target) {
          theta_hi = theta_mid;  // can go lower
        } else {
          theta_lo = theta_mid;  // too aggressive
        }
      }
      theta = theta_hi;
    }
    k = 1.0 / std::sqrt(theta);
    // Evaluate tau at the chosen theta via hard constraint (V(tau)=0).
    auto [tau_sel, d_inf_sel] = EvalThetaCandidate(
        kkt, cost_rhs, b, W, decomp, bT_ones, theta);
    if (tau_sel <= 0) {
      // No positive root — abort this outer iteration.
      result.iterations = outer + 1;
      break;
    }
    tau = tau_sel;

    // Evaluate the direction and reported quantities at the consistent
    // (W, decomp, tau) BEFORE the geodesic step, so V(tau)=0 holds
    // exactly and reported eq_err ≈ machine precision.
    RowSpace d_step = EvaluateDirection(decomp, k, tau, theta);
    double d_inf = normInf(d_step);
    double d_sq = squaredNorm(d_step);
    double mu = 1.0 / (k * k);
    double gap = mu * (m - d_sq);

    RowSpace sqrtW_step = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones_step = kkt.MakeRowSpace();
    setOnes(ones_step);
    RowSpace lam_step = quadraticRepresentation(sqrtW_step, ones_step + d_step);
    lam_step *= (1.0 / k);
    double bT_lambda = dot(b, lam_step);
    Eigen::VectorXd x_step = decomp.y0 / k + tau * decomp.y1_0
                             + theta * decomp.y1_theta;
    auto x_rhs_step = kkt.MakeSolverRHS();
    x_rhs_step = kkt.MakeBlockVariable(x_step);
    double cT_x = cost_rhs.dot(x_rhs_step);
    double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;
    double eq_err_final = std::abs(bT_lambda + cT_x + mu_over_tau
                                    - theta * (bT_ones + 1.0));

    // Take exactly one geodesic step per theta update — no inner
    // centering loop.
    if (d_inf > 1e-14) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d_step);
    }
    int centering_iters = 0;

    if (verbose) {
      double kappa = mu_over_tau;
      // Report the de-homogenized (physical) primal/dual values:
      // x_phys = x_lifted / tau, lambda_phys = lambda_lifted / tau.
      double bTl_phys = (tau > 1e-30) ? bT_lambda / tau : 0.0;
      double cTx_phys = (tau > 1e-30) ? cT_x / tau : 0.0;
      printf("  %3d  %8.6f  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.4e  %12.2e  %3d\n",
             outer, theta, tau, kappa, k, d_inf, d_sq, gap,
             bTl_phys, cTx_phys, mu_over_tau, eq_err_final, centering_iters);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Termination: mu below tolerance with d_inf small.  Same criterion
    // for both algorithms so iteration counts are directly comparable.
    if (mu < tolerance && d_inf <= 1.001) {
      if (verbose) printf("  TERMINATED: mu = %.2e < tolerance, d_inf = %.2e\n",
                          mu, d_inf);
      break;
    }
    if (outer + 1 == max_outer_iterations) {
      if (verbose) printf("  TERMINATED: reached max_outer_iterations = %d\n",
                          max_outer_iterations);
    }
  }

  // Recover primal x.  De-homogenize: x_phys = x_lifted / tau.
  if (k > 0 && tau > 0) {
    auto decomp = ComputeFullDecomposition(kkt, cost_rhs, b, W);
    Eigen::VectorXd x_lifted =
        decomp.y0 / k + tau * decomp.y1_0 + theta * decomp.y1_theta;
    result.x = x_lifted / tau;

    // Compute lambda at the CURRENT (k, tau, theta) — consistent with x.
    // lambda_lifted = (1/k) P(W^{1/2})(e + d(k,tau,theta));
    // lambda_phys   = lambda_lifted / tau.
    RowSpace d_cur = EvaluateDirection(decomp, k, tau, theta);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones = kkt.MakeRowSpace();
    setOnes(ones);
    RowSpace lambda = quadraticRepresentation(sqrtW, ones + d_cur);
    lambda *= (1.0 / (k * tau));

    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;

    if (verbose) {
      printf("  Optimality: dual_res=%.2e, compl=%.2e, "
             "min_s=%.2e, min_lam=%.2e\n",
             result.optimality.dual_residual,
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  return result;
}

double GeodesicLineSearch(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& W) {
  const RowSpace b = kkt.GetAffineTerm();
  RowSpace d0 = kkt.MakeRowSpace();
  RowSpace d1 = kkt.MakeRowSpace();
  ComputeDecomposition(kkt, cost_rhs, b, W, d0, d1);

  return lineSearchK(d0, d1);
}

// Solve V(τ) = 0 for τ at fixed (theta, k); returns (tau, d_inf).
// Same algebra as EvalThetaCandidate but with k passed in independently
// of theta (so we can hold k = 1/sqrt(theta_mid) during a binary search).
static std::pair<double, double> EvalKCandidate(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const NewtonDecomposition& decomp,
    double bT_ones,
    double theta_val,
    double k_cand) {
  if (k_cand <= 0) return {-1, 1e30};
  double mu = 1.0 / (k_cand * k_cand);

  auto dc = ComputeDualityCoeffs(kkt, cost_rhs, b, W, decomp);
  double beta_coeff = dc.sigma1 + dc.gamma1;

  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace ones_v = kkt.MakeRowSpace();
  setOnes(ones_v);
  RowSpace e_plus_d0 = ones_v + decomp.d0;
  RowSpace arg = addScaled(e_plus_d0, decomp.d1_theta, 1.0, k_cand * theta_val);
  double sigma0 = dot(b, quadraticRepresentation(sqrtW, arg)) / k_cand;

  auto y0_rhs = kkt.MakeSolverRHS();
  y0_rhs = kkt.MakeBlockVariable(decomp.y0);
  double cT_y0 = cost_rhs.dot(y0_rhs);
  auto yt_rhs = kkt.MakeSolverRHS();
  yt_rhs = kkt.MakeBlockVariable(decomp.y1_theta);
  double cT_yt = cost_rhs.dot(yt_rhs);
  double gamma0 = cT_y0 / k_cand + theta_val * cT_yt;

  double alpha_coeff = sigma0 + gamma0;
  double R = theta_val * (bT_ones + 1.0);

  double B = alpha_coeff - R;
  double disc = B * B - 4.0 * beta_coeff * mu;
  if (disc < 0) return {-1, 1e30};

  double sqrt_disc = std::sqrt(disc);
  double tau1 = (-B + sqrt_disc) / (2.0 * beta_coeff);
  double tau2 = (-B - sqrt_disc) / (2.0 * beta_coeff);

  auto eval_dinf = [&](double tau) -> std::pair<double, double> {
    if (tau <= 0) return {-1, 1e30};
    RowSpace d = EvaluateDirection(decomp, k_cand, tau, theta_val);
    return {tau, normInf(d)};
  };

  auto [t1, dinf1] = eval_dinf(tau1);
  auto [t2, dinf2] = eval_dinf(tau2);

  if (t1 > 0 && (t2 <= 0 || dinf1 <= dinf2)) return {t1, dinf1};
  if (t2 > 0) return {t2, dinf2};
  return {-1, 1e30};
}

GeodesicResult SolveGeodesicPhaseOne(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_outer_iterations,
    int /*max_centering_steps*/,
    double tolerance,
    bool verbose,
    bool phase1_only) {
  const RowSpace b = kkt.GetAffineTerm();
  const int m = W.total_rows();

  RowSpace ones_bTe = kkt.MakeRowSpace();
  setOnes(ones_bTe);
  const double bT_ones = dot(b, ones_bTe);

  GeodesicResult result{};
  int total_fac = 0;
  int total_sol = 0;

  double k = 1.0, tau = 1.0, theta = 1.0;

  if (verbose) {
    printf("  %3s  %12s  %10s  %12s  %12s  %12s  %12s  %12s"
           "  %12s  %12s  %12s  %4s\n",
           "out", "theta", "tau", "kappa", "mu", "d_inf", "d_sqr",
           "gap", "bTl", "cTx", "eq_err", "ph");
    printf("  %s\n", std::string(146, '-').c_str());
  }

  bool theta_zero = false;

  for (int outer = 0; outer < max_outer_iterations; ++outer) {
    auto decomp = ComputeFullDecomposition(kkt, cost_rhs, b, W);
    total_fac++;
    total_sol += 3;

    int phase = theta_zero ? 2 : 1;

    if (!theta_zero) {
      // Phase 1: try theta=0 first via lineSearchK with bound 1.1.
      // d at theta=0 is d0 + k * tau * d1_0; we hold tau at its current
      // value (last selected by V(tau)=0).  If a positive k exists,
      // commit to phase 2.
      RowSpace tau_d1_0 = decomp.d1_0; tau_d1_0 *= tau;
      double k_zero = lineSearchK(decomp.d0, tau_d1_0, 1.1);
      if (k_zero > 0) {
        theta = 0.0;
        theta_zero = true;
        k = k_zero;
        phase = 2;
        if (phase1_only) {
          result.mu = 1.0 / (k * k);
          result.tau = tau;
          result.iterations = outer + 1;
          result.total_factorizations = total_fac;
          result.total_solves = total_sol;
          if (verbose) {
            RowSpace d_p1 = EvaluateDirection(decomp, k, tau, 0.0);
            double dinf_p1 = normInf(d_p1);

            RowSpace b_sc = kkt.GetAffineTerm(); b_sc *= tau;
            auto c_sc = kkt.MakeSolverRHS(); c_sc = cost_rhs; c_sc *= tau;
            RowSpace r_chk = kkt.MakeRowSpace();
            setOnes(r_chk); r_chk *= (1.0 / k);
            RowSpace d_hyb = kkt.MakeRowSpace();
            RowSpace delta_hyb = kkt.MakeRowSpace();
            ComputeHybridDirection(kkt, c_sc, b_sc, W, r_chk,
                                    d_hyb, delta_hyb);
            double dinf_hyb = normInf(d_hyb);

            printf("  PHASE1 DONE: theta=0 at iter %d"
                   " (k=%.2e, tau=%.2e)\n"
                   "    d_inf phase1=%.6e  hybrid=%.6e  diff=%.2e\n",
                   outer, k, tau, dinf_p1, dinf_hyb,
                   std::abs(dinf_p1 - dinf_hyb));
            fprintf(stderr, "XCHECK cost_rhs sn[0]:");
            for (int j = 0; j < std::min(4, (int)cost_rhs.supernodes->block(0).rows()); ++j)
              fprintf(stderr, " %.6e", cost_rhs.supernodes->block(0)(j, 0));
            fprintf(stderr, "\n");
          }
          break;
        }
      } else {
        // Binary search for smallest theta admitting V(τ)=0 with d_inf<=1,
        // with k tied to theta as k = 1/sqrt(theta).  Arithmetic mean
        // (matches phase_one_debug branch).
        double theta_lo = 0.0;
        double theta_hi = theta;
        for (int bisect = 0; bisect < 30; ++bisect) {
          double theta_mid = 0.5 * (theta_lo + theta_hi);
          double k_mid = 1.0 / std::sqrt(theta_mid);
          auto [tau_try, d_inf_try] = EvalKCandidate(
              kkt, cost_rhs, b, W, decomp, bT_ones, theta_mid, k_mid);
          if (tau_try > 0 && d_inf_try <= 1.0) {
            theta_hi = theta_mid;
          } else {
            theta_lo = theta_mid;
          }
        }
        theta = theta_hi;
        k = 1.0 / std::sqrt(theta);
        auto [tau_sel, d_inf_sel] = EvalKCandidate(
            kkt, cost_rhs, b, W, decomp, bT_ones, theta, k);
        if (tau_sel <= 0) {
          if (verbose) printf("  TERMINATED: phase 1 V(τ)=0 has no positive root\n");
          break;
        }
        tau = tau_sel;
      }
    } else {
      // Phase 2 (theta=0): tau frozen, find largest k with d_inf <= 1.
      RowSpace tau_d1_0 = decomp.d1_0; tau_d1_0 *= tau;
      double k_new = lineSearchK(decomp.d0, tau_d1_0);
      if (k_new > 0) k = k_new;
    }

    // Evaluate quantities at the consistent (W, decomp, tau, k, theta)
    // BEFORE the geodesic step, so reported values are coherent.
    RowSpace d_step = EvaluateDirection(decomp, k, tau, theta);
    double d_inf = normInf(d_step);
    double d_sq = squaredNorm(d_step);
    double mu = 1.0 / (k * k);
    double gap = mu * (m - d_sq);

    RowSpace sqrtW_step = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones_step = kkt.MakeRowSpace();
    setOnes(ones_step);
    RowSpace lam_step = quadraticRepresentation(sqrtW_step, ones_step + d_step);
    lam_step *= (1.0 / k);
    double bT_lambda = dot(b, lam_step);
    Eigen::VectorXd x_step = decomp.y0 / k + tau * decomp.y1_0
                             + theta * decomp.y1_theta;
    auto x_rhs_step = kkt.MakeSolverRHS();
    x_rhs_step = kkt.MakeBlockVariable(x_step);
    double cT_x = cost_rhs.dot(x_rhs_step);
    double mu_over_tau = (tau > 1e-30) ? mu / tau : 0.0;
    double eq_err = std::abs(bT_lambda + cT_x + mu_over_tau
                             - theta * (bT_ones + 1.0));

    if (d_inf > 1e-14) {
      double alpha = std::min(1.0, 2.0 / (d_inf * d_inf));
      geodesicUpdate(W, alpha, d_step);
    }

    if (verbose) {
      double bTl_phys = (tau > 1e-30) ? bT_lambda / tau : 0.0;
      double cTx_phys = (tau > 1e-30) ? cT_x / tau : 0.0;
      printf("  %3d  %12.4e  %10.2e  %12.4e  %12.4e  %12.4e  %12.4e  %12.4e"
             "  %12.4e  %12.4e  %12.2e  %4d\n",
             outer, theta, tau, mu_over_tau, mu, d_inf, d_sq, gap,
             bTl_phys, cTx_phys, eq_err, phase);
    }

    result.iter_stats.push_back({mu, d_inf, d_sq, gap});
    result.iterations = outer + 1;
    result.mu = mu;
    result.tau = tau;
    result.d_inf_norm = d_inf;
    result.d_sq_norm = d_sq;
    result.complementarity = gap;
    result.total_factorizations = total_fac;
    result.total_solves = total_sol;

    // Termination: mu below tolerance with d_inf small.
    // When phase1_only, keep going until theta=0 is reached.
    if (!phase1_only && mu < tolerance && d_inf <= 1.001) {
      if (verbose) printf("  TERMINATED: mu = %.2e < tolerance, d_inf = %.2e\n",
                          mu, d_inf);
      break;
    }
    // Phase 2 divergence guard.
    if (theta_zero && d_inf > 10.0) {
      if (verbose) printf("  TERMINATED: phase 2 diverging (d_inf = %.2e)\n",
                          d_inf);
      break;
    }
  }

  // Recover primal x.  De-homogenize: x_phys = x_lifted / tau.
  if (k > 0 && tau > 0) {
    auto decomp = ComputeFullDecomposition(kkt, cost_rhs, b, W);
    Eigen::VectorXd x_lifted =
        decomp.y0 / k + tau * decomp.y1_0 + theta * decomp.y1_theta;
    result.x = x_lifted / tau;
    RowSpace d_final = EvaluateDirection(decomp, k, 1.0, 0.0);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones = kkt.MakeRowSpace();
    setOnes(ones);
    RowSpace lambda = quadraticRepresentation(sqrtW, ones + d_final);
    lambda *= (1.0 / k);
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
  }
  return result;
}


GeodesicResult SolveGeodesicLP(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    int max_outer_iterations,
    int max_centering_steps,
    double tolerance,
    bool verbose) {
  double k = 0.0;
  const int m = W.total_rows();
  constexpr double theta = 1.0;
  RowSpace ones_b = kkt.MakeRowSpace();
  setOnes(ones_b);

  RowSpace b = kkt.MakeRowSpace();
  auto cost_rhs_blend = kkt.MakeSolverRHS();
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
    b = addScaled(ones_b, kkt.GetAffineTerm(), theta, 1.0 - theta);
    cost_rhs_blend.SetZero();
    kkt.AccumulateAtranspose(ones_b, cost_rhs_blend);
    cost_rhs_blend *= theta;
    cost_rhs_blend.AddScaled(1.0 - theta, cost_rhs);

    // Decompose: 1 factor + 2 back-solves.
    RowSpace d0 = kkt.MakeRowSpace();
    RowSpace d1 = kkt.MakeRowSpace();
    Eigen::VectorXd y0, y1;
    ComputeDecomposition(kkt, cost_rhs_blend, b, W, d0, d1, &y0, &y1);

    total_fac += 1;
    total_sol += 2;

    // Line search for k.
    double k_new = lineSearchK(d0, d1);
    double k_prev = k;

    // On first iteration or when line search fails (k_new <= k),
    // use the minimum-norm k: k* = -<d0,d1> / ||d1||^2.
    if (k_new > k) {
      k = k_new;
    } else if (outer == 0 || k_new == 0) {
      double d0d1 = dot(d0, d1);
      double d1sq = squaredNorm(d1);
      if (d1sq > 1e-30) {
        double k_min_norm = std::max(0.0, -d0d1 / d1sq);
        if (k_min_norm > k) k = k_min_norm;
      }
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

    if (s_dot_x < tolerance && d_inf < 1.01) break;
  }

  // Optimality check.
  // For the LP path: lambda = (1/k) * P(W^{1/2})(e + d) where d is from
  // the last decomposition.  But after the geodesic step, W changed and d
  // is stale.  Recompute at the current W.
  if (result.x.size() > 0) {
    double k_final = 1.0 / std::sqrt(result.mu);
    RowSpace d_final = kkt.MakeRowSpace();
    Eigen::VectorXd y_final;
    ComputeDirectNewtonStep(kkt, cost_rhs_blend, b, W, k_final, d_final, y_final);

    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace ones = kkt.MakeRowSpace();
    setOnes(ones);
    RowSpace lambda = quadraticRepresentation(sqrtW, ones + d_final);
    lambda *= (1.0 / k_final);

    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    result.optimality = CheckOptimality(kkt, cost_rhs_blend, x_rhs, lambda);
    result.optimality.mu = result.mu;

    if (verbose) {
      printf("  Optimality: dual_res=%.2e, compl=%.2e, "
             "min_s=%.2e, min_lam=%.2e\n",
             result.optimality.dual_residual,
             result.optimality.complementarity,
             result.optimality.min_slack,
             result.optimality.min_dual);
    }
  }

  return result;
}

HybridDirection ComputeHybridDirection(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    const RowSpace& b,
    const RowSpace& W,
    const RowSpace& r,
    RowSpace& d,
    RowSpace& delta) {
  auto y = kkt.MakeSolverRHS();
  y = cost_rhs;
  y *= -1;
  RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
  RowSpace v = addScaled(quadraticRepresentation(W, b),
                         quadraticRepresentation(sqrtW, r), -1, 2.0);
  kkt.AccumulateAtranspose(v, y);
  kkt.SolveSolverRHS(y);

  RowSpace row = kkt.MakeRowSpace();
  kkt.MultiplyA(y, row);
  RowSpace slack_dir = addScaled(b, row, 1.0, 1.0);
  delta = addScaled(r,
      quadraticRepresentation(sqrtW, slack_dir), 1.0, -1.0);
  d = solveLyapunovForD(r, delta);

  return {gap(r, delta), normInf(d), squaredNorm(d), minSlack(r, delta)};
}

HybridDirection HybridCenteringStep(
    KKTSolverBase& kkt,
    const SolverRHS& cost_rhs,
    RowSpace& W,
    RowSpace& r) {
  const RowSpace b = kkt.GetAffineTerm();
  kkt.SetScaling(W);
  kkt.AssembleAndFactor();

  RowSpace d = kkt.MakeRowSpace();
  RowSpace delta = kkt.MakeRowSpace();
  auto info = ComputeHybridDirection(kkt, cost_rhs, b, W, r, d, delta);

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
    bool verbose,
    double initial_k,
    double tau) {
  RowSpace b = kkt.GetAffineTerm();
  const int m = b.total_rows();

  // Scale problem data by tau: b_scaled = tau*b, c_scaled = tau*c.
  // This puts the hybrid on the same central path as the HSD model
  // at the given tau.
  auto cost_scaled = kkt.MakeSolverRHS(); cost_scaled = cost_rhs;
  if (tau != 1.0) {
    b *= tau;
    cost_scaled *= tau;
  }

  RowSpace r = kkt.MakeRowSpace();
  setOnes(r);

  // Initial scaling: use caller-supplied k if positive, otherwise
  // decompose at W to find the minimum-norm k.
  if (initial_k <= 0) {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
  }
  // When initial_k > 0, reuse the existing factorization from the caller.
  if (initial_k > 0) {
    r *= (1.0 / initial_k);
  } else {
    RowSpace d0 = kkt.MakeRowSpace();
    RowSpace d1 = kkt.MakeRowSpace();
    ComputeDecomposition(kkt, cost_scaled, b, W, d0, d1);
    double d0d1 = dot(d0, d1);
    double d1sq = squaredNorm(d1);
    if (d1sq > 1e-30) {
      double k_init = std::max(1e-6, -d0d1 / d1sq);
      r *= (1.0 / k_init);
    }
  }

  int total_fac = 0;
  int total_sol = 0;

  GeodesicResult result{};
  int r_updates = 0;

  RowSpace ones = kkt.MakeRowSpace();
  setOnes(ones);

  int r_updates_this_fac = 0;
  double g = 0, d_inf = 0, d_sq = 0, mslack = 0;

  // Unscaled b for bTl computation (b was scaled by tau above).
  RowSpace b_unscaled = kkt.GetAffineTerm();

  if (verbose) {
    printf("  %3s  %12s  %10s %10s  %12s  %12s  %12s  %6s  %6s\n",
           "it", "gap", "d_pre", "d_post", "|r|^2/m", "bTl", "cTx",
           "r_upd", "step");
    printf("  %s\n", std::string(100, '-').c_str());
  }

  RowSpace last_delta = kkt.MakeRowSpace();

  if (verbose && initial_k > 0) {
    fprintf(stderr, "HYBRID cost_rhs sn[0]:");
    for (int j = 0; j < std::min(4, (int)cost_rhs.supernodes->block(0).rows()); ++j)
      fprintf(stderr, " %.6e", cost_rhs.supernodes->block(0)(j, 0));
    fprintf(stderr, "\nHYBRID cost_scaled sn[0]:");
    for (int j = 0; j < std::min(4, (int)cost_scaled.supernodes->block(0).rows()); ++j)
      fprintf(stderr, " %.6e", cost_scaled.supernodes->block(0)(j, 0));
    fprintf(stderr, "\n");
  }

  for (int iter = 0; iter < max_iterations; ++iter) {
    RowSpace d = kkt.MakeRowSpace();
    RowSpace delta = kkt.MakeRowSpace();
    auto info = ComputeHybridDirection(kkt, cost_scaled, b, W, r, d, delta);
    last_delta = delta;
    total_sol++;

    double d_inf_pre = info.d_inf;
    g = info.gap;
    d_inf = info.d_inf;
    d_sq = info.d_sq;
    mslack = info.min_slack;

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
      // Shrink r using Delta, then refactor so the next direction
      // computation reflects the current (W, r) state accurately.
      shrinkR(r, delta);
      kkt.SetScaling(W);
      if (!kkt.AssembleAndFactor()) break;
      total_fac++;
      r_updates_this_fac++;
      r_updates++;
    }
    // Recompute direction at updated (W, r) with fresh factorization.
    {
      RowSpace d2 = kkt.MakeRowSpace();
      RowSpace delta2 = kkt.MakeRowSpace();
      auto info2 = ComputeHybridDirection(kkt, cost_scaled, b, W, r, d2, delta2);
      total_sol++;
      g = info2.gap;
      d_inf = info2.d_inf;
      d_sq = info2.d_sq;
      mslack = info2.min_slack;
      last_delta = delta2;
    }
    if (verbose) {
      double mu_r = squaredNorm(r) / m;
      // Compute physical bTl and cTx (divided by tau).
      // lambda_lifted = P(W^{1/2})(r + delta2).
      RowSpace sqrtW_v = EuclideanJordanAlgebra::sqrt(W);
      RowSpace lam_v = quadraticRepresentation(sqrtW_v, r + last_delta);
      double bTl_phys = dot(b_unscaled, lam_v) / tau;
      // cTx: re-solve and dot with a fresh copy of cost_scaled.
      auto y_v = kkt.MakeSolverRHS();
      y_v = cost_scaled;
      y_v *= -1;
      RowSpace v_v = addScaled(quadraticRepresentation(W, b),
                               quadraticRepresentation(sqrtW_v, r), -1, 2.0);
      kkt.AccumulateAtranspose(v_v, y_v);
      kkt.SolveSolverRHS(y_v);
      auto c_fresh = kkt.MakeSolverRHS(); c_fresh = cost_scaled;
      double cTx_phys = kkt.dot(c_fresh, y_v) / (tau * tau);
      printf("  %3d  %12.4e  %10.4e %10.4e  %12.4e  %12.4e  %12.4e  %6d  %s\n",
             iter, g, d_inf_pre, d_inf, mu_r, bTl_phys, cTx_phys,
             r_updates_this_fac, (info.gap < 0) ? "center" : "shrink");
    }
    if (std::abs(g) < tolerance && d_inf <= 1.001) break;
  }

  result.iter_stats.push_back({g / m, d_inf, d_sq, g,
                               r_updates_this_fac, mslack});
  result.d_inf_norm = d_inf;
  result.d_sq_norm = d_sq;
  result.mu = squaredNorm(r) / m;
  result.complementarity = g;
  result.total_factorizations = total_fac;
  result.total_solves = total_sol;

  // Recover x: solve Gram * y = RHS at the final (W, r).
  // The last ComputeHybridDirection already solved this;
  // redo to extract y as x (no k scaling in hybrid).
  {
    kkt.SetScaling(W);
    kkt.AssembleAndFactor();
    auto y = kkt.MakeSolverRHS();
    y = cost_scaled;
    y *= -1;
    RowSpace sqrtW_final = EuclideanJordanAlgebra::sqrt(W);
    RowSpace v = addScaled(quadraticRepresentation(W, b),
                           quadraticRepresentation(sqrtW_final, r), -1, 2.0);
    kkt.AccumulateAtranspose(v, y);
    kkt.SolveSolverRHS(y);
    int nr = kkt.number_of_variables();
    result.x.resize(nr);
    y.supernodes->GatherInto(result.x);
    // De-homogenize: x_phys = x_scaled / tau.
    if (tau != 1.0 && tau > 0) result.x /= tau;
  }

  // Optimality check against the UNSCALED problem (original cost_rhs).
  {
    auto x_rhs = kkt.MakeSolverRHS();
    x_rhs = kkt.MakeBlockVariable(result.x);
    RowSpace sqrtW = EuclideanJordanAlgebra::sqrt(W);
    RowSpace lambda = quadraticRepresentation(sqrtW, r + last_delta);
    if (tau != 1.0 && tau > 0) lambda *= (1.0 / tau);
    result.optimality = CheckOptimality(kkt, cost_rhs, x_rhs, lambda);
    result.optimality.mu = result.mu;
  }

  if (verbose) {
    printf("  Optimality: dual_res=%.2e, compl=%.2e, "
           "min_s=%.2e, min_lam=%.2e\n",
           result.optimality.dual_residual,
           result.optimality.complementarity,
           result.optimality.min_slack,
           result.optimality.min_dual);
  }

  return result;
}

}  // namespace conex
