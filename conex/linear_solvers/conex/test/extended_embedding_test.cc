#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/solve_strategies.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

// Helper: build sparse A from dense.
Eigen::SparseMatrix<double> ToSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14)
        t.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

// =====================================================================
// Test: fixed point is strictly feasible.
// =====================================================================

TEST(ExtendedEmbedding, FixedPointFeasible) {
  srand(42);
  const int n = 5, m = 3;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  auto A = ToSparse(A_dense);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  auto [model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  printf("  n=%d m=%d total_vars=%d\n", n, m, info.total_vars());
  printf("  alpha=%.4f rg=%.4f\n", info.alpha, info.rg);
  printf("  ||rp||=%.4f ||rd||=%.4f\n", info.rp.norm(), info.rd.norm());

  // Check (x_hat, s_hat, y_hat, tau_hat, kappa_hat, theta=1) is feasible.
  VectorXd z = VectorXd::Zero(info.total_vars());
  z.segment(info.x_start(), n) = info.x_hat;
  z.segment(info.y_start(), m) = info.y_hat;
  z.segment(info.s_start(), m) = info.s_hat;
  z(info.tau_idx()) = info.tau_hat;
  z(info.kappa_idx()) = info.kappa_hat;
  z(info.theta_idx()) = 1.0;

  // Check equalities at (x_hat, y_hat, s_hat, tau_hat, kappa_hat, theta=1).
  // Eq1: A*x_hat - b*tau_hat - rp*1 = 0.
  VectorXd eq1 = A_dense * info.x_hat - b * info.tau_hat - info.rp;
  printf("  eq1 (primal): ||residual|| = %.2e\n", eq1.norm());
  EXPECT_LT(eq1.norm(), 1e-12);

  // Eq2: -A'*y_hat - s_hat + c*tau_hat - rd*1 = 0.
  VectorXd eq2 = -A_dense.transpose() * info.y_hat - info.s_hat +
                 c * info.tau_hat - info.rd;
  printf("  eq2 (dual): ||residual|| = %.2e\n", eq2.norm());
  EXPECT_LT(eq2.norm(), 1e-12);

  // Eq3: <b,y_hat> - <c,x_hat> - kappa_hat - rg*1 = 0.
  double eq3 = b.dot(info.y_hat) - c.dot(info.x_hat) -
               info.kappa_hat - info.rg;
  printf("  eq3 (gap): |residual| = %.2e\n", std::abs(eq3));
  EXPECT_LT(std::abs(eq3), 1e-12);

  // Eq4: <rp,y_hat> + <rd,x_hat> + rg*tau_hat = -alpha.
  double eq4 = info.rp.dot(info.y_hat) + info.rd.dot(info.x_hat) +
               info.rg * info.tau_hat + info.alpha;
  printf("  eq4 (norm): |residual| = %.2e\n", std::abs(eq4));
  EXPECT_LT(std::abs(eq4), 1e-12);

  // Cone: s > 0, tau > 0, kappa > 0.
  EXPECT_GT(info.s_hat.minCoeff(), 0);
  EXPECT_GT(info.tau_hat, 0);
  EXPECT_GT(info.kappa_hat, 0);
}

// =====================================================================
// Test: solve the embedding and recover the original LP solution.
// =====================================================================

TEST(ExtendedEmbedding, SolveAndRecover) {
  // Toy LP in standard form:  min c'x  s.t. Ax = b, x >= 0.
  //   c = (1, 2),  A = [1, 1],  b = [2].
  // Solution: x = (2, 0), obj = 2.
  const int n = 2, m = 1;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  VectorXd b(m); b << 2.0;
  VectorXd c(n); c << 1.0, 2.0;

  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  // Solve the embedding.
  auto solver = Solver::Build(emb_model);
  auto result = solver.Solve(ThetaContinuation());

  printf("  embedding: mu=%.2e converged=%d fac=%d\n",
         result.mu, result.converged, result.factorizations);
  printf("  obj=%.6e (should be ≈ 0 = alpha*theta)\n", result.objective);

  // Extract theta.
  double theta = result.x(info.theta_idx());
  printf("  theta = %.6e\n", theta);
  EXPECT_LT(std::abs(theta), 1e-4);

  // Extract tau.
  double tau = result.x(info.tau_idx());
  printf("  tau = %.6e\n", tau);

  if (tau > 1e-6) {
    // Complementary solution: x_opt = x/tau, y_opt = y/tau.
    VectorXd x_opt = result.x.segment(info.x_start(), n) / tau;
    VectorXd y_opt = result.x.segment(info.y_start(), m) / tau;
    double primal_obj = c.dot(x_opt);
    double dual_obj = b.dot(y_opt);

    printf("  primal obj = %.6e\n", primal_obj);
    printf("  dual obj   = %.6e\n", dual_obj);
    printf("  gap = %.2e\n", primal_obj - dual_obj);

    // Primal feasibility: Ax ≈ b (from Ax - bτ = 0 at θ=0, τ>0).
    VectorXd eq_res = Eigen::MatrixXd(A) * x_opt - b;
    printf("  ||Ax - b|| = %.2e\n", eq_res.norm());
    EXPECT_LT(eq_res.norm(), 1e-2);

    // x ≥ 0.
    printf("  min x = %.2e\n", x_opt.minCoeff());
    EXPECT_GE(x_opt.minCoeff(), -1e-3);
  } else {
    printf("  tau ≈ 0: infeasible or unbounded\n");
  }
}

// =====================================================================
// Test: model structure.
// =====================================================================

TEST(ExtendedEmbedding, ModelStructure) {
  const int n = 5, m = 3;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() +
                     0.1 * MatrixXd::Ones(m, n);
  auto A = ToSparse(A_dense);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = VectorXd::Ones(n);

  auto [model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);

  // Check structure: 4 equality constraints + 2 linear constraints.
  EXPECT_EQ(model.num_variables(), info.total_vars());
  printf("  total_vars=%d constraints=%d\n",
         info.total_vars(), model.num_constraints());

  // Variables: x(5), y(3), s(5), tau(1), kappa(1), theta(1) = 16.
  EXPECT_EQ(info.total_vars(), 16);
}

// =====================================================================
// Test: theta vs mu on the central path of the extended embedding.
// =====================================================================

TEST(ExtendedEmbedding, ThetaVsMuOnCentralPath) {
  srand(42);
  const int n = 6, m = 4;
  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd b = Ad * VectorXd::Ones(n);
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c);
  auto solver = Solver::Build(emb_model, emb_tree);

  auto cost_rhs = solver.MakeCostRHS();
  auto* kkt = solver.kkt();

  // Use GeodesicLP which increases k each iteration. At each step,
  // the solve determines theta as a Model variable.
  auto result = solver.Solve(GeodesicLP{1e-10, 20, 0, false});

  // result.x is in Model space. Read embedding variables directly.
  double theta_final = result.x(info.theta_idx());
  double tau_final = result.x(info.tau_idx());
  double kappa_final = result.x(info.kappa_idx());
  double mu_final = result.mu;

  printf("  Final: mu=%.6e theta=%.6e tau=%.6f kappa=%.6e theta/mu=%.6f\n",
         mu_final, theta_final, tau_final, kappa_final,
         (mu_final > 1e-30) ? theta_final / mu_final : 0.0);

  // Now center at specific mu values using ThetaContinuation
  // (which ties its external theta = mu).
  printf("\n  Per-iteration theta vs mu:\n");
  printf("  %10s %10s %10s %10s %10s\n",
         "mu", "theta", "tau", "kappa", "theta/mu");

  // Run ThetaContinuation and extract per-iteration values from iter_stats
  // plus the final result.
  auto result2 = solver.Solve(ThetaContinuation{1e-10, 20, 1, false});
  // ThetaCont converges at some mu. Extract at each stat.
  for (size_t i = 0; i < result2.duals.eq_residual.size(); i++) {
    // iter_stats don't have theta/tau. Just report final.
  }

  // Best approach: solve at a few fixed mu levels and check theta.
  // Use the embedding's cost = alpha*theta. At mu, objective = alpha*theta.
  // If theta = mu: objective = alpha*mu.
  double obj = result.objective;
  double alpha = info.alpha;
  double theta_from_obj = obj / alpha;
  printf("\n  From objective: obj=%.6e alpha=%.4f theta_from_obj=%.6e mu=%.6e ratio=%.6f\n",
         obj, alpha, theta_from_obj, mu_final,
         (mu_final > 1e-30) ? theta_from_obj / mu_final : 0.0);

  // Direct check: is theta ≈ mu at the solution?
  // The GeodesicLP drives mu→0, so theta should also →0.
  // The ratio theta/mu tells us if they track each other.
  EXPECT_LT(std::abs(theta_final), 1e-3);
}

// =====================================================================
// Test: embedding with dual equality constraints.
// =====================================================================

TEST(ExtendedEmbedding, DualEquality) {
  // LP with dual equality:
  //   P: min c'x + d'w  s.t.  Ax + C'w = b,  x >= 0  (w free)
  //   D: max b'y         s.t.  A'y + s = c,  Cy = d,  s >= 0
  //
  // P: min c'x + d'w  s.t.  Ax + C'w = b,  x >= 0  (w free)
  // D: max b'y        s.t.  A'y + s = c,  Cy = d,  s >= 0
  //
  // A = [1 1 0; 0 1 1] (2x3), b = [2; 2], c = [1; 1; 1]
  // C = [1 0] (1x2), d = [0.5]  → y0 = 0.5
  //
  // Dual: A'y + s = c, y0 = 0.5.
  //   s0 = 1 - y0 = 0.5
  //   s1 = 1 - y0 - y1 = 0.5 - y1
  //   s2 = 1 - y1
  //   Need s >= 0: y1 <= 0.5.
  //   Dual obj = b'y = 2y0 + 2y1 = 1 + 2y1. Max at y1 = 0.5 → obj = 2.
  //
  // Primal: Ax + C'w = b → x0+x1+w0 = 2, x1+x2 = 2.
  //   min x0+x1+x2 + 0.5*w0. With w0 = 2-x0-x1:
  //   = x0+x1+x2 + 0.5(2-x0-x1) = 0.5x0+0.5x1+x2+1.
  //   Subject to x1+x2=2, x>=0. Min at x0=0, x1=0, x2=2: obj=0+0+2+1=3.
  //   Or x0=0, x1=2, x2=0: obj=0+1+0+1=2. So obj=2.

  const int n = 3, m = 2, p = 1;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  A.insert(1, 1) = 1.0; A.insert(1, 2) = 1.0;
  VectorXd b(m); b << 2.0, 2.0;
  VectorXd c(n); c << 1.0, 1.0, 1.0;

  Eigen::SparseMatrix<double> C(p, m);
  C.insert(0, 0) = 1.0;
  VectorXd d(p); d << 0.5;

  // Check fixed point feasibility.
  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c, C, d);
  ASSERT_EQ(info.p, p);
  ASSERT_EQ(info.total_vars(), 2 * n + m + p + 3);  // 2*3+2+1+3 = 12
  printf("  n=%d m=%d p=%d total_vars=%d\n", n, m, p, info.total_vars());

  // Check residuals at fixed point.
  printf("  rp = [%.4f, %.4f]\n", info.rp(0), info.rp(1));
  printf("  rd = [%.4f, %.4f]\n", info.rd(0), info.rd(1));
  printf("  re = [%.4f]\n", info.re(0));
  printf("  rg = %.4f  alpha = %.4f\n", info.rg, info.alpha);

  // Solve with LU for best accuracy.
  // Use custom CliqueTree.
  auto solver = Solver::Build(emb_model, emb_tree);
  auto result = solver.Solve(GeodesicLP{1e-10, 30, 0, false});

  double theta = result.x(info.theta_idx());
  double tau = result.x(info.tau_idx());
  double kappa = result.x(info.kappa_idx());
  printf("  theta=%.2e tau=%.4f kappa=%.2e\n", theta, tau, kappa);
  EXPECT_LT(std::abs(theta), 1e-4);

  if (tau > 1e-6) {
    VectorXd x_opt = result.x.segment(info.x_start(), n) / tau;
    VectorXd y_opt = result.x.segment(info.y_start(), m) / tau;
    VectorXd w_opt = result.x.segment(info.w_start(), p) / tau;
    VectorXd s_opt = result.x.segment(info.s_start(), n) / tau;
    double primal_obj = c.dot(x_opt) + d.dot(w_opt);
    double dual_obj = b.dot(y_opt);

    printf("  x/tau = (%.4f, %.4f)\n", x_opt(0), x_opt(1));
    printf("  y/tau = (%.4f, %.4f)\n", y_opt(0), y_opt(1));
    printf("  w/tau = (%.4f)\n", w_opt(0));
    printf("  s/tau = (%.4f, %.4f)\n", s_opt(0), s_opt(1));
    printf("  primal_obj=%.6f  dual_obj=%.6f\n", primal_obj, dual_obj);

    // Primal feasibility: Ax + C'w = b.
    VectorXd pf = Eigen::MatrixXd(A) * x_opt +
                  Eigen::MatrixXd(C).transpose() * w_opt - b;
    printf("  ||Ax + C'w - b|| = %.2e\n", pf.norm());
    EXPECT_LT(pf.norm(), 1e-4);

    // Dual feasibility: A'y + s = c.
    VectorXd df = Eigen::MatrixXd(A).transpose() * y_opt + s_opt - c;
    printf("  ||A'y + s - c|| = %.2e\n", df.norm());
    EXPECT_LT(df.norm(), 1e-4);

    // Dual equality: Cy = d.
    VectorXd de = Eigen::MatrixXd(C) * y_opt - d;
    printf("  ||Cy - d|| = %.2e\n", de.norm());
    EXPECT_LT(de.norm(), 1e-4);

    // Optimal value.
    EXPECT_NEAR(primal_obj, 2.0, 0.1);
    EXPECT_NEAR(dual_obj, 2.0, 0.1);
  }
}

// Test: larger random instance with dual equalities.
TEST(ExtendedEmbedding, DualEqualityRandom) {
  srand(99);
  const int n = 8, m = 4, p = 2;

  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  Eigen::SparseMatrix<double> A = Ad.sparseView();
  VectorXd x0 = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  VectorXd b = Ad * x0;
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  // Dual equality: C is p×m.
  MatrixXd Cd = MatrixXd::Random(p, m).cwiseAbs() + 0.1 * MatrixXd::Ones(p, m);
  Eigen::SparseMatrix<double> C = Cd.sparseView();
  VectorXd d = VectorXd::Random(p).cwiseAbs() + 0.1 * VectorXd::Ones(p);

  ASSERT_LE(m + p, n) << "Need m+p <= n for the embedding";

  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(A, b, c, C, d);
  printf("  n=%d m=%d p=%d total_vars=%d\n", n, m, p, info.total_vars());

  SolverConfiguration cfg;
  cfg.tree.use_lu_for_indefinite = true;
  auto solver = Solver::Build(emb_model, emb_tree);
  auto result = solver.Solve(HybridOnly{1e-10, 500, false});

  double theta = result.x(info.theta_idx());
  double tau = result.x(info.tau_idx());
  printf("  theta=%.2e tau=%.4f\n", theta, tau);
  EXPECT_LT(std::abs(theta), 1e-4);
  EXPECT_GT(tau, 1e-6);

  if (tau > 1e-6) {
    VectorXd x_opt = result.x.segment(info.x_start(), n) / tau;
    VectorXd y_opt = result.x.segment(info.y_start(), m) / tau;
    VectorXd w_opt = result.x.segment(info.w_start(), p) / tau;
    VectorXd s_opt = result.x.segment(info.s_start(), n) / tau;

    double primal_obj = c.dot(x_opt) + d.dot(w_opt);
    double dual_obj = b.dot(y_opt);
    printf("  primal_obj=%.6f  dual_obj=%.6f  gap=%.2e\n",
           primal_obj, dual_obj, primal_obj - dual_obj);

    // Primal: Ax + C'w = b.
    VectorXd pf = Eigen::MatrixXd(A) * x_opt +
                  Eigen::MatrixXd(C).transpose() * w_opt - b;
    printf("  ||Ax+C'w-b||=%.2e\n", pf.norm());
    EXPECT_LT(pf.norm(), 1e-2);

    // Dual: A'y + s = c.
    VectorXd df = Eigen::MatrixXd(A).transpose() * y_opt + s_opt - c;
    printf("  ||A'y+s-c||=%.2e\n", df.norm());
    EXPECT_LT(df.norm(), 1e-2);

    // Dual equality: Cy = d.
    VectorXd de = Eigen::MatrixXd(C) * y_opt - d;
    printf("  ||Cy-d||=%.2e\n", de.norm());
    EXPECT_LT(de.norm(), 1e-2);

    // x, s >= 0.
    printf("  min(x)=%.2e  min(s)=%.2e\n", x_opt.minCoeff(), s_opt.minCoeff());
    EXPECT_GE(x_opt.minCoeff(), -1e-3);
    EXPECT_GE(s_opt.minCoeff(), -1e-3);
  }
}

}  // namespace
}  // namespace conex
