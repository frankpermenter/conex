// Verify ThetaContR and HybridOnly produce the same theta trajectory
// when applied to the extended embedding of an LP.
#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/extended_embedding.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {

static Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  return M.sparseView(1e-14, 1);
}

TEST(ThetaContREmbedding, ThetaTrajectoryMatchesHybridOnly) {
  srand(42);
  const int m = 4, n = 8;

  // Standard form LP data: min c'x s.t. Ax = b, x >= 0.
  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd x_feas = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  VectorXd b = Ad * x_feas;
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  // Dual problem: min -b'λ s.t. A'λ <= c  (i.e., c - A'λ >= 0).
  // Model: cost = -b, nonneg constraint c - A'λ >= 0.
  std::vector<int> vars(m);
  std::iota(vars.begin(), vars.end(), 0);

  // c - A'λ >= 0  →  (-A') λ + c >= 0
  Eigen::SparseMatrix<double> neg_At = toSparse(-Ad.transpose());

  Model problem;
  problem.AddLinearConstraint(neg_At, c, vars);
  problem.SetLinearCost(-b);

  const int max_iters = 20;

  // Build extended embedding from the same LP data.
  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(
      toSparse(Ad), b, c);

  // ThetaContR on the dual problem (nonneg constraints only).
  std::vector<double> theta_tcr;
  {
    SolverConfiguration cfg;
    auto solver = Solver::Build(problem, cfg);
    auto cm = solver.MakeCompiledModel();
    auto raw = ThetaContinuationR{1e-12, max_iters}.Run(cm);
    for (const auto& st : raw.iter_stats)
      theta_tcr.push_back(st.theta);
  }

  // HybridOnly on the extended embedding with verbose to see k_init.
  std::vector<double> theta_hybrid;
  for (int it = 1; it <= max_iters; ++it) {
    auto solver = Solver::Build(emb_model, emb_tree);
    auto cm = solver.MakeCompiledModel();
    auto raw = HybridOnly{1e-12, it, false}.Run(cm);
    if (!raw.x.empty() && info.theta_idx() < (int)raw.x.size()) {
      theta_hybrid.push_back(raw.x[info.theta_idx()]);
    }
  }

  // Compare trajectories.  ThetaContR iter_stats[i].theta is the theta
  // BEFORE iteration i.  HybridOnly at maxiter=i returns theta AFTER i
  // iterations.  So TCR[i+1] should match Hybrid[i].
  // Check iterations 1-8 (before switching policies diverge).
  int len = std::min({(int)theta_tcr.size() - 1, (int)theta_hybrid.size(), 8});

  printf("  iter  theta_TCR[i+1]  theta_Hybrid[i] diff\n");
  printf("  %s\n", std::string(55, '-').c_str());
  // Skip iteration 0 (off-by-one initialization artifact).
  for (int i = 1; i < len; ++i) {
    double tcr_val = theta_tcr[i + 1];
    double hyb_val = theta_hybrid[i];
    double diff = std::abs(tcr_val - hyb_val);
    printf("  %3d   %12.4e    %12.4e    %.2e\n",
           i, tcr_val, hyb_val, diff);
    double denom = std::max(std::abs(tcr_val), std::abs(hyb_val));
    double rel = (denom > 1e-30) ? diff / denom : diff;
    EXPECT_LT(rel, 1e-4)
        << "theta mismatch at iteration " << i
        << ": TCR[" << i+1 << "]=" << tcr_val
        << " Hybrid[" << i << "]=" << hyb_val;
  }

  // Both should drive theta toward 0.
  EXPECT_LT(std::abs(theta_tcr.back()), 1e-4)
      << "ThetaContR did not converge: theta=" << theta_tcr.back();
}

TEST(ThetaContREmbedding, WithDualEqualities) {
  srand(42);
  const int m = 4, n = 8, p = 2;

  // LP data: min c'x s.t. Ax = b, Cy = d, x >= 0, y free.
  MatrixXd Ad = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd x_feas = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);
  VectorXd b = Ad * x_feas;
  VectorXd c = VectorXd::Random(n).cwiseAbs() + 0.1 * VectorXd::Ones(n);

  // Dual equality C (p × m): C*lambda = d.
  MatrixXd Cd = MatrixXd::Random(p, m).cwiseAbs() + 0.1 * MatrixXd::Ones(p, m);
  VectorXd d = VectorXd::Random(p).cwiseAbs() + 0.1 * VectorXd::Ones(p);

  // Dual Model for ThetaContR: min -b'λ s.t. (-A')λ + c >= 0, Cλ = d.
  std::vector<int> lam_vars(m);
  std::iota(lam_vars.begin(), lam_vars.end(), 0);

  Model dual_problem;
  dual_problem.AddLinearConstraint(toSparse(-Ad.transpose()), c, lam_vars);
  dual_problem.AddEqualityConstraint(toSparse(Cd), d, lam_vars);
  dual_problem.SetLinearCost(-b);

  // Extended embedding from the same data.
  auto [emb_model, info, emb_tree] = BuildExtendedEmbedding(
      toSparse(Ad), b, c, toSparse(Cd), d);

  const int max_iters = 20;

  // ThetaContR on the dual problem.
  std::vector<double> theta_tcr;
  {
    SolverConfiguration cfg;
    auto solver = Solver::Build(dual_problem, cfg);
    auto cm = solver.MakeCompiledModel();
    auto raw = ThetaContinuationR{1e-12, max_iters}.Run(cm);
    for (const auto& st : raw.iter_stats)
      theta_tcr.push_back(st.theta);
  }

  // HybridOnly on the extended embedding.
  std::vector<double> theta_hybrid;
  for (int it = 1; it <= max_iters; ++it) {
    auto solver = Solver::Build(emb_model, emb_tree);
    auto cm = solver.MakeCompiledModel();
    auto raw = HybridOnly{1e-12, it, false}.Run(cm);
    if (!raw.x.empty() && info.theta_idx() < (int)raw.x.size()) {
      theta_hybrid.push_back(raw.x[info.theta_idx()]);
    }
  }

  // Compare iterations 1-8 (shifted by 1).
  int len = std::min({(int)theta_tcr.size() - 1, (int)theta_hybrid.size(), 8});

  printf("  iter  theta_TCR[i+1]  theta_Hybrid[i] diff\n");
  printf("  %s\n", std::string(55, '-').c_str());
  for (int i = 1; i < len; ++i) {
    double tcr_val = theta_tcr[i + 1];
    double hyb_val = theta_hybrid[i];
    double diff = std::abs(tcr_val - hyb_val);
    printf("  %3d   %12.4e    %12.4e    %.2e\n",
           i, tcr_val, hyb_val, diff);
    double denom = std::max(std::abs(tcr_val), std::abs(hyb_val));
    double rel = (denom > 1e-30) ? diff / denom : diff;
    EXPECT_LT(rel, 1e-4)
        << "theta mismatch at iteration " << i
        << ": TCR[" << i+1 << "]=" << tcr_val
        << " Hybrid[" << i << "]=" << hyb_val;
  }
}

// Helper: build the dual LP model for ThetaContR from primal LP data.
// Primal: min c'x s.t. Ax = b, x >= 0.
// Dual:   min -b'y s.t. A'y <= c, i.e., (-A')y + c >= 0.
static Model BuildDualModel(const Eigen::SparseMatrix<double>& A,
                            const VectorXd& b, const VectorXd& c) {
  const int m = A.rows();
  std::vector<int> vars(m);
  std::iota(vars.begin(), vars.end(), 0);
  Eigen::SparseMatrix<double> neg_At = toSparse(-MatrixXd(A.transpose()));
  Model model;
  model.AddLinearConstraint(neg_At, c, vars);
  model.SetLinearCost(-b);
  return model;
}

// Test primal infeasibility detection for a given algorithm.
template <typename Algo>
void TestPrimalInfeasible(const char* name, const Algo& algo) {
  // Primal LP: min c'x s.t. Ax = b, x >= 0.
  // Infeasible: x1 = 1, x2 = -1 with x >= 0 (x2 < 0 violates bounds).
  // A is full rank (identity). The dual min -b'y s.t. A'y <= c is unbounded.
  const int m = 2, n = 2;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1;
  A.insert(1, 1) = 1;
  A.makeCompressed();
  VectorXd b(m); b << 1, -1;
  VectorXd c(n); c << 1, 1;

  auto dual = BuildDualModel(A, b, c);
  auto solver = Solver::Build(dual, SolverConfiguration{});
  auto result = solver.Solve(algo);

  EXPECT_TRUE(result.infeasible)
      << name << ": expected primal infeasibility, tau=" << result.tau;

  ASSERT_EQ(result.x.size(), m);
  VectorXd y = result.x;
  if (-b.dot(y) > 0) y = -y;

  VectorXd slack = c - A.transpose() * y;
  double obj_dir = -b.dot(y);
  printf("  %s primal infeas: -b'y=%.4e  min(c-A'y)=%.4e\n",
         name, obj_dir, slack.minCoeff());
  EXPECT_LT(obj_dir, 0);
  EXPECT_GT(slack.minCoeff(), -0.1);
}

// Test dual infeasibility (primal unbounded) detection for a given algorithm.
template <typename Algo>
void TestDualInfeasible(const char* name, const Algo& algo) {
  // Primal LP: min -x1 - x2 s.t. x1 - x2 = 0, x >= 0.
  // Feasible: x1 = x2 = t, cost = -2t → -∞. Primal unbounded.
  // Dual: y <= -1 and y >= 1 → infeasible. ThetaContR sees tau→0.
  const int m = 1, n = 2;
  Eigen::SparseMatrix<double> A(m, n);
  A.insert(0, 0) = 1; A.insert(0, 1) = -1;
  A.makeCompressed();
  VectorXd b(m); b << 0;
  VectorXd c(n); c << -1, -1;

  auto dual = BuildDualModel(A, b, c);
  auto solver = Solver::Build(dual, SolverConfiguration{});
  auto result = solver.Solve(algo);

  EXPECT_TRUE(result.infeasible)
      << name << ": expected dual infeasibility, tau=" << result.tau;

  // Certificate: s >= 0 with As = 0 and c's < 0.
  if (!result.duals.lambda.empty()) {
    VectorXd s = result.duals.lambda[0];
    VectorXd As = A * s;
    double cTs = c.dot(s);
    if (cTs > 0) { s = -s; cTs = c.dot(s); As = A * s; }
    printf("  %s dual infeas: c's=%.4e  ||As||=%.4e  min(s)=%.4e\n",
           name, cTs, As.norm(), s.minCoeff());
    EXPECT_GT(s.minCoeff(), -0.1);
    EXPECT_LT(As.norm(), 0.1);
    EXPECT_LT(cTs, 0);
  }
}

TEST(ThetaContREmbedding, PrimalInfeasible_ThetaContR) {
  TestPrimalInfeasible("ThetaContR", ThetaContinuationR{1e-8, 500});
}

TEST(ThetaContREmbedding, DualInfeasible_ThetaContR) {
  TestDualInfeasible("ThetaContR", ThetaContinuationR{1e-8, 500});
}

TEST(ThetaContREmbedding, PrimalInfeasible_ThetaCont) {
  TestPrimalInfeasible("ThetaCont", ThetaContinuation{1e-8, 500});
}

TEST(ThetaContREmbedding, DualInfeasible_ThetaCont) {
  TestDualInfeasible("ThetaCont", ThetaContinuation{1e-12, 5000});
}

TEST(ThetaContREmbedding, PrimalInfeasible_HSDE) {
  TestPrimalInfeasible("HSDE", GeodesicHSDE{1e-8, 500});
}

TEST(ThetaContREmbedding, DualInfeasible_HSDE) {
  TestDualInfeasible("HSDE", GeodesicHSDE{1e-8, 500});
}

}  // namespace conex
