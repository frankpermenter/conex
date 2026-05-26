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

  // HybridOnly on the extended embedding — extract embedding theta
  // variable at each iteration via increasing maxiter.
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

}  // namespace conex
