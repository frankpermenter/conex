// Verify that equality elimination produces identical GeodesicLP iterates
// and correct Model-space residuals.
#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/solver.h"

namespace conex {

TEST(EqualityElimination, GeodesicLP_Invariance) {
  // Build a QP with equality constraints and a non-trivial clique tree.
  const int n = 20;
  const int m = 15;
  const int p = 5;
  const int bw = 6;

  srand(42);

  // Banded inequality.
  std::vector<Eigen::Triplet<double>> A_trips;
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);
  for (int i = 0; i < m; ++i) {
    int start = (i * (n - bw)) / m;
    for (int j = start; j < start + bw && j < n; ++j)
      A_trips.emplace_back(i, j, 0.5 + 0.3 * std::abs(std::sin(i * 5 + j)));
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(A_trips.begin(), A_trips.end());
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Diagonal Q.
  Eigen::MatrixXd Q_dense = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) Q_dense(i, i) = 1.0 + 0.2 * i;
  Eigen::SparseMatrix<double> Q = Q_dense.sparseView();

  Eigen::VectorXd c = Eigen::VectorXd::Random(n) * 0.5;

  // Equality constraints: each row touches a few consecutive variables.
  std::vector<Eigen::Triplet<double>> C_trips;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(p);
  for (int i = 0; i < p; ++i) {
    int start = (i * n) / p;
    for (int j = start; j < start + 3 && j < n; ++j)
      C_trips.emplace_back(i, j, 1.0 + 0.1 * j);
    d(i) = 0.5 + 0.3 * i;
  }
  Eigen::SparseMatrix<double> C(p, n);
  C.setFromTriplets(C_trips.begin(), C_trips.end());

  Model model;
  model.AddLinearConstraint(A, b, all_vars);
  model.AddQuadraticCost(Q, all_vars);
  model.SetLinearCost(c);
  model.AddEqualityConstraint(C, d, all_vars);

  const int max_iters = 10;

  // Solve WITHOUT equality elimination (dense, to avoid tree solver issues).
  auto solver_orig = Solver::BuildDense(model);
  auto cm_orig = solver_orig.MakeCompiledModel();
  auto raw_orig = GeodesicLP{1e-10, max_iters}.Run(cm_orig);

  // Solve WITH equality elimination.
  SolverConfiguration elim_cfg;
  elim_cfg.eliminate_equalities = true;
  auto solver_elim = Solver::Build(model, elim_cfg);
  auto result_elim = solver_elim.Solve(GeodesicLP{1e-10, max_iters});

  // Also get full SolveResult from the original for Model-space comparison.
  auto result_orig = solver_orig.Solve(GeodesicLP{1e-10, max_iters});

  // 1. Iteration count and mu should match.
  auto cm_elim = solver_elim.MakeCompiledModel();
  auto raw_elim = GeodesicLP{1e-10, max_iters}.Run(cm_elim);
  ASSERT_EQ(raw_orig.iter_stats.size(), raw_elim.iter_stats.size())
      << "Iteration count mismatch";

  for (size_t i = 0; i < raw_orig.iter_stats.size(); ++i) {
    // mu should match.
    double mu_orig = raw_orig.iter_stats[i].mu;
    double mu_elim = raw_elim.iter_stats[i].mu;
    double mu_rel = std::abs(mu_orig - mu_elim) /
                    std::max(std::abs(mu_orig), 1e-30);
    EXPECT_LT(mu_rel, 1e-8)
        << "mu mismatch at iteration " << i
        << ": orig=" << mu_orig << " elim=" << mu_elim;

    // Newton direction squared norm (d_sqr) should match.
    double dsq_orig = raw_orig.iter_stats[i].d_sqr;
    double dsq_elim = raw_elim.iter_stats[i].d_sqr;
    double dsq_rel = std::abs(dsq_orig - dsq_elim) /
                     std::max(std::abs(dsq_orig), 1e-30);
    EXPECT_LT(dsq_rel, 1e-8)
        << "d_sqr mismatch at iteration " << i
        << ": orig=" << dsq_orig << " elim=" << dsq_elim;
  }

  // 3. Model-space residuals: objectives should match.
  EXPECT_NEAR(result_orig.objective, result_elim.objective,
              1e-6 * std::abs(result_orig.objective) + 1e-10)
      << "Objective mismatch";

  // 4. Equality residual Cx - d should be near zero for both.
  for (const auto& r : result_orig.duals.eq_residual)
    EXPECT_LT(r.norm(), 1e-6) << "Original eq residual too large";
  ASSERT_FALSE(result_elim.duals.eq_residual.empty())
      << "Eliminated solve should report equality residuals";
  for (const auto& r : result_elim.duals.eq_residual)
    EXPECT_LT(r.norm(), 1e-6) << "Eliminated eq residual too large";

  // 5. Equality duals ν should be recovered.
  ASSERT_FALSE(result_elim.duals.nu.empty())
      << "Eliminated solve should recover equality duals";

  // 6. Primal solutions should match.
  EXPECT_LT((result_orig.x - result_elim.x).norm(),
            1e-4 * result_orig.x.norm() + 1e-8)
      << "Primal solution mismatch";
}

TEST(EqualityElimination, HybridR_Invariance) {
  const int n = 20;
  const int m = 15;
  const int p = 5;
  const int bw = 6;

  srand(42);

  std::vector<Eigen::Triplet<double>> A_trips;
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);
  for (int i = 0; i < m; ++i) {
    int start = (i * (n - bw)) / m;
    for (int j = start; j < start + bw && j < n; ++j)
      A_trips.emplace_back(i, j, 0.5 + 0.3 * std::abs(std::sin(i * 5 + j)));
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(A_trips.begin(), A_trips.end());
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  Eigen::MatrixXd Q_dense = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) Q_dense(i, i) = 1.0 + 0.2 * i;
  Eigen::SparseMatrix<double> Q = Q_dense.sparseView();

  Eigen::VectorXd c = Eigen::VectorXd::Random(n) * 0.5;

  std::vector<Eigen::Triplet<double>> C_trips;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(p);
  for (int i = 0; i < p; ++i) {
    int start = (i * n) / p;
    for (int j = start; j < start + 3 && j < n; ++j)
      C_trips.emplace_back(i, j, 1.0 + 0.1 * j);
    d(i) = 0.5 + 0.3 * i;
  }
  Eigen::SparseMatrix<double> C(p, n);
  C.setFromTriplets(C_trips.begin(), C_trips.end());

  Model model;
  model.AddLinearConstraint(A, b, all_vars);
  model.AddQuadraticCost(Q, all_vars);
  model.SetLinearCost(c);
  model.AddEqualityConstraint(C, d, all_vars);

  const int max_iters = 15;

  // Without elimination (dense).
  auto solver_orig = Solver::BuildDense(model);
  auto cm_orig = solver_orig.MakeCompiledModel();
  auto raw_orig = HybridR{1e-10, max_iters}.Run(cm_orig);

  // With elimination.
  SolverConfiguration elim_cfg;
  elim_cfg.eliminate_equalities = true;
  auto solver_elim = Solver::Build(model, elim_cfg);
  auto cm_elim = solver_elim.MakeCompiledModel();
  auto raw_elim = HybridR{1e-10, max_iters}.Run(cm_elim);

  ASSERT_EQ(raw_orig.iter_stats.size(), raw_elim.iter_stats.size())
      << "Iteration count mismatch";

  for (size_t i = 0; i < raw_orig.iter_stats.size(); ++i) {
    double mu_orig = raw_orig.iter_stats[i].mu;
    double mu_elim = raw_elim.iter_stats[i].mu;
    double mu_rel = std::abs(mu_orig - mu_elim) /
                    std::max(std::abs(mu_orig), 1e-30);
    EXPECT_LT(mu_rel, 1e-8)
        << "mu mismatch at iteration " << i
        << ": orig=" << mu_orig << " elim=" << mu_elim;

    double dsq_orig = raw_orig.iter_stats[i].d_sqr;
    double dsq_elim = raw_elim.iter_stats[i].d_sqr;
    double dsq_rel = std::abs(dsq_orig - dsq_elim) /
                     std::max(std::abs(dsq_orig), 1e-30);
    EXPECT_LT(dsq_rel, 1e-8)
        << "d_sqr mismatch at iteration " << i
        << ": orig=" << dsq_orig << " elim=" << dsq_elim;
  }

  // Model-space check.
  auto result_orig = solver_orig.Solve(HybridR{1e-10, max_iters});
  auto result_elim = solver_elim.Solve(HybridR{1e-10, max_iters});

  EXPECT_NEAR(result_orig.objective, result_elim.objective,
              1e-3 * std::abs(result_orig.objective) + 1e-6)
      << "Objective mismatch";

  EXPECT_LT((result_orig.x - result_elim.x).norm(),
            1e-2 * result_orig.x.norm() + 1e-6)
      << "Primal solution mismatch";
}

}  // namespace conex
