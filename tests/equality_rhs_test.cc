// Verify that equality RHS d is not lost when a row's primal support
// spans different cliques than its dual variable.
#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/solver.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

TEST(EqualityRHS, TreeMatchesDense) {
  // Build a problem where equality constraints force d to be assigned
  // to a clique that may not contain the row's primal support.
  // Use a banded structure to get multiple cliques.
  const int n = 30;
  const int m = 20;
  const int p = 8;
  const int bw = 6;

  srand(42);

  // Banded inequality constraints.
  std::vector<Eigen::Triplet<double>> A_trips;
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);
  for (int i = 0; i < m; ++i) {
    int start = (i * (n - bw)) / m;
    for (int j = start; j < start + bw && j < n; ++j)
      A_trips.emplace_back(i, j, 1.0 + 0.1 * std::abs(std::sin(i * 7 + j)));
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(A_trips.begin(), A_trips.end());
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Diagonal Q.
  Eigen::MatrixXd Q_dense = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) Q_dense(i, i) = 1.0 + 0.1 * i;
  Eigen::SparseMatrix<double> Q = Q_dense.sparseView();

  Eigen::VectorXd c = Eigen::VectorXd::Random(n) * 0.5;

  // Equality constraints: each row touches variables spread across
  // the full range, forcing the dual to land in a different clique
  // than some of the primal support.
  std::vector<Eigen::Triplet<double>> C_trips;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(p);
  for (int i = 0; i < p; ++i) {
    // Row i touches variables near the start AND near the end.
    int v1 = i;
    int v2 = n - 1 - i;
    C_trips.emplace_back(i, v1, 1.0);
    if (v2 != v1) C_trips.emplace_back(i, v2, 1.0);
    d(i) = 1.0 + i;
  }
  Eigen::SparseMatrix<double> C(p, n);
  C.setFromTriplets(C_trips.begin(), C_trips.end());

  Model model;
  model.AddLinearConstraint(A, b, all_vars);
  model.AddQuadraticCost(Q, all_vars);
  model.SetLinearCost(c);
  model.AddEqualityConstraint(C, d, all_vars);

  // Solve with tree solver.
  SolverConfiguration tree_cfg;
  auto tree_solver = Solver::Build(model, tree_cfg);
  ASSERT_NE(tree_solver.tree_solver(), nullptr);
  EXPECT_GT(tree_solver.tree_solver()->num_subsystems(), 1);
  auto tree_cm = tree_solver.MakeCompiledModel();
  auto tree_result = GeodesicLP{1e-10, 5}.Run(tree_cm);

  // Solve with dense solver.
  auto dense_solver = Solver::BuildDense(model);
  auto dense_cm = dense_solver.MakeCompiledModel();
  auto dense_result = GeodesicLP{1e-10, 5}.Run(dense_cm);

  // Both should produce the same mu and d_inf at each iteration.
  ASSERT_EQ(tree_result.iter_stats.size(), dense_result.iter_stats.size());
  for (size_t i = 0; i < tree_result.iter_stats.size(); ++i) {
    double tree_mu = tree_result.iter_stats[i].mu;
    double dense_mu = dense_result.iter_stats[i].mu;
    double rel = std::abs(tree_mu - dense_mu) /
                 std::max(std::abs(dense_mu), 1e-30);
    EXPECT_LT(rel, 1e-10)
        << "mu mismatch at iteration " << i
        << ": tree=" << tree_mu << " dense=" << dense_mu;
  }
}

}  // namespace conex
