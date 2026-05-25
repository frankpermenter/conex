#include <gtest/gtest.h>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/solver.h"
#include "conex/linear_solvers/kkt_tree_solver.h"

namespace conex {

// Build a sparse QP with equality constraints whose clique tree has
// multiple supernodes.  The sparsity pattern is banded: each constraint
// touches a sliding window of variables, forcing the tree to have
// separators.
TEST(GapConsistency, GeodesicLP_BandedQP) {
  const int n = 30;
  const int m = 20;
  const int p = 5;
  const int bw = 8;

  srand(42);

  // Banded inequality.
  std::vector<Eigen::Triplet<double>> A_trips;
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);
  for (int i = 0; i < m; ++i) {
    int start = (i * (n - bw)) / m;
    for (int j = start; j < start + bw && j < n; ++j)
      A_trips.emplace_back(i, j, 0.1 + 0.5 * std::abs(std::sin(i * 7 + j * 3)));
  }
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(A_trips.begin(), A_trips.end());
  std::vector<int> all_vars(n);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Banded Q.
  Eigen::MatrixXd Q_dense = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    Q_dense(i, i) = 2.0 + 0.1 * i;
    if (i + 1 < n) { Q_dense(i, i+1) = 0.3; Q_dense(i+1, i) = 0.3; }
    if (i + 2 < n) { Q_dense(i, i+2) = 0.1; Q_dense(i+2, i) = 0.1; }
  }
  Eigen::SparseMatrix<double> Q = Q_dense.sparseView();

  Eigen::VectorXd c = Eigen::VectorXd::Random(n) * 0.5;

  // Equality constraints.
  std::vector<Eigen::Triplet<double>> C_trips;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(p);
  for (int i = 0; i < p; ++i) {
    int start = (i * n) / p;
    for (int j = start; j < start + 4 && j < n; ++j)
      C_trips.emplace_back(i, j, 1.0 + 0.1 * j);
    d(i) = 1.0 + i;
  }
  Eigen::SparseMatrix<double> C(p, n);
  C.setFromTriplets(C_trips.begin(), C_trips.end());

  Model model;
  model.AddLinearConstraint(A, b, all_vars);
  model.AddQuadraticCost(Q, all_vars);
  model.SetLinearCost(c);
  model.AddEqualityConstraint(C, d, all_vars);

  SolverConfiguration config;
  auto solver = Solver::Build(model, config);

  // Verify non-trivial tree.
  auto* ts = solver.tree_solver();
  ASSERT_NE(ts, nullptr);
  EXPECT_GT(ts->num_subsystems(), 1)
      << "Need a multi-clique tree for this test";

  // Run GeodesicLP on the CompiledModel to get iter_stats.
  auto cm = solver.MakeCompiledModel();
  GeodesicLP algo{1e-10, 3};
  auto raw = algo.Run(cm);

  ASSERT_GE((int)raw.iter_stats.size(), 3);
  for (int i = 0; i < 3; ++i) {
    double rel_err = raw.iter_stats[i].gap_error;  // already relative
    EXPECT_LT(rel_err, 1e-10)
        << "Relative gap error at iteration " << i << ": " << rel_err;
  }
}

}  // namespace conex
