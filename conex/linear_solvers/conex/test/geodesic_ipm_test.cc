#include <gtest/gtest.h>

#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

TEST(GeodesicBarrierQP, CentralPathConvergence) {
  // Test: min c^T x  s.t. Ax >= b,  with b = ones(m), c = A^T ones(m).
  //
  // Central path at x=0: s = Ax - b = -b, but with the >= convention
  // s = Ax - b.  At x=0, s = -b = -1 (infeasible).
  //
  // Actually, for >= constraint with slack s = Ax - b >= 0, we need
  // Ax >= b.  At x=0 with b=1 that fails.  So the central path is at
  // some x_* where s = A x_* - 1 = 1.  But x doesn't matter because
  // the geodesic iteration only tracks W.
  //
  // At W = ones, k = 1 (in the API's stored -A, -b):
  //   RHS = cost + (-A)^T(-1 + 2) = A^T 1 - A^T 1 = 0  =>  y = 0
  //   d   = 1 + 1*(-1 - 0) = 0  for every row
  //
  // So W = ones is a fixed point for any A.  Perturb and verify d -> 0.
  srand(42);
  const int n = 5, m = 8;

  // Arbitrary A (rectangular, need not be square).
  std::vector<Eigen::Triplet<double>> trips;
  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Model Ax >= b via -Ax <= -b.
  Eigen::SparseMatrix<double> negA = -A;
  VectorXd neg_b = -b;
  Problem problem;
  problem.AddLinearConstraint(negA, neg_b, vars);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = expansion.Reduce(c);
  cost_rhs = kkt->MakeBlockVariable(c_r);

  // Initialize W = ones + small perturbation.
  RowSpace W = kkt->MakeRowSpace();
  setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

  // Phase 1: center at k = 1.
  double k = 1.0;
  auto result = GeodesicCenter(*kkt, cost_rhs, W, k, 100, 1e-10);

  printf("k=%.2f: %d iters, ||d||_inf=%.2e\n",
         k, result.iterations, result.d_inf_norm);
  EXPECT_LT(result.d_inf_norm, 1e-8);
  RowSpace W_ones = kkt->MakeRowSpace();
  setOnes(W_ones);
  EXPECT_NEAR(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.0, 1e-6);

  // Phase 2: line-search for k, then re-center.
  const int num_updates = 6;
  for (int step = 0; step < num_updates; ++step) {
    double k_new = GeodesicLineSearch(*kkt, cost_rhs, W);
    EXPECT_GE(k_new, k);

    k = k_new;
    result = GeodesicCenter(*kkt, cost_rhs, W, k, 100, 1e-10);
    printf("k=%.4f  mu=%.2e: %d iters, d_inf=%.2e, d_sqr=%.2e, "
           "s_dot_x=%.2e\n",
           k, result.mu, result.iterations, result.d_inf_norm,
           result.d_sq_norm, result.complementarity);
    EXPECT_LT(result.d_inf_norm, 1e-8);
  }

  // After increasing k, W should have moved away from ones.
  EXPECT_GT(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.01);
}

TEST(GeodesicBarrierQP, PerComponentR) {
  // Verify that GeodesicCenterR with r = ones reduces to GeodesicCenter
  // with k = 1 (since sqrt(mu) = 1/k = 1 when k = 1).
  srand(42);
  const int n = 5, m = 8;

  MatrixXd A_dense = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Eigen::SparseMatrix<double> negA = -A;
  VectorXd neg_b = -b;
  Problem problem;
  problem.AddLinearConstraint(negA, neg_b, vars);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = expansion.Reduce(c);
  cost_rhs = kkt->MakeBlockVariable(c_r);

  // Scalar version: center at k=1.
  VectorXd W_init = VectorXd::Ones(m) + 0.01 * VectorXd::Random(m);
  RowSpace W1 = kkt->MakeRowSpace();
  setFromVector(W1, W_init);
  RowSpace W2 = kkt->MakeRowSpace();
  setFromVector(W2, W_init);  // same initial W
  auto r1 = GeodesicCenter(*kkt, cost_rhs, W1, 1.0, 100, 1e-10);

  // Per-component version: r = ones (equivalent to sqrt(mu)=1, k=1).
  RowSpace r = kkt->MakeRowSpace();
  setOnes(r);
  auto r2 = GeodesicCenterR(*kkt, cost_rhs, W2, r, 1.0, 100, 1e-10);

  EXPECT_EQ(r1.iterations, r2.iterations);
  EXPECT_NEAR(normInf(addScaled(W1, W2, 1.0, -1.0)), 0.0, 1e-12);

  // Non-uniform r: should still converge (d -> 0).
  RowSpace W3 = kkt->MakeRowSpace();
  setFromVector(W3, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));
  RowSpace r_nonuniform = kkt->MakeRowSpace();
  setFromVector(r_nonuniform,
                VectorXd::Ones(m) + 0.5 * VectorXd::Random(m).cwiseAbs());
  auto r3 = GeodesicCenterR(*kkt, cost_rhs, W3, r_nonuniform, 1.0, 100, 1e-10);
  EXPECT_LT(r3.d_inf_norm, 1e-8);

  printf("PerComponentR: scalar=%d iters, r=ones=%d iters, r=nonuniform=%d iters\n",
         r1.iterations, r2.iterations, r3.iterations);
}
TEST(GeodesicBarrierQP, MultipleConstraints) {
  // Geodesic IPM on a problem with two separate AddLinearConstraint calls.
  // min c^T x s.t. A1*x >= b1, A2*x >= b2.
  // Central path at W=ones when b=ones and c = (A1^T + A2^T) * ones.
  srand(99);
  const int n = 6, m1 = 10, m2 = 8;

  MatrixXd A1_dense = MatrixXd::Random(m1, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m1, n);
  MatrixXd A2_dense = MatrixXd::Random(m2, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m2, n);

  auto toSparse = [](const MatrixXd& M) {
    std::vector<Eigen::Triplet<double>> trips;
    for (int i = 0; i < M.rows(); ++i)
      for (int j = 0; j < M.cols(); ++j)
        trips.emplace_back(i, j, M(i, j));
    Eigen::SparseMatrix<double> S(M.rows(), M.cols());
    S.setFromTriplets(trips.begin(), trips.end());
    return S;
  };

  Eigen::SparseMatrix<double> A1 = toSparse(A1_dense);
  Eigen::SparseMatrix<double> A2 = toSparse(A2_dense);
  VectorXd b1 = VectorXd::Ones(m1);
  VectorXd b2 = VectorXd::Ones(m2);
  // c = A1^T ones + A2^T ones (central path at W=ones for Ax >= b).
  VectorXd c = A1.transpose() * VectorXd::Ones(m1) +
               A2.transpose() * VectorXd::Ones(m2);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // Model Ax >= b via -Ax <= -b.
  Eigen::SparseMatrix<double> negA1 = -A1;
  Eigen::SparseMatrix<double> negA2 = -A2;
  VectorXd neg_b1 = -b1;
  VectorXd neg_b2 = -b2;
  Problem problem;
  problem.AddLinearConstraint(negA1, neg_b1, vars);
  problem.AddLinearConstraint(negA2, neg_b2, vars);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = expansion.Reduce(c);
  cost_rhs = kkt->MakeBlockVariable(c_r);

  const int m = m1 + m2;

  // --- Test 1: GeodesicCenter at k=1, W=ones is fixed point ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    // Small perturbation.
    setFromVector(W, VectorXd::Ones(m) + 0.01 * VectorXd::Random(m));

    auto result = GeodesicCenter(*kkt, cost_rhs, W, 1.0, 100, 1e-10);
    EXPECT_LT(result.d_inf_norm, 1e-8);

    // W should return to ones.
    RowSpace W_ones = kkt->MakeRowSpace();
    setOnes(W_ones);
    EXPECT_NEAR(normInf(addScaled(W, W_ones, 1.0, -1.0)), 0.0, 1e-6);

    printf("MultipleConstraints center: %d iters, d_inf=%.2e\n",
           result.iterations, result.d_inf_norm);
  }

  // --- Test 2: SolveGeodesicLP (0 centering) ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8);
    EXPECT_LT(result.complementarity, 1e-7);

    printf("MultipleConstraints geodesic: %d fac, %d sol, gap=%.2e\n",
           result.total_factorizations, result.total_solves,
           result.complementarity);
  }

  // --- Test 3: SolveGeodesicHybrid ---
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);

    auto result = SolveGeodesicHybrid(*kkt, cost_rhs, W, 50, 1e-8);
    EXPECT_LT(std::abs(result.complementarity), 1e-7);

    printf("MultipleConstraints hybrid: %d fac, %d sol, gap=%.2e\n",
           result.total_factorizations, result.total_solves,
           result.complementarity);
  }

  // --- Test 4: Verify RowSpace has correct number of segments ---
  {
    RowSpace W = kkt->MakeRowSpace();
    // With 2 constraints decomposed across cliques, num_constraints >= 2.
    EXPECT_GE(W.num_constraints(), 2);
    // Total rows = m1 + m2.
    EXPECT_EQ(W.total_rows(), m);
    // Every segment has ops.
    for (int i = 0; i < W.num_constraints(); ++i) {
      EXPECT_NE(W.ops[i], nullptr);
    }

    printf("MultipleConstraints segments: %d (from %d + %d rows)\n",
           W.num_constraints(), m1, m2);
  }
}

}  // namespace
}  // namespace conex
