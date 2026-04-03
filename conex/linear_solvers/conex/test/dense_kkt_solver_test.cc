#include "conex/common/dense_kkt_solver.h"
#include <gtest/gtest.h>

namespace conex {
namespace {

TEST(DenseKKTSolver, SolveSPD) {
  // 3x3 SPD system: M x = b.
  Eigen::MatrixXd M(3, 3);
  M << 4, 2, 1,
       2, 5, 3,
       1, 3, 6;
  Eigen::VectorXd b(3);
  b << 1, 2, 3;

  DenseKKTSolver solver(3);
  solver.SetMatrix(M);
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::VectorXd x = solver.Solve(b);
  Eigen::VectorXd expected = M.llt().solve(b);
  EXPECT_LT((x - expected).norm(), 1e-12);
}

TEST(DenseKKTSolver, PartitionScatterGather) {
  // Verify the partition scatter/gather round-trips.
  DenseKKTSolver solver(4);
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(4, 4) * 2.0;
  solver.SetMatrix(M);
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::VectorXd b(4);
  b << 1, 2, 3, 4;
  Eigen::VectorXd x = solver.Solve(b);

  // After Solve, partition should contain the solution.
  auto& partition = solver.partition();
  EXPECT_EQ(partition.num_blocks(), 1);
  EXPECT_EQ(partition.block_size(0), 4);
  EXPECT_EQ(partition.num_variables(), 4);

  // Gather from partition should recover x.
  Eigen::VectorXd gathered(4);
  partition.GatherInto(gathered);
  EXPECT_LT((gathered - x).norm(), 1e-12);
}

TEST(DenseKKTSolver, PolymorphicAccess) {
  // Use through KKTSolverBase pointer.
  auto solver = std::make_unique<DenseKKTSolver>(2);
  Eigen::MatrixXd M(2, 2);
  M << 3, 1,
       1, 2;
  solver->SetMatrix(M);

  KKTSolverBase* base = solver.get();
  ASSERT_TRUE(base->AssembleAndFactor());
  EXPECT_EQ(base->number_of_variables(), 2);

  Eigen::VectorXd b(2);
  b << 5, 4;
  Eigen::VectorXd x = base->Solve(b);
  Eigen::VectorXd expected = M.llt().solve(b);
  EXPECT_LT((x - expected).norm(), 1e-12);

  // Scatter/gather via base.
  base->ScatterToBlocks(b);
  Eigen::VectorXd out(2);
  base->GatherFromBlocks(out);
  EXPECT_LT((out - b).norm(), 1e-12);
}

TEST(DenseKKTSolver, AddToMatrixAndResolve) {
  // Build Q + A^T A incrementally.
  int n = 3;
  DenseKKTSolver solver(n);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * 0.1;
  Eigen::MatrixXd A(2, 3);
  A << 1, 2, 0,
       0, 1, 3;

  solver.ZeroMatrix();
  solver.AddToMatrix(Q);
  solver.AddToMatrix(A.transpose() * A);
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::VectorXd rhs = A.transpose() * Eigen::VectorXd::Ones(2);
  Eigen::VectorXd x = solver.Solve(rhs);

  Eigen::VectorXd expected = (Q + A.transpose() * A).llt().solve(rhs);
  EXPECT_LT((x - expected).norm(), 1e-10);

  // Re-solve with different weights (2*A^T A).
  solver.ZeroMatrix();
  solver.AddToMatrix(Q);
  solver.AddToMatrix(2.0 * A.transpose() * A);
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::VectorXd x2 = solver.Solve(rhs);
  Eigen::VectorXd expected2 = (Q + 2.0 * A.transpose() * A).llt().solve(rhs);
  EXPECT_LT((x2 - expected2).norm(), 1e-10);
}

TEST(DenseKKTSolver, SeparateAssembleAndFactor) {
  // Exercise the separate Assemble() + Factor() path (vs AssembleAndFactor).
  int n = 3;
  DenseKKTSolver solver(n);
  Eigen::MatrixXd M(3, 3);
  M << 4, 1, 0,
       1, 3, 1,
       0, 1, 2;
  solver.SetMatrix(M);

  solver.Assemble();

  // KKTMatrix should return the assembled matrix before factoring.
  Eigen::MatrixXd K = solver.KKTMatrix();
  EXPECT_LT((K - M).norm(), 1e-12);

  ASSERT_TRUE(solver.Factor());

  Eigen::VectorXd b(3);
  b << 1, 2, 3;
  Eigen::VectorXd x = solver.Solve(b);
  EXPECT_LT((M * x - b).norm(), 1e-10);
}

TEST(DenseKKTSolver, BlockVariableRoundTrip) {
  // Exercise MakeBlockVariable, SolveInto, DenseBlockPartition::block().
  int n = 3;
  DenseKKTSolver solver(n);
  Eigen::MatrixXd M = Eigen::MatrixXd::Identity(n, n) * 2.0;
  solver.SetMatrix(M);
  ASSERT_TRUE(solver.AssembleAndFactor());

  Eigen::VectorXd b(3);
  b << 2, 4, 6;
  auto rhs = solver.MakeBlockVariable(b);
  auto dest = solver.MakeBlockVariable();

  // Access block(0) on the partition.
  EXPECT_EQ(rhs.partition().num_blocks(), 1);
  auto blk = rhs.partition().block(0);
  EXPECT_EQ(blk.rows(), n);
  EXPECT_LT((blk.col(0) - b).norm(), 1e-12);

  solver.SolveInto(rhs, dest);

  Eigen::VectorXd x = dest.Gather();
  Eigen::VectorXd expected = M.llt().solve(b);
  EXPECT_LT((x - expected).norm(), 1e-12);
}

TEST(DenseKKTSolver, MakePartition) {
  // MakePartition returns a standalone copy matching the solver's structure.
  DenseKKTSolver solver(5);
  auto p = solver.MakePartition();
  EXPECT_EQ(p->num_blocks(), 1);
  EXPECT_EQ(p->block_size(0), 5);
  EXPECT_EQ(p->num_variables(), 5);

  p->Resize(2);
  p->SetZero();
  auto blk = p->block(0);
  EXPECT_EQ(blk.rows(), 5);
  EXPECT_EQ(blk.cols(), 2);
}

TEST(DenseKKTSolver, BarrierQPManual) {
  // Solve min 0.5 x^T Q x + c^T x  s.t. A x <= b
  // using DenseKKTSolver to form the weighted normal equation at each step.
  //
  // Problem: min 0.5 ||x||^2  s.t.  x_i <= 1  (box constraints, n=2)
  // Optimal: x* = (0, 0).
  const int n = 2;
  const int m = 2;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(m, n);  // x <= 1
  Eigen::VectorXd b = Eigen::VectorXd::Ones(m);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);  // feasible start

  double t = 1.0;
  const double mu = 10.0;
  const double tol = 1e-8;

  for (int outer = 0; outer < 20; ++outer) {
    double gap = static_cast<double>(m) / t;
    if (gap < tol) break;

    for (int newton = 0; newton < 50; ++newton) {
      Eigen::VectorXd s = b - A * x;
      if (s.minCoeff() <= 0) break;

      // Weights: w_i = 1/(t * s_i^2)
      Eigen::VectorXd w(m);
      for (int i = 0; i < m; ++i) w(i) = 1.0 / (t * s(i) * s(i));

      // Weighted KKT: (Q + A^T diag(w) A)
      Eigen::MatrixXd H = Q + A.transpose() * w.asDiagonal() * A;

      // Gradient: Q x + c + (1/t) A^T (1/s)
      Eigen::VectorXd inv_s(m);
      for (int i = 0; i < m; ++i) inv_s(i) = 1.0 / (t * s(i));
      Eigen::VectorXd grad = Q * x + c + A.transpose() * inv_s;

      // Solve H dx = -grad using DenseKKTSolver.
      DenseKKTSolver solver(n);
      solver.SetMatrix(H);
      ASSERT_TRUE(solver.AssembleAndFactor());
      Eigen::VectorXd dx = solver.Solve(-grad);

      double lambda_sq = grad.dot(-dx);
      if (-lambda_sq / 2.0 < tol * 0.01) break;

      // Step with feasibility check.
      double alpha = 1.0;
      Eigen::VectorXd Adx = A * dx;
      for (int i = 0; i < m; ++i)
        if (Adx(i) > 0) alpha = std::min(alpha, 0.99 * s(i) / Adx(i));

      x += alpha * dx;
    }
    t *= mu;
  }

  // Optimal is x* = (0,0), objective = 0.
  EXPECT_LT(x.norm(), 1e-6);
  double obj = 0.5 * x.dot(Q * x) + c.dot(x);
  EXPECT_LT(std::abs(obj), 1e-10);
}

}  // namespace
}  // namespace conex
