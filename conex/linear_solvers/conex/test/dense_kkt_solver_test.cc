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

}  // namespace
}  // namespace conex
