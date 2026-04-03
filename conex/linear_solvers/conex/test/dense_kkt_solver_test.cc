#include "conex/common/dense_kkt_solver.h"
#include "conex/algorithms/barrier_qp.h"
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

TEST(DenseKKTSolver, BarrierQPGenericInterface) {
  // Solve min 0.5 x^T Q x + c^T x  s.t. A x <= b
  // using the generic SolveBarrierQP(KKTSolverBase&, ...) interface
  // with DenseKKTSolver providing the constraint-based overrides.
  //
  // Problem: min 0.5 ||x||^2  s.t.  x_i <= 1  (box constraints, n=2)
  // Optimal: x* = (0, 0), objective = 0.
  const int n = 2;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(n);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(n, n);  // x <= 1
  Eigen::VectorXd b = Eigen::VectorXd::Ones(n);

  DenseKKTSolver solver(n);
  solver.SetConstraintData(Q, A, b);

  auto c_rhs = solver.MakeTreeRHS();
  c_rhs.supernodes->ScatterFrom(c);
  c_rhs.blocks_fully_gathered = true;

  auto x = solver.MakeTreeRHS();
  x.supernodes->SetZero();  // feasible start at origin
  x.blocks_fully_gathered = true;

  auto result = SolveBarrierQP(solver, c_rhs, x);

  EXPECT_LT(result.x.norm(), 1e-4);
  EXPECT_LT(std::abs(result.objective), 1e-6);
  EXPECT_LT(result.duality_gap, 1e-6);
}

TEST(DenseKKTSolver, BarrierQPNonTrivial) {
  // min 0.5 x^T Q x + c^T x  s.t. A x <= b
  // Q = I, c = (-1, -1), A = [1 0; 0 1; 1 1], b = (2, 2, 3)
  // Unconstrained min at (1,1); all constraints satisfied there.
  const int n = 2;
  const int m = 3;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n);
  Eigen::VectorXd c(n);
  c << -1, -1;
  Eigen::MatrixXd A(m, n);
  A << 1, 0,
       0, 1,
       1, 1;
  Eigen::VectorXd b(m);
  b << 2, 2, 3;

  DenseKKTSolver solver(n);
  solver.SetConstraintData(Q, A, b);

  auto c_rhs = solver.MakeTreeRHS();
  c_rhs.supernodes->ScatterFrom(c);
  c_rhs.blocks_fully_gathered = true;

  // Feasible start at origin.
  auto x = solver.MakeTreeRHS();
  x.supernodes->SetZero();
  x.blocks_fully_gathered = true;

  auto result = SolveBarrierQP(solver, c_rhs, x);

  // Optimal: x* = (1, 1), objective = 0.5*(1+1) + (-1-1) = -1.
  EXPECT_NEAR(result.x(0), 1.0, 1e-5);
  EXPECT_NEAR(result.x(1), 1.0, 1e-5);
  EXPECT_NEAR(result.objective, -1.0, 1e-5);
}

}  // namespace
}  // namespace conex
