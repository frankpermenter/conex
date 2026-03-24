#include "conex/sparse_linear_constraint.h"

#include <numeric>
#include <set>

#include "conex/constraint_manager.h"
#include "conex/kkt_solver_factory.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Build a block-diagonal sparse matrix from dense blocks.
static Eigen::SparseMatrix<double> BlockDiagonal(
    const std::vector<MatrixXd>& blocks) {
  int total_rows = 0, total_cols = 0;
  for (const auto& B : blocks) {
    total_rows += B.rows();
    total_cols += B.cols();
  }
  Eigen::SparseMatrix<double> A(total_rows, total_cols);
  std::vector<Eigen::Triplet<double>> triplets;
  int row_offset = 0, col_offset = 0;
  for (const auto& B : blocks) {
    for (int i = 0; i < B.rows(); i++) {
      for (int j = 0; j < B.cols(); j++) {
        if (B(i, j) != 0.0) {
          triplets.emplace_back(row_offset + i, col_offset + j, B(i, j));
        }
      }
    }
    row_offset += B.rows();
    col_offset += B.cols();
  }
  A.setFromTriplets(triplets.begin(), triplets.end());
  return A;
}

// Helper: build a star-shaped sparse matrix (many spokes sharing a hub).
static Eigen::SparseMatrix<double> StarShaped(int hub_size, int spoke_size,
                                               int num_spokes,
                                               int rows_per_spoke) {
  int num_vars = hub_size + spoke_size * num_spokes;
  int num_rows = rows_per_spoke * num_spokes;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int s = 0; s < num_spokes; s++) {
    int spoke_start = hub_size + s * spoke_size;
    for (int r = 0; r < rows_per_spoke; r++) {
      int row = s * rows_per_spoke + r;
      for (int j = 0; j < hub_size; j++) {
        triplets.emplace_back(
            row, j, 0.1 * (1.0 + static_cast<double>(rand()) / RAND_MAX));
      }
      for (int j = 0; j < spoke_size; j++) {
        triplets.emplace_back(row, spoke_start + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());
  return A;
}

// Helper: build a solver from a sparse matrix.
struct SolverSetup {
  std::unique_ptr<SparseLinearConstraintAssembler> assembler;
  ConstraintManager cm;
};

static SolverSetup MakeSetup(const Eigen::SparseMatrix<double>& A) {
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);
  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  SolverSetup s;
  s.cm = ConstraintManager(A.cols());
  s.assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  s.cm.AddCustomAssembler(s.assembler.get());
  return s;
}

// Test multi-threaded left-looking factorization.
// Uses a star-shaped structure so the root has many children with large blocks,
// triggering the threaded accumulate path in GatherFromChildren.
GTEST_TEST(Multithreading, LeftLookingFactorization) {
  srand(77);
  auto A = StarShaped(20, 10, 30, 15);

  VectorXd x_true = VectorXd::Random(A.cols());
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  // Single-threaded reference.
  auto s1 = MakeSetup(A);
  SolverConfiguration config1;
  config1.num_threads = 1;
  auto solver1 = MakeTreeSolver(&s1.cm, config1);
  ASSERT_TRUE(solver1->AssembleAndFactor());
  VectorXd sol1 = solver1->Solve(rhs);
  EXPECT_NEAR((sol1 - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Multi-threaded (4 threads).
  auto s2 = MakeSetup(A);
  SolverConfiguration config2;
  config2.num_threads = 4;
  auto solver2 = MakeTreeSolver(&s2.cm, config2);
  ASSERT_TRUE(solver2->AssembleAndFactor());
  VectorXd sol2 = solver2->Solve(rhs);
  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol1 - sol2).norm(), 0, 1e-10 * x_true.norm());
}

// Test recursive solve path with parallel backward solve.
// Children have disjoint supernodes so backward solve runs siblings
// concurrently via ApplyInverseOfRightFactor.
GTEST_TEST(Multithreading, ParallelRecursiveSolve) {
  srand(99);
  auto A = StarShaped(15, 8, 20, 12);

  VectorXd x_true = VectorXd::Random(A.cols());
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  // Blocked solve (default) as reference.
  auto s1 = MakeSetup(A);
  SolverConfiguration config1;
  auto solver1 = MakeTreeSolver(&s1.cm, config1);
  ASSERT_TRUE(solver1->AssembleAndFactor());
  VectorXd sol1 = solver1->Solve(rhs);
  EXPECT_NEAR((sol1 - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Recursive solve, single-threaded.
  auto s2 = MakeSetup(A);
  SolverConfiguration config2;
  config2.num_threads = 1;
  auto solver2 = MakeTreeSolver(&s2.cm, config2);
  solver2->SetUseRecursiveSolve(true);
  ASSERT_TRUE(solver2->AssembleAndFactor());
  VectorXd sol2 = solver2->Solve(rhs);
  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol1 - sol2).norm(), 0, 1e-10 * x_true.norm());

  // Recursive solve, multi-threaded (parallel backward solve).
  auto s3 = MakeSetup(A);
  SolverConfiguration config3;
  config3.num_threads = 4;
  auto solver3 = MakeTreeSolver(&s3.cm, config3);
  solver3->SetUseRecursiveSolve(true);
  ASSERT_TRUE(solver3->AssembleAndFactor());
  VectorXd sol3 = solver3->Solve(rhs);
  EXPECT_NEAR((sol3 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol1 - sol3).norm(), 0, 1e-10 * x_true.norm());
}

// Test parallelize-roots-only with block-diagonal sparsity.
// Block-diagonal structure produces independent subtrees (multiple roots).
GTEST_TEST(Multithreading, ParallelizeRootsOnly) {
  srand(33);
  int num_blocks = 8;
  int rows_per_block = 10;
  int cols_per_block = 5;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A_sparse = BlockDiagonal(blocks);
  int num_vars = A_sparse.cols();

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A_sparse);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  // Single-threaded reference.
  auto s1 = MakeSetup(A_sparse);
  SolverConfiguration config1;
  config1.num_threads = 1;
  auto solver1 = MakeTreeSolver(&s1.cm, config1);
  ASSERT_TRUE(solver1->AssembleAndFactor());
  VectorXd sol1 = solver1->Solve(rhs);
  EXPECT_NEAR((sol1 - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Multi-threaded with parallelize_roots_only = true.
  auto s2 = MakeSetup(A_sparse);
  SolverConfiguration config2;
  config2.num_threads = 4;
  auto solver2 = MakeTreeSolver(&s2.cm, config2);
  solver2->SetParallelizeRootsOnly(true);
  ASSERT_TRUE(solver2->AssembleAndFactor());
  VectorXd sol2 = solver2->Solve(rhs);
  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol1 - sol2).norm(), 0, 1e-10 * x_true.norm());

  // Also test with recursive solve path.
  auto s3 = MakeSetup(A_sparse);
  SolverConfiguration config3;
  config3.num_threads = 4;
  auto solver3 = MakeTreeSolver(&s3.cm, config3);
  solver3->SetParallelizeRootsOnly(true);
  solver3->SetUseRecursiveSolve(true);
  ASSERT_TRUE(solver3->AssembleAndFactor());
  VectorXd sol3 = solver3->Solve(rhs);
  EXPECT_NEAR((sol3 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol1 - sol3).norm(), 0, 1e-10 * x_true.norm());
}

}  // namespace conex
