#include "conex/sparse_linear_constraint.h"

#include <iostream>
#include <numeric>
#include <set>

#include "conex/cholesky_solvers.h"
#include "conex/clique_ordering.h"
#include "conex/constraint_manager.h"
#include "conex/equality_constraint.h"
#include "conex/kkt_solver_factory.h"
#include "conex/kkt_tree_solver.h"
#include "conex/static_subsystem.h"
#include "conex/tree_utils.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Build a block-diagonal sparse matrix from dense blocks.
Eigen::SparseMatrix<double> BlockDiagonal(
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

GTEST_TEST(SparseLinearConstraint, Decomposition) {
  std::vector<MatrixXd> blocks = {
      MatrixXd::Random(4, 2), MatrixXd::Random(3, 3), MatrixXd::Random(5, 2)};
  auto A = BlockDiagonal(blocks);
  VectorXd b = VectorXd::Ones(A.rows());

  SparseLinearConstraint slc(A, b);
  EXPECT_EQ(static_cast<int>(slc.row_supports().size()), 3);

  auto groups = slc.GetConstraints(slc.row_supports());
  EXPECT_EQ(static_cast<int>(groups.size()), 3);
  int total_rows = 0;
  for (const auto& g : groups) {
    total_rows += g.A.rows();
  }
  EXPECT_EQ(total_rows, A.rows());
}

GTEST_TEST(SparseLinearConstraint, GetConstraintsWithSupersets) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.emplace_back(0, 0, 1.0);
  triplets.emplace_back(0, 1, 2.0);
  triplets.emplace_back(0, 2, 3.0);
  triplets.emplace_back(1, 0, 4.0);
  triplets.emplace_back(1, 1, 5.0);
  triplets.emplace_back(2, 1, 6.0);
  triplets.emplace_back(2, 2, 7.0);
  triplets.emplace_back(3, 3, 8.0);
  triplets.emplace_back(3, 4, 9.0);

  Eigen::SparseMatrix<double> A(4, 5);
  A.setFromTriplets(triplets.begin(), triplets.end());
  VectorXd b(4);
  b << 10, 11, 12, 13;

  SparseLinearConstraint slc(A, b);
  EXPECT_EQ(static_cast<int>(slc.row_supports().size()), 4);

  std::vector<std::vector<int>> targets = {{0, 1, 2}, {3, 4}};
  auto groups = slc.GetConstraints(targets);
  EXPECT_EQ(static_cast<int>(groups.size()), 2);

  bool found_big = false, found_small = false;
  for (const auto& g : groups) {
    if (g.A.rows() == 3) {
      EXPECT_EQ(g.A.cols(), 3);
      found_big = true;
    }
    if (g.A.rows() == 1) {
      EXPECT_EQ(g.A.cols(), 2);
      found_small = true;
    }
  }
  EXPECT_TRUE(found_big);
  EXPECT_TRUE(found_small);
}

GTEST_TEST(SparseLeastSquares, BlockDiagonal) {
  srand(99);
  int num_blocks = 5;
  int rows_per_block = 8;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A = BlockDiagonal(blocks);
  int num_vars = A.cols();

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  auto result = SparseLeastSquares(A, rhs);

  EXPECT_NEAR((result.x - x_true).norm(), 0, 1e-8 * x_true.norm());

  std::cout << "SparseLeastSquares (block-diagonal " << A.rows() << "x"
            << A.cols() << ", " << A.nonZeros() << " nnz):\n"
            << "  construction:       " << result.construction_time_us
            << " us\n"
            << "  assemble_and_factor: " << result.assemble_and_factor_time_us
            << " us\n"
            << "  solve:              " << result.solve_time_us << " us\n";
}

GTEST_TEST(SparseLeastSquares, Banded) {
  srand(42);
  int num_vars = 50;
  int bandwidth = 5;
  int rows_per_group = 10;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  auto result = SparseLeastSquares(A, rhs);

  EXPECT_NEAR((result.x - x_true).norm(), 0, 1e-8 * x_true.norm());

  std::cout << "SparseLeastSquares (banded " << A.rows() << "x" << A.cols()
            << ", " << A.nonZeros() << " nnz):\n"
            << "  construction:       " << result.construction_time_us
            << " us\n"
            << "  assemble_and_factor: " << result.assemble_and_factor_time_us
            << " us\n"
            << "  solve:              " << result.solve_time_us << " us\n";
}

GTEST_TEST(SparseLinearConstraintAssembler, CliqueTreeComparison) {
  srand(42);
  int num_vars = 80;
  int bandwidth = 10;
  int rows_per_group = 8;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A_sparse(num_rows, num_vars);
  A_sparse.setFromTriplets(triplets.begin(), triplets.end());
  VectorXd b_affine = VectorXd::Ones(num_rows) * 2.0;

  SparseLinearConstraint slc(A_sparse, b_affine);

  std::vector<std::vector<int>> maximal_cliques;
  CliqueTree tree_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      slc.row_supports(), &maximal_cliques,
      /*max_merge_supernode_size=*/0, SUPERNODE_REORDER_BFS_GREEDY, {});

  std::cout << "\n=== Tree solver clique tree ===\n";
  std::cout << "Input supports: " << slc.row_supports().size() << "\n";
  std::cout << "Maximal cliques: " << maximal_cliques.size() << "\n";
  std::cout << "Tree nodes: " << tree_tree.supernodes.size() << "\n";
  int tree_with_supernodes = 0;
  int tree_total_fill = 0;
  for (size_t i = 0; i < tree_tree.supernodes.size(); i++) {
    int sn_size = tree_tree.supernodes[i].size();
    int sep_size = tree_tree.separators[i].size();
    tree_total_fill += sn_size * (sn_size + sep_size);
    if (sn_size > 0) tree_with_supernodes++;
    if (i < 20 || sn_size == 0) {
      std::cout << "  node " << i << ": sn=" << sn_size
                << " sep=" << sep_size
                << " total=" << sn_size + sep_size
                << (sn_size == 0 ? " *** NO SUPERNODE ***" : "")
                << "\n";
    }
  }
  if (tree_tree.supernodes.size() > 20) {
    std::cout << "  ... (" << tree_tree.supernodes.size() - 20 << " more nodes)\n";
  }
  std::cout << "Nodes with supernodes: " << tree_with_supernodes
            << " / " << tree_tree.supernodes.size() << "\n";
  std::cout << "Total fill (sum of sn*(sn+sep)): " << tree_total_fill << "\n";
}

// Constrained sparse least squares: min ||Ax||^2 s.t. Cx = d.
// The KKT system is:
//   [A^T A   C^T] [x]   [0]
//   [C       0  ] [λ] = [d]
GTEST_TEST(SparseLeastSquares, EqualityConstraint) {
  srand(77);
  int num_blocks = 3;
  int rows_per_block = 6;
  int cols_per_block = 4;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A_sparse = BlockDiagonal(blocks);
  int num_vars = A_sparse.cols();

  // Equality constraint: sum of first block's variables = 1.
  // C is 1 x num_vars, nonzero on columns 0..cols_per_block-1.
  MatrixXd C = MatrixXd::Zero(1, cols_per_block);
  C.setOnes();
  VectorXd d(1);
  d(0) = 1.0;
  std::vector<int> eq_vars(cols_per_block);
  std::iota(eq_vars.begin(), eq_vars.end(), 0);

  // Build the system using ConstraintManager + MakeTreeSolver.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_sparse.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_sparse, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());
  cm.AddEqualityConstraint(EqualityConstraints(C, d), eq_vars);

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  // RHS: [0; d] — zero for the primal part, d for the equality constraint.
  int kkt_size = cm.SizeOfKKTSystem();
  VectorXd rhs = VectorXd::Zero(kkt_size);
  // The equality constraint RHS goes into the dual variable positions.
  rhs(num_vars) = d(0);

  bool ok = tree_solver->AssembleAndFactor();
  ASSERT_TRUE(ok);

  VectorXd sol = tree_solver->Solve(rhs);
  VectorXd x = sol.head(num_vars);

  // Verify equality constraint: C * x_{first block} = 1.
  double cx = 0;
  for (int i = 0; i < cols_per_block; i++) {
    cx += x(i);
  }
  EXPECT_NEAR(cx, 1.0, 1e-10);

  // Verify optimality via dense reference.
  MatrixXd A_dense(A_sparse);
  MatrixXd ATA = A_dense.transpose() * A_dense;
  // Dense KKT system:
  //   [ATA  C_full^T] [x]   [0]
  //   [C_full    0  ] [λ] = [d]
  MatrixXd C_full = MatrixXd::Zero(1, num_vars);
  for (int i = 0; i < cols_per_block; i++) {
    C_full(0, i) = 1.0;
  }
  MatrixXd KKT = MatrixXd::Zero(num_vars + 1, num_vars + 1);
  KKT.topLeftCorner(num_vars, num_vars) = ATA;
  KKT.bottomLeftCorner(1, num_vars) = C_full;
  KKT.topRightCorner(num_vars, 1) = C_full.transpose();
  VectorXd rhs_dense = VectorXd::Zero(num_vars + 1);
  rhs_dense(num_vars) = d(0);

  VectorXd sol_dense = KKT.fullPivLu().solve(rhs_dense);
  VectorXd x_dense = sol_dense.head(num_vars);

  EXPECT_NEAR((x - x_dense).norm(), 0, 1e-8);
}

// Test PQ tree reordering with non-trivial clique merging.
// Uses a banded + overlapping structure that produces many small cliques,
// then merges them with a large max_merge_supernode_size.
GTEST_TEST(SparseLeastSquares, PQTreeWithCliqueMerging) {
  srand(123);
  int num_vars = 60;
  int bandwidth = 6;
  int rows_per_group = 5;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  // Build via ConstraintManager + MakeTreeSolver with PQ tree reordering
  // and aggressive clique merging (max_merge_supernode_size = 20).
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration config;
  config.tree.supernode_reorder_method = SUPERNODE_REORDER_PQ_TREE;
  config.tree.max_merge_supernode_size = 20;
  auto tree_solver = MakeTreeSolver(&cm, config);

  bool ok = tree_solver->AssembleAndFactor();
  ASSERT_TRUE(ok);

  VectorXd sol = tree_solver->Solve(rhs);

  EXPECT_NEAR((sol - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Also verify with default BFS_GREEDY reordering and no merging for
  // comparison.
  auto slc2 = std::make_unique<SparseLinearConstraint>(A, b_zero);
  ConstraintManager cm2(num_vars);
  auto assembler2 = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc2), all_vars);
  cm2.AddCustomAssembler(assembler2.get());

  SolverConfiguration config2;
  config2.tree.supernode_reorder_method = SUPERNODE_REORDER_BFS_GREEDY;
  config2.tree.max_merge_supernode_size = 0;  // no merging
  auto tree_solver2 = MakeTreeSolver(&cm2, config2);

  bool ok2 = tree_solver2->AssembleAndFactor();
  ASSERT_TRUE(ok2);

  VectorXd sol2 = tree_solver2->Solve(rhs);

  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Both methods should produce the same answer.
  EXPECT_NEAR((sol - sol2).norm(), 0, 1e-8 * x_true.norm());
}

// Test aggressive supernode merging (large max_merge_supernode_size).
// A 5-variable bandwidth on a small system produces many tiny supernodes;
// setting merge threshold to 10 forces most of them to be absorbed into
// their parents, producing fewer, larger dense blocks.
GTEST_TEST(SparseLeastSquares, AggressiveCliqueMerging) {
  srand(55);
  int num_vars = 30;
  int bandwidth = 5;
  int rows_per_group = 6;
  int num_groups = num_vars - bandwidth + 1;
  int num_rows = rows_per_group * num_groups;

  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < num_groups; g++) {
    for (int r = 0; r < rows_per_group; r++) {
      int row = g * rows_per_group + r;
      for (int j = 0; j < bandwidth; j++) {
        triplets.emplace_back(row, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
      }
    }
  }
  Eigen::SparseMatrix<double> A(num_rows, num_vars);
  A.setFromTriplets(triplets.begin(), triplets.end());

  VectorXd x_true = VectorXd::Random(num_vars);
  MatrixXd A_dense(A);
  VectorXd rhs = A_dense.transpose() * (A_dense * x_true);

  // Solve with aggressive merging (threshold = 10).
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);
  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration config;
  config.tree.max_merge_supernode_size = 10;
  auto tree_solver = MakeTreeSolver(&cm, config);

  bool ok = tree_solver->AssembleAndFactor();
  ASSERT_TRUE(ok);

  VectorXd sol = tree_solver->Solve(rhs);
  EXPECT_NEAR((sol - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Solve with no merging for comparison.
  auto slc2 = std::make_unique<SparseLinearConstraint>(A, b_zero);
  ConstraintManager cm2(num_vars);
  auto assembler2 = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc2), all_vars);
  cm2.AddCustomAssembler(assembler2.get());

  SolverConfiguration config2;
  config2.tree.max_merge_supernode_size = 0;
  auto tree_solver2 = MakeTreeSolver(&cm2, config2);

  bool ok2 = tree_solver2->AssembleAndFactor();
  ASSERT_TRUE(ok2);

  VectorXd sol2 = tree_solver2->Solve(rhs);
  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_NEAR((sol - sol2).norm(), 0, 1e-8 * x_true.norm());
}

// Direct test of the MergeChildIntoParent utility function.
GTEST_TEST(TreeUtils, MergeChildIntoParent) {
  // Build a small clique tree manually:
  //   node 0 (root): sn={4,5}, sep={}
  //   node 1:         sn={2,3}, sep={4}   parent=0
  //   node 2:         sn={0,1}, sep={2}   parent=1
  CliqueTree ct;
  ct.supernodes = {{4, 5}, {2, 3}, {0, 1}};
  ct.separators = {{}, {4}, {2}};
  ct.node_to_parent = {-1, 0, 1};
  ct.post_order_position_to_clique = {2, 1, 0};

  // Merge node 2 (child) into node 1 (its parent).
  MergeChildIntoParent(ct, 2);

  // After merge: 2 nodes remain.
  ASSERT_EQ(static_cast<int>(ct.supernodes.size()), 2);
  ASSERT_EQ(static_cast<int>(ct.separators.size()), 2);
  ASSERT_EQ(static_cast<int>(ct.node_to_parent.size()), 2);

  // Node 1 (now index 1) should have absorbed {0,1} from the old node 2.
  // Its supernode should be {0, 1, 2, 3} (sorted).
  std::vector<int> expected_sn = {0, 1, 2, 3};
  EXPECT_EQ(ct.supernodes[1], expected_sn);

  // Separator of the merged node should still be {4}.
  std::vector<int> expected_sep = {4};
  EXPECT_EQ(ct.separators[1], expected_sep);

  // Root node should be unchanged.
  std::vector<int> expected_root_sn = {4, 5};
  EXPECT_EQ(ct.supernodes[0], expected_root_sn);
  EXPECT_EQ(ct.node_to_parent[0], -1);

  // Node 1's parent should be 0 (root).
  EXPECT_EQ(ct.node_to_parent[1], 0);

  // post_order should have been cleared (invalidated by merge).
  EXPECT_TRUE(ct.post_order_position_to_clique.empty());
}

// Test Assemble() + KKTMatrix() + Factor() + Solve() separately.
GTEST_TEST(SparseLeastSquares, AssembleThenKKTMatrix) {
  srand(88);
  int num_blocks = 3;
  int rows_per_block = 6;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A_sparse = BlockDiagonal(blocks);
  int num_vars = A_sparse.cols();

  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_sparse.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_sparse, b_zero);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration config;
  auto tree_solver = MakeTreeSolver(&cm, config);

  // Assemble without factoring.
  tree_solver->Assemble();

  // KKTMatrix should return A^T * A (in original variable order).
  MatrixXd KKT = tree_solver->KKTMatrix();
  MatrixXd A_dense(A_sparse);
  MatrixXd ATA = A_dense.transpose() * A_dense;

  EXPECT_NEAR((KKT - ATA).norm(), 0, 1e-10 * ATA.norm());

  // Now factor and solve.
  bool ok = tree_solver->Factor();
  ASSERT_TRUE(ok);

  VectorXd x_true = VectorXd::Random(num_vars);
  VectorXd rhs = ATA * x_true;
  VectorXd sol = tree_solver->Solve(rhs);

  EXPECT_NEAR((sol - x_true).norm(), 0, 1e-8 * x_true.norm());
}

// Test multi-column solve with pre-allocated workspace.
GTEST_TEST(SparseLeastSquares, MultiColumnSolve) {
  srand(44);
  int num_blocks = 4;
  int rows_per_block = 8;
  int cols_per_block = 3;

  std::vector<MatrixXd> blocks(num_blocks);
  for (int i = 0; i < num_blocks; i++) {
    blocks[i] = MatrixXd::Random(rows_per_block, cols_per_block);
  }
  auto A_sparse = BlockDiagonal(blocks);
  int num_vars = A_sparse.cols();

  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(A_sparse.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A_sparse, b_zero);
  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(num_vars);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  cm.AddCustomAssembler(assembler.get());

  SolverConfiguration config;
  config.rhs_cols = 3;
  auto solver = MakeTreeSolver(&cm, config);

  ASSERT_TRUE(solver->AssembleAndFactor());

  MatrixXd A_dense(A_sparse);
  MatrixXd ATA = A_dense.transpose() * A_dense;

  // Solve with 1 column.
  VectorXd x_true_1 = VectorXd::Random(num_vars);
  VectorXd rhs_1 = ATA * x_true_1;
  VectorXd sol_1 = solver->Solve(rhs_1);
  EXPECT_NEAR((sol_1 - x_true_1).norm(), 0, 1e-8 * x_true_1.norm());

  // Solve with 2 columns.
  MatrixXd X_true_2 = MatrixXd::Random(num_vars, 2);
  MatrixXd rhs_2 = ATA * X_true_2;
  MatrixXd sol_2 = solver->Solve(rhs_2);
  EXPECT_NEAR((sol_2 - X_true_2).norm(), 0, 1e-8 * X_true_2.norm());

  // Solve with 3 columns.
  MatrixXd X_true_3 = MatrixXd::Random(num_vars, 3);
  MatrixXd rhs_3 = ATA * X_true_3;
  MatrixXd sol_3 = solver->Solve(rhs_3);
  EXPECT_NEAR((sol_3 - X_true_3).norm(), 0, 1e-8 * X_true_3.norm());
}

namespace {

// Helper: build solver, optionally enable scatter_to_parent, solve A^T A x = rhs.
// Returns relative error vs ground truth.
double SolveAndCheck(const Eigen::SparseMatrix<double>& A,
                     bool scatter_to_parent, int merge_size = 5,
                     bool left_looking = true) {
  int nv = A.cols();
  Eigen::VectorXd b0 = Eigen::VectorXd::Zero(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b0);
  std::set<int> vs;
  for (const auto& s : slc->row_supports()) vs.insert(s.begin(), s.end());
  std::vector<int> av(vs.begin(), vs.end());
  ConstraintManager cm(nv);
  auto asm_ = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), av);
  cm.AddCustomAssembler(asm_.get());
  SolverConfiguration cfg;
  cfg.tree.max_merge_supernode_size = merge_size;
  cfg.tree.left_looking = left_looking;
  auto solver = MakeTreeSolver(&cm, cfg);
  if (scatter_to_parent) solver->SetScatterToParent(true);
  if (!solver->AssembleAndFactor()) return 1e30;
  VectorXd x_true = VectorXd::Random(nv);
  MatrixXd Ad(A);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);
  VectorXd sol = solver->Solve(rhs);
  return (sol - x_true).norm() / x_true.norm();
}

// Apply random column permutation.
Eigen::SparseMatrix<double> RandomPermute(
    const Eigen::SparseMatrix<double>& A, int seed) {
  int nv = A.cols();
  std::vector<int> perm(nv);
  std::iota(perm.begin(), perm.end(), 0);
  srand(seed);
  std::random_shuffle(perm.begin(), perm.end());
  Eigen::PermutationMatrix<Eigen::Dynamic> P(nv);
  for (int i = 0; i < nv; i++) P.indices()(i) = perm[i];
  return A * P;
}

// Build banded matrix (natural order, no permutation).
Eigen::SparseMatrix<double> Banded(int n, int bw, int rpg) {
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> t;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpg; r++)
      for (int j = 0; j < bw; j++)
        t.emplace_back(g * rpg + r, g + j,
                       0.5 + static_cast<double>(rand()) / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(t.begin(), t.end());
  return A;
}

// Build chain of k cliques (natural order, no permutation).
Eigen::SparseMatrix<double> Chain(int s, int sep, int k, int rpc) {
  int nv = k * s + sep, cs = s + sep, nr = k * rpc;
  std::vector<Eigen::Triplet<double>> t;
  for (int ci = 0; ci < k; ci++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < cs; j++)
        t.emplace_back(ci * rpc + r, ci * s + j,
                       0.5 + static_cast<double>(rand()) / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, nv);
  A.setFromTriplets(t.begin(), t.end());
  return A;
}

// Build star (many spokes sharing a hub).
Eigen::SparseMatrix<double> Star(int hub, int spoke, int ns, int rps) {
  int nv = hub + spoke * ns, nr = rps * ns;
  std::vector<Eigen::Triplet<double>> t;
  for (int s = 0; s < ns; s++) {
    int ss = hub + s * spoke;
    for (int r = 0; r < rps; r++) {
      int row = s * rps + r;
      for (int j = 0; j < hub; j++)
        t.emplace_back(row, j,
                       0.1 * (1.0 + static_cast<double>(rand()) / RAND_MAX));
      for (int j = 0; j < spoke; j++)
        t.emplace_back(row, ss + j,
                       0.5 + static_cast<double>(rand()) / RAND_MAX);
    }
  }
  Eigen::SparseMatrix<double> A(nr, nv);
  A.setFromTriplets(t.begin(), t.end());
  return A;
}

}  // namespace

// Test scatter-to-parent on a variety of patterns with random permutations,
// in both left-looking and right-looking modes.
GTEST_TEST(SparseLeastSquares, ScatterToParent) {
  double tol = 1e-8;

  for (bool left_looking : {true, false}) {
    const char* mode = left_looking ? "LL" : "RL";

    // Block diagonal.
    for (int seed : {10, 20, 30}) {
      srand(seed);
      auto A = RandomPermute(BlockDiagonal({MatrixXd::Random(8, 3),
                                            MatrixXd::Random(6, 4),
                                            MatrixXd::Random(10, 3)}), seed);
      EXPECT_LT(SolveAndCheck(A, false, 5, left_looking), tol)
          << mode << " blkdiag legacy seed=" << seed;
      EXPECT_LT(SolveAndCheck(A, true, 5, left_looking), tol)
          << mode << " blkdiag scatter seed=" << seed;
    }

    // Banded.
    for (int bw : {5, 10, 20}) {
      for (int seed : {42, 77}) {
        srand(seed);
        auto A = RandomPermute(Banded(60, bw, 6), seed);
        EXPECT_LT(SolveAndCheck(A, false, 5, left_looking), tol)
            << mode << " banded bw=" << bw;
        EXPECT_LT(SolveAndCheck(A, true, 5, left_looking), tol)
            << mode << " banded+scat bw=" << bw;
      }
    }

    // Chains, no merging.
    struct ChainCase { int s, sep, k; };
    for (auto [s, sep, k] : std::vector<ChainCase>{
         {1, 5, 20}, {1, 10, 30}, {1, 20, 15},
         {3, 5, 20}, {3, 10, 15},
         {5, 5, 20}, {5, 10, 20}, {5, 20, 10},
         {10, 5, 15}, {10, 10, 15}, {10, 20, 10}}) {
      for (int seed : {42, 99, 137}) {
        srand(seed);
        int rpc = s + sep + s;
        auto A = RandomPermute(Chain(s, sep, k, rpc), seed);
        EXPECT_LT(SolveAndCheck(A, false, 0, left_looking), tol)
            << mode << " chain legacy s=" << s << " sep=" << sep
            << " k=" << k << " seed=" << seed;
        EXPECT_LT(SolveAndCheck(A, true, 0, left_looking), tol)
            << mode << " chain scatter s=" << s << " sep=" << sep
            << " k=" << k << " seed=" << seed;
      }
    }

    // Star graphs.
    for (auto [hub, spoke, ns] : std::vector<std::tuple<int,int,int>>{
         {5, 3, 10}, {10, 5, 10}, {10, 5, 20}, {10, 8, 10}}) {
      for (int seed : {42, 77}) {
        srand(seed);
        int rps = hub + spoke + 5;
        auto A = RandomPermute(Star(hub, spoke, ns, rps), seed);
        EXPECT_LT(SolveAndCheck(A, false, 5, left_looking), tol)
            << mode << " star legacy hub=" << hub << " ns=" << ns;
        EXPECT_LT(SolveAndCheck(A, true, 5, left_looking), tol)
            << mode << " star scatter hub=" << hub << " ns=" << ns;
      }
    }

    // Chains with merging.
    for (int merge : {3, 5, 10}) {
      for (int seed : {42, 77}) {
        srand(seed);
        auto A = RandomPermute(Chain(3, 10, 20, 23), seed);
        EXPECT_LT(SolveAndCheck(A, false, merge, left_looking), tol)
            << mode << " chain+merge legacy merge=" << merge;
        EXPECT_LT(SolveAndCheck(A, true, merge, left_looking), tol)
            << mode << " chain+merge scatter merge=" << merge;
      }
    }
  }
}

// Verify repeated AssembleAndFactor + Solve produces the same answer.
GTEST_TEST(SparseLeastSquares, RepeatedAssembleAndFactor) {
  srand(55);
  int n = 60, bw = 10, rpg = 8;
  int ng = n - bw + 1, nr = rpg * ng;
  std::vector<Eigen::Triplet<double>> triplets;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpg; r++)
      for (int j = 0; j < bw; j++)
        triplets.emplace_back(g * rpg + r, g + j,
                              0.5 + static_cast<double>(rand()) / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(triplets.begin(), triplets.end());

  Eigen::VectorXd b0 = Eigen::VectorXd::Zero(nr);
  auto slc = std::make_unique<SparseLinearConstraint>(A, b0);
  std::set<int> vs;
  for (const auto& s : slc->row_supports()) vs.insert(s.begin(), s.end());
  std::vector<int> av(vs.begin(), vs.end());
  ConstraintManager cm(n);
  auto asm_ = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), av);
  cm.AddCustomAssembler(asm_.get());
  SolverConfiguration cfg;
  auto solver = MakeTreeSolver(&cm, cfg);

  VectorXd x_true = VectorXd::Random(n);
  MatrixXd Ad(A);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);

  // First solve.
  ASSERT_TRUE(solver->AssembleAndFactor());
  VectorXd sol1 = solver->Solve(rhs);
  EXPECT_NEAR((sol1 - x_true).norm(), 0, 1e-8 * x_true.norm());

  // Second solve — must produce identical result.
  ASSERT_TRUE(solver->AssembleAndFactor());
  VectorXd sol2 = solver->Solve(rhs);
  EXPECT_NEAR((sol2 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_EQ((sol1 - sol2).norm(), 0);

  // Third solve — same.
  ASSERT_TRUE(solver->AssembleAndFactor());
  VectorXd sol3 = solver->Solve(rhs);
  EXPECT_NEAR((sol3 - x_true).norm(), 0, 1e-8 * x_true.norm());
  EXPECT_EQ((sol1 - sol3).norm(), 0);
}

}  // namespace conex
