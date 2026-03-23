#include "conex/sparse_linear_constraint.h"

#include <iostream>
#include <set>

#include "conex/clique_ordering.h"
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

GTEST_TEST(SparseLinearConstraintAssembler, ThrowsOnNonTreeSolver) {
  std::vector<MatrixXd> blocks = {MatrixXd::Random(4, 2),
                                  MatrixXd::Random(3, 3)};
  auto A = BlockDiagonal(blocks);
  VectorXd b = VectorXd::Ones(A.rows());
  auto slc = std::make_unique<SparseLinearConstraint>(A, b);

  std::set<int> var_set;
  for (const auto& support : slc->row_supports()) {
    var_set.insert(support.begin(), support.end());
  }
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);

  EXPECT_THROW(assembler->SetDenseData(), std::runtime_error);
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

}  // namespace conex
