#include "conex/common/sparse_linear_constraint.h"

#include <iostream>
#include <numeric>
#include <set>

#include "conex/linear_solvers/cholesky_solvers.h"
#include "conex/common/clique_ordering.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/equality_constraint.h"
#include "conex/linear_solvers/kkt_solver_factory.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include "conex/linear_solvers/assembler_adapter.h"
#include "conex/linear_solvers/tree_utils.h"
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

}  // namespace conex
