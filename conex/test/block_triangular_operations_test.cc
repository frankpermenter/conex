#include "conex/block_triangular_operations.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using B = BlockTriangularOperations;

namespace {

class Allocator {
 public:
  template <typename T>
  explicit Allocator(T* object) : memory_(SizeOf(*object)) {
    Initialize(object, memory_.data());
  }

 private:
  Eigen::VectorXd memory_;
};

void InitializeToTestValues(TriangularMatrixWorkspace* mat) {
  for (auto& sn : mat->diagonal_blocks()) {
    sn.setConstant(1);
    sn.diagonal().array() += 10;
  }
  for (auto& sn : mat->off_diagonal_blocks()) {
    sn.setRandom();
  }
}

void DoCholeskyTestHelper(const std::vector<Clique>& cliques,
                          const std::vector<int>& tree, bool parallel) {
  CliqueTree clique_tree;
  clique_tree.parent_in_tree.parent = tree;
  clique_tree.cliques = cliques;
  TriangularMatrixWorkspace mat(clique_tree);
  Allocator allocate(&mat);
  InitializeToTestValues(&mat);

  for (auto& sn : mat.diagonal_blocks()) {
    sn.diagonal().setLinSpaced(sn.rows(), 10, 20);
  }

  Eigen::MatrixXd x = mat.MakeDenseMatrix();
  Eigen::LLT<MatrixXd> llt(x);

  MatrixXd L_ref = llt.matrixL();
  EXPECT_TRUE(llt.info() == Eigen::Success);

  if (parallel) {
    B::ParallelBlockCholeskyInPlace(&mat, /* max threads*/ 1);
  } else {
    B::BlockCholeskyInPlace(&mat, false /*use_batch_updates*/);
  }
  MatrixXd L_calc = mat.MakeDenseMatrix();
  MatrixXd error = L_calc - L_ref;
  error = error.triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, 1e-12);
}

void DoCholeskyTest(const std::vector<Clique>& cliques,
                    const std::vector<int> tree = {}) {
  std::srand(1);
  DoCholeskyTestHelper(cliques, tree, true);
  std::srand(1);
  DoCholeskyTestHelper(cliques, tree, false);
}

}  // namespace

/* When the clique tree has multiple leafs,
 * we cannot guarantee variables with the
 * same exiting block column will be grouped
 * contiguously in all branches.
 */
GTEST_TEST(LowerTriMultipleLeafNodes, Cholesky) {
  /*
            3 4 5
         /    |     \
     0 3 4  1 3 5   2 3 5
  */
  DoCholeskyTest({{0, 3, 4}, {1, 3, 4}, {2, 3, 4}, {3, 4, 5}},
                 {3, 3, 3, -1} /*tree*/);

  /*
            5 6 7
              |
            4 5 6
         /    |     \
     0 4 5  2 4 6  3 5 6
              |
            1 4 6
  */
  DoCholeskyTest(
      {{0, 4, 5}, {1, 4, 6}, {2, 4, 6}, {3, 5, 6}, {4, 5, 6}, {5, 6, 7}},
      {4, 2, 4, 4, 5, -1} /*tree*/);
}
GTEST_TEST(LowerTri, Cholesky) {


  DoCholeskyTest({{0, 1}, {2, 3}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});

  // Illustrates we can inject non-zero rows arbitrarily.
  // At second clique, row 3 is inserted in between rows 2 and 4.
  DoCholeskyTest({{0, 1, 2, 4}, {1, 2, 3, 4}, {2, 3, 4}, {3, 4}, {4}});

  DoCholeskyTest({{0, 1, 2, 3, 5}, {3, 4, 5, 6}, {5, 6, 7}});
  // Injects row 2 on top of (3, 4) at some block column 
  DoCholeskyTest({{0, 1, 3, 4}, {1, 2, 3, 4}, {2, 3, 4, 5}, {3, 4, 5}});

  std::vector<int> tree{2, 2, -1};
  DoCholeskyTest({{0, 1, 2, 7, 8, 9}, {3, 4, 5, 6, 8, 9}, {7, 8, 9}}, tree);
  DoCholeskyTest({{0, 1, 2, 3, 5, 6}, {1, 2, 3, 4, 5, 6}, {5, 6, 7}});

  DoCholeskyTest({{0, 1, 2}, {2}});
  DoCholeskyTest({{0, 1, 3}, {1, 2, 3}, {3, 4, 5}});
  DoCholeskyTest({{0, 1, 2}, {1, 2, 3}, {3, 4, 2}});
}

void DoInverseTest(const std::vector<Clique>& cliques,
                   std::vector<int> tree = {}) {
  CliqueTree clique_tree;
  clique_tree.parent_in_tree.parent = tree;
  clique_tree.cliques = cliques;
  TriangularMatrixWorkspace mat(clique_tree);
  Allocator allocate(&mat);
  InitializeToTestValues(&mat);

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseInPlace(mat, &y2);
  EXPECT_NEAR((L * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseTest) {
  DoInverseTest({{0, 1, 2, 3, 4}, {3, 4, 5}});
  DoInverseTest({{0, 1, 2, 3}});
  DoInverseTest({{0, 1, 2, 3}, {3, 4}, {4, 5, 6}});
}

void DoInverseOfTransposeTest(const std::vector<Clique>& cliques,
                              const std::vector<int>& tree = {}) {
  CliqueTree clique_tree;
  clique_tree.parent_in_tree.parent = tree;
  clique_tree.cliques = cliques;
  TriangularMatrixWorkspace mat(clique_tree);
  Allocator allocate(&mat);
  InitializeToTestValues(&mat);

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseOfTransposeInPlace(mat, &y2);
  EXPECT_NEAR((L.transpose() * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseOfTranspose) {
  DoInverseOfTransposeTest({{0, 1, 2, 3}, {3, 4, 5}});
  DoInverseOfTransposeTest({{0, 1, 2, 3}, {3, 4, 5}, {5, 6}});
  DoInverseOfTransposeTest({{0, 1, 2, 3}});
}

void DoLDLTTest(bool diagonal, const std::vector<Clique>& cliques,
                const std::vector<int>& tree = {}) {
  CliqueTree clique_tree;
  clique_tree.parent_in_tree.parent = tree;
  clique_tree.cliques = cliques;
  TriangularMatrixWorkspace mat(clique_tree);
  Allocator allocate(&mat);
  InitializeToTestValues(&mat);

  // Set to identity.
  for (auto& sn : mat.diagonal_blocks()) {
    if (diagonal) {
      sn.setZero();
    }
    // Make indefinite.
    sn.diagonal().setLinSpaced(sn.rows(), -100, 99);
  }

  if (diagonal) {
    for (auto& s : mat.off_diagonal_blocks()) {
      s.setZero();
    }
  }
  Eigen::MatrixXd X = mat.MakeDenseMatrix().selfadjointView<Eigen::Lower>();

  std::vector<Eigen::RLDLT<Eigen::Ref<MatrixXd>>> factorization;
  B::BlockLDLTInPlace(&mat, &factorization);

  Eigen::VectorXd z = Eigen::VectorXd::Random(X.cols());
  z.setConstant(0);
  z(1) = 1;

  Eigen::VectorXd y = X * z;
  // X = M D M ^T z = y
  // z = inv(M^{T}) (MD)^{-1} y
  B::ApplyBlockInverseOfMD(mat, factorization, &y);
  B::ApplyBlockInverseOfMTranspose(mat, factorization, &y);
  EXPECT_NEAR((z - y).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, LDLT) {
  bool diagonal = true;
  DoLDLTTest(diagonal, {{0, 1}});
  DoLDLTTest(diagonal, {{0, 1, 2}, {2}});
  DoLDLTTest(diagonal, {{0, 1, 2, 3, 4}, {3, 4}, {5, 6, 7}});
  DoLDLTTest(diagonal, {{0, 1, 3}, {1, 2, 3}, {3, 4, 5}});
  DoLDLTTest(diagonal, {{0, 1, 2}, {1, 2, 3}, {2, 3, 4}});
  DoLDLTTest(diagonal, {{0, 1}, {2, 3}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});
  diagonal = false;
  DoLDLTTest(diagonal, {{0, 1, 2}, {2}});
  DoLDLTTest(diagonal, {{0, 1, 2, 3, 5}, {3, 4, 5}, {5, 6, 7}});
  DoLDLTTest(diagonal, {{0, 1, 3}, {1, 2, 3}, {3, 4, 5}});
  DoLDLTTest(diagonal, {{0, 1, 2}, {1, 2, 3}, {2, 3, 4}});
  DoLDLTTest(diagonal, {{0, 1}, {2, 3}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});
}

}  // namespace conex
