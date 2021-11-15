#include "conex/block_triangular_operations.h"
#include "conex/clique_ordering.h"
#include "conex/supernodal_solver.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using B = BlockTriangularOperations;

namespace {

std::vector<int> SupernodeSize(std::vector<Clique>& cliques,
                               const std::vector<int>& tree_in = {}) {
  std::vector<int> y;
  std::vector<int> tree = tree_in;

  if (tree.size() == 0) {
    tree.resize(cliques.size());
    std::iota(tree.begin(), tree.end() - 1, 1);
    tree.back() = -1;
  }

  for (size_t j = 0; j < cliques.size(); j++) {
    std::vector<int> temp;
    if (tree.at(j) >= 0) {
      IntersectionOfSorted(cliques.at(j), cliques.at(tree.at(j)), &temp);
      y.push_back(cliques.at(j).size() - temp.size());
    } else {
      y.push_back(cliques.at(j).size());
    }
  }
  return y;
}

SparseTriangularMatrix MakeSparseTriangularMatrix(
    int N, const std::vector<Clique>& cliques_,
    const std::vector<int>& tree = {}) {
  auto cliques = cliques_;
  Sort(&cliques);
  auto supernode_size = SupernodeSize(cliques, tree);
  auto mat = SparseTriangularMatrix(N, cliques, supernode_size);

  mat.SetConstant(1);
  for (auto& sn : mat.supernodes()) {
    sn.diagonal().array() += 10;
  }
  for (auto& sn : mat.separator()) {
    sn.setRandom();
  }

  return mat;
}

int GetMax(const std::vector<Clique>& cliques) {
  int max = cliques.at(0).at(0);
  for (const auto& c : cliques) {
    for (const auto ci : c) {
      if (ci > max) {
        max = ci;
      }
    }
  }
  return max;
}

void DoCholeskyTestHelper(const std::vector<Clique>& cliques,
                          const std::vector<int> tree, bool use_batch_updates) {
  auto mat = MakeSparseTriangularMatrix(GetMax(cliques) + 1, cliques, tree);

  Eigen::MatrixXd x = mat.MakeDenseMatrix();
  Eigen::LLT<MatrixXd> llt(x);
  MatrixXd L = llt.matrixL();
  EXPECT_TRUE(llt.info() == Eigen::Success);

  B::BlockCholeskyInPlace(&mat.workspace_, use_batch_updates);
  MatrixXd error = mat.MakeDenseMatrix() - L;
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
     /     |     \
   0 3 4  1 3 5   2 3 5
*/
  //DoCholeskyTest({{0, 3, 4}, {1, 3, 5}, {2, 3, 5}, {3, 4, 5}},    {3, 3, 3, -1} /*tree*/      );
  DoCholeskyTest({{0, 3, 4}, {1, 3, 4}, {2, 3, 4}, {3, 4, 5}},    {3, 3, 3, -1} /*tree*/      );

  
/*
         4 5 6
     /     |    \
   0 4 5  2 4 6   3 5 6
           |          
          1 4 6
*/
  DoCholeskyTest({{0, 4, 5}, {1, 4, 6}, {2, 4, 6}, {3, 5, 6},  {4, 5, 6} },    {4, 2, 4, 4, -1} /*tree*/      );
  return;
}
GTEST_TEST(LowerTri, Cholesky) {
  // Illustrates we can inject non-zero rows arbitrarily.
  // At second clique, row 3 is inserted in between rows 2 and 4.
  DoCholeskyTest({{0, 1, 2, 4}, {1, 2, 3, 4}, {2, 3, 4}, {3, 4}, {4}});

  DoCholeskyTest({{0, 1, 2, 3, 5}, {3, 4, 5, 6}, {5, 6, 7}});
  // Injects row 2 on top of (3, 4) at some block column onto
  DoCholeskyTest({{0, 1, 3, 4}, {1, 2, 3, 4}, {2, 3, 4, 5}, {3, 4, 5}});

  std::vector<int> tree{2, 2, -1};
  DoCholeskyTest({{0, 1, 2, 7, 8, 9}, {3, 4, 5, 6, 8, 9}, {7, 8, 9}}, tree);
  DoCholeskyTest({{0, 1, 2, 3, 5, 6}, {1, 2, 3, 4, 5, 6}, {5, 6, 7}});

  DoCholeskyTest({{0, 1, 2}, {2}});
  DoCholeskyTest({{0, 1, 3}, {1, 2, 3}, {3, 4, 5}});
  DoCholeskyTest({{0, 1, 2}, {1, 2, 3}, {3, 4, 2}});
  DoCholeskyTest({{0, 1}, {2, 3}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});

  // The batch update should cause exception since
  // entering_col(5) < entering_col(4).
  EXPECT_THROW(DoCholeskyTest({{0, 1, 2, 3, 5}, {1, 2, 3, 4, 5, 6}}),
               std::runtime_error);
}

void DoInverseTest(const std::vector<Clique>& cliques) {
  auto mat = MakeSparseTriangularMatrix(GetMax(cliques) + 1, cliques);

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseInPlace(mat.workspace_, &y2);
  EXPECT_NEAR((L * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseTest) {
  DoInverseTest({{0, 1, 2, 3, 4}, {3, 4, 5}});
  DoInverseTest({{0, 1, 2, 3}});
  DoInverseTest({{0, 1, 2, 3}, {3, 4}, {4, 5, 6}});
}

void DoInverseOfTransposeTest(const std::vector<Clique>& cliques) {
  auto mat = MakeSparseTriangularMatrix(GetMax(cliques) + 1, cliques);

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseOfTransposeInPlace(mat.workspace_, &y2);
  EXPECT_NEAR((L.transpose() * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseOfTranspose) {
  DoInverseOfTransposeTest({{0, 1, 2, 3}, {3, 4, 5}});
  DoInverseOfTransposeTest({{0, 1, 2, 3}, {3, 4, 5}, {5, 6}});
  DoInverseOfTransposeTest({{0, 1, 2, 3}});
}

void DoLDLTTest(bool diagonal, const std::vector<Clique>& cliques) {
  auto mat = MakeSparseTriangularMatrix(GetMax(cliques) + 1, cliques);

  // Set to identity.
  for (auto& sn : mat.supernodes()) {
    if (diagonal) {
      sn.setZero();
    }
    // Make indefinite.
    sn.diagonal().setLinSpaced(sn.rows(), -100, 99);
  }

  if (diagonal) {
    for (auto& s : mat.separator()) {
      s.setZero();
    }
  }
  Eigen::MatrixXd X = mat.MakeDenseMatrix().selfadjointView<Eigen::Lower>();

  std::vector<Eigen::RLDLT<Eigen::Ref<MatrixXd>>> factorization;
  B::BlockLDLTInPlace(&mat.workspace_, &factorization);

  Eigen::VectorXd z = Eigen::VectorXd::Random(X.cols());
  z.setConstant(0);
  z(1) = 1;

  Eigen::VectorXd y = X * z;
  // X = M D M ^T z = y
  // z = inv(M^{T}) (MD)^{-1} y
  B::ApplyBlockInverseOfMD(mat.workspace_, factorization, &y);
  B::ApplyBlockInverseOfMTranspose(mat.workspace_, factorization, &y);
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
