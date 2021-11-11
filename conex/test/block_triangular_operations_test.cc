#include "conex/block_triangular_operations.h"
#include "conex/supernodal_solver.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using B = BlockTriangularOperations;

void RunningIntersectionClosure(std::vector<Clique>* cliques) {
  if (cliques->size() < 2) {
    return;
  }
  int n = cliques->size();
  for (int i = 0; i < n - 2; i++) {
    for (int j = n - 1; j > i + 1; j--) {
      std::vector<int> temp;
      IntersectionOfSorted(cliques->at(i), cliques->at(j), &temp);
      if (temp.size() == 0) {
        continue;
      }
      for (int k = j - 1; k > i; k--) {
        cliques->at(k) = UnionOfSorted(cliques->at(k), temp);
      }
    }
  }
}

std::vector<int> ResidualSize(std::vector<Clique>& cliques) {
  std::vector<int> y;
  for (size_t j = 0; j < cliques.size() - 1; j++) {
    std::vector<int> temp;
    IntersectionOfSorted(cliques.at(j), cliques.at(j + 1), &temp);
    y.push_back(cliques.at(j).size() - temp.size());
  }
  y.push_back(cliques.back().size());
  return y;
}

SparseTriangularMatrix MakeSparseTriangularMatrix(
    int N, const std::vector<Clique>& cliques_) {
  auto cliques = cliques_;
  Sort(&cliques);
  RunningIntersectionClosure(&cliques);
  auto supernode_size = ResidualSize(cliques);
  return SparseTriangularMatrix(N, cliques, supernode_size);
}

SparseTriangularMatrix GetFillInPattern(
    int N, const std::vector<Clique>& cliques_input) {
  auto mat = MakeSparseTriangularMatrix(N, cliques_input);

  for (int j = static_cast<int>(mat.cliques_.size()) - 1; j >= 0; j--) {
    // Initialize columns of super nodes.
    mat.supernodes.at(j).setConstant(1);
    mat.separator.at(j).setConstant(1);

    // Update other columns: the (seperator, seperator) components.
    int index = 0;
    auto s_s = mat.workspace_.seperator_diagonal.at(j);
    int n = mat.cliques_.at(j).size();
    for (int i = mat.supernode_size.at(j); i < n; i++) {
      for (int k = i; k < n; k++) {
        *s_s.at(index++) += 1;
      }
    }
  }
  return mat;
}

SparseTriangularMatrix RandomSparseMatrix(
    int N, const std::vector<Clique>& cliques_input) {
  auto mat = MakeSparseTriangularMatrix(N, cliques_input);

  for (int j = static_cast<int>(mat.cliques_.size()) - 1; j >= 0; j--) {
    // Initialize columns of super nodes.
    int r = mat.supernodes.at(j).rows();
    int c = mat.supernodes.at(j).cols();
    mat.supernodes.at(j) = MatrixXd::Random(r, c);
    r = mat.separator.at(j).rows();
    c = mat.separator.at(j).cols();
    mat.separator.at(j) = MatrixXd::Random(r, c);
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

void DoCholeskyTest(const std::vector<Clique>& cliques) {
  auto mat = GetFillInPattern(GetMax(cliques) + 1, cliques);
  for (auto& sn : mat.supernodes) {
    sn.diagonal().array() += 100;
  }

  Eigen::MatrixXd x = mat.MakeDenseMatrix();
  Eigen::LLT<MatrixXd> llt(x);
  MatrixXd L = llt.matrixL();
  EXPECT_TRUE(llt.info() == Eigen::Success);

  B::BlockCholeskyInPlace(&mat.workspace_);
  MatrixXd error = mat.MakeDenseMatrix() - L;
  error = error.triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, Cholesky) {
  DoCholeskyTest({{0, 1, 2}, {2}});
  DoCholeskyTest({{0, 1, 2, 4, 7}, {3, 4}, {5, 6, 7}});
  DoCholeskyTest({{0, 1, 5}, {1, 2, 5}, {3, 4, 5}});
  DoCholeskyTest({{0, 1, 2}, {1, 2, 3}, {3, 4, 2}});
  DoCholeskyTest({{0, 1}, {2, 4}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});
}

void DoInverseTest(const std::vector<Clique>& cliques) {
  auto mat = GetFillInPattern(GetMax(cliques) + 1, cliques);
  for (auto& sn : mat.supernodes) {
    sn.diagonal().array() += 10;
  }

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseInPlace(mat.workspace_, &y2);
  EXPECT_NEAR((L * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseTest) {
  DoInverseTest({{0, 1, 2, 3}, {3, 4, 5}});
  DoInverseTest({{0, 1, 2, 3}});
  DoInverseTest({{0, 1, 2, 3}, {3, 4}, {4, 5, 6}});
}

void DoInverseOfTransposeTest(const std::vector<Clique>& cliques) {
  auto mat = GetFillInPattern(GetMax(cliques) + 1, cliques);
  for (auto& sn : mat.supernodes) {
    sn.diagonal().array() += 10;
  }

  Eigen::MatrixXd L = mat.MakeDenseMatrix().triangularView<Eigen::Lower>();
  Eigen::VectorXd b;
  b.setLinSpaced(L.rows(), -1, 1);

  Eigen::VectorXd y2 = b;
  B::ApplyBlockInverseOfTransposeInPlace(mat.workspace_, &y2);
  EXPECT_NEAR((L.transpose() * y2 - b).norm(), 0, 1e-12);
}

GTEST_TEST(LowerTri, InverseOfTranspose) {
  DoInverseOfTransposeTest({{0, 1, 2, 5}, {3, 4, 5}});
  DoInverseOfTransposeTest({{0, 1, 2, 5}, {3, 4, 5}, {5, 6}});
  DoInverseOfTransposeTest({{0, 1, 2, 3}});
}

MatrixXd Submatrix(const MatrixXd& T, const Clique& c) {
  MatrixXd y(c.size(), c.size());
  int i = 0;
  for (auto ci : c) {
    int j = 0;
    for (auto cj : c) {
      y(i, j) = T(ci, cj);
      j++;
    }
    i++;
  }
  return y;
}

void DoLDLTTest(bool diagonal, const std::vector<Clique>& cliques) {
  auto mat = GetFillInPattern(GetMax(cliques) + 1, cliques);

  // Set to identity.
  for (auto& sn : mat.workspace_.diagonal) {
    if (diagonal) {
      sn.setZero();
    }
    int n = sn.diagonal().size();
    for (int i = 0; i < n; i++) {
      sn.diagonal()(i) = -101 + i * 100;
    }
  }

  if (diagonal) {
    for (auto& sn : mat.workspace_.off_diagonal) {
      sn.setZero();
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
  DoLDLTTest(diagonal, {{0, 1, 2, 4, 7}, {3, 4}, {5, 6, 7}});
  DoLDLTTest(diagonal, {{0, 1, 5}, {1, 2, 5}, {3, 4, 5}});
  DoLDLTTest(diagonal, {{0, 1, 2}, {1, 2, 3}, {3, 4, 2}});
  DoLDLTTest(diagonal, {{0, 1}, {2, 4}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});

  diagonal = false;
  DoLDLTTest(diagonal, {{0, 1, 2}, {2}});
  DoLDLTTest(diagonal, {{0, 1, 2, 4, 7}, {3, 4}, {5, 6, 7}});
  DoLDLTTest(diagonal, {{0, 1, 5}, {1, 2, 5}, {3, 4, 5}});
  DoLDLTTest(diagonal, {{0, 1, 2}, {1, 2, 3}, {3, 4, 2}});
  DoLDLTTest(diagonal, {{0, 1}, {2, 4}, {3, 4}, {5, 6, 7}, {7, 8, 9, 10}});
}

}  // namespace conex
