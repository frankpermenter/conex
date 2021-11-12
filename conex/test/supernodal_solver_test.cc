#include "conex/supernodal_solver.h"
#include "conex/block_triangular_operations.h"
#include "conex/debug_macros.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

std::vector<int> ResidualSize(std::vector<Clique>& path) {
  std::vector<int> y;
  for (size_t j = 0; j < path.size() - 1; j++) {
    std::vector<int> temp;
    IntersectionOfSorted(path.at(j), path.at(j + 1), &temp);
    y.push_back(path.at(j).size() - temp.size());
  }
  y.push_back(path.back().size());
  return y;
}

void RunningIntersectionClosure(std::vector<Clique>* path) {
  if (path->size() < 2) {
    return;
  }
  int n = path->size();
  for (int i = 0; i < n - 2; i++) {
    for (int j = n - 1; j > i + 1; j--) {
      std::vector<int> temp;
      IntersectionOfSorted(path->at(i), path->at(j), &temp);
      if (temp.size() == 0) {
        continue;
      }
      for (int k = j - 1; k > i; k--) {
        path->at(k) = UnionOfSorted(path->at(k), temp);
      }
    }
  }
}

SparseTriangularMatrix MakeSparseTriangularMatrix(
    int N, const std::vector<Clique>& path_) {
  auto path = path_;
  Sort(&path);
  RunningIntersectionClosure(&path);
  auto supernode_size = ResidualSize(path);
  return SparseTriangularMatrix(N, path, supernode_size);
}

SparseTriangularMatrix GetFillInPattern(
    int N, const std::vector<Clique>& cliques_input) {
  auto mat = MakeSparseTriangularMatrix(N, cliques_input);

  for (int j = static_cast<int>(mat.cliques().size()) - 1; j >= 0; j--) {
    // Initialize columns of super nodes.
    mat.supernodes().at(j).setConstant(1);
    mat.separator().at(j).setConstant(1);
  }
  return mat;
}

using Eigen::MatrixXd;
using std::vector;

int GetMax(const vector<Clique>& cliques) {
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

MatrixXd GetMatrix(int N, const vector<Clique>& c) {
  MatrixXd M(N, N);
  M.setZero();
  for (unsigned int k = 0; k < c.size(); k++) {
    int i = 0;
    for (auto ci : c.at(k)) {
      int j = 0;
      for (auto cj : c.at(k)) {
        M(ci, cj) = 1;
        j++;
      }
      i++;
    }
  }
  return M;
}

MatrixXd GetMatrix(int N, const vector<Clique>& c, double val) {
  MatrixXd M(N, N);
  M.setZero();
  for (unsigned int k = 0; k < c.size(); k++) {
    int i = 0;
    for (auto ci : c.at(k)) {
      int j = 0;
      for (auto cj : c.at(k)) {
        M(ci, cj) = val;
        j++;
      }
      i++;
    }
  }
  return M;
}

bool DoPatternTest(const vector<Clique>& cliques) {
  int N = GetMax(cliques) + 1;
  MatrixXd error =
      GetMatrix(N, cliques) - (GetFillInPattern(N, cliques)).MakeDenseMatrix();
  error = error.triangularView<Eigen::Lower>();
  return error.norm() == 0;
}

GTEST_TEST(Basic, Basic) {
  vector<Clique> cliques1{{0, 1, 5}, {1, 2, 5}, {3, 4, 5}};

  EXPECT_TRUE(DoPatternTest(cliques1));

  vector<Clique> cliques2{{0, 1, 2}};
  EXPECT_TRUE(DoPatternTest(cliques2));

  EXPECT_TRUE(DoPatternTest({{0, 1, 2, 4}, {3, 4}, {5, 6, 7}}));
}

GTEST_TEST(GetPattern, Basic) {
  vector<Clique> cliques{{0, 1, 2, 5}, {1, 4, 2, 5}, {3, 4, 5}};
}

GTEST_TEST(LowerTri, Constant) {
  vector<Clique> cliques{{0, 1, 5}, {1, 2, 5}, {3, 4, 5}};

  auto mat = MakeSparseTriangularMatrix(GetMax(cliques) + 1, cliques);
  mat.SetConstant(-1);
  auto y = mat.MakeDenseMatrix();
  auto yref = GetMatrix(GetMax(cliques) + 1, cliques, -1);
  MatrixXd error = y - yref;
  error = error.triangularView<Eigen::Lower>();
  EXPECT_TRUE(error.norm() == 0);
}

}  // namespace conex
