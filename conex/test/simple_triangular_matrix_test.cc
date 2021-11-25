#include "conex/simple_triangular_matrix.h"
#include "conex/tree_utils.h"
#include "conex/debug_macros.h"
#include "conex/tree_traversal.h"

#include <numeric>

#include "gtest/gtest.h"

#include <Eigen/Dense>
namespace conex {

using Eigen::MatrixXd;
using std::vector;

#if 0


class ParseTreeData {
  struct SimpleTriangularMatrixData {
    std::vector<int> entering_block;
    std::vector<int> exiting_block;
    int num_blocks = 0;
  };

  struct DirectSumTriangularMatrixData {
    vector<int> children;
  };
  // Root = DS
  //  
  //
  //     C
  //  D  D  D
  //  D  D  D
  //  D  D  D 

 public:
  ParseTreeData(RootedTree* tree, std::vector<vector<int>>* cliques) : tree_(tree), 
    cliques_(cliques) {}

  void BuildCompressedTree() {
    std::vector<int> num_children = NumberOfChildren(*tree_);
    std::vector<int> merged_tree_parent = GetRootNodes(*tree_);
    std::stack<int> root_stack;
    for (auto r: merged_tree_parent) {
      root_stack.push(r);
    }
    vector<vector<int>> paths;
    vector<int> path_parents;
    while (root_stack.size() > 0) {
      int root = root_stack.top();
      root_stack.pop();

      vector<int> children = GetChildren(*tree_, root);
      for (auto& c : children) {
        std::vector<int> path; 
        path.push_back(root);
        path.push_back(c);
        auto descendants = GetChildren(*tree_, c);
        while (descendants.size() == 1) {
          path.push_back(descendants.back());
          descendants = GetChildren(*tree_, path.back());
        }
        if (descendants.size() > 1) {
          for (auto s : descendants) {
            root_stack.push(s);
          }
        }
        paths.push_back(path);
        path_parents.push_back(root);
      }
    }
  }

 private:
  RootedTree* tree_;
  vector<vector<int>>* cliques_;
  vector<SimpleTriangularMatrixData> simple_matrices_;
  vector<DirectSumTriangularMatrixData> direct_sum_;
};

namespace {

MatrixXd LowerTri(const Eigen::MatrixXd& x) {
  return x.triangularView<Eigen::Lower>();
}

vector<Eigen::MatrixXd> GetCompressedBlockColumns(
    const MatrixXd& Ref, const std::vector<int> block_sizes) {
  vector<Eigen::MatrixXd> y(block_sizes.size());

  vector<int> offsets(block_sizes.size());
  offsets.at(0) = 0;
  std::partial_sum(block_sizes.begin(), block_sizes.end(), offsets.begin() + 1);

  for (size_t j = 0; j < block_sizes.size(); j++) {
    MatrixXd temp(Ref.rows(), block_sizes.at(j));
    int nnz_rows = 0;
    for (int i = 0; i < Ref.rows(); i++) {
      const auto& segment =
          Ref.row(i).middleCols(offsets.at(j), block_sizes.at(j));
      if (segment.norm() > 0) {
        temp.row(nnz_rows++) = segment;
      }
    }
    y.at(j) = temp.topRows(nnz_rows);
  }
  return y;
}

void DoTest(const std::vector<int>& block_sizes,
            const std::vector<SimpleTriangularMatrixTriplet>& triplets,
            const MatrixXd& Ref) {
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.AssembleFromCompressedColumns(
      GetCompressedBlockColumns(Ref, block_sizes));

  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15);


  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15);
  SimpleTriangularMatrix::LLT llt = mat.llt();
  llt.compute();
  MatrixXd llt_calc = mat.MakeDenseMatrix();
  MatrixXd llt_ref = Eigen::LLT<MatrixXd>(Ref).matrixL();
  MatrixXd error = (llt_calc - llt_ref).triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, 1e-15);
}

}  // namespace
GTEST_TEST(SimpleTri, Construct) {
  std::vector<int> block_sizes{2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 2}};
  MatrixXd Ref(6, 6);
  // clang-format off
  Ref << 9, 1, 0, 0, 0, 0,
         1, 9, 0, 0, 0, 0,
         0, 0, 8, 1, 0, 0,
         0, 0, 1, 8, 0, 0,
         1, 1, 1, 1, 7, 1,
         1, 1, 1, 1, 1, 7;
  // clang-format on
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, ConstructDifferentSizes) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 1}, {2, 1, 1}};
  MatrixXd Ref(7, 7);
  // clang-format off
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());

  DoTest(block_sizes, triplets, Ref);
}
GTEST_TEST(SimpleTri, EmptyTriplets) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{};
  MatrixXd Ref(7, 7);
  // clang-format off
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 1,
         0, 0, 0, 0, 0, 1, 1;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, NonzeroOnAllBlocks) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{
      {1, 0, 2}, {2, 0, 1}, {2, 1, 1}};
  MatrixXd Ref(7, 7);
  // clang-format off
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, OutofOrderRows) {
  std::vector<int> block_sizes{2, 2, 2, 1};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{3, 0, 1}, {2, 1, 1}};
  MatrixXd Ref(7, 7);
  // clang-format off
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 1, 1, 0,
         0, 0, 0, 0, 1, 1, 0,
         1, 1, 1, 1, 1, 1, 1;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, AddTwo) {
  std::vector<int> block_sizes{2, 2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{
      {2, 0, 1}, {3, 0, 1}, {2, 1, 1}, {3, 1, 1}};
  MatrixXd Ref(8, 8);
  // clang-format off
  Ref << 1, 1, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1, 1;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());
  DoTest(block_sizes, triplets, Ref);
}


GTEST_TEST(SimpleTri, DontFactorLastBlock) {
  std::vector<int> block_sizes{2, 2};
  MatrixXd R11(2, 2); 
  MatrixXd R21(2, 2);
  MatrixXd R22(2, 2);
  MatrixXd Ref(4, 4);
  // clang-format off
  R11 << 2, 1, 
         1, 2;
  R21 << .1, 1, 
         1, .1;
  R22 << 2, 1, 
         1, 4;
  Ref <<  R11, R21.transpose() * 0,
          R21, R22; 
  // clang-format on
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {1, 0, 2} };
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.AssembleFromCompressedColumns(
      GetCompressedBlockColumns(Ref, block_sizes));
  auto llt = mat.llt();
  llt.compute(false);
  auto M = mat.MakeDenseMatrix();
  MatrixXd last_block_ref = R22 - R21 * R11.inverse() * R21.transpose();
  MatrixXd last_block_calc = M.bottomRightCorner(block_sizes.back(), block_sizes.back()); 
  MatrixXd error = (last_block_ref - last_block_calc).triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, 1e-12);
}



GTEST_TEST(SimpleTri, DirectSum) {

  // clang-format off
  MatrixXd L1(4, 2);
  MatrixXd R1(2, 2);
  L1 << 20, 0, 
        2, 20, 
        4, 4, 
        4, 4;
  R1 << 20, 2,
        2, 20;
  vector<MatrixXd> cols_1(2); 
  cols_1.at(0) = L1;
  cols_1.at(1) = R1;

  MatrixXd L2(4, 2);
  MatrixXd R2(2, 2);
  L2 << 20, 0, 
       -2, 20, 
        4, 4, 
        0, 4;
  R2 << 22, 2,
        2, 22;
  vector<MatrixXd> cols_2(2); 
  cols_2.at(0) = L2;
  cols_2.at(1) = R2;
  // clang-format on

  std::vector<int> block_sizes{2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{1, 0, 2}};

  vector<SimpleTriangularMatrix> mats;
  mats.emplace_back(block_sizes, triplets);
  mats.emplace_back(block_sizes, triplets);
  mats.at(0).AssembleFromCompressedColumns(cols_1);
  mats.at(1).AssembleFromCompressedColumns(cols_2);

  // clang-format off
  MatrixXd L3(4, 2);
  MatrixXd R3(3, 3);
  L3 << 30, 0, 
       -3, 30, 
        4, 4, 
        0, 4;
  R3 << 33, 3, 3,
        3, 33, 2,
        3, 2, 55;

  vector<MatrixXd> cols_3(2); 
  cols_3.at(0) = L3;
  cols_3.at(1) = R3;
  // clang-format on

  std::vector<int> block_sizes_2{2, 3};
  std::vector<SimpleTriangularMatrixTriplet> triplets_2{{1, 0, 2}};
  mats.emplace_back(block_sizes_2, triplets_2);
  mats.at(2).AssembleFromCompressedColumns(cols_3);

  TriangularMatrixDirectSum mat(mats);
  MatrixXd full_mat = mat.MakeDenseMatrix();

  // full_mat = L1
  //               L2
  //                  L3
  //            L1 L2 L3  R1 + R2 + R3
  // Easy sanity check.
  double squared_norm = full_mat.leftCols(block_sizes.at(0) * 2 + block_sizes_2.at(0)).squaredNorm();
  EXPECT_NEAR(L1.squaredNorm() + L2.squaredNorm() + L3.squaredNorm(), squared_norm, 1e-12);
  double sum = R1.colwise().sum().sum() + R2.colwise().sum().sum() + R3.colwise().sum().sum();
  EXPECT_NEAR(sum, full_mat.bottomRightCorner(block_sizes_2.back(), block_sizes_2.back()).colwise().sum().sum(), 1e-12);

  auto llt = mat.llt();
  llt.compute();
  MatrixXd llt_ref = Eigen::LLT<MatrixXd>(full_mat).matrixL();
  EXPECT_NEAR((llt.matrixL() - llt_ref).norm(), 0, 1e-12);
}

GTEST_TEST(SimpleTri, IncrementSubmatrix) {
  // **   
  // **  
  // ** **
  // ** **
  // ** ** ***
  // ** ** ***
  //       ***
  std::vector<int> block_sizes{2, 2, 3};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{1, 0, 2},  {2, 0, 3}};

  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(0);

  // x = x11 0
  //      0  0
  //     x12 0  x22
  std::vector<std::pair<int, int>> submatrix_partition{{0, 1},   {2, 3}};
  MatrixXd submatrix(4, 4);

  // clang-format off
  submatrix << 1, 0, 0, 0,
               1, 2, 0, 0,
               1, 3, 4, 4,
               3, 4, 4, 5;
  // clang-format on
  mat.IncrementSubmatrix(submatrix, submatrix_partition);
  MatrixXd X_calc = mat.MakeDenseMatrix(); 
  MatrixXd X_ref(7, 7); 
  // clang-format off
  X_ref << 1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 
           1, 0, 0, 0, 2, 0, 0, 
           1, 0, 0, 0, 3, 4, 4, 
           3, 0, 0, 0, 4, 4, 5;
  // clang-format on
  EXPECT_NEAR((X_ref - mat.MakeDenseMatrix()).norm(), 0, 1e-15); 

  submatrix_partition.at(0) = std::pair<int, int>(1, 1);
  mat.IncrementSubmatrix(submatrix, submatrix_partition);
  // clang-format off
  X_ref << 1, 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0, 0, 
           1, 0, 1, 0, 4, 0, 0, 
           1, 0, 1, 0, 6, 8, 8, 
           3, 0, 3, 0, 8, 8, 10;
  // clang-format on
  EXPECT_NEAR((X_ref - mat.MakeDenseMatrix()).norm(), 0, 1e-15); 

  submatrix_partition.at(0) = std::pair<int, int>(0, 2);
  submatrix_partition.at(1) = std::pair<int, int>(1, 2);
  mat.IncrementSubmatrix(submatrix, submatrix_partition);
  // clang-format off
  X_ref << 2, 0, 0, 0, 0, 0, 0,
           1, 2, 0, 0, 0, 0, 0,
           1, 3, 5, 0, 0, 0, 0,
           3, 4, 4, 5, 0, 0, 0, 
           1, 0, 1, 0, 4, 0, 0, 
           1, 0, 1, 0, 6, 8, 8, 
           3, 0, 3, 0, 8, 8, 10;
  // clang-format on
  EXPECT_NEAR(LowerTri(X_ref - mat.MakeDenseMatrix()).norm(), 0, 1e-15); 
}
#endif 


GTEST_TEST(LowerTri, AssembleFromCliques) {
  vector<vector<int>> cliques{ {0, 2, 4},  {1,  4}, {3, 2, 4} };
  // Sort by earliest exiting clique, break ties with entering.
  std::vector<int> enter(5, -1);
  std::vector<int> exit(5, -1);
  std::vector<int> block_sizes(3);
  std::vector<int> permutation(5); std::iota(permutation.begin(), permutation.end(), 0);
  for (size_t i = 0; i < cliques.size(); i++) {
    for (auto n : cliques.at(i)) {
      if (enter.at(n) == -1) {
        enter.at(n) = i;
        exit.at(n) = i;
      } else {
        exit.at(n) = i;
      }
    }
  }

  for (auto i : exit) {
    block_sizes.at(i)++; 
  }
  
  std::vector<SimpleTriangularMatrixTriplet> triplets;
  for (size_t i = 0; i < 5; i++  ) {
    if (enter.at(i) != exit.at(i)) {
      triplets.push_back({exit.at(i), enter.at(i), 1});
    }
  }

  MatrixXd M(cliques.size(), cliques.size()); M.setZero();

  std::sort(permutation.begin(), permutation.end(), 
            [enter, exit](const int& i, const int& j) { 
            return (exit[i] < exit[j]) || (exit[i] == exit[j] && enter[i] < enter[j]);
            });

  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1);
  DUMP(mat.MakeDenseMatrix());
}

} // namespace conex
