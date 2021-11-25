#include "conex/simple_triangular_matrix.h"
#include "conex/debug_macros.h"
#include "conex/tree_traversal.h"
#include "conex/tree_utils.h"

#include <numeric>

#include "gtest/gtest.h"

#include <Eigen/Dense>
namespace conex {

using Eigen::MatrixXd;
using std::vector;

MatrixXd LowerTri(const Eigen::MatrixXd& x) {
  return x.triangularView<Eigen::Lower>();
}
MatrixXd SparsityPattern(const Eigen::MatrixXd& x) {
  MatrixXd y = x;
  y.setZero();
  for (int i = 0; i < y.rows(); i++) {
    for (int j = 0; j < y.cols(); j++) {
      if (std::fabs(x(i, j)) > 1e-18) {
        y(i, j) = 1;
      }
    }
  }
  return y;
}

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

vector<int> PermuteClique(const std::vector<int> clique,
                          const std::vector<int> permutation) {

  vector<int> variable_to_elimination_position(permutation.size());
  for (size_t i = 0; i < permutation.size(); i++) {
    variable_to_elimination_position.at(permutation.at(i)) = i;
  }

  vector<int> y(clique.size());
  for (size_t i = 0; i < clique.size(); i++) {
    y.at(i) = variable_to_elimination_position.at(clique.at(i));
  }
  std::sort(y.begin(), y.end());
  return y;
}
vector<Eigen::MatrixXd> GetCompressedBlockColumns(
    const MatrixXd& M, const std::vector<int> permutation,
    const vector<std::vector<int>> clique, const std::vector<int> block_sizes) {
  int num_vars = permutation.size();
  Eigen::PermutationMatrix<-1> P(num_vars);
  P.indices() = Eigen::Map<const Eigen::VectorXi>(permutation.data(), num_vars);
  MatrixXd data = P.transpose() * M * P;
  vector<MatrixXd> columns(clique.size());
  int offset = 0;
  for (size_t i = 0; i < clique.size(); i++) {
    vector<int> c = PermuteClique(clique.at(i), permutation);
    columns.at(i).resize(c.size(), block_sizes.at(i));
    int r = 0; 
    for (auto row : c) {
      columns.at(i).row(r) =
          data.block(row, offset, 1, block_sizes.at(i));
      r++;
    }
    offset += block_sizes.at(i);
  }
  return columns;
}

GTEST_TEST(LowerTri, AssembleFromCliques) {
  vector<vector<int>> cliques{{0, 1, 2, 3, 4, 5, 18, 19, 20, 21},
                              {0, 1, 2, 3, 4, 5, 14, 15, 16, 17},
                              {0, 1, 2, 3, 4, 5, 10, 11, 12, 13},
                              {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}};
  int num_vars = 22;
  int num_cliques = cliques.size();

  MatrixXd M(num_vars, num_vars);
  M << 0.0134083, 0.000148945, 0.000261851, 0, -0.0608643, 0.0115304,
      7.4517e-08, 3.171e-09, 3.171e-09, 2.1e-09, 9.84639e-06, -1.06274e-07,
      0.000589091, 0.000162831, 7.4517e-08, 3.171e-09, 3.171e-09, 2.1e-09,
      7.4517e-08, 3.171e-09, 3.171e-09, 2.1e-09, 0.000148945, 0.0111433,
      -0.000170184, 0.0608643, 0, 0.00171292, 1.8146e-06, 0.000922838,
      0.000247331, 1.17739e-05, 0.000110413, 2.45966e-05, 4.68547e-05,
      9.31172e-06, 7.2088e-07, 0.000934513, 0.000250184, 1.182e-05,
      -3.78324e-07, 0.000922706, 0.000247327, 1.17762e-05, 0.000261851,
      -0.000170184, 0.00261106, -0.0115304, -0.00171292, 0, 1.24791e-05,
      -0.00023386, -5.7423e-05, -1.04314e-06, -0.00123567, -1.91943e-06,
      -4.09842e-06, -8.1467e-07, 1.259e-05, 7.5795e-07, 2.575e-08, -1.3e-08,
      1.26048e-05, 0.00023537, 5.74743e-05, 1.01724e-06, 0, 0.0608643,
      -0.0115304, 0.9549, 0, 0, 6.12132e-09, 0.00435634, 0.00101376,
      -7.32747e-19, 0.00943465, 5.26474e-08, 1.42834e-19, 2.84442e-20, 0,
      0.00435636, 0.00101376, 0, -6.12132e-09, 0.00435634, 0.00101376,
      -7.32747e-19, -0.0608643, 0, -0.00171292, 0, 0.9549, 0, 0, 0, 0, 0,
      -0.000525492, 6.22843e-11, 0.000311023, 6.19348e-05, 0, 0, 0, 0, 0, 0, 0,
      0, 0.0115304, 0.00171292, 0, 0, 0, 0.9549, 0, 0, 0, 0, 4.59747e-05,
      7.1191e-10, 0.00355486, 0.000707919, 0, 0, 0, 0, 0, 0, 0, 0, 7.4517e-08,
      1.8146e-06, 1.24791e-05, 6.12132e-09, 0, 0, 1.259e-05, 7.56439e-07,
      2.53057e-08, -1.30029e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.171e-09,
      0.000922838, -0.00023386, 0.00435634, 0, 0, 7.56439e-07, 0.000446162,
      0.000136541, 1.182e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3.171e-09,
      0.000247331, -5.7423e-05, 0.00101376, 0, 0, 2.53057e-08, 0.000136541,
      8.17987e-05, 1.182e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.1e-09,
      1.17739e-05, -1.04314e-06, -7.32747e-19, 0, 0, -1.30029e-08, 1.182e-05,
      1.182e-05, 1.182e-05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9.84639e-06,
      0.000110413, -0.00123567, 0.00943465, -0.000525492, 4.59747e-05, 0, 0, 0,
      0, 0.00108955, -2.32398e-07, -1.05708e-09, 2.88035e-12, 0, 0, 0, 0, 0, 0,
      0, 0, -1.06274e-07, 2.45966e-05, -1.91943e-06, 5.26474e-08, 6.22843e-11,
      7.1191e-10, 0, 0, 0, 0, -2.32398e-07, 2.4669e-05, -6.98759e-08,
      -3.29445e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0.000589091, 4.68547e-05,
      -4.09842e-06, 1.42834e-19, 0.000311023, 0.00355486, 0, 0, 0, 0,
      -1.05708e-09, -6.98759e-08, 0.000330065, 0.000111248, 0, 0, 0, 0, 0, 0, 0,
      0, 0.000162831, 9.31172e-06, -8.1467e-07, 2.84442e-20, 6.19348e-05,
      0.000707919, 0, 0, 0, 0, 2.88035e-12, -3.29445e-08, 0.000111248,
      7.47225e-05, 0, 0, 0, 0, 0, 0, 0, 0, 7.4517e-08, 7.2088e-07, 1.259e-05, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.259e-05, 7.5795e-07, 2.575e-08, -1.3e-08,
      0, 0, 0, 0, 3.171e-09, 0.000934513, 7.5795e-07, 0.00435636, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 7.5795e-07, 0.000446165, 0.000136541, 1.182e-05, 0, 0, 0,
      0, 3.171e-09, 0.000250184, 2.575e-08, 0.00101376, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 2.575e-08, 0.000136541, 8.17984e-05, 1.182e-05, 0, 0, 0, 0, 2.1e-09,
      1.182e-05, -1.3e-08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.3e-08, 1.182e-05,
      1.182e-05, 1.182e-05, 0, 0, 0, 0, 7.4517e-08, -3.78324e-07, 1.26048e-05,
      -6.12132e-09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.259e-05,
      7.59461e-07, 2.61943e-08, -1.29971e-08, 3.171e-09, 0.000922706,
      0.00023537, 0.00435634, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      7.59461e-07, 0.000446162, 0.000136541, 1.182e-05, 3.171e-09, 0.000247327,
      5.74743e-05, 0.00101376, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      2.61943e-08, 0.000136541, 8.17987e-05, 1.182e-05, 2.1e-09, 1.17762e-05,
      1.01724e-06, -7.32747e-19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -1.29971e-08, 1.182e-05, 1.182e-05, 1.182e-05;
  // Sort by earliest exiting clique, break ties with entering.
  std::vector<int> enter(num_vars, -1);
  std::vector<int> exit(num_vars, -1);
  std::vector<int> block_sizes(num_cliques);
  std::vector<int> permutation(num_vars);
  std::iota(permutation.begin(), permutation.end(), 0);
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
  for (int i = 0; i < num_vars; i++) {
    if (enter.at(i) != exit.at(i)) {
      triplets.push_back({exit.at(i), enter.at(i), 1});
    }
  }

  std::sort(permutation.begin(), permutation.end(),
            [enter, exit](const int& i, const int& j) {
              return (exit[i] < exit[j]) ||
                     (exit[i] == exit[j] && enter[i] < enter[j]);
            });

  Eigen::PermutationMatrix<-1> P(num_vars);
  P.indices() = Eigen::Map<Eigen::VectorXi>(permutation.data(), num_vars);
  SimpleTriangularMatrix mat(block_sizes, triplets);

  Eigen::LLT<MatrixXd> llt_ref((P.transpose()*M * P));
  mat.AssembleFromDenseMatrix(P.transpose()*M * P);
  auto llt = mat.llt(); llt.compute();
  MatrixXd L_ref = llt_ref.matrixL();
  MatrixXd error = LowerTri(llt.matrixL() - L_ref);

  BlockSparseSymmetricMatrix b(cliques.size(), enter, exit);
  b.SetFromDenseMatrix(M);
  auto llt_2 = b.llt(); llt_2.compute();
  MatrixXd error_2 = LowerTri(llt_2.matrixL() - L_ref);
  EXPECT_NEAR(error_2.norm(), 0, 1e-14);
  EXPECT_NEAR(error.norm(), 0, 1e-14);


}

}  // namespace conex
