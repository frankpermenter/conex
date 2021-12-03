#define CONEX_ENABLE_TIMER 1
#include "conex/simple_triangular_matrix.h"
#include "conex/debug_macros.h"
#include "conex/tree_traversal.h"
#include "conex/tree_utils.h"

#include <numeric>

#include "gtest/gtest.h"

#include <Eigen/Dense>
namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


namespace {

MatrixXd LowerTri(const Eigen::MatrixXd& x) {
  return x.triangularView<Eigen::Lower>();
}

void DoBlockCholeskyTest(const Eigen::MatrixXd& M,
                         const vector<std::vector<int>>& cliques) {
  auto b = MakeBlockSparseMatrix(M, cliques);
  auto llt_calc = b.llt(); llt_calc.compute();

  Eigen::LLT<MatrixXd> llt_ref((llt_calc.matrixP().transpose()*M * llt_calc.matrixP()));
  MatrixXd L_ref = llt_ref.matrixL();

  MatrixXd error = LowerTri(llt_calc.matrixL() - L_ref);
  EXPECT_NEAR(error.norm(), 0, 1e-14);

  VectorXd x;
  x.setLinSpaced(L_ref.cols(), -1, 1.1);
  VectorXd Lx = L_ref * x;
  llt_calc.ApplyInverseOfL(&Lx);
  EXPECT_NEAR( (Lx - x).norm(), 0, 1e-12);

  VectorXd Ltx = L_ref.transpose() * x;
  llt_calc.ApplyInverseOfLt(&Ltx);
  EXPECT_NEAR((Ltx - x).norm(), 0, 1e-12);
}

void DoBlockLDLTTest(const Eigen::MatrixXd& M,
                         const vector<std::vector<int>>& cliques) {
  auto b = MakeBlockSparseMatrix(M, cliques);
  auto llt_calc = b.llt(); llt_calc.compute();

  MatrixXd L = llt_calc.matrixL();
  MatrixXd D = llt_calc.vectorD().asDiagonal();
  MatrixXd P = llt_calc.matrixP();
  DUMP(P);
  DUMP(L);
  DUMP(D);

  MatrixXd M_permuted = llt_calc.matrixP().transpose()*M * llt_calc.matrixP();
  DUMP(M_permuted);


  DUMP(M_permuted - L*L.transpose());
  DUMP(M_permuted - L*D*L.transpose());
  //EXPECT_NEAR( (M_permuted - L*D*L.transpose()).norm(), 0);
  return;

  VectorXd x;
  x.setLinSpaced(L.cols(), -1, 1.1);
  VectorXd Lx = L * x;
  llt_calc.ApplyInverseOfL(&Lx);
  EXPECT_NEAR( (Lx - x).norm(), 0, 1e-12);

  VectorXd Ltx = L.transpose() * x;
  llt_calc.ApplyInverseOfLt(&Ltx);
  EXPECT_NEAR((Ltx - x).norm(), 0, 1e-12);
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
  const double eps = 1e-14;
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.AssembleFromCompressedColumns(
      GetCompressedBlockColumns(Ref, block_sizes));

  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, eps);


  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, eps);
  SimpleTriangularMatrix::LLT llt = mat.llt();
  llt.compute();
  MatrixXd llt_calc = mat.MakeDenseMatrix();
  MatrixXd L_ref = Eigen::LLT<MatrixXd>(Ref).matrixL();
  MatrixXd error = (llt_calc - L_ref).triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, eps);


  VectorXd x;
  x.setLinSpaced(L_ref.cols(), -1, 1.1);
  VectorXd Lx = L_ref * x;
  llt.ApplyInverseOfL(&Lx);
  EXPECT_NEAR( (Lx - x).norm(), 0, 1e-12);

  VectorXd Ltx = L_ref.transpose() * x;
  llt.ApplyInverseOfLt(&Ltx);
  EXPECT_NEAR((Ltx - x).norm(), 0, 1e-12);

}

}  // namespace
#if 0
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

  //TriangularMatrixDirectSum mat(mats);
  //MatrixXd full_mat = mat.MakeDenseMatrix();

  //// full_mat = L1
  ////               L2
  ////                  L3
  ////            L1 L2 L3  R1 + R2 + R3
  //// Easy sanity check.
  //double squared_norm = full_mat.leftCols(block_sizes.at(0) * 2 + block_sizes_2.at(0)).squaredNorm();
  //EXPECT_NEAR(L1.squaredNorm() + L2.squaredNorm() + L3.squaredNorm(), squared_norm, 1e-12);
  //double sum = R1.colwise().sum().sum() + R2.colwise().sum().sum() + R3.colwise().sum().sum();
  //EXPECT_NEAR(sum, full_mat.bottomRightCorner(block_sizes_2.back(), block_sizes_2.back()).colwise().sum().sum(), 1e-12);

  //auto llt = mat.llt();
  //llt.compute();
  //MatrixXd llt_ref = Eigen::LLT<MatrixXd>(full_mat).matrixL();
  //EXPECT_NEAR((llt.matrixL() - llt_ref).norm(), 0, 1e-12);
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




GTEST_TEST(BlockSymmetricMatrixCholesky, Arrow) {
  vector<vector<int>> cliques{ {0, 1}, { 1, 2, 3}, {1, 5}, {1, 4} };
  MatrixXd M(6, 6);
  M << 9, 0, 0, 0, 0, 0, 
       1, 9, 0, 0, 0, 0, 
       0, 2, 9, 0, 0, 0, 
       0, 2, 2, 9, 0, 0,
       0, 3, 0, 0, 9, 0,
       0, 4, 0, 0, 0, 9;
  DoBlockCholeskyTest(M.selfadjointView<Eigen::Lower>(), cliques); 
}

#endif
#if 1
GTEST_TEST(BlockSymmetricMatrixCholesky, MassMatrix) {
  vector<vector<int>> cliques{{0, 1, 2, 3, 4, 5, 18, 19, 20, 21},
                              {0, 1, 2, 3, 4, 5, 14, 15, 16, 17},
                              {0, 1, 2, 3, 4, 5, 10, 11, 12, 13},
                              {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}};
  int num_vars = 22;

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
  DoBlockLDLTTest(M, cliques);
}
#endif
#if 1
GTEST_TEST(BlockSymmetricMatrixCholesky, BlockDiag) {
  vector<vector<int>> cliques;

  int size_blocks = 4;
  int num_blocks = 2;
  cliques.clear();
  cliques.resize(num_blocks);

  for (int i = 0; i < size_blocks; i++) {
    for (int j = 0; j < num_blocks; j++) {
      cliques.at(j).push_back(i + j * size_blocks);
    }
  }

  MatrixXd M(num_blocks * size_blocks, num_blocks * size_blocks);
  M.setZero();
  for (int j = 0; j < num_blocks; j++) {
    MatrixXd Mi(size_blocks, size_blocks);
    Mi.setRandom();
    MatrixXd Mt = Mi.transpose();
    Mi = Mi * Mt;
    M.block(j*size_blocks, j * size_blocks, size_blocks, size_blocks) = Mi;
  }

  //DoBlockCholeskyTest(M, cliques);
  DoBlockLDLTTest(M, cliques);
}
#endif
}  // namespace conex
