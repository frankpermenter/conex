#include "conex/block_triangular_operations.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

namespace {

using Eigen::MatrixXd;
void DoTest(const std::vector<int>& block_sizes,
       const std::vector<SimpleTriangularMatrixTriplet>& triplets,
       const MatrixXd& Ref) {

  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
  SimpleTriangularMatrix::LLT llt = mat.llt();
  llt.SchurComplementInPlace(0);
  int d = block_sizes.at(0);
  int n = Ref.rows() - d;
  MatrixXd V = Ref.bottomLeftCorner(n, d);

  MatrixXd schur_complement_ref = Ref.bottomRightCorner(n, n) - V*V.transpose();
  MatrixXd schur_complement_calc = mat.MakeDenseMatrix().bottomRightCorner(n, n);

  MatrixXd error = (schur_complement_calc - schur_complement_ref).triangularView<Eigen::Lower>();
  EXPECT_NEAR(error.norm(), 0, 1e-15); 
}


}


GTEST_TEST(SimpleTri, Construct) {
  std::vector<int> block_sizes{2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 2}}; 
  MatrixXd Ref(6, 6);
  Ref << 1, 1, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0,
         0, 0, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1;
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, ConstructDifferentSizes) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 1},  {2, 1, 1}  }; 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;

  DoTest(block_sizes, triplets, Ref);
}
GTEST_TEST(SimpleTri, EmptyTriplets) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{}; 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 1,
         0, 0, 0, 0, 0, 1, 1;
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, NonzeroOnAllBlocks) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {1, 0, 2}, {2, 0, 1}, {2, 1, 1} }; 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;
  DoTest(block_sizes, triplets, Ref);
}

GTEST_TEST(SimpleTri, OutofOrderRows) {
  std::vector<int> block_sizes{2, 2, 2, 1};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {3, 0, 1}, {2, 1, 1} }; 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 1, 1, 0,
         0, 0, 0, 0, 1, 1, 0,
         1, 1, 1, 1, 1, 1, 1;
  DoTest(block_sizes, triplets, Ref);
}


GTEST_TEST(SimpleTri, AddTwo) {
  std::vector<int> block_sizes{2, 2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {2, 0, 1},   {3, 0, 1},   {2, 1, 1},  {3, 1, 1} }; 
  MatrixXd Ref(8, 8);
  Ref << 1, 1, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1, 1;

  DoTest(block_sizes, triplets, Ref);
}




}  // namespace conex
