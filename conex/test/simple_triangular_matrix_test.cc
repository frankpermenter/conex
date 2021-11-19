#include "conex/block_triangular_operations.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;

GTEST_TEST(SimpleTri, Construct) {
  std::vector<int> block_sizes{2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 2}}; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(6, 6);
  Ref << 1, 1, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0,
         0, 0, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1,
         1, 1, 1, 1, 1, 1;
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}

GTEST_TEST(SimpleTri, ConstructDifferentSizes) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{2, 0, 1},  {2, 1, 1}  }; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;

  DUMP(mat.MakeDenseMatrix());
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}
GTEST_TEST(SimpleTri, EmptyTriplets) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{}; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         0, 0, 0, 0, 0, 1, 1,
         0, 0, 0, 0, 0, 1, 1;
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}

GTEST_TEST(SimpleTri, NonzeroOnAllBlocks) {
  std::vector<int> block_sizes{2, 3, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {1, 0, 2}, {2, 0, 1}, {2, 1, 1} }; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1;
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}

GTEST_TEST(SimpleTri, OutofOrderRows) {
  std::vector<int> block_sizes{2, 2, 2, 1};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {3, 0, 1}, {2, 1, 1} }; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(7, 7);
  Ref << 1, 1, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0,
         0, 0, 1, 1, 1, 1, 0,
         0, 0, 0, 0, 1, 1, 0,
         1, 1, 1, 1, 1, 1, 1;
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}

GTEST_TEST(SimpleTri, AddTwo) {
  std::vector<int> block_sizes{2, 2, 2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{ {2, 0, 1},   {3, 0, 1},   {2, 1, 1},  {3, 1, 1} }; 
  SimpleTriangularMatrix mat(block_sizes, triplets);
  mat.SetConstant(1); 
  MatrixXd Ref(8, 8);
  Ref << 1, 1, 0, 0, 0, 0, 0, 0,
         1, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         0, 0, 1, 1, 0, 0, 0, 0,
         1, 1, 1, 1, 1, 1, 0, 0,
         0, 0, 1, 1, 1, 1, 0, 0,
         1, 1, 1, 1, 1, 1, 1, 1,
         0, 0, 1, 1, 1, 1, 1, 1;
  DUMP(Ref);
  EXPECT_NEAR((mat.MakeDenseMatrix() - Ref).norm(), 0, 1e-15); 
}




}  // namespace conex
