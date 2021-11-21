#include "conex/simple_triangular_matrix.h"
#include "conex/debug_macros.h"

#include <numeric>

#include "gtest/gtest.h"

#include <Eigen/Dense>
namespace conex {

using Eigen::MatrixXd;
using std::vector;
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


class TriangularMatrix {
  std::vector<int> tree;
  std::vector<int> subtree_roots;

  //      1
  //   2     3
  //   4     5
  //  6 7    8 
  //
  //  
  //  SimpleTriangularMatrix:  DS(1,  LastClique( LastClique( Path(2, LastClique( DS(4, {6, 7}))),    )
  //
  //
  //  Tri = DS   if children > 1
  //  Tri = Simp if children > 1

};


class DS {

};


// T
class TriangularMatrixDirectSum {
 public:
  TriangularMatrixDirectSum(std::vector<SimpleTriangularMatrix>& matrices) : matrices_(matrices) {}
  std::vector<SimpleTriangularMatrix>& matrices_;

  MatrixXd MakeFullMatrix() {
    auto& matrices = matrices_;
    int common_block_offset = 0;
    int common_block_size = 0;
    for (const auto& mat : matrices) {
      common_block_offset += mat.cols() - mat.block_sizes().back();
      if (mat.block_sizes().back() > common_block_size) {
        common_block_size = mat.block_sizes().back();
      }
    }
    int size = common_block_offset + common_block_size;
    MatrixXd M(size, size); M.setZero();
    int i = 0; 
    int offset = 0; 
    for (const auto& mat : matrices) {
      int last_block = mat.block_sizes().back();
      int block_size = mat.cols() - last_block;

      MatrixXd Mi = mat.MakeDenseMatrix();
      M.block(offset, offset, block_size, block_size) = Mi.topLeftCorner(block_size, block_size);
      M.block(common_block_offset, offset, last_block, block_size) = Mi.bottomLeftCorner(last_block, block_size);
      M.block(common_block_offset, common_block_offset, last_block, block_size) += Mi.bottomRightCorner(last_block, last_block);
      i++;
      offset += block_size;
    }

    return M;
  }

  MatrixXd InPlaceLLT() {
    auto& matrices = matrices_;
    int common_block_offset = 0;
    int common_block_size = 0;
    for (const auto& mat : matrices) {
      common_block_offset += mat.cols() - mat.block_sizes().back();
      if (mat.block_sizes().back() > common_block_size) {
        common_block_size = mat.block_sizes().back();
      }
    }

    for (auto& mat : matrices) {
      auto llt = mat.llt();
      llt.compute(false);
    }
    MatrixXd L = MakeFullMatrix();
    MatrixXd L_bottom = L.bottomRightCorner(common_block_size, common_block_size);
    Eigen::LLT<Eigen::Ref<MatrixXd>> llt(L_bottom);
    L.bottomRightCorner(common_block_size, common_block_size) = llt.matrixL();
    return L;
  }
};

GTEST_TEST(SimpleTri, DirectSum) {
  vector<int> master_block_sizes{2, 2, 3};

  // clang-format off
  MatrixXd Ref(4, 4);
  Ref << 4, 0, 0, 0,
         1, 4, 0, 0,
         1, 1, 2, 0,
         1, 1, 1, 2;
  // clang-format on
  Ref += 10 * MatrixXd::Identity(Ref.rows(), Ref.rows());

  std::vector<int> block_sizes{2, 2};
  std::vector<SimpleTriangularMatrixTriplet> triplets{{1, 0, 2}};
  vector<SimpleTriangularMatrix> mats;
  mats.emplace_back(block_sizes, triplets);
  mats.emplace_back(block_sizes, triplets);
  mats.at(0).AssembleFromCompressedColumns(GetCompressedBlockColumns(Ref, block_sizes));
  mats.at(1).AssembleFromCompressedColumns(GetCompressedBlockColumns(Ref, block_sizes));

  TriangularMatrixDirectSum mat(mats);
  MatrixXd full_mat = mat.MakeFullMatrix();
  MatrixXd llt = mat.InPlaceLLT();
  MatrixXd llt_ref = Eigen::LLT<MatrixXd>(full_mat).matrixL();
  EXPECT_NEAR((llt - llt_ref).norm(), 0, 1e-12);
}

}  // namespace conex
