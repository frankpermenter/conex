#include "conex/simple_triangular_matrix.h"
#include "conex/tree_utils.h"
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

class TriangularMatrixDirectSum {
 public:
  TriangularMatrixDirectSum(std::vector<SimpleTriangularMatrix>& matrices) : matrices_(matrices) {}
  std::vector<SimpleTriangularMatrix>& matrices_;

  MatrixXd MakeDenseMatrix() {
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
      M.block(common_block_offset, common_block_offset, last_block, last_block) += mat.diagonal_blocks().back(); 
      i++;
      offset += block_size;
    }

    return M;
  }

  class LLT {
   public:
    bool compute() {
      auto& matrices = matrix_.matrices_;
      int common_block_offset = 0;
      int common_block_size = 0;
      for (const auto& mat : matrices) {
        common_block_offset += mat.cols() - mat.block_sizes().back();
        if (mat.block_sizes().back() > common_block_size) {
          common_block_size = mat.block_sizes().back();
        }
      }

      common_block_.resize(common_block_size, common_block_size);
      common_block_.setZero();

      for (auto& mat : matrices) {
        auto llt = mat.llt();
        llt.compute(false);
      }

      for (const auto& mat : matrices) {
        int last_block = mat.block_sizes().back();
        common_block_.topLeftCorner(last_block, last_block) += mat.diagonal_blocks().back(); 
      }
      llt_of_diag_.emplace_back(common_block_);
      return true;
    }

    Eigen::MatrixXd matrixL() { 
      MatrixXd L = matrix_.MakeDenseMatrix();
      int common_block_size = common_block_.rows();
      L.bottomRightCorner(common_block_size, common_block_size) = llt_of_diag_.back().matrixL();
      return L;
    }
   private:
    LLT(TriangularMatrixDirectSum* matrix) : matrix_(*matrix) { }
    bool ready() { return true; }
    TriangularMatrixDirectSum& matrix_;
    std::vector<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>> llt_of_diag_;
    Eigen::MatrixXd common_block_;
    friend class TriangularMatrixDirectSum;
    bool factorization_ready_ = false;
  };

  LLT llt() { return LLT(this); }
};

// Builds a triangular matrices from a direct sum.
GTEST_TEST(SimpleTri, DirectSum) {

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

  // clang-format off
  MatrixXd data_2(5, 5);
  data_2 << 4, 0, 0, 0, 0,
            1, 4, 0, 0, 0,
            1, 1, 2, 0, 0,
            1, 1, 1, 2, 0,
            0, 0, 1, 2, 2;
  // clang-format on
  data_2 += 10 * MatrixXd::Identity(data_2.rows(), data_2.rows());

  std::vector<int> block_sizes_2{2, 3};
  std::vector<SimpleTriangularMatrixTriplet> triplets_2{{1, 0, 2}};
  mats.emplace_back(block_sizes_2, triplets_2);
  mats.at(2).AssembleFromCompressedColumns(GetCompressedBlockColumns(data_2, block_sizes_2));






  TriangularMatrixDirectSum mat(mats);
  MatrixXd full_mat = mat.MakeDenseMatrix();
  DUMP(full_mat);
  auto llt = mat.llt();
  llt.compute();
  MatrixXd llt_ref = Eigen::LLT<MatrixXd>(full_mat).matrixL();
  EXPECT_NEAR((llt.matrixL() - llt_ref).norm(), 0, 1e-12);
}

}  // namespace conex
