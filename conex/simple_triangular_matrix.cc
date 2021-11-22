#include "conex/simple_triangular_matrix.h"

#include <numeric>

namespace conex {

using Eigen::MatrixXd;
using S = SimpleTriangularMatrix;
using std::vector;

namespace {

template <typename T>
vector<size_t> sort_indexes(const vector<T>& v) {
  vector<size_t> indices(v.size());
  iota(indices.begin(), indices.end(), 0);

  stable_sort(indices.begin(), indices.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return indices;
}


using BlockData = std::vector<std::pair<int, int>>;
template <typename T>
class BlockMatrix {
 public:
  BlockMatrix(const Eigen::Ref<T>& X, const BlockData& blocks)
      : X_(X), blocks_(blocks) {}

  BlockMatrix(const Eigen::Ref<T>& X, const BlockData& blocks,
              int initial_index)
      : X_(X), blocks_(blocks) {
    for (int i = 0; i < initial_index; i++) {
      GotoNextBlock();
    }
  }

  bool GotoNextBlock() {
    current_block_offset_ += blocks_.at(current_block_index_).second;
    current_block_index_++;

    if (current_block_index_ >= static_cast<int>(blocks_.size())) {
      return true;
    }
    return false;
  }

  void GotoBlock(int i) {
    while (blocks_.at(current_block_offset_).first != i) {
      GotoNextBlock();
    }
  }

  int CurrentBlockNumber() { return blocks_.at(current_block_index_).first; }

  int CurrentBlockOffset() { return current_block_offset_; }

  int CurrentBlockSize() { return blocks_.at(current_block_index_).second; }

  Eigen::Ref<T> CurrentBlock() {
    return X_.middleCols(current_block_offset_, CurrentBlockSize());
  }

  Eigen::Ref<T> X_;
  const BlockData& blocks_;
  size_t current_block_offset_ = 0;
  int current_block_index_ = 0;
};
}  // namespace


void S::DoIncrementOffDiagonalBlockColumns(const Eigen::MatrixXd& x, int block, 
                                      int column_start) {
  off_diagonal_blocks_[block].middleRows(column_start, x.cols()) += x.transpose();
}

void S::DoIncrementDiagonalBlockColumns(const Eigen::MatrixXd& x, int block, 
                                     int column_start) {
  diagonal_blocks_[block].middleCols(column_start, x.cols()) += x;
}


S::SimpleTriangularMatrix(
    const std::vector<int>& block_column_sizes,
    const std::vector<SimpleTriangularMatrixTriplet>& input_triplets)
    : block_column_sizes_(block_column_sizes),
      off_diagonal_triplets_(input_triplets) {
  num_cols_ =
      std::accumulate(block_column_sizes.begin(), block_column_sizes.end(), 0);
  num_blocks_ = block_column_sizes.size();
  diagonal_blocks_.resize(num_blocks_);
  off_diagonal_blocks_.resize(num_blocks_ - 1);
  for (int i = 0; i < num_blocks_; i++) {
    diagonal_blocks_[i].resize(block_column_sizes[i], block_column_sizes[i]);
  }

  std::vector<int> off_diagonal_size(num_blocks_ - 1, 0);
  Eigen::MatrixXi M(num_blocks_, num_blocks_);
  M.setZero();
  for (auto s : input_triplets) {
    for (int i = s.block_col; i < s.block_row; i++) {
      off_diagonal_size.at(i) += s.num_rows_entering;
      M(s.block_row, i) += s.num_rows_entering;
    }
  }
  for (size_t i = 0; i < block_column_sizes.size() - 1; i++) {
    off_diagonal_blocks_[i].resize(block_column_sizes[i], off_diagonal_size[i]);
  }

  off_diagonal_partition_.resize(num_blocks_);
  for (int i = 0; i < M.cols() - 1; i++) {
    for (int j = i + 1; j < M.rows(); j++) {
      if (M(j, i) > 0) {
        off_diagonal_partition_.at(i).emplace_back(j, M(j, i));
      }
    }
  }
}

MatrixXd S::MakeDenseMatrix() const {
  MatrixXd M(num_cols_, num_cols_);
  M.setZero();
  int offset = 0;
  for (size_t i = 0; i < block_column_sizes_.size(); i++) {
    M.block(offset, offset, block_column_sizes_[i], block_column_sizes_[i]) =
        diagonal_blocks_.at(i);
    offset += block_column_sizes_[i];
  }

  std::vector<int> global_offsets(num_blocks_, 0);
  std::partial_sum(block_column_sizes_.begin(), block_column_sizes_.end() - 1,
                   global_offsets.begin() + 1);

  for (size_t i = 0; i < block_column_sizes_.size() - 1; i++) {
    if (off_diagonal_partition_.at(i).size() > 0) {
      BlockMatrix<const MatrixXd> block(off_diagonal_blocks_.at(i),
                                        off_diagonal_partition_.at(i));
      do {
        int row_block = block.CurrentBlockNumber();
        int num_rows = block.CurrentBlockSize();
        M.block(global_offsets.at(row_block), global_offsets.at(i), num_rows,
                block_column_sizes_.at(i)) = block.CurrentBlock().transpose();
      } while (!block.GotoNextBlock());
    }
  }

  return M;
}

// Replace bottom right corner C_22 with  C22 - C12' inv(C11) C12.
// We assume that (C11)^{-1/2} C12 has already been computed
// and stored in the block C12.  The full matrix C starts
// at the diagonal block (i, i).
void S::LLT::SchurComplementInPlace(int block) {
  auto& Rdata = matrix_.off_diagonal_blocks_.at(block);
  if (Rdata.size() == 0) {
    return;
  }
  const BlockData& input_block_info = matrix_.off_diagonal_partition_.at(block);
  auto& off_diagonal_blocks = matrix_.off_diagonal_blocks_;
  auto& diagonal_blocks = matrix_.diagonal_blocks_;
  BlockMatrix<MatrixXd> input_i(Rdata, input_block_info);
  for (size_t i = 0; i < input_block_info.size() - 1; i++) {
    int size_i = input_i.CurrentBlockSize();
    BlockMatrix<MatrixXd> output(
        off_diagonal_blocks.at(input_i.CurrentBlockNumber()),
        matrix_.off_diagonal_partition_.at(input_i.CurrentBlockNumber()));
    BlockMatrix<MatrixXd> input_j(Rdata, input_block_info, i + 1);
    for (size_t j = i + 1; j < input_block_info.size(); j++) {
      int size_j = input_j.CurrentBlockSize();
      output.GotoBlock(input_j.CurrentBlockNumber());
      output.CurrentBlock().topLeftCorner(size_i, size_j) -=
          input_i.CurrentBlock().transpose() * input_j.CurrentBlock();
      input_j.GotoNextBlock();
    }

    diagonal_blocks.at(input_i.CurrentBlockNumber())
        .topLeftCorner(size_i, size_i) -=
        input_i.CurrentBlock().transpose() * input_i.CurrentBlock();
    input_i.GotoNextBlock();
  }
  int size_i = input_i.CurrentBlockSize();
  diagonal_blocks.at(input_i.CurrentBlockNumber())
      .topLeftCorner(size_i, size_i) -=
      input_i.CurrentBlock().transpose() * input_i.CurrentBlock();
}

bool S::LLT::compute(bool factor_last_block) {
  llt_of_diag_.clear();
  for (int i = 0; i < matrix_.num_blocks_ - 1; i++) {
    llt_of_diag_.emplace_back(matrix_.diagonal_blocks_[i]);
    if (matrix_.off_diagonal_blocks_[i].size() > 0) {
      llt_of_diag_.back().matrixL().solveInPlace(
          matrix_.off_diagonal_blocks_[i]);
      SchurComplementInPlace(i);
    }
  }
  if (factor_last_block) {
    llt_of_diag_.emplace_back(matrix_.diagonal_blocks_.back());
  }
  return true;
}

void S::IncrementSubmatrix(const Eigen::MatrixXd& x, 
                        std::vector<std::pair<int, int>> diagonal_blocks,
                        std::vector<std::pair<int, int>> row_partition) {
  int offset = 0;
  for (size_t i = 0; i < diagonal_blocks.size(); i++) {
    const auto& d = diagonal_blocks.at(i);
    diagonal_blocks_.at(d.first).topLeftCorner(d.second, d.second) += x.block(offset, offset, 
                                                                             d.second, d.second);
    int r_offset = offset + d.second;

    BlockMatrix<MatrixXd> blocks(off_diagonal_blocks_.at(d.first), off_diagonal_partition_.at(d.first));
    for (size_t j = i+1; j < row_partition.size(); j++) {
      auto&o = row_partition.at(j);
      blocks.GotoBlock(o.first);
      blocks.CurrentBlock().leftCols(o.second).topRows(d.second) += x.block(r_offset, offset, o.second, d.second);
      r_offset += o.second;
    }

    offset += d.second;
  }
}


using D = TriangularMatrixDirectSum;
MatrixXd D::MakeDenseMatrix() {
    using Eigen::MatrixXd;
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

bool D::LLT::compute(bool factor_last_block) {
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

  if (factor_last_block) {
    llt_of_diag_.emplace_back(common_block_);
  }
  factorization_ready_ = true;
  return true;
}

MatrixXd D::LLT::matrixL() { 
  MatrixXd L = matrix_.MakeDenseMatrix();
  int common_block_size = common_block_.rows();
  L.bottomRightCorner(common_block_size, common_block_size) = llt_of_diag_.back().matrixL();
  return L;
}

}  // namespace conex
