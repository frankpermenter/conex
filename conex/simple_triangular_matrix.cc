#include "conex/simple_triangular_matrix.h"
#include "conex/debug_macros.h"

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

struct TriangularMatrixInputs {
  TriangularMatrixInputs(int num_cliques, int num_vars) : 
      elimination_position_to_variable(num_vars),
      block_sizes(num_cliques, 0) {
        
      }
  std::vector<int> block_sizes;
  std::vector<int> triplets; 
  std::vector<int> elimination_position_to_variable;
};

std::vector<int> CalculateBlockSizes(int num_blocks,
                  const std::vector<int>& enter, 
                  const std::vector<int>& exit) {

  std::vector<int> block_sizes(num_blocks, 0);
  for (auto i : exit) {
    block_sizes.at(i)++;
  }
  return block_sizes;
}

std::vector<SimpleTriangularMatrixTriplet> MakeTripets(
                const std::vector<int>& enter, 
                const std::vector<int>& exit) {

  std::vector<SimpleTriangularMatrixTriplet> triplets;
  for (size_t i = 0; i < exit.size(); i++) {
    if (enter.at(i) != exit.at(i)) {
      triplets.push_back({exit.at(i), enter.at(i), 1});
    }
  }
  return triplets;
}


using BlockData = std::vector<std::pair<int, int>>;
template <typename T>
class BlockMatrix {
 public:
  BlockMatrix(const Eigen::Ref<T>& X, const BlockData& blocks)
      : X_(X), blocks_(blocks) { }

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
    while (blocks_.at(current_block_index_).first != i) {
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


BlockSparseSymmetricMatrix MakeBlockSparseMatrix(const Eigen::MatrixXd& M, 
                                                        const vector<vector<int>>& cliques) {

  int num_vars = M.rows();

  int num_cliques = cliques.size();
  std::vector<int> enter(num_vars, -1);
  std::vector<int> exit(num_vars, -1);
  std::vector<int> block_sizes(num_cliques);
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

  BlockSparseSymmetricMatrix mat(cliques.size(), enter, exit);
  mat.SetFromDenseMatrix(M);
  return mat;
}


BlockSparseSymmetricMatrix::BlockSparseSymmetricMatrix(
  const int num_blocks, 
  const std::vector<int>& start_block,
  const std::vector<int>& end_block) : 
    block_sizes_(CalculateBlockSizes(num_blocks, start_block, end_block)),
    elimination_position_to_variable_(start_block.size()),
    lower_triangular_matrix_(block_sizes_, MakeTripets(start_block, end_block)) {

  std::iota(elimination_position_to_variable_.begin(), elimination_position_to_variable_.end(), 0);
  std::sort(elimination_position_to_variable_.begin(), 
            elimination_position_to_variable_.end(),
            [start_block, end_block](const int& i, const int& j) {
              return (end_block[i] < end_block[j]) ||
                     (end_block[i] == end_block[j] && start_block[i] < start_block[j]);
            });
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
                           const std::vector<std::pair<int, int>>& partition) {

  int offset = 0;
  for (size_t i = 0; i < partition.size() - 1; i++) {
    const auto& d = partition.at(i);

    diagonal_blocks_.at(d.first).topLeftCorner(d.second, d.second) += x.block(offset, offset, 
                                                                           d.second, d.second);

    int r_offset = offset + d.second;

    BlockMatrix<MatrixXd> blocks(off_diagonal_blocks_.at(d.first), 
                                 off_diagonal_partition_.at(d.first));
    for (size_t j = i+1; j < partition.size(); j++) {
      auto&o = partition.at(j);
      blocks.GotoBlock(o.first);
      blocks.CurrentBlock().leftCols(o.second).topRows(d.second) += 
          x.block(r_offset, offset, o.second, d.second).transpose();
      r_offset += o.second;
    }

    offset += d.second;
  }

  const auto& d = partition.back();
  diagonal_blocks_.at(d.first).topLeftCorner(d.second, d.second) += x.block(offset, offset, 
                                                                           d.second, d.second);

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

D::TriangularMatrixDirectSum(std::vector<SimpleTriangularMatrix>& matrices) : matrices_(matrices) {
  int common_block_size = 0;
  for (const auto& mat : matrices) {
    if (mat.block_sizes().back() > common_block_size) {
      common_block_size = mat.block_sizes().back();
    }
  }
  common_block_.resize(common_block_size, common_block_size);
  common_block_.setZero();
}

bool D::LLT::compute(bool factor_last_block) {
  auto& matrices = matrix_.matrices_;
  auto& common_block_ = matrix_.root_matrix();

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
  int common_block_size = matrix_.common_block_.rows();
  L.bottomRightCorner(common_block_size, common_block_size) = llt_of_diag_.back().matrixL();
  return L;
}


void S::AssembleFromDenseMatrix(const Eigen::MatrixXd& A) {
  int c = 0;

  std::vector<int> global_offsets(num_blocks_, 0);
  std::partial_sum(block_column_sizes_.begin(), block_column_sizes_.end() - 1,
                   global_offsets.begin() + 1);

  for (size_t i = 0; i < block_column_sizes_.size(); i++) {
    int csize = block_column_sizes_.at(i);
    diagonal_blocks_.at(i) = A.block(c, c, csize, csize);
    int r = 0;
    for (auto& row : off_diagonal_partition_.at(i)) {
      int r_offset = global_offsets.at(row.first);
      off_diagonal_blocks_.at(i).middleCols(r, row.second) = A.block(r_offset, c, row.second, csize).transpose();
      r += row.second;
    }
    c += csize;
  }
}

}  // namespace conex
