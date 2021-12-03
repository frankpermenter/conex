#define CONEX_ENABLE_TIMER 1
#include "conex/simple_triangular_matrix.h"
#include "conex/dense_triangular_factorizations.h"
#include "conex/debug_macros.h"

#include <numeric>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using S = SimpleTriangularMatrix;
using std::vector;

namespace {

class PartitionVectorIterator {
 public:
  PartitionVectorIterator(VectorXd& b, int N, const vector<int>& sizes)
      : b_(b), N_(N), sizes_(sizes) {
    Reset();
  }

  Eigen::Ref<VectorXd> b_i() { return b_.segment(start_i, size_i); }
  Eigen::Ref<VectorXd> b_i_plus_1() {
    return b_.segment(start_i_plus_1, size_i_plus_1);
  }
  void Reset() {
    i_ = sizes_.size() - 1;
    size_i = sizes_[i_];
    start_i = N_ - size_i;
  }
  void Decrement() {
    start_i_plus_1 = start_i;
    size_i_plus_1 = size_i;
    i_--;
    size_i = sizes_[i_];
    start_i = start_i_plus_1 - size_i;
  }

  int i_ = 0;
  int start_i_plus_1;
  int start_i;
  int size_i_plus_1;
  int size_i;
  VectorXd& b_;
  const int N_;
  const vector<int>& sizes_;
  void Set(int i) {
    if (i < 0) {
      assert(0);
    }
    if (i > i_) {
      assert(0);
    }
    while (i < i_) {
      Decrement();
    }
  }
};

class PartitionVectorForwardIterator {
 public:
  PartitionVectorForwardIterator(VectorXd& b, const vector<int>& sizes)
      : b_(b), sizes_(sizes) {
    Reset();
  }

  Eigen::Ref<VectorXd> b_i() { return b_.segment(start_i, size_i); }
  Eigen::Ref<VectorXd> b_i_minus_1() {
    return b_.segment(start_i_minus_1, size_i_minus_1);
  }

  void Reset() {
    i_ = 0;
    size_i = sizes_[i_];
    start_i = 0;
  }
  void Increment() {
    start_i_minus_1 = start_i;
    size_i_minus_1 = size_i;
    i_++;
    size_i = sizes_[i_];
    start_i = start_i_minus_1 + size_i_minus_1;
  }

  int i_ = 0;
  int start_i_minus_1;
  int start_i;
  int size_i_minus_1;
  int size_i;
  VectorXd& b_;
  const vector<int>& sizes_;
  void Set(int i) {
    if (i > 0) {
      assert(0);
    }
    if (i < i_) {
      assert(0);
    }
    while (i > i_) {
      Increment();
    }
  }
};


template <typename T>
vector<size_t> sort_indexes(const vector<T>& v) {
  vector<size_t> indices(v.size());
  iota(indices.begin(), indices.end(), 0);

  stable_sort(indices.begin(), indices.end(),
              [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

  return indices;
}

vector<int> CalculateBlockSizes(int num_blocks,
                                     const vector<int>& enter,
                                     const vector<int>& exit) {
  vector<int> block_sizes(num_blocks, 0);
  for (auto i : exit) {
    block_sizes[i]++;
  }
  return block_sizes;
}

vector<SimpleTriangularMatrixTriplet> MakeTripets(
    const vector<int>& enter, const vector<int>& exit) {
  vector<SimpleTriangularMatrixTriplet> triplets;
  for (size_t i = 0; i < exit.size(); i++) {
    if (enter[i] != exit[i]) {
      triplets.push_back({exit[i], enter[i], 1});
    }
  }
  return triplets;
}

using BlockData = vector<std::pair<int, int>>;
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
    current_block_offset_ += blocks_[current_block_index_].second;
    current_block_index_++;

    if (current_block_index_ >= static_cast<int>(blocks_.size())) {
      return true;
    }
    return false;
  }

  void GotoBlock(int i) {
    while (blocks_[current_block_index_].first != i) {
      GotoNextBlock();
    }
  }

  int CurrentBlockNumber() { return blocks_[current_block_index_].first; }

  int CurrentBlockOffset() { return current_block_offset_; }

  int CurrentBlockSize() { return blocks_[current_block_index_].second; }

  Eigen::Ref<T> CurrentBlock() {
    return X_.middleCols(current_block_offset_, CurrentBlockSize());
  }

  Eigen::Ref<T> X_;
  const BlockData& blocks_;
  size_t current_block_offset_ = 0;
  int current_block_index_ = 0;
};
}  // namespace

BlockSparseSymmetricMatrix MakeBlockSparseMatrix(
    const MatrixXd& M, const vector<vector<int>>& cliques) {
  int num_vars = M.rows();

  int num_cliques = cliques.size();
  vector<int> enter(num_vars, -1);
  vector<int> exit(num_vars, -1);
  vector<int> block_sizes(num_cliques);
  for (size_t i = 0; i < cliques.size(); i++) {
    for (auto n : cliques[i]) {
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

bool BlockSparseSymmetricMatrix::LLT::compute(bool compute_ldlt) {
  llt_.compute(true /*factor last block*/, compute_ldlt);
  return true;
}

BlockSparseSymmetricMatrix::BlockSparseSymmetricMatrix(
    const int num_blocks, const vector<int>& start_block,
    const vector<int>& end_block)
    : block_sizes_(CalculateBlockSizes(num_blocks, start_block, end_block)),
      elimination_position_to_variable_(start_block.size()),
      lower_triangular_matrix_(block_sizes_,
                               MakeTripets(start_block, end_block)) {
  std::iota(elimination_position_to_variable_.begin(),
            elimination_position_to_variable_.end(), 0);
  std::sort(elimination_position_to_variable_.begin(),
            elimination_position_to_variable_.end(),
            [start_block, end_block](const int& i, const int& j) {
              return (end_block[i] < end_block[j]) ||
                     (end_block[i] == end_block[j] &&
                      start_block[i] < start_block[j]);
            });
}

S::SimpleTriangularMatrix(
    const vector<int>& block_column_sizes,
    const vector<SimpleTriangularMatrixTriplet>& input_triplets)
    : block_column_sizes_(block_column_sizes),
      off_diagonal_triplets_(input_triplets) {
  num_cols_ =
      std::accumulate(block_column_sizes.begin(), block_column_sizes.end(), 0);
  num_blocks_ = block_column_sizes.size();

  vector<int> off_diagonal_size(num_blocks_, 0);
  Eigen::MatrixXi M(num_blocks_, num_blocks_);
  M.setZero();
  for (auto s : input_triplets) {
    for (int i = s.block_col; i < s.block_row; i++) {
      off_diagonal_size[i] += s.num_rows_entering;
      M(s.block_row, i) += s.num_rows_entering;
    }
  }

#if USE_SEPARATE_STORAGE
  diagonal_blocks_.resize(num_blocks_);
  off_diagonal_blocks_.resize(num_blocks_ - 1);
  for (int i = 0; i < num_blocks_; i++) {
    diagonal_blocks_[i].resize(block_column_sizes[i], block_column_sizes[i]);
  }
  for (size_t i = 0; i < block_column_sizes.size() - 1; i++) {
    off_diagonal_blocks_[i].resize(block_column_sizes[i], off_diagonal_size[i]);
  }
#else
  block_columns_.resize(num_blocks_);
  for (int i = 0; i < num_blocks_; i++) {
    block_columns_[i].resize(block_column_sizes[i] + off_diagonal_size[i],
                             block_column_sizes[i]);
  }
#endif

  off_diagonal_partition_.resize(num_blocks_);
  for (int i = 0; i < M.cols() - 1; i++) {
    for (int j = i + 1; j < M.rows(); j++) {
      if (M(j, i) > 0) {
        off_diagonal_partition_[i].emplace_back(j, M(j, i));
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
        diagonal_blocks(i);
    offset += block_column_sizes_[i];
  }

  vector<int> global_offsets(num_blocks_, 0);
  std::partial_sum(block_column_sizes_.begin(), block_column_sizes_.end() - 1,
                   global_offsets.begin() + 1);

  for (size_t i = 0; i < block_column_sizes_.size() - 1; i++) {
    if (off_diagonal_partition_[i].size() > 0) {
      BlockMatrix<const MatrixXd> block(off_diagonal_blocks(i),
                                        off_diagonal_partition_[i]);
      do {
        int row_block = block.CurrentBlockNumber();
        int num_rows = block.CurrentBlockSize();
        M.block(global_offsets[row_block], global_offsets[i], num_rows,
                block_column_sizes_[i]) = block.CurrentBlock().transpose();
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
  if (matrix_.off_diagonal_blocks(block).size() == 0) {
    return;
  }
  const BlockData& input_block_info = matrix_.off_diagonal_partition_[block];
 
  if (vector_d_computed_) {
    const MatrixXd& D = matrix_.diagonal_blocks(block).diagonal().asDiagonal();
   
    BlockMatrix<MatrixXd> input_i(matrix_.off_diagonal_blocks(block),
                                  input_block_info);

    for (size_t i = 0; i < input_block_info.size() - 1; i++) {
      int size_i = input_i.CurrentBlockSize();
      BlockMatrix<MatrixXd> output(
          matrix_.off_diagonal_blocks(input_i.CurrentBlockNumber()),
          matrix_.off_diagonal_partition_[input_i.CurrentBlockNumber()]);
      BlockMatrix<MatrixXd> input_j(matrix_.off_diagonal_blocks(block),
                                    input_block_info, i + 1);
      for (size_t j = i + 1; j < input_block_info.size(); j++) {
        int size_j = input_j.CurrentBlockSize();
        output.GotoBlock(input_j.CurrentBlockNumber());
        output.CurrentBlock().topLeftCorner(size_i, size_j).noalias() -=
            input_i.CurrentBlock().transpose() * D * input_j.CurrentBlock();
        input_j.GotoNextBlock();
      }

      matrix_.diagonal_blocks(input_i.CurrentBlockNumber())
          .topLeftCorner(size_i, size_i)
          .noalias() -=
          input_i.CurrentBlock().transpose() * D * input_i.CurrentBlock();
      input_i.GotoNextBlock();
    }
    int size_i = input_i.CurrentBlockSize();
    matrix_.diagonal_blocks(input_i.CurrentBlockNumber())
        .topLeftCorner(size_i, size_i)
        .noalias() -= input_i.CurrentBlock().transpose() * D * input_i.CurrentBlock();
  } else {
    BlockMatrix<MatrixXd> input_i(matrix_.off_diagonal_blocks(block),
                                  input_block_info);

    for (size_t i = 0; i < input_block_info.size() - 1; i++) {
      int size_i = input_i.CurrentBlockSize();
      BlockMatrix<MatrixXd> output(
          matrix_.off_diagonal_blocks(input_i.CurrentBlockNumber()),
          matrix_.off_diagonal_partition_[input_i.CurrentBlockNumber()]);
      BlockMatrix<MatrixXd> input_j(matrix_.off_diagonal_blocks(block),
                                    input_block_info, i + 1);
      for (size_t j = i + 1; j < input_block_info.size(); j++) {
        int size_j = input_j.CurrentBlockSize();
        output.GotoBlock(input_j.CurrentBlockNumber());
        output.CurrentBlock().topLeftCorner(size_i, size_j).noalias() -=
            input_i.CurrentBlock().transpose()  * input_j.CurrentBlock();
        input_j.GotoNextBlock();
      }

      matrix_.diagonal_blocks(input_i.CurrentBlockNumber())
          .topLeftCorner(size_i, size_i)
          .noalias() -=
          input_i.CurrentBlock().transpose() * input_i.CurrentBlock();
      input_i.GotoNextBlock();
    }
    int size_i = input_i.CurrentBlockSize();
    matrix_.diagonal_blocks(input_i.CurrentBlockNumber())
        .topLeftCorner(size_i, size_i)
        .noalias() -= input_i.CurrentBlock().transpose() * input_i.CurrentBlock();
  }
}

bool S::LLT::compute(bool factor_last_block, bool compute_ldlt) {
  vector_d_computed_ = compute_ldlt;
  for (int i = 0; i < matrix_.num_blocks_ - 1; i++) {
    if (matrix_.off_diagonal_blocks(i).size() > 0) {
      if (compute_ldlt) {
        RectangularDenseLDLTInPlace(matrix_.diagonal_blocks(i),
                                    matrix_.off_diagonal_blocks(i));
      } else {
        RectangularDenseCholeskyInPlace(matrix_.diagonal_blocks(i),
                                    matrix_.off_diagonal_blocks(i));
      }
    } else {
      if (compute_ldlt) {
        RectangularDenseLDLTInPlace(matrix_.diagonal_blocks(i),
                                    matrix_.off_diagonal_blocks(i));
      } else {
        DenseCholeskyInPlace(matrix_.diagonal_blocks(i));
      }
    }
    if (matrix_.off_diagonal_blocks(i).size() > 0) {
      SchurComplementInPlace(i);
    }
  }
  if (factor_last_block) {
      if (compute_ldlt) {
        int i = matrix_.num_blocks_ - 1;
        MatrixXd empty(0, 0);
        RectangularDenseLDLTInPlace(matrix_.diagonal_blocks(i), empty);
      } else {
        DenseCholeskyInPlace(matrix_.diagonal_blocks(matrix_.num_blocks_ - 1));
      }
  }
  factorization_ready_ = true;
  return true;
}

void S::IncrementSubmatrix(const MatrixXd& x,
                           const vector<std::pair<int, int>>& partition) {
  int offset = 0;
  for (size_t i = 0; i < partition.size() - 1; i++) {
    const auto& d = partition[i];

    diagonal_blocks(d.first).topLeftCorner(d.second, d.second).noalias() +=
        x.block(offset, offset, d.second, d.second);

    int r_offset = offset + d.second;

    BlockMatrix<MatrixXd> blocks(off_diagonal_blocks(d.first),
                                 off_diagonal_partition_[d.first]);
    for (size_t j = i + 1; j < partition.size(); j++) {
      auto& o = partition[j];
      blocks.GotoBlock(o.first);
      blocks.CurrentBlock().leftCols(o.second).topRows(d.second).noalias() +=
          x.block(r_offset, offset, o.second, d.second).transpose();
      r_offset += o.second;
    }

    offset += d.second;
  }

  const auto& d = partition.back();
  diagonal_blocks(d.first).topLeftCorner(d.second, d.second).noalias() +=
      x.block(offset, offset, d.second, d.second);
}

void S::LLT::ApplyInverseOfL(VectorXd* y) const {
  PartitionVectorForwardIterator ypart(*y, matrix_.block_column_sizes_);

  for (int i = 0; i < matrix_.num_blocks() - 1; i++) {
    if (matrix_.diagonal_blocks(i).size() == 0) {
      ypart.Increment();
      continue;
    }
    if (vector_d_computed_) {
      matrix_.diagonal_blocks(i).triangularView<Eigen::UnitLower>().solveInPlace(ypart.b_i());
    } else {
      matrix_.diagonal_blocks(i).triangularView<Eigen::Lower>().solveInPlace(ypart.b_i());
    }
    if (matrix_.off_diagonal_blocks(i).size() > 0) {
      BlockMatrix block(matrix_.off_diagonal_blocks(i), matrix_.off_diagonal_partition_[i]);
      do {
        int row_block = block.CurrentBlockNumber();
        int num_rows = block.CurrentBlockSize();
        y->middleRows(global_offsets_[row_block], num_rows) -= block.CurrentBlock().transpose() * ypart.b_i();
      } while (!block.GotoNextBlock());
    }
    ypart.Increment();
  }
  if (vector_d_computed_) {
    matrix_.diagonal_blocks(matrix_.num_blocks_ -1).triangularView<Eigen::UnitLower>().solveInPlace(ypart.b_i());
  } else {
    matrix_.diagonal_blocks(matrix_.num_blocks_ -1).triangularView<Eigen::Lower>().solveInPlace(ypart.b_i());
  }
}

S::LLT::LLT(SimpleTriangularMatrix* matrix) : matrix_(*matrix), 
       global_offsets_(matrix_.num_blocks()) {
      std::partial_sum(matrix_.block_sizes().begin(), matrix_.block_sizes().end() - 1,
                       global_offsets_.begin() + 1);

    }

void S::LLT::ApplyInverseOfLt(VectorXd* y) const {
  PartitionVectorIterator y_partitioned(*y, y->rows(),
                                        matrix_.block_sizes());
  PartitionVectorForwardIterator b_partitioned(*y, matrix_.block_sizes());
  for (int k = static_cast<int>(matrix_.num_blocks() - 1); k > 0; k--) {
    if (matrix_.diagonal_blocks(k).size() == 0) {
      y_partitioned.Decrement();
      continue;
    }

    if (vector_d_computed_) {
      matrix_.diagonal_blocks(k).triangularView<Eigen::UnitLower>().transpose().solveInPlace(
          y_partitioned.b_i());
    } else {
      matrix_.diagonal_blocks(k).triangularView<Eigen::Lower>().transpose().solveInPlace(
          y_partitioned.b_i());
    }

    b_partitioned.Reset();

    for (int j = 0; j < k; j++) {
      int size = 0;
      int offset = 0;
      for (auto a : matrix_.off_diagonal_partition_[j])  {
        if (a.first == k) {
          size = a.second;
          break;
        } else {
          offset += a.second;
        }
      }

      y->middleRows(global_offsets_[j], matrix_.block_sizes()[j])-= matrix_.off_diagonal_blocks(j).middleCols(offset, size) 
                                                                   * y_partitioned.b_i().head(size);
      b_partitioned.Increment();
    }

    y_partitioned.Decrement();
  }
  if (matrix_.diagonal_blocks(0).size() > 0) {
    if (vector_d_computed_) {
    matrix_.diagonal_blocks(0).triangularView<Eigen::UnitLower>().transpose().solveInPlace(
        y_partitioned.b_i());
    } else {
      matrix_.diagonal_blocks(0).triangularView<Eigen::Lower>().transpose().solveInPlace(
        y_partitioned.b_i());
    }
  }
}

void S::AssembleFromDenseMatrix(const MatrixXd& A) {
  int c = 0;

  vector<int> global_offsets(num_blocks_, 0);
  std::partial_sum(block_column_sizes_.begin(), block_column_sizes_.end() - 1,
                   global_offsets.begin() + 1);

  for (size_t i = 0; i < block_column_sizes_.size(); i++) {
    int csize = block_column_sizes_[i];
    diagonal_blocks(i) = A.block(c, c, csize, csize);
    int r = 0;
    for (auto& row : off_diagonal_partition_[i]) {
      int r_offset = global_offsets[row.first];
      off_diagonal_blocks(i).middleCols(r, row.second) =
          A.block(r_offset, c, row.second, csize).transpose();
      r += row.second;
    }
    c += csize;
  }
}

}  // namespace conex
