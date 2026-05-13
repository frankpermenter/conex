#include "conex/common/block_partition.h"
#include "conex/common/arena.h"
#include <cstring>

namespace conex {

// --- DenseBlockPartition ---

void DenseBlockPartition::Resize(int cols) {
  if (data_.rows() != n_ || data_.cols() != cols) {
    data_.resize(n_, cols);
  }
}

void DenseBlockPartition::ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> x) {
  if (data_.rows() != x.rows() || data_.cols() != x.cols()) {
    data_.resize(x.rows(), x.cols());
  }
  data_ = x;
}

void DenseBlockPartition::GatherInto(Eigen::Ref<Eigen::MatrixXd> x) const {
  x = data_;
}

// --- StandaloneBlockPartition ---

void StandaloneBlockPartition::BindArena(Arena& arena, int cols) {
  size_t bytes = static_cast<size_t>(total_rows_) * cols * sizeof(double);
  arena_data_ = static_cast<double*>(arena.Alloc(bytes));
  std::memset(arena_data_, 0, bytes);
  arena_cols_ = cols;
}

void StandaloneBlockPartition::Resize(int cols) {
  if (arena_data_) return;  // arena-backed: already sized at BindArena
  if (data_.rows() != total_rows_ || data_.cols() != cols) {
    data_.resize(total_rows_, cols);
  }
}

void StandaloneBlockPartition::SetZero() {
  if (arena_data_) {
    std::memset(arena_data_, 0,
                static_cast<size_t>(total_rows_) * arena_cols_ * sizeof(double));
  } else {
    data_.setZero();
  }
}

void StandaloneBlockPartition::ScatterFrom(
    Eigen::Ref<const Eigen::MatrixXd> x) {
  const int n = static_cast<int>(perm_.size());
  const int nc = arena_data_ ? arena_cols_ : static_cast<int>(x.cols());
  if (!arena_data_) {
    if (data_.rows() != total_rows_ || data_.cols() != x.cols())
      data_.resize(total_rows_, x.cols());
  }
  SetZero();
  const int x_rows = static_cast<int>(x.rows());
  if (arena_data_) {
    Eigen::Map<Eigen::MatrixXd> m(arena_data_, total_rows_, arena_cols_);
    for (int i = 0; i < n && i < x_rows; ++i)
      m.row(perm_[i]) = x.row(i).head(arena_cols_);
  } else {
    for (int i = 0; i < n && i < x_rows; ++i)
      data_.row(perm_[i]) = x.row(i);
  }
}

void StandaloneBlockPartition::GatherInto(
    Eigen::Ref<Eigen::MatrixXd> x) const {
  const int n = static_cast<int>(perm_.size());
  if (arena_data_) {
    Eigen::Map<const Eigen::MatrixXd> m(arena_data_, total_rows_, arena_cols_);
    for (int i = 0; i < n; ++i)
      x.row(i).head(arena_cols_) = m.row(perm_[i]);
  } else {
    for (int i = 0; i < n; ++i)
      x.row(i) = data_.row(perm_[i]);
  }
}

}  // namespace conex
