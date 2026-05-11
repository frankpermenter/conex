#include "conex/common/block_partition.h"

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

void StandaloneBlockPartition::Resize(int cols) {
  if (data_.rows() != total_rows_ || data_.cols() != cols) {
    data_.resize(total_rows_, cols);
  }
}

void StandaloneBlockPartition::ScatterFrom(
    Eigen::Ref<const Eigen::MatrixXd> x) {
  const int n = static_cast<int>(perm_.size());
  if (data_.rows() != total_rows_ || data_.cols() != x.cols())
    data_.resize(total_rows_, x.cols());
  data_.setZero();
  // x may have fewer rows than perm_.size() (e.g., primal cost without
  // dual variables).  Only scatter the rows that x provides.
  const int x_rows = static_cast<int>(x.rows());
  for (int i = 0; i < n && i < x_rows; ++i) {
    int ep = perm_(i);
    data_.row(ep) = x.row(i);
  }
}

void StandaloneBlockPartition::GatherInto(
    Eigen::Ref<Eigen::MatrixXd> x) const {
  const int n = static_cast<int>(perm_.size());
  for (int i = 0; i < n; ++i) {
    int ep = perm_(i);
    x.row(i) = data_.row(ep);
  }
}

}  // namespace conex
