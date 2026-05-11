#pragma once
#include <Eigen/Dense>
#include <memory>

#include "conex/common/block_partition.h"

namespace conex {

// A matrix stored in block-partitioned form.  Created via
// solver.MakeBlockVariable() to get a partition that matches the
// solver's block structure (supernodes/separators).
//
// Supports single-column (vector) and multi-column (batched RHS) use.
class BlockVariable {
 public:
  BlockVariable() = default;
  explicit BlockVariable(std::unique_ptr<BlockPartition> partition,
                         int cols = 1)
      : partition_(std::move(partition)) {
    partition_->Resize(cols);
    partition_->SetZero();
  }

  // Scatter a dense matrix into block storage.
  void ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> x) {
    if (partition_->cols() != x.cols()) partition_->Resize(x.cols());
    partition_->ScatterFrom(x);
  }

  // Gather block storage into a dense matrix.
  Eigen::MatrixXd Gather() const {
    Eigen::MatrixXd x(partition_->num_variables(), partition_->cols());
    partition_->GatherInto(x);
    return x;
  }

  // Set all blocks to zero.
  void SetZero() { partition_->SetZero(); }

  // Number of scalar variables (rows).
  int size() const { return partition_->num_variables(); }

  // Number of columns.
  int cols() const { return partition_->cols(); }

  // Dot product with a dense vector.
  double dot(const Eigen::VectorXd& v) const {
    return Gather().col(0).dot(v);
  }

  // Axpy: this += alpha * other (block-wise).
  void AddScaled(double alpha, const BlockVariable& other) {
    int nb = partition_->num_blocks();
    for (int k = 0; k < nb; ++k) {
      partition_->block(k) += alpha * other.partition().block(k);
    }
  }

  // Access the underlying partition (for block-level operations).
  BlockPartition& partition() { return *partition_; }
  const BlockPartition& partition() const { return *partition_; }

 private:
  std::unique_ptr<BlockPartition> partition_;
};

}  // namespace conex
