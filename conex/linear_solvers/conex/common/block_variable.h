#pragma once
#include <Eigen/Dense>
#include <memory>

#include "conex/common/block_partition.h"

namespace conex {

// A vector stored in block-partitioned form.  Created via
// solver.MakeBlockVariable() to get a partition that matches the
// solver's block structure (supernodes/separators).
//
// Supports scatter/gather to move between block and dense forms,
// and element access for inspection.
class BlockVariable {
 public:
  BlockVariable() = default;
  explicit BlockVariable(std::unique_ptr<BlockPartition> partition)
      : partition_(std::move(partition)) {
    partition_->Resize(1);
    partition_->SetZero();
  }

  // Scatter a dense vector into block storage.
  void ScatterFrom(const Eigen::VectorXd& x) {
    partition_->ScatterFrom(x);
  }

  // Gather block storage into a dense vector.
  Eigen::VectorXd Gather() const {
    Eigen::VectorXd x(partition_->num_variables());
    partition_->GatherInto(x);
    return x;
  }

  // Set all blocks to zero.
  void SetZero() { partition_->SetZero(); }

  // Number of scalar variables.
  int size() const { return partition_->num_variables(); }

  // Access the underlying partition (for block-level operations).
  BlockPartition& partition() { return *partition_; }
  const BlockPartition& partition() const { return *partition_; }

 private:
  std::unique_ptr<BlockPartition> partition_;
};

}  // namespace conex
