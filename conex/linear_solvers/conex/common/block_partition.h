#pragma once
#include <Eigen/Dense>

namespace conex {

// Abstract block partition of a vector.  After Solve(), the solver's
// partition contains the solution distributed across blocks.  Algorithms
// use this to compute residuals and products in block space without
// knowing the solver's internal structure.
//
// A tree solver partitions by supernodes/separators.
// A dense solver has one block containing all variables.
// A block-diagonal solver has one block per diagonal block.
class BlockPartition {
 public:
  virtual ~BlockPartition() = default;

  // Number of blocks in the partition.
  virtual int num_blocks() const = 0;

  // Size (number of rows) of block k.
  virtual int block_size(int k) const = 0;

  // Total number of variables across all blocks.
  virtual int num_variables() const = 0;

  // Number of columns (typically 1 for vectors, >1 for multi-RHS).
  virtual int cols() const = 0;

  // Ensure the partition is allocated for the given number of columns.
  virtual void Resize(int cols) = 0;

  // Set all blocks to zero.
  virtual void SetZero() = 0;

  // Scatter a vector in original variable order into block storage.
  virtual void ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> x) = 0;

  // Gather from block storage into a vector in original variable order.
  virtual void GatherInto(Eigen::Ref<Eigen::MatrixXd> x) const = 0;

  // Access block k (mutable).
  virtual Eigen::Ref<Eigen::MatrixXd> block(int k) = 0;

  // Access block k (const).
  virtual Eigen::Ref<const Eigen::MatrixXd> block(int k) const = 0;
};

// Trivial partition: one block containing all variables.
// Scatter/gather are identity (copy).
class DenseBlockPartition : public BlockPartition {
 public:
  DenseBlockPartition() = default;
  explicit DenseBlockPartition(int n) : n_(n) {}

  int num_blocks() const override { return 1; }
  int block_size(int /*k*/) const override { return n_; }
  int num_variables() const override { return n_; }
  int cols() const override { return data_.cols(); }

  void Resize(int cols) override {
    if (data_.rows() != n_ || data_.cols() != cols) {
      data_.resize(n_, cols);
    }
  }

  void SetZero() override { data_.setZero(); }

  void ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> x) override {
    if (data_.rows() != x.rows() || data_.cols() != x.cols()) {
      data_.resize(x.rows(), x.cols());
    }
    data_ = x;
  }

  void GatherInto(Eigen::Ref<Eigen::MatrixXd> x) const override {
    x = data_;
  }

  Eigen::Ref<Eigen::MatrixXd> block(int /*k*/) override { return data_; }
  Eigen::Ref<const Eigen::MatrixXd> block(int /*k*/) const override {
    return data_;
  }

 private:
  int n_ = 0;
  Eigen::MatrixXd data_;
};

}  // namespace conex
