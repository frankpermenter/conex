#pragma once
#include "conex/common/kkt_solver_interface.h"
#include <Eigen/Dense>

namespace conex {

// Dense KKT solver: stores and factors a full n×n matrix.
// Uses a trivial 1-block partition (DenseBlockPartition).
// Useful as a reference implementation and for small problems.
class DenseKKTSolver : public KKTSolverBase {
 public:
  explicit DenseKKTSolver(int n) : n_(n), partition_(n) {
    M_.setZero(n, n);
  }

  // Set the matrix directly.
  void SetMatrix(const Eigen::MatrixXd& M) { M_ = M; }

  // Add to the matrix (for accumulating Q + A^T W A).
  void AddToMatrix(const Eigen::MatrixXd& delta) { M_ += delta; }

  // Zero the matrix (before re-assembly).
  void ZeroMatrix() { M_.setZero(); }

  Eigen::MatrixXd& matrix() { return M_; }
  const Eigen::MatrixXd& matrix() const { return M_; }

  int number_of_variables() const override { return n_; }

  BlockPartition& partition() override { return partition_; }
  const BlockPartition& partition() const override { return partition_; }

 private:
  void DoAssemble() override {}  // No-op; matrix set externally.

  bool DoFactor() override {
    llt_.compute(M_);
    return llt_.info() == Eigen::Success;
  }

  bool DoAssembleAndFactor() override {
    return DoFactor();
  }

  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool /*permute*/) const override {
    b = llt_.solve(b);
    // Update partition with solution.
    partition_.ScatterFrom(b);
  }

  Eigen::MatrixXd DoKKTMatrix(bool /*permute*/) const override {
    return M_;
  }

  int n_;
  Eigen::MatrixXd M_;
  Eigen::LLT<Eigen::MatrixXd> llt_;
  mutable DenseBlockPartition partition_;
};

}  // namespace conex
