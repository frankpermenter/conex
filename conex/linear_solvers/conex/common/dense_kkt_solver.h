#pragma once
#include "conex/common/kkt_solver_interface.h"
#include <Eigen/Dense>

namespace conex {

// Dense KKT solver: stores and factors a full n×n matrix.
// Supports both PD (LLT) and indefinite (LDLT) systems.
// Uses the base class's DenseBlockPartition (trivial 1-block partition).
class DenseKKTSolver : public KKTSolverBase {
 public:
  explicit DenseKKTSolver(int n) : n_(n), partition_(n) {
    M_.setZero(n, n);
  }

  void SetMatrix(const Eigen::MatrixXd& M) { M_ = M; }
  void AddToMatrix(const Eigen::MatrixXd& delta) { M_ += delta; }
  void ZeroMatrix() { M_.setZero(); }

  Eigen::MatrixXd& matrix() { return M_; }
  const Eigen::MatrixXd& matrix() const { return M_; }

  int number_of_variables() const override { return n_; }

  BlockPartition& partition() override { return partition_; }
  const BlockPartition& partition() const override { return partition_; }

 private:
  void DoAssemble() override {}

  bool DoFactor() override {
    ldlt_.compute(M_);
    return ldlt_.info() == Eigen::Success;
  }

  bool DoAssembleAndFactor() override { return DoFactor(); }

  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool /*permute*/) const override {
    b = ldlt_.solve(b);
    partition_.ScatterFrom(b);
  }

  Eigen::MatrixXd DoKKTMatrix(bool /*permute*/) const override {
    return M_;
  }

  int n_;
  Eigen::MatrixXd M_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_;
  mutable DenseBlockPartition partition_;
};

}  // namespace conex
