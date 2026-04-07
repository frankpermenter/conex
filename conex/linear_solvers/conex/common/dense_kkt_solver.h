#pragma once
#include "conex/common/kkt_solver_interface.h"
#include <Eigen/Dense>

namespace conex {

// Dense KKT solver: stores and factors a full n×n matrix.
// Supports both PD (LLT) and indefinite (LDLT) systems.
// Uses the base class's DenseBlockPartition (trivial 1-block partition).
//
// Optional constraint-based interface: call SetConstraintData(Q, A, b)
// to enable the generic KKTSolverBase methods (MultiplyA, AccumulateQx,
// SetWeights, etc.).  The solver then builds M = Q + A^T diag(w) A
// at each AssembleAndFactor.
class DenseKKTSolver : public KKTSolverBase {
 public:
  explicit DenseKKTSolver(int n) : n_(n) {
    M_.setZero(n, n);
  }

  void SetMatrix(const Eigen::MatrixXd& M) { M_ = M; }
  void AddToMatrix(const Eigen::MatrixXd& delta) { M_ += delta; }
  void ZeroMatrix() { M_.setZero(); }

  Eigen::MatrixXd& matrix() { return M_; }
  const Eigen::MatrixXd& matrix() const { return M_; }

  // Set constraint data for the generic interface.
  // After this call, SetWeights + AssembleAndFactor rebuilds M = Q + A^T W A.
  void SetConstraintData(const Eigen::MatrixXd& Q,
                         const Eigen::MatrixXd& A,
                         const Eigen::VectorXd& b) {
    Q_ = Q;
    A_ = A;
    b_ = b;
    has_constraints_ = true;
    weights_ = Eigen::VectorXd::Ones(A.rows());
  }

  int number_of_variables() const override { return n_; }

  // --- Generic KKTSolverBase overrides ---

  RowSpace MakeRowSpace(int cols = 1) override {
    RowSpace rs;
    if (has_constraints_) rs.data.setZero(A_.rows(), cols);
    return rs;
  }

  void MultiplyA(const SolverRHS& x, RowSpace& out) override {
    if (!has_constraints_) return;
    int nc = out.cols();
    Eigen::MatrixXd xv(n_, nc);
    x.supernodes->GatherInto(xv);
    out.data = A_ * xv;
  }

  void AccumulateAtranspose(const RowSpace& v, SolverRHS& rhs) override {
    if (!has_constraints_) return;
    Eigen::MatrixXd atv = A_.transpose() * v.data;
    int nc = atv.cols();
    Eigen::MatrixXd cur(n_, nc);
    rhs.supernodes->GatherInto(cur);
    cur += atv;
    rhs.supernodes->ScatterFrom(cur);
    rhs.blocks_fully_gathered = true;
  }

  void AccumulateQx(const SolverRHS& x, SolverRHS& rhs) override {
    if (!has_constraints_) return;
    Eigen::VectorXd xv(n_);
    x.supernodes->GatherInto(xv);
    Eigen::VectorXd qx = Q_ * xv;
    Eigen::MatrixXd cur(n_, 1);
    rhs.supernodes->GatherInto(cur);
    cur += qx;
    rhs.supernodes->ScatterFrom(cur);
    rhs.blocks_fully_gathered = true;
  }

  void SetWeights(const RowSpace& w) override {
    if (!has_constraints_) return;
    weights_ = w.data.col(0);
    M_ = Q_ + A_.transpose() * weights_.asDiagonal() * A_;
  }

  RowSpace GetAffineTerm() override {
    RowSpace rs;
    if (has_constraints_) rs.data = b_.reshaped(b_.size(), 1);
    return rs;
  }

 private:
  void DoAssemble() override {
    if (has_constraints_) {
      M_ = Q_ + A_.transpose() * weights_.asDiagonal() * A_;
    }
  }

  bool DoFactor() override {
    ldlt_.compute(M_);
    return ldlt_.info() == Eigen::Success;
  }

  bool DoAssembleAndFactor() override {
    DoAssemble();
    return DoFactor();
  }

  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool /*permute*/) const override {
    b = ldlt_.solve(b);
  }

  Eigen::MatrixXd DoKKTMatrix(bool /*permute*/) const override {
    return M_;
  }

  int n_;
  Eigen::MatrixXd M_;
  Eigen::LDLT<Eigen::MatrixXd> ldlt_;

  // Constraint data (optional).
  bool has_constraints_ = false;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::VectorXd weights_;
};

}  // namespace conex
