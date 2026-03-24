#pragma once
#include "conex/tree_solver/kkt_subsystem.h"
#include "conex/tree_solver/static_subsystem.h"
#include <Eigen/Dense>

namespace conex {

// Interface for providing diagonal + low-rank data to a subsystem.
// Implementations supply the diagonal d and low-rank factor U such that
// the supernode block is D + U * U^T.
class LowRankDiagonalDataSource {
 public:
  virtual ~LowRankDiagonalDataSource() = default;

  // Return the variable indices for this data source.
  virtual std::vector<int> variables() const = 0;

  // Write current diagonal and low-rank factor into the provided storage.
  // Called once per AssembleAndFactor cycle.
  //   d:  output vector of length n (supernode size)
  //   U:  output matrix of size n x r (low-rank factor)
  // The elimination_positions vector maps original variable indices to
  // elimination-order positions, so the data source can reorder if needed.
  virtual void GetData(Eigen::VectorXd& d, Eigen::MatrixXd& U,
                       const std::vector<int>& elimination_positions) const = 0;
};

// A KKT subsystem whose supernode block has the structure D + U * U^T,
// where D is diagonal (n x n) and U is low-rank (n x r).
//
// Factorization uses the matrix inversion lemma (Woodbury identity):
//   (D + U U^T)^{-1} = D^{-1} - D^{-1} U M^{-1} U^T D^{-1}
// where M = I + U^T D^{-1} U  (r x r, factored by LLT).
//
// This is efficient when r << n: factorization is O(n r^2) instead of O(n^3).
class LowRankPlusDiagonalSubsystem : public KKTSubsystem {
 public:
  // Set the diagonal and low-rank factor.  Must be called before
  // AssembleAndFactor.  Dimensions: d is (n,), U is (n, r).
  void SetData(const Eigen::VectorXd& d, const Eigen::MatrixXd& U) {
    d_ = d;
    U_ = U;
  }

  // Direct access for in-place updates.
  Eigen::VectorXd& diagonal() { return d_; }
  const Eigen::VectorXd& diagonal() const { return d_; }
  Eigen::MatrixXd& low_rank_factor() { return U_; }
  const Eigen::MatrixXd& low_rank_factor() const { return U_; }

 private:
  bool DoEliminateSupernodeColumns() override {
    const int n = static_cast<int>(d_.size());
    const int r = static_cast<int>(U_.cols());
    if (n == 0) return true;

    // D^{-1}
    d_inv_.resize(n);
    for (int i = 0; i < n; ++i) {
      if (std::abs(d_(i)) < 1e-15) {
        d_inv_(i) = 0.0;
      } else {
        d_inv_(i) = 1.0 / d_(i);
      }
    }

    // V = D^{-1} U
    V_ = d_inv_.asDiagonal() * U_;

    // M = I + U^T D^{-1} U = I + U^T V
    M_ = Eigen::MatrixXd::Identity(r, r);
    M_.noalias() += U_.transpose() * V_;

    // Factor M
    M_llt_.compute(M_);
    if (M_llt_.info() != Eigen::Success) {
      return false;
    }

    factored_ = true;
    return true;
  }

  // Schur complement: sep_schur -= S * (D + U U^T)^{-1} * S^T
  // Using Woodbury: (D + UU^T)^{-1} = D^{-1} - V M^{-1} V^T
  //   sep_schur -= S D^{-1} S^T - S V M^{-1} (S V)^T
  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0 || separator_rows().cols() == 0) return;
    const int sep = separator_rows().rows();

    auto S = separator_rows();  // sep x n

    // SV = S * D^{-1} * U  (sep x r)
    Eigen::MatrixXd SV = S * V_;

    // M^{-1} * (S V)^T  (r x sep)
    Eigen::MatrixXd MiSVt = M_llt_.solve(SV.transpose());

    // D^{-1} S^T  (n x sep)
    Eigen::MatrixXd DinvSt = d_inv_.asDiagonal() * S.transpose();

    for (int j = 0; j < sep; j++) {
      separator_schur_complement().col(j).tail(sep - j).noalias() -=
          S.bottomRows(sep - j) * DinvSt.col(j);
      separator_schur_complement().col(j).tail(sep - j).noalias() +=
          SV.bottomRows(sep - j) * MiSVt.col(j);
    }
  }

  // Solve (D + U U^T) x = y  in-place via Woodbury.
  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    if (y.rows() == 0) return;
    Eigen::MatrixXd Vty = V_.transpose() * y;
    Eigen::MatrixXd MiVty = M_llt_.solve(Vty);
    y = d_inv_.asDiagonal() * y - V_ * MiVty;
  }

  // F = I (Schur complement mode).
  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    (void)y;
  }

  bool factored_ = false;
  Eigen::VectorXd d_;       // diagonal (n)
  Eigen::MatrixXd U_;       // low-rank factor (n x r)
  Eigen::VectorXd d_inv_;   // D^{-1} (n)
  Eigen::MatrixXd V_;       // D^{-1} U (n x r)
  Eigen::MatrixXd M_;       // I + U^T D^{-1} U (r x r)
  Eigen::LLT<Eigen::MatrixXd> M_llt_;
};

// Adapter that writes diagonal + low-rank data into a
// LowRankPlusDiagonalSubsystem, bypassing the lazy evaluator path.
class LowRankDiagonalAdapter : public KKTAssemblerToSubsystemAdapter {
 public:
  LowRankDiagonalAdapter(LowRankDiagonalDataSource* source)
      : KKTAssemblerToSubsystemAdapter(nullptr), source_(source) {}

  std::vector<int> variables() const override { return source_->variables(); }

  void UpdateData() override {
    source_->GetData(subsystem_->diagonal(), subsystem_->low_rank_factor(),
                     elimination_positions());
  }

  // Called by the tree solver after creating subsystems.
  void BindSubsystem(LowRankPlusDiagonalSubsystem* subsystem) {
    subsystem_ = subsystem;
  }

 private:
  LowRankDiagonalDataSource* source_;
  LowRankPlusDiagonalSubsystem* subsystem_ = nullptr;
};

}  // namespace conex
