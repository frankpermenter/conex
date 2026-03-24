#pragma once
#include "conex/tree_solver/kkt_subsystem.h"
#include <Eigen/Dense>

namespace conex {

// A KKT subsystem whose supernode block has the structure D + U * U^T,
// where D is diagonal (n x n) and U is low-rank (n x r).
//
// Factorization uses the matrix inversion lemma (Woodbury identity):
//   (D + U U^T)^{-1} = D^{-1} - D^{-1} U M^{-1} U^T D^{-1}
// where M = I + U^T D^{-1} U  (r x r, factored by LLT).
//
// This is efficient when r << n: factorization is O(n r^2) instead of O(n^3).
//
// The supernode_submatrix() storage from the tree solver is not used for
// factorization data; instead the diagonal and low-rank factor are stored
// separately. The separator_rows / separator_schur_complement storage is
// used normally.
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
  // Factor (D + U U^T) using the Woodbury identity.
  //   d_inv_ = D^{-1}
  //   M_ = I + U^T D^{-1} U  (r x r)
  //   M_llt_ = LLT(M_)
  //   V_ = D^{-1} U  (precomputed for solves)
  bool DoEliminateSupernodeColumns() override {
    const int n = static_cast<int>(d_.size());
    const int r = static_cast<int>(U_.cols());
    if (n == 0) return true;

    // D^{-1}
    d_inv_.resize(n);
    for (int i = 0; i < n; ++i) {
      if (std::abs(d_(i)) < 1e-15) {
        d_inv_(i) = 0.0;  // regularize zero diagonal
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
  // So: sep_schur -= S D^{-1} S^T - S V M^{-1} V^T S^T
  //              i.e. sep_schur -= S D^{-1} S^T - (S V) M^{-1} (S V)^T
  //
  // Wait: the sign.  Woodbury says inv = D^{-1} - V M^{-1} V^T, so:
  //   sep_schur -= S * (D^{-1} - V M^{-1} V^T) * S^T
  //             = S D^{-1} S^T - S V M^{-1} V^T S^T
  void DoComputeSeparatorSchurComplement() override {
    if (separator_rows().rows() == 0 || separator_rows().cols() == 0) return;
    const int sep = separator_rows().rows();

    // T1 = S * D^{-1}  (sep x n, but applied as row scaling)
    // sep_schur -= T1 * S^T = S * D^{-1} * S^T
    auto S = separator_rows();  // sep x n

    // T2 = S * V = S * D^{-1} * U  (sep x r)
    Eigen::MatrixXd SV = S * V_;

    // T3 = M^{-1} * (S V)^T  (r x sep)
    Eigen::MatrixXd MiSVt = M_llt_.solve(SV.transpose());

    // sep_schur -= S D^{-1} S^T - SV M^{-1} SV^T
    //           = S (D^{-1} S^T) - SV (M^{-1} SV^T)
    Eigen::MatrixXd DinvSt = d_inv_.asDiagonal() * S.transpose();

    for (int j = 0; j < sep; j++) {
      separator_schur_complement().col(j).tail(sep - j).noalias() -=
          S.bottomRows(sep - j) * DinvSt.col(j);
      separator_schur_complement().col(j).tail(sep - j).noalias() +=
          SV.bottomRows(sep - j) * MiSVt.col(j);
    }
  }

  // Solve (D + U U^T) x = y  in-place.
  // Using Woodbury: x = D^{-1} y - V M^{-1} V^T y
  // This is the "left factor" solve: E^{-1} = (D + UU^T)^{-1}, F = I.
  void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    if (y.rows() == 0) return;
    // temp = V^T y  (r x cols)
    Eigen::MatrixXd Vty = V_.transpose() * y;
    // temp = M^{-1} V^T y
    Eigen::MatrixXd MiVty = M_llt_.solve(Vty);
    // y = D^{-1} y - V M^{-1} V^T y
    y = d_inv_.asDiagonal() * y - V_ * MiVty;
  }

  // F = I for Schur complement mode.
  void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const override {
    (void)y;  // F^{-1} = I
  }

  bool factored_ = false;
  Eigen::VectorXd d_;       // diagonal (n)
  Eigen::MatrixXd U_;       // low-rank factor (n x r)
  Eigen::VectorXd d_inv_;   // D^{-1} (n)
  Eigen::MatrixXd V_;       // D^{-1} U (n x r)
  Eigen::MatrixXd M_;       // I + U^T D^{-1} U (r x r)
  Eigen::LLT<Eigen::MatrixXd> M_llt_;  // factorization of M
};

}  // namespace conex
