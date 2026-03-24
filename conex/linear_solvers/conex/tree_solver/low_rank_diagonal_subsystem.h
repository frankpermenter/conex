#pragma once
#include <unordered_map>
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

// Adapter for a structured clique: holds raw A blocks from decomposition
// and stacks them as U = [A1^T | A2^T | ...] during UpdateData.
// One instance per structured clique.
class StackedLowRankAdapter : public KKTAssemblerToSubsystemAdapter {
 public:
  struct Block {
    Eigen::MatrixXd A;           // m_i x n_clique (original variable order)
    std::vector<int> variables;  // original variable indices for this block
  };

  StackedLowRankAdapter(std::vector<Block> blocks,
                         const std::vector<int>& clique_variables)
      : KKTAssemblerToSubsystemAdapter(nullptr),
        blocks_(std::move(blocks)),
        clique_variables_(clique_variables) {}

  std::vector<int> variables() const override { return clique_variables_; }

  void BindSubsystem(LowRankPlusDiagonalSubsystem* subsystem) {
    subsystem_ = subsystem;
  }

  void UpdateData() override {
    const auto& elim = elimination_positions();
    const auto& supernodes = subsystem_->supernodes();
    const auto& separators = subsystem_->separators();
    const int n_sn = static_cast<int>(supernodes.size());
    const int n_sep = static_cast<int>(separators.size());

    // Build elimination position -> supernode local index.
    // Supernodes are sorted by elimination position after SetVariableOrdering.
    std::unordered_map<int, int> elim_to_sn_local;
    for (int i = 0; i < n_sn; ++i) {
      elim_to_sn_local[supernodes[i]] = i;
    }
    std::unordered_map<int, int> elim_to_sep_local;
    for (int i = 0; i < n_sep; ++i) {
      elim_to_sep_local[separators[i]] = i;
    }

    // Build original variable -> elimination position.
    // elim[i] = elimination position of clique_variables_[i].
    const int n_vars = static_cast<int>(clique_variables_.size());
    std::unordered_map<int, int> orig_var_to_elim;
    for (int i = 0; i < n_vars; ++i) {
      orig_var_to_elim[clique_variables_[i]] = elim[i];
    }

    // Compute total rank.
    int total_rank = 0;
    for (const auto& block : blocks_) total_rank += block.A.rows();

    // Initialize: D = 0, U = 0, separator_rows = 0, separator_schur = 0.
    subsystem_->diagonal().setZero(n_sn);
    subsystem_->low_rank_factor().setZero(n_sn, total_rank);
    subsystem_->separator_rows().setZero();
    // Note: separator_schur_complement is NOT zeroed here — the tree solver
    // zeros the full arena in UpdateAssemblerData before calling UpdateData.

    auto& U = subsystem_->low_rank_factor();

    int col = 0;
    for (const auto& block : blocks_) {
      const int m = block.A.rows();
      const int block_cols = block.A.cols();

      // For each column j of block.A (corresponding to block.variables[j]):
      //   - If the variable maps to a supernode position: write A[:,j]^T
      //     into U row at that supernode's local index.
      //   - If separator: accumulate A_sep^T A_sn into separator_rows
      //     and A_sep^T A_sep into separator_schur_complement.

      // Build per-block mappings.
      std::vector<int> block_col_to_sn(block_cols, -1);
      std::vector<int> block_col_to_sep(block_cols, -1);
      for (int j = 0; j < block_cols; ++j) {
        int var = block.variables[j];
        auto it_elim = orig_var_to_elim.find(var);
        if (it_elim == orig_var_to_elim.end()) continue;
        int ep = it_elim->second;
        auto it_sn = elim_to_sn_local.find(ep);
        if (it_sn != elim_to_sn_local.end()) {
          block_col_to_sn[j] = it_sn->second;
        } else {
          auto it_sep = elim_to_sep_local.find(ep);
          if (it_sep != elim_to_sep_local.end()) {
            block_col_to_sep[j] = it_sep->second;
          }
        }
      }

      // Extract A_S (supernode columns) and A_P (separator columns).
      // A_S columns go into U; A_P columns contribute to separator_rows
      // and separator_schur.
      for (int j = 0; j < block_cols; ++j) {
        if (block_col_to_sn[j] >= 0) {
          // Supernode column: U(sn_local, col:col+m) = A[:,j]
          U.row(block_col_to_sn[j]).segment(col, m) =
              block.A.col(j).transpose();
        }
      }

      // separator_rows += A_P^T A_S  (n_sep x n_sn)
      // separator_schur += A_P^T A_P (n_sep x n_sep, lower triangle)
      for (int jp = 0; jp < block_cols; ++jp) {
        int sep_i = block_col_to_sep[jp];
        if (sep_i < 0) continue;
        // A_P^T A_S: sep row sep_i, supernode columns
        for (int js = 0; js < block_cols; ++js) {
          int sn_j = block_col_to_sn[js];
          if (sn_j < 0) continue;
          subsystem_->separator_rows()(sep_i, sn_j) +=
              block.A.col(jp).dot(block.A.col(js));
        }
        // A_P^T A_P: lower triangle
        for (int jp2 = jp; jp2 < block_cols; ++jp2) {
          int sep_j = block_col_to_sep[jp2];
          if (sep_j < 0) continue;
          double val = block.A.col(jp).dot(block.A.col(jp2));
          subsystem_->separator_schur_complement()(
              std::max(sep_i, sep_j), std::min(sep_i, sep_j)) += val;
        }
      }

      col += m;
    }
  }

 private:
  std::vector<Block> blocks_;
  std::vector<int> clique_variables_;  // union of all block variables
  LowRankPlusDiagonalSubsystem* subsystem_ = nullptr;
};

}  // namespace conex
