#pragma once

#include <unordered_map>
#include <vector>

#include "conex/common/error_checking_macros.h"
#include "conex/common/supernodal_assembler_base.h"
#include <Eigen/Dense>
namespace conex {

// Per-clique equality constraint Cx = d.  Produces the indefinite
// saddle-point matrix [0 C'; C 0] in the primal+dual variable space.
// Parallel to LinearConstraint and QuadraticCost: owns data,
// implements BlockAssembler via nested class.
class EqualityConstraint final : public SupernodalAssemblerBase {
 public:
  EqualityConstraint(const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& b,
                     const std::vector<int>& primal_variables,
                     const std::vector<int>& dual_variables);

  BlockAssembler* GetBlockAssembler() override {
    assembler_.bind(&A_, static_cast<int>(primal_variables().size()));
    return &assembler_;
  }

  bool is_positive_definite() const { return false; }

  // Block assembler for the indefinite [0 C'; C 0] matrix.
  class Assembler : public BlockAssembler {
   public:
    Assembler() = default;
    void bind(const Eigen::MatrixXd* A, int num_primal) {
      A_ = A;
      num_primal_ = num_primal;
      n_ = num_primal + A->rows();
    }

    void set_order(const std::vector<int>& perm) override {
      if (order_set_) return;
      const int n = n_;
      Q_perm_.setZero(n, n);
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
          int oi = perm[i], oj = perm[j];
          double val = 0;
          if (oi >= num_primal_ && oj < num_primal_) {
            val = (*A_)(oi - num_primal_, oj);
          } else if (oj >= num_primal_ && oi < num_primal_) {
            val = (*A_)(oj - num_primal_, oi);
          }
          Q_perm_(i, j) = val;
        }
      }
      order_set_ = true;
    }

    int rows() const override { return n_; }
    int cols() const override { return n_; }

    bool RegisterContributions(
        int clique_id, const std::vector<int>& perm,
        const std::vector<BlockContribution>& blocks) override {
      if (!order_set_) set_order(perm);
      registered_blocks_[clique_id] = blocks;
      return true;
    }

    void ContributeBlocks(int clique_id) override {
      auto it = registered_blocks_.find(clique_id);
      if (it == registered_blocks_.end()) return;
      for (const auto& bc : it->second) {
        using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
        Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
            bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));
        if (bc.lower_only) {
          const auto src = Q_perm_.block(bc.q_row, bc.q_col, bc.rows, bc.cols);
          for (int j = 0; j < bc.cols; ++j)
            dest.col(j).tail(bc.rows - j) += src.col(j).tail(bc.rows - j);
        } else {
          dest.noalias() += Q_perm_.block(bc.q_row, bc.q_col, bc.rows, bc.cols);
        }
      }
    }

   private:
    const Eigen::MatrixXd* A_ = nullptr;
    int num_primal_ = 0;
    int n_ = 0;
    Eigen::MatrixXd Q_perm_;
    bool order_set_ = false;
    std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
  };

 private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  Assembler assembler_;
};

}  // namespace conex
