#pragma once

#include <unordered_map>
#include <vector>

#include "conex/common/constraint.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/workspace.h"
#include <Eigen/Dense>
namespace conex {

struct WorkspaceEqualityConstraints {
  using DenseMatrix = Eigen::MatrixXd;

  friend int SizeOf(const WorkspaceEqualityConstraints&) { return 0; }

  friend void Initialize(WorkspaceEqualityConstraints*, double*) {}

  friend void print(const WorkspaceEqualityConstraints&) {}
  Eigen::Map<DenseMatrix, Eigen::Aligned> W{NULL, 0, 0};
};

class EqualityConstraints : public Constraint {
 public:
  EqualityConstraints(){};
  EqualityConstraints(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);

  Eigen::MatrixXd constraint_matrix() const { return A_; }
  Eigen::MatrixXd affine_term() const { return b_; }
  Eigen::MatrixXd A_;
  Eigen::MatrixXd b_;

  int number_of_variables() const override { return A_.cols(); }

  WorkspaceEqualityConstraints workspace_;
  WorkspaceEqualityConstraints* workspace() { return &workspace_; }

 private:
  Workspace do_get_workspace() override { return Workspace(workspace()); }

  int do_number_of_variables() const override { return number_of_variables(); }
};

// Lazy evaluator for the indefinite equality constraint matrix [0 A'; A 0].
class EqualityLazyMatrix : public BlockAssembler {
 public:
  EqualityLazyMatrix() = default;
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

class SupernodalAssemblerEqualities final : public SupernodalAssemblerBase {
 public:
  SupernodalAssemblerEqualities(const Eigen::MatrixXd& A,
                                const Eigen::VectorXd& b,
                                const std::vector<int>& primal_variables,
                                const std::vector<int>& dual_variables);

  int UpdateMatrix(double value, int row, int col) {
    CONEX_RETURN_ON_FAIL(row < A_.rows() && col < A_.cols(),
                         "Indices are out of bounds.");

    A_(row, col) = value;
    return CONEX_SUCCESS;
  }

  const Eigen::VectorXd& affine_term() const { return b_; }
  const Eigen::MatrixXd& constraint_matrix() const { return A_; }

  bool is_dynamic() const override { return false; }
  bool is_positive_definite() const override { return false; }

  BlockAssembler* GetBlockAssembler() override {
    lazy_.bind(&A_, static_cast<int>(primal_variables().size()));
    return &lazy_;
  }

 private:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  EqualityLazyMatrix lazy_;
};

}  // namespace conex
