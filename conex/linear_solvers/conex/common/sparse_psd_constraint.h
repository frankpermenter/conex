// TODO: DensePSDLazyEvaluator and DensePSDSubAssembler are identical to
// DenseQuadraticTermLazyEvaluator and DenseQuadraticTermSubAssembler in
// sparse_quadratic_term.h.  Remove this duplicate and reuse the one in
// sparse_quadratic_term.h.
#pragma once
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/supernodal_assembler_base.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Lazy evaluator for a dense symmetric sub-block of Q.
// Used for per-clique sub-assemblers after Decompose.
class DensePSDLazyEvaluator : public BlockAssembler {
 public:
  void bind(const Eigen::MatrixXd* Q_block) { Q_ = Q_block; }

  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int m = static_cast<int>(perm.size());
    Q_perm_.resize(m, m);
    for (int i = 0; i < m; ++i)
      for (int j = 0; j <= i; ++j) {
        double val = (*Q_)(perm[i], perm[j]);
        Q_perm_(i, j) = val;
        Q_perm_(j, i) = val;
      }
    order_set_ = true;
  }

  void add_block(int row, int col, int rows, int cols,
                 Eigen::Ref<Eigen::MatrixXd> dest) const override {
    dest.noalias() += Q_perm_.block(row, col, rows, cols);
  }

  void add_block_lower(int pos, int size,
                       Eigen::Ref<Eigen::MatrixXd> dest) const override {
    const auto src = Q_perm_.block(pos, pos, size, size);
    for (int j = 0; j < size; ++j)
      dest.col(j).tail(size - j) += src.col(j).tail(size - j);
  }

  int rows() const override { return Q_ ? static_cast<int>(Q_->rows()) : 0; }
  int cols() const override { return Q_ ? static_cast<int>(Q_->cols()) : 0; }

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
  const Eigen::MatrixXd* Q_ = nullptr;
  Eigen::MatrixXd Q_perm_;
  bool order_set_ = false;
  std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
};

// Per-clique assembler for a dense sub-block of Q.
class DensePSDSubAssembler : public SupernodalAssemblerBase {
 public:
  DensePSDSubAssembler(Eigen::MatrixXd Q_block,
                        const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables), Q_block_(std::move(Q_block)) {
    evaluator_.bind(&Q_block_);
  }

  BlockAssembler* GetBlockAssembler() override { return &evaluator_; }
  bool is_positive_definite() const override { return true; }
  bool is_dynamic() const override { return false; }

 private:
  Eigen::MatrixXd Q_block_;
  DensePSDLazyEvaluator evaluator_;
};

// Top-level assembler for a sparse PSD matrix Q.
// Provides cliques (edges from Q's sparsity) to the clique tree builder.
// Decompose() extracts per-clique dense sub-blocks.
class SparsePSDAssembler : public SupernodalAssemblerBase {
 public:
  SparsePSDAssembler(const Eigen::SparseMatrix<double>& Q,
                      const std::vector<int>& variables);
  SparsePSDAssembler(const Eigen::MatrixXd& Q,
                      const std::vector<int>& variables);

  // get_cliques returns edges {i,j} for each Q(i,j)!=0 as 2-element vectors.
  std::vector<std::vector<int>> get_cliques() const override;

  // Decompose into per-clique sub-assemblers with dense Q sub-blocks.
  std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) override;

  bool is_positive_definite() const override { return true; }
  bool is_dynamic() const override { return false; }

  // Return the sparse matrix for edge-based clique ordering.
  const Eigen::SparseMatrix<double>* sparse_matrix() const {
    return Q_sparse_;
  }

  // Not used directly — Decompose creates sub-assemblers.
  BlockAssembler* GetBlockAssembler() override { return nullptr; }

 private:
  const Eigen::SparseMatrix<double>* Q_sparse_ = nullptr;
  const Eigen::MatrixXd* Q_dense_ = nullptr;
  bool dense_ = false;

  // Owned sub-assemblers created by Decompose.
  std::list<DensePSDSubAssembler> owned_sub_assemblers_;
};

// Solve (Q + A^T A) x = rhs.
struct SparsePSDLeastSquaresResult {
  Eigen::VectorXd x;
  double construction_time_us;
  double factor_time_us;
  double solve_time_us;
};

SparsePSDLeastSquaresResult SparsePSDLeastSquares(
    const Eigen::SparseMatrix<double>& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

SparsePSDLeastSquaresResult SparsePSDLeastSquares(
    const Eigen::MatrixXd& Q,
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& rhs);

}  // namespace conex
