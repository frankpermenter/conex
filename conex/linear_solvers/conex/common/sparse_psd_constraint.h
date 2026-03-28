// TODO: DensePSDLazyEvaluator and DensePSDSubAssembler are identical to
// DenseQuadraticTermLazyEvaluator and DenseQuadraticTermSubAssembler in
// sparse_quadratic_term.h.  Remove this duplicate and reuse the one in
// sparse_quadratic_term.h.
#pragma once
#include <list>
#include <memory>
#include <vector>

#include "conex/common/supernodal_assembler_base.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Lazy evaluator for a dense symmetric sub-block of Q.
// Used for per-clique sub-assemblers after Decompose.
class DensePSDLazyEvaluator : public LazySymmetricMatrix {
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

 private:
  const Eigen::MatrixXd* Q_ = nullptr;
  Eigen::MatrixXd Q_perm_;
  bool order_set_ = false;
};

// Per-clique assembler for a dense sub-block of Q.
class DensePSDSubAssembler : public SupernodalAssemblerBase {
 public:
  DensePSDSubAssembler(Eigen::MatrixXd Q_block,
                        const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables), Q_block_(std::move(Q_block)) {
    evaluator_.bind(&Q_block_);
  }

  LazySymmetricMatrix* GetLazyEvaluator() override { return &evaluator_; }
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
  LazySymmetricMatrix* GetLazyEvaluator() override { return nullptr; }

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
