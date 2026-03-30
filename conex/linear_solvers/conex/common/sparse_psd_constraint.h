#pragma once
#include <list>
#include <memory>
#include <vector>

#include "conex/common/sparse_quadratic_term.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Aliases: DensePSDLazyEvaluator and DensePSDSubAssembler are identical
// to DenseQuadraticTermLazyEvaluator and DenseQuadraticTermSubAssembler.
using DensePSDLazyEvaluator = DenseQuadraticTermLazyEvaluator;
using DensePSDSubAssembler = DenseQuadraticTermSubAssembler;

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
