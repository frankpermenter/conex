// PSD constraint: Σ A_i x_i + B ≽ 0.
//
// Stores A_k as sparse matrices. No dense n²×p vectorization.
//
// Gram assembly: one-at-a-time WAW with sparse inner products.
// MultiplyA / ContributeAtranspose: sparse mat-vec via nonzero entries.
// Memory: O(n² + Σ nnz_k) vs O(n²·p) for the dense vectorized approach.

#pragma once
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/cone_constraint.h"
#include "conex/common/linear_constraint.h"
#include "conex/common/linear_workspace.h"

namespace conex {

struct PSDEntry { int i, j; double value; };

// BlockAssembler for PSD constraints with sparse A_k.
class PSDBlockAssembler : public GramEvaluator {
 public:
  void set_psd_dim(int n) { psd_n_ = n; }

  void bind_matrices(const std::vector<Eigen::SparseMatrix<double>>* A_list,
                     int num_vars) {
    A_list_ = A_list;
    num_vars_ = num_vars;
  }

  int rows() const override { return num_vars_; }
  int cols() const override { return num_vars_; }

  void set_order(const std::vector<int>& perm) override;
  void update_weights() override;
  void ContributeBlocks(int clique_id) override;

  Eigen::MatrixXd SparseMultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const;

  void SparseContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep, int nc) const;

 private:
  int psd_n_ = 0;
  int num_vars_ = 0;
  const std::vector<Eigen::SparseMatrix<double>>* A_list_ = nullptr;
  std::vector<Eigen::SparseMatrix<double>> A_list_perm_;
  std::vector<std::vector<PSDEntry>> entries_perm_;
  Eigen::MatrixXd W_cache_;
  std::vector<double> waw_buf_;

  void ComputeWAW(int k, int n, const double* W_data, double* buf);
  double SparseIP(int l, const double* waw) const;
};

// PSD constraint: stores sparse A_k, owns its own workspace.
// Inherits from ConeConstraint — no LinearConstraint baggage.
class PSDConstraint : public ConeConstraint {
 public:
  PSDConstraint(int n,
                const std::vector<Eigen::SparseMatrix<double>>& A_list,
                const Eigen::SparseMatrix<double>& B);

  int number_of_variables() const override {
    return static_cast<int>(A_list_.size());
  }

  BlockAssembler* GetBlockAssembler() override {
    psd_assembler_.bind(&workspace_, nullptr);
    return &psd_assembler_;
  }

  Eigen::MatrixXd affine_term() const override { return b_vec_; }
  int num_rows() const override { return psd_n_ * psd_n_; }
  const EuclideanJordanAlgebra::BarrierConeOperations* cone_ops() const override;

  Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const override {
    return psd_assembler_.SparseMultiplyA(supernodes, sep, nc);
  }

  void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep,
      int nc) const override {
    psd_assembler_.SparseContributeAtranspose(V, supernodes, sep, nc);
  }

  void SetScaling(const Eigen::VectorXd& scaling) override;
  void SetWeights(const Eigen::VectorXd& weights) override;

  size_t RequiredArenaBytes() const override;
  void BindArenaMemory(double* ptr, size_t bytes) override;

 private:
  int psd_n_;
  std::vector<Eigen::SparseMatrix<double>> A_list_;
  Eigen::VectorXd b_vec_;
  WorkspaceLinear workspace_;
  PSDBlockAssembler psd_assembler_;
};

}  // namespace conex
