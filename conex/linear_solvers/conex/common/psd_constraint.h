// PSD constraint: Σ A_i x_i + B ≽ 0.
//
// Stores A_k as sparse matrices. No dense n²×p vectorization.
//
// Gram assembly: one-at-a-time WAW with sparse inner products.
// MultiplyA / ContributeAtranspose: sparse mat-vec via SparseMatrix iterators.
// Memory: O(n² + Σ nnz_k) vs O(n²·p) for the dense vectorized approach.

#pragma once
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cblas.h>

#include "conex/common/block_partition.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/linear_constraint.h"
#include "conex/common/psd_cone_ops.h"

namespace conex {

struct PSDEntry { int i, j; double value; };

// BlockAssembler for PSD constraints with sparse A_k.
// Owns the permuted sparse data and the WAW Gram assembly.
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

  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int m = static_cast<int>(perm.size());

    // Reindex: permuted sparse matrices and extracted entries.
    A_list_perm_.resize(m);
    entries_perm_.resize(m);
    for (int k = 0; k < m; ++k) {
      A_list_perm_[k] = (*A_list_)[perm[k]];
      auto& entries = entries_perm_[k];
      entries.clear();
      for (int outer = 0; outer < A_list_perm_[k].outerSize(); ++outer)
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_list_perm_[k], outer);
             it; ++it)
          entries.push_back({static_cast<int>(it.row()),
                             static_cast<int>(it.col()), it.value()});
    }

    // No dense A_perm_ or WA_perm_.
    A_perm_.resize(0, 0);
    WA_perm_.resize(0, 0);
    order_set_ = true;
    weights_dirty_ = true;
  }

  void update_weights() override {
    const int n = psd_n_;
    W_cache_ = Eigen::Map<const Eigen::MatrixXd>(ws_->W.data(), n, n);
    weights_dirty_ = false;
  }

  // Gram assembly: G(k,l) = trace(W A_k W · A_l).
  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;

    const int n = psd_n_;
    const int n2 = n * n;
    const double* W_data = W_cache_.data();
    if (static_cast<int>(waw_buf_.size()) != n2) waw_buf_.resize(n2);

    for (const auto& bc : it->second) {
      if (bc.lower_only) {
        for (int ri = 0; ri < bc.rows; ++ri) {
          int k = bc.q_row + ri;
          ComputeWAW(k, n, W_data, waw_buf_.data());
          for (int ci = 0; ci <= ri; ++ci) {
            int l = bc.q_row + ci;
            bc.dest[ci * bc.dest_ld + ri] += SparseIP(l, waw_buf_.data());
          }
        }
      } else {
        for (int ri = 0; ri < bc.rows; ++ri) {
          int k = bc.q_row + ri;
          ComputeWAW(k, n, W_data, waw_buf_.data());
          for (int ci = 0; ci < bc.cols; ++ci) {
            int l = bc.q_col + ci;
            bc.dest[ci * bc.dest_ld + ri] += SparseIP(l, waw_buf_.data());
          }
        }
      }
    }
  }

  // Sparse A * x: result(j*n+i) += Σ_k x_k · A_k(i,j).
  Eigen::MatrixXd SparseMultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const {
    const int n = psd_n_;
    const int n2 = n * n;
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(n2, nc);
    for (const auto& vbc : vector_blocks_) {
      auto blk = vbc.dest_is_sn
          ? supernodes.block(vbc.dest_block)
          : sep.block(vbc.dest_block, nc);
      for (int j = 0; j < vbc.length; ++j) {
        int k = vbc.q_start + j;
        for (int col = 0; col < nc; ++col) {
          double xval = blk(vbc.dest_offset + j, col);
          if (xval == 0.0) continue;
          for (const auto& e : entries_perm_[k])
            result(e.j * n + e.i, col) += e.value * xval;
        }
      }
    }
    return result;
  }

  // Sparse A^T * v: for each k, trace(A_k · mat(v)).
  void SparseContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep, int nc) const {
    const int n = psd_n_;
    for (const auto& vbc : vector_blocks_) {
      auto blk = vbc.dest_is_sn
          ? supernodes.block(vbc.dest_block)
          : sep.block(vbc.dest_block, nc);
      for (int j = 0; j < vbc.length; ++j) {
        int k = vbc.q_start + j;
        for (int col = 0; col < nc; ++col) {
          double sum = 0;
          for (const auto& e : entries_perm_[k])
            sum += e.value * V(e.j * n + e.i, col);
          blk(vbc.dest_offset + j, col) += sum;
        }
      }
    }
  }

  const std::vector<Eigen::SparseMatrix<double>>& A_list_perm() const {
    return A_list_perm_;
  }

 private:
  int psd_n_ = 0;
  int num_vars_ = 0;
  const std::vector<Eigen::SparseMatrix<double>>* A_list_ = nullptr;
  std::vector<Eigen::SparseMatrix<double>> A_list_perm_;
  std::vector<std::vector<PSDEntry>> entries_perm_;
  Eigen::MatrixXd W_cache_;
  std::vector<double> waw_buf_;

  void ComputeWAW(int k, int n, const double* W_data, double* buf) {
    std::memset(buf, 0, n * n * sizeof(double));
    for (const auto& e : entries_perm_[k])
      cblas_dger(CblasColMajor, n, n, e.value,
                 W_data + e.i * n, 1, W_data + e.j, n, buf, n);
  }

  double SparseIP(int l, const double* waw) const {
    double sum = 0;
    const int n = psd_n_;
    for (const auto& e : entries_perm_[l])
      sum += e.value * waw[e.j * n + e.i];
    return sum;
  }
};

// PSD constraint that inherits from LinearConstraint for tree solver
// compatibility, but stores sparse A_k and overrides all dense operations.
class PSDConstraint : public LinearConstraint {
 public:
  PSDConstraint(int n,
                const std::vector<Eigen::SparseMatrix<double>>& A_list,
                const Eigen::SparseMatrix<double>& B)
      : LinearConstraint(Eigen::MatrixXd(n * n, 0),
                         Eigen::VectorXd::Zero(n * n)),
        psd_n_(n), A_list_(A_list) {
    cone_ops_ = &EuclideanJordanAlgebra::psdConeOps();

    // Vectorize B into affine term (stored in base class constraint_affine_).
    constraint_affine_.resize(n * n, 1);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        constraint_affine_(j * n + i, 0) = B.coeff(i, j);

    psd_assembler_.set_psd_dim(n);
    psd_assembler_.bind_matrices(&A_list_, static_cast<int>(A_list_.size()));
  }

  int number_of_variables() const override {
    return static_cast<int>(A_list_.size());
  }

  BlockAssembler* GetBlockAssembler() override {
    psd_assembler_.bind(&workspace_, &constraint_matrix_);
    return &psd_assembler_;
  }

  const GramEvaluator& gram() const override { return psd_assembler_; }

  int num_rows() const { return psd_n_ * psd_n_; }

  // Virtual overrides — sparse, no A_perm_.
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

  void SetScaling(const Eigen::VectorXd& scaling) override {
    CONEX_DEMAND(scaling.size() == psd_n_ * psd_n_,
                 "Scaling size must match n².");
    workspace_.W = scaling;
    psd_assembler_.update_weights();
  }

  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == psd_n_ * psd_n_,
                 "Weights size must match n².");
    Eigen::Map<const Eigen::MatrixXd> W2(weights.data(), psd_n_, psd_n_);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(W2);
    Eigen::MatrixXd W_mat = eig.eigenvectors() *
        eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
        eig.eigenvectors().transpose();
    workspace_.W = Eigen::Map<Eigen::VectorXd>(W_mat.data(), weights.size());
    psd_assembler_.update_weights();
  }

  size_t RequiredArenaBytes() const override {
    return get_size_aligned(psd_n_ * psd_n_) * sizeof(double);
  }

  void BindArenaMemory(double* ptr, size_t /*bytes*/) override {
    using Map = Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>;
    int n2 = psd_n_ * psd_n_;
    new (&workspace_.W) Map(ptr, n2, 1);
    workspace_.W.setConstant(1.0);
    workspace_.n_ = n2;
    workspace_.num_vars_ = 0;
  }

 private:
  int psd_n_;
  std::vector<Eigen::SparseMatrix<double>> A_list_;
  PSDBlockAssembler psd_assembler_;
};

}  // namespace conex
