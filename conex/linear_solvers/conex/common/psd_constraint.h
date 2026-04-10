// PSD constraint: Σ A_i x_i + B ≽ 0.
//
// Stores A_k in sparse form. Implements the tree solver interface
// without vectorizing into a dense n²×p matrix.
//
// Gram assembly: one-at-a-time WAW with sparse inner products.
// MultiplyA / ContributeAtranspose: sparse mat-vec via nonzero entries.
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
class PSDBlockAssembler : public GramEvaluator {
 public:
  void set_psd_dim(int n) { psd_n_ = n; }
  void bind_sparse(const std::vector<std::vector<PSDEntry>>* entries,
                    int num_vars) {
    sparse_entries_ = entries;
    num_vars_ = num_vars;
  }

  int rows() const override { return num_vars_; }
  int cols() const override { return num_vars_; }

  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = psd_n_;
    const int n2 = n * n;
    const int m = static_cast<int>(perm.size());
    // Build permuted sparse entries (for ContributeBlocks).
    entries_perm_.resize(m);
    for (int k = 0; k < m; ++k) {
      entries_perm_[k] = (*sparse_entries_)[perm[k]];
    }
    // Populate dense A_perm_ for base-class MultiplyA / ContributeAtranspose
    // (template methods that can't be overridden virtually).
    A_perm_.resize(n2, m);
    A_perm_.setZero();
    for (int k = 0; k < m; ++k) {
      for (const auto& e : entries_perm_[k]) {
        A_perm_(e.j * n + e.i, k) = e.value;
      }
    }
    // No WA_perm_ needed — Gram is assembled via sparse ContributeBlocks.
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

  // Sparse A * x: result = Σ_k x_k · vec(A_k).
  template <typename SepAccessor>
  Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SepAccessor& sep, int nc) const {
    const int n = psd_n_;
    const int n2 = n * n;
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(n2, nc);
    for (const auto& vbc : vector_blocks_) {
      auto blk = vbc.dest_is_sn ? supernodes.block(vbc.dest_block)
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

  // Sparse A^T * v: trace(A_k · mat(v)) for each k.
  template <typename SepAccessor>
  void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SepAccessor& sep, int nc) const {
    const int n = psd_n_;
    for (const auto& vbc : vector_blocks_) {
      auto blk = vbc.dest_is_sn ? supernodes.block(vbc.dest_block)
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

 private:
  int psd_n_ = 0;
  int num_vars_ = 0;
  const std::vector<std::vector<PSDEntry>>* sparse_entries_ = nullptr;
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
// compatibility, but bypasses all dense storage.
class PSDConstraint : public LinearConstraint {
 public:
  PSDConstraint(int n,
                const std::vector<Eigen::SparseMatrix<double>>& A_list,
                const Eigen::SparseMatrix<double>& B)
      : LinearConstraint(Eigen::MatrixXd(n * n, 0),
                         Eigen::VectorXd::Zero(n * n)),
        psd_n_(n) {
    cone_ops_ = &EuclideanJordanAlgebra::psdConeOps();

    // Extract sparse entries.
    const int p = static_cast<int>(A_list.size());
    sparse_entries_.resize(p);
    for (int k = 0; k < p; ++k) {
      for (int outer = 0; outer < A_list[k].outerSize(); ++outer)
        for (Eigen::SparseMatrix<double>::InnerIterator it(A_list[k], outer);
             it; ++it)
          sparse_entries_[k].push_back(
              {static_cast<int>(it.row()), static_cast<int>(it.col()),
               it.value()});
    }

    // Vectorize B into affine term (stored in base class constraint_affine_).
    constraint_affine_.resize(n * n, 1);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        constraint_affine_(j * n + i, 0) = B.coeff(i, j);

    psd_assembler_.set_psd_dim(n);
    psd_assembler_.bind_sparse(&sparse_entries_, p);
  }

  int number_of_variables() const override {
    return static_cast<int>(sparse_entries_.size());
  }

  BlockAssembler* GetBlockAssembler() override {
    psd_assembler_.bind(&workspace_, &constraint_matrix_);
    return &psd_assembler_;
  }

  const GramEvaluator& gram() const override { return psd_assembler_; }

  int num_rows() const { return psd_n_ * psd_n_; }

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

  // Arena: only W (n²) — no dense WA or weighted_constraints.
  size_t RequiredArenaBytes() const override {
    return get_size_aligned(psd_n_ * psd_n_) * sizeof(double);
  }

  void BindArenaMemory(double* ptr, size_t /*bytes*/) override {
    using Map = Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>;
    int n2 = psd_n_ * psd_n_;
    // Only allocate W in the arena. Redirect workspace_.W to it.
    new (&workspace_.W) Map(ptr, n2, 1);
    workspace_.W.setConstant(1.0);  // Identity diagonal for init.
    // Set n_ and num_vars_ so SizeOf doesn't break on stale values.
    workspace_.n_ = psd_n_ * psd_n_;
    workspace_.num_vars_ = 0;
  }

 private:
  int psd_n_;
  std::vector<std::vector<PSDEntry>> sparse_entries_;
  PSDBlockAssembler psd_assembler_;
};

}  // namespace conex
