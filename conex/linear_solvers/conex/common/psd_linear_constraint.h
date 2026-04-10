// PSD-cone specializations of GramEvaluator and LinearConstraint.
//
// Sparse-aware Gram assembly: stores A_k in sparse form, computes
// WAW_i = W A_i W one at a time via rank-1 BLAS dger updates, then
// takes sparse inner products G(i,j) = Σ_{(r,c)∈A_j} a_rc · WAW_i(r,c).
//
// Memory: O(n²) temp buffer + sparse A_k (vs O(n²·p) for dense WA_perm_).
// Compute: O(p · nnz · n²) for WAW + O(p² · nnz) for inner products.

#pragma once
#include <cmath>
#include <cstring>
#include <vector>

#include <Eigen/Cholesky>

#include <cblas.h>

#include "conex/common/linear_constraint.h"

namespace conex {

class PSDGramEvaluator : public GramEvaluator {
 public:
  void set_psd_dim(int n) { psd_n_ = n; }

  // Override: skip WA_perm_ allocation (we don't use it).
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) A_perm_.col(i) = A_->col(perm[i]);
    // No WA_perm_ needed — Gram is assembled directly in ContributeBlocks.
    WA_perm_.resize(0, 0);
    weights_dirty_ = true;
    order_set_ = true;
  }

  // Cache W from workspace (stored directly, no Cholesky).
  void update_weights() override {
    if (sparse_entries_.empty() && A_perm_.cols() > 0) {
      ExtractSparseEntries();
    }
    const int n = psd_n_;
    W_cache_ = Eigen::Map<const Eigen::MatrixXd>(ws_->W.data(), n, n);
    weights_dirty_ = false;
  }

  // Override: compute Gram blocks via one-at-a-time WAW with sparse A_k.
  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;

    const int n = psd_n_;
    const int n2 = n * n;
    const double* W_data = W_cache_.data();

    // Temp buffer for WAW_i (n×n, reused per variable).
    if (static_cast<int>(waw_buf_.size()) != n2) waw_buf_.resize(n2);

    for (const auto& bc : it->second) {
      if (bc.lower_only) {
        // Diagonal block: G(i,j) for i,j in [q_row, q_row+rows).
        for (int ri = 0; ri < bc.rows; ++ri) {
          int k = bc.q_row + ri;
          ComputeWAW(k, n, W_data, waw_buf_.data());
          // Inner products with A_j for j <= i (lower triangle).
          for (int ci = 0; ci <= ri; ++ci) {
            int l = bc.q_row + ci;
            bc.dest[ci * bc.dest_ld + ri] += SparseInnerProduct(l, waw_buf_.data());
          }
        }
      } else {
        // Off-diagonal block.
        for (int ri = 0; ri < bc.rows; ++ri) {
          int k = bc.q_row + ri;
          ComputeWAW(k, n, W_data, waw_buf_.data());
          for (int ci = 0; ci < bc.cols; ++ci) {
            int l = bc.q_col + ci;
            bc.dest[ci * bc.dest_ld + ri] += SparseInnerProduct(l, waw_buf_.data());
          }
        }
      }
    }
  }

 private:
  int psd_n_ = 0;
  Eigen::MatrixXd W_cache_;        // W = LL^T (n×n)
  std::vector<double> waw_buf_;    // temp n×n buffer for WAW_i

  struct Entry { int i, j; double value; };
  std::vector<std::vector<Entry>> sparse_entries_;

  // WAW_k = Σ_{(i,j)∈A_k} a_ij · W(:,i) · W(j,:) via BLAS dger.
  void ComputeWAW(int k, int n, const double* W_data, double* buf) {
    std::memset(buf, 0, n * n * sizeof(double));
    for (const auto& e : sparse_entries_[k]) {
      // W(:,i) starts at W_data + i*n, stride 1.
      // W(j,:) starts at W_data + j, stride n.
      cblas_dger(CblasColMajor, n, n, e.value,
                 W_data + e.i * n, 1,
                 W_data + e.j, n,
                 buf, n);
    }
  }

  // G(k,l) = Σ_{(r,c)∈A_l} a_rc · WAW_k(r,c).
  double SparseInnerProduct(int l, const double* waw) const {
    double sum = 0;
    const int n = psd_n_;
    for (const auto& e : sparse_entries_[l]) {
      sum += e.value * waw[e.j * n + e.i];  // column-major: (i,j) → j*n+i
    }
    return sum;
  }

  void ExtractSparseEntries() {
    const int n = psd_n_;
    const int n2 = n * n;
    const int m = A_perm_.cols();
    sparse_entries_.resize(m);
    for (int k = 0; k < m; ++k) {
      const double* col = A_perm_.col(k).data();
      auto& entries = sparse_entries_[k];
      entries.clear();
      for (int p = 0; p < n2; ++p) {
        if (col[p] != 0.0) {
          entries.push_back({p % n, p / n, col[p]});
        }
      }
    }
  }
};

// LinearConstraint for PSD cone segments.
class PSDLinearConstraint : public LinearConstraint {
 public:
  PSDLinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                      const Eigen::MatrixXd& constraint_affine)
      : LinearConstraint(constraint_matrix, constraint_affine) {
    int n2 = constraint_matrix.rows();
    int n = static_cast<int>(std::round(std::sqrt(static_cast<double>(n2))));
    psd_gram_.set_psd_dim(n);
    psd_n_ = n;
  }

  BlockAssembler* GetBlockAssembler() override {
    psd_gram_.bind(&workspace_, &constraint_matrix_);
    return &psd_gram_;
  }

  const GramEvaluator& gram() const { return psd_gram_; }

  // weights = vec(W²). Compute W = sqrt(W²), store directly.
  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    Eigen::Map<const Eigen::MatrixXd> W2(weights.data(), psd_n_, psd_n_);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(W2);
    Eigen::MatrixXd W_mat = eig.eigenvectors() *
        eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
        eig.eigenvectors().transpose();
    workspace_.W = Eigen::Map<Eigen::VectorXd>(W_mat.data(), weights.size());
    psd_gram_.update_weights();
  }

  // scaling = vec(W). Store directly — no Cholesky needed.
  void SetScaling(const Eigen::VectorXd& scaling) override {
    CONEX_DEMAND(scaling.size() == constraint_matrix_.rows(),
                 "Scaling vector size must match number of constraint rows.");
    workspace_.W = scaling;
    psd_gram_.update_weights();
  }

 private:
  PSDGramEvaluator psd_gram_;
  int psd_n_ = 0;
};

}  // namespace conex
