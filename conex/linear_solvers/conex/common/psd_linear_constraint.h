// PSD-cone specializations of GramEvaluator and LinearConstraint.
// The Gram uses kron(W,W) via Cholesky: WA_k = vec(L^T * A_k * L).
//
// Sparse optimization: A_k is typically very sparse. Instead of O(n³) dense
// multiplies we use rank-1 BLAS dger updates: O(nnz · n²) per variable.
//
// svec optimization: since L^T A_k L is symmetric, WA_perm_ stores the
// symmetric vectorization (lower triangle, off-diag scaled by √2).
// Row count drops from n² to n(n+1)/2, halving the DSYRK cost.

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

  // Override set_order to resize WA_perm_ to svec dimension.
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) A_perm_.col(i) = A_->col(perm[i]);
    // svec dimension: n_psd * (n_psd + 1) / 2.
    const int svec_dim = psd_n_ * (psd_n_ + 1) / 2;
    WA_perm_.resize(svec_dim, m);
    weights_dirty_ = true;
    order_set_ = true;
  }

  void update_weights() override {
    if (sparse_entries_.empty() && A_perm_.cols() > 0) {
      ExtractSparseEntries();
    }

    const int n = psd_n_;
    const int m = A_perm_.cols();
    const double* L_data = ws_->W.data();
    const double sqrt2 = std::sqrt(2.0);

    // Temp buffer for full n×n result (reused across columns).
    if (temp_n2_.size() != n * n)
      temp_n2_.resize(n * n);

    for (int k = 0; k < m; ++k) {
      double* buf = temp_n2_.data();
      const auto& entries = sparse_entries_[k];

      // Accumulate rank-1 updates into full n×n buffer.
      std::memset(buf, 0, n * n * sizeof(double));
      for (const auto& e : entries) {
        cblas_dger(CblasColMajor, n, n, e.value,
                   L_data + e.i, n, L_data + e.j, n, buf, n);
      }

      // Extract svec: lower triangle with √2 scaling on off-diagonal.
      double* dst = WA_perm_.col(k).data();
      int idx = 0;
      for (int col = 0; col < n; ++col) {
        // Diagonal entry.
        dst[idx++] = buf[col * n + col];
        // Below-diagonal entries (scaled by √2).
        for (int row = col + 1; row < n; ++row) {
          dst[idx++] = sqrt2 * buf[col * n + row];
        }
      }
    }
    weights_dirty_ = false;
  }

 private:
  int psd_n_ = 0;
  std::vector<double> temp_n2_;  // n×n scratch buffer

  struct Entry { int i, j; double value; };
  std::vector<std::vector<Entry>> sparse_entries_;

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

  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    int n2 = weights.size();
    Eigen::Map<const Eigen::MatrixXd> W2(weights.data(), psd_n_, psd_n_);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(W2);
    Eigen::MatrixXd W_mat = eig.eigenvectors() *
        eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
        eig.eigenvectors().transpose();
    Eigen::LLT<Eigen::MatrixXd> llt(W_mat);
    Eigen::MatrixXd L = llt.matrixL();
    workspace_.W = Eigen::Map<Eigen::VectorXd>(L.data(), n2);
    psd_gram_.update_weights();
  }

  void SetScaling(const Eigen::VectorXd& scaling) override {
    CONEX_DEMAND(scaling.size() == constraint_matrix_.rows(),
                 "Scaling vector size must match number of constraint rows.");
    int n2 = scaling.size();
    Eigen::Map<const Eigen::MatrixXd> W_mat(scaling.data(), psd_n_, psd_n_);
    Eigen::LLT<Eigen::MatrixXd> llt(W_mat);
    Eigen::MatrixXd L = llt.matrixL();
    workspace_.W = Eigen::Map<Eigen::VectorXd>(L.data(), n2);
    psd_gram_.update_weights();
  }

 private:
  PSDGramEvaluator psd_gram_;
  int psd_n_ = 0;
};

}  // namespace conex
