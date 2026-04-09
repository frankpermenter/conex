// PSD-cone specializations of GramEvaluator and LinearConstraint.
// The Gram uses kron(W,W) via Cholesky: WA_k = vec(L^T * A_k * L).

#pragma once
#include <cmath>

#include <Eigen/Cholesky>

#include "conex/common/linear_constraint.h"

namespace conex {

// GramEvaluator for PSD cone segments.
// Overrides update_weights to compute WA_k = vec(L^T * mat(A_k) * L)
// where ws_->W stores vec(L) and W_mat = LL^T.
// Gram = (WA)^T(WA) = A^T kron(W,W) A.
class PSDGramEvaluator : public GramEvaluator {
 public:
  void set_psd_dim(int n) { psd_n_ = n; }

  void update_weights() override {
    const int n = psd_n_;
    Eigen::Map<const Eigen::MatrixXd> L(ws_->W.data(), n, n);
    const int m = A_perm_.cols();
    for (int k = 0; k < m; ++k) {
      Eigen::Map<const Eigen::MatrixXd> Ak(A_perm_.col(k).data(), n, n);
      Eigen::Map<Eigen::MatrixXd> WAk(WA_perm_.col(k).data(), n, n);
      WAk.noalias() = L.transpose() * Ak * L;
    }
    weights_dirty_ = false;
  }

 private:
  int psd_n_ = 0;
};

// LinearConstraint for PSD cone segments.
// Overrides SetWeights to Cholesky-factorize the weight matrix,
// and GetBlockAssembler / gram() to use the PSD evaluator.
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

  // weights = vec(W²) where W² is n x n PSD (same convention as nonneg:
  // caller passes the squared weight).  We compute W = sqrt(W²) via
  // eigendecomposition, then Cholesky-factorize W = LL^T and store vec(L).
  // Gram = A^T kron(W, W) A  (not kron(W², W²)).
  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    int n2 = weights.size();
    Eigen::Map<const Eigen::MatrixXd> W2(weights.data(), psd_n_, psd_n_);
    // W = symmetric square root of W².
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(W2);
    Eigen::MatrixXd W_mat = eig.eigenvectors() *
        eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
        eig.eigenvectors().transpose();
    // Cholesky of W (not W²).
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
