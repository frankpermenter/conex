#include "conex/common/psd_constraint.h"

#include <cstring>

#include <cblas.h>
#include <Eigen/Eigenvalues>

#include "conex/common/psd_cone_ops.h"

namespace conex {

// --- PSDBlockAssembler ---

void PSDBlockAssembler::ComputeWAW(int k, int n, const double* W_data,
                                    double* buf) {
  std::memset(buf, 0, n * n * sizeof(double));
  for (const auto& e : entries_perm_[k])
    cblas_dger(CblasColMajor, n, n, e.value,
               W_data + e.i * n, 1, W_data + e.j, n, buf, n);
}

double PSDBlockAssembler::SparseIP(int l, const double* waw) const {
  double sum = 0;
  const int n = psd_n_;
  for (const auto& e : entries_perm_[l])
    sum += e.value * waw[e.j * n + e.i];
  return sum;
}

void PSDBlockAssembler::set_order(const std::vector<int>& perm) {
  if (order_set_) return;
  const int m = static_cast<int>(perm.size());

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

  A_perm_.resize(0, 0);
  WA_perm_.resize(0, 0);
  order_set_ = true;
  weights_dirty_ = true;
}

void PSDBlockAssembler::update_weights() {
  const int n = psd_n_;
  W_cache_ = Eigen::Map<const Eigen::MatrixXd>(ws_->W.data(), n, n);
  weights_dirty_ = false;
}

void PSDBlockAssembler::ContributeBlocks(int clique_id) {
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

void PSDBlockAssembler::SparseMultiplyA(
    const BlockPartition& supernodes, const SeparatorScratch& sep,
    int nc, Eigen::Ref<Eigen::MatrixXd> result) const {
  const int n = psd_n_;
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
}

void PSDBlockAssembler::SparseContributeAtranspose(
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

// --- PSDConstraint ---

PSDConstraint::PSDConstraint(
    int n,
    const std::vector<Eigen::SparseMatrix<double>>& A_list,
    const Eigen::SparseMatrix<double>& B)
    : psd_n_(n), A_list_(A_list), workspace_(n * n, 0) {
  b_vec_.resize(n * n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      b_vec_(j * n + i) = B.coeff(i, j);

  psd_assembler_.set_psd_dim(n);
  psd_assembler_.bind_matrices(&A_list_, static_cast<int>(A_list_.size()));
}

const EuclideanJordanAlgebra::BarrierConeOperations* PSDConstraint::cone_ops() const {
  return &EuclideanJordanAlgebra::psdConeOps();
}

void PSDConstraint::SetScaling(const double* w, int size) {
  workspace_.W = Eigen::Map<const Eigen::VectorXd>(w, size);
  psd_assembler_.update_weights();
}

void PSDConstraint::SetWeights(const double* w, int size) {
  Eigen::Map<const Eigen::MatrixXd> W2(w, psd_n_, psd_n_);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(W2);
  Eigen::MatrixXd W_mat = eig.eigenvectors() *
      eig.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal() *
      eig.eigenvectors().transpose();
  workspace_.W = Eigen::Map<Eigen::VectorXd>(W_mat.data(), size);
  psd_assembler_.update_weights();
}

size_t PSDConstraint::RequiredArenaBytes() const {
  return get_size_aligned(psd_n_ * psd_n_) * sizeof(double);
}

void PSDConstraint::BindArenaMemory(double* ptr, size_t /*bytes*/) {
  using Map = Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>;
  int n2 = psd_n_ * psd_n_;
  new (&workspace_.W) Map(ptr, n2, 1);
  workspace_.W.setConstant(1.0);
  workspace_.n_ = n2;
  workspace_.num_vars_ = 0;
}

}  // namespace conex
