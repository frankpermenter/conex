#pragma once
#include <unordered_map>

#include <Eigen/Dense>

#include "conex/common/block_partition.h"
#include "conex/common/blas_wrapper.h"
#include "conex/common/cone_constraint.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/linear_workspace.h"
#include "conex/common/tree_rhs.h"

namespace conex {

class GramEvaluator : public BlockAssembler {
 public:
  GramEvaluator() = default;
  void bind(WorkspaceLinear* ws, const Eigen::MatrixXd* A) {
    ws_ = ws;
    A_ = A;
  }

  // Permute columns of A.  Weight computation is deferred.
  void set_order(const std::vector<int>& perm) override {
    if (order_set_) return;
    const int n = A_->rows();
    const int m = static_cast<int>(perm.size());
    A_perm_.resize(n, m);
    for (int i = 0; i < m; ++i) A_perm_.col(i) = A_->col(perm[i]);
    WA_perm_.resize(n, m);
    weights_dirty_ = true;
    order_set_ = true;
  }

  // Recompute WA_perm_ from weights stored in ws_->W.
  // Default: WA_perm_ = diag(W) * A_perm_.
  //   Gram = (WA)^T(WA) = A^T diag(W²) A.
  // Subclasses (e.g. PSD) override for different weight structures.
  virtual void update_weights() {
    WA_perm_.noalias() = ws_->W.asDiagonal() * A_perm_;
    weights_dirty_ = false;
  }

  int rows() const override { return ws_->num_vars_; }
  int cols() const override { return ws_->num_vars_; }

  void set_sn_count(int c) override { sn_count_ = c; }

  bool RegisterContributions(
      int clique_id, const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) override {
    if (!order_set_) set_order(perm);
    registered_blocks_[clique_id] = blocks;
    return true;
  }

  void RegisterVectorContributions(
      const std::vector<VectorBlockContribution>& blocks) override {
    vector_blocks_ = blocks;
  }


  // Set scale factor for the trace inner product (1 for nonneg/PSD, 2 for SOC).
  void set_atranspose_scale(double s) { atranspose_scale_ = s; }

  // Accumulate A_perm_^T * V into supernode blocks and separator scratch.
  // V is (m x batch).  Writes directly to parent destinations.
  // Scaled by atranspose_scale_ (2 for SOC trace inner product).
  template <typename SepAccessor>
  void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SepAccessor& sep, int nc) const {
    for (const auto& vbc : vector_blocks_) {
      auto atv = atranspose_scale_ *
          A_perm_.middleCols(vbc.q_start, vbc.length).transpose() * V;
      if (vbc.dest_is_sn) {
        auto blk = supernodes.block(vbc.dest_block);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sn write out of bounds");
        blk.middleRows(vbc.dest_offset, vbc.length) += atv;
      } else {
        auto blk = sep.block(vbc.dest_block, nc);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sep write out of bounds");
        blk.middleRows(vbc.dest_offset, vbc.length) += atv;
      }
    }
  }

  // Compute A_perm_ * x by reading from supernode blocks and separator scratch.
  template <typename SepAccessor>
  Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SepAccessor& sep, int nc) const {
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(A_perm_.rows(), nc);
    for (const auto& vbc : vector_blocks_) {
      auto cols = A_perm_.middleCols(vbc.q_start, vbc.length);
      if (vbc.dest_is_sn) {
        auto blk = supernodes.block(vbc.dest_block);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sn read out of bounds");
        result.noalias() += cols * blk.middleRows(vbc.dest_offset, vbc.length);
      } else {
        auto blk = sep.block(vbc.dest_block, nc);
        CONEX_DEMAND(vbc.dest_offset + vbc.length <= blk.rows(),
                     "VBC sep read out of bounds");
        result.noalias() += cols * blk.middleRows(vbc.dest_offset, vbc.length);
      }
    }
    return result;
  }

  void ContributeBlocks(int clique_id) override {
    ensure_weights_fresh();
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;
    const int m = WA_perm_.rows();  // number of constraint rows
    for (const auto& bc : it->second) {
      if (bc.lower_only) {
        // Diagonal block: C += WA^T * WA (lower triangle).
        const double* A_ptr = WA_perm_.data() + bc.q_row * m;
        if (!blas::Dsyrk(bc.rows, m, 1.0, A_ptr, m, bc.dest, bc.dest_ld)) {
          using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
          Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
              bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));
          dest.selfadjointView<Eigen::Lower>().rankUpdate(
              WA_perm_.middleCols(bc.q_row, bc.rows).transpose());
        }
      } else {
        // Off-diagonal block: C += WA_row^T * WA_col.
        const double* A_row = WA_perm_.data() + bc.q_row * m;
        const double* A_col = WA_perm_.data() + bc.q_col * m;
        if (!blas::Dgemm(bc.rows, bc.cols, m, 1.0,
                         A_row, m, A_col, m, bc.dest, bc.dest_ld)) {
          using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
          Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
              bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));
          dest.noalias() +=
              WA_perm_.middleCols(bc.q_row, bc.rows).transpose() *
              WA_perm_.middleCols(bc.q_col, bc.cols);
        }
      }
    }
  }

 protected:
  void ensure_weights_fresh() {
    if (weights_dirty_) update_weights();
  }

  WorkspaceLinear* ws_ = nullptr;
  const Eigen::MatrixXd* A_ = nullptr;
  Eigen::MatrixXd A_perm_;
  Eigen::MatrixXd WA_perm_;
  bool order_set_ = false;
  bool weights_dirty_ = true;

  std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
  double atranspose_scale_ = 1.0;

 protected:
  std::vector<VectorBlockContribution> vector_blocks_;

 private:
  int sn_count_ = 0;
};

class LinearConstraint : public ConeConstraint {
 public:
  LinearConstraint(const Eigen::MatrixXd& constraint_matrix,
                   const Eigen::MatrixXd& constraint_affine);

  int number_of_variables() const override { return constraint_matrix_.cols(); }

  BlockAssembler* GetBlockAssembler() override {
    gram_evaluator_.bind(&workspace_, &constraint_matrix_);
    return &gram_evaluator_;
  }

  Eigen::MatrixXd affine_term() const override { return constraint_affine_; }
  int num_rows() const override { return constraint_matrix_.rows(); }
  const EuclideanJordanAlgebra::SymmetricConeOperations* cone_ops() const override {
    return cone_ops_;
  }

  void SetWeights(const Eigen::VectorXd& weights) override {
    CONEX_DEMAND(weights.size() == constraint_matrix_.rows(),
                 "Weight vector size must match number of constraint rows.");
    workspace_.W = weights.array().sqrt().matrix();
    gram_evaluator_.update_weights();
  }

  void SetScaling(const Eigen::VectorXd& scaling) override {
    CONEX_DEMAND(scaling.size() == constraint_matrix_.rows(),
                 "Scaling vector size must match number of constraint rows.");
    workspace_.W = scaling;
    gram_evaluator_.update_weights();
  }

  Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const override {
    return gram().MultiplyA(supernodes, sep, nc);
  }

  void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep,
      int nc) const override {
    gram().ContributeAtranspose(V, supernodes, sep, nc);
  }

  // Access the GramEvaluator (used by default MultiplyA/ContributeAtranspose).
  virtual const GramEvaluator& gram() const { return gram_evaluator_; }

  // Public cone_ops pointer — set by constraint assemblers.
  const EuclideanJordanAlgebra::SymmetricConeOperations* cone_ops_ = nullptr;

  // --- z-space operations (nonneg: W = -∇F(z) = 1/z) ---

  void ComputeGradient(Eigen::VectorXd& grad) const override {
    const auto& W = workspace_.W;
    grad.resize(W.size());
    grad = -W;  // ∇F(z) = -1/z = -W
  }

  void HessianProduct(const Eigen::VectorXd& v,
                      Eigen::VectorXd& out) const override {
    const auto& W = workspace_.W;
    out.resize(W.size());
    // H(z) = diag(1/z²) = diag(W²), so H(z)v = W²·v.
    out = W.array().square() * v.array();
  }

  double StepSize(const Eigen::VectorXd& target_k) const override {
    const auto& W = workspace_.W;
    // d = e - W·target_k (eigenvalue-space direction).
    double d_inf = 0;
    for (int i = 0; i < W.size(); ++i) {
      double di = 1.0 - W(i) * target_k(i);
      d_inf = std::max(d_inf, std::abs(di));
    }
    return std::min(1.0, 2.0 / (d_inf * d_inf));
  }

  void GeodesicStep(double alpha, const Eigen::VectorXd& target) override {
    auto& W = workspace_.W;
    // d_i = 1 - W_i * target_i, then W_i *= exp(alpha * d_i).
    for (int i = 0; i < W.size(); ++i) {
      double di = 1.0 - W(i) * target(i);
      W(i) *= std::exp(alpha * di);
    }
    gram_evaluator_.update_weights();
  }

  double LineSearch(const Eigen::VectorXd& target0,
                    const Eigen::VectorXd& target1) const override {
    const auto& W = workspace_.W;
    const int m = static_cast<int>(W.size());
    // d0 = e - W·target0, d1 = -W·target1.
    Eigen::VectorXd d0(m), d1(m);
    for (int i = 0; i < m; ++i) {
      d0(i) = 1.0 - W(i) * target0(i);
      d1(i) = -W(i) * target1(i);
    }
    // Reuse existing lineSearchK: max k with ||d0 + k*d1||_inf <= 1.
    double k_max = std::numeric_limits<double>::max();
    for (int i = 0; i < m; ++i) {
      if (d1(i) > 0)
        k_max = std::min(k_max, (1.0 - d0(i)) / d1(i));
      else if (d1(i) < 0)
        k_max = std::min(k_max, (-1.0 - d0(i)) / d1(i));
    }
    return k_max;
  }

  double HessianNormSquared(const Eigen::VectorXd& target) const override {
    const auto& W = workspace_.W;
    // ||target - z||²_H = Σ W_i² (target_i - 1/W_i)²
    //                    = Σ (W_i target_i - 1)² = ||d||²
    double result = 0;
    for (int i = 0; i < W.size(); ++i) {
      double di = W(i) * target(i) - 1.0;
      result += di * di;
    }
    return result;
  }

  double BarrierParameter() const override {
    return static_cast<double>(num_rows());
  }

  // ArenaAllocatable interface.
  size_t RequiredArenaBytes() const override {
    return SizeOf(workspace_) * sizeof(double);
  }
  void BindArenaMemory(double* ptr, size_t /*bytes*/) override {
    Initialize(&workspace_, ptr);
  }

 protected:
  WorkspaceLinear workspace_;
  GramEvaluator gram_evaluator_;
  Eigen::MatrixXd constraint_matrix_;
  Eigen::MatrixXd constraint_affine_;
};

}  // namespace conex
