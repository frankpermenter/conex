#pragma once
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/tree_rhs.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Lazy evaluator for a dense symmetric sub-block of Q.
// Used for per-clique sub-assemblers after Decompose.
class DenseQuadraticTermLazyEvaluator : public BlockAssembler {
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

  int rows() const override { return Q_ ? static_cast<int>(Q_->rows()) : 0; }
  int cols() const override { return Q_ ? static_cast<int>(Q_->cols()) : 0; }
  void set_sn_count(int c) override { sn_count_ = c; }
  int sn_count() const { return sn_count_; }

  const Eigen::MatrixXd& Q_perm() const { return Q_perm_; }

  bool RegisterContributions(
      int clique_id, const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) override {
    if (!order_set_) set_order(perm);
    registered_blocks_[clique_id] = blocks;
    return true;
  }

  void ContributeBlocks(int clique_id) override {
    auto it = registered_blocks_.find(clique_id);
    if (it == registered_blocks_.end()) return;
    for (const auto& bc : it->second) {
      using StrideType = Eigen::Stride<Eigen::Dynamic, 1>;
      Eigen::Map<Eigen::MatrixXd, 0, StrideType> dest(
          bc.dest, bc.rows, bc.cols, StrideType(bc.dest_ld, 1));
      if (bc.lower_only) {
        const auto src = Q_perm_.block(bc.q_row, bc.q_col, bc.rows, bc.cols);
        for (int j = 0; j < bc.cols; ++j)
          dest.col(j).tail(bc.rows - j) += src.col(j).tail(bc.rows - j);
      } else {
        dest.noalias() += Q_perm_.block(bc.q_row, bc.q_col, bc.rows, bc.cols);
      }
    }
  }

  void RegisterVectorContributions(
      const std::vector<VectorBlockContribution>& blocks) override {
    vector_blocks_ = blocks;
  }

  // Compute Q_perm_ * x, reading x from supernode blocks + sep scratch.
  // Result is accumulated into the SAME blocks (Q maps x-space to x-space).
  template <typename SepAccessor>
  void MultiplyQx(const BlockPartition& x_sn, const SepAccessor& x_sep,
                   BlockPartition& out_sn, SepAccessor& out_sep, int nc) const {
    const int nv = static_cast<int>(vector_blocks_.size());
    for (int i = 0; i < nv; ++i) {
      const auto& vi = vector_blocks_[i];
      for (int j = 0; j < nv; ++j) {
        const auto& vj = vector_blocks_[j];
        auto Q_sub = Q_perm_.block(vi.q_start, vj.q_start, vi.length, vj.length);
        // Read x from block j, multiply, accumulate into block i.
        if (vj.dest_is_sn) {
          auto x_block = x_sn.block(vj.dest_block)
              .middleRows(vj.dest_offset, vj.length);
          if (vi.dest_is_sn) {
            out_sn.block(vi.dest_block)
                .middleRows(vi.dest_offset, vi.length) += Q_sub * x_block;
          } else {
            out_sep.block(vi.dest_block, nc)
                .middleRows(vi.dest_offset, vi.length) += Q_sub * x_block;
          }
        } else {
          auto x_block = x_sep.block(vj.dest_block, nc)
              .middleRows(vj.dest_offset, vj.length);
          if (vi.dest_is_sn) {
            out_sn.block(vi.dest_block)
                .middleRows(vi.dest_offset, vi.length) += Q_sub * x_block;
          } else {
            out_sep.block(vi.dest_block, nc)
                .middleRows(vi.dest_offset, vi.length) += Q_sub * x_block;
          }
        }
      }
    }
  }

 private:
  const Eigen::MatrixXd* Q_ = nullptr;
  Eigen::MatrixXd Q_perm_;
  bool order_set_ = false;
  int sn_count_ = 0;
  std::unordered_map<int, std::vector<BlockContribution>> registered_blocks_;
  std::vector<VectorBlockContribution> vector_blocks_;
};

// Per-clique assembler for a dense sub-block of Q.
class DenseQuadraticTermSubAssembler : public SupernodalAssemblerBase {
 public:
  DenseQuadraticTermSubAssembler(Eigen::MatrixXd Q_block,
                        const std::vector<int>& variables)
      : SupernodalAssemblerBase(variables), Q_block_(std::move(Q_block)) {
    evaluator_.bind(&Q_block_);
  }

  BlockAssembler* GetBlockAssembler() override { return &evaluator_; }
  bool is_positive_definite() const override { return true; }
  bool is_dynamic() const override { return false; }

  const Eigen::MatrixXd& Q_block() const { return Q_block_; }
  DenseQuadraticTermLazyEvaluator& evaluator() { return evaluator_; }
  const DenseQuadraticTermLazyEvaluator& evaluator() const { return evaluator_; }

 private:
  Eigen::MatrixXd Q_block_;
  DenseQuadraticTermLazyEvaluator evaluator_;
};

// Top-level assembler for a sparse quadratic term Q.
// Provides cliques (edges from Q's sparsity) to the clique tree builder.
// Decompose() extracts per-clique dense sub-blocks.
class SparseQuadraticTermAssembler : public SupernodalAssemblerBase {
 public:
  SparseQuadraticTermAssembler(const Eigen::SparseMatrix<double>& Q,
                      const std::vector<int>& variables);
  SparseQuadraticTermAssembler(const Eigen::MatrixXd& Q,
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

  // Bind partition info for block-space operations (no-op for now).
  void BindPartition(const class KKTSolverBase& solver);
  bool partition_bound() const { return !block_info_.empty(); }

  // Compute Q*x in block space: reads x from partition blocks,
  // accumulates Q_perm * x_block into result (global vector, size n).
  Eigen::VectorXd ComputeBlockProduct(
      const class KKTSolverBase& solver) const;

  // Accumulate Q*x into an existing partition (adds to blocks in-place).
  // For building the gradient Q*x + c + A^T*v without leaving block space.
  void AccumulateBlockProduct(
      class KKTSolverBase& solver,
      const class BlockPartition& x_partition) const;

  // Access decomposed sub-assemblers (available after Decompose).
  const std::list<DenseQuadraticTermSubAssembler>& sub_assemblers() const {
    return owned_sub_assemblers_;
  }

  // Accumulate Q * x into a TreeRHS.
  // sep_in must be pre-populated via ScatterSeparators(x).
  void ComputeProduct(const TreeRHS& x,
                      const SeparatorScratch& sep_in,
                      TreeRHS& rhs) const {
    int nc = x.cols();
    for (const auto& sub : owned_sub_assemblers_) {
      sub.evaluator().MultiplyQx(
          *x.supernodes, sep_in,
          *rhs.supernodes, *rhs.separators, nc);
    }
  }

 private:
  const Eigen::SparseMatrix<double>* Q_sparse_ = nullptr;
  const Eigen::MatrixXd* Q_dense_ = nullptr;
  bool dense_ = false;

  // Owned sub-assemblers created by Decompose.
  std::list<DenseQuadraticTermSubAssembler> owned_sub_assemblers_;
  // Per-sub-assembler block mapping (set by BindPartition).
  struct BlockInfo { int block_index; };
  std::vector<BlockInfo> block_info_;
};

}  // namespace conex

// For backward compatibility — types moved to algorithms/least_squares.h.
#include "conex/algorithms/least_squares.h"
