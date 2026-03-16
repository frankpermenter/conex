#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <unordered_map>

#include "conex/kkt_solver_interface.h"
#include "conex/kkt_subsystem.h"
#include "conex/static_subsystem.h"
#include "conex/tree_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// A matrix partitioned by the supernode structure of the elimination tree.
// Each node contributes a supernode block (sn_rows x cols) and a separator
// block (sep_rows x cols), stored at SIMD-aligned pointers in a single arena.
class SupernodePartitionMatrix {
 public:
  SupernodePartitionMatrix() = default;

  // Set the block structure from the subsystem list.
  // perm_inv maps elimination position -> original variable index.
  void SetPartition(const std::vector<KKTSubsystemBase*>& subsystems,
                    int num_vars,
                    const Eigen::VectorXi& perm_inv);

  // (Re)allocate arena for the given number of columns.
  void Resize(int cols);

  bool empty() const { return blocks_.empty(); }
  int cols() const { return cols_; }
  void SetZero();

  // Scatter/gather between an original-order matrix and supernode blocks.
  void ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> b);
  void GatherInto(Eigen::Ref<Eigen::MatrixXd> b) const;

  // Scatter/gather between an elimination-order matrix and supernode blocks.
  // Copies block-wise (supernodes are contiguous in elimination order).
  void ScatterFromElimOrder(Eigen::Ref<const Eigen::MatrixXd> b);
  void GatherIntoElimOrder(Eigen::Ref<Eigen::MatrixXd> b) const;

  // Block accessors (by subsystem index).
  Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> supernode(int k) {
    return {blocks_[k].supernode_data, blocks_[k].sn_rows, cols_};
  }
  Eigen::Map<Eigen::MatrixXd, Eigen::Aligned> separator(int k) {
    return {blocks_[k].separator_data, blocks_[k].sep_rows, cols_};
  }
  Eigen::Map<const Eigen::MatrixXd, Eigen::Aligned> supernode(int k) const {
    return {blocks_[k].supernode_data, blocks_[k].sn_rows, cols_};
  }
  Eigen::Map<const Eigen::MatrixXd, Eigen::Aligned> separator(int k) const {
    return {blocks_[k].separator_data, blocks_[k].sep_rows, cols_};
  }

  int supernode_rows(int k) const { return blocks_[k].sn_rows; }
  int separator_rows(int k) const { return blocks_[k].sep_rows; }

 private:
  struct Block {
    double* supernode_data = nullptr;
    double* separator_data = nullptr;
    int sn_rows = 0;
    int sep_rows = 0;
    int sn_start = 0;  // first supernode index in elimination order
  };
  struct VarMapping {
    int block_index;
    int row_in_block;
  };
  std::vector<Block> blocks_;
  std::vector<VarMapping> var_mapping_;
  int cols_ = 0;
  std::unique_ptr<void, decltype(&std::free)> arena_{nullptr, &std::free};
  size_t arena_bytes_ = 0;
};

enum class ContributionType { kPositiveDefinite, kIndefinite };

// Provides labeled write access to the supernode/separator storage blocks
// of a single subsystem.  Created by SymmetricLinearSystemTreeSolver::
// MakeContributor after the tree has been finalized.
class SubmatrixContributor {
 public:
  SubmatrixContributor() = default;

  // Supernode range in elimination order (always contiguous).
  int supernode_start() const { return sn_start_; }
  int supernode_count() const { return sn_count_; }

  // Separator indices in elimination order (sorted).
  const std::vector<int>& separator_indices() const { return sep_indices_; }

  // Mutable access to the three storage blocks.
  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() {
    return subsystem_->supernode_submatrix();
  }
  Eigen::Ref<Eigen::MatrixXd> separator_rows() {
    return subsystem_->separator_rows();
  }
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() {
    return subsystem_->separator_schur_complement();
  }

  // Write a symmetric matrix Q into the storage blocks, permuting from
  // original variable order to elimination order.  elim_positions[i] is the
  // elimination position of Q's i-th row/column.  Only the lower triangle
  // is written.
  void WriteSymmetric(const Eigen::MatrixXd& Q,
                      const std::vector<int>& elim_positions);

  // Lazy variant: LazyMatrix must provide:
  //   void set_order(const std::vector<int>& perm);
  //   Eigen::MatrixXd block(int row, int col, int rows, int cols) const;
  //   int rows() const;
  //   int cols() const;
  // set_order is called with a permutation that reorders the lazy matrix's
  // indices to maximize contiguous block writes (supernodes first, then
  // separators, each sorted by local elimination index).
  template <typename LazyMatrix>
  void WriteSymmetricLazy(LazyMatrix& lazy,
                          const std::vector<int>& elim_positions);

  // Declare the contribution type.  If any contributor to a subsystem is
  // indefinite, the solver uses LU factorization for that clique.
  void set_type(ContributionType type);

 private:
  friend class SymmetricLinearSystemTreeSolver;
  KKTSubsystemBase* subsystem_ = nullptr;
  int sn_start_ = 0;
  int sn_count_ = 0;
  std::vector<int> sep_indices_;
};

// --- Template implementation of WriteSymmetricLazy ---
template <typename LazyMatrix>
void SubmatrixContributor::WriteSymmetricLazy(
    LazyMatrix& lazy, const std::vector<int>& elim_positions) {
  const int n = static_cast<int>(elim_positions.size());
  CONEX_DEMAND(lazy.rows() == n && lazy.cols() == n,
               "Lazy matrix dimensions must match elim_positions size.");

  // Classify each index: supernode or separator, with local offset.
  struct VarInfo {
    bool is_sn;
    int local;
  };
  std::vector<VarInfo> info(n);
  std::unordered_map<int, int> sep_to_local;
  sep_to_local.reserve(sep_indices_.size());
  for (int i = 0; i < static_cast<int>(sep_indices_.size()); ++i) {
    sep_to_local[sep_indices_[i]] = i;
  }
  for (int i = 0; i < n; ++i) {
    int ep = elim_positions[i];
    if (ep >= sn_start_ && ep < sn_start_ + sn_count_) {
      info[i] = {true, ep - sn_start_};
    } else {
      auto it = sep_to_local.find(ep);
      CONEX_DEMAND(
          it != sep_to_local.end(),
          "elim_positions entry not in contributor's sparsity pattern.");
      info[i] = {false, it->second};
    }
  }

  // Compute optimal permutation: supernodes sorted by local index first,
  // then separators sorted by local index.  This maximizes contiguous runs.
  std::vector<int> perm(n);
  for (int i = 0; i < n; ++i) perm[i] = i;
  std::sort(perm.begin(), perm.end(), [&](int a, int b) {
    if (info[a].is_sn != info[b].is_sn) return info[a].is_sn > info[b].is_sn;
    return info[a].local < info[b].local;
  });

  // Reorder the lazy matrix and elim_positions to match the optimal order.
  lazy.set_order(perm);
  std::vector<int> perm_elim(n);
  std::vector<VarInfo> perm_info(n);
  for (int i = 0; i < n; ++i) {
    perm_elim[i] = elim_positions[perm[i]];
    perm_info[i] = info[perm[i]];
  }

  // Find maximal contiguous runs in the permuted order.
  struct Run {
    int q_start;
    int length;
    bool is_sn;
    int local_start;
  };
  std::vector<Run> runs;
  runs.reserve(n);
  int i = 0;
  while (i < n) {
    Run run{i, 1, perm_info[i].is_sn, perm_info[i].local};
    while (i + run.length < n &&
           perm_info[i + run.length].is_sn == run.is_sn &&
           perm_info[i + run.length].local == run.local_start + run.length) {
      run.length++;
    }
    runs.push_back(run);
    i += run.length;
  }

  // Dispatch blocks into storage using lazy evaluation.
  auto sn_sub = supernode_submatrix();
  auto sep_r = separator_rows();
  auto sep_sc = separator_schur_complement();

  const int nr = static_cast<int>(runs.size());
  for (int ci = 0; ci < nr; ++ci) {
    const auto& cr = runs[ci];
    for (int ri = ci; ri < nr; ++ri) {
      const auto& rr = runs[ri];
      Eigen::MatrixXd Qblk =
          lazy.block(rr.q_start, cr.q_start, rr.length, cr.length);

      if (ri == ci) {
        if (rr.is_sn) {
          sn_sub.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .template triangularView<Eigen::Lower>() += Qblk;
        } else {
          sep_sc.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .template triangularView<Eigen::Lower>() += Qblk;
        }
      } else if (rr.is_sn && cr.is_sn) {
        if (rr.local_start > cr.local_start) {
          sn_sub.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .noalias() += Qblk;
        } else {
          sn_sub.block(cr.local_start, rr.local_start, cr.length, rr.length)
              .noalias() += Qblk.transpose();
        }
      } else if (!rr.is_sn && cr.is_sn) {
        sep_r.block(rr.local_start, cr.local_start, rr.length, cr.length)
            .noalias() += Qblk;
      } else if (rr.is_sn && !cr.is_sn) {
        sep_r.block(cr.local_start, rr.local_start, cr.length, rr.length)
            .noalias() += Qblk.transpose();
      } else {
        if (rr.local_start > cr.local_start) {
          sep_sc.block(rr.local_start, cr.local_start, rr.length, cr.length)
              .noalias() += Qblk;
        } else {
          sep_sc.block(cr.local_start, rr.local_start, cr.length, rr.length)
              .noalias() += Qblk.transpose();
        }
      }
    }
  }
}

struct Options {
  bool validate_leaf_nodes = false;
  bool check_for_zero_pivots = false;
  int root_node = 0;
  int num_threads = 1;
};

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:
  int number_of_variables() const;
  void AddSubsystem(KKTSubsystemBase* system);
  void RepairTreeInPlace(std::vector<int>* parent_ptr);

  void Finalize(const CliqueTree& clique_tree);
  void Finalize(const Options& options = Options());
  void Finalize(const std::vector<int>& subsystem_to_parent_subsystem,
                bool check_for_zero_pivots = true);

  void SetFactorizationMode(bool left_looking);
  void SetNumThreads(int num_threads);
  void SetParallelizeRootsOnly(bool enable);
  void ReserveSolveWorkspace(int rhs_cols);
  void EnableAutoUpdateAtAssemble(bool enable) {
    auto_update_assemblers_ = enable;
  }
  void UpdateAssemblerData();

  std::vector<int> subsystem_to_parent() { return subsystem_to_parent_; }
  const std::vector<int>& variable_to_elimination_position() const {
    return variable_to_elimination_position_;
  }

  Eigen::SparseMatrix<double> MakeSparseKKTMatrix(
      bool permute_to_elimination_order = true) const;

  void ComputeSeparatorOffsets();
  std::vector<int> ComputePostOrdering() const;
  void push_back(std::unique_ptr<KKTAssemblerToSubsystemAdapter>&& system);

  // Create a contributor that provides labeled write access to the storage
  // blocks of the subsystem containing the given elimination indices.
  // Throws if the indices are not all within a single subsystem's sparsity
  // pattern.
  SubmatrixContributor MakeContributor(
      const std::vector<int>& elim_indices) const;

  void SetEliminationTree(
      const std::vector<int>& variable_to_elimination_position);

 private:
  void SetEliminationOrder(
      const std::vector<int>& variable_to_elimination_position);
  void FinalizeHelper(const std::vector<int>& subsystem_to_parent_subsystem);
  Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;
  bool DoAssembleAndFactor() override;
  bool DoFactor() override;
  bool CheckForZeroPivot(const std::vector<int>& parent,
                         std::vector<int>* subsystems_with_zero_piviot);
  void AllocateArenaAndBind();

  std::vector<KKTSubsystemBase*> roots_;
  std::vector<KKTSubsystemBase*> subsystems_;
  std::vector<std::unique_ptr<KKTSubsystemBase>> owned_subsystems_;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>>
      assembler_to_subsystem_adapter_;
  std::vector<int> variable_to_elimination_position_;
  std::vector<int> subsystem_to_parent_;
  bool auto_update_assemblers_ = false;
  int num_threads_ = 1;
  bool parallelize_roots_only_ = false;
  mutable int reserved_solve_workspace_cols_ = 0;
  // Flat post-order traversal for non-recursive solve.
  std::vector<KKTSubsystemBase*> solve_order_;
  // Cached permutation data for solve (avoids per-solve allocations).
  int cached_num_vars_ = 0;
  Eigen::VectorXi cached_perm_;         // variable -> elimination position
  Eigen::VectorXi cached_perm_inv_;     // elimination position -> variable
  std::unique_ptr<void, decltype(&std::free)> arena_memory_{nullptr,
                                                            &std::free};
  size_t arena_bytes_ = 0;
  // Block-partitioned solve data (mutable: scratch space used in const solve).
  mutable SupernodePartitionMatrix solve_matrix_;
  // Per-node precomputed child scatter info for blocked solve.
  struct ChildScatterOp {
    int child_block_index;
    std::vector<KKTSubsystemBase::Offset> sn_offsets;
    std::vector<KKTSubsystemBase::Offset> sep_offsets;
  };
  struct NodeScatterInfo {
    int block_index;
    std::vector<ChildScatterOp> children;
  };
  std::vector<NodeScatterInfo> solve_scatter_info_;  // indexed by solve_order pos
  void AllocateSolveArena();
  // Consolidated workspace arena for all subsystems' solve scratch buffers.
  std::unique_ptr<void, decltype(&std::free)> workspace_arena_{nullptr,
                                                                &std::free};
  size_t workspace_arena_bytes_ = 0;
  int workspace_arena_cols_ = 0;
  void AllocateWorkspaceArena(int rhs_cols);
};

}  // namespace conex
