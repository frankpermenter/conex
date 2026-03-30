#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include "conex/common/block_partition.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/tree_solver/kkt_subsystem.h"
#include "conex/tree_solver/static_subsystem.h"
#include "conex/tree_solver/tree_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Forward declaration.
class SupernodePartitionMatrix;

// BlockPartition adapter for the tree solver's supernodal partition.
// Each block corresponds to one supernode in the elimination tree.
// Scatter/gather use the elimination ordering's variable mapping.
class TreeBlockPartition : public BlockPartition {
 public:
  TreeBlockPartition() = default;
  void Bind(SupernodePartitionMatrix* spm, int num_vars) {
    spm_ = spm;
    num_vars_ = num_vars;
  }

  int num_blocks() const override;
  int block_size(int k) const override;
  int num_variables() const override { return num_vars_; }
  int cols() const override;
  void Resize(int cols) override;
  void SetZero() override;
  void ScatterFrom(Eigen::Ref<const Eigen::MatrixXd> x) override;
  void GatherInto(Eigen::Ref<Eigen::MatrixXd> x) const override;
  Eigen::Ref<Eigen::MatrixXd> block(int k) override;
  Eigen::Ref<const Eigen::MatrixXd> block(int k) const override;

  // Tree-specific: access supernode/separator separately.
  SupernodePartitionMatrix& raw() { return *spm_; }
  const SupernodePartitionMatrix& raw() const { return *spm_; }

 private:
  SupernodePartitionMatrix* spm_ = nullptr;
  int num_vars_ = 0;
};

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
  int num_blocks_internal() const { return static_cast<int>(blocks_.size()); }

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

  // Precompute the optimal permutation and contiguous-run structure for
  // WriteSymmetricLazy.  Call once after the contributor is created (the
  // tree solver does this automatically for adapters).  Subsequent
  // WriteSymmetricLazy calls skip the permutation/run computation and
  // reuse the cached data.
  void PrecomputeLazyOrder(const std::vector<int>& elim_positions);

  // Register: precompute permutation, runs, and block destinations.
  // Called once at Finalize time with elimination positions.
  // Offers the two-phase protocol to the block assembler.
  template <typename BlockAssemblerT>
  void Register(BlockAssemblerT& lazy, const std::vector<int>& elim_positions);

  // Assemble: write the block assembler's data into subsystem storage.
  // Called at each AssembleAndFactor with no position arguments.
  template <typename BlockAssemblerT>
  void Assemble(BlockAssemblerT& lazy);

  // Declare the contribution type.  If any contributor to a subsystem is
  // indefinite, the solver uses LU factorization for that clique.
  void set_type(ContributionType type);

  // Clique (subsystem) index this contributor writes to.
  int clique_id() const { return clique_id_; }

 private:
  friend class SymmetricLinearSystemTreeSolver;
  KKTSubsystemBase* subsystem_ = nullptr;
  int sn_start_ = 0;
  int sn_count_ = 0;
  int clique_id_ = -1;
  std::vector<int> sep_indices_;

  // Cached lazy-order data (computed by PrecomputeLazyOrder).
  struct Run {
    int q_start;
    int length;
    bool is_sn;
    int local_start;
  };
  bool order_cached_ = false;
  std::vector<int> cached_perm_;
  std::vector<Run> cached_runs_;
};

// --- Template implementations ---

template <typename BlockAssemblerT>
void SubmatrixContributor::Register(
    BlockAssemblerT& lazy, const std::vector<int>& elim_positions) {
  const int n = static_cast<int>(elim_positions.size());
  CONEX_DEMAND(lazy.rows() == n && lazy.cols() == n,
               "Lazy matrix dimensions must match elim_positions size.");

  PrecomputeLazyOrder(elim_positions);
  lazy.set_sn_count(sn_count_);

  // Build BlockContribution list from cached runs.
  auto sn_sub = supernode_submatrix();
  auto sep_r = separator_rows();
  auto sep_sc = separator_schur_complement();

  std::vector<BlockContribution> blocks;
  const int nr = static_cast<int>(cached_runs_.size());
  for (int ci = 0; ci < nr; ++ci) {
    const auto& cr = cached_runs_[ci];
    for (int ri = ci; ri < nr; ++ri) {
      const auto& rr = cached_runs_[ri];
      BlockContribution bc;
      bc.lower_only = (ri == ci);

      if (ri == ci) {
        bc.q_row = rr.q_start;
        bc.q_col = rr.q_start;
        bc.rows = rr.length;
        bc.cols = rr.length;
        if (rr.is_sn) {
          bc.dest = &sn_sub(rr.local_start, rr.local_start);
          bc.dest_ld = sn_sub.outerStride();
        } else {
          bc.dest = &sep_sc(rr.local_start, rr.local_start);
          bc.dest_ld = sep_sc.outerStride();
        }
      } else if (rr.is_sn && cr.is_sn) {
        if (rr.local_start > cr.local_start) {
          bc.q_row = rr.q_start; bc.q_col = cr.q_start;
          bc.rows = rr.length; bc.cols = cr.length;
          bc.dest = &sn_sub(rr.local_start, cr.local_start);
        } else {
          bc.q_row = cr.q_start; bc.q_col = rr.q_start;
          bc.rows = cr.length; bc.cols = rr.length;
          bc.dest = &sn_sub(cr.local_start, rr.local_start);
        }
        bc.dest_ld = sn_sub.outerStride();
      } else if (!rr.is_sn && cr.is_sn) {
        bc.q_row = rr.q_start; bc.q_col = cr.q_start;
        bc.rows = rr.length; bc.cols = cr.length;
        bc.dest = &sep_r(rr.local_start, cr.local_start);
        bc.dest_ld = sep_r.outerStride();
      } else if (rr.is_sn && !cr.is_sn) {
        bc.q_row = cr.q_start; bc.q_col = rr.q_start;
        bc.rows = cr.length; bc.cols = rr.length;
        bc.dest = &sep_r(cr.local_start, rr.local_start);
        bc.dest_ld = sep_r.outerStride();
      } else {
        if (rr.local_start > cr.local_start) {
          bc.q_row = rr.q_start; bc.q_col = cr.q_start;
          bc.rows = rr.length; bc.cols = cr.length;
          bc.dest = &sep_sc(rr.local_start, cr.local_start);
        } else {
          bc.q_row = cr.q_start; bc.q_col = rr.q_start;
          bc.rows = cr.length; bc.cols = rr.length;
          bc.dest = &sep_sc(cr.local_start, rr.local_start);
        }
        bc.dest_ld = sep_sc.outerStride();
      }
      blocks.push_back(bc);
    }
  }

  bool ok = lazy.RegisterContributions(clique_id_, cached_perm_, blocks);
  CONEX_DEMAND(ok, "BlockAssembler must support RegisterContributions.");
}

template <typename BlockAssemblerT>
void SubmatrixContributor::Assemble(BlockAssemblerT& assembler) {
  CONEX_DEMAND(order_cached_, "Register must be called before Assemble.");
  assembler.ContributeBlocks(clique_id_);
}

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:
  int number_of_variables() const override;

  void Finalize(const CliqueTree& clique_tree, int rhs_cols = 1);

  void SetFactorizationMode(bool left_looking);
  void SetScatterToParent(bool enable);
  void SetNumThreads(int num_threads);
  void SetUseRecursiveSolve(bool enable) { use_recursive_solve_ = enable; }
  void SetUseGenericFactorization(bool enable) {
    use_generic_factorization_ = enable;
  }
  void SetUseLUForIndefinite(bool enable) {
    use_lu_for_indefinite_ = enable;
  }
  void EnableAutoUpdateAtAssemble(bool enable) {
    auto_update_assemblers_ = enable;
  }
  void UpdateAssemblerData();

  std::vector<int> subsystem_to_parent() { return subsystem_to_parent_; }
  const std::vector<int>& variable_to_elimination_position() const {
    return variable_to_elimination_position_;
  }

  void ComputeSeparatorOffsets();
  void push_back(std::unique_ptr<KKTAssemblerToSubsystemAdapter>&& system);

  // Access contributor (adapter) by index.
  KKTAssemblerToSubsystemAdapter* GetContributor(int index) {
    return contributors_.at(index).get();
  }

  // BlockPartition interface (from KKTSolverBase).
  BlockPartition& partition() override { return block_partition_; }
  const BlockPartition& partition() const override { return block_partition_; }

  std::unique_ptr<BlockPartition> MakePartition() override {
    std::vector<int> block_sizes, block_starts;
    for (int k = 0; k < num_subsystems(); ++k) {
      const auto& sn = subsystems_[k]->supernodes();
      block_sizes.push_back(static_cast<int>(sn.size()));
      block_starts.push_back(sn.empty() ? 0 : sn.front());
    }
    return std::make_unique<StandaloneBlockPartition>(
        block_sizes, block_starts, perm(), perm_inv());
  }

  // Tree-specific: access the raw SupernodePartitionMatrix.
  SupernodePartitionMatrix& raw_partition() { return solve_matrix_; }
  const SupernodePartitionMatrix& raw_partition() const { return solve_matrix_; }

  // Number of subsystems (blocks in the partition).
  int num_subsystems() const { return static_cast<int>(subsystems_.size()); }

  // Access the elimination permutation.
  const Eigen::VectorXi& perm() const { return cached_perm_; }
  const Eigen::VectorXi& perm_inv() const { return cached_perm_inv_; }

  // Pre-inject a subsystem for a specific clique index.  Must be called
  // before Finalize.  CreateSubsystems will use injected subsystems instead
  // of creating default ones.
  void InjectSubsystem(int clique_index,
                        std::unique_ptr<KKTSubsystemBase> subsystem) {
    if (static_cast<int>(injected_subsystems_.size()) <= clique_index) {
      injected_subsystems_.resize(clique_index + 1);
    }
    injected_subsystems_[clique_index] = std::move(subsystem);
  }


  // Solve with supernodes from an external partition.  The partition's
  // block(k) is used as supernode(k); separator scratch is internal.
  // The partition is modified in place (RHS in, solution out).
  void SolveBlockedInPlace(BlockPartition& supernodes) const;

  // DoSolveBlocked: solves directly in rhs's partition, copies to dest.
  bool DoSolveBlocked(const BlockPartition& rhs,
                      BlockPartition& dest) const override;

  void SetEliminationTree(
      const std::vector<int>& variable_to_elimination_position);

 private:
  // Solve using the internal solve_matrix_ (supernode + separator scratch).
  void SolveBlockedInPlace() const;
  void SetEliminationOrder(
      const std::vector<int>& variable_to_elimination_position);
  Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;
  bool DoAssembleAndFactor() override;
  bool DoFactor() override;

  std::vector<int> ClassifyCliques(const CliqueTree& clique_tree,
                                   std::vector<bool>* needs_indefinite);
  void CreateSubsystems(const std::vector<bool>& needs_indefinite);
  void ComputeEliminationOrder(const CliqueTree& clique_tree);
  void BindContributors(const std::vector<int>& adapter_to_clique);
  void AllocateArenaAndBind(int rhs_cols);

  std::vector<KKTSubsystemBase*> roots_;
  std::vector<KKTSubsystemBase*> subsystems_;
  std::vector<std::unique_ptr<KKTSubsystemBase>> owned_subsystems_;
  std::vector<std::unique_ptr<KKTSubsystemBase>> injected_subsystems_;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>>
      contributors_;
  std::vector<int> variable_to_elimination_position_;
  std::vector<int> subsystem_to_parent_;
  bool auto_update_assemblers_ = false;
  int num_threads_ = 1;
  bool use_recursive_solve_ = false;
  bool use_generic_factorization_ = false;
  bool use_lu_for_indefinite_ = false;
  // Leaf-parallel factorization: launch tasks from leaves, propagate up.
  std::vector<KKTSubsystemBase*> leaves_;
  bool DoAssembleAndFactorLeafParallel();
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
  mutable TreeBlockPartition block_partition_;

  // Separator scratch: arena-allocated storage for separator temporaries
  // during SolveBlockedInPlace(BlockPartition&).  Allocated once at
  // Finalize with reserved_solve_workspace_cols_ columns.
  struct SeparatorScratch {
    std::vector<int> sep_rows;
    std::vector<int> offsets;
    int total_rows = 0;
    int reserved_cols = 0;
    std::unique_ptr<void, decltype(&std::free)> arena{nullptr, &std::free};
    std::vector<double*> block_ptrs;  // pointer per subsystem

    void Init(const std::vector<KKTSubsystemBase*>& subsystems, int cols) {
      sep_rows.clear();
      offsets.clear();
      int off = 0;
      for (auto* s : subsystems) {
        int sr = static_cast<int>(s->separators().size());
        sep_rows.push_back(sr);
        offsets.push_back(off);
        off += sr;
      }
      total_rows = off;
      reserved_cols = cols;

      // Arena allocate with SIMD alignment.
      constexpr size_t kAlign = EIGEN_MAX_ALIGN_BYTES;
      size_t bytes = static_cast<size_t>(total_rows) * cols * sizeof(double);
      bytes = ((bytes + kAlign - 1) / kAlign) * kAlign;
      if (bytes > 0) {
        void* raw = nullptr;
        if (posix_memalign(&raw, kAlign, bytes) != 0) throw std::bad_alloc();
        arena.reset(raw);
      }

      // Set per-block pointers.
      block_ptrs.resize(sep_rows.size());
      double* base = static_cast<double*>(arena.get());
      for (size_t k = 0; k < sep_rows.size(); ++k) {
        block_ptrs[k] = base ? base + offsets[k] : nullptr;
      }
    }

    void SetZero() const {
      if (arena) {
        std::memset(arena.get(), 0,
                    static_cast<size_t>(total_rows) * reserved_cols *
                        sizeof(double));
      }
    }

    Eigen::Map<Eigen::MatrixXd> block(int k, int cols) const {
      return {block_ptrs[k], sep_rows[k], cols};
    }
  };
  mutable SeparatorScratch sep_scratch_;
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
};

}  // namespace conex
