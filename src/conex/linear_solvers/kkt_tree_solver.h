#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include "conex/common/block_partition.h"
#include "conex/common/cone_constraint.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/tree_rhs.h"
#include "conex/linear_solvers/kkt_subsystem.h"
#include "conex/linear_solvers/assembler_adapter.h"
#include "conex/linear_solvers/tree_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

// Forward declaration.
class SupernodePartitionMatrix;

// BlockPartition adapter for the tree solver's supernodal partition.
// Each block corresponds to one supernode in the elimination tree.
// Scatter/gather use the elimination ordering's variable mapping.
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
  // Called once at FinalizeStructure time with elimination positions.
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

 private:
  friend class SymmetricLinearSystemTreeSolver;
  KKTSubsystemBase* subsystem_ = nullptr;
  int sn_start_ = 0;
  int sn_count_ = 0;
  int clique_id_ = -1;
  int parent_clique_id_ = -1;
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

  // Build VectorBlockContributions for A^T*v / A*x.
  std::vector<VectorBlockContribution> vblocks;
  auto* parent = subsystem_->parent();
  for (const auto& run : cached_runs_) {
    if (run.is_sn) {
      // Supernode run → own supernode block.
      vblocks.push_back({run.q_start, run.length,
                         clique_id_, run.local_start, true});
    } else if (parent) {
      // Separator run → map to parent sn/sep via parent's offset tables.
      // Scan parent's sn_offsets for ranges overlapping this run.
      for (const auto& off :
           parent->local_supernode_to_source_separator(subsystem_)) {
        int overlap_start = std::max(run.local_start, off.second);
        int overlap_end =
            std::min(run.local_start + run.length, off.second + off.size);
        if (overlap_start < overlap_end) {
          int q_offset = overlap_start - run.local_start;
          int p_offset = off.first + (overlap_start - off.second);
          vblocks.push_back({run.q_start + q_offset,
                             overlap_end - overlap_start,
                             parent_clique_id_, p_offset, true});
        }
      }
      // Scan parent's sep_offsets for ranges overlapping this run.
      for (const auto& off :
           parent->local_separator_to_source_separator(subsystem_)) {
        int overlap_start = std::max(run.local_start, off.second);
        int overlap_end =
            std::min(run.local_start + run.length, off.second + off.size);
        if (overlap_start < overlap_end) {
          int q_offset = overlap_start - run.local_start;
          int p_offset = off.first + (overlap_start - off.second);
          vblocks.push_back({run.q_start + q_offset,
                             overlap_end - overlap_start,
                             parent_clique_id_, p_offset, false});
        }
      }
    }
  }
  lazy.RegisterVectorContributions(vblocks);
}

template <typename BlockAssemblerT>
void SubmatrixContributor::Assemble(BlockAssemblerT& assembler) {
  CONEX_DEMAND(order_cached_, "Register must be called before Assemble.");
  assembler.ContributeBlocks(clique_id_);
}

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:
  int number_of_variables() const override;

  void FinalizeStructure(const CliqueTree& clique_tree, int rhs_cols = 1,
                         Arena* arena = nullptr);

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

  void ComputeSeparatorOffsets();
  void push_back(std::unique_ptr<AssemblerAdapter>&& system);

  // BlockPartition interface (from KKTSolverBase).
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

  // Number of subsystems (blocks in the partition).
  int num_subsystems() const { return static_cast<int>(subsystems_.size()); }

  // Clique size = supernode rows + separator rows for subsystem k.
  int clique_size(int k) const {
    return static_cast<int>(subsystems_[k]->supernodes().size() +
                            subsystems_[k]->separators().size());
  }

  // Access the elimination permutation.
  const Eigen::VectorXi& perm() const { return cached_perm_; }
  const Eigen::VectorXi& perm_inv() const { return cached_perm_inv_; }

  // Extract the CliqueTree (variable IDs in original numbering).
  CliqueTree GetCliqueTree() const {
    CliqueTree ct;
    int ns = num_subsystems();
    ct.supernodes.resize(ns);
    ct.separators.resize(ns);
    ct.node_to_parent.resize(ns, -1);
    ct.post_order_position_to_clique.resize(ns);
    // Map subsystem pointer → index for parent lookup.
    std::map<const void*, int> ptr_to_idx;
    for (int k = 0; k < ns; ++k)
      ptr_to_idx[subsystems_[k]] = k;
    for (int k = 0; k < ns; ++k) {
      ct.supernodes[k] = subsystems_[k]->supernodes();
      ct.separators[k] = subsystems_[k]->separators();
      auto* p = subsystems_[k]->parent();
      auto it = ptr_to_idx.find(p);
      ct.node_to_parent[k] = (it != ptr_to_idx.end()) ? it->second : -1;
      ct.post_order_position_to_clique[k] = k;
    }
    return ct;
  }

  // Pre-inject a subsystem for a specific clique index.  Must be called
  // before FinalizeStructure.  CreateSubsystems will use injected subsystems instead
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

  // Solve using a pre-populated separator scratch (no zero).
  void SolveBlockedInPlace(BlockPartition& supernodes,
                           SeparatorScratch& scratch) const;

  // --- KKTSolverBase overrides ---
  SolverRHS MakeSolverRHS(int cols = 1) override;
  SolverRHS AllocSolverRHS(Arena& arena, int cols = 1) override;
  RowSpace MakeRowSpace(int cols = 1) override;
  RowSpaceInfo GetRowSpaceInfo() const override;
  void MultiplyA(const SolverRHS& x, RowSpace& out) override;
  void AccumulateAtranspose(const RowSpace& v, SolverRHS& rhs) override;
  void AccumulateQx(const SolverRHS& x, SolverRHS& rhs) override;
  bool has_quadratic_cost() const override {
    return !quadratic_sub_assemblers_.empty();
  }
  void SetWeights(const RowSpace& w) override;
  void SetScaling(const RowSpace& w) override;
  RowSpace GetAffineTerm() override;

  // Gather unscattered separator data into supernode blocks.
  void GatherSeparators(SolverRHS& rhs) {
    GatherSeparators(*rhs.supernodes, *rhs.separators);
    rhs.blocks_fully_gathered = true;
  }

  double dot(SolverRHS& a, SolverRHS& b) override {
    if (!a.blocks_fully_gathered) GatherSeparators(a);
    if (!b.blocks_fully_gathered) GatherSeparators(b);
    return a.dot(b);
  }
  double dot(SolverRHS& a, const BlockVariable& bv) override {
    if (!a.blocks_fully_gathered) GatherSeparators(a);
    return a.dot(bv);
  }

  void SolveSolverRHS(SolverRHS& rhs) override {
    // The forward pass accumulates adapter contributions in sep upward
    // into parent sn/sep with the correct sign — see SolveBlockedInPlace.
    // If the caller has the RHS in "fully gathered" form (full b in sn,
    // sep stale), zero sep so the forward pass starts from a clean slate.
    // Otherwise (lazy form, sep contains adapter contributions), use sep
    // contents as-is.
    if (rhs.blocks_fully_gathered) {
      rhs.separators->SetZero();
      rhs.blocks_fully_gathered = false;
    }
    SolveBlockedInPlace(*rhs.supernodes, *rhs.separators);
  }

  // Register decomposed sub-assemblers for the generic interface.
  void RegisterLinearSubAssembler(class ConeConstraint* lc) {
    cone_constraints_.push_back(lc);
  }
  void RegisterQuadraticSubAssembler(class QuadraticCost* qc) {
    quadratic_sub_assemblers_.push_back(qc);
  }
  void RegisterEqualitySubAssembler(class EqualityConstraint* ec) {
    equality_sub_assemblers_.push_back(ec);
  }
  const std::vector<class EqualityConstraint*>& equality_sub_assemblers() const {
    return equality_sub_assemblers_;
  }

  // Return the equality RHS d as a SolverRHS with d at the dual variable
  // positions and zeros elsewhere.  Returns a zero SolverRHS if there are
  // no equality constraints.
  SolverRHS EqualityAffineTermRHS();

  // Accumulate saddle-point product [0 C'; C 0] * [x; lambda] into rhs.
  // Reads from x_rhs (which contains both primal and dual in the block
  // structure) and accumulates into rhs.
  void AccumulateCtranspose(const SolverRHS& x, SolverRHS& rhs);

  // Populate a sep scratch from a BlockPartition's supernode blocks.
  // Top-down: copies parent sn/sep into child separator scratch.
  void ScatterSeparators(const BlockPartition& supernodes,
                         SeparatorScratch& scratch) const;

  // Gather separator scratch back into supernode blocks (additive).
  // Bottom-up: accumulates child separator scratch into parent sn/sep.
  void GatherSeparators(BlockPartition& supernodes,
                        const SeparatorScratch& scratch) const;

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
  void AllocateArenaAndBind(int rhs_cols, Arena* arena = nullptr);

  std::vector<KKTSubsystemBase*> roots_;
  std::vector<KKTSubsystemBase*> subsystems_;
  std::vector<std::unique_ptr<KKTSubsystemBase>> owned_subsystems_;
  std::vector<std::unique_ptr<KKTSubsystemBase>> injected_subsystems_;
  std::vector<std::unique_ptr<AssemblerAdapter>>
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

  mutable SeparatorScratch sep_scratch_;
  mutable SeparatorScratch sep_scratch_out_;
  std::vector<class ConeConstraint*> cone_constraints_;
  std::vector<class EqualityConstraint*> equality_sub_assemblers_;
  std::vector<class QuadraticCost*> quadratic_sub_assemblers_;
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

  // Cached separator metadata (computed once at FinalizeStructure).
  std::vector<int> cached_sep_rows_;       // sep_rows[k] per subsystem
  std::vector<int> cached_sep_offsets_;    // double offset per subsystem
  int cached_sep_total_per_col_ = 0;       // sum of sep_rows

  // Build a SeparatorScratch from a data buffer + cached metadata.
  SeparatorScratch MakeSepScratch(double* buf, int cols);

  // Owned storage for SeparatorScratch instances and their block_ptrs arrays.
  std::vector<std::unique_ptr<SeparatorScratch>> owned_solver_rhs_scratches_;
  std::vector<std::unique_ptr<double*[]>> owned_sep_block_ptrs_;
  std::vector<std::unique_ptr<double[]>> owned_sep_buffers_;
};

}  // namespace conex
