#pragma once

#include <memory>
#include <vector>

#include "conex/common/block_partition.h"
#include "conex/common/clique_tree.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/gpu_tree_solver/gpu_device_arena.h"
#include "conex/gpu_tree_solver/gpu_extend_add.cuh"
#include <Eigen/Dense>

// Forward declarations for CUDA handles (avoid including CUDA headers here).
struct cusolverDnContext;
struct cublasContext;
typedef struct CUstream_st* cudaStream_t;

namespace conex {

// GPU implementation of the supernodal sparse Cholesky tree solver.
//
// Reuses the symbolic factorization (CliqueTree, elimination ordering,
// parent-child Offset maps) from the CPU tree solver. Numeric operations
// (assemble, factor, solve) run on the GPU using cuSOLVER and cuBLAS.
//
// Factorization proceeds level-by-level (leaves first, root last).
// All supernodes at the same tree depth are independent and processed
// as a batch of dense operations.
class GpuTreeSolver : public KKTSolverBase {
 public:
  GpuTreeSolver();
  ~GpuTreeSolver() override;

  // Non-copyable, movable.
  GpuTreeSolver(const GpuTreeSolver&) = delete;
  GpuTreeSolver& operator=(const GpuTreeSolver&) = delete;
  GpuTreeSolver(GpuTreeSolver&&) noexcept;
  GpuTreeSolver& operator=(GpuTreeSolver&&) noexcept;

  // Initialize from a symbolic CliqueTree (computed on CPU).
  // This computes the elimination ordering, level assignment,
  // arena layout, and extend-add offset tables.
  // Must be called before Assemble/Factor/Solve.
  void FinalizeStructure(const CliqueTree& clique_tree, int rhs_cols = 1);

  // Set the host-side assembler data for a supernode.
  // The caller provides the dense block data that will be copied
  // to the device during DoAssemble().
  // sn_idx: supernode index (in post-order).
  // data: column-major dense matrix of size total_size x total_size
  //       where total_size = sn_size + sep_size.
  void SetSupernodeData(int sn_idx, const Eigen::MatrixXd& data);

  // KKTSolverBase interface.
  int number_of_variables() const override { return num_vars_; }

  // Generic interface stubs (GPU solver uses raw matrix path, not constraints).
  RowSpace MakeRowSpace(int /*cols*/ = 1) override { return {}; }
  void MultiplyA(const SolverRHS&, RowSpace&) override {}
  void AccumulateAtranspose(const RowSpace&, SolverRHS&) override {}
  void AccumulateQx(const SolverRHS&, SolverRHS&) override {}
  void SetWeights(const RowSpace&) override {}
  RowSpace GetAffineTerm() override { return {}; }

  // Access internals for testing.
  const std::vector<SupernodeDescriptor>& descriptors() const {
    return descriptors_;
  }
  const std::vector<std::vector<int>>& levels() const { return levels_; }
  int num_levels() const { return static_cast<int>(levels_.size()); }

 private:
  void DoAssemble() override;
  bool DoFactor() override;
  bool DoAssembleAndFactor() override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool permute_to_elimination_order) const override;
  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order) const override;

  // Factor one level of the tree (all supernodes at this depth).
  bool FactorLevel(int level);

  // Forward solve: leaf to root.
  void ForwardSolve(double* d_rhs, int cols) const;

  // Backward solve: root to leaf.
  void BackwardSolve(double* d_rhs, int cols) const;

  // Symbolic data.
  CliqueTree clique_tree_;
  int num_vars_ = 0;
  int rhs_cols_ = 1;
  Eigen::VectorXi perm_;       // variable -> elimination position
  Eigen::VectorXi perm_inv_;   // elimination position -> variable

  // Per-supernode descriptors and level assignment.
  std::vector<SupernodeDescriptor> descriptors_;
  std::vector<std::vector<int>> levels_;  // levels_[k] = supernodes at depth k
  std::vector<int> sn_starts_;  // elimination position of first var per supernode

  // Device memory.
  GpuDeviceArena arena_;
  double* d_rhs_ = nullptr;        // device RHS/solution buffer
  int* d_info_ = nullptr;          // cuSOLVER status per supernode
  ScatterOp* d_scatter_ops_ = nullptr;  // device scatter op table

  // Per-level scatter op offsets into d_scatter_ops_.
  std::vector<int> scatter_op_offsets_;  // scatter_op_offsets_[level] = start
  std::vector<int> scatter_op_counts_;   // scatter_op_counts_[level] = count

  // Host staging buffer for assembly.
  std::vector<Eigen::MatrixXd> host_data_;

  // Per-supernode separator elimination positions (for gather/scatter in solve).
  int* d_sep_indices_ = nullptr;       // flat device array of all separator elim positions
  std::vector<int> sep_indices_offsets_;  // per-supernode offset into d_sep_indices_
  double* d_gather_buf_ = nullptr;     // contiguous buffer for gathered separator entries
  int max_sep_size_ = 0;

  // Persistent cuSOLVER workspace (avoids per-supernode cudaMalloc/cudaFree).
  double* d_potrf_work_ = nullptr;
  int potrf_work_size_ = 0;

  // Batched factorization: groups of same-size supernodes per level.
  struct BatchGroup {
    int sn_size;
    int sep_size;
    std::vector<int> indices;  // supernode indices in this group
  };
  std::vector<std::vector<BatchGroup>> level_groups_;  // [level] -> groups
  double** d_batch_ptrs_ = nullptr;  // device buffer for pointer arrays
  int* d_batch_sep_offsets_ = nullptr;  // device buffer for batched sep index offsets
  int max_batch_size_ = 0;
  int max_batch_gather_ = 0;  // max(sep_size * batch_size) for gather buffer

  // CUDA handles (mutable: solve is logically const but uses GPU state).
  mutable cusolverDnContext* cusolver_ = nullptr;
  mutable cublasContext* cublas_ = nullptr;
  mutable cudaStream_t stream_ = nullptr;


  // Pre-flattened scatter ops (host side, uploaded to d_scatter_ops_).
  std::vector<ScatterOp> host_scatter_ops_;
};

}  // namespace conex
