#pragma once
#include <cstdint>
#include <cstdlib>
#include <memory>

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
};

}  // namespace conex
