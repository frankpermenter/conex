#pragma once
#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "conex/common/arena_allocatable.h"
#include "conex/common/debug_macros.h"
#include "conex/common/error_checking_macros.h"
#include <Eigen/Core>
#include <Eigen/Sparse>
// A KKT sub-system is a symmetric system of linear equations of the
// form:
//
//  H  A'  B' [dx]
//  A -W   0  [dl]
//  B  0   0  [vv]
//
// It arises from a cone program of the form:
//
// min f(x) + c'x
//     Bx = f
//     b - Ax \in K
//
// where H denotes the Hessian of H and W is a weight matrix
// associated with the cone K.
//
// Two KKT sub-systems are linked by the variable dx. Given $N$ systems,
// goal we provide helper methods for factoring
//
// \sum^N_i (H + AWA')_i  B1', B2', .. BN
//  B1                     0    0    .. 0
//  B2                     0    0    .. 0
//  ..                               ..
//  BN                     0    0       0
//
// For this, we arrange
//
//    A   B
//    C   D
//
// Our goal is to recursively factor this matrix block-wise.
// Towards this, suppose A = E F and let GH = D - B' A^{-1} B.
// Then a block factorization is given by
//
//    [ E         0]   [   F    E^{-1} B ]
//
//    [ B' F^{-1}  G]  [   0    H        ]
//
// Moreover, a linear system solve is given by
//
//     z1 = E^{-1}  y1
//     z2 = G^{-1} (y2 - B' F^{-1} z1)
//
//     x2 = H^{-1} z2
//     x1 = F^{-1} z1
//
// We associate each block-column with subsets of variables of a KKT subsystem.
//
//  K = H_i + A_iWA_i  B^T_i
//      B_i
//
// We assume that the KKT subsystem can
//  - Add i to its list of shared_variables.
//  - Partition shared_variables into [N, S] and partition the entire
//  - matrix into
//
//    K = K_{P, P },
//        K_{N, P },  K_{N, N}
//                 ,  K_{S, N}, K_{S, N}
//
//    where P denotes private variables
//  - Perform random access updates of K_{N, N} and K_{S, N}.
//  - Compute a factorization K_{P \cup N}  = EF.
//  - Left multiply by E^{-1}
//  - Right multiply by F^{-1}
//  - Construct: K_{S} F^{-1} E^{-1} K_S^T

namespace conex {

class KKTSubsystemStorage {
 public:
  virtual ~KKTSubsystemStorage() = default;
  virtual size_t RequiredArenaBytes(size_t num_supernodes,
                                    size_t num_separators) const = 0;
  virtual void BindArenaMemory(double* ptr, size_t bytes, size_t num_supernodes,
                               size_t num_separators) = 0;
  virtual Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() = 0;
  virtual Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() = 0;
  virtual Eigen::Ref<Eigen::MatrixXd> separator_rows() = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> separator_rows() const = 0;
};

class DenseKKTSubsystemStorage final : public KKTSubsystemStorage {
 public:
  using AlignedMatrixMap =
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>,
                 Eigen::Aligned>;
  size_t RequiredArenaBytes(size_t num_supernodes,
                            size_t num_separators) const override;
  void BindArenaMemory(double* ptr, size_t bytes, size_t num_supernodes,
                       size_t num_separators) override;

  AlignedMatrixMap& supernode_map() { return supernode_submatrix_; }

  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() override {
    return supernode_submatrix_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() override {
    return separator_schur_complement_;
  }
  Eigen::Ref<Eigen::MatrixXd> separator_rows() override {
    return separator_rows_;
  }
  Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const override {
    return supernode_submatrix_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement() const override {
    return separator_schur_complement_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override {
    return separator_rows_;
  }

 private:
  AlignedMatrixMap supernode_submatrix_{nullptr, 0, 0};
  AlignedMatrixMap separator_rows_{nullptr, 0, 0};
  AlignedMatrixMap separator_schur_complement_{nullptr, 0, 0};
};

class KKTSubsystemBase : public ArenaAllocatable {
 public:
  std::vector<int> separators() const { return separators_; }
  std::vector<int> supernodes() const { return supernodes_; }
  virtual Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() = 0;
  virtual Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() = 0;
  virtual Eigen::Ref<Eigen::MatrixXd> separator_rows() = 0;
  //  virtual Eigen::Ref<Eigen::MatrixXd> separator_columns() = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const = 0;
  virtual Eigen::Ref<const Eigen::MatrixXd> separator_rows() const = 0;
  //  virtual Eigen::Ref<const Eigen::MatrixXd> separator_columns() const = 0;

  void SetFactorizationMode(bool left_looking) { left_looking_ = left_looking; }
  void SetScatterToParent(bool enable) { scatter_to_parent_ = enable; }
  void SetSeparators(const std::vector<int>& separators) {
    separators_ = separators;
  }

  void SetSupernodes(const std::vector<int>& supernodes) {
    supernodes_ = supernodes;
  };

  void Initialize() { DoInitialize(); }

  void SetVariableOrdering(
      const std::vector<int>& variable_to_elimination_position);
  void SetNumThreads(int num_threads) {
    CONEX_DEMAND(num_threads > 0, "num_threads must be positive.");
    num_threads_ = num_threads;
  }

  const std::vector<KKTSubsystemBase*>& children() const { return children_; }

  void Assemble();
  bool Factor();
  void MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const;
  bool AssembleAndFactor();
  bool last_factor_failed() const { return last_factor_failed_; }

  KKTSubsystemBase* parent() const { return parent_; }

  void ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) const;
  void ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const;
  // Block-partitioned solve (no global vector).
  void ForwardSolveBlocked(Eigen::Ref<Eigen::MatrixXd> sn,
                           Eigen::Ref<Eigen::MatrixXd> sep) const;
  void BackwardSolveBlocked(Eigen::Ref<Eigen::MatrixXd> sn,
                            Eigen::Ref<const Eigen::MatrixXd> sep) const;
  struct Offset {
    Offset(int x, int y, int z) : first(x), second(y), size(z) {}
    int first;
    int second;
    int size;
  };
  const std::vector<Offset>& local_supernode_to_source_separator(
      const KKTSubsystemBase* source) const {
    return local_supernode_to_source_separator_.at(source);
  }
  const std::vector<Offset>& local_separator_to_source_separator(
      const KKTSubsystemBase* source) const {
    return local_separator_to_source_separator_.at(source);
  }

  void ComputeSeparatorOffsets();
  virtual void MarkIndefinite() {}

  // Virtual scatter hooks for supernode updates.  Scatter/PartialScatter
  // and GatherFromChildren route through these instead of writing directly
  // to supernode_submatrix().  Subclasses (e.g., diagonal-only subsystems)
  // can override to intercept or validate writes.
  virtual void AccumulateIntoSupernode(
      Eigen::Ref<const Eigen::MatrixXd> delta);
  virtual void BlockwiseAccumulateIntoSupernode(
      Eigen::Ref<const Eigen::MatrixXd> source,
      const std::vector<Offset>& row_offsets,
      const std::vector<Offset>& col_offsets);
  // Supernode dimensions for sizing temporaries (may differ from
  // supernode_submatrix() shape for structured subsystems).
  virtual std::pair<int, int> supernode_dimensions() const;


  // Bind externally-owned memory for solve workspaces.
  // The tree solver calls this to consolidate all workspace allocations.
  void BindSolveWorkspace(double* ws1, int ws1_rows, int ws1_cols,
                          double* ws2, int ws2_rows, int ws2_cols,
                          double* ws3, int ws3_rows, int ws3_cols);

 private:
  friend class SymmetricLinearSystemTreeSolver;
  virtual void DoInitialize(){}
  virtual bool DoEliminateSupernodeColumns() = 0;
  virtual void DoComputeSeparatorSchurComplement() = 0;
  virtual void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;
  virtual void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;
  virtual void DoMultiplyAndDecrementByOffDiagonalSubMatrix(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const;


  bool IsRoot() const;
  void AccumulateColumnUpdate(
      const KKTSubsystemBase* target,
      Eigen::Ref<Eigen::MatrixXd> supernode_delta,
      Eigen::Ref<Eigen::MatrixXd> separator_delta,
      Eigen::Ref<Eigen::MatrixXd> separator_schur_delta) const;
  void GatherFromChildren();
  void ProvideColumnUpdate(KKTSubsystemBase* target);
  void ReceiveColumnUpdate(const KKTSubsystemBase* source,
                           size_t start_index_of_source);

  void AddChild(KKTSubsystemBase* child) {
    CONEX_DEMAND(child, "Received nullptr");
    children_.push_back(child);
    child->SetParent(this);
  }

  void Reset() {
    parent_ = nullptr;
    children_.clear();
  }

  void SetParent(KKTSubsystemBase* parent) {
    CONEX_DEMAND(parent, "Received nullptr");
    CONEX_DEMAND(parent_ == nullptr, "Parent already assigned.");
    parent_ = parent;
  }

  void DoScatterSeparatorSubmatrix();
  void ComputeOffsets(const KKTSubsystemBase* source,
                      int source_separators_start);
  KKTSubsystemBase* parent_ = nullptr;
  std::vector<KKTSubsystemBase*> children_;

  std::map<const KKTSubsystemBase*, std::vector<Offset>>
      local_supernode_to_source_separator_;
  std::map<const KKTSubsystemBase*, std::vector<Offset>>
      local_separator_to_source_separator_;

 protected:
  // Computes E^{-1} S^T x_sep and writes to output.
  // Default uses separator_rows()^T (correct when separator_rows stores
  // S F^{-1} and E^{-1} F^T = I, e.g. LLT where S L^{-T} is cached).
  // Kernels where E^{-1} S^T differs from (S F^{-1})^T must override.
  virtual void DoBackwardScatter(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const;
  virtual void DoBackwardScatterFromGatheredSeparator(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> gathered_sep) const;
  virtual void DoMultiplyByTransposeOfOffDiagonalSubMatrix(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const;
  std::vector<int> separators_;
  std::vector<int> supernodes_;
  bool left_looking_ = true;
  bool scatter_to_parent_ = false;
  bool variable_set_equals_sorted_supernodes_;
  bool variable_set_equals_sorted_separators_;
  int num_threads_ = 1;
  // Atomic counter for leaf-parallel factorization.
  std::atomic<int> pending_children_{0};
  int solve_workspace_cols_ = 0;
  mutable double* ws1_data_ = nullptr;
  mutable double* ws2_data_ = nullptr;
  mutable double* ws3_data_ = nullptr;
  int ws1_rows_ = 0, ws2_rows_ = 0, ws3_rows_ = 0;
  bool ws_arena_bound_ = false;
  bool last_factor_failed_ = false;
  // Owned storage (used when NOT arena-bound).
  mutable Eigen::MatrixXd solve_workspace1_;
  mutable Eigen::MatrixXd solve_workspace2_;
  mutable Eigen::MatrixXd solve_workspace3_;

  // Workspace accessors — return a mutable Map view.
  Eigen::Map<Eigen::MatrixXd> ws1() const {
    if (ws_arena_bound_) {
      return {ws1_data_, ws1_rows_, solve_workspace_cols_};
    }
    return {solve_workspace1_.data(), solve_workspace1_.rows(),
            solve_workspace1_.cols()};
  }
  Eigen::Map<Eigen::MatrixXd> ws2() const {
    if (ws_arena_bound_) {
      return {ws2_data_, ws2_rows_, solve_workspace_cols_};
    }
    return {solve_workspace2_.data(), solve_workspace2_.rows(),
            solve_workspace2_.cols()};
  }
  Eigen::Map<Eigen::MatrixXd> ws3() const {
    if (ws_arena_bound_) {
      return {ws3_data_, ws3_rows_, solve_workspace_cols_};
    }
    return {solve_workspace3_.data(), solve_workspace3_.rows(),
            solve_workspace3_.cols()};
  }
};

class KKTSubsystem : public KKTSubsystemBase {
 public:
  KKTSubsystem();
  explicit KKTSubsystem(std::unique_ptr<KKTSubsystemStorage>&& storage);

  size_t RequiredArenaBytes() const override;
  void BindArenaMemory(double* ptr, size_t bytes) override;

  DenseKKTSubsystemStorage& dense_storage() {
    return static_cast<DenseKKTSubsystemStorage&>(*storage_);
  }

  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() override {
    return storage_->supernode_submatrix();
  }
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() override {
    return storage_->separator_schur_complement();
  }
  Eigen::Ref<Eigen::MatrixXd> separator_rows() override {
    return storage_->separator_rows();
  }
  Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const override {
    return static_cast<const KKTSubsystemStorage&>(*storage_)
        .supernode_submatrix();
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const override {
    return static_cast<const KKTSubsystemStorage&>(*storage_)
        .separator_schur_complement();
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override {
    return static_cast<const KKTSubsystemStorage&>(*storage_).separator_rows();
  }

 private:
  std::unique_ptr<KKTSubsystemStorage> storage_;
};

}  // namespace conex
