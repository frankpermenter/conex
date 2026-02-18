#pragma once
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "conex/debug_macros.h"
#include "conex/error_checking_macros.h"
#include <Eigen/Dense>
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
  virtual void Initialize(size_t num_supernodes, size_t num_separators) = 0;
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
  void Initialize(size_t num_supernodes, size_t num_separators) override;
  size_t RequiredArenaBytes(size_t num_supernodes,
                            size_t num_separators) const override;
  void BindArenaMemory(double* ptr, size_t bytes, size_t num_supernodes,
                       size_t num_separators) override;
  Eigen::Ref<Eigen::MatrixXd> supernode_submatrix() override;
  Eigen::Ref<Eigen::MatrixXd> separator_schur_complement() override;
  Eigen::Ref<Eigen::MatrixXd> separator_rows() override;
  Eigen::Ref<const Eigen::MatrixXd> supernode_submatrix() const override;
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement() const override;
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override;

 private:
  bool using_arena_memory_ = false;
  std::optional<AlignedMatrixMap> separator_schur_complement_map_;
  std::optional<AlignedMatrixMap> supernode_submatrix_map_;
  std::optional<AlignedMatrixMap> separator_rows_map_;
  Eigen::MatrixXd separator_schur_complement_storage_;
  Eigen::MatrixXd supernode_submatrix_storage_;
  Eigen::MatrixXd separator_rows_storage_;
};

class KKTSubsystemBase {
 public:
  std::vector<int> separators() const { return separators_; }
  std::vector<int> supernodes() const { return supernodes_; }
  virtual ~KKTSubsystemBase() = default;
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
  void SetSeparators(const std::vector<int>& separators) {
    separators_ = separators;
  }

  void SetSupernodes(const std::vector<int>& supernodes) {
    supernodes_ = supernodes;
  };

  void Initialize() { DoInitialize(); }
  void AddSupernode(int i) { supernodes_.push_back(i); }

  void AddSeparator(int i) { separators_.push_back(i); }

  int ComputePostOrdering(int offset,
                          std::vector<int>* variable_to_elimination_position);

  void SetVariableOrdering(
      const std::vector<int>& variable_to_elimination_position);
  void SetNumThreads(int num_threads) {
    CONEX_DEMAND(num_threads > 0, "num_threads must be positive.");
    num_threads_ = num_threads;
  }

  void AddChild(KKTSubsystemBase* child) {
    CONEX_DEMAND(child, "Received nullptr");
    children_.push_back(child);
    child->SetParent(this);
  }

  void Assemble();
  bool Factor();
  bool is_valid_leaf() { return DoIsValidLeaf(); }

  void MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const;
  void AddSparseMatrixTriplets(std::vector<Eigen::Triplet<double>>*) const;
  bool AssembleAndFactor();

  KKTSubsystemBase* parent() const { return parent_; }

  void ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) const;
  void ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const;
  void ReserveSolveWorkspace(int rhs_cols);

  void Reset() {
    parent_ = nullptr;
    children_.clear();
  }

  struct Offset {
    Offset(int x, int y, int z) : first(x), second(y), size(z) {}
    int first;
    int second;
    int size;
  };
  std::vector<Offset> local_supernode_to_source_separator(
      const KKTSubsystemBase* source) const {
    return local_supernode_to_source_separator_.at(source);
  }
  std::vector<Offset> local_separator_to_source_separator(
      const KKTSubsystemBase* source) const {
    return local_separator_to_source_separator_.at(source);
  }

  void ComputeSeparatorOffsets();
  bool variable_set_equals_sorted_separators() {
    return variable_set_equals_sorted_separators_;
  }
  bool variable_set_equals_sorted_supernodes() {
    return variable_set_equals_sorted_supernodes_;
  }
  virtual size_t RequiredArenaBytes() const { return 0; }
  virtual void BindArenaMemory(double* /*ptr*/, size_t /*bytes*/) {}

 private:
  virtual void DoInitialize(){};
  virtual bool DoEliminateSupernodeColumns() = 0;
  virtual void DoComputeSeparatorSchurComplement() = 0;
  virtual void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;
  virtual void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;
  virtual void DoMultiplyAndDecrementByOffDiagonalSubMatrix(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const;

  virtual void DoMultiplyByTransposeOfOffDiagonalSubMatrix(
      Eigen::Ref<Eigen::MatrixXd> output,
      Eigen::Ref<const Eigen::MatrixXd> input) const;

  bool IsRoot() const;
  void AccumulateColumnUpdate(
      const KKTSubsystemBase* target,
      Eigen::Ref<Eigen::MatrixXd> supernode_delta,
      Eigen::Ref<Eigen::MatrixXd> separator_delta) const;
  void ApplyLeftLookingChildUpdates();
  void ProvideColumnUpdate(KKTSubsystemBase* target);
  void ReceiveColumnUpdate(const KKTSubsystemBase* source,
                           size_t start_index_of_source);

  void SetParent(KKTSubsystemBase* parent) {
    CONEX_DEMAND(parent, "Received nullptr");
    CONEX_DEMAND(parent_ == nullptr, "Parent already assigned.");
    parent_ = parent;
  }

  virtual bool DoIsValidLeaf() { return true; }
  void DoComputeOffsets();
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
  std::vector<int> separators_;
  std::vector<int> supernodes_;
  bool left_looking_ = true;
  bool variable_set_equals_sorted_supernodes_;
  bool variable_set_equals_sorted_separators_;
  int num_threads_ = 1;
  int solve_workspace_cols_ = 0;
  mutable Eigen::MatrixXd solve_workspace1_;
  mutable Eigen::MatrixXd solve_workspace2_;
  mutable Eigen::MatrixXd solve_workspace3_;
};

class KKTSubsystem : public KKTSubsystemBase {
 public:
  KKTSubsystem();
  explicit KKTSubsystem(std::unique_ptr<KKTSubsystemStorage>&& storage);
  void SetStorage(std::unique_ptr<KKTSubsystemStorage>&& storage);

  void DoInitialize() override {
    storage_->Initialize(supernodes_.size(), separators_.size());
  }
  size_t RequiredArenaBytes() const override;
  void BindArenaMemory(double* ptr, size_t bytes) override;

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
