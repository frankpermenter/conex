#pragma once
#include <map>
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

class KKTSubsystemBase {
 public:
  KKTSubsystemBase(const std::vector<int>& shared_assembler_variables, int)
      : variables_(shared_assembler_variables) {}

  KKTSubsystemBase() : variables_({}) {}

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

  const std::vector<int>& shared_variables() const { return variables_; }

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

  std::vector<int>& variable_to_local_elimination_rank() {
    return variable_to_local_elimination_position_;
  }

  int ComputePostOrdering(int offset,
                          std::vector<int>* variable_to_elimination_position);

  void SetVariableOrdering(
      const std::vector<int>& variable_to_elimination_position);

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
      const KKTSubsystemBase* source) {
    return local_supernode_to_source_separator_.at(source);
  }
  std::vector<Offset> local_separator_to_source_separator(
      const KKTSubsystemBase* source) {
    return local_separator_to_source_separator_.at(source);
  }

  void ComputeSeparatorOffsets();
  bool variable_set_equals_sorted_separators() {
    return variable_set_equals_sorted_separators_;
  }
  bool variable_set_equals_sorted_supernodes() {
    return variable_set_equals_sorted_supernodes_;
  }

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
      Eigen::MatrixXd* output, Eigen::Ref<const Eigen::MatrixXd> input) const;

  bool IsRoot() const;
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

  std::vector<int> variable_to_local_elimination_position_;
  std::map<const KKTSubsystemBase*, std::vector<Offset>>
      local_supernode_to_source_separator_;
  std::map<const KKTSubsystemBase*, std::vector<Offset>>
      local_separator_to_source_separator_;

 protected:
  std::vector<int> separators_;
  std::vector<int> supernodes_;
  const std::vector<int> variables_;
  bool left_looking_ = true;
  bool variable_set_equals_sorted_supernodes_;
  bool variable_set_equals_sorted_separators_;
};

class KKTSubsystem : public KKTSubsystemBase {
 public:
  KKTSubsystem(const std::vector<int>& shared_assembler_variables,
               int number_of_private_variables)
      : KKTSubsystemBase(shared_assembler_variables,
                         number_of_private_variables) {}

  KKTSubsystem() {}

  void DoInitialize() override {
    supernode_submatrix_.resize(supernodes_.size(), supernodes_.size());
    separator_rows_.resize(separators_.size(), supernodes_.size());
    separator_schur_complement_.resize(separators_.size(), separators_.size());
  }

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
  Eigen::Ref<const Eigen::MatrixXd> separator_schur_complement()
      const override {
    return separator_schur_complement_;
  }
  Eigen::Ref<const Eigen::MatrixXd> separator_rows() const override {
    return separator_rows_;
  }

 private:
  Eigen::MatrixXd separator_schur_complement_;
  Eigen::MatrixXd supernode_submatrix_;
  Eigen::MatrixXd separator_rows_;
};

}  // namespace conex
