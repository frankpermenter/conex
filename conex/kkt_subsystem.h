#include <vector>
#include "conex/error_checking_macros.h"
#include <Eigen/Dense>

#include "conex/debug_macros.h"
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
class KKTSubsystem {
 public:
  KKTSubsystem(const std::vector<int>& vars) : variables_(vars) {}

  virtual void DoInitialize() {
    supernode_submatrix_.resize(supernodes_.size(), supernodes_.size());
    separator_rows_.resize(separators_.size(), supernodes_.size());
  }
  const std::vector<int>& shared_variables() const;
  const std::vector<int>& supernodes() { return supernodes_; }
  void AddVariables(const std::vector<int>& i);
  void SetSeparators(const std::vector<int>& separators) {
    separators_ = separators;
  };
  void SetSupernodes(const std::vector<int>& supernodes) {
    supernodes_ = supernodes;
  };

  void AddChild(KKTSubsystem* child) {
    CONEX_DEMAND(child, "Received nullptr");
    children_.push_back(child);
    child->SetParent(this);
  }

  void SetParent(KKTSubsystem* parent) {
    CONEX_DEMAND(parent, "Received nullptr");
    CONEX_DEMAND(parent_ == nullptr, "Parent already assigned.");
    parent_ = parent;
  }

  void Assemble() {
    DoInitialize();
    for (auto child : children_) {
      child->Assemble();
    }
    if (!IsRoot()) {
      DoScatterSeparatorSubmatrix();
    }
  }

  void MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const {
    for (auto child : children_) {
      child->MakeKKTMatrix(full_matrix);
    }
    for (int j = 0; j < supernodes_.size(); j++) {
      for (int i = 0; i < supernodes_.size(); i++) {
        full_matrix->coeffRef(supernodes_.at(i), supernodes_.at(j)) =
            supernode_submatrix_(i, j);
      }
      for (int i = 0; i < separators_.size(); i++) {
        full_matrix->coeffRef(separators_.at(i), supernodes_.at(j)) =
            separator_rows_(i, j);
      }
    }
  }

  void AssembleAndFactor() {
    DoInitialize();
    for (auto child : children_) {
      child->AssembleAndFactor();
    }
    DoEliminateSupernodeColumns();
    DoComputeSeparatorSchurComplement();
    if (!IsRoot()) {
      DoScatterSeparatorSubmatrix();
    }
  }

  KKTSubsystem* parent() const { return parent_; }
  Eigen::MatrixXd SeparatorSchurComplement();
  void SetSupernodeColumns(const Eigen::MatrixXd& submatrix,
                           std::vector<int>& rows, std::vector<int>& cols);

  void ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x);
  void ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x);

 protected:


  virtual void DoEliminateSupernodeColumns() = 0;
  virtual void DoComputeSeparatorSchurComplement() = 0;
  virtual void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) = 0;
  virtual void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) = 0;


  bool IsRoot() const; 

  void IncrementSubmatrix(const Eigen::MatrixXd& S,
                          const std::vector<int>& vars, size_t start_index) {
    if (start_index > vars.size()) {
      return;
    }

    size_t col_index = start_index;
    CONEX_ASSERT(vars.at(col_index) >= supernodes_.at(0),
                 "Submatrix has been eliminated.");
    for (; col_index < vars.size(); col_index++) {
      if (vars.at(col_index) > supernodes_.back()) {
        // The remaining columns must belong to our parent.
        break;
      }
      IncrementSupernodeColumn(S, vars, col_index);
    }

    if (col_index < vars.size()) {
      CONEX_DEMAND(parent_, "Parent pointer is null.");
      parent_->IncrementSubmatrix(S, vars, col_index);
    }
  }

  void DoScatterSeparatorSubmatrix() {
    if (parent_) {
      parent_->IncrementSubmatrix(separator_schur_complement_, separators_,
                                  0 /*start index*/);
    }
  }

  KKTSubsystem* parent_ = nullptr;
  std::vector<KKTSubsystem*> children_;
  Eigen::MatrixXd separator_schur_complement_;
  Eigen::MatrixXd supernode_submatrix_;
  Eigen::MatrixXd separator_rows_;

 private:
  std::vector<int> separators_;
  std::vector<int> supernodes_;
  std::vector<int> variables_;
 public:


 private:
  Eigen::MatrixXd SeparatorRows(const Eigen::MatrixXd& x) const;
  void IncrementSupernodeColumn(const Eigen::MatrixXd source_data,
                                const std::vector<int>& source_column_labels,
                                int source_column_index);
  size_t GetSupernodePosition(size_t global_label);
  size_t GetSeparatorPosition(size_t global_label);
};

}  // namespace conex
// namespace conex
