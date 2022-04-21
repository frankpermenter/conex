#pragma once
#include <vector>
#include <Eigen/Dense>

#include "conex/error_checking_macros.h"
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

//   KKTSystem
//
//      MakeTree(subsystem_shared_variables)
//      returns: elimination tree, 
//               fill-in.
//               post-ordering.
//
//      subsystem.Assemble(fill_in, ordering)
//
//      Solve(Residual) {  subsystem.Factor(); 
//                         subsystem.LeftFactorInverse, 
//                         subsystem.RightFactorInverse }
//
//
//      Solve(dual_weight) {  subsystem.Factor(); 
//                         subsystem.LeftFactorInverse, 
//                         subsystem.RightFactorInverse }
//
//    private:
//
//     linear_cost_;
//     subsystems_;
//     tree_;
// 


class KKTSubsystem {
 public:
  KKTSubsystem(const std::vector<int>& shared_assembler_variables,
               int number_of_private_variables) 
               : variables_(shared_assembler_variables), 
                 number_of_private_variables_(number_of_private_variables) {}

  virtual void DoInitialize() {
    supernode_submatrix_.resize(supernodes_.size(), supernodes_.size());
    separator_rows_.resize(separators_.size(), supernodes_.size());
    separator_schur_complement_.resize(separators_.size(), separators_.size());
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


  int ComputePostOrdering(int offset, std::vector<int>* variable_to_elimination_position);

  void SetVariableOrdering(const std::vector<int>& shared_variable_to_elimination_position);
  void SetPostOrdering(const std::vector<int>& shared_variable_to_elimination_position);


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

  void Assemble();

  void MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const;
  void AssembleAndFactor();

  KKTSubsystem* parent() const { return parent_; }

  void ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) const;
  void ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const;


  void IsDefiniteInSubtree(const std::vector<int>& vars,  
                           std::vector<int>*degree) const {
    DoIsDefinite(vars, degree);
    for (auto c : children_) {
      c->IsDefiniteInSubtree(supernodes_, degree);
    }
  }

  bool ValidateRoot() const {
    std::vector<int> is_definite(supernode_submatrix_.size());
    IsDefiniteInSubtree(supernodes_, &is_definite);
    for (auto c : is_definite) {
      if (!c) {
        return false;
      }
    }
    return true;
  }

 protected:
  void SetSupernodeColumns(const Eigen::MatrixXd& submatrix,
                           std::vector<int>& rows, std::vector<int>& cols);

  void DoIsDefinite(const std::vector<int>& vars, std::vector<int>* degree) const {} 
  virtual void DoEliminateSupernodeColumns() = 0;
  virtual void DoComputeSeparatorSchurComplement() = 0;
  virtual void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;
  virtual void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(
      Eigen::Ref<Eigen::MatrixXd> y) const = 0;

  bool IsRoot() const;

  KKTSubsystem* parent_ = nullptr;
  std::vector<KKTSubsystem*> children_;
  Eigen::MatrixXd separator_schur_complement_;
  Eigen::MatrixXd supernode_submatrix_;
  Eigen::MatrixXd separator_rows_;

  std::vector<int>& variable_to_local_elimination_rank()  {
    return variable_to_local_elimination_position_;
    //variable_to_elimination_position_.clear();
    //for (size_t i = 0; i < variables_.size(); i++) {
    //  variable_to_elimination_position_.push_back(i);
    //}
    //return variable_to_elimination_position_;
  }

  double& submatrix(int i, int j);

 private:
  // separators_ and supernodes_ are disjoint and their
  // union is a subset of shared_assembler_variables_
  std::vector<int> separators_;
  std::vector<int> supernodes_;
  const std::vector<int> variables_;
  std::vector<int> variable_to_local_elimination_position_;
  int number_of_private_variables_ = 0;

  void InplaceLeftMultiplyBySeparatorRowsTimesInverseOfRightFactor(
      Eigen::Ref<Eigen::MatrixXd>& temp) const;
  Eigen::MatrixXd SeparatorRows(const Eigen::MatrixXd& x) const;
  void IncrementSupernodeColumn(const Eigen::MatrixXd source_data,
                                const std::vector<int>& source_column_labels,
                                int source_column_index);
  size_t GetSupernodePosition(int global_label);
  size_t GetSeparatorPosition(int global_label);
  void DoScatterSeparatorSubmatrix() {
    if (parent_) {
      parent_->IncrementSubmatrix(separator_schur_complement_, separators_,
                                  0 /*start index*/);
    }
  }

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
};

}  // namespace conex
// namespace conex
