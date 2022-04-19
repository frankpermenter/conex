#include <Eigen/Dense>
#include <vector>
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
class KKTSubsystem {
 public:
  KKTSubsystem(const std::vector<int>& vars) : variables_(vars) {}

  virtual void DoInitialize() = 0;
  const std::vector<int>& shared_variables() const;
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

  void SetParent(KKTSubsystem* child) {
    CONEX_DEMAND(child, "Received nullptr");
    CONEX_DEMAND(parent_ == nullptr, "Parent already assigned.");
    parent_ = child;
  }

  void AssembleAndFactor(bool only_assemble = false) {
    for (auto child : children_) {
      child->AssembleAndFactor(only_assemble);
    }
    if (!only_assemble) {
      DoEliminateSupernodeColumns();
      DoComputeSeparatorSchurComplement(); 
    }
    if (!IsRoot()) {
      DoScatterSeparatorSubmatrix();
    }
  }


  Eigen::MatrixXd SeparatorSchurComplement();
  void SetSupernodeColumns(const Eigen::MatrixXd& submatrix, std::vector<int>&rows, std::vector<int>&cols);
 protected:
    virtual void DoEliminateSupernodeColumns() = 0;
    virtual void DoComputeSeparatorSchurComplement() = 0;
    virtual void DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(Eigen::Ref<Eigen::MatrixXd> y) = 0;
    virtual void DoApplyInverseOfRightFactorOfSupernodeSubmatrix(Eigen::Ref<Eigen::MatrixXd> y) = 0;


    void IncrementSubmatrix(const Eigen::MatrixXd& S, 
                            const std::vector<int>& vars, size_t start_index) {
      if (start_index > vars.size()) {
        return;
      }

      size_t col_index = start_index;

      CONEX_ASSERT(vars.at(col_index) >= supernodes_.at(0), "Submatrix has been eliminated." ); 
      for (; col_index < vars.size(); col_index++)  {
        if (vars.at(col_index) > supernodes_.back()) {
          // The remaining columns must belong to our parent.
          break;
        }
        IncrementSupernodeColumn(S,  vars, col_index);
      }

      if (col_index < vars.size()) {
        CONEX_DEMAND(parent_, "Parent pointer is null.");
        parent_->IncrementSubmatrix(S, vars, col_index);
      }
    }

    void DoScatterSeparatorSubmatrix() {
      if (parent_) {
        parent_->IncrementSubmatrix(separator_schur_complement_, separators_,  0 /*start index*/);
      }
    }

    std::vector<int> separators_;
    std::vector<int> supernodes_;
    std::vector<int> variables_;
    KKTSubsystem* parent_ = nullptr;
    std::vector<KKTSubsystem*> children_;
    Eigen::MatrixXd  separator_schur_complement_;
  public:
    Eigen::MatrixXd  supernode_submatrix_;
    Eigen::MatrixXd  separator_rows_;

   //  L 
   //  SR^{-1}  D
   void ApplyInverseOfLeftFactor(Eigen::MatrixXd* x) {
      for (auto child : children_ ) {
        child->ApplyInverseOfLeftFactor(x);
      }

      Eigen::Ref<Eigen::MatrixXd> ref = x->middleRows(supernodes_.at(0),  
                                        supernodes_.back() -supernodes_.at(0) + 1);
      DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(ref);

      // Inverse of right factor
      Eigen::MatrixXd temp = ref;
      DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
      Eigen::MatrixXd residual = separator_rows_ * temp;
      for (int i = 0; i < residual.rows(); i++) {
        x->row(separators_[i]) -= residual.row(i);
      }
   }

   Eigen::MatrixXd SeparatorRows(Eigen::MatrixXd* x) {
      Eigen::MatrixXd separator_rows_of_x(separators_.size(), x->cols()); 
      for (int i = 0; i < separator_rows_of_x.rows(); i++) {
        separator_rows_of_x.row(i) = x->row(separators_[i]); 
      }
      return separator_rows_of_x;
   }

   bool IsRoot() const {
    return parent_ == nullptr;
   }

   //    L 
   // SL^T{-1}   R
   //
   //  L^T  L^{-1} S^T
   //       R
   void ApplyInverseOfRightFactor(Eigen::MatrixXd* x) {
      Eigen::Ref<Eigen::MatrixXd> ref = x->middleRows(supernodes_.at(0),  
                                        supernodes_.back() -supernodes_.at(0) + 1);
      if (!IsRoot()) {
        // Subtract L^{-1} S^T 
        Eigen::MatrixXd temp = separator_rows_.transpose() * SeparatorRows(x);
        DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(temp);
        ref.noalias() -= temp; 
      }

      DoApplyInverseOfRightFactorOfSupernodeSubmatrix(ref);
      for (auto child : children_ ) {
        child->ApplyInverseOfRightFactor(x);
      }
   }

    void IncrementSupernodeColumn(const Eigen::MatrixXd source_data, 
                                    const std::vector<int>& source_column_labels,
                                    int source_column_index) {
      int local_column_index = GetSupernodePosition(source_column_labels.at(source_column_index));
      size_t i = source_column_index;
      for (; i < source_column_labels.size(); i++) {
        if (source_column_labels.at(i) > supernodes_.back()) {
          break;
        }
        int local_row = GetSupernodePosition(source_column_labels.at(i));
        supernode_submatrix_(local_row, local_column_index) += source_data(i, source_column_index);
      }

      for (; i < source_column_labels.size(); i++) {
        if (source_column_labels.at(i) > separators_.back()) {
          break;
        }
        int local_row = GetSeparatorPosition(source_column_labels.at(i));
        supernode_submatrix_(local_row, local_column_index) += source_data(i, source_column_index);
      }
    }
    size_t GetSupernodePosition(size_t global_label) {
      for (size_t i = 0; i < supernodes_.size(); ++i) {
        if (supernodes_.at(i) == global_label) {
          return i;
        }
      }
      throw;
    }
    size_t GetSeparatorPosition(size_t global_label) {
      for (size_t i = 0; i < separators_.size(); ++i) {
        if (separators_.at(i) == global_label) {
          return i;
        }
      }
      throw;
    }
};






}
// namespace conex
