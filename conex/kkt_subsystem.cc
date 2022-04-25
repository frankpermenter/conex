#include "conex/kkt_subsystem.h"

#include "conex/debug_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

using T = KKTSubsystem;

// Iterate from the leafs of the tree upwards using recursion.
// At each leaf, we consider the triangular system
//
//  L           [x_supernodes] = b_[supernodes]
//  SR^{-1}  D  [x_seperator]    b_[separator]
//
// We then do one iteration of forward substitution
//
//  x_supernodes = L^{-1} b_supernodes
//  b_[separator] -=   SR^{-1} [x_supernodes]
void T::ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) const {
  // Do recursion all the way down to a leaf node.
  for (auto child : children_) {
    child->ApplyInverseOfLeftFactor(x);
  }
  if (supernodes_.size() == 0) {
    return;
  }

  // We have reached leaf. So solve for x_supernode in place.
  Eigen::Ref<Eigen::MatrixXd> x_supernodes = x.middleRows(
      supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(x_supernodes);

  // Update residual via separator_rows * LeftFactor^{-1} * x_{supernodes}
  if (separators_.size() > 0) {
    Eigen::MatrixXd temp = x_supernodes;
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
    for (int i = 0; i < separator_rows_.rows(); i++) {
      x.row(separators_[i]) -= separator_rows_.row(i) * temp;
    }
  }
}

Eigen::MatrixXd T::SeparatorRows(const Eigen::MatrixXd& x) const {
  Eigen::MatrixXd separator_rows_of_x(separators_.size(), x.cols());
  for (int i = 0; i < separator_rows_of_x.rows(); i++) {
    separator_rows_of_x.row(i) = x.row(separators_[i]);
  }
  return separator_rows_of_x;
}

bool T::IsRoot() const { return parent_ == nullptr; }

// Iterate from the root of the tree downwards using depth-first search. At each
// node, we consider the triangular system
//
//  R    L^{-1} S^T          [x_supernodes] = b_[supernodes]
//            R_{seperator}  [x_separator]    b_[separator]
//
// Since we have already solved for x_separator, we first
// update the residual via
//
//  b_[supernodes] -=  L^{-1} S^T [x_separator]
//
// We then compute x_supernodes = R^{-1} b_supernodes.
void T::ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) const {
  if (supernodes_.size() > 0) {
    Eigen::Ref<Eigen::MatrixXd> x_supernodes = x.middleRows(
        supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);

    // Update residual using x_separator computed by ascendants in tree.
    if (separators_.size() > 0) {
      Eigen::MatrixXd temp = separator_rows_.transpose() * SeparatorRows(x);
      DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(temp);
      x_supernodes.noalias() -= temp;
    }
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(x_supernodes);
  }

  for (auto child : children_) {
    child->ApplyInverseOfRightFactor(x);
  }
}

void T::IncrementSupernodeColumn(const Eigen::MatrixXd source_data,
                                 const std::vector<int>& source_column_labels,
                                 int source_column_index) {
  int local_column_index =
      GetSupernodePosition(source_column_labels.at(source_column_index));
  size_t i = source_column_index;
  for (; i < source_column_labels.size(); i++) {
    if (source_column_labels.at(i) > supernodes_.back()) {
      break;
    }
    int local_row = GetSupernodePosition(source_column_labels.at(i));
    supernode_submatrix_(local_row, local_column_index) +=
        source_data(i, source_column_index);
  }

  for (; i < source_column_labels.size(); i++) {
    if (source_column_labels.at(i) > separators_.back()) {
      break;
    }
    int local_row = GetSeparatorPosition(source_column_labels.at(i));
    supernode_submatrix_(local_row, local_column_index) +=
        source_data(i, source_column_index);
  }
}
size_t T::GetSupernodePosition(int global_label) {
  for (size_t i = 0; i < supernodes_.size(); ++i) {
    if (supernodes_.at(i) == global_label) {
      return i;
    }
  }
  throw;
}
size_t T::GetSeparatorPosition(int global_label) {
  for (size_t i = 0; i < separators_.size(); ++i) {
    if (separators_.at(i) == global_label) {
      return i;
    }
  }
  throw;
}

void T::MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const {
  for (auto child : children_) {
    child->MakeKKTMatrix(full_matrix);
  }
  for (size_t j = 0; j < supernodes_.size(); j++) {
    for (size_t i = 0; i < supernodes_.size(); i++) {
      DUMP(i);
      DUMP(j);
      DUMP(supernode_submatrix_(i, j));
      full_matrix->coeffRef(supernodes_.at(i), supernodes_.at(j)) =
          supernode_submatrix_(i, j);
    }
    for (size_t i = 0; i < separators_.size(); i++) {
      full_matrix->coeffRef(separators_.at(i), supernodes_.at(j)) =
          separator_rows_(i, j);
    }
  }
}

void T::AssembleAndFactor() {
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

void T::Assemble() {
  DoInitialize();
  for (auto child : children_) {
    child->Assemble();
  }
  if (!IsRoot()) {
    DoScatterSeparatorSubmatrix();
  }
}

//double& T::submatrix(int i, int j) {
//  if (j > i) {
//    std::swap(i, j);
//  }
//  int num_supernodes = supernode_submatrix_.rows();
//  if (i < num_supernodes && j < num_supernodes) {
//    return supernode_submatrix_(i, j);
//  } else {
//    if (j < num_supernodes) {
//      return separator_rows_(i - num_supernodes, j);
//    } else {
//      return separator_schur_complement_(i - num_supernodes,
//                                         j - num_supernodes);
//    }
//  }
//}

int T::ComputePostOrdering(int offset,
                           std::vector<int>* variable_to_elimination_position) {
  for (auto& child : children_) {
    offset =
        child->ComputePostOrdering(offset, variable_to_elimination_position);
  }
  for (auto& s : supernodes_) {
    variable_to_elimination_position->at(s) = offset++;
  }
  return offset;
};

void T::SetVariableOrdering(
    const std::vector<int>& shared_variable_to_elimination_position) {
  for (auto& s : supernodes_) {
    s = shared_variable_to_elimination_position.at(s);
  }
  for (auto& e : separators_) {
    e = shared_variable_to_elimination_position.at(e);
  }
  std::sort(supernodes_.begin(), supernodes_.end());
  std::sort(separators_.begin(), separators_.end());

  std::vector<int> variable_elimination_position = variables_;
  for (auto& v : variable_elimination_position) {
    v = shared_variable_to_elimination_position.at(v);
  }

  variable_to_local_elimination_position_.resize(variables_.size());
  for (size_t i = 0; i < variables_.size(); i++) {
    bool found = false;
    for (size_t j = 0; j < supernodes_.size(); j++) {
      if (variable_elimination_position.at(i) == supernodes_.at(j)) {
        variable_to_local_elimination_position_.at(i) = j;
        found = true;
        break;
      }
    }
    if (found) {
      continue;
    }
    for (size_t j = 0; j < separators_.size(); j++) {
      if (variable_elimination_position.at(i) == separators_.at(j)) {
        variable_to_local_elimination_position_.at(i) = j + supernodes_.size();
        found = true;
        break;
      }
    }
    if (!found) {
      throw;
    }
  }
};

  void T::IncrementSubmatrix(const Eigen::MatrixXd& S,
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
}  // namespace conex
