#include "conex/kkt_subsystem.h"

#include "conex/debug_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

using T = KKTSubsystemBase;

void T::DoMultiplyAndDecrementByOffDiagonalSubMatrix(
    Eigen::Ref<MatrixXd> output,  Eigen::Ref<const MatrixXd> input) const {
  for (int i = 0; i < separator_rows().rows(); i++) {
    output.row(separators()[i]) -= separator_rows().row(i) * input;
  }
}
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
    DoMultiplyAndDecrementByOffDiagonalSubMatrix(x, temp);
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

void T::DoMultiplyByTransposeOfOffDiagonalSubMatrix(
    Eigen::MatrixXd* output,  Eigen::Ref<const Eigen::MatrixXd> input) const {
  *output = separator_rows().transpose() * SeparatorRows(input);
}

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
      Eigen::MatrixXd temp; 
      DoMultiplyByTransposeOfOffDiagonalSubMatrix(&temp, x);
      DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(temp);
      x_supernodes.noalias() -= temp;
    }
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(x_supernodes);
  }

  for (auto child : children_) {
    child->ApplyInverseOfRightFactor(x);
  }
}

// Loop over the separators of source and record the intersections.
void T::ComputeOffsets(const KKTSubsystemBase* source, int start_index) {
  const auto& source_column_labels = source->separators();
  int source_separator_index = start_index;

  if (local_supernode_to_source_separator_[source].size() > 0) {
    return;
  }
  if (local_separator_to_source_separator_[source].size() > 0) {
    return;
  }
  for (; source_separator_index < source_column_labels.size(); source_separator_index++) {
    size_t i = source_separator_index;
    int local_row = GetSupernodePosition(source_column_labels.at(i));
    if (local_row != -1) {
      local_supernode_to_source_separator_[source].push_back({local_row, i});
    } else {
      break;
    }
  }

  for (int index = source_separator_index; index < source_column_labels.size(); index++) {
    int local_row = GetSeparatorPosition(source_column_labels.at(index));
    if (local_row != -1) {
      local_separator_to_source_separator_[source].push_back({local_row, index});
    } else {
      throw;
    }
  }

  if (source_separator_index < source_column_labels.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ComputeOffsets(source, source_separator_index);
  }
}



void T::IncrementSupernodeColumn(const Eigen::MatrixXd& source_data,
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
    supernode_submatrix()(local_row, local_column_index) +=
        source_data(i, source_column_index);
  }

  for (; i < source_column_labels.size(); i++) {
    if (source_column_labels.at(i) > separators_.back()) {
      break;
    }
    int local_row = GetSeparatorPosition(source_column_labels.at(i));
    separator_rows()(local_row, local_column_index) +=
        source_data(i, source_column_index);
  }
}

int T::GetSupernodePosition(int global_label) {
  for (size_t i = 0; i < supernodes_.size(); ++i) {
    if (supernodes_.at(i) == global_label) {
      return i;
    }
  }
  return -1;
}

int T::GetSeparatorPosition(int global_label) {
  for (size_t i = 0; i < separators_.size(); ++i) {
    if (separators_.at(i) == global_label) {
      return i;
    }
  }
  return -1;
}

void T::MakeKKTMatrix(Eigen::MatrixXd* full_matrix) const {
  for (auto child : children_) {
    child->MakeKKTMatrix(full_matrix);
  }
  for (size_t j = 0; j < supernodes_.size(); j++) {
    for (size_t i = 0; i < supernodes_.size(); i++) {
      (*full_matrix)(supernodes_.at(i), supernodes_.at(j)) =
          supernode_submatrix()(i, j);
    }
    for (size_t i = 0; i < separators_.size(); i++) {
      (*full_matrix)(separators_.at(i), supernodes_.at(j)) =
          separator_rows()(i, j);
    }
  }
}

bool left_looking = false;
bool T::AssembleAndFactor() {
  DoInitialize();
  for (auto child : children_) {
    if (!child->AssembleAndFactor()) {
      return false;
    }
    if (left_looking) {
      child->ProvideColumnUpdate(this);
    }
  }
  if (!DoEliminateSupernodeColumns()) {
    return false;
  }
  DoComputeSeparatorSchurComplement();
  if (!IsRoot() && !left_looking) {
    DoScatterSeparatorSubmatrix();
  }
  return true;
}

void T::Assemble() {
  DoInitialize();
  for (auto child : children_) {
    child->Assemble();
    if (left_looking) {
      child->ProvideColumnUpdate(this);
    }
  }

  if (!IsRoot() && !left_looking) {
    DoScatterSeparatorSubmatrix();
  }
}

bool T::Factor() {
  for (auto& child : children_) {
    if (!child->Factor()) {
      return false;
    }
    if (left_looking) {
      child->ProvideColumnUpdate(this);
    }
  }
  if (!DoEliminateSupernodeColumns()) {
    return false;
  }
  // We assume that Assemble() has been called and already
  // scattered the separator sub-matrix.
  separator_schur_complement().setZero();
  DoComputeSeparatorSchurComplement();
  if (!IsRoot() && !left_looking) {
    DoScatterSeparatorSubmatrix();
  }
  return true;
}

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

void T::DoComputeOffsets() {
  if (parent_ && separators_.size() > 0) {
    parent_->ComputeOffsets(this, 0 /*start index*/);
  }
}

void T::ReceiveColumnUpdate(const KKTSubsystemBase* source, int start_index) {
  const auto& vars = source->separators();
  if (start_index > vars.size()) {
    return;
  }
  size_t col_index = start_index;

#if 1
  for (auto& c : local_supernode_to_source_separator_.at(source)) {
    for (auto& r : local_supernode_to_source_separator_.at(source)) {
      supernode_submatrix()(r.first, c.first) += source->separator_schur_complement()(r.second, c.second);
    }
    for (auto& r : local_separator_to_source_separator_[source]) {
      separator_rows()(r.first, c.first) += source->separator_schur_complement()(r.second, c.second);
    }
  }
  col_index = start_index;
  for (; col_index < vars.size(); col_index++) {
    if (vars.at(col_index) > supernodes_.back()) {
      // The remaining columns must belong to our parent.
      break;
    }
  }
  #else
  col_index = start_index;
  for (; col_index < vars.size(); col_index++) {
    if (vars.at(col_index) > supernodes_.back()) {
      // The remaining columns must belong to our parent.
      break;
    }
    IncrementSupernodeColumn(source->separator_schur_complement(), vars, col_index);
  }
  #endif


  if (col_index < vars.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ReceiveColumnUpdate(source, col_index);
  }
}

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

void T::DoScatterSeparatorSubmatrix() {
  if (parent_ && separators_.size() > 0) {
    parent_->ComputeOffsets(this, 0);
    parent_->ReceiveColumnUpdate(this, 0 /*start index*/);
  }
}

// Update target columns with local separator schur complement information.
// We update target column i if their is a local separator pair (j, i), 
// with (j \ge i).
void T::ProvideColumnUpdate(KKTSubsystemBase* target) {
  const std::vector<int>& target_supernodes = target->supernodes();
  const std::vector<int>& target_separators = target->separators();
  Eigen::Ref<MatrixXd> target_supernode_submatrix = target->supernode_submatrix();
  Eigen::Ref<MatrixXd> target_separator_rows = target->separator_rows();
  if (separators_.size() == 0 ||  target_supernodes.at(0) > separators_.back()) {
    return;
  }

  for (size_t i = 0;  i < target_supernodes.size(); i++) {
    int local_position_i = GetSeparatorPosition(target_supernodes.at(i));
    if (local_position_i == -1) {
      continue;
    }
    for (size_t j = i;  j < target_supernodes.size(); j++) {
      int local_position_j = GetSeparatorPosition(target_supernodes.at(j));
      if (local_position_j == -1) {
        continue;
      }
      if (local_position_j < local_position_i) {
      // we can't swap without invalidating beginning of loop.
      throw;
      }
      target_supernode_submatrix(j, i) += separator_schur_complement()(local_position_j, 
                                                                    local_position_i);
    }

    if (target_separators.size() == 0 ||  target_separators.at(0) > separators_.back()) {
      continue;
    }

    for (size_t j = 0;  j < target_separators.size(); j++) {
      int local_position_j = GetSeparatorPosition(target_separators.at(j));
      if (local_position_j == -1) {
        continue;
      }
      if (local_position_j < local_position_i) {
      throw;
      }
      target_separator_rows(j, i) += separator_schur_complement()(local_position_j, 
                                                                 local_position_i);
    }
  }

  for (auto& c : children_) {
    c->ProvideColumnUpdate(target);
  }
}

}  // namespace conex
