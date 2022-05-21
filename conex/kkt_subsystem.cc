#define CONEX_ENABLE_TIMER 0
#include "conex/kkt_subsystem.h"

#include "conex/debug_macros.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

using T = KKTSubsystemBase;
using std::vector;

namespace {

Eigen::MatrixXd Submatrix(const Eigen::MatrixXd& x,
                          const std::vector<int>& rows) {
  Eigen::MatrixXd separator_rows_of_x(rows.size(), x.cols());
  for (int i = 0; i < separator_rows_of_x.rows(); i++) {
    separator_rows_of_x.row(i) = x.row(rows[i]);
  }
  return separator_rows_of_x;
}

KKTSubsystemBase::Offset GetOverlappingSegment(
    const std::vector<int>& supernodes_, const std::vector<int>& variables,
    size_t global_label) {
  KKTSubsystemBase::Offset y{0, 0, 0};
  y.second = global_label;
  size_t start = 0;
  size_t size = 0;
  for (; start < supernodes_.size(); ++start) {
    if (supernodes_.at(start) == variables.at(global_label)) {
      size = 1;
      break;
    }
  }
  if (size >= 1) {
    while (start + size < supernodes_.size() &&
           global_label + size < variables.size() &&
           supernodes_.at(start + size) == variables.at(global_label + size)) {
      size++;
    }
    y.first = start;
    y.size = size;
  }
  return y;
}

void Update(const KKTSubsystemBase* source, KKTSubsystemBase* destination) {
  for (auto& c : destination->local_supernode_to_source_separator(source)) {
    for (auto& r : destination->local_supernode_to_source_separator(source)) {
      destination->supernode_submatrix().block(r.first, c.first, r.size,
                                               c.size) +=
          source->separator_schur_complement().block(r.second, c.second, r.size,
                                                     c.size);
    }
    for (auto& r : destination->local_separator_to_source_separator(source)) {
      destination->separator_rows().block(r.first, c.first, r.size, c.size) +=
          source->separator_schur_complement().block(r.second, c.second, r.size,
                                                     c.size);
    }
  }
}
}  // namespace

void T::DoMultiplyAndDecrementByOffDiagonalSubMatrix(
    Eigen::Ref<MatrixXd> output, Eigen::Ref<const MatrixXd> input) const {
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

bool T::IsRoot() const { return parent_ == nullptr; }

void T::DoMultiplyByTransposeOfOffDiagonalSubMatrix(
    Eigen::MatrixXd* output, Eigen::Ref<const Eigen::MatrixXd> input) const {
  *output = separator_rows().transpose() * Submatrix(input, separators_);
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
void T::ComputeOffsets(const KKTSubsystemBase* descendant, int start_index) {
  const auto& source_column_labels = descendant->separators();
  size_t source_separator_index = start_index;

  if (local_supernode_to_source_separator_.find(descendant) !=
      local_supernode_to_source_separator_.end()) {
    return;
  } else {
    local_supernode_to_source_separator_[descendant];
  }

  if (local_separator_to_source_separator_.find(descendant) !=
      local_separator_to_source_separator_.end()) {
    return;
  } else {
    local_separator_to_source_separator_[descendant];
  }

  while (source_separator_index < source_column_labels.size()) {
    auto local_row = GetOverlappingSegment(supernodes_, source_column_labels,
                                           source_separator_index);
    if (local_row.size != 0) {
      local_supernode_to_source_separator_[descendant].push_back(local_row);
      source_separator_index += local_row.size;
    } else {
      break;
    }
  }

  size_t index = source_separator_index;
  while (index < source_column_labels.size()) {
    auto local_row =
        GetOverlappingSegment(separators_, source_column_labels, index);
    if (local_row.size != 0) {
      local_separator_to_source_separator_[descendant].push_back(local_row);
      index += local_row.size;
    } else {
      // By the running intersection property, all separators must be present.
      std::runtime_error("Tree fails the running intersection property.");
    }
  }

  if (source_separator_index < source_column_labels.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ComputeOffsets(descendant, source_separator_index);
  }
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

bool T::AssembleAndFactor() {
  DoInitialize();
  for (auto child : children_) {
    if (!child->AssembleAndFactor()) {
      return false;
    }
    if (left_looking_) {
      START_TIMER(Update)
      child->ProvideColumnUpdate(this);
      END_TIMER
    }
  }
  START_TIMER(Eliminate)
  if (!DoEliminateSupernodeColumns()) {
    return false;
  }
  END_TIMER

  START_TIMER(ComputeSep)
  DoComputeSeparatorSchurComplement();
  END_TIMER
  if (!IsRoot() && !left_looking_) {
    START_TIMER(Scatter)
    DoScatterSeparatorSubmatrix();
    END_TIMER
  }
  return true;
}

void T::Assemble() {
  DoInitialize();
  for (auto& child : children_) {
    child->Assemble();
    if (left_looking_) {
      child->ProvideColumnUpdate(this);
    }
  }

  if (!IsRoot() && !left_looking_) {
    DoScatterSeparatorSubmatrix();
  }
}

bool T::Factor() {
  for (auto& child : children_) {
    if (!child->Factor()) {
      return false;
    }
    if (left_looking_) {
      child->ProvideColumnUpdate(this);
    }
  }
  if (supernodes_.size() > 0) {
    if (!DoEliminateSupernodeColumns()) {
      return false;
    }
  }
  // We assume that Assemble() has been called and already
  // scattered the separator sub-matrix.
  separator_schur_complement().setZero();
  DoComputeSeparatorSchurComplement();
  if (!IsRoot() && !left_looking_) {
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

std::vector<int> GetLocalEliminationPosition(
    const std::vector<int> variable_elimination_position,
    const std::vector<int> supernodes_, const std::vector<int> separators_) {
  std::vector<int> variable_to_local_elimination_position_(
      variable_elimination_position.size());
  for (size_t i = 0; i < variable_elimination_position.size(); i++) {
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
  return variable_to_local_elimination_position_;
}
void T::SetVariableOrdering(
    const std::vector<int>& shared_variable_to_elimination_position) {
  // Relabel and sort supernodes and separators.
  for (auto& s : supernodes_) {
    s = shared_variable_to_elimination_position.at(s);
  }

  for (auto& e : separators_) {
    e = shared_variable_to_elimination_position.at(e);
  }
  std::sort(supernodes_.begin(), supernodes_.end());
  std::sort(separators_.begin(), separators_.end());

  // Create map from relabelled variables to their
  // positions inside of the shared_variable vector.
  std::vector<int> variable_elimination_position = variables_;
  int v_last = -1;
  for (auto& v : variable_elimination_position) {
    v = shared_variable_to_elimination_position.at(v);
  }
  variable_set_equals_sorted_supernodes_ =
      variable_elimination_position == supernodes_;
  variable_set_equals_sorted_separators_ =
      variable_elimination_position == separators_;

  variable_to_local_elimination_position_ = GetLocalEliminationPosition(
      variable_elimination_position, supernodes_, separators_);
};

void T::DoComputeOffsets() {
  if (parent_ && separators_.size() > 0) {
    parent_->ComputeOffsets(this, 0 /*start index*/);
  }
}

void T::ReceiveColumnUpdate(const KKTSubsystemBase* source,
                            size_t start_index) {
  const auto& vars = source->separators();
  if (start_index > vars.size()) {
    return;
  }

  Update(source, this);

  size_t col_index = start_index;
  for (; col_index < vars.size(); col_index++) {
    if (supernodes_.size() == 0 || vars.at(col_index) > supernodes_.back()) {
      // The remaining columns must belong to our parent.
      break;
    }
  }

  if (col_index < vars.size()) {
    CONEX_DEMAND(parent_, "Parent pointer is null.");
    parent_->ReceiveColumnUpdate(source, col_index);
  }
}

void T::AddSparseMatrixTriplets(
    vector<Eigen::Triplet<double>>* triplets) const {
  int i = 0;
  for (auto& r : supernodes_) {
    int j = 0;
    for (auto& c : supernodes_) {
      if (supernode_submatrix()(i, j) != 0 && r >= c) {
        triplets->emplace_back(r, c, supernode_submatrix()(i, j));
      }
      j++;
    }
    i++;
  }

  i = 0;
  for (auto& r : separators_) {
    int j = 0;
    for (auto& c : supernodes_) {
      if (separator_rows()(i, j) != 0 && r >= c) {
        triplets->emplace_back(r, c, separator_rows()(i, j));
      }
      j++;
    }
    i++;
  }
}

void T::DoScatterSeparatorSubmatrix() {
  if (parent_ && separators_.size() > 0) {
    parent_->ReceiveColumnUpdate(this, 0 /*start index*/);
  }
}

void T::ComputeSeparatorOffsets() {
  for (auto& child : children_) {
    child->ComputeSeparatorOffsets();
  }

  if (!IsRoot()) {
    parent_->ComputeOffsets(this, 0);
  }
}

// Update target columns with local separator schur complement information.
// We update target column i if their is a local separator pair (j, i),
// with (j \ge i).
void T::ProvideColumnUpdate(KKTSubsystemBase* target) {
  const std::vector<int>& target_supernodes = target->supernodes();
  const std::vector<int>& target_separators = target->separators();
  if (target_supernodes.size() == 0) {
    return;
  }
  if (separators_.size() == 0 || target_supernodes.at(0) > separators_.back()) {
    return;
  }
  Update(this, target);

  for (auto& c : children_) {
    c->ProvideColumnUpdate(target);
  }
}

}  // namespace conex
