#include "conex/kkt_subsystem.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace conex {

using T = KKTSubsystem;
//  L
//  SR^{-1}  D
void T::ApplyInverseOfLeftFactor(Eigen::Ref<Eigen::MatrixXd> x) {
  for (auto child : children_) {
    child->ApplyInverseOfLeftFactor(x);
  }

  Eigen::Ref<Eigen::MatrixXd> ref = x.middleRows(
      supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);
  DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(ref);

  if (separators_.size() > 0) {
    // Compute S R^{-1} ref
    Eigen::MatrixXd temp = ref;
    DoApplyInverseOfRightFactorOfSupernodeSubmatrix(temp);
    Eigen::MatrixXd residual = separator_rows_ * temp;
    for (int i = 0; i < residual.rows(); i++) {
      x.row(separators_[i]) -= residual.row(i);
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

void T::ApplyInverseOfRightFactor(Eigen::Ref<Eigen::MatrixXd> x) {
  Eigen::Ref<Eigen::MatrixXd> ref = x.middleRows(
      supernodes_.at(0), supernodes_.back() - supernodes_.at(0) + 1);
  if (!IsRoot()) {
    // Subtract L^{-1} S^T
    Eigen::MatrixXd temp = separator_rows_.transpose() * SeparatorRows(x);
    DoApplyInverseOfLeftFactorOfSupernodeSubmatrix(temp);
    ref.noalias() -= temp;
  }

  DoApplyInverseOfRightFactorOfSupernodeSubmatrix(ref);
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

}  // namespace conex
