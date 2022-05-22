#include "conex/static_subsystem.h"

#include "conex/RLDLT.h"
#include "conex/cholesky_solvers.h"

namespace conex {
using T = KKTAssemblerToSubsystemAdapter;

namespace {
void AssignSubmatrix(const Eigen::MatrixXd& source,
                     Eigen::Ref<Eigen::MatrixXd> destination,
                     const std::vector<int>& source_to_dest_index) {
  destination.setZero();
  for (int j = 0; j < source.cols(); j++) {
    for (int i = j; i < source.rows(); i++) {
      int row = source_to_dest_index.at(i);
      int col = source_to_dest_index.at(j);
      if (col > row) {
        std::swap(row, col);
      }
      destination(row, col) = source(i, j);
    }
  }
}
}  // namespace
T::KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base)
    : assembler_(base) {
  using SystemTypePositiveDefinite = KKTCholeskySystem<
      CholeskySolver<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>, false>>;
  using SystemTypeIndefinite = KKTCholeskySystem<
      CholeskySolver<Eigen::RLDLT<Eigen::Ref<Eigen::MatrixXd>>, true>>;

  if (0) {  // base->is_positive_definite()) {
    kkt_subsystem_ = std::make_unique<SystemTypePositiveDefinite>();
  } else {
    kkt_subsystem_ = std::make_unique<SystemTypeIndefinite>();
  }
  kkt_subsystem_->SetFactorizationMode(true /*left looking*/);
}

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

void T::SetEliminationPosition(
    const std::vector<int>& shared_variable_to_elimination_position) {
  variable_index_to_elimination_position_ = assembler_->variables();
  for (auto& v : variable_index_to_elimination_position_) {
    v = shared_variable_to_elimination_position.at(v);
  }
  variable_set_equals_sorted_supernodes_ =
      variable_index_to_elimination_position_ == kkt_subsystem_->supernodes() &&
      kkt_subsystem_->separators().size() == 0;

  variable_set_equals_sorted_separators_ =
      variable_index_to_elimination_position_ == kkt_subsystem_->separators() &&
      kkt_subsystem_->supernodes().size() == 0;

  variable_to_local_elimination_position_ = GetLocalEliminationPosition(
      variable_index_to_elimination_position_, kkt_subsystem_->supernodes(),
      kkt_subsystem_->separators());
}

void T::UpdateData() {
  int n1 = kkt_subsystem_->supernodes().size();
  int n2 = kkt_subsystem_->separators().size();
  auto& source_submatrix = assembler_->submatrix_data()->G;

  if (variable_set_equals_sorted_supernodes_) {
    new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
        kkt_subsystem_->supernode_submatrix().data(), n1, n1);
    assembler_->SetDenseData();
    return;
  }

  if (variable_set_equals_sorted_separators_) {
    new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
        kkt_subsystem_->separator_schur_complement().data(), n2, n2);
    assembler_->SetDenseData();
    return;
  }
  Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
  assembler_->SetDenseData();
  AssignSubmatrix(source_submatrix, Q_in_elimination_order_,
                  variable_to_local_elimination_position_);

  kkt_subsystem_->supernode_submatrix().triangularView<Eigen::Lower>() =
      Q_in_elimination_order_.topLeftCorner(n1, n1)
          .triangularView<Eigen::Lower>();
  kkt_subsystem_->separator_rows() =
      Q_in_elimination_order_.bottomLeftCorner(n2, n1);
  kkt_subsystem_->separator_schur_complement().triangularView<Eigen::Lower>() =
      Q_in_elimination_order_.bottomRightCorner(n2, n2)
          .triangularView<Eigen::Lower>();
}

}  // namespace conex
