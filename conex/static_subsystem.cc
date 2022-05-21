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

  if (base->is_positive_definite()) {
    kkt_subsystem_ =
        std::make_unique<SystemTypePositiveDefinite>(assembler_->variables());
  } else {
    kkt_subsystem_ =
        std::make_unique<SystemTypeIndefinite>(assembler_->variables());
  }
  kkt_subsystem_->SetFactorizationMode(true /*left looking*/);
}

void T::UpdateData() {
  int n1 = kkt_subsystem_->supernodes().size();
  int n2 = kkt_subsystem_->separators().size();
  auto& source_submatrix = assembler_->submatrix_data()->G;

  if (kkt_subsystem_->variable_set_equals_sorted_supernodes()) {
    new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
        kkt_subsystem_->supernode_submatrix().data(), n1, n1);
    assembler_->SetDenseData();
    return;
  }

  if (kkt_subsystem_->variable_set_equals_sorted_separators()) {
    new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
        kkt_subsystem_->separator_schur_complement().data(), n2, n2);
    assembler_->SetDenseData();
    return;
  }
  Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
  assembler_->SetDenseData();
  AssignSubmatrix(source_submatrix, Q_in_elimination_order_,
                  kkt_subsystem_->variable_to_local_elimination_rank());

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
