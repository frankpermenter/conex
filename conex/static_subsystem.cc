#include "conex/static_subsystem.h"

#include "conex/RLDLT.h"
#include "conex/cholesky_solvers.h"
#include "conex/kkt_tree_solver.h"

namespace conex {
using T = KKTAssemblerToSubsystemAdapter;

namespace {}  // namespace
T::KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base)
    : assembler_(base) {}

T::~KKTAssemblerToSubsystemAdapter() = default;

KKTSubsystemBase* T::KKTAssemblerToSubsystemAdapter::create_subsystem(
    const SubsystemType& type) {
  using SystemTypePositiveDefinite = KKTCholeskySystem<
      CholeskySolver<Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>>, false>>;
  using SystemTypeIndefinite = KKTCholeskySystem<
      CholeskySolver<Eigen::RLDLT<Eigen::Ref<Eigen::MatrixXd>>, true>>;

  switch (type) {
    case SubsystemType::kPositiveDefinite:
      kkt_subsystem_ = std::make_unique<SystemTypePositiveDefinite>();
      break;
    case SubsystemType::kNegativeDefinite:
    case SubsystemType::kQuasiDefinite:
      kkt_subsystem_ = std::make_unique<SystemTypeIndefinite>();
  }
  kkt_subsystem_->SetFactorizationMode(true /*left looking*/);
  return kkt_subsystem_.get();
}

void T::set_contribution_type(ContributionType type) {
  contribution_type_value_ = static_cast<int>(type);
}

ContributionType T::contribution_type() const {
  return static_cast<ContributionType>(contribution_type_value_);
}

void T::BindContributor(std::unique_ptr<SubmatrixContributor> contributor) {
  contributor_ = std::move(contributor);
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
  supernode_map_bound_ = false;
  separator_map_bound_ = false;
  variable_index_to_elimination_position_ = assembler_->variables();
  for (auto& v : variable_index_to_elimination_position_) {
    v = shared_variable_to_elimination_position.at(v);
  }

  // The contributor path (or any path without a manually-created subsystem)
  // does not need the remaining setup — it will be done after binding.
  if (!kkt_subsystem_) return;

  variable_set_equals_sorted_supernodes_ =
      variable_index_to_elimination_position_ == kkt_subsystem_->supernodes() &&
      kkt_subsystem_->separators().size() == 0;

  variable_set_equals_sorted_separators_ =
      variable_index_to_elimination_position_ == kkt_subsystem_->separators() &&
      kkt_subsystem_->supernodes().size() == 0;

  variable_to_local_elimination_position_ = GetLocalEliminationPosition(
      variable_index_to_elimination_position_, kkt_subsystem_->supernodes(),
      kkt_subsystem_->separators());

  remap_lower_entries_.clear();
  remap_lower_entries_.reserve(
      variable_to_local_elimination_position_.size() *
      (variable_to_local_elimination_position_.size() + 1) / 2);
  for (size_t j = 0; j < variable_to_local_elimination_position_.size(); ++j) {
    for (size_t i = j; i < variable_to_local_elimination_position_.size();
         ++i) {
      int row = variable_to_local_elimination_position_.at(i);
      int col = variable_to_local_elimination_position_.at(j);
      if (col > row) {
        std::swap(row, col);
      }
      remap_lower_entries_.push_back(
          {static_cast<int>(i), static_cast<int>(j), row, col});
    }
  }
}

void T::UpdateData() {
  if (contributor_) {
    // Contributor path: populate assembler data, then write via contributor.
    // Storage is zeroed centrally by UpdateAssemblerData before any adapter
    // writes, so multiple contributors can accumulate into the same subsystem.
    assembler_->SetDenseData();
    const auto& G = assembler_->submatrix_data()->G;
    const int n = G.rows();
    Eigen::MatrixXd Q(n, n);
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) Q(i, j) = G(i, j);
    contributor_->WriteSymmetric(Q, variable_index_to_elimination_position_);
    return;
  }

  // Legacy path: direct storage access.
  int n1 = kkt_subsystem_->supernodes().size();
  int n2 = kkt_subsystem_->separators().size();
  auto& source_submatrix = assembler_->submatrix_data()->G;

  if (variable_set_equals_sorted_supernodes_) {
    if (!supernode_map_bound_) {
      new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
          kkt_subsystem_->supernode_submatrix().data(), n1, n1);
      supernode_map_bound_ = true;
    }
    assembler_->SetDenseData();
    return;
  }

  if (variable_set_equals_sorted_separators_) {
    if (!separator_map_bound_) {
      new (&source_submatrix) Eigen::Map<Eigen::MatrixXd, Eigen::Aligned>(
          kkt_subsystem_->separator_schur_complement().data(), n2, n2);
      separator_map_bound_ = true;
    }
    assembler_->SetDenseData();
    return;
  }
  Q_in_elimination_order_.resize(n1 + n2, n1 + n2);
  assembler_->SetDenseData();
  Q_in_elimination_order_.setZero();
  for (const auto& entry : remap_lower_entries_) {
    Q_in_elimination_order_(entry.dst_row, entry.dst_col) =
        source_submatrix(entry.src_row, entry.src_col);
  }
  CONEX_DEMAND(n1 >= 0 && n2 >= 0, "Invalid block sizes.");
  CONEX_DEMAND(n1 <= Q_in_elimination_order_.rows() &&
                   n1 <= Q_in_elimination_order_.cols(),
               "Invalid top-left block bounds.");
  CONEX_DEMAND(n2 <= Q_in_elimination_order_.rows() &&
                   n1 <= Q_in_elimination_order_.cols(),
               "Invalid bottom-left block bounds.");
  CONEX_DEMAND(n2 <= Q_in_elimination_order_.rows() &&
                   n2 <= Q_in_elimination_order_.cols(),
               "Invalid bottom-right block bounds.");

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
