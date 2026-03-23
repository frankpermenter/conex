#include "conex/static_subsystem.h"

#include "conex/debug_macros.h"
#include "conex/kkt_tree_solver.h"

namespace conex {
using T = KKTAssemblerToSubsystemAdapter;

#if CONEX_ENABLE_TIMER
static double g_dense_data_us = 0;
static double g_write_lazy_us = 0;

void ResetUpdateDataTimers() {
  g_dense_data_us = 0;
  g_write_lazy_us = 0;
}
void PrintUpdateDataTimers() {
  std::cout << "SetDenseData(us): " << static_cast<int>(g_dense_data_us)
            << ", WriteLazy(us): " << static_cast<int>(g_write_lazy_us)
            << ", ";
  ResetUpdateDataTimers();
}
#else
void ResetUpdateDataTimers() {}
void PrintUpdateDataTimers() {}
#endif
T::KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base)
    : assembler_(base) {}

T::~KKTAssemblerToSubsystemAdapter() = default;

void T::set_contribution_type(ContributionType type) {
  contribution_type_value_ = static_cast<int>(type);
}

ContributionType T::contribution_type() const {
  return static_cast<ContributionType>(contribution_type_value_);
}

void T::BindContributor(std::unique_ptr<SubmatrixContributor> contributor) {
  contributor_ = std::move(contributor);
}

void T::SetEliminationPosition(
    const std::vector<int>& shared_variable_to_elimination_position) {
  variable_index_to_elimination_position_ = assembler_->variables();
  for (auto& v : variable_index_to_elimination_position_) {
    v = shared_variable_to_elimination_position.at(v);
  }
}

void T::UpdateData() {
  CONEX_DEMAND(contributor_, "Contributor not bound.");

  // Always call SetDenseData to populate auxiliary fields (AW, AQc, etc.)
  // that the cone program reads from submatrix_data().
#if CONEX_ENABLE_TIMER
  auto t0 = std::chrono::high_resolution_clock::now();
#endif
  assembler_->SetDenseData();
#if CONEX_ENABLE_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
#endif

  // Use the lazy evaluator if the assembler provides one, writing blocks
  // of the Gram matrix directly into subsystem storage without copying G.
  if (auto* lazy = assembler_->GetLazyEvaluator()) {
    contributor_->WriteSymmetricLazy(
        *lazy, variable_index_to_elimination_position_);
#if CONEX_ENABLE_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    g_dense_data_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
    g_write_lazy_us += std::chrono::duration<double, std::micro>(t2 - t1).count();
#endif
    return;
  }

  // Fallback: copy the materialized G into subsystem storage.
  const auto& G = assembler_->submatrix_data()->G;
  const int n = G.rows();
  Eigen::MatrixXd Q(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) Q(i, j) = G(i, j);
  contributor_->WriteSymmetric(Q, variable_index_to_elimination_position_);
}

}  // namespace conex
