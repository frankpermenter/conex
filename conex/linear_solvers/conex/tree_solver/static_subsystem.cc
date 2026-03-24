#include "conex/tree_solver/static_subsystem.h"

#include "conex/common/debug_macros.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {
using T = KKTAssemblerToSubsystemAdapter;

#if CONEX_ENABLE_TIMER
static double g_write_lazy_us = 0;

void ResetUpdateDataTimers() { g_write_lazy_us = 0; }
void PrintUpdateDataTimers() {
  std::cout << "WriteLazy(us): " << static_cast<int>(g_write_lazy_us) << ", ";
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
  auto* lazy = assembler_->GetLazyEvaluator();
  CONEX_DEMAND(lazy, "Assembler must provide a lazy evaluator.");

#if CONEX_ENABLE_TIMER
  auto t0 = std::chrono::high_resolution_clock::now();
#endif
  contributor_->WriteSymmetricLazy(
      *lazy, variable_index_to_elimination_position_);
#if CONEX_ENABLE_TIMER
  auto t1 = std::chrono::high_resolution_clock::now();
  g_write_lazy_us +=
      std::chrono::duration<double, std::micro>(t1 - t0).count();
#endif
}

}  // namespace conex
