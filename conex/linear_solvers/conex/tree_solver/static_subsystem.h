#pragma once
#include "conex/tree_solver/kkt_subsystem.h"
#include "conex/common/supernodal_assembler_base.h"

namespace conex {

// Forward declarations for contributor support.
enum class ContributionType;
class SubmatrixContributor;

class KKTAssemblerToSubsystemAdapter {
 public:
  KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base);
  ~KKTAssemblerToSubsystemAdapter();

  // Contributor contract: declare contribution type; the tree solver
  // auto-creates subsystems and binds a contributor after Finalize.
  void set_contribution_type(ContributionType type);
  ContributionType contribution_type() const;

  // Bind a contributor (called internally by the tree solver).
  void BindContributor(std::unique_ptr<SubmatrixContributor> contributor);

  // Original variable indices from the assembler.
  std::vector<int> variables() const { return assembler_->variables(); }

  // Elimination positions of this adapter's variables (available after
  // SetEliminationPosition has been called).
  const std::vector<int>& elimination_positions() const {
    return variable_index_to_elimination_position_;
  }

  void SetEliminationPosition(
      const std::vector<int>& shared_variable_to_elimination_position);
  void UpdateData();

 private:
  SupernodalAssemblerBase* assembler_;
  std::unique_ptr<SubmatrixContributor> contributor_;
  int contribution_type_value_ = 0;  // ContributionType::kPositiveDefinite
  std::vector<int> variable_index_to_elimination_position_;
};

void PrintUpdateDataTimers();
void ResetUpdateDataTimers();

}  // namespace conex
