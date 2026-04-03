#include "conex/common/constraint_manager.h"
namespace conex {
using T = ConstraintManager;

int T::SizeOfKKTSystem() const { return new_dual_variable_start_; };

std::vector<CliqueProvider*> T::clique_assemblers() {
  return custom_assemblers_;
}

std::vector<const CliqueProvider*> T::clique_assemblers() const {
  std::vector<const CliqueProvider*> result(custom_assemblers_.begin(),
                                             custom_assemblers_.end());
  return result;
}

}  // namespace conex
