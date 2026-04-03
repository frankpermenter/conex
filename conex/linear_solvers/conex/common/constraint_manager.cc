#include "conex/common/constraint_manager.h"
namespace conex {
using T = ConstraintManager;

std::vector<CliqueProvider*> T::clique_assemblers() {
  return custom_assemblers_;
}

}  // namespace conex
