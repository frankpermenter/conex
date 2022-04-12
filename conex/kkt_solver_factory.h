#include "conex/conex.h"
#include "conex/constraint_manager.h"
#include "conex/kkt_solver_interface.h"

namespace conex {
class KKTSolverFactory {
 public:
  static std::unique_ptr<KKTSolverBase> create_unique(
      ConstraintManager* kkt, const SolverConfiguration& config);
};
}  // namespace conex
