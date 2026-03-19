#include "conex/conex.h"
#include "conex/constraint_manager.h"
#include "conex/kkt_solver_interface.h"
#include "conex/kkt_tree_solver.h"

namespace conex {
class KKTSolverFactory {
 public:
  static std::unique_ptr<KKTSolverBase> create_unique(
      ConstraintManager* kkt, const SolverConfiguration& config);
};

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config);

}  // namespace conex
