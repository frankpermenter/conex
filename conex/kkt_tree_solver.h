#include <Eigen/Dense>

#include "conex/kkt_solver_interface.h"
#include "conex/kkt_subsystem.h"

namespace conex {

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:

  void MakeTree(std::vector<int> subsystem_to_parent_subsystem);
  int number_of_variables() const;
  void AddSubsystem(KKTSubsystem* system);

 private:
  Eigen::MatrixXd DoKKTMatrix(bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;

  bool DoFactor() override;
  std::vector<KKTSubsystem*> roots_;
  std::vector<KKTSubsystem*> subsystems_;
  std::vector<KKTSubsystem*> indefinite_subsystems_;
  std::vector<int> variable_to_elimination_position_;
};

} // namespace conex
