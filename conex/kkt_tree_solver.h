#pragma once
#include "conex/kkt_solver_interface.h"
#include "conex/kkt_subsystem.h"
#include <Eigen/Dense>

namespace conex {

struct Options {
  bool validate_leaf_nodes = false;
  bool check_for_zero_pivots = false;
  int root_node = 0;
};

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:
  int number_of_variables() const;
  void AddSubsystem(KKTSubsystem* system);
  void RepairTreeInPlace(std::vector<int>* parent_ptr);

  void SetEliminationTree(const std::vector<int>& 
  subsystem_to_parent_subsystem);
  void Finalize(const Options& options = Options());
  void Finalize(const std::vector<int>& 
  subsystem_to_parent_subsystem,
                bool check_for_zero_pivots = true);

  std::vector<int> subsystem_to_parent() { return subsystem_to_parent_; }
  bool AssembleAndFactor();
 private:
  void FinalizeHelper(const std::vector<int>& subsystem_to_parent_subsystem);
  Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;
  bool DoFactor() override;
  bool CheckForZeroPivot(const std::vector<int>& parent,
                         std::vector<int>* subsystems_with_zero_piviot);

  std::vector<KKTSubsystem*> roots_;
  std::vector<KKTSubsystem*> subsystems_;
  std::vector<int> variable_to_elimination_position_;
  std::vector<int> subsystem_to_parent_;
};

}  // namespace conex
