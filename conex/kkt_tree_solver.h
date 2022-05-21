#pragma once
#include "conex/kkt_solver_interface.h"
#include "conex/kkt_subsystem.h"
#include "conex/static_subsystem.h"
#include "conex/tree_utils.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {

struct Options {
  bool validate_leaf_nodes = false;
  bool check_for_zero_pivots = false;
  int root_node = 0;
};

class SymmetricLinearSystemTreeSolver : public KKTSolverBase {
 public:
  int number_of_variables() const;
  void AddSubsystem(KKTSubsystemBase* system);
  void RepairTreeInPlace(std::vector<int>* parent_ptr);

  void Finalize(const CliqueTree& clique_tree);
  void Finalize(const Options& options = Options());
  void Finalize(const std::vector<int>& subsystem_to_parent_subsystem,
                bool check_for_zero_pivots = true);

  void SetFactorizationMode(bool left_looking);
  void UpdateAssemblerData();


  std::vector<int> subsystem_to_parent() { return subsystem_to_parent_; }

  Eigen::SparseMatrix<double> MakeSparseKKTMatrix(
      bool permute_to_elimination_order = true) const;

  void ComputeSeparatorOffsets();
  std::vector<int> ComputePostOrdering() const;
  void push_back(std::unique_ptr<KKTAssemblerToSubsystemAdapter>&& system);

 private:
  void SetEliminationTree(
      const std::vector<int>& variable_to_elimination_position);
  void SetEliminationOrder(
      const std::vector<int>& variable_to_elimination_position);
  void FinalizeHelper(const std::vector<int>& subsystem_to_parent_subsystem);
  Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order = true) const override;
  void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                      bool in_original_order) const;

  void DoAssemble() override;
  bool DoAssembleAndFactor() override;
  bool DoFactor() override;
  bool CheckForZeroPivot(const std::vector<int>& parent,
                         std::vector<int>* subsystems_with_zero_piviot);

  std::vector<KKTSubsystemBase*> roots_;
  std::vector<KKTSubsystemBase*> subsystems_;
  std::vector<std::unique_ptr<KKTAssemblerToSubsystemAdapter>>
      assembler_to_subsystem_adapter_;
  std::vector<int> variable_to_elimination_position_;
  std::vector<int> subsystem_to_parent_;
};

}  // namespace conex
