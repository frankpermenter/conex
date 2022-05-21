#pragma once
#include "conex/kkt_subsystem.h"
#include "conex/supernodal_assembler_base.h"

namespace conex {
class KKTAssemblerToSubsystemAdapter {
 public:
  KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base);
  KKTSubsystemBase* kkt_subsystem() { return kkt_subsystem_.get(); }
  void SetEliminationPosition(
      const std::vector<int>& shared_variable_to_elimination_position);
  void UpdateData();

 private:
  SupernodalAssemblerBase* assembler_;
  std::unique_ptr<KKTSubsystemBase> kkt_subsystem_;
  Eigen::MatrixXd Q_in_elimination_order_;
  std::vector<int> variable_to_local_elimination_position_;
  bool variable_set_equals_sorted_supernodes_ = false;
  bool variable_set_equals_sorted_separators_ = false;
};

}  // namespace conex
