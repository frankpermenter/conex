#pragma once
#include "conex/kkt_subsystem.h"
#include "conex/supernodal_assembler_base.h"

namespace conex {

enum class SubsystemType {
  kPositiveDefinite,
  kNegativeDefinite,
  kQuasiDefinite,
};

// Forward declarations for contributor support.
enum class ContributionType;
class SubmatrixContributor;

class KKTAssemblerToSubsystemAdapter {
 public:
  KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base);
  ~KKTAssemblerToSubsystemAdapter();

  KKTSubsystemBase* kkt_subsystem() { return kkt_subsystem_.get(); }
  KKTSubsystemBase* create_subsystem(const SubsystemType& type);

  // Contributor contract: declare contribution type; the tree solver
  // auto-creates subsystems and binds a contributor after Finalize.
  void set_contribution_type(ContributionType type);
  ContributionType contribution_type() const;

  // Bind a contributor (called internally by the tree solver).
  void BindContributor(std::unique_ptr<SubmatrixContributor> contributor);

  // Elimination positions of this adapter's variables (available after
  // SetEliminationPosition has been called).
  const std::vector<int>& elimination_positions() const {
    return variable_index_to_elimination_position_;
  }

  void SetEliminationPosition(
      const std::vector<int>& shared_variable_to_elimination_position);
  void UpdateData();

 private:
  struct RemapEntry {
    int src_row;
    int src_col;
    int dst_row;
    int dst_col;
  };
  SupernodalAssemblerBase* assembler_;
  std::unique_ptr<KKTSubsystemBase> kkt_subsystem_;
  std::unique_ptr<SubmatrixContributor> contributor_;
  int contribution_type_value_ = 0;  // ContributionType::kPositiveDefinite
  Eigen::MatrixXd Q_in_elimination_order_;
  std::vector<int> variable_to_local_elimination_position_;
  std::vector<RemapEntry> remap_lower_entries_;
  bool variable_set_equals_sorted_supernodes_ = false;
  bool variable_set_equals_sorted_separators_ = false;
  bool supernode_map_bound_ = false;
  bool separator_map_bound_ = false;
  std::vector<int> variable_index_to_elimination_position_;
};

}  // namespace conex
