#pragma once
#include "conex/kkt_subsystem.h"
#include "conex/supernodal_assembler_base.h"

namespace conex {
class KKTAssemblerToSubsystemAdapter  {
 public:
  KKTAssemblerToSubsystemAdapter(SupernodalAssemblerBase* base);
  KKTSubsystemBase* kkt_subsystem() { return kkt_subsystem_.get();}
  void UpdateData();
 private:
  SupernodalAssemblerBase* assembler_;
  std::unique_ptr<KKTSubsystemBase> kkt_subsystem_;
};

} // namespace
