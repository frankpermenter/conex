#include "conex/common/kkt_solver_interface.h"
#include "conex/common/arena.h"
#include <Eigen/Core>

namespace conex {

void KKTSolverBase::Assemble() {
  DoAssemble();
  assembled_ = true;
  factored_ = false;
}

bool KKTSolverBase::AssembleAndFactor() {
  assembled_ = false;
  factored_ = false;
  bool ok = DoAssembleAndFactor();
  if (ok) {
    assembled_ = true;
    factored_ = true;
  }
  return assembled_ && factored_;
}

bool KKTSolverBase::Factor() {
  CONEX_DEMAND(assembled_, "System has not been assembled.");
  assembled_ = false;
  if (DoFactor()) {
    factored_ = true;
  } else {
    factored_ = false;
  }
  return factored_;
}

SolverRHS KKTSolverBase::MakeSolverRHS(int cols) {
  SolverRHS rhs;
  auto p = MakePartition();
  p->Resize(cols);
  p->SetZero();
  rhs.supernodes = p.get();
  owned_tree_rhs_partitions_.push_back(std::move(p));
  return rhs;
}

SolverRHS KKTSolverBase::AllocSolverRHS(Arena& arena, int cols) {
  SolverRHS rhs;
  auto p = MakePartition();
  // Bind data to arena instead of heap-allocating MatrixXd.
  auto* sbp = dynamic_cast<StandaloneBlockPartition*>(p.get());
  if (sbp) {
    sbp->BindArena(arena, cols);
  } else {
    p->Resize(cols);
    p->SetZero();
  }
  rhs.supernodes = p.get();
  owned_tree_rhs_partitions_.push_back(std::move(p));
  return rhs;
}

void KKTSolverBase::SolveSolverRHS(SolverRHS& rhs) {
  // Default: gather, solve dense, scatter back.
  CONEX_DEMAND(factored_, "System has not been factored.");
  Eigen::MatrixXd b(number_of_variables(), rhs.cols());
  rhs.supernodes->GatherInto(b);
  DenseSolveInPlace(b.data(), b.rows(), b.cols(), true);
  rhs.supernodes->ScatterFrom(b);
}

}  // namespace conex
