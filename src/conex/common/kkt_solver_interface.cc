#include "conex/common/kkt_solver_interface.h"

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

Eigen::MatrixXd KKTSolverBase::Solve(
    Eigen::Ref<const Eigen::MatrixXd> b,
    bool permute_to_elimination_order) const {
  CONEX_DEMAND(factored_, "System has not been factored.");
  Eigen::MatrixXd x = b;
  DoSolveInPlace(x, permute_to_elimination_order);
  return x;
}

Eigen::MatrixXd KKTSolverBase::KKTMatrix(
    bool permute_to_elimination_order) const {
  CONEX_DEMAND(assembled_,
               "System has not been assembled or is factored in place.");
  return DoKKTMatrix(permute_to_elimination_order);
}

BlockVariable KKTSolverBase::MakeBlockVariable(int cols) {
  return BlockVariable(MakePartition(), cols);
}

BlockVariable KKTSolverBase::MakeBlockVariable(
    Eigen::Ref<const Eigen::MatrixXd> x) {
  auto bv = MakeBlockVariable(x.cols());
  bv.ScatterFrom(x);
  return bv;
}

void KKTSolverBase::SolveInto(const BlockVariable& rhs,
                               BlockVariable& dest) const {
  CONEX_DEMAND(factored_, "System has not been factored.");
  if (DoSolveBlocked(rhs.partition(), dest.partition())) return;
  // Fallback: dense round-trip.
  Eigen::MatrixXd b = rhs.Gather();
  Eigen::MatrixXd x = Solve(b);
  dest.ScatterFrom(x);
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

void KKTSolverBase::SolveSolverRHS(SolverRHS& rhs) {
  // Default: gather, solve dense, scatter back.
  CONEX_DEMAND(factored_, "System has not been factored.");
  Eigen::MatrixXd b(number_of_variables(), rhs.cols());
  rhs.supernodes->GatherInto(b);
  DoSolveInPlace(b, true);
  rhs.supernodes->ScatterFrom(b);
}

}  // namespace conex
