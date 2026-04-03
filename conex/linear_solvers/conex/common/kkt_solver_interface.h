#pragma once

#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/tree_rhs.h"
#include <Eigen/Dense>

namespace conex {

class KKTSolverBase {
 public:
  void Assemble() {
    DoAssemble();
    assembled_ = true;
    factored_ = false;
  }

  bool AssembleAndFactor() {
    assembled_ = false;
    factored_ = false;
    bool ok = DoAssembleAndFactor();
    if (ok) {
      assembled_ = true;
      factored_ = true;
    }
    return assembled_ && factored_;
  }

  bool Factor() {
    CONEX_DEMAND(assembled_, "System has not been assembled.");
    assembled_ = false;
    if (DoFactor()) {
      factored_ = true;
    } else {
      factored_ = false;
    }
    return factored_;
  }

  Eigen::MatrixXd Solve(Eigen::Ref<const Eigen::MatrixXd> b,
                        bool permute_to_elimination_order = true) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    Eigen::MatrixXd x = b;
    DoSolveInPlace(x, permute_to_elimination_order);
    return x;
  }

  Eigen::MatrixXd KKTMatrix(bool permute_to_elimination_order = false) const {
    CONEX_DEMAND(assembled_,
                 "System has not been assembled or is factored in place.");
    return DoKKTMatrix(permute_to_elimination_order);
  }

  // Block partition for the solution vector.  After Solve(), the
  // partition contains the solution distributed across blocks.
  virtual BlockPartition& partition() = 0;
  virtual const BlockPartition& partition() const = 0;

  // Convenience: scatter/gather through the partition.
  void ScatterToBlocks(Eigen::Ref<const Eigen::VectorXd> x) {
    auto& p = partition();
    if (p.cols() != 1) p.Resize(1);
    p.SetZero();
    p.ScatterFrom(x);
  }
  void GatherFromBlocks(Eigen::Ref<Eigen::VectorXd> x) const {
    partition().GatherInto(x);
  }

  // Create a BlockVariable with the same partition structure as this solver.
  BlockVariable MakeBlockVariable(int cols = 1) {
    return BlockVariable(MakePartition(), cols);
  }

  // Create a BlockVariable initialized from a dense vector or matrix.
  BlockVariable MakeBlockVariable(Eigen::Ref<const Eigen::MatrixXd> x) {
    auto bv = MakeBlockVariable(x.cols());
    bv.ScatterFrom(x);
    return bv;
  }

  // Solve into a BlockVariable.  If the solver supports blocked solve,
  // copies blocks directly without forming a dense vector.  Otherwise
  // falls back to gather → Solve → scatter.
  // rhs and dest may be single or multi-column (batched RHS).
  void SolveInto(const BlockVariable& rhs, BlockVariable& dest) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    if (DoSolveBlocked(rhs.partition(), dest.partition())) return;
    // Fallback: dense round-trip.
    Eigen::MatrixXd b = rhs.Gather();
    Eigen::MatrixXd x = Solve(b);
    dest.ScatterFrom(x);
  }

  virtual int number_of_variables() const = 0;

  virtual ~KKTSolverBase() = default;

  // Create a fresh partition matching this solver's block structure.
  // Subclasses override to provide the right partition type.
  virtual std::unique_ptr<BlockPartition> MakePartition() {
    return std::make_unique<DenseBlockPartition>(number_of_variables());
  }

  // Blocked solve: copy rhs blocks into the solver's internal partition,
  // solve in place, copy result into dest blocks.  Returns true if
  // the blocked path was used.  Default returns false (use dense fallback).
  virtual bool DoSolveBlocked(const BlockPartition& rhs,
                              BlockPartition& dest) const {
    (void)rhs;
    (void)dest;
    return false;
  }

  // --- Generic solver interface (no ConstraintId) ---
  // Solver loops over all constraints/costs internally.

  // Create a SolverRHS with owned partition, no separator scratch.
  // Tree solver overrides to provide separator scratch.
  virtual SolverRHS MakeSolverRHS(int cols = 1) {
    SolverRHS rhs;
    auto p = MakePartition();
    p->Resize(cols);
    p->SetZero();
    rhs.supernodes = p.get();
    owned_tree_rhs_partitions_.push_back(std::move(p));
    return rhs;
  }

  virtual RowSpace MakeRowSpace() = 0;
  virtual void MultiplyA(const SolverRHS& x, RowSpace& out) = 0;
  virtual void AccumulateAtranspose(const RowSpace& v, SolverRHS& rhs) = 0;
  virtual void AccumulateQx(const SolverRHS& x, SolverRHS& rhs) = 0;
  virtual void SetWeights(const RowSpace& w) = 0;

  // Get the affine terms (b vectors) for all linear constraints,
  // concatenated in RowSpace order.
  virtual RowSpace GetAffineTerm() = 0;

  // Dot product with lazy gather: tree solver overrides to fold
  // unscattered separator data before computing the dot product.
  // Default assumes blocks_fully_gathered is always true.
  virtual double dot(SolverRHS& a, SolverRHS& b) {
    return a.dot(b);
  }
  virtual double dot(SolverRHS& a, const BlockVariable& b) {
    return a.dot(b);
  }

  virtual void SolveSolverRHS(SolverRHS& rhs) {
    // Default: gather, solve dense, scatter back.
    CONEX_DEMAND(factored_, "System has not been factored.");
    Eigen::MatrixXd b(number_of_variables(), rhs.cols());
    rhs.supernodes->GatherInto(b);
    DoSolveInPlace(b, true);
    rhs.supernodes->ScatterFrom(b);
  }

 private:
  virtual void DoAssemble() = 0;
  virtual bool DoFactor() = 0;
  virtual bool DoAssembleAndFactor() = 0;
  virtual void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                              bool permute_to_elimination_order) const = 0;
  virtual Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order) const = 0;
  bool factored_ = false;
  bool assembled_ = false;

 protected:
  // Owned storage for MakeSolverRHS allocations.
  std::vector<std::unique_ptr<BlockPartition>> owned_tree_rhs_partitions_;
};

}  // namespace conex
