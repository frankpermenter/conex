#pragma once

#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/tree_rhs.h"
#include <Eigen/Dense>

namespace conex {

class KKTSolverBase {
 public:
  void Assemble();
  bool AssembleAndFactor();
  bool Factor();

  Eigen::MatrixXd Solve(Eigen::Ref<const Eigen::MatrixXd> b,
                        bool permute_to_elimination_order = true) const;

  Eigen::MatrixXd KKTMatrix(bool permute_to_elimination_order = false) const;

  // Create a BlockVariable with the same partition structure as this solver.
  BlockVariable MakeBlockVariable(int cols = 1);

  // Create a BlockVariable initialized from a dense vector or matrix.
  BlockVariable MakeBlockVariable(Eigen::Ref<const Eigen::MatrixXd> x);

  // Solve into a BlockVariable.  If the solver supports blocked solve,
  // copies blocks directly without forming a dense vector.  Otherwise
  // falls back to gather -> Solve -> scatter.
  // rhs and dest may be single or multi-column (batched RHS).
  void SolveInto(const BlockVariable& rhs, BlockVariable& dest) const;

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
  virtual SolverRHS MakeSolverRHS(int cols = 1);
  virtual SolverRHS AllocSolverRHS(Arena& arena, int cols = 1);

  virtual RowSpace MakeRowSpace(int cols = 1) = 0;

  // Row space layout: segment sizes and ops pointers, determined by
  // the clique tree. Used for arena allocation of RowSpace instances.
  struct RowSpaceInfo {
    std::vector<int> sizes;
    std::vector<const EuclideanJordanAlgebra::BarrierConeOperations*> ops;
  };
  virtual RowSpaceInfo GetRowSpaceInfo() const = 0;
  virtual void MultiplyA(const SolverRHS& x, RowSpace& out) = 0;
  virtual void AccumulateAtranspose(const RowSpace& v, SolverRHS& rhs) = 0;
  virtual void AccumulateQx(const SolverRHS& x, SolverRHS& rhs) = 0;
  virtual bool has_quadratic_cost() const = 0;
  virtual void SetWeights(const RowSpace& w) = 0;

  // Set scaling W directly: Gram = A^T W² A (nonneg) or A^T kron(W,W) A (PSD).
  // Unlike SetWeights (which takes W² and applies sqrt), this takes W.
  virtual void SetScaling(const RowSpace& w) = 0;

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

  virtual void SolveSolverRHS(SolverRHS& rhs);


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
