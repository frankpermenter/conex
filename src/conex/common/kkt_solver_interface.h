#pragma once

#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/error_checking_macros.h"
#include "conex/common/tree_rhs.h"

namespace conex {

class KKTSolverBase {
 public:
  void Assemble();
  bool AssembleAndFactor();
  bool Factor();

  virtual int number_of_variables() const = 0;
  virtual ~KKTSolverBase() = default;

  virtual std::unique_ptr<BlockPartition> MakePartition() {
    return std::make_unique<DenseBlockPartition>(number_of_variables());
  }

  virtual bool DoSolveBlocked(const BlockPartition& rhs,
                              BlockPartition& dest) const {
    (void)rhs; (void)dest; return false;
  }

  // --- Eigen-free solver interface ---
  virtual SolverRHS MakeSolverRHS(int cols = 1);
  virtual SolverRHS AllocSolverRHS(Arena& arena, int cols = 1);
  virtual RowSpace MakeRowSpace(int cols = 1) = 0;

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
  virtual void SetScaling(const RowSpace& w) = 0;
  virtual RowSpace GetAffineTerm() = 0;

  virtual double dot(SolverRHS& a, SolverRHS& b) { return a.dot(b); }
  virtual double dot(SolverRHS& a, const BlockVariable& b) { return a.dot(b); }

  // Gather the full solution vector from a SolverRHS into a dense vector.
  // The tree solver overrides to fold separator data into supernodes first.
  virtual void GatherInto(SolverRHS& rhs, Eigen::Ref<Eigen::MatrixXd> x) {
    rhs.supernodes->GatherInto(x);
  }

  virtual void SolveSolverRHS(SolverRHS& rhs);

  // Dense solve (raw pointer interface). Solves in-place on column-major data.
  // Subclasses override for their specific factorization.
  virtual void DenseSolveInPlace(double* data, int rows, int cols,
                                 bool permute_to_elimination_order) const = 0;
  // Dense KKT matrix (raw pointer). Writes n×n matrix into column-major data.
  virtual void DenseKKTMatrix(double* data, int n,
                              bool permute_to_elimination_order) const = 0;

 private:
  virtual void DoAssemble() = 0;
  virtual bool DoFactor() = 0;
  virtual bool DoAssembleAndFactor() = 0;
  bool factored_ = false;
  bool assembled_ = false;

 protected:
  std::vector<std::unique_ptr<BlockPartition>> owned_tree_rhs_partitions_;
};

}  // namespace conex
