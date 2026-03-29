#pragma once

#include "conex/common/block_partition.h"
#include "conex/common/block_variable.h"
#include "conex/common/error_checking_macros.h"
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
  BlockVariable MakeBlockVariable() {
    return BlockVariable(MakePartition());
  }

  // Create a BlockVariable initialized from a dense vector.
  BlockVariable MakeBlockVariable(const Eigen::VectorXd& x) {
    auto bv = MakeBlockVariable();
    bv.ScatterFrom(x);
    return bv;
  }

  // Solve into a BlockVariable: gathers rhs to dense, solves, scatters
  // the result into dest.
  void SolveInto(const BlockVariable& rhs, BlockVariable& dest) const {
    CONEX_DEMAND(factored_, "System has not been factored.");
    Eigen::VectorXd b = rhs.Gather();
    Eigen::MatrixXd x = Solve(b);
    dest.ScatterFrom(x.col(0));
  }

  virtual int number_of_variables() const = 0;

  virtual ~KKTSolverBase() = default;

  // Create a fresh partition matching this solver's block structure.
  // Subclasses override to provide the right partition type.
  virtual std::unique_ptr<BlockPartition> MakePartition() {
    return std::make_unique<DenseBlockPartition>(number_of_variables());
  }

 private:
  virtual void DoAssemble() = 0;
  virtual bool DoFactor() = 0;
  virtual bool DoAssembleAndFactor() {
    DoAssemble();
    return DoFactor();
  }
  virtual void DoSolveInPlace(Eigen::Ref<Eigen::MatrixXd> b,
                              bool permute_to_elimination_order) const = 0;
  virtual Eigen::MatrixXd DoKKTMatrix(
      bool permute_to_elimination_order) const = 0;
  bool factored_ = false;
  bool assembled_ = false;
};

}  // namespace conex
