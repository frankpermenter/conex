// Interface for cone constraints in the tree solver.
//
// Each cone type (nonneg, PSD, SOC) implements this interface.
// The tree solver holds vector<ConeConstraint*> and calls these
// methods without knowing the cone type.

#pragma once
#include <Eigen/Core>

#include "conex/common/arena_allocatable.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/tree_rhs.h"

namespace conex {

namespace EuclideanJordanAlgebra { class BarrierConeOperations; }

class ConeConstraint : public SupernodalAssemblerBase, public ArenaAllocatable {
 public:
  virtual ~ConeConstraint() = default;

  virtual BlockAssembler* GetBlockAssembler() = 0;

  // Compute out += A * x, where x is read from rhs and out is a raw
  // column-major buffer of size num_rows() x nc.
  virtual void MultiplyA(const SolverRHS& rhs, double* out, int nc) const = 0;

  // Accumulate A^T * V into rhs, where V is a raw column-major buffer
  // of size num_rows() x nc.
  virtual void ContributeAtranspose(
      const double* v, int v_rows, SolverRHS& rhs, int nc) const = 0;

  virtual void SetScaling(const Eigen::VectorXd& w) = 0;
  virtual void SetWeights(const Eigen::VectorXd& w) = 0;
  virtual Eigen::MatrixXd affine_term() const = 0;
  virtual int num_rows() const = 0;
  virtual const EuclideanJordanAlgebra::BarrierConeOperations* cone_ops() const = 0;

};

}  // namespace conex
