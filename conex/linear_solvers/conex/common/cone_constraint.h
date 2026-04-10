// Interface for cone constraints in the tree solver.
//
// Each cone type (nonneg, PSD, SOC) implements this interface.
// The tree solver holds vector<ConeConstraint*> and calls these
// methods without knowing the cone type.

#pragma once
#include <Eigen/Dense>

#include "conex/common/arena_allocatable.h"
#include "conex/common/block_partition.h"
#include "conex/common/cone_ops.h"
#include "conex/common/supernodal_assembler_base.h"
#include "conex/common/tree_rhs.h"

namespace conex {

class ConeConstraint : public SupernodalAssemblerBase, public ArenaAllocatable {
 public:
  virtual ~ConeConstraint() = default;

  // Block assembler for Gram matrix registration and assembly.
  virtual BlockAssembler* GetBlockAssembler() = 0;

  // A * x: constraint-space product from supernode/separator data.
  virtual Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const = 0;

  // A^T * v: accumulate into supernode/separator blocks.
  virtual void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep, int nc) const = 0;

  // Set the scaling W (direct, no sqrt).
  virtual void SetScaling(const Eigen::VectorXd& w) = 0;

  // Set weights W² (caller passes squared weight).
  virtual void SetWeights(const Eigen::VectorXd& w) = 0;

  // Affine term b in Ax + b ≥ 0 (vectorized for PSD).
  virtual Eigen::MatrixXd affine_term() const = 0;

  // Number of constraint rows (n² for PSD, m for nonneg/SOC).
  virtual int num_rows() const = 0;

  // Cone operations for EJA dispatch.
  virtual const EuclideanJordanAlgebra::ConeOps* cone_ops() const = 0;
};

}  // namespace conex
