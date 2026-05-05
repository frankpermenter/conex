// Interface for cone constraints in the tree solver.
//
// Each cone type (nonneg, PSD, SOC) implements this interface.
// The tree solver holds vector<ConeConstraint*> and calls these
// methods without knowing the cone type.

#pragma once
#include <Eigen/Dense>

#include "conex/common/arena_allocatable.h"
#include "conex/common/supernodal_assembler_base.h"

namespace conex {

class BlockPartition;
struct SeparatorScratch;
namespace EuclideanJordanAlgebra { class SymmetricConeOperations; }

class ConeConstraint : public SupernodalAssemblerBase, public ArenaAllocatable {
 public:
  virtual ~ConeConstraint() = default;

  virtual BlockAssembler* GetBlockAssembler() = 0;

  virtual Eigen::MatrixXd MultiplyA(
      const BlockPartition& supernodes, const SeparatorScratch& sep,
      int nc) const = 0;

  virtual void ContributeAtranspose(
      const Eigen::Ref<const Eigen::MatrixXd>& V,
      BlockPartition& supernodes, SeparatorScratch& sep, int nc) const = 0;

  virtual void SetScaling(const Eigen::VectorXd& w) = 0;
  virtual void SetWeights(const Eigen::VectorXd& w) = 0;
  virtual Eigen::MatrixXd affine_term() const = 0;
  virtual int num_rows() const = 0;
  virtual const EuclideanJordanAlgebra::SymmetricConeOperations* cone_ops() const = 0;
};

}  // namespace conex
