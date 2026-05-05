// Interface for cone constraints in the tree solver.
//
// Each cone type (nonneg, PSD, SOC) implements this interface.
// The tree solver holds vector<ConeConstraint*> and calls these
// methods without knowing the cone type.

#pragma once
#include <cstdlib>
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

  // --- z-space operations for geodesic IPM on general cones ---
  //
  // These methods operate on the internal state set by SetScaling(z).
  // For symmetric cones, z is stored internally as W = -∇F(z);
  // implementations forward to existing W-space operations.
  // For general cones with BarrierOps, z is stored directly.

  // Gradient of the barrier at the current scaling point.
  // Symmetric cone: grad = -W (since ∇F(z) = -W).
  virtual void ComputeGradient(Eigen::VectorXd& grad) const {
    (void)grad; std::abort();
  }

  // Hessian-vector product: out = ∇²F(z) · v.
  // Symmetric cone: out = P(W) v (= diag(W²)v for nonneg).
  virtual void HessianProduct(const Eigen::VectorXd& v,
                              Eigen::VectorXd& out) const {
    (void)v; (void)out; std::abort();
  }

  // Step size from the z-space tangent (target - z).
  // Symmetric cone: converts to d-space, returns min(1, 2/||d||²_∞).
  // General cone: returns 1/(1 + ||target - z||_H).
  virtual double StepSize(const Eigen::VectorXd& target_k) const {
    (void)target_k; std::abort(); return 0;
  }

  // Geodesic step: update internal state via Exp_z(α(target - z)).
  // Symmetric cone: converts to W-space direction, does W·exp(αd).
  // General cone: calls BarrierOps::ExponentialMap.
  virtual void GeodesicStep(double alpha, const Eigen::VectorXd& target) {
    (void)alpha; (void)target; std::abort();
  }

  // Line search: largest k with z + ż(k) feasible, where
  //   ż(k) = target0 + k·target1 - z.
  // Symmetric cone: converts to d0, d1 in eigenvalue space, calls lineSearchK.
  // General cone: binary search with isInterior.
  virtual double LineSearch(const Eigen::VectorXd& target0,
                            const Eigen::VectorXd& target1) const {
    (void)target0; (void)target1; std::abort(); return 0;
  }

  // Squared Hessian norm: ||target - z||²_{H(z)}.
  virtual double HessianNormSquared(const Eigen::VectorXd& target) const {
    (void)target; std::abort(); return 0;
  }

  // Barrier parameter ν for this cone.
  virtual double BarrierParameter() const { std::abort(); return 0; }
};

}  // namespace conex
