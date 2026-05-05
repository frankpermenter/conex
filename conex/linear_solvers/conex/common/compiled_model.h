#pragma once
#include "conex/common/kkt_solver_interface.h"

namespace conex {

// CompiledModel: the interface algorithms use to interact with a compiled
// conic optimization problem.  Bundles the KKT solver (factorization +
// back-solve) with model operations (A, A^T, Q) and the cost vector.
//
// Created by Solver::Build() or directly from a KKTSolverBase + cost.
class CompiledModel {
 public:
  CompiledModel(KKTSolverBase& kkt, SolverRHS cost_rhs)
      : kkt_(kkt), cost_rhs_(std::move(cost_rhs)) {}

  // Model properties.
  bool has_quadratic_cost() const { return kkt_.has_quadratic_cost(); }
  int number_of_variables() const { return kkt_.number_of_variables(); }
  const SolverRHS& cost_rhs() const { return cost_rhs_; }

  // Allocation.
  RowSpace MakeRowSpace(int cols = 1) { return kkt_.MakeRowSpace(cols); }
  SolverRHS MakeSolverRHS(int cols = 1) { return kkt_.MakeSolverRHS(cols); }
  BlockVariable MakeBlockVariable(int cols = 1) {
    return kkt_.MakeBlockVariable(cols);
  }
  BlockVariable MakeBlockVariable(Eigen::Ref<const Eigen::MatrixXd> x) {
    return kkt_.MakeBlockVariable(x);
  }

  // Model operations.
  void MultiplyA(const SolverRHS& x, RowSpace& out) {
    kkt_.MultiplyA(x, out);
  }
  void AccumulateAtranspose(const RowSpace& v, SolverRHS& rhs) {
    kkt_.AccumulateAtranspose(v, rhs);
  }
  void AccumulateQx(const SolverRHS& x, SolverRHS& rhs) {
    kkt_.AccumulateQx(x, rhs);
  }
  RowSpace GetAffineTerm() { return kkt_.GetAffineTerm(); }

  // Inner products.
  double dot(SolverRHS& a, SolverRHS& b) { return kkt_.dot(a, b); }
  double dot(SolverRHS& a, const BlockVariable& b) {
    return kkt_.dot(a, b);
  }

  // Linear solver.
  void SetScaling(const RowSpace& w) { kkt_.SetScaling(w); }
  bool AssembleAndFactor() { return kkt_.AssembleAndFactor(); }
  void SolveSolverRHS(SolverRHS& rhs) { kkt_.SolveSolverRHS(rhs); }

  // z-space operations.
  void ComputeGradient(RowSpace& grad) { kkt_.ComputeGradient(grad); }
  void HessianProduct(const RowSpace& v, RowSpace& out) {
    kkt_.HessianProduct(v, out);
  }
  double StepSize(const RowSpace& target_k) {
    return kkt_.StepSize(target_k);
  }
  void GeodesicStep(double alpha, const RowSpace& target) {
    kkt_.GeodesicStep(alpha, target);
  }
  double LineSearch(const RowSpace& target0, const RowSpace& target1) {
    return kkt_.LineSearch(target0, target1);
  }
  double HessianNormSquared(const RowSpace& target) {
    return kkt_.HessianNormSquared(target);
  }
  double BarrierParameter() { return kkt_.BarrierParameter(); }

  // Access the underlying KKT solver (e.g. for dynamic_cast to a concrete type).
  KKTSolverBase& kkt() { return kkt_; }
  const KKTSolverBase& kkt() const { return kkt_; }

 private:
  KKTSolverBase& kkt_;
  SolverRHS cost_rhs_;
};

}  // namespace conex
