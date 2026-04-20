#pragma once
#include <memory>
#include <vector>

#include "conex/common/conex.h"
#include "conex/common/kkt_system.h"
#include "conex/common/model.h"
#include "conex/common/solve_result.h"

namespace conex {

// Solver: preprocesses a Model, builds a KKT system, and dispatches
// algorithms via the Solve() template.
//
// Solve() accepts any algorithm type that implements:
//   GeodesicResult Run(KKTSolverBase& kkt, const SolverRHS& cost_rhs);
//
// The algorithm operates in reduced space.  Solver handles preprocessing,
// coordinate conversion, and objective computation.
//
// For direct KKT access without preprocessing (e.g., unit tests),
// use KKTSystem::Build() instead.
class Solver {
 public:
  static Solver Build(const Model& model,
                      const SolverConfiguration& config = {});

  static Solver Build(const Model& model,
                      const TreeSpec& tree,
                      const SolverConfiguration& config = {});

  static Solver BuildDense(const Model& model);

  // Run an algorithm and return the result in Model (original) space.
  // The Algorithm type must implement:
  //   GeodesicResult Run(KKTSolverBase& kkt, const SolverRHS& cost_rhs);
  template <typename Algorithm>
  SolveResult Solve(const Algorithm& algo);

  // Access the underlying KKT system.
  KKTSystem& system() { return system_; }
  const KKTSystem& system() const { return system_; }

  // Convenience: forward to system.
  KKTSolverBase* kkt() { return system_.kkt(); }
  const KKTSolverBase* kkt() const { return system_.kkt(); }
  SymmetricLinearSystemTreeSolver* tree_solver() {
    return system_.tree_solver();
  }
  const SymmetricLinearSystemTreeSolver* tree_solver() const {
    return system_.tree_solver();
  }
  const std::vector<int>& dual_variables(ConstraintId id) const {
    return system_.dual_variables(id);
  }

  // Preprocessing: map between original and reduced variable spaces.
  bool was_reduced() const { return expansion_.was_reduced(); }
  Eigen::VectorXd ExpandSolution(const Eigen::VectorXd& x_reduced) const {
    return expansion_.Expand(x_reduced);
  }
  Eigen::VectorXd ReduceVector(const Eigen::VectorXd& v) const {
    return expansion_.Reduce(v);
  }

  // Build the cost RHS in solver format (reduced space).
  SolverRHS MakeCostRHS();

  Solver();
  ~Solver();
  Solver(Solver&&) noexcept;
  Solver& operator=(Solver&&) noexcept;

 private:
  // Compute c'x + (1/2)x'Qx in reduced space using KKT operations.
  double ComputeObjective(const SolverRHS& cost_rhs,
                          const Eigen::VectorXd& x_reduced);

  // Extract per-constraint dual variables from RowSpace lambda.
  ConstraintDuals ExtractDuals(const Eigen::VectorXd& x_reduced,
                               const RowSpace& lambda);

  KKTSystem system_;
  Model reduced_model_;
  Expansion expansion_;
  Eigen::VectorXd reduced_linear_cost_;
};

// =====================================================================
// Solve() implementation (template — must be in header)
// =====================================================================

template <typename Algorithm>
SolveResult Solver::Solve(const Algorithm& algo) {
  auto cost_rhs = MakeCostRHS();
  auto raw = algo.Run(*kkt(), cost_rhs);

  SolveResult result;
  result.x = ExpandSolution(raw.x);
  result.objective = ComputeObjective(cost_rhs, raw.x);
  result.mu = raw.mu;
  result.dual_residual = raw.optimality.dual_residual;
  result.complementarity = raw.optimality.complementarity;
  result.iterations = raw.iterations;
  result.factorizations = raw.total_factorizations;
  result.converged = raw.mu < 1e-6;
  if (raw.lambda.total_rows() > 0) {
    result.duals = ExtractDuals(raw.x, raw.lambda);
  }
  return result;
}

}  // namespace conex
