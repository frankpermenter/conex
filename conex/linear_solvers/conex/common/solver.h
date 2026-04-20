#pragma once
#include <memory>
#include <vector>

#include "conex/common/conex.h"
#include "conex/common/kkt_system.h"
#include "conex/common/model.h"

namespace conex {

// Solver: preprocesses a Model and builds a KKT system.
//
// Build() removes structurally rank-deficient columns, then constructs
// the KKT system from the reduced model.  Use ExpandSolution() and
// ReduceVector() to convert between original and reduced spaces.
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
  // Returns a zero RHS if no linear cost was set.
  SolverRHS MakeCostRHS();

  Solver();
  ~Solver();
  Solver(Solver&&) noexcept;
  Solver& operator=(Solver&&) noexcept;

 private:
  KKTSystem system_;
  Expansion expansion_;
  Eigen::VectorXd reduced_linear_cost_;
};

}  // namespace conex
