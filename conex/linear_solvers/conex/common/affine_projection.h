// Projection onto the affine subspace {b - Ax : x ∈ R^n}.
// Constructed from a Model; hides the KKT solver.

#pragma once
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/common/tree_rhs.h"

namespace conex {

class AffineProjection {
 public:
  static AffineProjection Build(const Model& problem);

  // Project s onto {b - Ax}.
  void Project(RowSpace& s) const;

  // Create a Variable with the right segment layout.
  RowSpace MakeVariable() const;

  // Access the affine term b.
  RowSpace GetAffineTerm() const;

  // Access the underlying solver (for recovering x, etc.).
  Solver* solver() const { return &solver_; }

 private:
  AffineProjection() = default;
  mutable Solver solver_;
};

}  // namespace conex
