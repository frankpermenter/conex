// Alternating projections: find a point in the intersection of the
// affine constraint subspace {b - Ax : x} and the cone K.
//
// Uses only KKTSolverBase (for affine projection) and ConeOps (for
// cone projection via EJA::Variable).

#pragma once
#include "conex/common/kkt_solver_interface.h"

namespace conex {

struct AlternatingProjectionsResult {
  int iterations;
  double residual;  // ||s - project_K(s)|| at termination
};

// Alternate between projecting onto the cone and projecting onto
// the affine subspace {b - Ax}.  Finds a feasible point (if one exists).
AlternatingProjectionsResult AlternatingProjections(
    KKTSolverBase& kkt,
    RowSpace& s,
    int max_iterations = 100,
    double tolerance = 1e-8,
    bool verbose = false);

}  // namespace conex
