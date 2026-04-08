#include "conex/algorithms/alternating_projections.h"

#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

AlternatingProjectionsResult AlternatingProjections(
    const AffineProjection& affine,
    RowSpace& s,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace s_proj = affine.MakeVariable();

  AlternatingProjectionsResult result{};

  for (int iter = 0; iter < max_iterations; ++iter) {
    // 1. Project s onto the cone.
    project(s_proj, s);

    // 2. Residual: distance from s to its cone projection.
    RowSpace diff = addScaled(s, s_proj, 1.0, -1.0);
    double res = std::sqrt(squaredNorm(diff));

    result.iterations = iter + 1;
    result.residual = res;

    if (verbose) {
      printf("  i=%3d  residual=%.4e\n", iter, res);
    }

    if (res < tolerance) {
      s = s_proj;
      break;
    }

    // 3. Project s_proj onto the affine subspace {b - Ax}.
    s = s_proj;
    affine.Project(s);
  }

  return result;
}

}  // namespace conex
