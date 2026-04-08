#include "conex/algorithms/alternating_projections.h"

#include <cstdio>

#include "conex/common/eja_ops.h"

namespace conex {

AlternatingProjectionsResult AlternatingProjections(
    KKTSolverBase& kkt,
    RowSpace& s,
    int max_iterations,
    double tolerance,
    bool verbose) {
  RowSpace b = kkt.GetAffineTerm();

  // Set identity weights so the Gram is A^T A.
  RowSpace weights = kkt.MakeRowSpace();
  setOnes(weights);
  kkt.SetWeights(weights);
  kkt.AssembleAndFactor();

  RowSpace s_proj = kkt.MakeRowSpace();

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
    //    Solve (A^T A) x = A^T (b - s_proj), then s = b - A x.
    RowSpace r = addScaled(b, s_proj, 1.0, -1.0);

    auto rhs = kkt.MakeSolverRHS();
    rhs.SetZero();
    kkt.AccumulateAtranspose(r, rhs);
    kkt.SolveSolverRHS(rhs);

    // s = b - A * x.
    kkt.MultiplyA(rhs, s);
    s = addScaled(b, s, 1.0, -1.0);
  }

  return result;
}

}  // namespace conex
