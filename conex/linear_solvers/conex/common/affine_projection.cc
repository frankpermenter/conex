#include "conex/common/affine_projection.h"

#include "conex/common/eja_ops.h"

namespace conex {

AffineProjection AffineProjection::Build(const Problem& problem) {
  AffineProjection ap;
  ap.solver_ = Solver::Build(problem);

  // Factor with unit scalar weights: Gram = A^T I A = A^T A.
  // Set all entries to 1.0 directly (not via EJA setOnes, which
  // produces identity matrices for PSD segments).
  auto* kkt = ap.solver_.solver();
  RowSpace weights = kkt->MakeRowSpace();
  for (int i = 0; i < weights.num_constraints(); ++i)
    for (int j = 0; j < weights.sizes[i]; ++j)
      weights.segment_ptr(i)[j] = 1.0;
  kkt->SetWeights(weights);
  kkt->AssembleAndFactor();

  return ap;
}

void AffineProjection::Project(RowSpace& s) const {
  auto* kkt = solver_.solver();
  RowSpace b = kkt->GetAffineTerm();

  // Solve (A^T A) x = A^T (s - b).  Then s = Ax + b.
  RowSpace r = addScaled(s, b, 1.0, -1.0);
  auto rhs = kkt->MakeSolverRHS();
  rhs.SetZero();
  kkt->AccumulateAtranspose(r, rhs);
  kkt->SolveSolverRHS(rhs);

  // s = A*x + b.
  kkt->MultiplyA(rhs, s);
  s = addScaled(s, b, 1.0, 1.0);
}

RowSpace AffineProjection::MakeVariable() const {
  return solver_.solver()->MakeRowSpace();
}

RowSpace AffineProjection::GetAffineTerm() const {
  return solver_.solver()->GetAffineTerm();
}

}  // namespace conex
