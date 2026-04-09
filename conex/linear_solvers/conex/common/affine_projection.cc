#include "conex/common/affine_projection.h"

#include "conex/common/eja_ops.h"

namespace conex {

AffineProjection AffineProjection::Build(const Problem& problem) {
  AffineProjection ap;
  ap.solver_ = Solver::Build(problem);

  // Factor with identity weights: Gram = A^T I A = A^T A.
  // setOnes produces the EJA identity for each segment:
  //   nonneg → all-ones vector (diag(1) weighting)
  //   PSD    → identity matrix  (kron(I,I) weighting via Cholesky)
  auto* kkt = ap.solver_.solver();
  RowSpace weights = kkt->MakeRowSpace();
  EuclideanJordanAlgebra::setOnes(weights);
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
