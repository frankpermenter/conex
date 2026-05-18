#include "conex/linear_solvers/cholesky_solvers.h"

#ifdef EIGEN_USE_BLAS
#include <lapacke.h>
#endif

namespace conex {

int DynamicSubsystem::Dsytrf(int n, double* A, int lda, int* ipiv,
                              double* work, int lwork) {
#ifdef EIGEN_USE_BLAS
  (void)work;
  (void)lwork;
  // LAPACKE_dsytrf handles workspace internally.
  return LAPACKE_dsytrf(LAPACK_COL_MAJOR, 'L', n, A, lda, ipiv);
#else
  // Fallback: use Eigen's LDLT.
  Eigen::Map<Eigen::MatrixXd> M(A, n, n);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
  auto D = ldlt.vectorD();
  for (int i = 0; i < n; ++i) {
    A[i * lda + i] = D(i);
    ipiv[i] = i + 1;  // 1-based, positive = 1x1 pivot
  }
  if (work) work[0] = 1;
  return (ldlt.info() == Eigen::Success) ? 0 : 1;
#endif
}

void DynamicSubsystem::Dsytrs(int n, int nrhs, const double* A, int lda,
                               const int* ipiv, double* B, int ldb) {
#ifdef EIGEN_USE_BLAS
  LAPACKE_dsytrs(LAPACK_COL_MAJOR, 'L', n, nrhs, A, lda, ipiv, B, ldb);
#else
  // Fallback: re-factor and solve.
  Eigen::Map<const Eigen::MatrixXd> M(A, n, n);
  Eigen::Map<Eigen::MatrixXd> Bm(B, ldb, nrhs);
  Eigen::MatrixXd full(n, n);
  full.triangularView<Eigen::Lower>() = M;
  full.triangularView<Eigen::StrictlyUpper>() = M.transpose();
  Eigen::LDLT<Eigen::MatrixXd> ldlt(full);
  Bm.topRows(n) = ldlt.solve(Bm.topRows(n));
#endif
}

}  // namespace conex
