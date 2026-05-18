#include "conex/linear_solvers/cholesky_solvers.h"

#ifdef EIGEN_USE_BLAS
#include <lapacke.h>
#endif

namespace conex {

int DynamicSubsystem::Dsytrf(int n, double* A, int lda, int* ipiv,
                              double* work, int lwork) {
#ifdef EIGEN_USE_BLAS
  // Use LAPACKE row-major wrapper which handles workspace internally.
  // Note: LAPACK_COL_MAJOR matches Eigen's column-major storage.
  lapack_int info = LAPACKE_dsytrf_work(LAPACK_COL_MAJOR, 'L', n, A, lda,
                                         ipiv, work, lwork);
  return static_cast<int>(info);
#else
  // Fallback: use Eigen's LDLT.
  Eigen::Map<Eigen::MatrixXd> M(A, n, n);
  Eigen::LDLT<Eigen::MatrixXd> ldlt(M);
  auto D = ldlt.vectorD();
  for (int i = 0; i < n; ++i) {
    A[i * lda + i] = D(i);
    ipiv[i] = i + 1;
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
