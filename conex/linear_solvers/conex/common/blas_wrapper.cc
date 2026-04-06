#include "conex/common/blas_wrapper.h"

#ifdef EIGEN_USE_BLAS
#include <cblas.h>
#endif

namespace conex { namespace blas {

bool Dsyrk(int n, int m, double alpha, const double* A, int lda,
           double* C, int ldc) {
#ifdef EIGEN_USE_BLAS
  cblas_dsyrk(CblasColMajor, CblasLower, CblasTrans,
              n, m, alpha, A, lda, 1.0, C, ldc);
  return true;
#else
  (void)n; (void)m; (void)alpha; (void)A; (void)lda; (void)C; (void)ldc;
  return false;
#endif
}

bool Dgemm(int rows, int cols, int m, double alpha,
           const double* A, int lda, const double* B, int ldb,
           double* C, int ldc) {
#ifdef EIGEN_USE_BLAS
  cblas_dgemm(CblasColMajor, CblasTrans, CblasNoTrans,
              rows, cols, m, alpha, A, lda, B, ldb, 1.0, C, ldc);
  return true;
#else
  (void)rows; (void)cols; (void)m; (void)alpha;
  (void)A; (void)lda; (void)B; (void)ldb; (void)C; (void)ldc;
  return false;
#endif
}

}}  // namespace conex::blas
