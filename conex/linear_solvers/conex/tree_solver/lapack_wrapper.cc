// Isolated compilation unit for LAPACK calls.
// lapacke.h defines macros (lapack_complex_float etc.) that conflict
// with C++ template headers, so it must not be included in headers
// or translation units that also include Eigen/gtest.

#ifdef EIGEN_USE_LAPACKE
#include <lapacke.h>
#endif

namespace conex { namespace internal {

int LapackPotrf(int n, double* data, int ld) {
#ifdef EIGEN_USE_LAPACKE
  return LAPACKE_dpotrf(LAPACK_COL_MAJOR, 'L', n, data, ld);
#else
  (void)n; (void)data; (void)ld;
  return -1;  // LAPACK not available.
#endif
}

}}  // namespace conex::internal
