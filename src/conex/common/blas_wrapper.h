#pragma once

namespace conex { namespace blas {

// C += alpha * A^T * A  (lower triangle only).
// A is m x n (column-major), C is n x n with leading dimension ldc.
// Returns true if BLAS was available, false to fall back to Eigen.
bool Dsyrk(int n, int m, double alpha, const double* A, int lda,
           double* C, int ldc);

// C += alpha * A^T * B
// A is m x rows (column-major), B is m x cols (column-major).
// C is rows x cols with leading dimension ldc.
bool Dgemm(int rows, int cols, int m, double alpha,
           const double* A, int lda, const double* B, int ldb,
           double* C, int ldc);

}}  // namespace conex::blas
