#pragma once
#include "conex/debug_macros.h"
#include <Eigen/Dense>
using Eigen::MatrixXd;
void CalcDenseLtdlInPlace(Eigen::Ref<MatrixXd> A) {
  const int n = A.rows();
  for (int k = n - 1; k >= 0; --k) {
    const double a_kk_inv = 1.0/A(k, k);
    const double a_kk_sqrt_inv = std::sqrt(a_kk_inv); 
    A(k, k) *= a_kk_sqrt_inv;
    for (int i = k - 1; i >= 0; --i) {
      const double a = A(k, i) * a_kk_inv;
      for (int j = i; j >= 0; j--) {
        A(i, j) -= a * A(k, j);
      }
      A(k, i) *= a_kk_sqrt_inv;
    }
  }
}

void DenseCholeskyInPlace(Eigen::Ref<MatrixXd> A) {
  const int n = A.rows();
  for (int k = 0; k < n; k++) {
    const double a = sqrt(A(k, k));
    for (int i = k; i < n; i++) {
      A(i, k) /= a;
      for (int j = k + 1; j <= i; j++) {
        A(i, j) -= A(i, k) * A(j, k);
      }
    }
  }
}

// Factor as U U^T where U is upper triangular.
//
// For upper-triangular U = [u0, u1, u2], the product U U^T
// decomposes as
//
//     u_0u^T_0  u_1u^T_1   u_2u^T_2
//  A = * 0 0     * * 0     * * *
//      0 0 0  +  * * 0  +  * * *
//      0 0 0     0 0 0     * * *
//
//  So, we compute the
void DenseCholeskyInPlaceUpperTriVect(Eigen::Ref<MatrixXd> A) {
  const int n = A.rows();
  auto& U = A;
  for (int k = n - 1; k > 0; k--) {
 // U.col(n - 1).head(n).array() /= std::sqrt(A(n - 1, n - 1));
    auto Uk = U.col(k);
    Uk /= std::sqrt(A(k, k));
    for (int j = k - 1; j >= 0; j--) {
      U.col(j).head(k) -= Uk.head(k) * U(j, k);
    }
    //U.col(k - 1).head(k).array() /= std::sqrt(A(k - 1, k - 1));
  }
  U(0, 0) /= std::sqrt(U(0, 0));
}


void DenseCholeskyInPlaceUpperTriPartialVect(Eigen::Ref<MatrixXd> A) {
  const int n = A.rows();
  auto& U = A;
  for (int k = n - 1; k >= 0; k--) {
    const double a_kk_inv = 1.0/U(k, k);
    const double a_kk_sqrt_inv = std::sqrt(a_kk_inv); 
    U.col(k).head(k).array() *= a_kk_sqrt_inv;
    U(k, k) *= a_kk_sqrt_inv;
    for (int j = k - 1; j >= 0; j--) {
      for (int i = j; i >= 0; i--) {
        U(i, j) -= U(i, k) * U(j, k);
      }
    }
  }
}

#define Adata(i, j) *(base + j * n + i)
void DenseCholeskyInPlaceUpperTriScalar(Eigen::Ref<MatrixXd> A) {
  const int n = A.rows();
  double* base = A.data();
  for (int k = n - 1; k >= 0; k--) {
    const double a_kk_inv = 1.0/Adata(k, k);
    const double a_kk_sqrt_inv = std::sqrt(a_kk_inv); 
    Adata(k, k) *= a_kk_sqrt_inv;
    for (int j = k - 1; j >= 0; j--) {
      const double a = Adata(j, k) * a_kk_inv;
      // Inner loop down rows
      for (int i = j; i >= 0; i--) {
        Adata(i, j) -= Adata(i, k) *  a;
      }
      Adata(j, k) *= a_kk_sqrt_inv;
    }
  }

// Faster! Why?
//  const int n = A.rows();
//  for (int k = n - 1; k >= 0; --k) {
//    const double a_kk_inv = 1.0/A(k, k);
//    const double a_kk_sqrt_inv = std::sqrt(a_kk_inv); 
//    A(k, k) *= a_kk_sqrt_inv;
//    for (int j = k - 1; j >= 0; --j) {
//      const double a = A(k, j) * a_kk_jnv;
//      for (int i = j; i >= 0; i--) {
//        A(j, i) -= a * A(k, i);
//      }
//      A(k, j) *= a_kk_sqrt_jnv;
//    }
//  }

}




void PartialDenseCholeskyInPlace(Eigen::Ref<MatrixXd> A,
                                 Eigen::Ref<MatrixXd> B) {
  const int n = A.rows();

  // Divide column k of by sqrt(A(k, k)) and
  // then subtract a_{k+1}:end, k} a_{k+1}:end, k}^T from bottom
  // right corner.
  for (int k = 0; k < n; k++) {
    double a = sqrt(A(k, k));
    // Subtract a_i a_j
    for (int i = k; i < n; i++) {
      A(i, k) /= a;
      const double a_ik = A(i, k);
      for (int j = k + 1; j <= i; j++) {
        A(i, j) -= a_ik * A(j, k);
      }
    }

    for (int i = 0; i < B.cols(); i++) {
      B(k, i) /= a;
      const double b_ik = B(k, i);
      for (int j = k + 1; j < n; j++) {
        B(j, i) -= b_ik * A(j, k);
      }
    }
  }
}





void PartialDenseCholeskyInPlace(Eigen::MatrixXd* Ainout) {
  auto& A = *Ainout;
  const int cols = A.cols();
  const int rows = A.rows();
  for (int k = 0; k < cols; k++) {
    double a = sqrt(A(k, k));
    for (int i = k; i < rows; i++) {
      for (int j = k + 1; j < std::min(i + 1, cols); j++) {
        A(i, j) -= A(i, k) * A(j, k);
      }
    }
  }
}

void EigenDenseCholeskyInPlace(Eigen::Ref<Eigen::MatrixXd> A) {
  Eigen::LLT<Eigen::Ref<Eigen::MatrixXd>> mat(A);
}

void DenseLDLTInPlace(Eigen::Ref<Eigen::MatrixXd> A) {
  const int n = A.rows();
  Eigen::VectorXd d(n);
  for (int k = 0; k < n; k++) {
    d(k) = A(k, k);
    for (int i = k; i < n; i++) {
      for (int j = k + 1; j <= i; j++) {
        A(i, j) -= A(i, k) * A(j, k) / d(k);
      }
    }
  }
  d = d.array().sqrt();
  A = A * d.cwiseInverse().asDiagonal();
}


//void PartialDenseLDLTInPlace(Eigen::Ref<MatrixXd> A,
//                                 Eigen::Ref<MatrixXd> B) {
//  const int n = A.rows();
//
//  // Divide column k of by sqrt(A(k, k)) and
//  // then subtract a_{k+1}:end, k} a_{k+1}:end, k}^T from bottom
//  // right corner.
//  for (int k = 0; k < n; k++) {
//    for (int i = k + 1; i < n; i++) {
//      for (int j = k + 1; j <= i; j++) {
//        A(i, j) -=  A(i, k)  * A(j, k);
//      }
//      A(i, k) /=  A(k, k);
//    }
//
//    for (int i = 0; i < B.cols(); i++) {
//      const double b = B(k, i) / A(k, k);
//      for (int j = k + 1; j < n; j++) {
//        B(j, i) -= b * A(j, k);
//      }
//      B(k, i) = b;
//    }
//  }
//}

void PartialDenseLDLTInPlace(Eigen::Ref<MatrixXd> A,
                                 Eigen::Ref<MatrixXd> B) {
  const int n = A.rows();

  // Divide column k of by sqrt(A(k, k)) and
  // then subtract a_{k+1}:end, k} a_{k+1}:end, k}^T from bottom
  // right corner.
  for (int k = 0; k < n; k++) {
    double a = sqrt(A(k, k));
    // Subtract a_i a_j
    for (int i = k; i < n; i++) {
      A(i, k) /= a;
      const double a_ik = A(i, k);
      for (int j = k + 1; j <= i; j++) {
        A(i, j) -= a_ik * A(j, k);
      }
    }

    for (int i = 0; i < B.cols(); i++) {
      B(k, i) /= a;
      const double b_ik = B(k, i);
      for (int j = k + 1; j < n; j++) {
        B(j, i) -= b_ik * A(j, k);
      }
    }
  }

  //Eigen::VectorXd d_sqrt = A.diagonal();
  //A = A * d_sqrt.cwiseInverse();
  //B = B * d_sqrt.cwiseInverse();
  //DUMP(A);
  //A.diagonal() = d_sqrt * d_sqrt;

}




