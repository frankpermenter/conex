#include "conex/generalized_eigenvalues.h"

#include <chrono>

#include "gtest/gtest.h"

#include "conex/debug_macros.h"
#include "conex/test/test_util.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LLT;


GTEST_TEST(Eigenvalues, BasicTest) {
  int n = 4;
  MatrixXd A(n, n);
  A << 3, 1, 0, 1, 
       1, 3, 1, 0, 
       0, 1, 4, 1, 
       1, 0, 1, 5;
  A = A / A.trace();

  VectorXd r0(n);
  r0 << 1, 2, 0, 4;
  MatrixXd B = MatrixXd::Identity(n, n);
  
  B = MatrixXd::Random(n, n);
  MatrixXd Bt = B.transpose(); B = B + Bt; 
  DUMP(B);
  A = A * A.transpose();

  VectorXd eig_calc =
      GeneralizedEigenvalues(A, B, r0, n);

  MatrixXd AinvB = LLT<MatrixXd>(A).solve(B);
  auto eig_ref = eig(AinvB).eigenvalues;
  DUMP(eig_calc);
  DUMP(eig_ref);

  std::sort(eig_calc.data(), eig_calc.data() + n);
  std::sort(eig_ref.data(), eig_ref.data() + n);
  for (int i = 0; i < n; i++) {
    EXPECT_NEAR(eig_calc(i), eig_ref(i), 1e-12);
  }

  double upper_bound = 1e30;
  double lower_bound = -1e30;
  for (int i = 0; i < n; i++) {
    if (-1.0/eig_calc(i) > 0)  {
      if (-1.0/eig_calc(i) < upper_bound) {
         upper_bound = -1.0/eig_calc(i);
      }
    } else {
      if (-1.0/eig_calc(i) > lower_bound) {
        lower_bound = -1.0/eig_calc(i);
      }
    }
  }

  DUMP(upper_bound);
  DUMP(lower_bound);
  double lambda_max = eig_calc.maxCoeff();
  DUMP(eig(A + upper_bound * B).eigenvalues.minCoeff());
  DUMP(eig(A + lower_bound * B).eigenvalues.minCoeff());


}

}  // namespace conex
