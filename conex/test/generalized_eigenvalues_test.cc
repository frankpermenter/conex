#include "conex/generalized_eigenvalues.h"

#include <chrono>

#include "gtest/gtest.h"

#include "conex/debug_macros.h"
#include "conex/test/test_util.h"
#include <Eigen/Dense>

namespace conex {

using Eigen::LLT;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void DoBasicTest(const MatrixXd& A, const MatrixXd& B,
                 bool verify_eigenvalues = true) {
  int n = A.rows();
  VectorXd r0 = VectorXd::Random(n, 1);

  VectorXd eig_calc = GeneralizedEigenvalues(A, B, r0, n);
  MatrixXd AinvB = LLT<MatrixXd>(A).solve(B);
  auto eig_ref = eig(AinvB).eigenvalues;

  std::sort(eig_calc.data(), eig_calc.data() + eig_calc.size());
  std::sort(eig_ref.data(), eig_ref.data() + eig_ref.size());
  if (verify_eigenvalues) {
    for (int i = 0; i < n; i++) {
      EXPECT_NEAR(eig_calc(i), eig_ref(i), 1e-12);
    }
  }

  double upper_bound = 1e30;
  double lower_bound = -1e30;
  for (int i = 0; i < eig_calc.size(); i++) {
    if (-1.0 / eig_calc(i) > 0) {
      if (-1.0 / eig_calc(i) < upper_bound) {
        upper_bound = -1.0 / eig_calc(i);
      }
    } else {
      if (-1.0 / eig_calc(i) > lower_bound) {
        lower_bound = -1.0 / eig_calc(i);
      }
    }
  }
  EXPECT_NEAR(eig(A + upper_bound * B).eigenvalues.minCoeff(), 0, 1e-12);
  EXPECT_NEAR(eig(A + lower_bound * B).eigenvalues.minCoeff(), 0, 1e-12);
  EXPECT_TRUE(
      eig(A + (upper_bound + lower_bound) * .5 * B).eigenvalues.minCoeff() > 0);
}

GTEST_TEST(Eigenvalues, BasicTest) {
  int n = 4;
  MatrixXd A(n, n);
  // clang-format off
  A << 3, 1, 0, 1, 
       1, 3, 1, 0, 
       0, 1, 4, 1, 
       1, 0, 1, 5;
  // clang-format on
  A = A / A.trace();

  MatrixXd B = MatrixXd::Identity(n, n);

  B = MatrixXd::Random(n, n);
  MatrixXd Bt = B.transpose();
  B = B + Bt;

  DoBasicTest(A, B);
}

GTEST_TEST(RepeatedEigenvalues, BasicTest) {
  int n = 4;
  MatrixXd A(n, n);
  // clang-format off
  A << 3, 0, 0, 0, 
       0, 3.0, 0, 0, 
       0, 0, -4, 0, 
       0, 0, 0, -4;
  // clang-format on

  MatrixXd B = MatrixXd::Identity(n, n);

  DoBasicTest(B, A, false /*Don't verify eigenvalues*/);
}

GTEST_TEST(Eigenvalues, RandomTest) {
  int n = 8;
  DoBasicTest(RandomPSD(n), RandomSym(n));
}

}  // namespace conex
