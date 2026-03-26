#include "conex/common/sparse_quadratic_term.h"

#include <numeric>
#include <vector>

#include "conex/common/sparse_linear_constraint.h"
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Diagonal Q + banded A.  (Q + A^T A) should be full rank.
TEST(SparseQuadraticTerm, DiagonalQBandedA) {
  srand(42);
  const int n = 30, bw = 4, rpc = 5;
  int num_groups = n - bw + 1;
  int num_rows = rpc * num_groups;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < num_groups; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(num_rows, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Diagonal Q = 0.1 * I.
  std::vector<Eigen::Triplet<double>> q_trips;
  for (int i = 0; i < n; i++) q_trips.emplace_back(i, i, 0.1);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  MatrixXd Ad(A);
  MatrixXd Qd(Q);
  MatrixXd M = Qd + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Solve error: " << err;

  // Verify normal equation.
  VectorXd residual = M * result.x - rhs;
  EXPECT_LT(residual.norm(), 1e-8 * rhs.norm())
      << "Normal equation residual: " << residual.norm();
}

// Arrow-pattern Q: verify correct clique structure.
// Q has edges {0,3}, {1,3}, {2,3} — a star, NOT a 4-clique.
TEST(SparseQuadraticTerm, ArrowPatternQ) {
  const int n = 4;
  std::vector<Eigen::Triplet<double>> q_trips;
  // Diagonal.
  for (int i = 0; i < n; i++) q_trips.emplace_back(i, i, 1.0);
  // Off-diagonal: star centered at var 3.
  q_trips.emplace_back(0, 3, 0.1); q_trips.emplace_back(3, 0, 0.1);
  q_trips.emplace_back(1, 3, 0.2); q_trips.emplace_back(3, 1, 0.2);
  q_trips.emplace_back(2, 3, 0.3); q_trips.emplace_back(3, 2, 0.3);
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  // A is identity (just to have some A).
  Eigen::SparseMatrix<double> A(n, n);
  A.setIdentity();

  MatrixXd Qd(Q);
  MatrixXd M = Qd + MatrixXd::Identity(n, n);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Arrow Q solve error: " << err;
}

// Dense Q (as MatrixXd) + sparse A.
TEST(SparseQuadraticTerm, DenseQ) {
  srand(42);
  const int n = 15, m = 20;
  MatrixXd Q = MatrixXd::Random(n, n);
  Q = Q.transpose() * Q + 0.1 * MatrixXd::Identity(n, n);  // PSD

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int r = 0; r < m; r++)
    for (int j = 0; j < 3; j++)
      a_trips.emplace_back(r, rand() % n, 0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  MatrixXd Ad(A);
  MatrixXd M = Q + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Dense Q solve error: " << err;
}

// Q = 0: should reduce to standard least squares.
TEST(SparseQuadraticTerm, ZeroQ) {
  srand(42);
  const int n = 20, bw = 3, rpc = 5;
  int ng = n - bw + 1, nr = rpc * ng;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Q = 0 (empty sparse matrix).
  Eigen::SparseMatrix<double> Q(n, n);

  MatrixXd Ad(A);
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = Ad.transpose() * (Ad * x_true);

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-8) << "Zero Q solve error: " << err;
}

// Sparse tridiagonal Q + banded A: larger problem.
TEST(SparseQuadraticTerm, TridiagonalQ) {
  srand(42);
  const int n = 50, bw = 3, rpc = 4;
  int ng = n - bw + 1, nr = rpc * ng;

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int g = 0; g < ng; g++)
    for (int r = 0; r < rpc; r++)
      for (int j = 0; j < bw; j++)
        a_trips.emplace_back(g * rpc + r, g + j,
                             0.5 + (double)rand() / RAND_MAX);
  Eigen::SparseMatrix<double> A(nr, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Tridiagonal Q: Q(i,i) = 2, Q(i,i+1) = Q(i+1,i) = -0.5.
  std::vector<Eigen::Triplet<double>> q_trips;
  for (int i = 0; i < n; i++) {
    q_trips.emplace_back(i, i, 2.0);
    if (i + 1 < n) {
      q_trips.emplace_back(i, i + 1, -0.5);
      q_trips.emplace_back(i + 1, i, -0.5);
    }
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(q_trips.begin(), q_trips.end());

  MatrixXd Ad(A);
  MatrixXd Qd(Q);
  MatrixXd M = Qd + Ad.transpose() * Ad;
  VectorXd x_true = VectorXd::Random(n);
  VectorXd rhs = M * x_true;

  auto result = SparseQuadraticTermLeastSquares(Q, A, rhs);

  double err = (result.x - x_true).norm() / x_true.norm();
  EXPECT_LT(err, 1e-10) << "Tridiagonal Q solve error: " << err;

  VectorXd residual = M * result.x - rhs;
  EXPECT_LT(residual.norm(), 1e-8 * rhs.norm());
}

}  // namespace
}  // namespace conex
