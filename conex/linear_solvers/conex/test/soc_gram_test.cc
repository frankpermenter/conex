#include <gtest/gtest.h>
#include <cstdio>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/soc_cone_ops.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

// Build the SOC Gram matrix from the definition:
//   G_ij = <P(W) A_i, A_j>
// where A_i is column i of the constraint matrix (a SOC element),
// P(W) is the quadratic representation, and <.,.> is the trace
// inner product (= 2 * Euclidean dot for SOC).
MatrixXd GramFromDefinition(const MatrixXd& A, const VectorXd& w) {
  const int n = A.rows();  // SOC dimension (1 + vector dim)
  const int p = A.cols();  // number of variables
  EuclideanJordanAlgebra::SOCConeOps ops;

  MatrixXd G(p, p);
  for (int i = 0; i < p; ++i) {
    VectorXd PWAi(n);
    ops.quadraticRepresentation(PWAi.data(), w.data(), A.col(i).data(), n);
    for (int j = 0; j <= i; ++j) {
      G(i, j) = ops.dot(PWAi.data(), A.col(j).data(), n);
      G(j, i) = G(i, j);
    }
  }
  return G;
}

// Build the SOC Gram matrix from the closed-form formula:
//   P(W) = 2ww^T - det(w) R
// where det(w) = w0^2 - ||w1||^2, R = diag(1, -1, ..., -1).
//   G = A^T P(W) A = 2(A^T w)(A^T w)^T - det(w) A^T R A
MatrixXd GramFromFormula(const MatrixXd& A, const VectorXd& w) {
  const int n = A.rows();
  const int p = A.cols();

  double w0 = w(0);
  VectorXd w1 = w.tail(n - 1);
  double det_w = w0 * w0 - w1.squaredNorm();

  // A^T w (p-vector).
  VectorXd Atw = A.transpose() * w;

  // A^T R A where R = diag(1, -1, ..., -1).
  // R A scales row 0 by +1, rows 1..n-1 by -1.
  VectorXd A0 = A.row(0).transpose();      // first row (p-vector)
  MatrixXd A1 = A.bottomRows(n - 1);       // remaining rows ((n-1) x p)
  MatrixXd AtRA = A0 * A0.transpose() - A1.transpose() * A1;

  // G = 2(Atw)(Atw)^T - det_w * AtRA
  // But the trace inner product has a factor of 2, so:
  //   G_ij = <P(W)A_i, A_j> = 2 * (P(W)A_i)^T A_j  (not quite)
  // Actually, for SOC the trace inner product is:
  //   <a, b> = 2(t_a t_b + x_a · x_b)
  // And <P(W)A_i, A_j> uses this inner product.
  //
  // P(W) as a matrix acts on vectors: P(W)v = (2ww^T - det(w)R) v.
  // Then <P(W)A_i, A_j> = A_j^T (2I_trace) P(W) A_i
  // where I_trace = diag(2, 2, ..., 2) / 2... no.
  //
  // Actually: <a, b>_SOC = 2(a0*b0 + a1·b1) = 2 a^T b for standard
  // Euclidean dot. So <P(W)A_i, A_j> = 2 * A_j^T P(W) A_i.
  // G_ij = 2 * A_j^T P(W) A_i = 2 * A_j^T (2ww^T - det(w)R) A_i
  //       = 2 * (2(A^Tw)_j(A^Tw)_i - det(w)(A^T R A)_{ji})
  //
  // So G = 2 * (2 Atw Atw^T - det_w * AtRA).

  return 2.0 * (2.0 * Atw * Atw.transpose() - det_w * AtRA);
}

TEST(SOCGram, DefinitionMatchesFormula) {
  srand(42);
  const int vec_dim = 5;     // vector part dimension
  const int n = 1 + vec_dim; // SOC dimension
  const int p = 4;           // number of variables

  // Random constraint matrix A (n x p).
  MatrixXd A = MatrixXd::Random(n, p);

  // Random weight w in the SOC interior (w0 > ||w1||).
  VectorXd w(n);
  w.tail(vec_dim) = 0.5 * VectorXd::Random(vec_dim);
  w(0) = w.tail(vec_dim).norm() + 1.0;  // ensure interior

  MatrixXd G_def = GramFromDefinition(A, w);
  MatrixXd G_form = GramFromFormula(A, w);

  printf("Gram (definition):\n");
  for (int i = 0; i < p; ++i) {
    for (int j = 0; j < p; ++j) printf(" %10.4f", G_def(i, j));
    printf("\n");
  }
  printf("Gram (formula):\n");
  for (int i = 0; i < p; ++i) {
    for (int j = 0; j < p; ++j) printf(" %10.4f", G_form(i, j));
    printf("\n");
  }

  double err = (G_def - G_form).lpNorm<Eigen::Infinity>();
  printf("max diff: %.2e\n", err);
  EXPECT_LT(err, 1e-10);

  // Also test with sparse A (some zero columns).
  MatrixXd A_sparse = MatrixXd::Zero(n, p + 2);
  A_sparse.leftCols(p) = A;
  // Columns p and p+1 are zero.
  VectorXd w2(n);
  w2.tail(vec_dim) = VectorXd::Random(vec_dim);
  w2(0) = w2.tail(vec_dim).norm() + 2.0;

  MatrixXd G2_def = GramFromDefinition(A_sparse, w2);
  MatrixXd G2_form = GramFromFormula(A_sparse, w2);
  double err2 = (G2_def - G2_form).lpNorm<Eigen::Infinity>();
  printf("sparse max diff: %.2e\n", err2);
  EXPECT_LT(err2, 1e-10);
}

// Test the full SOC constraint pipeline: Model → Solver → Gram.
// Verify the Gram from the tree solver matches the direct formula.
TEST(SOCGram, SolverGramMatchesFormula) {
  srand(77);
  const int vec_dim = 4;
  const int n = 1 + vec_dim;  // SOC dimension
  const int p = 3;            // variables

  MatrixXd A_dense = MatrixXd::Random(n, p);
  VectorXd b = VectorXd::Random(n);
  b(0) = std::abs(b(0)) + b.tail(vec_dim).norm() + 1.0;  // feasible at x=0

  // Random weight in SOC interior.
  VectorXd w(n);
  w.tail(vec_dim) = 0.3 * VectorXd::Random(vec_dim);
  w(0) = w.tail(vec_dim).norm() + 1.5;

  std::vector<int> vars(p);
  std::iota(vars.begin(), vars.end(), 0);

  // Build via solver.
  conex::Model problem;
  Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();
  problem.AddSOCConstraint(A_sparse, b, vars);
  auto solver = conex::Solver::Build(problem);
  auto* kkt = solver.solver();

  // Set scaling to w.
  conex::RowSpace W = kkt->MakeRowSpace();
  ASSERT_EQ(W.total_rows(), n);
  for (int i = 0; i < n; ++i) W.segment_ptr(0)[i] = w(i);
  kkt->SetScaling(W);
  kkt->AssembleAndFactor();

  // Extract Gram by solving with unit vectors.
  MatrixXd G_solver(p, p);
  for (int j = 0; j < p; ++j) {
    VectorXd ej = VectorXd::Zero(p);
    ej(j) = 1.0;
    G_solver.col(j) = kkt->Solve(ej);
  }
  // G_solver is Gram^{-1}.  Invert to get Gram.
  MatrixXd Gram_solver = G_solver.inverse();

  // Direct formula.
  MatrixXd Gram_formula = GramFromFormula(A_dense, w);

  double err = (Gram_solver - Gram_formula).lpNorm<Eigen::Infinity>();
  printf("Solver vs formula max diff: %.2e\n", err);
  EXPECT_LT(err, 1e-8);
}

}  // namespace
}  // namespace conex
