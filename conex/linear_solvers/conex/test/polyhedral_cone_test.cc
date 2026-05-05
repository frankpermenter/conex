// Compare solving an LP via nonneg constraints vs polyhedral cone barrier.
//
// Nonneg:     min c^T x  s.t. Ax + b >= 0
// Polyhedral: min [c;0]^T w  s.t. [A,b]w >= 0, y = 1  (w = [x; y])
//
// Both should produce the same optimal x.

#include <gtest/gtest.h>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/algorithms/solve_strategies.h"
#include "conex/common/model.h"
#include "conex/common/polyhedral_cone_ops.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace conex {
namespace {

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (M(i, j) != 0)
        trips.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(trips.begin(), trips.end());
  return S;
}

TEST(PolyhedralCone, NonnegVsPolyhedral) {
  srand(99);
  const int n = 5, m = 10;

  MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() + 0.1 * MatrixXd::Ones(m, n);
  VectorXd b = VectorXd::Ones(m);
  // Cost: c = A^T * ones so that x=0 is on the central path at k=1.
  VectorXd c = A.transpose() * VectorXd::Ones(m);

  std::vector<int> vars_n(n);
  std::iota(vars_n.begin(), vars_n.end(), 0);

  // === Nonneg formulation ===
  Model nonneg_model;
  nonneg_model.AddLinearConstraint(toSparse(A), b, vars_n);
  nonneg_model.SetLinearCost(c);

  fprintf(stderr, "  Building nonneg solver...\n");
  auto nonneg_solver = Solver::Build(nonneg_model);
  printf("  Built nonneg. Running...\n");
  auto nonneg_cm = nonneg_solver.MakeCompiledModel();
  auto nonneg_result = GeodesicBarrierLP{1e-8, 30, true}.Run(nonneg_cm);

  printf("\n=== Nonneg formulation ===\n");
  printf("  iters=%d, gap=%.2e, mu=%.2e, c^Tx=%.6f\n",
         nonneg_result.iterations, nonneg_result.complementarity,
         nonneg_result.mu, c.dot(nonneg_result.x));

  // === Polyhedral formulation ===
  // w = [x; y] ∈ R^{n+1}.  C = [A, b] so Cw = Ax + by.
  // Constraint: w ∈ K = {w : Cw >= 0}, via identity map (slack = w).
  // Equality: y = 1.  Cost: [c; 0]^T w.
  const int np = n + 1;
  MatrixXd C(m, np);
  C.leftCols(n) = A;
  C.col(n) = b;

  EuclideanJordanAlgebra::PolyhedralConeOps poly_ops(C);

  // Constraint matrix = identity (the cone variable is w itself).
  MatrixXd I_np = MatrixXd::Identity(np, np);
  VectorXd b_zero = VectorXd::Zero(np);

  // Equality: e_{n+1}^T w = 1.
  MatrixXd E(1, np);
  E.setZero();
  E(0, n) = 1.0;
  VectorXd d(1);
  d(0) = 1.0;

  // Cost: [c; 0].
  VectorXd c_ext(np);
  c_ext.head(n) = c;
  c_ext(n) = 0;

  std::vector<int> vars_np(np);
  std::iota(vars_np.begin(), vars_np.end(), 0);

  Model poly_model;
  poly_model.AddBarrierConstraint(toSparse(I_np), b_zero, vars_np, &poly_ops);
  poly_model.AddEqualityConstraint(toSparse(E), d, vars_np);
  poly_model.SetLinearCost(c_ext);

  fprintf(stderr, "  Building polyhedral solver (nconstr=%d)...\n",
          poly_model.num_constraints());
  // Use BuildDense to skip structural rank / tree decomposition.
  auto poly_solver = Solver::BuildDense(poly_model);
  printf("  Built. num_vars=%d\n", poly_solver.kkt()->number_of_variables());
  auto poly_cm = poly_solver.MakeCompiledModel();

  // Enable arena zeroing (needed for BarrierGramEvaluator).
  if (auto* ts = poly_solver.tree_solver()) {
    ts->EnableAutoUpdateAtAssemble(true);
  }

  // Initialize z = w_0 where w_0 = [0; 1] (feasible: Cw_0 = b > 0).
  RowSpace z = poly_cm.MakeRowSpace();
  // Set z to the affine term... but b_zero = 0, so we need to set
  // z manually to a feasible interior point.
  VectorXd w0(np);
  w0.head(n).setZero();
  w0(n) = 1.0;  // y = 1, x = 0 → Cw = A*0 + b*1 = b > 0.
  z.col() = w0;

  auto poly_result = SolveGeodesicBarrierLP(poly_cm, z, 60, 1e-8, true);

  printf("\n=== Polyhedral formulation ===\n");
  printf("  iters=%d, gap=%.2e, mu=%.2e\n",
         poly_result.iterations, poly_result.complementarity,
         poly_result.mu);

  // Extract x from w = [x; y].  The solver works in reduced space;
  // poly_result.x has n+1 components (plus equality duals).
  // x_poly = poly_result.x(0..n-1).
  VectorXd x_poly = poly_result.x.head(n);
  double obj_poly = c.dot(x_poly);
  printf("  x_poly = [");
  for (int i = 0; i < n; ++i) printf("%.4f%s", x_poly(i), i<n-1?", ":"");
  printf("], c^Tx=%.6f\n", obj_poly);

  VectorXd x_nn = nonneg_result.x;
  double obj_nn = c.dot(x_nn);
  printf("  x_nn   = [");
  for (int i = 0; i < n; ++i) printf("%.4f%s", x_nn(i), i<n-1?", ":"");
  printf("], c^Tx=%.6f\n", obj_nn);

  printf("  ||x_nn - x_poly|| = %.2e\n", (x_nn - x_poly).norm());
  printf("  obj diff = %.2e\n", std::abs(obj_nn - obj_poly));

  // Both should converge.
  EXPECT_LT(nonneg_result.mu, 1e-6) << "Nonneg should converge";
  EXPECT_LT(poly_result.mu, 1e-3) << "Polyhedral should converge";

  // Objectives should match.
  EXPECT_NEAR(obj_nn, obj_poly, 1e-2) << "Objectives should match";
}

}  // namespace
}  // namespace conex
