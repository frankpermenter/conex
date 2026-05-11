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
#include "conex/linear_solvers/kkt_tree_solver.h"

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

// Benchmark: average iterations to reach gap targets across many random LPs.
TEST(PolyhedralCone, ConvergenceProfile) {
  const int n = 5, m = 10;
  const int num_problems = 200;
  const std::vector<double> gap_targets = {
      1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10,
      1e-11, 1e-12, 1e-13};
  const int max_iter = 40;

  // Accumulators: iters_to_target[method][target_idx] = list of iterations.
  std::vector<std::vector<int>> nn_iters(gap_targets.size());
  std::vector<std::vector<int>> poly_iters(gap_targets.size());
  std::vector<std::vector<int>> sym_iters(gap_targets.size());

  for (int prob = 0; prob < num_problems; ++prob) {
    srand(prob + 1);
    MatrixXd A = MatrixXd::Random(m, n).cwiseAbs() +
                 0.1 * MatrixXd::Ones(m, n);
    VectorXd b = VectorXd::Ones(m);
    VectorXd c = A.transpose() * VectorXd::Ones(m);

    std::vector<int> vars_n(n);
    std::iota(vars_n.begin(), vars_n.end(), 0);

    // Nonneg.
    Model nn_model;
    nn_model.AddLinearConstraint(toSparse(A), b, vars_n);
    nn_model.SetLinearCost(c);
    auto nn_solver = Solver::Build(nn_model);
    auto nn_cm = nn_solver.MakeCompiledModel();
    auto nn_result = GeodesicBarrierLP{1e-14, max_iter, 0, false}.Run(nn_cm);

    // Polyhedral.
    const int np = n + 1;
    MatrixXd C(m, np);
    C.leftCols(n) = A;
    C.col(n) = b;
    EuclideanJordanAlgebra::PolyhedralConeOps poly_ops(C);

    MatrixXd I_np = MatrixXd::Identity(np, np);
    VectorXd b_zero = VectorXd::Zero(np);
    MatrixXd E(1, np); E.setZero(); E(0, n) = 1.0;
    VectorXd d(1); d(0) = 1.0;
    VectorXd c_ext(np); c_ext.head(n) = c; c_ext(n) = 0;

    std::vector<int> vars_np(np);
    std::iota(vars_np.begin(), vars_np.end(), 0);

    Model poly_model;
    poly_model.AddBarrierConstraint(toSparse(I_np), b_zero, vars_np, &poly_ops);
    poly_model.AddEqualityConstraint(toSparse(E), d, vars_np);
    poly_model.SetLinearCost(c_ext);

    VectorXd w0(np); w0.head(n).setZero(); w0(n) = 1.0;

    auto run_poly = [&](const EuclideanJordanAlgebra::BarrierConeOperations* ops) {
      Model m;
      m.AddBarrierConstraint(toSparse(I_np), b_zero, vars_np, ops);
      m.AddEqualityConstraint(toSparse(E), d, vars_np);
      m.SetLinearCost(c_ext);
      auto s = Solver::BuildDense(m);
      auto cm = s.MakeCompiledModel();
      return GeodesicBarrierLP{1e-14, max_iter, 0, false, w0}.Run(cm);
    };

    auto poly_result = run_poly(&poly_ops);

    EuclideanJordanAlgebra::PolyhedralConeOps sym_ops(C, /*use_symmetric=*/true);
    auto sym_result = run_poly(&sym_ops);

    // Record iterations to each gap target.
    for (int ti = 0; ti < (int)gap_targets.size(); ++ti) {
      double tgt = gap_targets[ti];
      int nn_it = max_iter;
      for (int i = 0; i < (int)nn_result.iter_stats.size(); ++i) {
        if (nn_result.iter_stats[i].complementarity < tgt &&
            nn_result.iter_stats[i].complementarity > 0) {
          nn_it = i + 1; break;
        }
      }
      nn_iters[ti].push_back(nn_it);

      int poly_it = max_iter;
      for (int i = 0; i < (int)poly_result.iter_stats.size(); ++i) {
        if (poly_result.iter_stats[i].complementarity < tgt &&
            poly_result.iter_stats[i].complementarity > 0) {
          poly_it = i + 1; break;
        }
      }
      poly_iters[ti].push_back(poly_it);

      int sym_it = max_iter;
      for (int i = 0; i < (int)sym_result.iter_stats.size(); ++i) {
        if (sym_result.iter_stats[i].complementarity < tgt &&
            sym_result.iter_stats[i].complementarity > 0) {
          sym_it = i + 1; break;
        }
      }
      sym_iters[ti].push_back(sym_it);
    }
  }

  // Print average iterations.
  printf("\n=== Average iterations to gap target (%d problems, n=%d, m=%d) ===\n",
         num_problems, n, m);
  printf("  %10s  %12s  %12s  %12s  %8s  %8s\n",
         "gap_target", "nonneg", "verlet", "symmetric", "v_ratio", "s_ratio");
  printf("  %s\n", std::string(72, '-').c_str());
  for (int ti = 0; ti < (int)gap_targets.size(); ++ti) {
    double nn_avg = 0, poly_avg = 0, sym_avg = 0;
    for (int v : nn_iters[ti]) nn_avg += v;
    for (int v : poly_iters[ti]) poly_avg += v;
    for (int v : sym_iters[ti]) sym_avg += v;
    nn_avg /= num_problems;
    poly_avg /= num_problems;
    sym_avg /= num_problems;
    printf("  %10.0e  %12.2f  %12.2f  %12.2f  %8.2f  %8.2f\n",
           gap_targets[ti], nn_avg, poly_avg, sym_avg,
           poly_avg / nn_avg, sym_avg / nn_avg);
  }

  // Basic sanity: both should converge on most problems.
  double nn_avg_final = 0, poly_avg_final = 0;
  for (int v : nn_iters.back()) nn_avg_final += v;
  for (int v : poly_iters.back()) poly_avg_final += v;
  nn_avg_final /= num_problems;
  poly_avg_final /= num_problems;
  EXPECT_LT(nn_avg_final, max_iter - 1) << "Nonneg should converge";
  EXPECT_LT(poly_avg_final, max_iter - 1) << "Polyhedral should converge";
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

  auto nonneg_solver = Solver::Build(nonneg_model);
  auto nonneg_cm = nonneg_solver.MakeCompiledModel();
  auto nonneg_result = GeodesicBarrierLP{1e-8, 30, 0, false}.Run(nonneg_cm);

  printf("\n=== Nonneg formulation ===\n");
  printf("  iters=%d, gap=%.2e, mu=%.2e, c^Tx=%.6f\n",
         nonneg_result.iterations, nonneg_result.complementarity,
         nonneg_result.mu, c.dot(nonneg_result.x));

  // === Polyhedral formulation ===
  const int np = n + 1;
  MatrixXd C(m, np);
  C.leftCols(n) = A;
  C.col(n) = b;

  EuclideanJordanAlgebra::PolyhedralConeOps poly_ops(C);

  MatrixXd I_np = MatrixXd::Identity(np, np);
  VectorXd b_zero = VectorXd::Zero(np);
  MatrixXd E(1, np); E.setZero(); E(0, n) = 1.0;
  VectorXd d(1); d(0) = 1.0;
  VectorXd c_ext(np); c_ext.head(n) = c; c_ext(n) = 0;

  std::vector<int> vars_np(np);
  std::iota(vars_np.begin(), vars_np.end(), 0);

  Model poly_model;
  poly_model.AddBarrierConstraint(toSparse(I_np), b_zero, vars_np, &poly_ops);
  poly_model.AddEqualityConstraint(toSparse(E), d, vars_np);
  poly_model.SetLinearCost(c_ext);

  VectorXd w0(np);
  w0.head(n).setZero();
  w0(n) = 1.0;

  auto poly_solver = Solver::BuildDense(poly_model);
  auto poly_cm = poly_solver.MakeCompiledModel();
  auto poly_result = GeodesicBarrierLP{1e-8, 60, 0, true, w0}.Run(poly_cm);

  // Frozen-J comparison.
  auto poly_solver_f1 = Solver::BuildDense(poly_model);
  auto poly_cm_f1 = poly_solver_f1.MakeCompiledModel();
  auto poly_f1 = GeodesicBarrierLP{1e-8, 60, 1, true, w0}.Run(poly_cm_f1);

  auto poly_solver_f2 = Solver::BuildDense(poly_model);
  auto poly_cm_f2 = poly_solver_f2.MakeCompiledModel();
  auto poly_f2 = GeodesicBarrierLP{1e-8, 60, 2, true, w0}.Run(poly_cm_f2);

  printf("\n=== Polyhedral formulation ===\n");
  printf("  frozen=0: %2d iters, %2d fac, gap=%.2e, mu=%.2e\n",
         poly_result.iterations, poly_result.total_factorizations,
         poly_result.complementarity, poly_result.mu);
  printf("  frozen=1: %2d iters, %2d fac, gap=%.2e, mu=%.2e\n",
         poly_f1.iterations, poly_f1.total_factorizations,
         poly_f1.complementarity, poly_f1.mu);
  printf("  frozen=2: %2d iters, %2d fac, gap=%.2e, mu=%.2e\n",
         poly_f2.iterations, poly_f2.total_factorizations,
         poly_f2.complementarity, poly_f2.mu);
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
