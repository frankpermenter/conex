// Solver comparison: geodesic IPM vs barrier method on random LPs.
// Reports duality gap / complementarity vs iteration count.
//
// Usage: ./solver_comparison [m] [n] [seed]
//   Default m=50, n=20, seed=42.

#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/barrier_qp.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Generate a random feasible LP: min c^T x s.t. Ax <= b.
// x0 = 0 is strictly feasible (b > 0).
// c = A^T * ones (so dual = ones is optimal-ish).
struct RandomLP {
  Eigen::SparseMatrix<double> A;
  VectorXd b;
  VectorXd c;
  int m, n;
};

RandomLP MakeRandomLP(int m, int n, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n);

  // b = A * 0 + 1 = ones, so x=0 is strictly feasible with slack = ones.
  VectorXd b = VectorXd::Ones(m);

  // c = A^T * ones — the LP has a finite optimum.
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  return {A, b, c, m, n};
}

void RunComparison(int m, int n, int seed) {
  auto lp = MakeRandomLP(m, n, seed);
  printf("LP: m=%d constraints, n=%d variables (seed=%d)\n\n", m, n, seed);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // ===== Barrier method =====
  {
    Eigen::SparseMatrix<double> Q(n, n);  // zero quadratic
    VectorXd x0 = VectorXd::Zero(n);      // feasible start

    auto result = SolveBarrierQP(Q, lp.c, lp.A, lp.b, x0,
                                  30, 50, 10.0, 1e-8);

    printf("=== Barrier Method ===\n");
    printf("  %d outer iterations, %d factorizations, %d solves\n",
           result.outer_iterations, result.total_newton_steps,
           result.total_newton_steps);
    printf("  %3s  %12s  %12s  %8s\n", "out", "mu", "gap", "newton");
    printf("  %s\n", std::string(40, '-').c_str());
    int cumulative_newton = 0;
    for (const auto& s : result.iter_stats) {
      cumulative_newton += s.newton_steps;
      printf("  %3d  %12.4e  %12.4e  %8d\n",
             static_cast<int>(&s - result.iter_stats.data()),
             s.mu, s.duality_gap, cumulative_newton);
    }
    printf("\n");
  }

  // ===== Geodesic IPM (1 centering step) =====
  {
    // Model Ax <= b as -Ax <= -b (i.e., Ax >= b in cone_program convention).
    // At W=ones, k=1 the geodesic has d=0 when cost = A^T ones and b = ones.
    // The API stores (-A, -b) so the sign mapping gives the correct fixed point.
    Eigen::SparseMatrix<double> negA = -lp.A;
    VectorXd neg_b = -lp.b;
    Problem problem;
    problem.AddLinearConstraint(negA, neg_b, vars);
    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();

    auto cost_rhs = kkt->MakeSolverRHS();
    VectorXd c_r = expansion.Reduce(lp.c);
    cost_rhs = kkt->MakeBlockVariable(c_r);

    VectorXd W = VectorXd::Ones(m);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 1, 1e-8);

    printf("=== Geodesic IPM (1 centering step) ===\n");
    printf("  %d outer, %d factorizations, %d solves\n",
           result.iterations, result.total_factorizations, result.total_solves);
    printf("  %3s  %12s  %12s  %12s  %12s\n",
           "out", "gap/m", "gap", "d_inf", "d_sqr");
    printf("  %s\n", std::string(56, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr);
    }
    printf("\n");
  }

  // ===== Geodesic IPM (full centering) =====
  {
    Eigen::SparseMatrix<double> negA = -lp.A;
    VectorXd neg_b = -lp.b;
    Problem problem;
    problem.AddLinearConstraint(negA, neg_b, vars);
    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();

    auto cost_rhs = kkt->MakeSolverRHS();
    VectorXd c_r = expansion.Reduce(lp.c);
    cost_rhs = kkt->MakeBlockVariable(c_r);

    VectorXd W = VectorXd::Ones(m);

    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 100, 1e-8);

    printf("=== Geodesic IPM (full centering) ===\n");
    printf("  %d outer, %d factorizations, %d solves\n",
           result.iterations, result.total_factorizations, result.total_solves);
    printf("  %3s  %12s  %12s  %12s  %12s\n",
           "out", "gap/m", "gap", "d_inf", "d_sqr");
    printf("  %s\n", std::string(56, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr);
    }
    printf("\n");
  }

  // ===== Geodesic IPM (Hybrid) =====
  {
    Eigen::SparseMatrix<double> negA = -lp.A;
    VectorXd neg_b = -lp.b;
    Problem problem;
    problem.AddLinearConstraint(negA, neg_b, vars);
    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();

    auto cost_rhs = kkt->MakeSolverRHS();
    VectorXd c_r = expansion.Reduce(lp.c);
    cost_rhs = kkt->MakeBlockVariable(c_r);

    VectorXd W = VectorXd::Ones(m);

    auto result = SolveGeodesicHybrid(*kkt, cost_rhs, W, 50, 1e-8, true);

    printf("=== Geodesic IPM (Hybrid) ===\n");
    printf("  %d outer, %d factorizations, %d solves\n",
           result.iterations, result.total_factorizations, result.total_solves);
    printf("  %3s  %12s  %12s  %12s  %12s\n",
           "out", "gap/m", "gap", "d_inf", "d_sqr");
    printf("  %s\n", std::string(56, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr);
    }
    printf("\n");
  }

  // ===== Geodesic IPM (Mehrotra) =====
  {
    Eigen::SparseMatrix<double> negA = -lp.A;
    VectorXd neg_b = -lp.b;
    Problem problem;
    problem.AddLinearConstraint(negA, neg_b, vars);
    auto [reduced, expansion] = Preprocess(problem);
    auto solver = Solver::Build(reduced);
    auto* kkt = solver.solver();

    auto cost_rhs = kkt->MakeSolverRHS();
    VectorXd c_r = expansion.Reduce(lp.c);
    cost_rhs = kkt->MakeBlockVariable(c_r);

    VectorXd W = VectorXd::Ones(m);

    auto result = SolveGeodesicMehrotra(*kkt, cost_rhs, W, 50, 1e-8, true);

    printf("=== Geodesic IPM (Mehrotra) ===\n");
    printf("  %d iterations\n", result.iterations);
    printf("  %3s  %12s  %12s  %12s  %12s\n",
           "out", "gap/m", "gap", "d_inf", "d_sqr");
    printf("  %s\n", std::string(56, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr);
    }
    printf("\n");
  }
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int m = 50, n = 20, seed = 42;
  if (argc > 1) m = std::atoi(argv[1]);
  if (argc > 2) n = std::atoi(argv[2]);
  if (argc > 3) seed = std::atoi(argv[3]);
  conex::RunComparison(m, n, seed);
  return 0;
}
