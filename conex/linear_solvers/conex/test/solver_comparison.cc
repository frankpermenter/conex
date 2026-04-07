// Solver comparison: geodesic IPM variants on random LPs.
// Reports gap vs iteration count and total factorizations/solves.
//
// Usage: ./solver_comparison [m] [n] [seed]
//   Default m=50, n=20, seed=42.

#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct RandomLP {
  Eigen::SparseMatrix<double> A;
  VectorXd b;
  VectorXd c;
  int m, n;
};

RandomLP MakeRandomLP(int m, int n, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<Eigen::Triplet<double>> trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(trips.begin(), trips.end());

  return {A, b, c, m, n};
}

void PrintResult(const char* name, const GeodesicResult& result) {
  printf("=== %s ===\n", name);
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

// Build solver from LP with -A, -b (cone_program sign convention).
struct SolverSetup {
  Solver solver;
  SolverRHS cost_rhs;
};

SolverSetup BuildSolver(const RandomLP& lp, const std::vector<int>& vars) {
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

  return {std::move(solver), cost_rhs};
}

void RunComparison(int m, int n, int seed) {
  auto lp = MakeRandomLP(m, n, seed);
  printf("LP: m=%d constraints, n=%d variables (seed=%d)\n\n", m, n, seed);

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // ===== Geodesic IPM (1 centering step) =====
  {
    auto [solver, cost_rhs] = BuildSolver(lp, vars);
    VectorXd W = VectorXd::Ones(m);
    auto result = SolveGeodesicLP(*solver.solver(), cost_rhs, W, 30, 1, 1e-8);
    PrintResult("Geodesic IPM (1 centering step)", result);
  }

  // ===== Geodesic IPM (Hybrid) =====
  {
    auto [solver, cost_rhs] = BuildSolver(lp, vars);
    VectorXd W = VectorXd::Ones(m);
    auto result = SolveGeodesicHybrid(*solver.solver(), cost_rhs, W, 50, 1e-8);
    PrintResult("Geodesic IPM (Hybrid)", result);
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
