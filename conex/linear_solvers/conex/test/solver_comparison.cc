// Solver comparison: geodesic IPM variants on random LPs.
// Reports gap vs iteration count and total factorizations/solves.
//
// Usage: ./solver_comparison [m] [n] [rank_Q] [seed]
//   Default m=50, n=20, rank_Q=0 (LP), seed=42.

#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/algorithms/geodesic_ipm.h"
#include "conex/common/eja_ops.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

struct RandomQP {
  Eigen::SparseMatrix<double> A;
  Eigen::SparseMatrix<double> Q;
  VectorXd b;
  VectorXd c;
  int m, n, rank_Q;
};

// Build:  min c^T x + 0.5 x^T Q x  s.t.  Ax >= b  (stored as -Ax <= -b).
//
//   A: m x n random dense.
//   b: ones(m).
//   c: A^T ones(m)  — central path at W=ones, k=1 when Q=0.
//   Q: R^T R where R is rank_Q x n random.  Q=0 when rank_Q=0.
//
// At x=0: slack = -b - (-A)*0 = -b = -ones  (infeasible for stored form).
// The geodesic IPM starts at W=ones and solves for the stored form.
RandomQP MakeRandomQP(int m, int n, int rank_Q, int seed) {
  srand(seed);
  MatrixXd A_dense = MatrixXd::Random(m, n);
  VectorXd b = VectorXd::Ones(m);
  VectorXd c = A_dense.transpose() * VectorXd::Ones(m);

  std::vector<Eigen::Triplet<double>> a_trips;
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < n; ++j)
      a_trips.emplace_back(i, j, A_dense(i, j));
  Eigen::SparseMatrix<double> A(m, n);
  A.setFromTriplets(a_trips.begin(), a_trips.end());

  // Q = R^T R where R is rank_Q x n.
  Eigen::SparseMatrix<double> Q(n, n);
  if (rank_Q > 0) {
    MatrixXd R = MatrixXd::Random(rank_Q, n);
    MatrixXd Q_dense = R.transpose() * R;
    std::vector<Eigen::Triplet<double>> q_trips;
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j)
        if (std::abs(Q_dense(i, j)) > 1e-14)
          q_trips.emplace_back(i, j, Q_dense(i, j));
    Q.resize(n, n);
    Q.setFromTriplets(q_trips.begin(), q_trips.end());
  }

  return {A, Q, b, c, m, n, rank_Q};
}

void PrintResult(const char* name, const GeodesicResult& result,
                 bool show_r_updates = false) {
  printf("=== %s ===\n", name);
  printf("  %d factorizations, %d solves\n",
         result.total_factorizations, result.total_solves);
  if (show_r_updates) {
    printf("  %3s  %12s  %12s  %12s  %12s  %6s  %12s\n",
           "fac", "gap/m", "gap", "d_inf", "d_sqr", "r_upd", "min_slack");
    printf("  %s\n", std::string(76, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e  %6d  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr,
             s.r_updates, s.min_slack);
    }
  } else {
    printf("  %3s  %12s  %12s  %12s  %12s\n",
           "fac", "gap/m", "gap", "d_inf", "d_sqr");
    printf("  %s\n", std::string(56, '-').c_str());
    for (size_t i = 0; i < result.iter_stats.size(); ++i) {
      const auto& s = result.iter_stats[i];
      printf("  %3d  %12.4e  %12.4e  %12.4e  %12.4e\n",
             static_cast<int>(i), s.mu, s.complementarity, s.d_inf, s.d_sqr);
    }
  }
  printf("\n");
}

// Build solver from LP with -A, -b (cone_program sign convention).
struct SolverSetup {
  Solver solver;
  SolverRHS cost_rhs;
};

SolverSetup BuildSolver(const RandomQP& qp, const std::vector<int>& vars) {
  Eigen::SparseMatrix<double> negA = -qp.A;
  VectorXd neg_b = -qp.b;
  Problem problem;
  problem.AddLinearConstraint(negA, neg_b, vars);
  if (qp.rank_Q > 0) problem.AddQuadraticCost(qp.Q, vars);
  problem.SetLinearCost(qp.c);
  auto [reduced, expansion] = Preprocess(problem);
  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();

  auto cost_rhs = kkt->MakeSolverRHS();
  VectorXd c_r = reduced.linear_cost();
  cost_rhs = kkt->MakeBlockVariable(c_r);

  return {std::move(solver), cost_rhs};
}

void RunComparison(int m, int n, int rank_Q, int seed) {
  auto qp = MakeRandomQP(m, n, rank_Q, seed);
  if (rank_Q > 0) {
    printf("QP: m=%d constraints, n=%d variables, rank(Q)=%d (seed=%d)\n\n",
           m, n, rank_Q, seed);
  } else {
    printf("LP: m=%d constraints, n=%d variables (seed=%d)\n\n", m, n, seed);
  }

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  // ===== Geodesic IPM (0 centering steps) =====
  {
    auto [solver, cost_rhs] = BuildSolver(qp, vars);
    RowSpace W = solver.solver()->MakeRowSpace();
    setOnes(W);
    auto result = SolveGeodesicLP(*solver.solver(), cost_rhs, W, 30, 0, 1e-8);
    PrintResult("Geodesic IPM (0 centering)", result);
  }

  // ===== Geodesic IPM (Hybrid) =====
  {
    auto [solver, cost_rhs] = BuildSolver(qp, vars);
    RowSpace W = solver.solver()->MakeRowSpace();
    setOnes(W);
    auto result = SolveGeodesicHybrid(*solver.solver(), cost_rhs, W, 50, 1e-8);
    PrintResult("Geodesic IPM (Hybrid)", result, true);
  }
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int m = 50, n = 20, rank_Q = 0, seed = 42;
  if (argc > 1) m = std::atoi(argv[1]);
  if (argc > 2) n = std::atoi(argv[2]);
  if (argc > 3) rank_Q = std::atoi(argv[3]);
  if (argc > 4) seed = std::atoi(argv[4]);
  conex::RunComparison(m, n, rank_Q, seed);
  return 0;
}
