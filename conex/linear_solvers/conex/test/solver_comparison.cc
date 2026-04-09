// Solver comparison: geodesic IPM variants on random LPs.
// Reports gap vs iteration count and total factorizations/solves.
//
// Usage: ./solver_comparison [m] [n] [rank_Q] [seed] [sdp_n] [sdp_p]
//   LP:  m constraints, n variables, rank_Q (0=LP). Default 50 20 0 42.
//   SDP: sdp_n matrix dim, sdp_p variables. Default 4 6.

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

// Build:  min c^T x + 0.5 x^T Q x  s.t.  Ax + b >= 0 
//
//   A: m x n random dense.
//   b: ones(m).
//   c: A^T ones(m)  
//   Q: R^T R where R is rank_Q x n random.  Q=0 when rank_Q=0.
//
//  Choice of (b, c) imply ones(m) is on central path with x = 0.
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
  Problem problem;
  problem.AddLinearConstraint(qp.A, qp.b, vars);
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

Eigen::SparseMatrix<double> toSparse(const MatrixXd& M) {
  std::vector<Eigen::Triplet<double>> t;
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j)
      if (std::abs(M(i, j)) > 1e-14)
        t.emplace_back(i, j, M(i, j));
  Eigen::SparseMatrix<double> S(M.rows(), M.cols());
  S.setFromTriplets(t.begin(), t.end());
  return S;
}

// Build:  min c^T x  s.t.  B + Σ x_j A_j ≽ 0
//
//   A_j: n x n random symmetric.
//   B: identity.
//   c_j: trace(A_j)   (so W=I at k=1 is centered with x=0).
//   p: number of free variables.
void RunSDPComparison(int n, int p, int seed) {
  srand(seed);
  printf("SDP: n=%d (matrix dim), p=%d variables (seed=%d)\n\n", n, p, seed);

  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> vars;
  VectorXd c(p);
  for (int k = 0; k < p; ++k) {
    MatrixXd Ak = MatrixXd::Random(n, n);
    Ak = 0.5 * (Ak + Ak.transpose());
    A_list.push_back(toSparse(Ak));
    vars.push_back(k);
    c(k) = Ak.trace();
  }
  MatrixXd B = MatrixXd::Identity(n, n);

  Problem problem;
  problem.AddPSDConstraint(A_list, toSparse(B), vars, /*use_chordal=*/false);
  problem.SetLinearCost(c);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.solver();
  auto cost_rhs = kkt->MakeSolverRHS();
  cost_rhs = kkt->MakeBlockVariable(c);

  // ===== Geodesic IPM (0 centering steps) =====
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    auto result = SolveGeodesicLP(*kkt, cost_rhs, W, 30, 0, 1e-8);
    PrintResult("SDP Geodesic IPM (0 centering)", result);
  }

  // ===== Geodesic IPM (Hybrid) =====
  {
    RowSpace W = kkt->MakeRowSpace();
    setOnes(W);
    auto result = SolveGeodesicHybrid(*kkt, cost_rhs, W, 50, 1e-8);
    PrintResult("SDP Geodesic IPM (Hybrid)", result, true);
  }
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  // Usage: ./solver_comparison [m] [n] [rank_Q] [seed] [sdp_n] [sdp_p]
  int m = 50, n = 20, rank_Q = 0, seed = 42;
  int sdp_n = 4, sdp_p = 6;
  if (argc > 1) m = std::atoi(argv[1]);
  if (argc > 2) n = std::atoi(argv[2]);
  if (argc > 3) rank_Q = std::atoi(argv[3]);
  if (argc > 4) seed = std::atoi(argv[4]);
  if (argc > 5) sdp_n = std::atoi(argv[5]);
  if (argc > 6) sdp_p = std::atoi(argv[6]);
  conex::RunComparison(m, n, rank_Q, seed);

  printf("\n%s\n\n", std::string(72, '=').c_str());
  conex::RunSDPComparison(sdp_n, sdp_p, seed);
  return 0;
}
