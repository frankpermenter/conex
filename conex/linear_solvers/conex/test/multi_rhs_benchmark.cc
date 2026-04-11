// Multi-RHS benchmark: compares single-column loops vs batched multi-column
// Solve and MultiplyA on an LQR-structured problem.
//
// Usage: ./multi_rhs_benchmark [T] [n_rhs_max]
//   Default T = 100, n_rhs_max = 16.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <vector>

#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/solver.h"
#include "conex/tree_solver/kkt_tree_solver.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace conex {
namespace {

using Clock = std::chrono::high_resolution_clock;
double us(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::micro>(b - a).count();
}

// Build an LQR problem and return the factored solver.
Solver BuildLQR(int nx, int nu, int T) {
  srand(42);
  Eigen::MatrixXd Ad = 0.9 * Eigen::MatrixXd::Identity(nx, nx) +
                        0.1 * Eigen::MatrixXd::Random(nx, nx);
  Eigen::MatrixXd Bd = Eigen::MatrixXd::Random(nx, nu);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
  Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(nu, nu);
  Eigen::MatrixXd Qf = 10.0 * Q;

  int step = nx + nu;
  int n_primal = T * step + nx;

  auto x_idx = [&](int t) -> std::vector<int> {
    std::vector<int> v(nx);
    std::iota(v.begin(), v.end(), t * step);
    return v;
  };
  auto u_idx = [&](int t) -> std::vector<int> {
    std::vector<int> v(nu);
    std::iota(v.begin(), v.end(), t * step + nx);
    return v;
  };
  auto xT_idx = [&]() -> std::vector<int> {
    std::vector<int> v(nx);
    std::iota(v.begin(), v.end(), T * step);
    return v;
  };

  std::vector<Eigen::Triplet<double>> qt;
  for (int t = 0; t < T; ++t) {
    auto xi = x_idx(t), ui = u_idx(t);
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (Q(r, c) != 0) qt.emplace_back(xi[r], xi[c], Q(r, c));
    for (int r = 0; r < nu; ++r)
      for (int c = 0; c < nu; ++c)
        if (R(r, c) != 0) qt.emplace_back(ui[r], ui[c], R(r, c));
  }
  {
    auto xi = xT_idx();
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (Qf(r, c) != 0) qt.emplace_back(xi[r], xi[c], Qf(r, c));
  }
  Eigen::SparseMatrix<double> Q_cost(n_primal, n_primal);
  Q_cost.setFromTriplets(qt.begin(), qt.end());

  std::vector<Eigen::Triplet<double>> ct;
  int eq_row = 0;
  for (int t = 0; t < T; ++t) {
    auto xt = x_idx(t), ut = u_idx(t);
    auto xt1 = (t < T - 1) ? x_idx(t + 1) : xT_idx();
    for (int r = 0; r < nx; ++r) {
      for (int c = 0; c < nx; ++c)
        if (Ad(r, c) != 0) ct.emplace_back(eq_row + r, xt[c], -Ad(r, c));
      for (int c = 0; c < nu; ++c)
        if (Bd(r, c) != 0) ct.emplace_back(eq_row + r, ut[c], -Bd(r, c));
      ct.emplace_back(eq_row + r, xt1[r], 1.0);
    }
    eq_row += nx;
  }
  for (int j = 0; j < nx; ++j)
    ct.emplace_back(eq_row + j, x_idx(0)[j], 1.0);
  int n_eq = eq_row + nx;
  Eigen::SparseMatrix<double> C_eq(n_eq, n_primal);
  C_eq.setFromTriplets(ct.begin(), ct.end());
  Eigen::VectorXd d_eq = Eigen::VectorXd::Zero(n_eq);

  std::vector<int> all_vars(n_primal);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  Problem problem;
  problem.AddQuadraticCost(Q_cost, all_vars);
  problem.AddEqualityConstraint(C_eq, d_eq, all_vars);

  auto solver = Solver::Build(problem);
  solver.solver()->AssembleAndFactor();
  return solver;
}

// Benchmark single-column loop vs multi-column batched solve.
void BenchmarkSolve(KKTSolverBase* kkt, int n_rhs, int repeats) {
  int n = kkt->number_of_variables();

  // Build random RHS columns.
  Eigen::MatrixXd rhs_data = Eigen::MatrixXd::Random(n, n_rhs);

  // --- Single-column loop ---
  auto t0 = Clock::now();
  for (int rep = 0; rep < repeats; ++rep) {
    for (int j = 0; j < n_rhs; ++j) {
      auto rhs = kkt->MakeSolverRHS();
      rhs.supernodes->ScatterFrom(rhs_data.col(j));
      rhs.blocks_fully_gathered = true;
      kkt->SolveSolverRHS(rhs);
    }
  }
  auto t1 = Clock::now();
  double single_us = us(t0, t1) / repeats;

  // --- Multi-column batched ---
  auto t2 = Clock::now();
  for (int rep = 0; rep < repeats; ++rep) {
    auto rhs = kkt->MakeSolverRHS(n_rhs);
    rhs.supernodes->ScatterFrom(rhs_data);
    rhs.blocks_fully_gathered = true;
    kkt->SolveSolverRHS(rhs);
  }
  auto t3 = Clock::now();
  double batch_us = us(t2, t3) / repeats;

  printf("  Solve   n_rhs=%2d: single=%8.0fus  batch=%8.0fus  speedup=%.2fx\n",
         n_rhs, single_us, batch_us, single_us / batch_us);
}

// Benchmark single-column loop vs multi-column batched MultiplyA.
void BenchmarkMultiplyA(KKTSolverBase* kkt, int n_rhs, int repeats) {
  int n = kkt->number_of_variables();

  Eigen::MatrixXd x_data = Eigen::MatrixXd::Random(n, n_rhs);

  // --- Single-column loop ---
  auto t0 = Clock::now();
  for (int rep = 0; rep < repeats; ++rep) {
    for (int j = 0; j < n_rhs; ++j) {
      auto x = kkt->MakeSolverRHS();
      x.supernodes->ScatterFrom(x_data.col(j));
      x.blocks_fully_gathered = true;
      auto row = kkt->MakeRowSpace();
      kkt->MultiplyA(x, row);
    }
  }
  auto t1 = Clock::now();
  double single_us = us(t0, t1) / repeats;

  // --- Multi-column batched ---
  auto t2 = Clock::now();
  for (int rep = 0; rep < repeats; ++rep) {
    auto x = kkt->MakeSolverRHS(n_rhs);
    x.supernodes->ScatterFrom(x_data);
    x.blocks_fully_gathered = true;
    auto row = kkt->MakeRowSpace(n_rhs);
    kkt->MultiplyA(x, row);
  }
  auto t3 = Clock::now();
  double batch_us = us(t2, t3) / repeats;

  printf("  A*x     n_rhs=%2d: single=%8.0fus  batch=%8.0fus  speedup=%.2fx\n",
         n_rhs, single_us, batch_us, single_us / batch_us);
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int T = 100;
  int n_rhs_max = 16;
  if (argc > 1) T = std::atoi(argv[1]);
  if (argc > 2) n_rhs_max = std::atoi(argv[2]);

  const int nx = 6, nu = 3;
  printf("Multi-RHS Benchmark: nx=%d, nu=%d, T=%d\n", nx, nu, T);
  printf("  vars=%d\n\n", T * (nx + nu) + nx);

  auto solver = conex::BuildLQR(nx, nu, T);
  auto* kkt = solver.solver();

  int repeats = 50;
  for (int n_rhs : {2, 4, 8, 16}) {
    if (n_rhs > n_rhs_max) break;
    conex::BenchmarkSolve(kkt, n_rhs, repeats);
    conex::BenchmarkMultiplyA(kkt, n_rhs, repeats);
    printf("\n");
  }
  return 0;
}
