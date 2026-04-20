// LQR benchmark: measures build + AssembleAndFactor + Solve for LQR problems
// of increasing horizon length. Uses single sparse Q and C matrices
// (exercises SparseQuadraticTermAssembler::Decompose and
// SparseEqualityConstraintAssembler::Decompose).
//
// Usage: ./lqr_benchmark [T_max]
//   Default T_max = 200.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <vector>

#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
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

void RunLQR(int nx, int nu, int T) {
  srand(42);
  Eigen::MatrixXd Ad = 0.9 * Eigen::MatrixXd::Identity(nx, nx) +
                        0.1 * Eigen::MatrixXd::Random(nx, nx);
  Eigen::MatrixXd Bd = Eigen::MatrixXd::Random(nx, nu);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
  Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(nu, nu);
  Eigen::MatrixXd Qf = 10.0 * Q;
  Eigen::VectorXd x0 = Eigen::VectorXd::Ones(nx);

  int step = nx + nu;
  int n_primal = T * step + nx;  // T stages of (x,u) + terminal x

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

  // Build one big sparse Q (block-diagonal: [Q R; Q R; ...; Qf]).
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

  // Build one big sparse C (dynamics + initial condition).
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
  // Initial condition: x_0 = x0.
  for (int j = 0; j < nx; ++j)
    ct.emplace_back(eq_row + j, x_idx(0)[j], 1.0);
  int n_eq = eq_row + nx;
  Eigen::SparseMatrix<double> C_eq(n_eq, n_primal);
  C_eq.setFromTriplets(ct.begin(), ct.end());
  Eigen::VectorXd d_eq = Eigen::VectorXd::Zero(n_eq);
  d_eq.tail(nx) = x0;

  std::vector<int> all_vars(n_primal);
  std::iota(all_vars.begin(), all_vars.end(), 0);

  // Build problem from single sparse Q and C.
  auto t0 = Clock::now();
  Model problem;
  problem.AddQuadraticCost(Q_cost, all_vars);
  auto c_eq = problem.AddEqualityConstraint(C_eq, d_eq, all_vars);
  auto t1 = Clock::now();

  // Build solver (exercises Decompose on the big sparse matrices).
  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();
  auto t2 = Clock::now();

  // AssembleAndFactor.
  kkt->AssembleAndFactor();
  auto t3 = Clock::now();

  // Solve.
  const auto& duals = solver.dual_variables(c_eq);
  int n_total = kkt->number_of_variables();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n_total);
  for (int j = 0; j < n_eq; ++j) rhs(duals[j]) = d_eq(j);
  Eigen::VectorXd sol = kkt->Solve(rhs);
  auto t4 = Clock::now();

  // Check initial condition.
  auto xi0 = x_idx(0);
  Eigen::VectorXd x_0_sol(nx);
  for (int i = 0; i < nx; ++i) x_0_sol(i) = sol(xi0[i]);
  double ic_err = (x_0_sol - x0).norm();

  // Check dynamics.
  double max_dyn_err = 0;
  for (int t = 0; t < T; ++t) {
    Eigen::VectorXd xt_sol(nx), ut_sol(nu), xt1_sol(nx);
    auto xti = x_idx(t), uti = u_idx(t);
    auto xt1i = (t < T - 1) ? x_idx(t + 1) : xT_idx();
    for (int i = 0; i < nx; ++i) xt_sol(i) = sol(xti[i]);
    for (int i = 0; i < nu; ++i) ut_sol(i) = sol(uti[i]);
    for (int i = 0; i < nx; ++i) xt1_sol(i) = sol(xt1i[i]);
    double err = (xt1_sol - Ad * xt_sol - Bd * ut_sol).norm();
    max_dyn_err = std::max(max_dyn_err, err);
  }

  int n_cliques = solver.tree_solver()
      ? solver.tree_solver()->num_subsystems() : 1;

  printf("  T=%4d  vars=%5d  cliq=%4d | prob=%8.0fus  build=%8.0fus"
         "  asm+fac=%8.0fus  solve=%8.0fus | ic=%.1e dyn=%.1e\n",
         T, n_total, n_cliques,
         us(t0, t1), us(t1, t2), us(t2, t3), us(t3, t4),
         ic_err, max_dyn_err);
}

}  // namespace
}  // namespace conex

int main(int argc, char* argv[]) {
  int T_max = 200;
  if (argc > 1) T_max = std::atoi(argv[1]);

  const int nx = 4, nu = 2;
  printf("LQR Benchmark (sparse Q + C): nx=%d, nu=%d\n", nx, nu);
  printf("%-60s | %s\n",
         "  T      vars   cliq |     prob       build     asm+fac       solve",
         "  ic       dyn");
  printf("%s\n", std::string(100, '-').c_str());

  for (int T : {10, 20, 50, 100, 200, 500}) {
    if (T > T_max) break;
    conex::RunLQR(nx, nu, T);
  }
  return 0;
}
