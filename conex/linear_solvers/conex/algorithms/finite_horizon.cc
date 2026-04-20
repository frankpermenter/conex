#include "conex/algorithms/finite_horizon.h"

#include <chrono>
#include <numeric>

#include "conex/common/kkt_solver_interface.h"

#include "conex/common/model.h"
#include "conex/common/solver.h"

namespace conex {

namespace {
int XIndex(int t, int nx, int nu) { return t * (nx + nu); }
int UIndex(int t, int nx, int nu) { return t * (nx + nu) + nx; }
}  // namespace

LQRFromSparseMatricesResult SolveLQRFromSparseMatrices(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& Qf,
    const Eigen::VectorXd& x0,
    int T) {
  using clock = std::chrono::high_resolution_clock;
  LQRFromSparseMatricesResult result;

  const int nx = A.rows();
  const int nu = B.cols();
  const int n_vars = (T + 1) * nx + T * nu;

  auto t0 = clock::now();

  // Build sparse cost matrix Q_cost.
  std::vector<Eigen::Triplet<double>> qt;
  for (int t = 0; t < T; ++t) {
    int xi = XIndex(t, nx, nu);
    int ui = UIndex(t, nx, nu);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < nx; ++j)
        if (Q(i, j) != 0) qt.emplace_back(xi + i, xi + j, Q(i, j));
    for (int i = 0; i < nu; ++i)
      for (int j = 0; j < nu; ++j)
        if (R(i, j) != 0) qt.emplace_back(ui + i, ui + j, R(i, j));
  }
  int xT = XIndex(T, nx, nu);
  for (int i = 0; i < nx; ++i)
    for (int j = 0; j < nx; ++j)
      if (Qf(i, j) != 0) qt.emplace_back(xT + i, xT + j, Qf(i, j));
  Eigen::SparseMatrix<double> Q_cost(n_vars, n_vars);
  Q_cost.setFromTriplets(qt.begin(), qt.end());

  // Build sparse equality constraint: dynamics + initial condition.
  const int n_eq = (T + 1) * nx;
  std::vector<Eigen::Triplet<double>> ct;
  for (int t = 0; t < T; ++t) {
    int rb = t * nx;
    int xi = XIndex(t, nx, nu);
    int ui = UIndex(t, nx, nu);
    int xi1 = XIndex(t + 1, nx, nu);
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nx; ++c)
        if (A(r, c) != 0) ct.emplace_back(rb + r, xi + c, -A(r, c));
    for (int r = 0; r < nx; ++r)
      for (int c = 0; c < nu; ++c)
        if (B(r, c) != 0) ct.emplace_back(rb + r, ui + c, -B(r, c));
    for (int i = 0; i < nx; ++i)
      ct.emplace_back(rb + i, xi1 + i, 1.0);
  }
  int ic_row = T * nx;
  for (int i = 0; i < nx; ++i)
    ct.emplace_back(ic_row + i, i, 1.0);
  Eigen::SparseMatrix<double> C_eq(n_eq, n_vars);
  C_eq.setFromTriplets(ct.begin(), ct.end());

  Eigen::VectorXd d_eq = Eigen::VectorXd::Zero(n_eq);
  d_eq.tail(nx) = x0;

  // Build Model.
  std::vector<int> vars(n_vars);
  std::iota(vars.begin(), vars.end(), 0);

  Model problem;
  problem.AddQuadraticCost(Q_cost, vars);
  auto c_eq = problem.AddEqualityConstraint(C_eq, d_eq, vars);

  auto solver = Solver::Build(problem);
  auto* kkt = solver.kkt();

  auto t1 = clock::now();

  bool ok = kkt->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // RHS: dual vars get d_eq.
  const auto& dual_vars = solver.dual_variables(c_eq);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(kkt->number_of_variables());
  for (int i = 0; i < n_eq; ++i)
    rhs(dual_vars[i]) = d_eq(i);

  Eigen::VectorXd sol = kkt->Solve(rhs);

  auto t3 = clock::now();

  // Extract trajectory.
  result.x.resize(nx, T + 1);
  result.u.resize(nu, T);
  for (int t = 0; t <= T; ++t)
    result.x.col(t) = sol.segment(XIndex(t, nx, nu), nx);
  for (int t = 0; t < T; ++t)
    result.u.col(t) = sol.segment(UIndex(t, nx, nu), nu);

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

}  // namespace conex
