#include "conex/algorithms/finite_horizon.h"

#include <chrono>
#include <numeric>
#include <set>

#include "conex/common/conex.h"
#include "conex/common/constraint_manager.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/tree_solver/kkt_solver_factory.h"

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
  const int n_eq = (T + 1) * nx;

  auto t0 = clock::now();

  // Build sparse block-diagonal cost matrix.
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

  // Build sparse equality constraint matrix.
  // Rows 0..(T*nx-1): dynamics  [-A, -B, I] on [x_t, u_t, x_{t+1}]
  // Rows T*nx..n_eq-1: initial condition  [I] on x_0
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

  // Cost assembler.
  std::set<int> q_var_set;
  for (int k = 0; k < Q_cost.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(Q_cost, k); it; ++it) {
      q_var_set.insert(it.row());
      q_var_set.insert(it.col());
    }
  std::vector<int> q_vars(q_var_set.begin(), q_var_set.end());

  ConstraintManager cm(n_vars);
  auto q_asm = std::make_unique<SparseQuadraticTermAssembler>(Q_cost, q_vars);
  cm.AddCustomAssembler(std::move(q_asm));

  // Equality constraint assembler.
  auto sec = std::make_unique<SparseEqualityConstraint>(C_eq, d_eq);
  std::set<int> eq_var_set;
  for (const auto& s : sec->row_supports())
    eq_var_set.insert(s.begin(), s.end());
  std::vector<int> eq_primal(eq_var_set.begin(), eq_var_set.end());
  std::vector<int> dual_vars = cm.AllocateDualVariables(n_eq);
  auto eq_asm = std::make_unique<SparseEqualityConstraintAssembler>(
      std::move(sec), eq_primal, dual_vars);
  cm.AddCustomAssembler(std::move(eq_asm));

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&cm, config);

  auto t1 = clock::now();

  bool ok = solver->AssembleAndFactor();
  CONEX_DEMAND(ok, "AssembleAndFactor failed.");

  auto t2 = clock::now();

  // RHS = [0 (primal cost has no linear term); d (dual)].
  int sys_size = cm.SizeOfKKTSystem();
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(sys_size);
  for (int i = 0; i < n_eq; ++i)
    rhs(dual_vars[i]) = d_eq(i);

  Eigen::VectorXd sol = solver->Solve(rhs);

  auto t3 = clock::now();

  // Extract trajectory from stacked solution vector.
  Eigen::VectorXd z = sol.head(n_vars);
  result.x.resize(nx, T + 1);
  result.u.resize(nu, T);
  for (int t = 0; t <= T; ++t)
    result.x.col(t) = z.segment(XIndex(t, nx, nu), nx);
  for (int t = 0; t < T; ++t)
    result.u.col(t) = z.segment(UIndex(t, nx, nu), nu);

  result.construction_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  result.factor_time_us =
      std::chrono::duration<double, std::micro>(t2 - t1).count();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t3 - t2).count();

  return result;
}

}  // namespace conex
