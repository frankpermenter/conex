#include <cstdio>
#include <cmath>
#include <numeric>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "conex/common/conex.h"
#include "conex/common/eja_ops.h"
#include "conex/common/model.h"
#include "conex/common/solver.h"
#include "conex/algorithms/geodesic_ipm.h"
#include "conex/linear_solvers/kkt_tree_solver.h"
#include "conex/common/equality_constraint.h"
using namespace conex;

static SolverRHS BuildCostRHS(KKTSolverBase& kkt,
                               const Eigen::VectorXd& cost) {
  int nv = kkt.number_of_variables();
  Eigen::VectorXd cost_full = Eigen::VectorXd::Zero(nv);
  int nc = std::min((int)cost.size(), nv);
  cost_full.head(nc) = cost.head(nc);
  auto rhs = kkt.MakeSolverRHS();
  rhs = kkt.MakeBlockVariable(cost_full);
  return rhs;
}

// ========================================================================
// Test 1: Standard-form LP
//   min c'x  s.t.  Ax = b,  x >= 0
//   n=20, p=8.  Cost c > 0 ensures bounded optimum.
//   Cross-check against reduced (null-space) LP without equalities.
// ========================================================================
bool TestStandardFormLP() {
  printf("=== Test 1: Standard-form LP (n=20, p=8) ===\n");
  const int n = 20, p = 8;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(0.1, 2.0);
  std::normal_distribution<double> norm(0, 1);

  // Feasible point x_feas > 0.
  Eigen::VectorXd x_feas(n);
  for (int i = 0; i < n; ++i) x_feas(i) = unif(rng);

  // Random A.
  Eigen::MatrixXd A_dense(p, n);
  for (int i = 0; i < p; ++i)
    for (int j = 0; j < n; ++j)
      A_dense(i, j) = norm(rng);
  Eigen::VectorXd b = A_dense * x_feas;

  // Positive cost → bounded optimum (x=0 is lower bound on cost, Ax=b + x≥0 is bounded).
  Eigen::VectorXd cost(n);
  for (int i = 0; i < n; ++i) cost(i) = unif(rng);

  // Build Model.
  Model prob;
  Eigen::SparseMatrix<double> I_sp(n, n); I_sp.setIdentity();
  Eigen::VectorXd zeros = Eigen::VectorXd::Zero(n);
  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);
  prob.AddLinearConstraint(I_sp, zeros, vars);

  Eigen::SparseMatrix<double> A_sp = A_dense.sparseView();
  prob.AddEqualityConstraint(A_sp, b, vars);
  prob.SetLinearCost(cost);

  // Solve.
  auto solver = Solver::Build(prob);
  auto* kkt = solver.kkt();
  printf("  n_total = %d\n", kkt->number_of_variables());
  auto cost_rhs = BuildCostRHS(*kkt, cost);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W); kkt->AssembleAndFactor();

  CompiledModel cm(*kkt, cost_rhs);
  auto r = SolveGeodesicThetaContinuation(
      cm, W, 500, 1, 1e-8, true);

  Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(r.x.data(), r.x.size()).head(n);
  double obj = cost.dot(x);
  Eigen::VectorXd eq_res = A_dense * x - b;
  double eq_err = eq_res.norm();
  double min_x = x.minCoeff();

  printf("  iters=%d, mu=%.2e, obj=%.6f\n", r.iterations, r.mu, obj);
  printf("  ||Ax - b|| = %.2e, min(x) = %.6f\n", eq_err, min_x);

  bool pass = true;
  if (eq_err > 1e-4) { printf("  FAIL: equality residual\n"); pass = false; }
  if (min_x < -1e-4) { printf("  FAIL: negative x\n"); pass = false; }
  if (r.mu > 1e-6) { printf("  FAIL: didn't converge\n"); pass = false; }

  // Cross-check: reduced LP via null space.
  Eigen::FullPivLU<Eigen::MatrixXd> lu(A_dense);
  Eigen::MatrixXd N = lu.kernel();
  int nz = N.cols();
  printf("  null dim = %d\n", nz);

  if (nz == n - p) {
    Model reduced;
    Eigen::SparseMatrix<double> N_sp = N.sparseView();
    std::vector<int> zvars(nz);
    std::iota(zvars.begin(), zvars.end(), 0);
    reduced.AddLinearConstraint(N_sp, x_feas, zvars);
    Eigen::VectorXd rc = N.transpose() * cost;
    reduced.SetLinearCost(rc);

    auto s2 = Solver::Build(reduced);
    auto* k2 = s2.kkt();
    auto cr2 = k2->MakeSolverRHS();
    cr2 = k2->MakeBlockVariable(rc);
    RowSpace W2 = k2->MakeRowSpace();
    setOnes(W2); k2->SetScaling(W2); k2->AssembleAndFactor();
    CompiledModel cm2(*k2, cr2);
    auto r2 = SolveGeodesicThetaContinuation(cm2, W2, 500, 1, 1e-8, false);

    double obj2 = cost.dot(x_feas);
    if (r2.x.size() >= nz) obj2 += rc.dot(Eigen::Map<const Eigen::VectorXd>(r2.x.data(), r2.x.size()).head(nz));
    printf("  Reduced obj = %.6f, Equality obj = %.6f, |diff| = %.2e\n",
           obj2, obj, std::abs(obj - obj2));
    if (std::abs(obj - obj2) > 1e-2) {
      printf("  FAIL: objectives don't match\n");
      pass = false;
    }
  }
  return pass;
}

// ========================================================================
// Test 2: Maxcut SDP (Goemans-Williamson relaxation)
//   max (1/4) <L, X>  s.t.  X ≽ 0,  diag(X) = 1
//   Equivalently: min -(1/4) <L, X>.
//
//   Variables: upper triangle of X, with A_k encoding the symmetric
//   basis so that X = Σ x_k A_k.  Diagonal A_k = e_i e_i',
//   off-diag A_k = (e_i e_j' + e_j e_i') (full, not halved).
//   Then X(i,j) = x_{var(i,j)} directly.
// ========================================================================
bool TestMaxcutSDP() {
  printf("\n=== Test 2: Maxcut SDP (n=6) ===\n");
  const int n = 6;
  const int nvar = n * (n + 1) / 2;

  auto var_idx = [&](int i, int j) -> int {
    if (i > j) std::swap(i, j);
    return i * n - i * (i - 1) / 2 + (j - i);
  };

  // Random graph.
  std::mt19937 rng(123);
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  struct Edge { int i, j; double w; };
  std::vector<Edge> edges;
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
      if (unif(rng) < 0.5)
        edges.push_back({i, j, unif(rng) + 0.1});

  printf("  n=%d, edges=%d, vars=%d\n", n, (int)edges.size(), nvar);

  // PSD constraint: X = Σ x_k A_k ≽ 0.
  // A_k for diagonal: e_i e_i'.
  // A_k for off-diag: e_i e_j' + e_j e_i'  (so X(i,j) = x_k directly).
  std::vector<Eigen::SparseMatrix<double>> A_list;
  std::vector<int> psd_vars;
  for (int i = 0; i < n; ++i) {
    for (int j = i; j < n; ++j) {
      Eigen::SparseMatrix<double> Aij(n, n);
      if (i == j) {
        Aij.insert(i, i) = 1.0;
      } else {
        Aij.insert(i, j) = 1.0;
        Aij.insert(j, i) = 1.0;
      }
      Aij.makeCompressed();
      A_list.push_back(Aij);
      psd_vars.push_back(var_idx(i, j));
    }
  }
  Eigen::SparseMatrix<double> B(n, n);

  Model prob;
  prob.AddPSDConstraint(A_list, B, psd_vars, false);

  // Equality: X_ii = 1.
  for (int i = 0; i < n; ++i) {
    Eigen::SparseMatrix<double> C(1, 1);
    C.insert(0, 0) = 1.0; C.makeCompressed();
    Eigen::VectorXd d(1); d(0) = 1.0;
    std::vector<int> eq_var = {var_idx(i, i)};
    prob.AddEqualityConstraint(C, d, eq_var);
  }

  // Cost: min Σ w_ij x_{ij}  (≡ min <W, X>, maxcut = const - min).
  // Since L = D - W, <L, X> = Tr(D) - <W, X> = Σ w_ij - Σ w_ij X_ij.
  // Maxcut = max (1/4)<L, X> = (1/4)(Σ w_ij) - (1/4) min Σ w_ij X_ij.
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(nvar);
  double sum_w = 0;
  for (const auto& e : edges) {
    cost(var_idx(e.i, e.j)) = e.w;
    sum_w += e.w;
  }
  prob.SetLinearCost(cost);

  auto solver = Solver::Build(prob);
  auto* kkt = solver.kkt();
  printf("  n_total = %d\n", kkt->number_of_variables());
  auto cost_rhs = BuildCostRHS(*kkt, cost);
  RowSpace W = kkt->MakeRowSpace();
  setOnes(W);
  kkt->SetScaling(W); kkt->AssembleAndFactor();

  CompiledModel cm(*kkt, cost_rhs);
  auto r = SolveGeodesicThetaContinuation(
      cm, W, 500, 1, 1e-8, true);

  Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(r.x.data(), r.x.size()).head(nvar);
  double obj = cost.dot(x);
  double maxcut_bound = (sum_w - obj) / 4.0;

  // Reconstruct X.
  Eigen::MatrixXd X(n, n);
  for (int i = 0; i < n; ++i)
    for (int j = i; j < n; ++j) {
      X(i, j) = x(var_idx(i, j));
      X(j, i) = X(i, j);
    }

  double max_diag_err = 0;
  for (int i = 0; i < n; ++i)
    max_diag_err = std::max(max_diag_err, std::abs(X(i, i) - 1.0));

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(X);
  double min_eig = eig.eigenvalues().minCoeff();

  printf("  iters=%d, mu=%.2e\n", r.iterations, r.mu);
  printf("  <W, X> = %.6f, maxcut bound = %.6f\n", obj, maxcut_bound);
  printf("  max |X_ii - 1| = %.2e\n", max_diag_err);
  printf("  min eigenvalue(X) = %.6f\n", min_eig);
  printf("  eigenvalues:");
  for (int i = 0; i < n; ++i) printf(" %.4f", eig.eigenvalues()(i));
  printf("\n");

  printf("  X =\n");
  for (int i = 0; i < n; ++i) {
    printf("    ");
    for (int j = 0; j < n; ++j) printf(" %7.4f", X(i, j));
    printf("\n");
  }

  bool pass = true;
  if (max_diag_err > 1e-3) { printf("  FAIL: diag != 1\n"); pass = false; }
  if (min_eig < -1e-3) { printf("  FAIL: X not PSD\n"); pass = false; }
  if (r.mu > 1e-6) { printf("  FAIL: didn't converge\n"); pass = false; }
  // Maxcut bound should be positive.
  if (maxcut_bound < -1e-3) { printf("  FAIL: negative maxcut bound\n"); pass = false; }

  return pass;
}

int main() {
  bool pass = true;
  pass &= TestStandardFormLP();
  pass &= TestMaxcutSDP();
  printf("\n%s\n", pass ? "ALL PASSED" : "SOME TESTS FAILED");
  return pass ? 0 : 1;
}
