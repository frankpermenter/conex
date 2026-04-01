#include "conex/algorithms/irls.h"

#include <chrono>
#include <cmath>
#include <numeric>

#include "conex/common/problem.h"
#include "conex/common/solver.h"

namespace conex {

IRLSResult SolveIRLS(
    const Eigen::SparseMatrix<double>& A,
    const Eigen::VectorXd& b,
    int max_iterations,
    double epsilon,
    double tolerance) {
  using clock = std::chrono::high_resolution_clock;
  IRLSResult result;
  const int m = A.rows();
  const int n = A.cols();

  std::vector<int> vars(n);
  std::iota(vars.begin(), vars.end(), 0);

  Problem problem;
  problem.AddLinearConstraint(A, b, vars);

  auto [reduced, expansion] = Preprocess(problem);

  auto solver = Solver::Build(reduced);
  auto* kkt = solver.solver();
  kkt->AssembleAndFactor();

  auto x = kkt->MakeTreeRHS();
  auto rhs = kkt->MakeTreeRHS();
  auto row = kkt->MakeRowSpace();

  // Get b in internal (sub-constraint) ordering.
  RowSpace b_internal = kkt->GetAffineTerm();

  auto t0 = clock::now();

  RowSpace w = kkt->MakeRowSpace();
  for (int i = 0; i < w.total_rows(); ++i) w.data(i) = 1.0;
  double prev_obj = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iterations; ++iter) {
    kkt->SetWeights(w);
    if (!kkt->AssembleAndFactor()) break;

    // RHS = A^T W b.
    RowSpace wb = kkt->MakeRowSpace();
    wb.data = w.data.asDiagonal() * b_internal.data;
    rhs.SetZero();
    kkt->AccumulateAtranspose(wb, rhs);

    // Solve.
    kkt->SolveTreeRHS(rhs);
    x = rhs;

    // Residual: r = Ax - b.
    kkt->MultiplyA(x, row);
    Eigen::VectorXd r = row.data - b_internal.data;
    double obj = r.lpNorm<1>();

    if (std::abs(prev_obj - obj) < tolerance * std::abs(obj) + 1e-15) {
      result.iterations = iter + 1;
      break;
    }
    prev_obj = obj;
    result.iterations = iter + 1;

    for (int i = 0; i < m; ++i)
      w.data(i) = 1.0 / std::max(std::abs(r(i)), epsilon);
  }

  auto t1 = clock::now();
  Eigen::VectorXd x_final(reduced.num_variables());
  x.supernodes->GatherInto(x_final);
  result.x = expansion.Expand(x_final);
  kkt->MultiplyA(x, row);
  result.l1_objective = (row.data - b_internal.data).lpNorm<1>();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
