#include "conex/algorithms/irls.h"

#include <chrono>
#include <cmath>
#include <set>

#include "conex/common/constraint_manager.h"
#include "conex/common/conex.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"

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

  // Build solver once.
  Eigen::VectorXd b_zero = Eigen::VectorXd::Zero(m);
  auto slc = std::make_unique<SparseLinearConstraint>(A, b_zero);
  std::set<int> var_set;
  for (const auto& sup : slc->row_supports())
    var_set.insert(sup.begin(), sup.end());
  std::vector<int> all_vars(var_set.begin(), var_set.end());

  ConstraintManager cm(n);
  auto assembler = std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars);
  auto* asm_ptr = assembler.get();
  cm.AddCustomAssembler(asm_ptr);

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&cm, config);

  auto t0 = clock::now();

  // Initial solve: uniform weights (standard least squares).
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m);
  Eigen::MatrixXd Ad(A);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  double prev_obj = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Set weights and solve A^T W A x = A^T W b.
    asm_ptr->SetWeights(weights);
    bool ok = solver->AssembleAndFactor();
    if (!ok) break;

    Eigen::VectorXd rhs = Ad.transpose() * (weights.asDiagonal() * b);
    x = solver->Solve(rhs);

    // Compute residual and update weights.
    Eigen::VectorXd r = Ad * x - b;
    double obj = r.lpNorm<1>();

    // Check convergence.
    if (std::abs(prev_obj - obj) < tolerance * std::abs(obj) + 1e-15) {
      result.iterations = iter + 1;
      break;
    }
    prev_obj = obj;
    result.iterations = iter + 1;

    // IRLS weight update: w_i = 1 / max(|r_i|, epsilon).
    for (int i = 0; i < m; ++i) {
      weights(i) = 1.0 / std::max(std::abs(r(i)), epsilon);
    }
  }

  auto t1 = clock::now();
  result.x = x;
  result.l1_objective = (Ad * x - b).lpNorm<1>();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
