// TODO: Migrate to Problem + Solver API (conex/common/problem.h, solver.h).
// Currently uses ConstraintManager + MakeTreeSolver directly.
#include "conex/algorithms/irls.h"

#include <chrono>
#include <cmath>
#include <set>

#include "conex/common/constraint_manager.h"
#include "conex/common/conex.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"

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
  cm.AddCustomAssembler(std::make_unique<SparseLinearConstraintAssembler>(
      std::move(slc), all_vars));
  cm.Preprocess();

  auto* asm_ptr = dynamic_cast<SparseLinearConstraintAssembler*>(
      cm.clique_assemblers().back());

  SolverConfiguration config;
  auto solver = MakeTreeSolver(&cm, config);

  solver->AssembleAndFactor();
  if (auto* tree = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.get()))
    asm_ptr->BindPartition(*tree);

  auto t0 = clock::now();

  const int n_solve = cm.GetNumberOfVariables();
  Eigen::VectorXd weights = Eigen::VectorXd::Ones(m);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n_solve);

  double prev_obj = std::numeric_limits<double>::max();

  for (int iter = 0; iter < max_iterations; ++iter) {
    // Set weights and solve A^T W A x = A^T W b.
    asm_ptr->SetWeights(weights);
    bool ok = solver->AssembleAndFactor();
    if (!ok) break;

    // RHS = A^T W b.  Use per-clique A^T product.
    Eigen::VectorXd wb = weights.asDiagonal() * b;
    Eigen::VectorXd rhs = asm_ptr->ComputeTransposeProduct(wb);
    x = solver->Solve(rhs);

    // Compute residual using block partition (no gather for A*x).
    solver->ScatterToBlocks(x);
    Eigen::VectorXd r = asm_ptr->ComputeBlockResiduals(*solver) - b;
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
  result.x = cm.ExpandSolution(x);
  solver->ScatterToBlocks(x);
  result.l1_objective =
      (asm_ptr->ComputeBlockResiduals(*solver) - b).lpNorm<1>();
  result.solve_time_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count();
  return result;
}

}  // namespace conex
