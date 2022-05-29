#include "kkt_solver_factory.h"

#include "conex/conjugate_gradient_solvers.h"
#include "conex/kkt_solver.h"

namespace conex {
using std::vector;

void IncrementSubvector(std::vector<int>* y, const std::vector<int>& indices) {
  for (auto i : indices) {
    y->at(i)++;
  }
}
std::unique_ptr<KKTSolverBase> MakeSupernodalSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  vector<vector<int>> cliques = c->variables();
  vector<vector<int>> dual_vars = c->equality_constraint_multipliers();

  auto solver_temp = std::make_unique<SupernodalKKTSolver>(cliques, dual_vars);

  solver_temp->SetIterativeRefinementIterations(
      config.iterative_refinement_iterations);
  if (config.kkt_solver == CONEX_KKT_SOLVER_SUPERNODAL) {
    if (c->equality_constraints().size() > 0) {
      solver_temp->SetSolverMode(CONEX_LDLT_FACTORIZATION);
    } else {
      solver_temp->SetSolverMode(CONEX_LLT_FACTORIZATION);
    }
  } else {
    solver_temp->SetSolverMode(CONEX_QR_FACTORIZATION);
  }
  solver_temp->Bind(c->clique_assemblers());
  return solver_temp;
}

std::unique_ptr<KKTSolverBase> MakeCGSolver(ConstraintManager* kkt,
                                            const SolverConfiguration& config) {
  SparseEqualityConstraints equality_constraints;
  for (const auto& eq : kkt->equality_constraints()) {
    const Eigen::MatrixXd& A = eq.constraint_matrix();
    for (int i = 0; i < A.rows(); i++) {
      std::vector<double> entries;
      for (int j = 0; j < A.cols(); j++) {
        entries.push_back(A(i, j));
      }
      equality_constraints.matrix_entries.push_back(entries);
      equality_constraints.columns.push_back(eq.variables());
    }
  }
  std::vector<std::vector<int>> cliques_of_G;
  std::vector<SupernodalAssemblerBase*> clique_assemblers_of_G;
  std::vector<int> degree(kkt->GetNumberOfVariables(), 0);
  for (const auto& c : kkt->clique_assemblers()) {
    if (c->is_positive_definite()) {
      cliques_of_G.push_back(c->variables());
      clique_assemblers_of_G.push_back(c);
      IncrementSubvector(&degree, c->variables());
      CONEX_DEMAND(
          c->number_of_auxiliary_variables() == 0,
          "Auxiliary variables only supported for equality constraints");
    }
  }
  for (const auto d : degree) {
    CONEX_DEMAND(d > 0,
                 "Primal schur-complement matrix is not positive definite.  "
                 "Please presolve variables using equality constraints or add "
                 "inequalities/quadratic penalty terms.");
  }
  int number_of_equations =
      kkt->SizeOfKKTSystem() - kkt->GetNumberOfVariables();
  CONEX_DEMAND(number_of_equations ==
                   static_cast<int>(equality_constraints.columns.size()),
               "KKT system is malformed");
  return std::make_unique<ConstrainedLeastSquaresConjugateGradientSolver>(
      cliques_of_G, clique_assemblers_of_G, equality_constraints.columns,
      equality_constraints.matrix_entries);
}

std::unique_ptr<KKTSolverBase> KKTSolverFactory::create_unique(
    ConstraintManager* kkt, const SolverConfiguration& config) {
  switch (config.kkt_solver) {
    case CONEX_KKT_SOLVER_SUPERNODAL:
    case CONEX_KKT_SOLVER_SUPERNODAL_QR:
      return MakeSupernodalSolver(kkt, config);
      break;
    case CONEX_KKT_SOLVER_CG:
      return MakeCGSolver(kkt, config);
      break;
  }
  throw std::runtime_error("Invalid KKT Solver.");
  return std::unique_ptr<KKTSolverBase>{};
}

}  // namespace conex
