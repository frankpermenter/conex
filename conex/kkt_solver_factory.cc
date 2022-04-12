#include "kkt_solver_factory.h"
#include "conex/conjugate_gradient_solvers.h"
#include "conex/kkt_solver.h"

namespace conex {

std::unique_ptr<KKTSolverBase> MakeSupernodalSolver(
    ConstraintManager* kkt, const SolverConfiguration& config) {
  auto solver_temp = std::make_unique<SupernodalKKTSolver>(kkt);
  solver_temp->SetIterativeRefinementIterations(
      config.iterative_refinement_iterations);
  solver_temp->SetSolverMode(config.kkt_solver);
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
  std::vector<std::vector<int> > cliques_of_G;
  std::vector<SupernodalAssemblerBase*> clique_assemblers_of_G;
  for (const auto& c : kkt->clique_assemblers()) {
    if (c->is_positive_definite()) {
      cliques_of_G.push_back(c->variables());
      clique_assemblers_of_G.push_back(c);
    }
  }
  return std::make_unique<ConstrainedLeastSquaresConjugateGradientSolver>(
      cliques_of_G, clique_assemblers_of_G, equality_constraints.columns,
      equality_constraints.matrix_entries);
}

std::unique_ptr<KKTSolverBase> KKTSolverFactory::create_unique(
    ConstraintManager* kkt, const SolverConfiguration& config) {
  switch (config.kkt_solver) {
    case CONEX_KKT_SOLVER_LLT:
    case CONEX_KKT_SOLVER_LDLT:
    case CONEX_KKT_SOLVER_QR:
      return MakeSupernodalSolver(kkt, config);
      break;
    case CONEX_KKT_SOLVER_CG:
      return MakeCGSolver(kkt, config);
      break;
  }
  std::runtime_error("Invalid KKT Solver.");
  return std::unique_ptr<KKTSolverBase>{};
}

}  // namespace conex
