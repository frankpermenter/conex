#include "kkt_solver_factory.h"

#include "conex/clique_ordering.h"
#include "conex/conjugate_gradient_solvers.h"
#include "conex/kkt_solver.h"
#include "conex/kkt_tree_solver.h"

namespace conex {

std::vector<bool> MakeValidLeafIndicator(
    const std::vector<std::vector<int>>& cliques, int dual_variable_start) {
  std::vector<bool> valid_leaf(cliques.size(), true);
  int i = 0;
  for (auto& c : cliques) {
    if (c.back() >= dual_variable_start) {
      valid_leaf.at(i) = false;
    }
    i++;
  }
  return valid_leaf;
}

using std::vector;
void IncrementSubvector(std::vector<int>* y, const std::vector<int>& indices) {
  for (auto i : indices) {
    y->at(i)++;
  }
}

std::unique_ptr<KKTSolverBase> MakeSupernodalSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  vector<vector<int>> cliques = c->variables();

  CliqueTree clique_tree = MakeCliqueTree(
      cliques, MakeValidLeafIndicator(cliques, c->GetNumberOfVariables()));

  auto solver_temp = std::make_unique<SupernodalKKTSolver>(
      cliques, c->SizeOfKKTSystem(), clique_tree.post_order_position_to_clique,
      clique_tree.supernodes, clique_tree.separators);

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

SubsystemType ClassifySupernodeSubmatrix(const std::vector<int>& vars,
                                         int number_of_primal_variables) {
  bool all_primal_variables = true;
  bool all_dual_variables = true;
  for (auto& v : vars) {
    if (v < number_of_primal_variables) {
      all_dual_variables = false;
    }
    if (v >= number_of_primal_variables) {
      all_primal_variables = false;
    }
  }
  if (all_primal_variables) {
    return SubsystemType::kPositiveDefinite;
  }
  if (all_dual_variables) {
    return SubsystemType::kNegativeDefinite;
  }
  return SubsystemType::kQuasiDefinite;
}

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  vector<vector<int>> cliques = c->variables();
  auto& clique_assemblers_ptrs_ = c->clique_assemblers();

  auto tree_solver_ =
      std::make_unique<::conex::SymmetricLinearSystemTreeSolver>();

  CliqueTree clique_tree = MakeCliqueTree(
      cliques, MakeValidLeafIndicator(cliques, c->GetNumberOfVariables()));
  int i = 0;
  for (auto& clique : clique_assemblers_ptrs_) {
    auto adapter =
        std::make_unique<::conex::KKTAssemblerToSubsystemAdapter>(clique);
    auto* subsystem = adapter->create_subsystem(ClassifySupernodeSubmatrix(
        clique_tree.supernodes.at(i), c->GetNumberOfVariables()));
    tree_solver_->AddSubsystem(subsystem);
    tree_solver_->push_back(std::move(adapter));
    ++i;
  }
  tree_solver_->Finalize(clique_tree);
  tree_solver_->SetFactorizationMode(true /*left looking*/);
  tree_solver_->EnableAutoUpdateAtAssemble(true);

  return tree_solver_;
}

vector<int> DiagonalOfBarrierHessian(const ConstraintManager* kkt) {
  std::vector<int> degree(kkt->GetNumberOfVariables(), 0);
  for (const auto& c : kkt->clique_assemblers()) {
    if (c->is_positive_definite()) {
      IncrementSubvector(&degree, c->variables());
    }
  }
  return degree;
}

vector<int> FindEqualityConstraintsContainingVariable(
    const std::list<SupernodalAssemblerEqualities>& equations, int variable) {
  vector<int> constraints;
  int constraint = 0;
  for (const auto& e : equations) {
    const auto& vars = e.variables();
    bool result = std::find(begin(vars), end(vars), variable) != vars.end();
    if (result) {
      constraints.push_back(constraint);
    }
    constraint++;
  }
  return constraints;
}

std::unique_ptr<KKTSolverBase> MakeCGSolver(ConstraintManager* kkt,
                                            const SolverConfiguration& config) {
  SparseEqualityConstraints equality_constraints;
  int constraint_number = 0;
  for (const auto& eq : kkt->equality_constraint_manager().data) {
    const Eigen::MatrixXd& A = eq.constraint_matrix();
    for (int i = 0; i < A.rows(); i++) {
      std::vector<double> entries;
      for (int j = 0; j < A.cols(); j++) {
        entries.push_back(A(i, j));
      }
      equality_constraints.matrix_entries.push_back(entries);

      equality_constraints.columns.push_back(
          kkt->equality_constraint_manager().variables.at(constraint_number));
    }
    constraint_number++;
  }
  std::vector<std::vector<int>> cliques_of_G;
  std::vector<SupernodalAssemblerBase*> clique_assemblers_of_G;
  for (const auto& c : kkt->clique_assemblers()) {
    if (c->is_positive_definite()) {
      cliques_of_G.push_back(c->variables());
      clique_assemblers_of_G.push_back(c);
      CONEX_DEMAND(
          c->number_of_auxiliary_variables() == 0,
          "Auxiliary variables only supported for equality constraints");
    }
  }

  for (const auto& d : DiagonalOfBarrierHessian(kkt)) {
    CONEX_DEMAND(d > 0,
                 "Primal schur-complement matrix is not positive definite.  "
                 "Please presolve variables using equality constraints or add "
                 "inequalities/quadratic penalty terms.");
  }
  int number_of_equations =
      kkt->SizeOfKKTSystem() - kkt->GetNumberOfVariables();
  CONEX_CHECK(number_of_equations ==
              static_cast<int>(equality_constraints.columns.size()));
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
    case CONEX_KKT_SOLVER_TREE:
      return MakeTreeSolver(kkt, config);
    case CONEX_KKT_SOLVER_SPARSE_QR:
      std::unique_ptr<SymmetricLinearSystemTreeSolver> ptr =
          MakeTreeSolver(kkt, config);
      return std::make_unique<EigenSparseCholesky>(std::move(ptr));
      break;
  }
  std::runtime_error("Invalid KKT Solver.");
  return std::unique_ptr<KKTSolverBase>{};
}

}  // namespace conex
