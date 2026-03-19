#include "kkt_solver_factory.h"

#include "conex/clique_ordering.h"
#include "conex/conjugate_gradient_solvers.h"
#include "conex/kkt_simplicial_solver.h"
#include "conex/kkt_solver.h"
#include "conex/kkt_tree_solver.h"
#include "conex/tree_utils.h"

namespace conex {
namespace {
using std::vector;

void IncrementSubvector(std::vector<int>* y, const std::vector<int>& indices) {
  for (auto i : indices) {
    y->at(i)++;
  }
}
}  // namespace
std::unique_ptr<KKTSolverBase> MakeSupernodalSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  vector<vector<int>> cliques = c->variables();
  vector<vector<int>> dual_vars = c->equality_constraint_multipliers();

  CliqueTree clique_tree =
      MakePrimalDualCliqueTree(cliques, dual_vars, config.clique_tree_method);

  auto solver_temp = std::make_unique<SupernodalKKTSolver>(
      cliques, c->SizeOfKKTSystem(), clique_tree.post_order_position_to_clique,
      clique_tree.supernodes, clique_tree.separators);

  solver_temp->SetIterativeRefinementIterations(
      config.iterative_refinement_iterations);
  if (config.kkt_solver == CONEX_KKT_SOLVER_SUPERNODAL) {
    if (c->equality_constraints().data.size() > 0) {
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
ContributionType ClassifyCliqueContribution(
    const SupernodalAssemblerBase* assembler, int number_of_primal_variables) {
  for (auto v : assembler->variables()) {
    if (v >= number_of_primal_variables) {
      return ContributionType::kIndefinite;
    }
  }
  return ContributionType::kPositiveDefinite;
}

std::unique_ptr<SymmetricLinearSystemTreeSolver> MakeTreeSolver(
    ConstraintManager* c, const SolverConfiguration& config) {
  auto clique_assemblers_ptrs_ = c->clique_assemblers();
  vector<vector<int>> cliques;
  for (const auto& assembler : clique_assemblers_ptrs_) {
    auto c_cliques = assembler->get_cliques();
    cliques.insert(cliques.end(), c_cliques.begin(), c_cliques.end());
  }

  auto tree_solver_ =
      std::make_unique<::conex::SymmetricLinearSystemTreeSolver>();

  vector<int> dual_vars_flat;
  for (const auto& dv : c->equality_constraint_multipliers()) {
    dual_vars_flat.insert(dual_vars_flat.end(), dv.begin(), dv.end());
  }

  vector<vector<int>> maximal_cliques;
  CliqueTree clique_tree = MakeCliqueTreeMinDegreeFromRowSupports(
      cliques, &maximal_cliques, /*max_merge_supernode_size=*/5,
      SUPERNODE_REORDER_BFS_GREEDY, dual_vars_flat);

  // Decompose assemblers against maximal cliques.
  vector<SupernodalAssemblerBase*> decomposed;
  for (auto* assembler : clique_assemblers_ptrs_) {
    auto subs = assembler->Decompose(maximal_cliques);
    decomposed.insert(decomposed.end(), subs.begin(), subs.end());
  }

  int num_primal = c->GetNumberOfVariables();
  for (auto* assembler : decomposed) {
    auto adapter =
        std::make_unique<::conex::KKTAssemblerToSubsystemAdapter>(assembler);
    adapter->set_contribution_type(
        ClassifyCliqueContribution(assembler, num_primal));
    tree_solver_->push_back(std::move(adapter));
  }
  tree_solver_->Finalize(clique_tree);
  tree_solver_->SetFactorizationMode(true /*left looking*/);
  tree_solver_->EnableAutoUpdateAtAssemble(true);
  tree_solver_->SetNumThreads(config.num_threads);

  return tree_solver_;
}
namespace {
class PrimalVariables {
 public:
  PrimalVariables(ConstraintManager* kkt)
      : degree(kkt->GetNumberOfVariables(), 0), kkt_(kkt) {
    for (const auto& c : kkt->clique_assemblers()) {
      if (c->is_positive_definite()) {
        cliques_of_G.push_back(c->primal_variables());
        clique_assemblers_of_G.push_back(c);
        IncrementSubvector(&degree, c->primal_variables());
        CONEX_DEMAND(
            c->dual_variables().size() == 0,
            "Auxiliary variables only supported for equality constraints");
      }
    }
  }
  bool ValidateStrictConvexity() {
    for (const auto d : degree) {
      CONEX_DEMAND(
          d > 0,
          "Primal schur-complement matrix is not positive definite.  "
          "Please presolve variables using equality constraints or add "
          "inequalities/quadratic penalty terms.");
    }
    return true;
  }
  void MakeStrictlyConvex() {
    int i = 0;
    for (const auto d : degree) {
      if (d == 0) {
        kkt_->AddQuadraticCost(Eigen::MatrixXd::Identity(1, 1), {i});
      }
      i++;
    }
  }
  std::vector<std::vector<int>> cliques_of_G;
  std::vector<SupernodalAssemblerBase*> clique_assemblers_of_G;
  std::vector<int> degree;
  ConstraintManager* kkt_;
};
};  // namespace
std::unique_ptr<KKTSolverBase> MakeCGSolver(ConstraintManager* kkt,
                                            const SolverConfiguration& config) {
  SparseEqualityConstraints equality_constraints;
  for (const auto& eq : kkt->equality_constraints().assemblers) {
    const Eigen::MatrixXd& A = eq.constraint_matrix();
    for (int i = 0; i < A.rows(); i++) {
      std::vector<double> entries;
      for (int j = 0; j < A.cols(); j++) {
        entries.push_back(A(i, j));
      }
      CONEX_CHECK(A.cols() == static_cast<int>(eq.primal_variables().size()));
      equality_constraints.matrix_entries.push_back(entries);
      equality_constraints.columns.push_back(eq.primal_variables());
    }
  }
  PrimalVariables p(kkt);
  p.ValidateStrictConvexity();
  int number_of_equations =
      kkt->SizeOfKKTSystem() - kkt->GetNumberOfVariables();
  CONEX_DEMAND(number_of_equations ==
                   static_cast<int>(equality_constraints.columns.size()),
               "KKT system is malformed");

  auto solver = std::make_unique<SupernodalKKTSolver>(p.cliques_of_G);
  solver->Bind(p.clique_assemblers_of_G);
  return std::make_unique<ConstrainedLeastSquaresConjugateGradientSolver>(
      std::move(solver), equality_constraints.columns,
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
  throw std::runtime_error("Invalid KKT Solver.");
  return std::unique_ptr<KKTSolverBase>{};
}

}  // namespace conex
