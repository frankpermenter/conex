#include "conex/kkt_solver_factory.h"

#include "conex/clique_ordering.h"
#include "conex/kkt_tree_solver.h"
#include "conex/tree_utils.h"

namespace conex {
using std::vector;

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
      cliques, &maximal_cliques, config.tree.max_merge_supernode_size,
      config.tree.supernode_reorder_method, dual_vars_flat);

  // Decompose assemblers against maximal cliques.
  vector<SupernodalAssemblerBase*> decomposed;
  for (auto* assembler : clique_assemblers_ptrs_) {
    auto subs = assembler->Decompose(maximal_cliques);
    decomposed.insert(decomposed.end(), subs.begin(), subs.end());
    if (config.tree.precompute_gram) {
      assembler->set_precompute_gram(true);
    }
  }

  int num_primal = c->GetNumberOfVariables();
  for (auto* assembler : decomposed) {
    auto adapter =
        std::make_unique<::conex::KKTAssemblerToSubsystemAdapter>(assembler);
    adapter->set_contribution_type(
        ClassifyCliqueContribution(assembler, num_primal));
    tree_solver_->push_back(std::move(adapter));
  }
  tree_solver_->Finalize(clique_tree, config.rhs_cols);
  tree_solver_->SetFactorizationMode(config.tree.left_looking);
  tree_solver_->EnableAutoUpdateAtAssemble(true);
  tree_solver_->SetNumThreads(config.num_threads);

  return tree_solver_;
}

}  // namespace conex
