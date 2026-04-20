#include "conex/common/solver.h"

#include "conex/algorithms/tree_solver_builder.h"
#include "conex/common/psd_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_psd_constraint.h"
#include "conex/common/sparse_soc_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/structural_rank.h"
#include "conex/tree_solver/assembler_adapter.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

Solver::Solver() = default;
Solver::~Solver() = default;
Solver::Solver(Solver&&) noexcept = default;
Solver& Solver::operator=(Solver&&) noexcept = default;

Solver Solver::Build(const Problem& problem,
                     const SolverConfiguration& config) {
  auto [reduced, expansion] = RemoveStructuralRankDeficiency(problem);
  Solver s;
  s.expansion_ = std::move(expansion);
  s.reduced_linear_cost_ = reduced.linear_cost();
  if (config.use_quotient_amd) {
    s.BuildQuotientAMD(reduced, config);
  } else {
    s.BuildInternal(reduced, config);
  }
  return s;
}

Solver Solver::Build(const Problem& problem,
                     const TreeSpec& tree,
                     const SolverConfiguration& config) {
  // TreeSpec references original constraint IDs — skip preprocessing.
  Solver s;
  s.expansion_.original_n = problem.num_variables();
  s.expansion_.col_map.resize(problem.num_variables());
  std::iota(s.expansion_.col_map.begin(), s.expansion_.col_map.end(), 0);
  s.reduced_linear_cost_ = problem.linear_cost();
  s.BuildFromTree(problem, tree, config);
  return s;
}

Solver Solver::BuildDense(const Problem& problem) {
  // Dense path uses TreeSpec internally — skip preprocessing.
  TreeSpec tree;
  int clique = tree.AddClique();
  for (int i = 0; i < problem.num_constraints(); ++i)
    tree.Assign(i, clique);
  return Build(problem, tree);
}

KKTSolverBase* Solver::solver() {
  return static_cast<KKTSolverBase*>(tree_solver_.get());
}

const KKTSolverBase* Solver::solver() const {
  return static_cast<const KKTSolverBase*>(tree_solver_.get());
}

SymmetricLinearSystemTreeSolver* Solver::tree_solver() {
  return tree_solver_.get();
}

const SymmetricLinearSystemTreeSolver* Solver::tree_solver() const {
  return tree_solver_.get();
}

const std::vector<int>& Solver::dual_variables(ConstraintId id) const {
  return dual_var_map_.at(id);
}

const Eigen::VectorXd& Solver::linear_cost() const {
  return reduced_linear_cost_;
}

void Solver::BuildInternal(const Problem& problem,
                           const SolverConfiguration& config,
                           const CliqueTree* tree_override) {
  const int n = problem.num_variables();
  cm_ = std::make_unique<ConstraintManager>(n);

  linear_assemblers_.resize(problem.num_constraints(), nullptr);
  psd_assemblers_.resize(problem.num_constraints(), nullptr);
  soc_assemblers_.resize(problem.num_constraints(), nullptr);
  quadratic_assemblers_.resize(problem.num_constraints(), nullptr);
  equality_assemblers_.resize(problem.num_constraints(), nullptr);

  for (int i = 0; i < problem.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
        auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
        auto asm_ptr = std::make_unique<SparseLinearConstraintAssembler>(
            std::move(slc), data.vars);
        linear_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
        auto asm_ptr = std::make_unique<SparsePSDConstraintAssembler>(
            data.A_list, data.B, data.vars, data.use_chordal);
        psd_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Problem::SOCConstraintData>) {
        auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
        auto asm_ptr = std::make_unique<SparseSOCConstraintAssembler>(
            std::move(slc), data.vars);
        soc_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
        auto asm_ptr = std::make_unique<SparseQuadraticTermAssembler>(
            data.Q_sparse, data.vars);
        quadratic_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T,
                                          Problem::EqualityConstraintData>) {
        auto sec = std::make_unique<SparseEqualityConstraint>(
            data.C, data.d);
        auto dual = cm_->AllocateDualVariables(data.C.rows());
        dual_var_map_[i] = dual;
        auto asm_ptr = std::make_unique<SparseEqualityConstraintAssembler>(
            std::move(sec), data.primal_vars, dual);
        equality_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));
      }
    }, problem.constraint(i));
  }

  if (tree_override) {
    // Use the provided clique tree instead of computing one.
    auto assemblers = cm_->clique_assemblers();
    int num_primal = problem.num_variables();
    auto ts = std::make_unique<SymmetricLinearSystemTreeSolver>();
    std::vector<SupernodalAssemblerBase*> decomposed;
    std::vector<std::vector<int>> maximal_cliques(tree_override->supernodes.size());
    for (int i = 0; i < (int)tree_override->supernodes.size(); ++i) {
      maximal_cliques[i] = tree_override->supernodes[i];
      maximal_cliques[i].insert(maximal_cliques[i].end(),
                                tree_override->separators[i].begin(),
                                tree_override->separators[i].end());
      std::sort(maximal_cliques[i].begin(), maximal_cliques[i].end());
    }
    for (auto* assembler : assemblers) {
      auto subs = assembler->Decompose(maximal_cliques);
      for (auto* sub : subs) {
        auto adapter = std::make_unique<AssemblerAdapter>(sub);
        bool is_pd = true;
        for (int v : sub->variables())
          if (v >= num_primal) { is_pd = false; break; }
        adapter->set_contribution_type(
            is_pd ? ContributionType::kPositiveDefinite
                  : ContributionType::kIndefinite);
        ts->push_back(std::move(adapter));
      }
    }
    ts->SetUseGenericFactorization(config.tree.use_generic_factorization);
    ts->SetUseLUForIndefinite(config.tree.use_lu_for_indefinite);
    ts->FinalizeStructure(*tree_override, config.rhs_cols);
    ts->SetFactorizationMode(config.tree.left_looking);
    ts->EnableAutoUpdateAtAssemble(true);
    ts->SetNumThreads(config.num_threads);
    tree_solver_ = std::move(ts);
  } else {
    tree_solver_ = MakeTreeSolver(cm_.get(), config);
  }
  RegisterAssemblersWithTreeSolver();
}

void Solver::BuildFromTree(const Problem& problem,
                           const TreeSpec& tree,
                           const SolverConfiguration& config) {
  // Use the TreeSolverBuilder to convert TreeSpec → CliqueTree,
  // then forward to BuildInternal which creates proper assemblers.
  auto builder = std::make_unique<TreeSolverBuilder>();

  std::vector<int> cids(tree.num_cliques());
  for (int k = 0; k < tree.num_cliques(); ++k) {
    if (tree.parent(k) < 0)
      cids[k] = builder->AddClique();
    else
      cids[k] = builder->AddClique(cids[tree.parent(k)]);
  }

  int next_dual = problem.num_variables();
  for (int i = 0; i < problem.num_constraints(); ++i) {
    int clique = tree.clique_of(i);
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Problem::LinearConstraintData> ||
                     std::is_same_v<T, Problem::SOCConstraintData>) {
        Eigen::MatrixXd Ad(data.A);
        builder->AddLinearConstraint(cids[clique], Ad, data.b, data.vars);
      } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
        int n = data.B.rows();
        auto psd = std::make_unique<PSDConstraint>(n, data.A_list, data.B);
        builder->AddPSDConstraint(cids[clique], std::move(psd), data.vars);
      } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
        int nv = static_cast<int>(data.vars.size());
        Eigen::MatrixXd Qd(nv, nv);
        if (data.Q_dense.size() > 0) {
          Qd = data.Q_dense;
        } else {
          Eigen::MatrixXd Qfull(data.Q_sparse);
          for (int r = 0; r < nv; ++r)
            for (int c = 0; c < nv; ++c)
              Qd(r, c) = Qfull(data.vars[r], data.vars[c]);
        }
        builder->AddCost(cids[clique], Qd, data.vars);
      } else if constexpr (std::is_same_v<T,
                                          Problem::EqualityConstraintData>) {
        int p = data.C.rows();
        std::vector<int> dual(p);
        for (int j = 0; j < p; ++j) dual[j] = next_dual++;
        Eigen::MatrixXd Cd(data.C);
        builder->AddEquality(cids[clique], Cd, data.d,
                              data.primal_vars, dual);
      }
    }, problem.constraint(i));
  }

  // Build to get the CliqueTree, then discard the solver.
  auto result = builder->Build();
  // Forward to BuildInternal with the computed tree.
  BuildInternal(problem, config, &result.clique_tree);
}

void Solver::BuildQuotientAMD(const Problem& problem,
                              const SolverConfiguration& config) {
  // One clique per constraint, let BuildFromTree handle the rest.
  TreeSpec tree;
  for (int i = 0; i < problem.num_constraints(); ++i) {
    tree.AddClique();
    tree.Assign(i, i);
  }
  BuildFromTree(problem, tree, config);
}

void Solver::RegisterAssemblersWithTreeSolver() {
  if (!tree_solver_) return;
  for (auto* slca : linear_assemblers_) {
    if (!slca) continue;
    for (const auto& lc : slca->constraints())
      tree_solver_->RegisterLinearSubAssembler(lc.get());
  }
  for (auto* pasm : psd_assemblers_) {
    if (!pasm) continue;
    for (const auto& lc : pasm->constraints())
      tree_solver_->RegisterLinearSubAssembler(lc.get());
  }
  for (auto* sasm : soc_assemblers_) {
    if (!sasm) continue;
    for (const auto& lc : sasm->constraints())
      tree_solver_->RegisterLinearSubAssembler(lc.get());
  }
  for (auto* qasm : quadratic_assemblers_) {
    if (!qasm) continue;
    for (auto& qc : qasm->constraints())
      tree_solver_->RegisterQuadraticSubAssembler(&qc);
  }
  for (auto* easm : equality_assemblers_) {
    if (!easm) continue;
    for (auto& ec : easm->constraints())
      tree_solver_->RegisterEqualitySubAssembler(&ec);
  }
}

}  // namespace conex
