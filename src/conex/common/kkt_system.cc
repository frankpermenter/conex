#include "conex/common/kkt_system.h"

#include "conex/algorithms/tree_solver_builder.h"
#include "conex/common/psd_constraint.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_psd_constraint.h"
#include "conex/common/sparse_soc_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/barrier_linear_constraint.h"
#include "conex/tree_solver/assembler_adapter.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

KKTSystem::KKTSystem() = default;
KKTSystem::~KKTSystem() = default;
KKTSystem::KKTSystem(KKTSystem&&) noexcept = default;
KKTSystem& KKTSystem::operator=(KKTSystem&&) noexcept = default;

KKTSystem KKTSystem::Build(const Model& model,
                           const SolverConfiguration& config) {
  KKTSystem s;
  if (config.use_quotient_amd) {
    s.BuildQuotientAMD(model, config);
  } else {
    s.BuildInternal(model, config);
  }
  return s;
}

KKTSystem KKTSystem::Build(const Model& model,
                           const TreeSpec& tree,
                           const SolverConfiguration& config) {
  KKTSystem s;
  s.BuildFromTree(model, tree, config);
  return s;
}

KKTSystem KKTSystem::BuildDense(const Model& model) {
  TreeSpec tree;
  int clique = tree.AddClique();
  for (int i = 0; i < model.num_constraints(); ++i)
    tree.Assign(i, clique);
  return Build(model, tree);
}

KKTSystem KKTSystem::Build(const Model& model,
                           const CliqueTree& tree,
                           const SolverConfiguration& config) {
  KKTSystem s;
  s.BuildInternal(model, config, &tree);
  return s;
}

KKTSolverBase* KKTSystem::kkt() {
  return static_cast<KKTSolverBase*>(tree_solver_.get());
}

const KKTSolverBase* KKTSystem::kkt() const {
  return static_cast<const KKTSolverBase*>(tree_solver_.get());
}

SymmetricLinearSystemTreeSolver* KKTSystem::tree_solver() {
  return tree_solver_.get();
}

const SymmetricLinearSystemTreeSolver* KKTSystem::tree_solver() const {
  return tree_solver_.get();
}

const std::vector<int>& KKTSystem::dual_variables(ConstraintId id) const {
  return dual_var_map_.at(id);
}

void KKTSystem::BuildInternal(const Model& model,
                              const SolverConfiguration& config,
                              const CliqueTree* tree_override) {
  const int n = model.num_variables();
  cm_ = std::make_unique<ConstraintManager>(n);

  linear_assemblers_.resize(model.num_constraints(), nullptr);
  psd_assemblers_.resize(model.num_constraints(), nullptr);
  soc_assemblers_.resize(model.num_constraints(), nullptr);
  quadratic_assemblers_.resize(model.num_constraints(), nullptr);
  equality_assemblers_.resize(model.num_constraints(), nullptr);

  for (int i = 0; i < model.num_constraints(); ++i) {
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;

      if constexpr (std::is_same_v<T, Model::LinearConstraintData>) {
        auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
        auto asm_ptr = std::make_unique<SparseLinearConstraintAssembler>(
            std::move(slc), data.vars);
        linear_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        auto asm_ptr = std::make_unique<SparsePSDConstraintAssembler>(
            data.A_list, data.B, data.vars, data.use_chordal);
        psd_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Model::SOCConstraintData>) {
        auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
        auto asm_ptr = std::make_unique<SparseSOCConstraintAssembler>(
            std::move(slc), data.vars);
        soc_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
        auto asm_ptr = std::make_unique<SparseQuadraticTermAssembler>(
            data.Q_sparse, data.vars);
        quadratic_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T,
                                          Model::EqualityConstraintData>) {
        auto sec = std::make_unique<SparseEqualityConstraint>(
            data.C, data.d);
        auto dual = cm_->AllocateDualVariables(data.C.rows());
        dual_var_map_[i] = dual;
        auto asm_ptr = std::make_unique<SparseEqualityConstraintAssembler>(
            std::move(sec), data.primal_vars, dual);
        equality_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));

      } else if constexpr (std::is_same_v<T,
                                          Model::BarrierConstraintData>) {
        auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
        auto asm_ptr = std::make_unique<SparseBarrierConstraintAssembler>(
            std::move(slc), data.vars, data.ops);
        linear_assemblers_[i] = asm_ptr.get();
        cm_->AddCustomAssembler(std::move(asm_ptr));
      }
    }, model.constraint(i));
  }

  if (tree_override) {
    auto assemblers = cm_->clique_assemblers();
    int num_primal = model.num_variables();
    auto ts = std::make_unique<SymmetricLinearSystemTreeSolver>();
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

void KKTSystem::BuildFromTree(const Model& model,
                              const TreeSpec& tree,
                              const SolverConfiguration& config) {
  auto builder = std::make_unique<TreeSolverBuilder>();

  std::vector<int> cids(tree.num_cliques());
  for (int k = 0; k < tree.num_cliques(); ++k) {
    if (tree.parent(k) < 0)
      cids[k] = builder->AddClique();
    else
      cids[k] = builder->AddClique(cids[tree.parent(k)]);
  }

  int next_dual = model.num_variables();
  for (int i = 0; i < model.num_constraints(); ++i) {
    int clique = tree.clique_of(i);
    std::visit([&](const auto& data) {
      using T = std::decay_t<decltype(data)>;
      if constexpr (std::is_same_v<T, Model::LinearConstraintData> ||
                     std::is_same_v<T, Model::SOCConstraintData>) {
        Eigen::MatrixXd Ad(data.A);
        builder->AddLinearConstraint(cids[clique], Ad, data.b, data.vars);
      } else if constexpr (std::is_same_v<T, Model::PSDConstraintData>) {
        int n = data.B.rows();
        auto psd = std::make_unique<PSDConstraint>(n, data.A_list, data.B);
        builder->AddPSDConstraint(cids[clique], std::move(psd), data.vars);
      } else if constexpr (std::is_same_v<T, Model::QuadraticCostData>) {
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
                                          Model::EqualityConstraintData>) {
        int p = data.C.rows();
        std::vector<int> dual(p);
        for (int j = 0; j < p; ++j) dual[j] = next_dual++;
        Eigen::MatrixXd Cd(data.C);
        builder->AddEquality(cids[clique], Cd, data.d,
                              data.primal_vars, dual);
      }
    }, model.constraint(i));
  }

  auto result = builder->Build();
  BuildInternal(model, config, &result.clique_tree);
}

void KKTSystem::BuildQuotientAMD(const Model& model,
                                 const SolverConfiguration& config) {
  TreeSpec tree;
  for (int i = 0; i < model.num_constraints(); ++i) {
    tree.AddClique();
    tree.Assign(i, i);
  }
  BuildFromTree(model, tree, config);
}

void KKTSystem::RegisterAssemblersWithTreeSolver() {
  if (!tree_solver_) return;
  int seg = 0;
  // Registration order determines RowSpace segment layout.
  // Pass 1: Linear.
  for (int i = 0; i < (int)linear_assemblers_.size(); ++i) {
    auto* slca = linear_assemblers_[i];
    if (!slca) continue;
    rs_segment_offset_[i] = seg;
    for (const auto& lc : slca->constraints()) {
      tree_solver_->RegisterLinearSubAssembler(lc.get());
      seg++;
    }
  }
  // Pass 2: PSD.
  for (int i = 0; i < (int)psd_assemblers_.size(); ++i) {
    auto* pasm = psd_assemblers_[i];
    if (!pasm) continue;
    rs_segment_offset_[i] = seg;
    for (const auto& lc : pasm->constraints()) {
      tree_solver_->RegisterLinearSubAssembler(lc.get());
      seg++;
    }
  }
  // Pass 3: SOC.
  for (int i = 0; i < (int)soc_assemblers_.size(); ++i) {
    auto* sasm = soc_assemblers_[i];
    if (!sasm) continue;
    rs_segment_offset_[i] = seg;
    for (const auto& lc : sasm->constraints()) {
      tree_solver_->RegisterLinearSubAssembler(lc.get());
      seg++;
    }
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

Eigen::VectorXd KKTSystem::GatherConstraintRows(
    ConstraintId id, const RowSpace& rs) const {
  int offset = rs_segment_offset_.at(id);
  // Linear and SOC assemblers both inherit GatherRows from
  // SparseLinearConstraintAssembler.
  if (linear_assemblers_[id]) {
    return linear_assemblers_[id]->GatherRows(rs, offset);
  }
  if (soc_assemblers_[id]) {
    return soc_assemblers_[id]->GatherRows(rs, offset);
  }
  // PSD: return flat vectorized form.
  if (psd_assemblers_[id]) {
    // PSD sub-constraints aren't decomposed the same way.
    // For now, the RowSpace segment is contiguous for PSD.
    int n2 = rs.sizes[offset];
    return rs.col().segment(rs.offsets[offset], n2);
  }
  return {};
}

}  // namespace conex
