#pragma once
#include <memory>
#include <vector>

#include "conex/common/block_variable.h"
#include "conex/common/conex.h"
#include "conex/common/problem.h"
#include "conex/common/tree_spec.h"
#include "conex/algorithms/tree_solver_builder.h"
#include "conex/common/sparse_linear_constraint.h"
#include "conex/common/sparse_psd_constraint.h"
#include "conex/common/sparse_quadratic_term.h"
#include "conex/common/sparse_equality_constraint.h"
#include "conex/common/structural_rank.h"
#include "conex/tree_solver/kkt_solver_factory.h"
#include "conex/tree_solver/kkt_tree_solver.h"

namespace conex {

// Solver: wraps a tree solver built from a Problem.
// Maps ConstraintId handles to internal assemblers for SetWeights,
// ComputeResidual, and ComputeTransposeProduct.
class Solver {
 public:
  // Build a solver from a problem (automatic tree via AMD).
  static Solver Build(const Problem& problem,
                      const SolverConfiguration& config = {}) {
    Solver s;
    s.BuildInternal(problem, config);
    return s;
  }

  // Build with a custom tree topology.
  static Solver Build(const Problem& problem,
                      const TreeSpec& tree,
                      const SolverConfiguration& config = {}) {
    Solver s;
    s.BuildFromTree(problem, tree, config);
    return s;
  }

  // Factor the assembled system.  Must be called before SolveInto.
  bool AssembleAndFactor() { return tree_solver_->AssembleAndFactor(); }

  // Create a BlockVariable matching this solver's partition.
  BlockVariable MakeBlockVariable(int cols = 1) {
    return tree_solver_->MakeBlockVariable(cols);
  }
  BlockVariable MakeBlockVariable(Eigen::Ref<const Eigen::MatrixXd> x) {
    return tree_solver_->MakeBlockVariable(x);
  }

  // Solve: rhs in, solution out (blocked, no dense vectors).
  void SolveInto(const BlockVariable& rhs, BlockVariable& dest) const {
    tree_solver_->SolveInto(rhs, dest);
  }

  // Dense solve (convenience).
  Eigen::VectorXd Solve(Eigen::Ref<const Eigen::VectorXd> rhs) const {
    return tree_solver_->Solve(rhs);
  }

  // Set per-row weights on a linear constraint (for IRLS/barrier).
  void SetWeights(ConstraintId id, const Eigen::VectorXd& weights) {
    auto* slca = linear_assemblers_.at(id);
    CONEX_DEMAND(slca, "Constraint is not a linear constraint.");
    slca->SetWeights(weights);
  }

  // Compute A*x for a linear constraint.
  // x must have been scattered into the solver's partition.
  Eigen::VectorXd ComputeResidual(ConstraintId id,
                                   const Eigen::VectorXd& x) const {
    auto* slca = linear_assemblers_.at(id);
    CONEX_DEMAND(slca, "Constraint is not a linear constraint.");
    return slca->ComputeResiduals(x);
  }

  // Compute A'*v for a linear constraint.
  Eigen::VectorXd ComputeTransposeProduct(ConstraintId id,
                                           const Eigen::VectorXd& v) const {
    auto* slca = linear_assemblers_.at(id);
    CONEX_DEMAND(slca, "Constraint is not a linear constraint.");
    return slca->ComputeTransposeProduct(v);
  }

  // Access the underlying tree solver (for ScatterToBlocks etc).
  SymmetricLinearSystemTreeSolver& tree_solver() { return *tree_solver_; }
  const SymmetricLinearSystemTreeSolver& tree_solver() const {
    return *tree_solver_;
  }

  int num_variables() const { return tree_solver_->number_of_variables(); }

 private:
  void BuildInternal(const Problem& problem,
                     const SolverConfiguration& config) {
    const int n = problem.num_variables();
    cm_ = std::make_unique<ConstraintManager>(n);

    // Map each constraint to its assembler.
    linear_assemblers_.resize(problem.num_constraints(), nullptr);

    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;

        if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          auto slc = std::make_unique<SparseLinearConstraint>(data.A, data.b);
          auto asm_ptr = std::make_unique<SparseLinearConstraintAssembler>(
              std::move(slc), data.vars);
          linear_assemblers_[i] = asm_ptr.get();
          cm_->AddCustomAssembler(std::move(asm_ptr));

        } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
          auto asm_ptr = std::make_unique<SparseQuadraticTermAssembler>(
              data.Q_sparse, data.vars);
          cm_->AddCustomAssembler(std::move(asm_ptr));

        } else if constexpr (std::is_same_v<T,
                                            Problem::EqualityConstraintData>) {
          auto sec = std::make_unique<SparseEqualityConstraint>(
              data.C, data.d);
          auto dual = cm_->AllocateDualVariables(data.C.rows());
          auto asm_ptr = std::make_unique<SparseEqualityConstraintAssembler>(
              std::move(sec), data.primal_vars, dual);
          cm_->AddCustomAssembler(std::move(asm_ptr));
        }
      }, problem.constraint(i));
    }

    tree_solver_ = MakeTreeSolver(cm_.get(), config);
  }

  void BuildFromTree(const Problem& problem,
                     const TreeSpec& tree,
                     const SolverConfiguration& config) {
    builder_ = std::make_unique<TreeSolverBuilder>();

    std::vector<int> cids(tree.num_cliques());
    for (int k = 0; k < tree.num_cliques(); ++k) {
      if (tree.parent(k) < 0)
        cids[k] = builder_->AddClique();
      else
        cids[k] = builder_->AddClique(cids[tree.parent(k)]);
    }

    linear_assemblers_.resize(problem.num_constraints(), nullptr);

    for (int i = 0; i < problem.num_constraints(); ++i) {
      int clique = tree.clique_of(i);
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          Eigen::MatrixXd Ad(data.A);
          builder_->AddLinearConstraint(cids[clique], Ad, data.b, data.vars);
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
          builder_->AddCost(cids[clique], Qd, data.vars);
        } else if constexpr (std::is_same_v<T,
                                            Problem::EqualityConstraintData>) {
          Eigen::MatrixXd Cd(data.C);
          builder_->AddEquality(cids[clique], Cd, data.d,
                                data.primal_vars, data.dual_vars);
        }
      }, problem.constraint(i));
    }

    auto result = builder_->Build();
    tree_solver_ = std::move(result.solver);
  }

  std::unique_ptr<TreeSolverBuilder> builder_;
  std::unique_ptr<ConstraintManager> cm_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  // Per-constraint assembler pointers (nullptr for non-linear constraints).
  std::vector<SparseLinearConstraintAssembler*> linear_assemblers_;
};

}  // namespace conex
