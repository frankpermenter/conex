#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/block_variable.h"
#include "conex/common/conex.h"
#include "conex/common/dense_kkt_solver.h"
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
  // Build a solver from a problem.
  // If config.use_quotient_amd is true, uses weighted AMD on the
  // constraint graph (faster for problems with known block structure).
  // Otherwise uses variable-level AMD on the full KKT matrix.
  static Solver Build(const Problem& problem,
                      const SolverConfiguration& config = {}) {
    Solver s;
    if (config.use_quotient_amd) {
      s.BuildQuotientAMD(problem, config);
    } else {
      s.BuildInternal(problem, config);
    }
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

  // Build a dense solver (reference / small problems).
  // Assembles the full KKT matrix from the Problem.
  static Solver BuildDense(const Problem& problem) {
    Solver s;
    s.BuildDenseInternal(problem);
    return s;
  }

  // Factor the assembled system.  Must be called before SolveInto.
  bool AssembleAndFactor() { return solver()->AssembleAndFactor(); }

  // Create a BlockVariable matching this solver's partition.
  BlockVariable MakeBlockVariable(int cols = 1) {
    return solver()->MakeBlockVariable(cols);
  }
  BlockVariable MakeBlockVariable(Eigen::Ref<const Eigen::MatrixXd> x) {
    return solver()->MakeBlockVariable(x);
  }

  // Solve: rhs in, solution out (blocked, no dense vectors).
  void SolveInto(const BlockVariable& rhs, BlockVariable& dest) const {
    solver()->SolveInto(rhs, dest);
  }

  // Dense solve (convenience).
  Eigen::VectorXd Solve(Eigen::Ref<const Eigen::VectorXd> rhs) const {
    return solver()->Solve(rhs);
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

  // Compute A * x per-clique using VectorBlockContributions.
  // Scatters x's separators into sep_scratch_in, then each constraint
  // reads from supernode blocks + sep_scratch_in via registered offsets.
  Eigen::VectorXd MultiplyA(ConstraintId id,
                             const BlockVariable& x) const {
    auto* slca = linear_assemblers_.at(id);
    CONEX_DEMAND(slca, "Constraint is not a linear constraint.");

    if (!tree_solver_ || !slca->partition_bound()) {
      return slca->ComputeResiduals(x.Gather());
    }

    auto& sep_in = tree_solver_->sep_scratch_in();
    tree_solver_->ScatterSeparators(x.partition(), sep_in);

    const auto& constraints = slca->constraints();
    const auto& row_map = slca->row_map();
    int m = slca->num_global_rows();
    int nc = x.cols();

    std::vector<Eigen::MatrixXd> locals(constraints.size());
    for (size_t ci = 0; ci < constraints.size(); ++ci) {
      locals[ci] = constraints[ci]->gram().MultiplyA(
          x.partition(), sep_in, nc);
    }

    Eigen::VectorXd result = Eigen::VectorXd::Zero(m);
    for (int r = 0; r < m; ++r) {
      const auto& rm = row_map[r];
      if (rm.constraint_index >= 0)
        result(r) = locals[rm.constraint_index](rm.local_row, 0);
    }
    return result;
  }

  // TreeRHS is now in conex namespace (from tree_rhs.h).

  // Accumulate A^T * v into a TreeRHS.
  void AccumulateAtranspose(ConstraintId id,
                            const Eigen::VectorXd& v,
                            TreeRHS& rhs) const {
    auto* slca = linear_assemblers_.at(id);
    CONEX_DEMAND(slca, "Constraint is not a linear constraint.");
    slca->ComputeTransposeProduct(v, rhs);
  }

  // Accumulate Q * x into a TreeRHS.
  // sep_scratch_in must be pre-populated via ScatterSeparators(x).
  void AccumulateQx(ConstraintId id,
                    const BlockVariable& x,
                    TreeRHS& rhs) const {
    CONEX_DEMAND(tree_solver_, "AccumulateQx requires tree solver.");
    auto* qasm = quadratic_assemblers_.at(id);
    CONEX_DEMAND(qasm, "Constraint is not a quadratic cost.");
    qasm->ComputeProduct(x, tree_solver_->sep_scratch_in(), rhs);
  }

  // Convenience: compute A^T * v with full gather.
  void MultiplyAtranspose(ConstraintId id,
                          const Eigen::VectorXd& v,
                          BlockVariable& result) const {
    if (!tree_solver_) {
      auto* slca = linear_assemblers_.at(id);
      CONEX_DEMAND(slca, "Constraint is not a linear constraint.");
      result.ScatterFrom(slca->ComputeTransposeProduct(v));
      return;
    }
    result.SetZero();
    tree_solver_->sep_scratch_out().SetZero();
    auto rhs = tree_solver_->MakeTreeRHS(result);
    AccumulateAtranspose(id, v, rhs);
    tree_solver_->GatherSeparators(result.partition(),
                                    tree_solver_->sep_scratch_out());
  }

  // Convenience: compute Q * x with full gather.
  void MultiplyQ(ConstraintId id,
                 const BlockVariable& x,
                 BlockVariable& result) const {
    if (!tree_solver_) {
      auto* qasm = quadratic_assemblers_.at(id);
      CONEX_DEMAND(qasm, "Constraint is not a quadratic cost.");
      Eigen::VectorXd x_dense = x.Gather().col(0);
      Eigen::VectorXd Qx = Eigen::VectorXd::Zero(x_dense.size());
      for (const auto& sub : qasm->sub_assemblers()) {
        const auto& vars = sub.primal_variables();
        int nv = static_cast<int>(vars.size());
        Eigen::VectorXd xl(nv);
        for (int j = 0; j < nv; ++j) xl(j) = x_dense(vars[j]);
        Eigen::VectorXd ql = sub.Q_block() * xl;
        for (int j = 0; j < nv; ++j) Qx(vars[j]) += ql(j);
      }
      result.ScatterFrom(Qx);
      return;
    }
    tree_solver_->ScatterSeparators(x.partition(),
                                     tree_solver_->sep_scratch_in());
    result.SetZero();
    tree_solver_->sep_scratch_out().SetZero();
    auto rhs = tree_solver_->MakeTreeRHS(result);
    AccumulateQx(id, x, rhs);
    tree_solver_->GatherSeparators(result.partition(),
                                    tree_solver_->sep_scratch_out());
  }

  // Access the underlying solver.
  KKTSolverBase* solver() {
    if (dense_solver_) return static_cast<KKTSolverBase*>(dense_solver_.get());
    return static_cast<KKTSolverBase*>(tree_solver_.get());
  }
  const KKTSolverBase* solver() const {
    if (dense_solver_) return static_cast<const KKTSolverBase*>(dense_solver_.get());
    return static_cast<const KKTSolverBase*>(tree_solver_.get());
  }

  // Access the tree solver specifically (nullptr if dense).
  SymmetricLinearSystemTreeSolver* tree_solver() { return tree_solver_.get(); }
  const SymmetricLinearSystemTreeSolver* tree_solver() const { return tree_solver_.get(); }

  int num_variables() const { return solver()->number_of_variables(); }

  // Get the dual variable indices allocated for an equality constraint.
  const std::vector<int>& dual_variables(ConstraintId id) const {
    return dual_var_map_.at(id);
  }

 private:
  void BuildInternal(const Problem& problem,
                     const SolverConfiguration& config) {
    const int n = problem.num_variables();
    cm_ = std::make_unique<ConstraintManager>(n);

    // Map each constraint to its assembler.
    linear_assemblers_.resize(problem.num_constraints(), nullptr);
    quadratic_assemblers_.resize(problem.num_constraints(), nullptr);

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
    quadratic_assemblers_.resize(problem.num_constraints(), nullptr);

    // Allocate dual variables: start after the max primal index.
    int next_dual = problem.num_variables();

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
          int p = data.C.rows();
          std::vector<int> dual(p);
          for (int j = 0; j < p; ++j) dual[j] = next_dual++;
          dual_var_map_[i] = dual;
          Eigen::MatrixXd Cd(data.C);
          builder_->AddEquality(cids[clique], Cd, data.d,
                                data.primal_vars, dual);
        }
      }, problem.constraint(i));
    }

    auto result = builder_->Build();
    tree_solver_ = std::move(result.solver);
  }

  void BuildQuotientAMD(const Problem& problem,
                        const SolverConfiguration& config) {
    builder_ = std::make_unique<TreeSolverBuilder>();
    linear_assemblers_.resize(problem.num_constraints(), nullptr);
    quadratic_assemblers_.resize(problem.num_constraints(), nullptr);

    // Allocate dual variables.
    int next_dual = problem.num_variables();

    // One clique per constraint, all with parent = -1.
    // TreeSolverBuilder's quotient AMD will discover the tree.
    std::vector<int> cids(problem.num_constraints());
    for (int i = 0; i < problem.num_constraints(); ++i)
      cids[i] = builder_->AddClique();  // no parent → triggers quotient AMD

    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          Eigen::MatrixXd Ad(data.A);
          builder_->AddLinearConstraint(cids[i], Ad, data.b, data.vars);
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
          builder_->AddCost(cids[i], Qd, data.vars);
        } else if constexpr (std::is_same_v<T,
                                            Problem::EqualityConstraintData>) {
          int p = data.C.rows();
          std::vector<int> dual(p);
          for (int j = 0; j < p; ++j) dual[j] = next_dual++;
          dual_var_map_[i] = dual;
          Eigen::MatrixXd Cd(data.C);
          builder_->AddEquality(cids[i], Cd, data.d,
                                data.primal_vars, dual);
        }
      }, problem.constraint(i));
    }

    auto result = builder_->Build();
    tree_solver_ = std::move(result.solver);
  }

  void BuildDenseInternal(const Problem& problem) {
    int n = problem.num_variables();

    // Allocate dual variables for equality constraints.
    int next_dual = n;
    for (int i = 0; i < problem.num_constraints(); ++i) {
      if (auto* eq = std::get_if<Problem::EqualityConstraintData>(
              &problem.constraint(i))) {
        int p = eq->C.rows();
        std::vector<int> dual(p);
        for (int j = 0; j < p; ++j) dual[j] = next_dual++;
        dual_var_map_[i] = dual;
      }
    }
    int n_total = next_dual;

    auto ds = std::make_unique<DenseKKTSolver>(n_total);

    // Assemble the full KKT matrix.
    for (int i = 0; i < problem.num_constraints(); ++i) {
      std::visit([&](const auto& data) {
        using T = std::decay_t<decltype(data)>;
        if constexpr (std::is_same_v<T, Problem::LinearConstraintData>) {
          // A'A on vars.
          Eigen::MatrixXd Ad(data.A);
          Eigen::MatrixXd AtA = Ad.transpose() * Ad;
          for (int r = 0; r < static_cast<int>(data.vars.size()); ++r)
            for (int c = 0; c < static_cast<int>(data.vars.size()); ++c)
              ds->matrix()(data.vars[r], data.vars[c]) += AtA(r, c);
        } else if constexpr (std::is_same_v<T, Problem::QuadraticCostData>) {
          Eigen::MatrixXd Qd = data.Q_dense.size() > 0
              ? data.Q_dense : Eigen::MatrixXd(data.Q_sparse);
          for (int r = 0; r < static_cast<int>(data.vars.size()); ++r)
            for (int c = 0; c < static_cast<int>(data.vars.size()); ++c)
              ds->matrix()(data.vars[r], data.vars[c]) += Qd(r, c);
        } else if constexpr (std::is_same_v<T,
                                            Problem::EqualityConstraintData>) {
          const auto& dual = dual_var_map_.at(i);
          Eigen::MatrixXd Cd(data.C);
          // [0, C'; C, 0] block.
          for (int r = 0; r < Cd.rows(); ++r)
            for (int c = 0; c < static_cast<int>(data.primal_vars.size()); ++c) {
              ds->matrix()(dual[r], data.primal_vars[c]) += Cd(r, c);
              ds->matrix()(data.primal_vars[c], dual[r]) += Cd(r, c);
            }
        }
      }, problem.constraint(i));
    }

    dense_solver_ = std::move(ds);
  }

  std::unique_ptr<TreeSolverBuilder> builder_;
  std::unique_ptr<ConstraintManager> cm_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  std::unique_ptr<DenseKKTSolver> dense_solver_;
  std::vector<SparseLinearConstraintAssembler*> linear_assemblers_;
  std::vector<SparseQuadraticTermAssembler*> quadratic_assemblers_;
  std::unordered_map<int, std::vector<int>> dual_var_map_;
};

}  // namespace conex
