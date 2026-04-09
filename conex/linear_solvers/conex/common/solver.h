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

  // Get the dual variable indices allocated for an equality constraint.
  const std::vector<int>& dual_variables(ConstraintId id) const {
    return dual_var_map_.at(id);
  }

 private:
  void BuildInternal(const Problem& problem,
                     const SolverConfiguration& config) {
    const int n = problem.num_variables();
    cm_ = std::make_unique<ConstraintManager>(n);

    linear_assemblers_.resize(problem.num_constraints(), nullptr);
    psd_assemblers_.resize(problem.num_constraints(), nullptr);
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

    tree_solver_ = MakeTreeSolver(cm_.get(), config);
    RegisterAssemblersWithTreeSolver();
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
        } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          Eigen::SparseMatrix<double> A_vec;
          Eigen::VectorXd b_vec;
          VectorizePSD(data.A_list, data.B, &A_vec, &b_vec);
          Eigen::MatrixXd Ad(A_vec);
          builder_->AddLinearConstraint(cids[clique], Ad, b_vec, data.vars);
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
    RegisterAssemblersWithTreeSolver();
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
        } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          Eigen::SparseMatrix<double> A_vec;
          Eigen::VectorXd b_vec;
          VectorizePSD(data.A_list, data.B, &A_vec, &b_vec);
          Eigen::MatrixXd Ad(A_vec);
          builder_->AddLinearConstraint(cids[i], Ad, b_vec, data.vars);
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
    RegisterAssemblersWithTreeSolver();
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
          Eigen::MatrixXd Ad(data.A);
          Eigen::MatrixXd AtA = Ad.transpose() * Ad;
          for (int r = 0; r < static_cast<int>(data.vars.size()); ++r)
            for (int c = 0; c < static_cast<int>(data.vars.size()); ++c)
              ds->matrix()(data.vars[r], data.vars[c]) += AtA(r, c);
        } else if constexpr (std::is_same_v<T, Problem::PSDConstraintData>) {
          Eigen::SparseMatrix<double> A_vec;
          Eigen::VectorXd b_vec;
          VectorizePSD(data.A_list, data.B, &A_vec, &b_vec);
          Eigen::MatrixXd Ad(A_vec);
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

  void RegisterAssemblersWithTreeSolver() {
    if (!tree_solver_) return;
    // Register decomposed sub-assemblers (not top-level assemblers).
    // Sub-assemblers have VectorBlockContributions from Register.
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

  std::unique_ptr<TreeSolverBuilder> builder_;
  std::unique_ptr<ConstraintManager> cm_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  std::unique_ptr<DenseKKTSolver> dense_solver_;
  std::vector<SparseLinearConstraintAssembler*> linear_assemblers_;
  std::vector<class SparsePSDConstraintAssembler*> psd_assemblers_;
  std::vector<SparseQuadraticTermAssembler*> quadratic_assemblers_;
  std::vector<SparseEqualityConstraintAssembler*> equality_assemblers_;
  std::unordered_map<int, std::vector<int>> dual_var_map_;
};

}  // namespace conex
