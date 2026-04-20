#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/clique_tree.h"
#include "conex/common/conex.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/problem.h"
#include "conex/common/tree_spec.h"

namespace conex {

class ConstraintManager;
class SparseLinearConstraintAssembler;
class SparsePSDConstraintAssembler;
class SparseSOCConstraintAssembler;
class SparseQuadraticTermAssembler;
class SparseEqualityConstraintAssembler;
class SymmetricLinearSystemTreeSolver;
class TreeSolverBuilder;

class Solver {
 public:
  // Build preprocesses the problem (removes structurally rank-deficient
  // columns) before constructing the tree solver.  Use ExpandSolution()
  // and ReduceVector() to convert between original and reduced spaces.
  static Solver Build(const Problem& problem,
                      const SolverConfiguration& config = {});

  static Solver Build(const Problem& problem,
                      const TreeSpec& tree,
                      const SolverConfiguration& config = {});

  static Solver BuildDense(const Problem& problem);

  KKTSolverBase* solver();
  const KKTSolverBase* solver() const;

  SymmetricLinearSystemTreeSolver* tree_solver();
  const SymmetricLinearSystemTreeSolver* tree_solver() const;

  const std::vector<int>& dual_variables(ConstraintId id) const;

  // Preprocessing: map between original and reduced variable spaces.
  bool was_reduced() const { return expansion_.was_reduced(); }
  Eigen::VectorXd ExpandSolution(const Eigen::VectorXd& x_reduced) const {
    return expansion_.Expand(x_reduced);
  }
  Eigen::VectorXd ReduceVector(const Eigen::VectorXd& v) const {
    return expansion_.Reduce(v);
  }
  const Expansion& expansion() const { return expansion_; }

  // The linear cost in the reduced variable space (empty if no cost set).
  const Eigen::VectorXd& linear_cost() const;

  // Build the cost RHS in solver format (reduced space).
  // Returns a zero RHS if no linear cost was set.
  SolverRHS MakeCostRHS();

  // Prevent implicit copy (unique_ptr members).
  Solver();
  ~Solver();
  Solver(Solver&&) noexcept;
  Solver& operator=(Solver&&) noexcept;

 private:
  void BuildInternal(const Problem& problem,
                     const SolverConfiguration& config,
                     const CliqueTree* tree_override = nullptr);
  void BuildFromTree(const Problem& problem,
                     const TreeSpec& tree,
                     const SolverConfiguration& config);
  void BuildQuotientAMD(const Problem& problem,
                        const SolverConfiguration& config);
  void RegisterAssemblersWithTreeSolver();

  std::unique_ptr<TreeSolverBuilder> builder_;
  std::unique_ptr<ConstraintManager> cm_;
  std::unique_ptr<SymmetricLinearSystemTreeSolver> tree_solver_;
  std::vector<SparseLinearConstraintAssembler*> linear_assemblers_;
  std::vector<SparsePSDConstraintAssembler*> psd_assemblers_;
  std::vector<SparseSOCConstraintAssembler*> soc_assemblers_;
  std::vector<SparseQuadraticTermAssembler*> quadratic_assemblers_;
  std::vector<SparseEqualityConstraintAssembler*> equality_assemblers_;
  std::unordered_map<int, std::vector<int>> dual_var_map_;
  Expansion expansion_;
  Eigen::VectorXd reduced_linear_cost_;
};

}  // namespace conex
