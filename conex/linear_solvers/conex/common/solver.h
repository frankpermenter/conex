#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/chordal_decomp.h"
#include "conex/common/conex.h"
#include "conex/common/problem.h"
#include "conex/common/tree_spec.h"

namespace conex {

class ConstraintManager;
class KKTSolverBase;
class SparseLinearConstraintAssembler;
class SparsePSDConstraintAssembler;
class SparseSOCConstraintAssembler;
class SparseQuadraticTermAssembler;
class SparseEqualityConstraintAssembler;
class SymmetricLinearSystemTreeSolver;
class TreeSolverBuilder;

class Solver {
 public:
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

  // If chordal decomposition was applied, returns the expansion for
  // mapping solutions back.  Otherwise nullptr.
  const ChordalExpansion* chordal_expansion() const {
    return chordal_expansion_.get();
  }

  // Prevent implicit copy (unique_ptr members).
  Solver();
  ~Solver();
  Solver(Solver&&) noexcept;
  Solver& operator=(Solver&&) noexcept;

 private:
  void BuildInternal(const Problem& problem,
                     const SolverConfiguration& config);
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
  std::unique_ptr<ChordalExpansion> chordal_expansion_;
};

}  // namespace conex
