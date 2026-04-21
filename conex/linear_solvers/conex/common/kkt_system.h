#pragma once
#include <memory>
#include <unordered_map>
#include <vector>

#include "conex/common/clique_tree.h"
#include "conex/common/conex.h"
#include "conex/common/kkt_solver_interface.h"
#include "conex/common/model.h"
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

// Owns a fully constructed KKT system: tree solver, assemblers, and
// constraint manager.  This is the result of compiling a Model into
// something that can be factored and solved.
//
// Tests and algorithms use this directly.  Solver uses it internally.
class KKTSystem {
 public:
  // Build a KKT system from a Model.  No preprocessing is performed —
  // the caller is responsible for passing a reduced Model if desired.
  static KKTSystem Build(const Model& model,
                         const SolverConfiguration& config = {});

  static KKTSystem Build(const Model& model,
                         const TreeSpec& tree,
                         const SolverConfiguration& config = {});

  static KKTSystem BuildDense(const Model& model);

  KKTSolverBase* kkt();
  const KKTSolverBase* kkt() const;

  SymmetricLinearSystemTreeSolver* tree_solver();
  const SymmetricLinearSystemTreeSolver* tree_solver() const;

  const std::vector<int>& dual_variables(ConstraintId id) const;

  // Gather a RowSpace value back to a dense vector in the original
  // row order of Model constraint `id`.  Only valid for linear/SOC
  // constraints (cone constraints with vector-valued duals).
  Eigen::VectorXd GatherConstraintRows(
      ConstraintId id, const RowSpace& rs) const;

  // Segment offset in the RowSpace for each Model constraint.
  // Set during RegisterAssemblersWithTreeSolver.
  int rowspace_segment_offset(ConstraintId id) const {
    return rs_segment_offset_.at(id);
  }

  KKTSystem();
  ~KKTSystem();
  KKTSystem(KKTSystem&&) noexcept;
  KKTSystem& operator=(KKTSystem&&) noexcept;

 private:
  void BuildInternal(const Model& model,
                     const SolverConfiguration& config,
                     const CliqueTree* tree_override = nullptr);
  void BuildFromTree(const Model& model,
                     const TreeSpec& tree,
                     const SolverConfiguration& config);
  void BuildQuotientAMD(const Model& model,
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
  std::unordered_map<int, int> rs_segment_offset_;
};

}  // namespace conex
