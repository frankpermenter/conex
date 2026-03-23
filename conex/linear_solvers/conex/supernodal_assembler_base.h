#pragma once
#include <memory>
#include <vector>

#include "conex/constraint_interface.h"
#include "conex/newton_step.h"
#include <Eigen/Dense>
namespace conex {

// Interface for lazy evaluation of a symmetric matrix.  Provides block
// accessors so that entries can be computed on demand and written directly
// into tree-solver storage without materializing the full matrix.
class LazySymmetricMatrix {
 public:
  virtual ~LazySymmetricMatrix() = default;
  virtual void set_order(const std::vector<int>& perm) = 0;

  // Add block to dest:  dest += Q(row:row+rows, col:col+cols)
  virtual void add_block(int row, int col, int rows, int cols,
                         Eigen::Ref<Eigen::MatrixXd> dest) const = 0;

  // Add lower triangle of diagonal block:
  //   dest.triangularView<Lower>() += Q(pos:pos+size, pos:pos+size)
  virtual void add_block_lower(int pos, int size,
                               Eigen::Ref<Eigen::MatrixXd> dest) const = 0;

  virtual int rows() const = 0;
  virtual int cols() const = 0;
};

// Manages the transfer of clique submatrix to supernodal data structure.
// The SetDenseData triggers an update the submatrix which
// is store in an Eigen::Map.
class SupernodalAssemblerBase : public IVariableShape {
 public:
  SupernodalAssemblerBase(const std::vector<int>& shared_variables) {
    SetPrimalVariables(shared_variables);
  }
  SupernodalAssemblerBase(const std::vector<int>& primal_variables,
                          const std::vector<int>& dual_variables) {
    std::vector<int> variables = primal_variables;
    variables.insert(variables.end(), dual_variables.begin(),
                     dual_variables.end());
    SetPrimalVariables(primal_variables);
    SetDualVariables(dual_variables);
  }
  SupernodalAssemblerBase(){};
  virtual ~SupernodalAssemblerBase(){};

  int number_of_variables() const override {
    return primal_variables().size() + dual_variables().size();
  }

  virtual bool is_dynamic() const { return false; }
  virtual bool is_positive_definite() const { return true; }

  // Decompose this assembler into sub-assemblers aligned with the given
  // maximal cliques.  Default: returns {this} (no decomposition).
  // Called by the tree solver which provides maximal cliques.
  virtual std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) {
    (void)maximal_cliques;
    return {this};
  }

  // Called after Decompose by MakeTreeSolver.  Override to register
  // decomposed cone-inequality assemblers with the ConstraintManager so
  // the IPM can iterate on them (PrepareStep / TakeStep).
  // Default: no-op (standard assemblers are already registered).
  virtual void RegisterDecomposedConeInequalities(
      class ConstraintManager* /*cm*/) {}

  virtual std::vector<int> variables() const {
    std::vector<int> variables = primal_variables_;
    variables.insert(variables.end(), dual_variables_.begin(),
                     dual_variables_.end());
    return variables;
  }
  std::vector<std::vector<int>> get_cliques() const override {
    return {variables()};
  }
  virtual const std::vector<int>& primal_variables() const {
    return primal_variables_;
  }
  virtual const std::vector<int>& dual_variables() const {
    return dual_variables_;
  }

  virtual void SetDenseData() = 0;
  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }
  virtual void set_precompute_gram(bool) {}

  void SetPrimalVariables(const std::vector<int>& variables) {
    primal_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
  };
  void SetDualVariables(const std::vector<int>& variables) {
    dual_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
    submatrix_data_.m_ = number_of_variables();
  };

  SchurComplementSystem* submatrix_data() { return &submatrix_data_; }

 protected:
  WorkspaceSchurComplement submatrix_data_;
  int num_variables_;
  std::vector<int> primal_variables_;
  std::vector<int> dual_variables_;
};

}  // namespace conex
