#pragma once
#include <memory>
#include <vector>

#include "conex/common/constraint_interface.h"
#include <Eigen/Dense>
namespace conex {

// A block contribution request: where to write and what region of the
// permuted matrix it corresponds to.
struct BlockContribution {
  int q_row;    // row offset in permuted layout
  int q_col;    // col offset in permuted layout
  int rows;     // number of rows
  int cols;     // number of columns
  double* dest; // destination storage (additive write)
  int dest_ld;  // leading dimension of dest
  bool lower_only;  // if true, only write lower triangle (diagonal block)
};

// Interface for assembling a symmetric matrix into tree-solver storage.
//
// Two-phase protocol:
//   1. RegisterContributions(clique_id, perm, blocks): called once at
//      Finalize.  The assembler saves perm and block destinations,
//      precomputes permuted data and scatter tables.
//   2. ContributeBlocks(clique_id): called at each assembly.
//      Uses saved info to write all blocks.
class BlockAssembler {
 public:
  virtual ~BlockAssembler() = default;

  // Permute internal data layout.  Called during RegisterContributions.
  virtual void set_order(const std::vector<int>& perm) = 0;

  // Register block contributions for a clique.  Called once at Finalize.
  virtual bool RegisterContributions(
      int clique_id,
      const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) = 0;

  // Write all blocks for the given clique.  Called at each assembly.
  virtual void ContributeBlocks(int clique_id) = 0;

  virtual int rows() const = 0;
  virtual int cols() const = 0;

  // Number of supernode columns in the permuted layout (set by contributor).
  virtual void set_sn_count(int) {}
};

// Base class for assemblers that feed data into the tree solver.
// Subclasses provide a BlockAssembler via GetBlockAssembler() so the
// tree solver can write blocks directly into subsystem storage.
class SupernodalAssemblerBase : public IVariableShape {
 public:
  SupernodalAssemblerBase(const std::vector<int>& shared_variables) {
    SetPrimalVariables(shared_variables);
  }
  SupernodalAssemblerBase(const std::vector<int>& primal_variables,
                          const std::vector<int>& dual_variables) {
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

  virtual BlockAssembler* GetBlockAssembler() { return nullptr; }

  void SetPrimalVariables(const std::vector<int>& variables) {
    primal_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
  };
  void SetDualVariables(const std::vector<int>& variables) {
    dual_variables_ = variables;
    num_variables_ = primal_variables_.size() + dual_variables_.size();
  };

 protected:
  int num_variables_;
  std::vector<int> primal_variables_;
  std::vector<int> dual_variables_;
};

}  // namespace conex
