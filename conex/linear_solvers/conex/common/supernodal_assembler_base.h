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

// Interface for lazy evaluation of a symmetric matrix.  Provides block
// accessors so that entries can be computed on demand and written directly
// into tree-solver storage without materializing the full matrix.
//
// Two-phase protocol:
//   1. RegisterContributions(perm, blocks): called once at Finalize.
//      The lazy matrix saves perm and block destinations, precomputes
//      permuted data and scatter tables.
//   2. ContributeBlocks(): called at each assembly with no arguments.
//      Uses the saved info to write all blocks.
//
// Legacy protocol (add_block/add_block_lower) is still supported for
// backward compatibility but should be replaced by the two-phase protocol.
class LazySymmetricMatrix {
 public:
  virtual ~LazySymmetricMatrix() = default;
  virtual void set_order(const std::vector<int>& perm) = 0;

  // --- Two-phase protocol ---

  // Register block contributions for a clique.  Called once at Finalize.
  // clique_id identifies the clique (subsystem index in the tree solver).
  // The lazy matrix should save perm, blocks, and precompute accordingly.
  // Returns true if the two-phase protocol is supported; if false,
  // the caller falls back to the legacy add_block/add_block_lower path.
  virtual bool RegisterContributions(
      int clique_id,
      const std::vector<int>& perm,
      const std::vector<BlockContribution>& blocks) {
    (void)clique_id;
    (void)perm;
    (void)blocks;
    return false;
  }

  // Write all blocks for the given clique.  Called at each assembly.
  // Only valid after RegisterContributions(clique_id, ...) returned true.
  virtual void ContributeBlocks(int clique_id) {
    (void)clique_id;
  }

  // --- Legacy protocol ---

  // Add block to dest:  dest += Q(row:row+rows, col:col+cols)
  virtual void add_block(int row, int col, int rows, int cols,
                         Eigen::Ref<Eigen::MatrixXd> dest) const = 0;

  // Add lower triangle of diagonal block:
  //   dest.triangularView<Lower>() += Q(pos:pos+size, pos:pos+size)
  virtual void add_block_lower(int pos, int size,
                               Eigen::Ref<Eigen::MatrixXd> dest) const = 0;

  virtual int rows() const = 0;
  virtual int cols() const = 0;

  // Number of supernode columns in the permuted layout (set by contributor).
  virtual void set_sn_count(int) {}
};

// Base class for assemblers that feed data into the tree solver.
// Subclasses provide a LazySymmetricMatrix via GetLazyEvaluator() so the
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

  virtual LazySymmetricMatrix* GetLazyEvaluator() { return nullptr; }
  virtual void set_precompute_gram(bool) {}

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
