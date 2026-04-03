#pragma once
#include <memory>
#include <vector>

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

// A vector contribution descriptor: where to accumulate A_perm_^T * v.
// Unlike BlockContribution, stores offsets (not pointers) since the
// destination (BlockVariable partition) changes between calls.
struct VectorBlockContribution {
  int q_start;       // column offset in A_perm_
  int length;        // number of contiguous columns
  int dest_block;    // subsystem index of destination block
  int dest_offset;   // row offset within the destination block
  bool dest_is_sn;   // true = supernode block, false = separator scratch
};

// Interface for assembling a symmetric matrix into tree-solver storage.
//
// Two-phase protocol:
//   1. RegisterContributions(clique_id, perm, blocks): called once at
//      FinalizeStructure.  The assembler saves perm and block destinations,
//      precomputes permuted data and scatter tables.
//   2. ContributeBlocks(clique_id): called at each assembly.
//      Uses saved info to write all blocks.
class BlockAssembler {
 public:
  virtual ~BlockAssembler() = default;

  // Permute internal data layout.  Called during RegisterContributions.
  virtual void set_order(const std::vector<int>& perm) = 0;

  // Register block contributions for a clique.  Called once at FinalizeStructure.
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

  // Register vector block contributions.  Called once at FinalizeStructure.
  virtual void RegisterVectorContributions(
      const std::vector<VectorBlockContribution>& /*blocks*/) {}
};

// Construction-time interface: provides sparsity (cliques), variable
// lists, and Decompose for the clique tree builder.
class CliqueProvider {
 public:
  CliqueProvider() = default;
  CliqueProvider(const std::vector<int>& primal_vars)
      : primal_variables_(primal_vars) {}
  CliqueProvider(const std::vector<int>& primal_vars,
                 const std::vector<int>& dual_vars)
      : primal_variables_(primal_vars), dual_variables_(dual_vars) {}
  virtual ~CliqueProvider() = default;

  virtual int number_of_variables() const {
    return primal_variables_.size() + dual_variables_.size();
  }
  virtual std::vector<std::vector<int>> get_cliques() const = 0;
  virtual std::vector<int> variables() const {
    std::vector<int> v = primal_variables_;
    v.insert(v.end(), dual_variables_.begin(), dual_variables_.end());
    return v;
  }
  virtual const std::vector<int>& primal_variables() const {
    return primal_variables_;
  }
  virtual const std::vector<int>& dual_variables() const {
    return dual_variables_;
  }
  virtual bool is_positive_definite() const { return true; }
  virtual bool is_dynamic() const { return false; }

  // Decompose into per-clique sub-assemblers.
  virtual std::vector<class SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) = 0;

  void SetPrimalVariables(const std::vector<int>& v) { primal_variables_ = v; }
  void SetDualVariables(const std::vector<int>& v) { dual_variables_ = v; }

 protected:
  std::vector<int> primal_variables_;
  std::vector<int> dual_variables_;
};

// Runtime sub-assembler base: variable list + BlockAssembler.
// Produced by CliqueProvider::Decompose.
class SupernodalAssemblerBase {
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

  virtual int number_of_variables() const {
    return primal_variables().size() + dual_variables().size();
  }

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
  virtual std::vector<std::vector<int>> get_cliques() const {
    return {variables()};
  }
  virtual const std::vector<int>& primal_variables() const {
    return primal_variables_;
  }
  virtual const std::vector<int>& dual_variables() const {
    return dual_variables_;
  }

  virtual BlockAssembler* GetBlockAssembler() = 0;

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
