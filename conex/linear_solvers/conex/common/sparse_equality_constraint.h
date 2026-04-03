#pragma once
#include <list>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "conex/common/equality_constraint.h"
#include "conex/common/supernodal_assembler_base.h"

namespace conex {

// Decomposes a sparse equality constraint matrix C into dense sub-blocks
// grouped by row support, analogous to SparseLinearConstraint.
// Each row of C x = d becomes part of a per-clique [0 C_k'; C_k 0] block.
class SparseEqualityConstraint {
 public:
  SparseEqualityConstraint(const Eigen::SparseMatrix<double>& C,
                           const Eigen::VectorXd& d);

  struct RowGroup {
    Eigen::MatrixXd C;
    Eigen::VectorXd d;
    std::vector<int> primal_variables;
    std::vector<int> dual_variables;
    std::vector<int> global_rows;
  };

  const std::vector<std::vector<int>>& row_supports() const {
    return unique_supports_;
  }

  const Eigen::SparseMatrix<double>& C() const { return C_; }
  const Eigen::VectorXd& d() const { return d_; }

  // Assign each row to the smallest target support containing it,
  // then build dense sub-blocks with primal and dual variables.
  std::vector<RowGroup> GetConstraints(
      const std::vector<std::vector<int>>& target_supports,
      const std::vector<int>& row_to_dual) const;

 private:
  Eigen::SparseMatrix<double> C_;
  Eigen::VectorXd d_;

  struct SupportGroup {
    std::vector<int> support;
    std::vector<int> rows;
  };
  std::vector<SupportGroup> support_groups_;
  std::vector<std::vector<int>> unique_supports_;
};

// Assembler that decomposes sparse equality constraints into per-clique
// EqualityConstraint.  Each row of C gets a dual variable;
// the indefinite [0 C'; C 0] blocks are split across maximal cliques.
class SparseEqualityConstraintAssembler : public CliqueProvider {
 public:
  // primal_variables: column indices of C that are nonzero.
  // dual_variables: one per row of C (size == C.rows()).
  SparseEqualityConstraintAssembler(
      std::unique_ptr<SparseEqualityConstraint> sec,
      const std::vector<int>& primal_variables,
      const std::vector<int>& dual_variables);

  // Returns one clique per support group: {support ∪ dual vars}.
  std::vector<std::vector<int>> get_cliques() const override;

  bool is_positive_definite() const override { return false; }

  std::vector<SupernodalAssemblerBase*> Decompose(
      const std::vector<std::vector<int>>& maximal_cliques) override;

  const Eigen::SparseMatrix<double>& sparse_matrix() const {
    return sec_->C();
  }
  const Eigen::VectorXd& rhs_vector() const { return sec_->d(); }

 private:
  std::unique_ptr<SparseEqualityConstraint> sec_;
  std::vector<int> row_to_dual_;
  std::list<EqualityConstraint> owned_assemblers_;
};

}  // namespace conex
