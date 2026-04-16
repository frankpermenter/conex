#pragma once
#include <Eigen/Dense>
#include "conex/common/problem.h"

namespace conex {

// Maps solutions from the decomposed (expanded) problem back to the
// original variable space.  Splitting variables are discarded.
struct ChordalExpansion {
  int original_n;   // number of variables in the original problem
  int expanded_n;   // number of variables in the decomposed problem

  // Extract the original variables from a decomposed solution.
  Eigen::VectorXd Extract(const Eigen::VectorXd& x_expanded) const {
    return x_expanded.head(original_n);
  }

  // Was the problem actually decomposed?
  bool was_decomposed() const { return expanded_n > original_n; }
};

// Decompose a Problem's PSD constraints using chordal sparsity with
// separator splitting variables.
//
// For each PSD constraint with use_chordal=true:
//   1. Compute aggregate sparsity (union of A_i and B nonzero patterns).
//   2. Build a clique tree via min-degree elimination.
//   3. For each clique K with supernode N and separator S:
//      - Introduce |S|*(|S|+1)/2 splitting variables for the S×S block.
//      - Child clique gets: original A,B at N rows + splitting vars at S×S.
//      - Parent clique gets: -splitting vars at S×S positions.
//      - B constants assigned to leaf of each entry's subtree.
//   4. The resulting Problem has more variables but only small PSD blocks.
//
// The decomposition is EXACT (same optimal value) by the positive
// definite completion theorem + separator splitting.
//
// Non-PSD constraints and PSD constraints with use_chordal=false are
// copied unchanged.  The cost vector is extended with zeros for
// splitting vars.
//
// Returns (decomposed_problem, expansion) where expansion.Extract()
// maps solutions back to the original variable space.
std::pair<Problem, ChordalExpansion> DecomposeChordalPSD(
    const Problem& problem);

// Check if any PSD constraint in the problem requests chordal decomp.
bool HasChordalPSD(const Problem& problem);

// Combined preprocessor: chordal decomposition then rank reduction.
// Chains DecomposeChordalPSD (if needed) with RemoveStructuralRankDeficiency.
// Returns the fully preprocessed Problem plus a combined expansion that
// undoes both transforms: Extract splits → Expand restores dropped cols.
struct PreprocessResult {
  Problem problem;
  Expansion rank_expansion;
  ChordalExpansion chordal_expansion;

  // Map a solution of the preprocessed problem back to the original space.
  Eigen::VectorXd Expand(const Eigen::VectorXd& x_preprocessed) const {
    // Step 1: Expand rank-reduced → rank-full (with split vars still present).
    Eigen::VectorXd x_full = rank_expansion.Expand(x_preprocessed);
    // Step 2: Extract original variables (drop split vars).
    return chordal_expansion.Extract(x_full);
  }
};

PreprocessResult PreprocessProblem(const Problem& problem);

}  // namespace conex
