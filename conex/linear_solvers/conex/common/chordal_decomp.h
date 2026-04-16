#pragma once
#include "conex/common/problem.h"

namespace conex {

// Decompose a Problem's PSD constraints using chordal sparsity with
// separator splitting variables.
//
// For each PSD constraint Σ A_i x_i + B ≽ 0:
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
// Non-PSD constraints (linear, SOC, quadratic, equality) are copied
// unchanged.  The cost vector is extended with zeros for splitting vars.
Problem DecomposeChordalPSD(const Problem& problem);

}  // namespace conex
