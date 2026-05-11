// SDPA-sparse format reader: converts SDP benchmarks to Model.
//
// Format: min <C, X> s.t. <A_i, X> = b_i, X ≽ 0.
//
// File structure:
//   Line 1: m (number of constraints)
//   Line 2: nblocks (number of blocks)
//   Line 3: block sizes (negative = diagonal block)
//   Line 4: b vector (m entries)
//   Remaining: triplets (constraint, block, row, col, value)
//     constraint=0 → objective C
//     constraint=1..m → constraint A_i
//
// We convert to: max b^T y s.t. C - Σ y_i A_i ≽ 0
// which is: AddPSDConstraint([-A_1, ..., -A_m], C, vars)
// with cost = -b (minimize -b^T y = maximize b^T y).

#pragma once
#include <string>
#include "conex/common/model.h"

namespace conex {

struct SDPAInfo {
  int num_constraints = 0;
  int num_blocks = 0;
  std::vector<int> block_sizes;
  int total_matrix_dim = 0;
};

std::pair<Model, SDPAInfo> ReadSDPA(const std::string& filename);

}  // namespace conex
