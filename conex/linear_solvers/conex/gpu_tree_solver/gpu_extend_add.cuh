#pragma once

namespace conex {

// One block-to-block scatter operation for the extend-add kernel.
// Copies a sub-block from a child's separator Schur complement
// into a parent's supernode, separator_rows, or separator_schur.
struct ScatterOp {
  double* src_ptr;   // device pointer to source block
  double* dst_ptr;   // device pointer to destination block
  int src_row;       // starting row in source
  int src_col;       // starting col in source
  int dst_row;       // starting row in destination
  int dst_col;       // starting col in destination
  int block_size;    // size of the sub-block (square)
  int src_ld;        // leading dimension of source
  int dst_ld;        // leading dimension of destination
};

// Launch the extend-add kernel for a batch of scatter operations.
// All operations in the batch are independent (same tree level).
// stream: CUDA stream for async execution.
void LaunchExtendAdd(const ScatterOp* d_ops, int num_ops, void* stream);

}  // namespace conex
