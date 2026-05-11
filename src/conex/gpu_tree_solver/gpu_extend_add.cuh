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

// Gather: dst[i, :] = src[indices[i], :] for i in [0, n).
// src has leading dimension src_ld, dst has leading dimension n.
// Both are column-major with `cols` columns.
void LaunchGather(double* dst, const double* src, const int* indices,
                  int n, int cols, int src_ld, void* stream);

// Scatter: dst[indices[i], :] = src[i, :] for i in [0, n).
// dst has leading dimension dst_ld, src has leading dimension n.
void LaunchScatter(const double* src, double* dst, const int* indices,
                   int n, int cols, int dst_ld, void* stream);

// Batched gather: for batch element b, gather n entries from src using
// indices at all_indices[offsets[b] .. offsets[b]+n-1] into
// dst[b*n .. b*n+n-1].  Total threads = batch_size * n.
void LaunchBatchGather(double* dst, const double* src,
                       const int* all_indices, const int* d_offsets,
                       int n, int batch_size, int cols, int src_ld,
                       void* stream);

// Batched scatter: reverse of batched gather.
void LaunchBatchScatter(const double* src, double* dst,
                        const int* all_indices, const int* d_offsets,
                        int n, int batch_size, int cols, int dst_ld,
                        void* stream);

// Batched atomic-add scatter: dst[indices[offsets[b]+k]] += src[b*n+k].
// Use when multiple batch elements write to overlapping positions.
void LaunchBatchScatterAdd(const double* src, double* dst,
                           const int* all_indices, const int* d_offsets,
                           int n, int batch_size, int cols, int dst_ld,
                           void* stream);

}  // namespace conex
