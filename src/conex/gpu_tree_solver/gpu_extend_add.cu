#include "conex/gpu_tree_solver/gpu_extend_add.cuh"

#include <cuda_runtime.h>
#include <stdexcept>

namespace conex {

// Each thread block handles one ScatterOp.
// Threads within the block cooperatively copy the block_size x block_size
// sub-block from source to destination using atomicAdd (multiple children
// may scatter into the same parent).
__global__ void ExtendAddKernel(const ScatterOp* ops, int num_ops) {
  int op_idx = blockIdx.x;
  if (op_idx >= num_ops) return;

  const ScatterOp& op = ops[op_idx];
  const int bs = op.block_size;

  // 2D thread indexing within the block.
  for (int col = threadIdx.y; col < bs; col += blockDim.y) {
    for (int row = threadIdx.x; row < bs; row += blockDim.x) {
      double val = op.src_ptr[(op.src_col + col) * op.src_ld + (op.src_row + row)];
      atomicAdd(&op.dst_ptr[(op.dst_col + col) * op.dst_ld + (op.dst_row + row)],
                val);
    }
  }
}

void LaunchExtendAdd(const ScatterOp* d_ops, int num_ops, void* stream) {
  if (num_ops == 0) return;

  // Use 16x16 thread blocks — handles sub-blocks up to 16x16 without looping,
  // and loops for larger blocks.
  dim3 threads(16, 16);
  dim3 blocks(num_ops);

  ExtendAddKernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
      d_ops, num_ops);

  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("ExtendAdd kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

// Gather kernel: dst[i + col*n] = src[indices[i] + col*src_ld].
__global__ void GatherKernel(double* dst, const double* src,
                             const int* indices, int n, int cols,
                             int src_ld) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  int idx = indices[i];
  for (int c = 0; c < cols; ++c) {
    dst[i + c * n] = src[idx + c * src_ld];
  }
}

void LaunchGather(double* dst, const double* src, const int* indices,
                  int n, int cols, int src_ld, void* stream) {
  if (n == 0) return;
  int threads = 256;
  int blocks = (n + threads - 1) / threads;
  GatherKernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
      dst, src, indices, n, cols, src_ld);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("Gather kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

// Scatter kernel: dst[indices[i] + col*dst_ld] = src[i + col*n].
__global__ void ScatterKernel(const double* src, double* dst,
                              const int* indices, int n, int cols,
                              int dst_ld) {
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  int idx = indices[i];
  for (int c = 0; c < cols; ++c) {
    dst[idx + c * dst_ld] = src[i + c * n];
  }
}

void LaunchScatter(const double* src, double* dst, const int* indices,
                   int n, int cols, int dst_ld, void* stream) {
  if (n == 0) return;
  int threads = 256;
  int blocks = (n + threads - 1) / threads;
  ScatterKernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
      src, dst, indices, n, cols, dst_ld);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("Scatter kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

// Batched gather: thread tid handles one (batch_element, entry) pair.
// dst layout: [batch_0: n entries][batch_1: n entries]...
// For cols > 1: dst[tid + c * total] = src[idx + c * src_ld].
__global__ void BatchGatherKernel(double* dst, const double* src,
                                  const int* all_indices, const int* offsets,
                                  int n, int batch_size, int cols,
                                  int src_ld) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int total = batch_size * n;
  if (tid >= total) return;
  int b = tid / n;
  int k = tid % n;
  int idx = all_indices[offsets[b] + k];
  for (int c = 0; c < cols; ++c) {
    dst[tid + c * total] = src[idx + c * src_ld];
  }
}

void LaunchBatchGather(double* dst, const double* src,
                       const int* all_indices, const int* d_offsets,
                       int n, int batch_size, int cols, int src_ld,
                       void* stream) {
  int total = batch_size * n;
  if (total == 0) return;
  int threads = 256;
  int blocks = (total + threads - 1) / threads;
  BatchGatherKernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
      dst, src, all_indices, d_offsets, n, batch_size, cols, src_ld);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("BatchGather kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

// Batched scatter: reverse of batched gather.
__global__ void BatchScatterKernel(const double* src, double* dst,
                                   const int* all_indices, const int* offsets,
                                   int n, int batch_size, int cols,
                                   int dst_ld) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int total = batch_size * n;
  if (tid >= total) return;
  int b = tid / n;
  int k = tid % n;
  int idx = all_indices[offsets[b] + k];
  for (int c = 0; c < cols; ++c) {
    dst[idx + c * dst_ld] = src[tid + c * total];
  }
}

void LaunchBatchScatter(const double* src, double* dst,
                        const int* all_indices, const int* d_offsets,
                        int n, int batch_size, int cols, int dst_ld,
                        void* stream) {
  int total = batch_size * n;
  if (total == 0) return;
  int threads = 256;
  int blocks = (total + threads - 1) / threads;
  BatchScatterKernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
      src, dst, all_indices, d_offsets, n, batch_size, cols, dst_ld);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("BatchScatter kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

// Batched atomic-add scatter for overlapping indices.
__global__ void BatchScatterAddKernel(const double* src, double* dst,
                                      const int* all_indices,
                                      const int* offsets, int n,
                                      int batch_size, int cols, int dst_ld) {
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  int total = batch_size * n;
  if (tid >= total) return;
  int b = tid / n;
  int k = tid % n;
  int idx = all_indices[offsets[b] + k];
  for (int c = 0; c < cols; ++c) {
    atomicAdd(&dst[idx + c * dst_ld], src[tid + c * total]);
  }
}

void LaunchBatchScatterAdd(const double* src, double* dst,
                           const int* all_indices, const int* d_offsets,
                           int n, int batch_size, int cols, int dst_ld,
                           void* stream) {
  int total = batch_size * n;
  if (total == 0) return;
  int threads = 256;
  int blocks = (total + threads - 1) / threads;
  BatchScatterAddKernel<<<blocks, threads, 0,
                          static_cast<cudaStream_t>(stream)>>>(
      src, dst, all_indices, d_offsets, n, batch_size, cols, dst_ld);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string("BatchScatterAdd kernel launch: ") +
                             cudaGetErrorString(err));
  }
}

}  // namespace conex
