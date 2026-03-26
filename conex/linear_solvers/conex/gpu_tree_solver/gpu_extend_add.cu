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

}  // namespace conex
