# GPU Tree Solver TODOs

## Current performance (block-arrow, RTX 5000 Ada)

| Problem | GPU tree | cuDSS | cuSOLVER-Sp | CPU |
|---|---|---|---|---|
| 100×100 blk-diag n=10k | factor 2.2ms, solve 0.6ms | 0.7+0.2ms | 2.4+1.5ms | 233+0.8ms |
| 50×100+sep20 arrow n=5k | factor 2.3ms, solve 3.1ms | 0.6+0.2ms | 17+1.5ms | 84+0.6ms |
| 5×400+sep20 arrow n=2k | factor 5.5ms, solve 2.2ms | 3.0+0.7ms | 13+3.9ms | 74+0.7ms |

## Solve path
- [x] Gather/scatter kernels for non-contiguous separator variables.
- [x] Batched trsm (`cublasDtrsmBatched`) for forward/backward solve.
- [ ] Batched separator gemm: siblings sharing separator positions need a reduction-based approach (atomicAdd gives ~3% error from non-deterministic ordering). Currently sequential gather-gemm-scatter — dominates solve time for block-arrow (3.1ms of 5.4ms total at 50×100+sep20).

## Factorization
- [x] Batched Cholesky: `cusolverDnDpotrfBatched` + `cublasDtrsmBatched` + `cublasDgemmBatched` for same-size supernodes. 20-56× speedup over sequential.
- [x] Persistent cuSOLVER workspace, batched info check.
- [ ] Custom shared-memory Cholesky for small supernodes (sn_size ≤ 32). Kernel launch overhead still significant at this size.
- [ ] Stream concurrency: independent subtrees could use separate streams.

## Extend-add
- [ ] Merge contiguous ScatterOps: currently one op per entry pair (block_size=1, 16×16 threads to copy one double). Detecting contiguous ranges would reduce kernel launches.

## Assembly
- [ ] Device-side assembly: write directly to device memory, skip H2D staging.
- [ ] Async assembly: overlap H2D copies with factorization of already-assembled levels.

## Integration
- [ ] `MakeGpuTreeSolver(ConstraintManager*, SolverConfiguration*)` factory function.
- [ ] Auto-select GPU vs CPU based on problem size.
- [ ] Test against CPU solver on SuiteSparse MTX matrices.

## Build
- [x] CUDA 12.2 with Ada (sm_89). Auto-detects `/usr/local/cuda-12.2`.
- [x] Gated behind `check_language(CUDA)` — CPU build unaffected.
- [x] Benchmark: `gpu_benchmark_cudss` compares GPU tree vs cuDSS vs cuSOLVER-Sp vs CPU.
- [ ] CI with CUDA.
