# GPU Tree Solver TODOs

## Solve path
- [ ] Scatter kernel for forward solve: separator variables are non-contiguous in the global solution vector. Need a small CUDA kernel to gather separator entries from `d_x` into a local buffer before `cublasDgemm`, and scatter the result back. Currently the forward/backward solve assumes contiguous layout.
- [ ] Backward solve separator update: `x_sn -= temp * x_sep` requires gathering `x_sep` from non-contiguous positions, multiplying by cached `L^{-1} S^T`, and scattering back into `x_sn`.

## Factorization performance
- [ ] Batched cuSOLVER for same-size supernodes within a level (`cusolverDnDpotrfBatched`). Currently each supernode is factored with an individual `cusolverDnDpotrf` call.
- [ ] Custom shared-memory kernel for small supernodes (sn_size <= 32). Kernel launch overhead dominates for tiny blocks — a single kernel that factors many small blocks in shared memory would be faster.
- [ ] Persistent cuSOLVER workspace: currently allocates/frees `d_work` per supernode per level. Pre-allocate once for the largest supernode.
- [ ] Stream concurrency: independent subtrees at the same level could use separate streams for overlap.

## Extend-add
- [ ] Merge contiguous ScatterOps: currently emits one op per entry pair (block_size=1). Detecting contiguous ranges in the parent and merging into larger block copies would reduce kernel launch overhead and improve coalescing.
- [ ] Benchmark atomicAdd vs segmented scatter for the case where multiple children write to the same parent.

## Assembly
- [ ] Device-side assembly path: allow contributors to write directly to device memory, skipping the host staging + H2D copy. Requires extending `SupernodalAssemblerBase` with device pointers.
- [ ] Async assembly: overlap H2D copies with factorization of already-assembled levels.

## KKTMatrix
- [ ] Implement `DoKKTMatrix`: download per-supernode blocks from device and reconstruct the full assembled matrix. Low priority (only used for debugging).

## Integration
- [ ] Factory function: `MakeGpuTreeSolver(ConstraintManager*, SolverConfiguration*)` that mirrors `MakeTreeSolver` but returns a `GpuTreeSolver`. Reuses the CPU symbolic analysis (clique ordering, Decompose) then hands off numeric data to the GPU solver.
- [ ] Threshold heuristic: auto-select GPU vs CPU based on problem size (total supernode volume, number of levels). GPU overhead isn't worth it for small problems.
- [ ] Test against CPU solver: verify `||x_gpu - x_cpu|| / ||x_cpu|| < tol` on the benchmark MTX matrices.

## Compiler warnings
- [ ] Remove unused variables (`total`, `zero`, `neg_one`) in gpu_tree_solver.cu.

## Build
- [x] Compiles with CUDA 12.0 (verified on Ubuntu 24.04, no GPU device).
- [x] Gated behind `check_language(CUDA)` — CPU build unaffected.
- [x] Unit test binary builds (`gpu_tree_solver_test`). Needs GPU to run.
- [ ] CI with CUDA: add a GPU build job that compiles and runs the GPU solver tests.
- [ ] Min compute capability: document requirement (>= 6.0 for `atomicAdd(double*)`).
