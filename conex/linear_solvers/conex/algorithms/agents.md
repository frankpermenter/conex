# Algorithms Guide

This directory contains iterative algorithms built on the solver abstraction.
The solver is a pure linear algebra layer — algorithms interact through
`KKTSolverBase` and `ConstraintManager`, not solver internals.

## Available Algorithms

| File | Algorithm | Problem |
|------|-----------|---------|
| `least_squares.cc` | Direct solve | `min \|Ax - b\|_2^2` and `(Q + A^T A)x = rhs` |
| `irls.cc` | IRLS | `min \|Ax - b\|_1` (L1 minimization) |
| `barrier_qp.cc` | Log-barrier IPM | `min 0.5 x^T Q x + c^T x` s.t. `Ax <= b` |

## Solver Abstraction

All algorithms use `KKTSolverBase` (not the tree solver directly).
Two implementations exist:

- **`SymmetricLinearSystemTreeSolver`** — supernodal chordal sparse Cholesky
- **`DenseKKTSolver`** — dense LLT (reference / small problems)
- **`GpuTreeSolver`** — GPU version of tree solver (requires CUDA)

## Standard Pattern

```cpp
// 1. Register constraints.
ConstraintManager cm(n);
cm.AddCustomAssembler(std::make_unique<SparseLinearConstraintAssembler>(...));
cm.Preprocess();  // drops structurally rank-deficient columns

// 2. Build solver (currently always a tree solver).
auto solver = MakeTreeSolver(&cm, config);

// 3. Optionally bind partition for fast block-space residuals.
solver->AssembleAndFactor();
if (auto* tree = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.get()))
    assembler->BindPartition(*tree);

// 4. Iterative loop.
assembler->SetWeights(weights);
solver->AssembleAndFactor();
Eigen::VectorXd dx = solver->Solve(rhs);

// 5. Expand solution if Preprocess reduced the problem.
result.x = cm.ExpandSolution(x);
```

## ConstraintManager::Preprocess

Call after registering all assemblers, before building the solver.
Checks structural rank of all `SparseLinearConstraintAssembler`s.
If rank-deficient, drops columns and rebuilds assemblers in reduced space.

```cpp
cm.Preprocess();
// After Preprocess:
cm.was_reduced()          // true if columns were dropped
cm.GetNumberOfVariables() // reduced count
cm.column_map()           // reduced_col -> original_col
cm.ExpandSolution(x)      // zero-pads dropped variables
cm.ReduceVector(v)        // slices to kept variables
```

For barrier QP with a Q matrix, reduce Q and c manually:
```cpp
cm.Preprocess();
Eigen::VectorXd c_r = cm.ReduceVector(c);
// Reduce Q using cm.column_map() ...
// Add Q assembler AFTER Preprocess with reduced variables.
```

## Block Partition

Every solver provides a `BlockPartition` via `solver->partition()`.
This abstracts the per-block decomposition of the solution vector.

```cpp
solver->ScatterToBlocks(x);     // vector → blocks
solver->GatherFromBlocks(x);    // blocks → vector
auto& p = solver->partition();
p.num_blocks();                  // tree: num supernodes, dense: 1
p.block(k);                      // Eigen::Ref to block k
```

## Residuals and Products (Block Space)

After solving, compute residuals without gathering to a global vector:

```cpp
// A * x (block-space, fast for large dense cliques):
solver->ScatterToBlocks(x);
Eigen::VectorXd Ax = assembler->ComputeBlockResiduals(*solver);

// A^T * v (per-clique dense A^T multiply):
Eigen::VectorXd Atv = assembler->ComputeTransposeProduct(v);

// Q * x (gathers globally, uses sparse Q):
Eigen::VectorXd Qx = q_assembler->ComputeBlockProduct(*solver);
```

`ComputeBlockResiduals` takes `const KKTSolverBase&` — works with any solver.
If `BindPartition` was called (tree solver only), uses the fast path with
`A_perm_` on contiguous supernode/separator blocks. Otherwise falls back
to gather + sparse matvec.

## Dependency Structure

```
algorithms/         depends on  common/, tree_solver/ (for MakeTreeSolver)
  irls.cc
  barrier_qp.cc
  least_squares.cc

common/             no dependency on tree_solver/
  constraint_manager.h    ← Preprocess, ExpandSolution, ReduceVector
  kkt_solver_interface.h  ← KKTSolverBase (abstract)
  block_partition.h       ← BlockPartition (abstract)
  sparse_linear_constraint.h ← assemblers, SetWeights, residuals
  clique_tree.h           ← CliqueTree (data struct, no solver logic)

tree_solver/        implements KKTSolverBase
  kkt_tree_solver.h  ← SymmetricLinearSystemTreeSolver
  kkt_solver_factory.h ← MakeTreeSolver

gpu_tree_solver/    implements KKTSolverBase (requires CUDA)
  gpu_tree_solver.h  ← GpuTreeSolver
```

`common/` has zero imports from `tree_solver/` or `gpu_tree_solver/`.
Algorithms import `tree_solver/` only for `MakeTreeSolver`.

## Adding a New Algorithm

1. Create `conex/algorithms/my_algo.h` and `.cc`.
2. Use `ConstraintManager` + `MakeTreeSolver` to build the solver.
3. Call `cm.Preprocess()` before building the solver.
4. Use `SetWeights` + `AssembleAndFactor` + `Solve` loop.
5. Return `cm.ExpandSolution(x)` if the problem might be rank-deficient.
6. Add to `CMakeLists.txt` in the `algorithms` library.
7. Add tests in `conex/test/algorithms_test.cc`.
