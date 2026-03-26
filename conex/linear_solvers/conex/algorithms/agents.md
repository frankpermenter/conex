# Block-Space API for Iterative Algorithms

This directory contains algorithms (IRLS, barrier QP) built on top of the
tree solver. The solver is a pure linear algebra layer — it knows nothing
about convergence, barriers, or iteration. Algorithms interact through
a small public API.

## Core Loop

Build the solver once, then iterate:

```cpp
// Setup (once):
auto solver = MakeTreeSolver(&cm, config);
auto* tree_solver = dynamic_cast<SymmetricLinearSystemTreeSolver*>(solver.get());
solver->AssembleAndFactor();            // initializes evaluators
a_assembler->BindPartition(*tree_solver); // maps constraints to blocks

// Iteration:
a_assembler->SetWeights(weights);       // update W (per-row)
solver->AssembleAndFactor();            // re-assembles A^T W A, re-factors
Eigen::VectorXd dx = solver->Solve(rhs);
```

## Block Space

After `Solve`, the solution lives in the `SupernodePartitionMatrix` — a
collection of per-clique dense blocks (supernode + separator). You can
compute residuals and products without leaving this representation.

### Scatter / Gather

```cpp
tree_solver->ScatterToBlocks(x);     // original-order vector → per-block storage
tree_solver->GatherFromBlocks(x);    // per-block storage → original-order vector
```

### A*x (residual)

```cpp
tree_solver->ScatterToBlocks(x);
Eigen::VectorXd Ax = a_assembler->ComputeBlockResiduals(*tree_solver);
```

Uses `A_perm_` (columns permuted to elimination order) directly on contiguous
block data. No per-variable gather. Dense BLAS on each clique's sub-block.

**Requires** `BindPartition` after the first `AssembleAndFactor`.

### A^T * v

```cpp
Eigen::VectorXd Atv = a_assembler->ComputeTransposeProduct(v);
```

Per-clique dense `A_clique^T * v_local`, scattered to global result.

### Q * x

```cpp
Eigen::VectorXd Qx = q_assembler->ComputeBlockProduct(*tree_solver);
```

Currently gathers to global then uses sparse Q. Per-clique Q_perm_ path
is available but not yet wired end-to-end.

### SetWeights

```cpp
a_assembler->SetWeights(weights);   // size = A.rows()
```

Distributes to per-clique `LinearConstraint` W vectors (stores `sqrt(w)`
since the evaluator computes `(WA)^T(WA) = A^T W^2 A`). Calls
`update_weights()` on each `GramEvaluator`.

## Block Structure

Each clique in the tree has:
- **Supernode block**: dense n_sn × n_sn matrix (factored in-place)
- **Separator rows**: dense n_sep × n_sn off-diagonal
- **Separator Schur complement**: dense n_sep × n_sep

The `A_perm_` for each constraint has columns ordered as
`[supernode_cols | separator_cols]`, matching the partition layout.
The `sn_count` on the `GramEvaluator` tracks the split point.

## What Lives Where

```
algorithms/         ← iterative algorithms (this directory)
  irls.cc           ← IRLS for L1 minimization
  barrier_qp.cc     ← barrier method for Ax <= b QP

common/             ← constraint types, assemblers, decomposition
  sparse_linear_constraint.h  ← SetWeights, ComputeResiduals, ComputeBlockResiduals
  sparse_quadratic_term.h     ← Q assembler, ComputeBlockProduct
  linear_constraint.h         ← per-clique LinearConstraint, ComputeBlockResidual

tree_solver/        ← factorization, solve, block partition
  kkt_solver_factory.h  ← MakeTreeSolver (entry point)
  kkt_tree_solver.h     ← partition(), ScatterToBlocks, GatherFromBlocks
```

Algorithms depend on `common/` and `tree_solver/`. The tree solver has
zero imports from `algorithms/`.

## Performance

Block-space A*x is 2-3x faster than sparse A*x for problems with dense
cliques (block size ≥ 20). The speedup comes from dense BLAS on contiguous
memory vs sparse column iteration with indirect indexing.

For small cliques or highly sparse A, the sparse matvec is faster.
Use `ComputeResiduals(x)` (gather path) for those cases.
