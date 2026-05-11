# Algorithms Guide

This directory contains algorithms and solver construction utilities built
on the tree solver.  There are two paths for building a solver:

1. **Automatic** — `ConstraintManager` + `MakeTreeSolver` discovers the
   clique tree via AMD on the sparse matrix.
2. **Custom** — `TreeSolverBuilder` constructs the clique tree directly
   from a known problem structure (LQR chain, scenario tree, etc.).

## Available Algorithms

| File | Entry Point | Problem |
|------|-------------|---------|
| `least_squares.cc` | `SparseLeastSquares` | min ‖Ax - b‖² |
| `least_squares.cc` | `SparseQuadraticTermLeastSquares` | (Q + A'A)x = rhs |
| `equality_constrained_least_squares.cc` | `EqualityConstrainedLeastSquares` | min ‖Ax - b‖² s.t. Cx = d |
| `irls.cc` | `SolveIRLS` | min ‖Ax - b‖₁ (L1 via IRLS) |
| `barrier_qp.cc` | `SolveBarrierQP` | min 0.5 x'Qx + c'x s.t. Ax ≤ b |
| `geodesic_ipm.cc` | `SolveGeodesicLP`, `SolveGeodesicBarrierLP`, `SolveGeodesicThetaContinuation`, `SolveGeodesicBarrierThetaContinuation`, `SolveGeodesicHSD`, `SolveGeodesicHybrid`, `SolveGeodesicPhaseOne` | Geodesic IPM for conic programs (LP/SDP/SOC/QP, symmetric and barrier cones) |
| `geodesic_hsde.cc` | `SolveGeodesicHSDE` | HSDE with scalar k, 2x2 (d_tau, theta) solve |
| `geodesic_hybrid_r.cc` | `SolveGeodesicThetaContinuationR`, `SolveGeodesicHybridR` | Theta-continuation and hybrid with per-component r-updates |
| `alternating_projections.cc` | `SolveAlternatingProjections` | Alternating projections for feasibility |
| `self_dual_embedding.cc` | `SolveSelfDualEmbedding` | HSD self-dual embedding |
| `solve_lp.cc` | `SolveLP` | Classical barrier method LP solver |
| `solve_strategies.h` | `GeodesicLP`, `GeodesicBarrierLP`, `ThetaContinuation`, etc. | Strategy structs wrapping algorithm entry points |
| `finite_horizon.cc` | `SolveLQRFromSparseMatrices` | LQR via sparse matrices + clique ordering |
| `lqr_tree_solver.cc` | `LQRTreeSolver` | LQR via direct chain tree construction |
| `tree_solver_builder.cc` | `TreeSolverBuilder` | Declarative tree solver construction |

## Solver Construction Paths

### Path 0: Problem + Solver (preferred)

```cpp
Problem p;
auto c1 = p.AddLinearConstraint(A, b, vars);
auto c2 = p.AddQuadraticCost(Q, vars);
auto [reduced, expansion] = Preprocess(p);  // optional
auto solver = Solver::Build(reduced);
solver.AssembleAndFactor();
auto x = solver.MakeBlockVariable();
auto rhs = solver.MakeBlockVariable(rhs_dense);
solver.SolveInto(rhs, x);
auto result = expansion.Expand(x.Gather());
```

With custom tree:
```cpp
TreeSpec tree;
int root = tree.AddClique();
int child = tree.AddClique(root);
tree.Assign(c1, child);
tree.Assign(c2, root);
auto solver = Solver::Build(p, tree);
```

### Path 1: Automatic (ConstraintManager) — deprecated

```cpp
ConstraintManager cm(n);
cm.AddCustomAssembler(std::make_unique<SparseLinearConstraintAssembler>(...));
cm.Preprocess();  // drops structurally rank-deficient columns
auto solver = MakeTreeSolver(&cm, config);
solver->AssembleAndFactor();
auto x = solver->Solve(rhs);
result = cm.ExpandSolution(x);
```

### Path 2: Custom (TreeSolverBuilder)

```cpp
TreeSolverBuilder b;
int root = b.AddClique();
int child = b.AddClique(root);
b.AddCost(child, Q, vars);                        // PD: Q block
b.AddLinearConstraint(child, A, b, vars);          // PD: A'A block
b.AddEquality(child, C, d, primal_vars, dual_vars); // indefinite: [0,C';C,0]
auto result = b.Build();
result.solver->AssembleAndFactor();
auto x = result.solver->Solve(rhs);
```

The builder automatically computes supernodes/separators from the
parent-child relationships and variable overlap.  If all parents are
unspecified, it runs weighted AMD on the quotient graph of clique
intersections to find an elimination tree automatically.

### Path 3: Static convenience

```cpp
auto result = TreeSolverBuilder::BuildFromSparseMatrices(Q, C, d);
```

Constructs sparse assemblers and runs variable-level AMD internally.

## Key Classes

### Assemblers (provide data to the tree solver)

| Class | File | Assembles | PD? |
|-------|------|-----------|-----|
| `SparseLinearConstraintAssembler` | `sparse_linear_constraint.h` | A'W²A | yes |
| `SparseQuadraticTermAssembler` | `sparse_quadratic_term.h` | sparse Q | yes |
| `SparseEqualityConstraintAssembler` | `sparse_equality_constraint.h` | sparse [0,C';C,0] | no |
| `SupernodalAssemblerEqualities` | `equality_constraint.h` | dense [0,C';C,0] | no |
| `DenseQuadraticTermSubAssembler` | `sparse_quadratic_term.h` | dense Q block | yes |
| `LinearConstraint` | `linear_constraint.h` | dense A'W²A | yes |

### Block Assembly Protocol

All assemblers provide a `BlockAssembler` (renamed from `LazySymmetricMatrix`)
via `GetBlockAssembler()`.  The tree solver uses a two-phase protocol:

1. **Register** (once at Finalize): the `BlockAssembler` receives permutation,
   block destinations (raw pointers + strides), and precomputes its internal
   layout.
2. **Assemble** (each AssembleAndFactor): `ContributeBlocks(clique_id)` writes
   all blocks using saved destinations.  No per-call block info computation.

Implementations: `GramEvaluator` (A'W²A with deferred weights),
`EqualityLazyMatrix`, `DenseQuadraticTermLazyEvaluator`, `DensePSDLazyEvaluator`.

### Infrastructure

| Class | File | Purpose |
|-------|------|---------|
| `ArenaAllocatable` | `arena_allocatable.h` | Base for arena-allocated workspace |
| `BlockContribution` | `supernodal_assembler_base.h` | Block dest/size/stride descriptor |
| `EliminationOrdering` | `clique_ordering.h` | Phase 1 output (order, later sets) |

## Clique Ordering

Split into two phases:

- **Phase 1** (`MakeCliqueTreeMinDegreeFromRowSupports`): bitset AMD on the
  variable graph.  `delayed_variables` (any variable without PD diagonal
  contribution) are eliminated after their neighbors.
- **Phase 2** (`MakeCliqueTreeFromEliminationOrdering`): supernode detection,
  tree construction, merging, post-order.  Can be called with a user-provided
  elimination ordering (e.g., from quotient AMD).

## ConstraintManager::Preprocess

Handles structurally rank-deficient columns (SLC assemblers) and
structurally redundant equality rows (with consistency checking).

```cpp
cm.Preprocess();
cm.was_reduced()          // true if columns were dropped
cm.GetNumberOfVariables() // reduced count
cm.ExpandSolution(x)      // zero-pads dropped variables
cm.ReduceVector(v)        // slices to kept variables
```

## Dependency Structure

```
algorithms/              depends on common/, linear_solvers/
  least_squares.cc
  equality_constrained_least_squares.cc
  irls.cc
  barrier_qp.cc
  geodesic_ipm.cc
  geodesic_hsde.cc
  geodesic_hybrid_r.cc
  alternating_projections.cc
  self_dual_embedding.cc
  solve_lp.cc
  solve_strategies.h
  finite_horizon.cc
  lqr_tree_solver.cc
  tree_solver_builder.cc

common/                  no dependency on linear_solvers/
  constraint_manager.h       Preprocess, ExpandSolution, ReduceVector
  supernodal_assembler_base.h  BlockAssembler, BlockContribution
  arena_allocatable.h        ArenaAllocatable base class
  sparse_linear_constraint.h SparseLinearConstraintAssembler
  sparse_equality_constraint.h SparseEqualityConstraintAssembler
  sparse_quadratic_term.h    SparseQuadraticTermAssembler
  equality_constraint.h      SupernodalAssemblerEqualities
  linear_constraint.h        LinearConstraint, GramEvaluator
  clique_ordering.h          EliminationOrdering, Phase 1 + Phase 2
  structural_rank.h          DropStructurallyDependentColumns/Rows

linear_solvers/             implements KKTSolverBase
  kkt_tree_solver.h      SymmetricLinearSystemTreeSolver
  kkt_solver_factory.h   MakeTreeSolver

gpu_linear_solvers/         implements KKTSolverBase (requires CUDA)
  gpu_tree_solver.h      GpuTreeSolver
```

## TODOs

- Merge `TreeSolverBuilder` and `ConstraintManager` into a single class.
- Deduplicate `DensePSDLazyEvaluator` (identical to `DenseQuadraticTermLazyEvaluator`).
- Remove legacy `add_block`/`add_block_lower` path from `BlockAssembler`
  (all evaluators now use `RegisterContributions`/`ContributeBlocks`).
