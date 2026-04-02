# Barrier QP Optimization TODOs

## 1. Redundant ScatterSeparators on x
`MultiplyA(x, row)` and `AccumulateQx(x, grad)` each call
`ScatterSeparators` on x when `blocks_fully_gathered=true`.
x doesn't change within a Newton step — scatter once, cache result.

**Fix**: Add a `ScatterSeparators(x)` call at the top of the Newton
step. Set `x.blocks_fully_gathered = false` so subsequent MultiplyA
and AccumulateQx skip the scatter. Or: add a dedicated
`PrepareForRead(TreeRHS&)` method that scatters once and flips the flag.

## 2. GatherSeparators on grad is wasted before solve
Line 72: `GatherSeparators(grad)` folds separator data into supernode
blocks. Then `dx = grad; dx *= -1; SolveTreeRHS(dx)` — the solve
zeros sep_scratch and doesn't use the folded data.

**Fix**: Skip `GatherSeparators(grad)`. Instead, `dx = grad` preserves
pending separators (now that `operator=` copies sep and preserves
`blocks_fully_gathered`). `dx *= -1` negates both blocks and sep.
`SolveTreeRHS(dx)` consumes sep directly. The `grad.dot(dx)` on
line 81 needs gathered data — compute it AFTER the solve using
`grad` (still ungathered) and `dx` (solved, gathered).

Actually: `grad.dot(dx)` requires both gathered. After solve, dx is
gathered. grad still has pending separators. So gather grad before
dot, or compute decrement differently.

Alternative: `lambda_sq = -(grad blocks) . (dx blocks)` is correct
when grad has been gathered. Keep the current GatherSeparators(grad)
for correctness, defer optimization.

## 3. Redundant AccumulateQx for objective
Lines 93-95: `AccumulateQx(x, qx_trial)` + `GatherSeparators(qx_trial)`
to compute `0.5 * x.dot(qx_trial)`. This is a SECOND Q*x computation
(the first was for the gradient on line 70). Q*x didn't change.

**Fix**: Reuse the Q*x from the gradient. `grad = c + Qx + A^T v`.
So `Qx = grad - c - A^T v`. But extracting Qx from grad requires
subtracting, which is messy. Alternative: compute Qx once, store it,
use for both gradient and objective.

## 4. Redundant ScatterSeparators in line search
Lines 106, 111: `MultiplyA(x_trial, row)` and
`AccumulateQx(x_trial, qx_trial)` each scatter x_trial. x_trial
changes per line search iteration, so the scatter is needed for
MultiplyA. But AccumulateQx re-scatters the same x_trial.

**Fix**: Same as #1 — scatter x_trial once per line search iteration.

## 5. MakeRowSpace allocations per Newton step
Lines 58, 64: `weights` and `scaled_inv_s` allocated every Newton step.

**Fix**: Allocate once outside the loop, reuse. `MakeRowSpace()` returns
a new RowSpace each time.

## 6. Dense VectorXd allocations
Line 54: `Eigen::VectorXd s = b - row.data` — allocates new vector.
Line 107: same for `s_new`.

**Fix**: Compute in-place: `row.data = b - row.data` or use a
pre-allocated scratch.

## Priority
Items 1 and 4 are the biggest wins (eliminate redundant tree traversals).
Item 5 is easy. Items 2, 3, 6 are minor.
