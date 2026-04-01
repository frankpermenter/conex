# Assembler Cleanup TODO

## 1. Split SupernodalAssemblerBase into two roles
- `CliqueProvider` (construction-time): `get_cliques()`, `Decompose()`, `variables()`, `is_positive_definite()`, `is_dynamic()`
- Sub-assemblers only need `BlockAssembler` + `primal_variables()` + `dual_variables()`

## 2. Move row_map and affine term into BlockAssembler
- Currently `row_map()`, `GetAffineTerm()`, `num_global_rows()`, `SetWeights()` live on `SparseLinearConstraintAssembler` (top-level)
- The tree solver's generic interface should access these per sub-assembler, not per top-level assembler

## 3. Register sub-assembler BlockAssemblers with the tree solver
- `RegisterAssembler(BlockAssembler*, ContributionType)` -- single method, caller provides PD/indefinite tag
- Replace `RegisterLinearAssembler` / `RegisterQuadraticAssembler` with unified registration
- Tree solver stores `vector<pair<BlockAssembler*, ContributionType>>`

## 4. Tree solver's generic interface iterates over registered BlockAssemblers
- `MultiplyA`: loop over linear BlockAssemblers, call `MultiplyA` on each GramEvaluator
- `AccumulateAtranspose`: loop over linear BlockAssemblers, call `ContributeAtranspose` on each
- `AccumulateQx`: loop over PD cost BlockAssemblers, call `MultiplyQx` on each
- `SetWeights`: loop over linear BlockAssemblers, set weights per sub-assembler
- `MakeRowSpace`: sum row counts from linear BlockAssemblers

## 5. Move row distribution into the tree solver or RowSpace
- Tree solver builds row_map from sub-assemblers' row counts at registration time
- `MultiplyA` writes each sub-assembler's output into the correct RowSpace segment
- `AccumulateAtranspose` reads each sub-assembler's input from the correct segment

## 6. Move affine term to BlockAssembler
- Add `affine_term()` to BlockAssembler (or LinearBlockAssembler)
- `GetAffineTerm` on tree solver gathers from each sub-assembler
- Remove `GetAffineTerm` from `SparseLinearConstraintAssembler`

## 7. Remove LinearConstraint's inheritance from SupernodalAssemblerBase
- `LinearConstraint` becomes: GramEvaluator + constraint_matrix_ + constraint_affine_ + primal_variables_
- Same for `DenseQuadraticTermSubAssembler`

## 8. Remove SparseLinearConstraintAssembler from the runtime path
- It becomes construction-only: `Decompose()` -> sub-assemblers, then done
- `row_map`, `SetWeights`, `GetAffineTerm`, `ComputeTransposeProduct` move to tree solver

## 9. Fix the multi-assembler crash and remove Consolidate() workaround
- With sub-assemblers registered directly, any number of constraints/costs works
- Remove `Consolidate()` from `BuildInternal`
- `ConsolidateMultipleConstraints` test passes via the direct path
