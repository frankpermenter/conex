# Assembler Cleanup TODO

## Done
- [x] 3. Register sub-assembler BlockAssemblers with the tree solver
- [x] 4. Tree solver's generic interface iterates over registered sub-assemblers
- [x] 5. Row distribution handled by sub-assemblers (RowSpace in internal ordering)
- [x] 9. Remove Consolidate() from BuildInternal, multi-assembler path works

## Remaining (inheritance cleanup)
- [ ] 1. Split SupernodalAssemblerBase: construction-time CliqueProvider vs runtime
- [ ] 2. Move row_map and affine term — no longer needed on top-level assembler for runtime
- [ ] 6. Affine term already on LinearConstraint (sub-assembler); remove from SparseLinearConstraintAssembler
- [ ] 7. Remove LinearConstraint's inheritance from SupernodalAssemblerBase
- [ ] 8. Remove SparseLinearConstraintAssembler from runtime path (construction-only)
