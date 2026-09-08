# RB-03 mapping characterization and exact-selector regression

From the PolyFEM root, with an existing Makefiles build of PolyFEM_bin and
unit_tests:

```sh
python3 tools/rb03/run_probe.py --build build --output /absolute/fresh/evidence
```

The runner compiles this standalone real-library probe into the fresh directory,
using the unit-test compiler/link recipe. It does not rebuild or alter production
binaries, and does not run scenes. It saves source, build commands, hashes,
stdout/stderr and exit status. The output directory must not already exist.
Generated OBJ/HDF5 fixtures are public synthetic data. The probe explicitly
supplies the transformation defaults expected by the production builder.

[Contract](../../docs/rb-03-contract.md) and
[validation](../../docs/rb-03-validation.md) explain the coordinate spaces,
measured defects, candidate definitions, indexing repair and remaining limits.
[The original dated result](results-20260908.json) preserves the pre-repair
97-check characterization and its defect measurements; it is not regenerated.

[The separate repair result](results-20260908-indexing.json) records 110 passing
checks against the repaired implementation. The current probe requires permutation stiffness to match the mapped reference,
checks contacts born/reappearing after an empty mapped snapshot, and tests the
actual external selector/obstacle builder with a FEM-only Hessian. Interpolated
stiffness and its mixed-obstacle behavior remain explicitly unresolved and
unchanged. The associated `[contact_stiffness_mapping]` unit tests compare
energy, gradient and Hessian against an independent surface-coordinate reference
for exact selectors, including 2D/3D edge-point cases and prescribed obstacles.

Tiny dense inverse/lift calculations compare models only. They are not proposed
production implementations. No arbitrary interpolated-stiffness model, high-order
support, physical accuracy or nonlinear-solve convergence is certified here.
