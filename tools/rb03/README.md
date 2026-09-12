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
checks against the repaired implementation. [The interpolation result](results-20260911-interpolation.json)
records 120 passing checks after the 2026-09-11 stage: the probe now requires
permutation stiffness to match the mapped reference, checks contacts
born/reappearing after an empty mapped snapshot, tests the actual external
selector/obstacle builder with a FEM-only Hessian, and requires interpolated
stencils to read the parent block condensed onto the stencil (218.3333 on the
averaged-point fixture, 133.3333 on the mixed FEM/obstacle proxy) with the
gap-normalized direction fallback on dependent rows (45) and an indefinite
parent block (123.75). The associated `[contact_stiffness_mapping]` unit
tests compare energy, gradient and Hessian against an independent
surface-coordinate reference for exact selectors, including 2D/3D edge-point
cases and prescribed obstacles, and compare interpolated stencils — six
synthetic variants and a real Q1 hex column built by the production builder —
against an independent dense condensation oracle.

`hex-scene/` holds a 4×4×4 Q1 hexahedral unit cube (`hex-cube.mesh`, generated
lattice) and `quasistatic-semi-hex.json`, the public `quasistatic-semi.json`
smoke on that mesh (same slab obstacle, material, boundary condition and
schedule); run it from that directory with `PolyFEM_bin --json
quasistatic-semi-hex.json -o /absolute/fresh/output --log_level debug` to see
the `interpolated map rows` diagnostic line. It is completion/diagnostic
evidence for the interpolated path, not a physical benchmark.

The dense inverse/lift calculations in the probe compare models only; the
production implementation is the bounded per-stencil condensation in
`BarrierContactForm::interpolated_stiffness`. No high-order support, physical
accuracy or nonlinear-solve convergence is certified here.
