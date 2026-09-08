# RB-03 mapping characterization

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
97 checks, measured defects, candidate definitions and limitations.
[The dated result](results-20260908.json) records unchanged production behavior.
Passing includes assertions that reproduce an **unrepaired** permutation defect;
it is not a regression claiming the defect is fixed. Replace those expectations
with mapped-reference agreement when implementing the bounded indexing repair.

Tiny dense inverse/lift calculations compare models only. They are not proposed
production implementations. No arbitrary interpolated-stiffness model, high-order
support, physical accuracy or nonlinear-solve convergence is certified here.
