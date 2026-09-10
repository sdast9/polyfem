# RB-13 stage 1 reference probe

Run from the PolyFEM repository with Python 3 and a C++17 compiler:

```bash
python3 tools/rb13/reference_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/evidence/probe
```

The output directory must not already exist. Verify `--ipc-source` against the
configured build's effective IPC override/cache and recipe pin before running.
The tested source is IPC `af317a65d69d0ac7c5efa4bf103bf75e280c323b`.
The runner compiles `barrier_bridge.cpp` with that checkout's actual
`src/ipc/barrier/barrier.cpp`. It needs no PolyFEM build or Python packages.
Only this primitive is compiled; the probe does not call BarrierContactForm,
the integrator classes, collision builders or the FEM solver.

It compares the compiled primitive with physical-distance formulas and 60-digit
energy-only finite differences at 15 interior points, then checks free-coordinate
compliance, prescribed-motion prediction, force balance and a bisection root.
Small algebraic controls cover the source-derived integrator scales and weights,
nonpositive-tangent rejection, and squared-gap controller interpretation.
Tolerances and conditioning scope are declared in the script before evaluation.
The missing-chain-term control demonstrates an incorrect derivation, not a
production defect. Model inputs and checks are embedded in the small script.

Outputs include `results.json`, `provenance.json` with source hashes and exact
compiler command, compiler log and raw IPC input/output. Failures return nonzero;
keep their output and use a new directory for a corrected run. The checked-in
[result](results-20260909-stage1.json) reports 101/101 checks passing.
Full baseline process/repository/cache/binary identity is in the local evidence
directory recorded in [validation](../../docs/rb-13-validation.md).

Read the [contract](../../docs/rb-13-contract.md) for assumptions and units.
Stage 1 does not implement conditional interval tests, stage 3's scope matrix,
a production law, or a scene accuracy/robustness test.
