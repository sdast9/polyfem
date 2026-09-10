# RB-13 standalone reference probes

## Stage 1

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

## Stage 2

```bash
python3 tools/rb13/band_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/evidence/stage2
```

The same source-override verification and fresh-directory rule apply. Read the
[predeclared protocol](stage2-protocol.md) and section 7 of the
[contract](../../docs/rb-13-contract.md). The runner compiles `band_bridge.cpp`
against actual IPC barrier primitives, calculates interval endpoints with
60-digit Decimal arithmetic and checks roots with compiled IPC forces.
Selected roots are compared with an independent Decimal bisection.

The [stage 2 result](results-20260910-stage2.json) has 23,396 passing checks,
2,282 scalar equilibria and four expected rejections (two invalid inputs and
two zero-coefficient cases without positive-gap equilibrium). It covers 81
exact-input classifications, seven uncertainty boxes, endpoint tightness and
the declared counterexamples. Counts include several assertions per root and
adjacent-pair monotonicity checks; they are not counts of FEM scenes.

The reference interval function reports `nonempty`, `zero_only`, `empty`,
`inactive_band_demand` or `mixed_band_applicability`. Empty bounds retain their
ordering; inactive/mixed inputs have no applicable all-box band interval.
Raw bounds are retained only as diagnostics. Coefficient fractions used in the
coverage grid are test samples, not controller choices or empty-interval fallbacks.

Outputs include compile/provenance logs, raw root requests/responses, complete
query metadata and summarized results. Only the small summary is checked in.
No production behavior changes, application uncertainty enclosure, floating-point
interval certificate, stage 3 completion or full-simulation guarantee is claimed.

## Stage 3

```bash
python3 tools/rb13/scope_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/evidence/stage3
```

Read the [predeclared protocol](stage3-protocol.md). The runner compiles
`scope_bridge.cpp` and effective IPC's `barrier.cpp` into a local shared library,
called through Python's standard-library ctypes. It imports stage 1's unchanged
small Cholesky solver and does not write bytecode into the repository.
No full PolyFEM or IPC library rebuild is needed. The tested platform is macOS
arm64; the filename uses `.dylib` and other platforms are not validated.

The [result](results-20260910-stage3.json) reports 654 checks, 78 scalar roots,
two full three-displacement/two-contact solves, 27 unit-conversion configurations,
12 nonlinear tangent cases and six explicitly rejected H/J cases.
Rayleigh values are evaluated from the source-traced quadratic expression;
the production stiffness assignment/controller is not executed.

All root histories/results, nonlinear prediction errors and unsupported-input
outcomes are retained. A successful test of a counterexample means it reproduced
the limitation, not that the corresponding approximation is accurate. Shared
production binaries, stage 1/2 results and production defaults stay unchanged.
See [contract section 8](../../docs/rb-13-contract.md) and the
[stage 3 validation record](../../docs/rb-13-validation.md) for the measured limits
and downstream decisions. Completing RB-13 does not select a production model.
