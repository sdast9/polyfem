# BFGS convergence audit

See the [findings, completed order fix and remaining repair plan](../../docs/bfgs-convergence-audit-20260922.md).

The standalone probe exercises the real PolySolve strategies and line searches:

- Scalar positive, zero and negative curvature, plus a convex-objective retune.
- Energy acceptance for zero/negative curvature by all three energy searches.
- Forty-five small solves, including pure-strategy and default-fallback cases.
- Forward-factory rejection of L-BFGS-B, separately from its supported boxed use.

It emits observations, including known failures. Exit zero means the probe
completed, **not** that all solvers converged. Actual status, residual and error
are in each `solve` row. Nonfinite scalar values are strings instead of invalid
JSON numbers. No geometry, user scenes or Teseo are loaded.

From the PolyFEM repository, with a rebuilt macOS Makefiles `build` containing
`PolyFEM_bin` and `unit_tests`, run:

```sh
python3 tools/bfgs_audit/run_probe.py --build build --output /absolute/fresh/evidence
```

The runner requires a fresh output directory. It freshly compiles the three
strategy translation units and links the existing configured support libraries.
It records source/library/executable hashes, the PolySolve revision and dirty
status, commands and exit codes. It currently targets this workspace's macOS
Accelerate/Makefiles build; it is not a cross-platform test runner.

Committed observations:

- `baseline-20260922.jsonl`: PolySolve `bce32a39`, before the order fix.
- `after-order-fix-20260922.jsonl`: same probe after moving the dense-BFGS update
  before the solve, published as `427e1458`. The baseline support libraries were
  reused; all three strategy translation units were freshly compiled.
- `validation-20260922.json`: compact hashes, build/test and public-scene results.

The corresponding raw commands, binaries, logs and full copied inputs remain in
the parent workspace's `outputs/bfgs-audit/20260922T102120Z/`. The permanent new
regressions live in PolySolve `tests/test_bfgs.cpp` and are part of its normal
test target. The fixed test verifies the current secant direction and convergence
without a gradient-descent fallback.
