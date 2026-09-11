# RB-16 spring controller comparison

From `polyfem/`:

```bash
python3 tools/rb16/controller_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/evidence
```

The output directory must be new. Verify the IPC source against CMake's effective
override and recipe pin. The runner compiles RB-13's unchanged scalar bridge with
that checkout's actual barrier primitive; it needs Python's standard library and
`/usr/bin/c++`. Tested on macOS arm64. It does not rebuild or execute PolyFEM.

Read the [protocol](stage1-protocol.md), [contract](../../docs/rb-16-contract.md)
and [validation](../../docs/rb-16-validation.md). The checked-in
[results](results-20260911-stage1.json) contain 126 completed spring trajectories,
50,108 checks and all configuration summaries. They are measurements, not goldens.
The [provenance](provenance-20260911-stage1.json) identifies inputs and the compiled
primitive. Full trajectories, Newton paths, coefficient events, compile/run logs
and retained exploratory results live in the fresh local evidence directory
specified by the validation record. Existing results are never overwritten.

The residual criterion is unchanged across the arithmetic fix. Stable energy
differences use factored polynomials and log1p, checked against 60-digit Decimal
values. No production source, configuration or physical model was changed.
Stage 1 is complete; actual real-form timing and public FEM cycles remain pending.
