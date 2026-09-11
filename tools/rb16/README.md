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
Stage 1 is complete. Stage 2 below adds real-form timing and public FEM cycles;
production controller/model decisions remain pending.

## Stage 2: assembled FEM and real contact callbacks

The existing Makefiles build must have `PolyFEM_bin` and `unit_tests` built.
Run each matrix with a new output directory:

```bash
python3 tools/rb14/run_fem_probe.py --build build --source tools/rb16/fem_controller_probe.cpp --config tools/rb16/stage2-cases.json --output /absolute/fresh/raw
python3 tools/rb14/run_fem_probe.py --build build --source tools/rb16/fem_controller_probe.cpp --config tools/rb16/stage2-integral-cases.json --output /absolute/fresh/integral
python3 tools/rb14/run_fem_probe.py --build build --source tools/rb16/fem_controller_probe.cpp --config tools/rb16/stage2-top-cases.json --output /absolute/fresh/top
python3 tools/rb14/run_fem_probe.py --build build --source tools/rb16/fem_controller_probe.cpp --config tools/rb16/stage2-validation-cases.json --output /absolute/fresh/controls
```

The driver compiles against the configured PolyFEM/IPC/PolySolve archives, snapshots
the source and records compile/link commands, input hashes and each process exit.
The [stage-2 protocol](stage2-protocol.md) preserves the raw-arithmetic matrix and
declares the separate integral-arithmetic and prescribed-compression comparisons.
The source is an experimental Newton driver with real assembled forms and CCD;
it is not execution of the entire production nonlinear/AL/restart pipeline.

[Results](results-20260911-stage2.json) retain 35/76 completed raw cycles and all
41 incomplete cycles, 76/76 paired integral cycles, and 8/8 prescribed-top cycles.
The three matrices passed 99,088 checks; checks passing does not imply a completed
cycle or contact-band success. [Provenance](provenance-20260911-stage2.json) also
records derivative controls, final-source replay, rebuild and focused regressions.
No production arithmetic or controller was changed.

`analyze_fem_controller.py --evidence <parent> --output <new.json>` reconstructs the
published summary using the recorded directory names `probe-01`, `integral-01`,
`top-01`, `derivative-controls` and `arithmetic-diagnostic`. Its output must be new.
Full trajectories/events stay in local evidence; hashes identify every source
result. Publication results are measurements, not automatically regenerated goldens.
