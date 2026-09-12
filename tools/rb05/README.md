# RB-05 candidate/resource failure containment: probe and runners

Tools behind [docs/rb-05-validation.md](../../docs/rb-05-validation.md).
Public inputs only; nothing here runs Teseo or a private scene.

```sh
# Bounded broad-phase probe (effective toolkit via the unit_tests flags/link line):
python3 tools/rb05/run_probe.py --build build --output /absolute/fresh/dir \
    [--configs quick|default|budget] [--max-items 2e8]

# End-to-end resource-limit runs on public scenes (unlimited / generous / tiny / sweep / unsupported):
python3 tools/rb05/run_scene_limits.py --binary build/PolyFEM_bin --output /absolute/fresh/dir [--threads 1]
```

`broad_phase_probe.cpp` builds one swept candidate set on two synthetic
triangulated sheets and reports the default hash grid's intermediates —
`(box, cell)` items, pre-filter pair emissions, final candidates, wall time,
maximum RSS — next to an independent pre-allocation estimate computed from
the boxes alone (it must equal the instrumented grid's counts), and can set
the toolkit's `BroadPhaseBudget` (`--budget-cell-items`, `--budget-emissions`).
A configuration whose estimated items exceed `--max-items` is reported with
its estimate only and never built, so the probe is bounded by construction.

`run_scene_limits.py` runs isolated copies of the public smokes with the
physical diagnostics on and the opt-in `solver/contact/CCD/resource_limits`
set to: nothing, generous values (solution must be bit-identical), a limit
the static build cannot meet, a limit only the first trial sweep exceeds
(the `aborted` attempt row, checked with `tools/rb04/check_solver_attempts.py`),
and a limit on a broad phase that cannot enforce one (refused at startup).
