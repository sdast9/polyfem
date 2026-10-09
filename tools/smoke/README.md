# Portable smoke runner (CI-09)

`run_smoke.py` is a standard-library-only replacement for the developer-only
zsh `run-smoke.sh` referenced by [docs/ci-portability-plan.md](../../docs/ci-portability-plan.md)
(CI-09). It takes an explicit binary, scene directory and output directory
instead of assuming a sibling `build/PolyFEM_bin` or absolute developer
paths, runs each scene as an argument list (no shell), and returns a single
aggregate pass/fail status.

```sh
python3 tools/smoke/run_smoke.py --binary build-cloud/PolyFEM_bin --output outputs/smoke/20260928T151500Z
```

Options:

- `--binary` (required): the `PolyFEM_bin` executable to run, built or
  packaged.
- `--scenes` (default `scenes/semi-implicit`): directory of scene `.json`
  files and their mesh/obj/msh assets.
- `--output` (required): a fresh directory for isolated per-scene
  input/output copies and the summary; refused if it already exists and is
  non-empty.
- scene names (positional, optional): run only these scenes (matched by
  filename stem, with or without `.json`); default is every `*.json` in
  `--scenes`.
- `--threads`: passed to the binary as `--max_threads`; omitted uses the
  binary's own default.
- `--timeout` (default 1800s per scene).

For each scene the runner copies that scene's `.json` plus the directory's
mesh/obj/msh assets into its own `<output>/<scene>/input`, runs the binary
from there with `-o <output>/<scene>/output --log_level debug`, and records:

- exit code (or timeout),
- the number of `[error]` log lines,
- the last few log lines matching `Semi-implicit barrier stiffness`,
  `Refreshed semi-implicit`, `hessian-scaled initial AL`, or `stall`,
- the worst (smallest) three `Minimum distance during solve: <x>` readings,
- wall time.

`summary.json` and `summary.md` are written into `--output`, and the table is
printed to stdout. The process exits nonzero if any scene exits nonzero,
times out, or logs an `[error]` line.

A healthy run of the five `scenes/semi-implicit` scenes exits 0 with 0 error
lines and minimum distances on the order of `1e-4` (the scenes clamp contact
at `dhat = 1e-3`). Because each scene runs from its own copied input
directory using an argument list, binary/output/scene paths containing
spaces or non-ASCII characters work the same as any other path.
