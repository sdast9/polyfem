# PolyFEM fork (sdast9/polyfem)

Semi-implicit barrier contact and robustness work on PolyFEM. `README.md` holds
the measured state and validation limits; `docs/` holds the plans and records.

## Build and test

In a cloud session, run `tools/cloud/setup-environment.sh` once if the toolchain
is missing, then:

    tools/cloud/compile.sh                         # PolyFEM_bin + unit_tests in build-cloud/
    OMP_NUM_THREADS=1 ./build-cloud/tests/unit_tests "[tag]"
    OMP_NUM_THREADS=1 ctest --test-dir build-cloud --output-on-failure

The first build downloads every dependency and takes a long time; later builds
are incremental (ccache). Run long builds and suites in the background.

Run the semi-implicit scenes end to end and get one aggregate pass/fail status
with `tools/smoke/run_smoke.py --binary build-cloud/PolyFEM_bin --output <fresh
dir>` (standard library only; see `tools/smoke/README.md`). This is CI-09's
portable replacement for the developer-only `run-smoke.sh`.

Dependencies are pinned by SHA in `cmake/recipes/`: `polysolve.cmake` →
`sdast9/polysolve` (work branch `iteration-callback`), `ipc_toolkit.cmake` →
`sdast9/ipc-toolkit` (work branch `semi-implicit-stiffness`),
`polyfem_data.cmake` → `sdast9/polyfem-data` (branch `fable-fixtures`). In
those forks `main` mirrors upstream. To change a dependency, commit and push to
its work branch, then move the pin to that exact SHA. Test data is published
to the data fork first, then pinned.

The fork keeps `option(POLYFEM_WITH_MISO ... OFF)`. Every upstream merge brings
back a plain `set(... ON)`; re-apply the option after merging.

Unit-suite baseline: the two golden scenes that used to fail
(`gcp-contact/cube-on-floor`, `multi-material/stretch-cubes`) were repaired by
CI-06 and CI-04 and pass on cloud Linux; macOS and Windows runs and a complete
suite since 2026-09-29 are unconfirmed, so treat a failure as new unless
[docs/open-items-20260929.md](docs/open-items-20260929.md) says otherwise.

## Working rules

* For a named item (`RB-XX`, `RBR-XX`, `EF-XX`, `CI-XX`), read
  `docs/robustness-plan.md` (or the plan it points to), the item's existing
  record in `docs/`, and `docs/correctness-remediation-plan.md` for the PF
  invariants. Follow the item's reproduction, scope, decision boundaries and
  acceptance checks. Update its record and status when you finish.
* Do not change defaults or model behavior without the user's decision.
  Decisions already made are recorded in the plan and records.
* The constraint floor has been removed. Ignore old positive settings and do
  not reactivate it. Keep CCD and the separate trial-displacement cap.
* Do not run Teseo unless the user asks.
* Keep inputs and evidence isolated per task, for example
  `outputs/<item>/<UTC timestamp>/`. Evidence is not committed unless a record
  needs it.
* After golden tests, add files by name. Never `git add -A`.
* Format before every commit with the repository's pinned formatter: run
  `pre-commit install` once per clone (the hook then formats staged C/C++
  files on each commit; re-add them if it changed anything), or
  `pre-commit run --files <changed files>` before committing. The CI check
  pins clang-format 21.1.8 (`.pre-commit-config.yaml`); other clang-format
  versions (Homebrew's, for example) format differently and fail it.
* Verify before pushing: rebuild, run the affected tests, and keep the smoke
  scenes (`scenes/semi-implicit`) byte-identical single-threaded unless the
  change is meant to move them.
* Publish finished work: commit and push to `main` on `origin`
  (sdast9/polyfem), and report what you pushed.

Not available in a cloud session: the Houdini assets (published separately to
sdast9/houdini-plugins, with their tests driving a local `PolyFEM_bin`), and
the local workspace's `run-smoke.sh` and evidence directories. Mention it when
a task's acceptance checks depend on them.
