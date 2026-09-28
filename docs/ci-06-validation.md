# CI-06 — GCP cube-on-floor reference mismatch

Date: 2026-09-28
Status: **evidence gathered, reference-generation bug fixed within stated
scope; the stored reference/tolerance is unchanged pending the user's
decision.** Cloud session, branch `cloud/linux-evidence`, on top of `5a70555a`.
Item of the [CI portability plan](ci-portability-plan.md#ci-06--gcp-cube-on-floor-reference-mismatch).

## Contract and authorization

- Scheduled task: gather Linux x86_64 CI evidence, item (c). In scope:
  repeat `gcp-contact/cube-on-floor/run.json` across fresh single-threaded
  processes, compare against the stored reference, report the spread and
  the solver's exit/residual lines, then fix the reference-generation bug
  in `tests/verify_run.cpp` (an existing scene-specific margin silently
  overwritten to `1e-5` on regeneration) with a focused test. Not
  authorized: changing the stored reference values or the `1e-5` margin,
  or any solver/model default. Where the item calls for a decision
  (whether to widen the tolerance / accept a new reference), measure and
  recommend, do not decide.
- Read: CLAUDE.md, [ci-portability-plan.md §CI-06](ci-portability-plan.md#ci-06--gcp-cube-on-floor-reference-mismatch),
  [ci-portability-evidence-2026-09-13.md](ci-portability-evidence-2026-09-13.md)
  (the `f52db28`/`6f8f569` margin history), `tests/verify_run.cpp`.

## Host

Cloud container: Intel Xeon @ 2.10 GHz, `nproc` 4, 15 GiB RAM, GCC 13.3.0
(Ubuntu 24.04), CMake 3.28.3, Ninja, Release, TBB, `POLYFEM_PORTABLE_BUILD=ON`,
`POLYFEM_WITH_MISO=OFF`. Full record: `outputs/ci-linux-evidence/20260928T105712Z/host.txt`.
Binary `PolyFEM_bin`/`unit_tests` built from polyfem `7b9e44526ff2`
(branch `cloud/linux-evidence`), ipc_toolkit `cf99893be74f`, polysolve
`43ca2e661069`, data `sdast9/polyfem-data@e6ed5cf`.

## Reproduction and repeat evidence

Five fresh, isolated, single-threaded processes of the harness's own path
(`run_manifest_env`, `POLYFEM_RUN_MANIFEST` = a one-line manifest naming
`gcp-contact/cube-on-floor/run.json`, `POLYFEM_RUN_DATA_DIR` = the pinned
data checkout — the same code path CTest's `contact_2d` uses, including the
solver override to `Eigen::SimplicialLDLT` and `set_max_threads(1)`).
Evidence: `outputs/ci-06/20260928T161716Z/` (`manifest.txt`, one
`run-manifest.json`-bearing run per invocation; the five `Computed tests`
lines are reproduced below in full).

Stored reference (`data/gcp-contact/cube-on-floor/run.json`'s `tests` block,
margin `1e-5`):

| Metric | Reference |
| --- | ---: |
| `err_l2` | 0.009827690001006802 |
| `err_h1` | 0.09881319455550931 |
| `err_h1_semi` | 0.0983232623925236 |
| `err_linf` | 0.02572930405859209 |
| `err_linf_grad` | 0.044732657488330614 |
| `err_lp` | 0.016938155138014152 |

Five fresh single-threaded runs, and each run's relative error against the
stored reference:

| Run | `err_h1_semi` | relerr vs. reference | `err_h1` relerr | `err_l2` relerr | `err_linf` relerr | `err_linf_grad` relerr | `err_lp` relerr |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.09785456691620291 | 0.0047669 | 0.0047207 | 0.0001065 | 0.0006097 | 0.0000000 (below 1e-5 abs) | 0.0000250 |
| 2 | 0.09791342068967918 | 0.0041683 | 0.0041275 | 0.0000479 | 0.0006586 | 0.0001164 | 0.0000771 |
| 3 | 0.09784373986069371 | 0.0048770 | 0.0048395 | 0.0010946 | 0.0005028 | 0.0000872 | 0.0012194 |
| 4 | 0.09793401546874200 | 0.0039588 | 0.0039338 | 0.0014254 | 0.0012286 | 0.0000937 | 0.0014056 |
| 5 | 0.09780911010262916 | 0.0052292 | 0.0051787 | 0.0001329 | 0.0002323 | 0.0001955 | 0.0002217 |

Run 4 reproduces the plan's recorded value almost exactly (`0.09793401546874200`
here vs. `0.09793400801488551` in the CI-04-era `ctest` run recorded in
[ci-portability-plan.md §1](ci-portability-plan.md#1-verified-state) — the
eighth significant digit differs, i.e. the same process-to-process
variation measured below). Every run violates the stored `1e-5` margin.

**Run-to-run spread (same binary, same input, single-threaded, five separate
processes) is itself non-trivial:** `err_h1_semi` ranges over
`[0.09780911010262916, 0.09793401546874200]`, a relative spread of
`1.275e-3` between the five runs — about a quarter to a third of the
`3.96e-3`–`5.23e-3` relative mismatch each run shows against the stored
reference. The other five metrics show the same pattern (run-to-run spread
`2.9e-4`–`1.8e-3`, vs. reference mismatch `2.5e-5`–`1.4e-2` — `err_lp` and
`err_l2` have their *smallest*-mismatch runs fall within the run-to-run
spread of their *largest*). This scene (`use_gcp_formulation: true`,
`SmoothContact`) is **not bit-reproducible single-threaded** on this host,
unlike `quasistatic-semi`'s semi-implicit path (bit-identical in five
serial repeats, [CI-07 item 9 evidence](rb-12-validation.md#linux-x8664-repeat-matrix-ci-07-item-9-2026-09-28)).
The likely source is address-order-dependent iteration (e.g. an
unordered container of collision candidates whose bucket order follows
process-specific ASLR) feeding a floating-point summation whose order
therefore differs run to run; this was not traced further; it is not a
semi-implicit-coefficient effect (this scene does not use that path) and
is disclosed, not fixed, as outside this item's scope. **Practically: the
mismatch against the stored reference is the same order of magnitude as
this scene's own run-to-run noise floor, not clearly separable from it on
this evidence alone** — a materially different conclusion from treating
the mismatch as proof of a real solver/model regression.

## Solver exit and residual lines

A direct `PolyFEM_bin` run (`outputs/ci-06/20260928T161716Z/direct-run-1/`,
default solver selection — `Eigen::PardisoLLT`, not the harness's forced
`SimplicialLDLT`, so its error numbers are not comparable to the table
above; captured only for the exit/residual text item (c) asks for) exits
**0** after 20/20 steps:

```
[SparseNewton][Backtracking] Finished: Gradient vector norm too small took 0.0249823s
  (iters=1 Δf=2.16337e-05 ‖∇f‖=0.00033307 ‖∇f‖_rel=0.0052026 ‖Δx‖=nan ‖Δx‖_rel=nan
   Δx⋅∇f(x)=nan 1/2Δx^THΔx=nan)
  (stopping criteria: iters=500 Δf=0 ‖∇f‖=0.00355595 ‖∇f‖_rel=1e-10 ‖Δx‖=0 ‖Δx‖_rel=0
   Δx⋅∇f(x)=-4.49641e-13 1/2Δx^THΔx=0)
-- L2 error: 0.009776058905332053
-- Lp error: 0.016788298268858585
-- H1 error: 0.09748319746364927
-- H1 semi error: 0.09699176490824524
-- Linf error: 0.02500425765613044
-- grad max error: 0.04452700627651118
total time: 9.569739s
```

Every one of the 20 steps in this run ends the same way ("Gradient vector
norm too small", 1 outer iteration) — the trajectory is well inside the
Newton stopping criteria at every step; the mismatch is not a
non-convergence or a near-miss on a tolerance boundary.

## Reference-generation bug: fixed

`tests/verify_run.cpp:209` wrote `out["margin"] = 1e-5` unconditionally
whenever a manifest's `*` prefix regenerates a scene's stored reference,
silently widening any scene that had pinned a wider margin back to the
global default — a live bug independent of whether `0.01` was ever the
right value for this scene (the toolchain would erase a deliberate `0.01`
the next time someone regenerated with `*gcp-contact/cube-on-floor/run.json`).
Fixed by extracting `resolve_reference_margin(in_args, tests_key)`, which
looks up the pre-existing `margin` in the loaded JSON (present before any
overwrite) and falls back to `1e-5` only when none was set, and using it in
place of the hardcoded literal. A focused unit test
(`CI-06 reference regeneration preserves an existing scene-specific
margin`, no solve required) checks the helper directly: an existing
`0.01` is preserved, a missing margin still defaults to `1e-5`, and a
missing `tests` key defaults to `1e-5`. Verified: `OMP_NUM_THREADS=1
./build-cloud/tests/unit_tests "CI-06 reference regeneration preserves an
existing scene-specific margin"` — 3 assertions, all pass. The stored
reference values and the `1e-5` margin on the live fixture are **unchanged**;
this only fixes what a *future* regeneration would do.

## Recommendation (measured, not decided)

- The `1e-5` margin was almost certainly never achievable for this scene on
  any platform: even same-binary, same-input, single-threaded repeats spread
  `1.275e-3` on `err_h1_semi`, over two orders of magnitude above `1e-5`.
  Whatever value a margin is set to for this scene, it must be well above the
  measured run-to-run noise floor (recommend ≥ 1e-2, matching the `f52db28`
  value the fixture briefly carried, and re-measuring the noise floor before
  finalizing — the CI-03 pattern of an explicit, justified value is the
  applicable precedent).
- The mismatch against the current stored reference is not clearly
  distinguishable from run-to-run noise on this evidence; before treating it
  as a solver regression, the same repeat should be run on the reference's
  generation platform/revision (or a bisection of the intervening PolyFEM/
  IPC/PolySolve history) to see whether the noise floor was always this
  size. This session did not do that bisection (out of scope for item c).
- The address-order-dependent single-threaded nondeterminism is a
  independent finding worth its own investigation (likely an
  unordered-container iteration order feeding a floating-point sum in the
  GCP/SmoothContact collision path) — flagged here, not pursued.

## Publication

PolyFEM commit: this record follows the `verify_run.cpp` fix commit on
`sdast9/polyfem:cloud/linux-evidence`. Evidence not committed (raw VTU/
run-manifest output, ~40 MB): `outputs/ci-06/20260928T161716Z/` in this
session's working tree only. No solver, tolerance, reference, or default
changed.
