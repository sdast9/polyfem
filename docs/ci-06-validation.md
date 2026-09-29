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

## 2026-09-29 — Canonical smooth-contact collision order (ipc-toolkit `f8dafef39e8`)

**Status: nondeterminism root-caused and fixed; the stored reference/margin
is still unchanged pending the user's decision.** Cloud session, branch
`cloud/ci-06-order`, on top of `5fdd1925c`. A previous cloud session
diagnosed and fixed the root cause on the IPC Toolkit side but hit the
usage limit before landing anything on PolyFEM; this session pins the fix,
builds, and re-measures.

### The fix (done and verified by the previous session; cited here)

The 2026-09-28 record above measured this scene as **not bit-reproducible
single-threaded** (five fresh processes spread `1.275e-3` relative on
`err_h1_semi`) and flagged an address-order-dependent container iteration
as the suspected cause, not traced further. The previous session traced it:
`sdast9/ipc-toolkit` branch `cloud/smooth-order`, commit
`f8dafef39e881d1aa51b2a7975d06766db66d7da` (parent `cf99893b`, the pin this
record's evidence above was measured against). `SmoothCollisionsBuilder<2>`
and `<3>::merge` gathered each thread's deduplicated collision map and
appended the maps in whatever order Abseil's ASLR-seeded `robin_map` hashed
them into — a process-specific, not scene-specific, ordering — instead of a
canonical one; face-vertex/edge-edge candidate lists had the same problem.
The fix gathers each thread's map and appends in ascending primitive-id key
order, and sorts the face-vertex/edge-edge lists by their primitive-id
pair. New coverage:
`tests/src/tests/potential/test_smooth_collision_order.cpp`.

The previous session's causal proof, on this exact scene
(`gcp-contact/cube-on-floor/run.json`, a probe build, two fresh processes):
collision membership and candidates were already identical between
processes; only emission order differed. Summing `E`/`g`/`H` in canonical
order made the two processes' sums bitwise equal; the first divergence
without the fix was at the first Newton update, downstream of the
summation order, not of anything physical. After the fix: 5/5 fresh
single-threaded harness processes gave identical metrics; the five
`scenes/semi-implicit` smokes were byte-identical before/after (55
VTU/VTM/PVD files; only run-manifest provenance differed).

### This session's work

- **Pin.** `cmake/recipes/ipc_toolkit.cmake` moved from `cf99893be74f`
  (this record's earlier evidence) to `f8dafef39e881d1aa51b2a7975d06766db66d7da`.
  Built with `tools/cloud/compile.sh` (default config,
  `POLYFEM_PORTABLE_BUILD=ON`, `POLYFEM_WITH_TRIANGLE=ON`); `PolyFEM_bin
  --build_info` confirms `ipc_toolkit.matches_declared_pin: true` against
  this exact SHA. No locale-gen was needed (the fix from
  [ci-portability-plan.md](ci-portability-plan.md) is on `main`).

- **IPC Toolkit tests (this session).** In a separate clone of
  `sdast9/ipc-toolkit` at `f8dafef39e8`, cloned
  `ipc-sim/ipc-toolkit-tests-data` into `tests/data`, configured a Release
  build with `IPC_TOOLKIT_BUILD_TESTS=ON` (top-level default), built
  `ipc_toolkit_tests`, and ran the smooth-contact selection
  (`OMP_NUM_THREADS=1 ./build-tests/tests/ipc_toolkit_tests
  "[smooth_potential]"`, which in a Release/`NDEBUG` build also matches the
  new order test's `[determinism]` tag): the new
  "Smooth collisions have a canonical order" case (both 2D and 3D, via
  `GENERATE`), "Smooth barrier potential codim", "Smooth barrier potential
  full gradient and hessian 3D" (finite-difference gradient/Hessian
  checks), "Smooth barrier potential real sim 2D C^2" and "…C^1". **5/5
  test cases, 1450/1450 assertions, exit 0.**

- **Determinism re-measured with the pinned PolyFEM build.** Five fresh,
  isolated, single-threaded processes of `gcp-contact/cube-on-floor/run.json`
  through `run_manifest_env` (same path as `contact_2d`'s ctest case:
  `Eigen::SimplicialLDLT`, `set_max_threads(1)`), each in its own process
  and directory (`outputs/ci-06/20260929T023242Z/run-{1..5}/`):

  | Run | `err_l2` | `err_h1` | `err_h1_semi` | `err_linf` | `err_linf_grad` | `err_lp` |
  | --- | --- | --- | --- | --- | --- | --- |
  | 1–5 | 0.0098260282514205325 | 0.098409800311031831 | 0.097918016554970344 | 0.025735020611998116 | 0.044738583163921243 | 0.016934065257996768 |

  **All five processes produced bitwise-identical values on every metric,
  to full `double` precision (17 significant digits, printed above) —
  0 spread, against the `1.275e-3` relative spread on `err_h1_semi` alone
  measured on the previous pin.** Relative error against the still-unchanged
  stored reference (margin `1e-5`):

  | Metric | Reference (17 s.f.) | Computed (17 s.f.) | Relative error |
  | --- | ---: | ---: | ---: |
  | `err_l2` | 0.0098276900010068023 | 0.0098260282514205325 | 1.6909e-4 |
  | `err_h1` | 0.098813194555509312 | 0.098409800311031831 | 4.0824e-3 |
  | `err_h1_semi` | 0.098323262392523605 | 0.097918016554970344 | 4.1216e-3 |
  | `err_linf` | 0.025729304058592091 | 0.025735020611998116 | 2.2218e-4 |
  | `err_linf_grad` | 0.044732657488330614 | 0.044738583163921243 | 1.3247e-4 |
  | `err_lp` | 0.016938155138014152 | 0.016934065257996768 | 2.4146e-4 |

  Every metric still exceeds the `1e-5` margin (the harness's authentication
  fails, exit 42, as expected — `contact_2d` still fails only on
  `cube-on-floor`, unchanged from before this fix: 28/29 fixtures authenticate,
  `cube-on-floor` the sole failure). The magnitudes (`1.3e-4`–`4.1e-3`) sit
  inside the `3.96e-3`–`5.23e-3` range the previous, non-reproducible
  five-run spread covered for `err_h1_semi` specifically — consistent with a
  real, now-pinned-down mismatch that was always in roughly this range, not
  with the fix having moved the answer to a new place.

- **Smoke scenes.** `python3 tools/smoke/run_smoke.py --binary
  build-cloud/PolyFEM_bin --output outputs/ci-06/20260929T023436Z-smoke`: all
  five `scenes/semi-implicit` scenes still exit 0 with 0 error lines
  (`quasistatic-adaptive`, `quasistatic-semi`, `quasistatic-semi-alhess`,
  `quasistatic-semi-friction`, `transient-semi`).

- **Targeted regression check.** `OMP_NUM_THREADS=1 ./build-cloud/tests/unit_tests
  "contact_2d"`: 28/29 fixtures authenticate; the sole failure is
  `cube-on-floor`, as always. `contact_3d` also carries a GCP fixture
  exercised by the same `SmoothCollisionsBuilder` code path
  (`gcp-contact/parallel-edge/run.json`, margin `1.2e-5`): **all 50
  assertions pass, 1/1 test case, exit 0** — that fixture authenticates
  cleanly against its stored reference on this pin, no regression.

### Reading the result

The fix does exactly what it was built to do: it removes the process-level
nondeterminism from this scene's collision emission order (0 spread across
5 fresh processes, where before there was a `1.275e-3` relative spread on
`err_h1_semi` alone) without moving the *value* outside the range that
noise already covered. It does **not**, by itself, make `cube-on-floor`
authenticate against its stored `1e-5`-margin reference — nothing in this
fix's scope claimed it would; the previous record was explicit that the
fix's job was to make the number reproducible, not to make it match. What
this fix changes about the reference-policy question:

- **The scene is now bit-reproducible single-threaded**, matching
  `quasistatic-semi`'s semi-implicit path (CI-07 item 9) rather than being
  an outlier. A same-noise-floor comparison against the reference's
  generation revision — the thing the 2026-09-28 record said this session's
  evidence could not by itself separate from a real regression — is now a
  well-posed, deterministic bisection instead of one contending with
  run-to-run noise on both ends.
- **The remaining `1.3e-4`–`4.1e-3` mismatch is real and reproducible**, not
  noise. It has not been traced to a specific cause (a genuine behavior
  change somewhere in `cf99893b`'s upstream merge or earlier, versus this
  scene's reference having been generated on a different revision, remain
  open and untraced, per the earlier record).

### Recommendation (measured, not decided)

This is the user's reference/tolerance decision to make; two justified
options, unchanged in substance from 2026-09-28's recommendation but now
resting on a deterministic (not noisy) measurement:

1. **Keep the `1e-5` margin and regenerate the reference** at the
   deterministic value this fix now produces
   (`err_l2=0.0098260282514205325`, `err_h1=0.098409800311031831`,
   `err_h1_semi=0.097918016554970344`, `err_linf=0.025735020611998116`,
   `err_linf_grad=0.044738583163921243`, `err_lp=0.016934065257996768`),
   published on the `sdast9/polyfem-data` fork (`fable-fixtures` branch) the
   way CI-03's twins were, with the regeneration going through
   `resolve_reference_margin` (already fixed to preserve a margin) so a
   future regeneration cannot silently widen it back to `1e-5`.
2. **Widen this fixture's margin to a justified value** (`≥ 1e-2`, matching
   the historical `f52db28` value) if the mismatch is judged to be a real,
   accepted difference from the reference's generation platform/revision
   rather than something to chase to zero — the CI-03 precedent (an
   explicit, justified per-fixture value, not a global change).

Either way, the deterministic single-threaded value above is now a solid
basis for the decision; before this fix, the noise floor and the mismatch
were the same order of magnitude and could not be told apart.

**The multi-thread claim is separate** (not measured this session — flagged
as a possible follow-up, not required by this item): whether the fix also
removes or reduces the thread-count divergence documented elsewhere (RB-12,
CI-07 item 9) on GCP/SmoothContact scenes specifically is a one-line
question to answer cheaply (a threaded repeat of this same scene) whenever
that's wanted, but was not measured here.

### Publication

PolyFEM: `cmake/recipes/ipc_toolkit.cmake` (the pin), this record — branch
`cloud/ci-06-order`. IPC Toolkit: the fix itself was already published by
the previous session as `sdast9/ipc-toolkit:cloud/smooth-order` at
`f8dafef39e881d1aa51b2a7975d06766db66d7da`; not modified or re-pushed by
this session (never push to `sdast9/ipc-toolkit`, per this task's rules).
Evidence: `outputs/ci-06/20260929T023242Z/` (five run directories,
`outputs/ci-06/20260929T023436Z-smoke/`), not committed (gitignored,
per-task evidence convention). No solver, tolerance, reference, or default
changed by this session.
