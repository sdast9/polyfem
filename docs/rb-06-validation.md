# RB-06 — Failed-attempt state isolation and rollback

Date: 2026-09-15
Status: **validated within stated scope** (the step transaction of the
nonlinear elastic VarForm path: every form's attempt state and the caller's
solution are restored to the last accepted state when a solve attempt fails;
nothing of a failed step is published; the restore is verified per failure
and recorded; a solve from the restored state equals a fresh control)
Selected stage: inventory → reproduction → in-memory transaction + failure
injection hook → validation → publication. No retry (RB-08 decision), no
timestep/load change, no acceptance tolerance, no disk checkpoint claim.

## Contract and authorization

- User-selected item: "Let's continue with RB-06" (2026-09-15); RB-06 was
  `not started`. No scope or policy decision beyond the plan's section:
  in-memory transaction work, not automatic retry. RB-08's decision (retry
  stays off) stands: a failed step still ends the run with a named failure
  (exit status 1, or 3 for a resource failure).
- Invariant (plan): a failed attempt must not contaminate the last accepted
  state or a subsequent attempt. Success criterion of this stage: after any
  exception escapes a step's nonlinear solve, (1) the caller's solution and
  every form's attempt state (contact snapshot, global trim/stiffness and
  memo, swept candidates, friction lag, AL multipliers and weights, lagged
  fields, form weights/scales/enabled flags, the problem's coordinate mode)
  equal the state captured at solve start, verified by a fingerprint and
  recorded; (2) the time-integration history is unchanged; (3) no step
  callback, frame, PVD entry, CSV row or restart state of the failed step is
  written, and earlier accepted frames are untouched; (4) the failed
  iterate and the failed attempt's form state survive only in the diagnostic
  records (RB-04 failure record, coefficient-event stream, manifest); (5) a
  solve from the restored state — driven by a test, never by production —
  equals a fresh control given identical inputs; (6) with no failure every
  public scene is bit-identical to the previous binary.
- Dependencies: RB-01 (cache ownership: the same invariant is now applied to
  the adhesion and smooth-contact forms, see R1), RB-02 (state inventory:
  extended below), RB-04 (failure record: the recording boundary and the
  meaning of `endpoint` are unchanged; two fields added), RB-05 (the swept
  cache lifecycle; the `aborted` row), RB-12 (manifest: the step record is
  amended with the rollback verification). Effective IPC `c24d803e`,
  PolySolve `bce32a39`, both local overrides, unchanged.
- Exclusions: the legacy `State` solve path (remeshing scenes; the same
  loop structure but no transaction), the FSI embedding (`NavierStokesFSI`
  drives the solid forms from its own problem; the stacked forms carry the
  state API for it but no transaction is installed there), the
  differentiable/adjoint solve body (`DifferentiableNonlinearElasticVarForm`
  in differentiable mode: every forward solve rebuilds its forms, so nothing
  carries over; its transient loop does not get the publication hooks), the
  conservative inversion check's committed quadrature refinements (only a
  pending candidate is dropped; miso builds only — this build forces the
  Discrete check), crash/power-loss safety of the files on disk (not
  claimed), any change to the AL scaled-weight retry or the stall restart
  (warm-started corrections at the same time, kept as they are), Teseo and
  private scenes (not run).

## Baseline and reproduction

- PolyFEM `52e4f7e48` on `main`, clean tree at start; no other session's
  build or run in progress. Toolkit `c24d803e` (`semi-implicit-stiffness`),
  PolySolve `bce32a39` (`iteration-callback`), local overrides per
  `CMakeCache.txt`; recipe pins equal the checkouts.
- Build: `polyfem/build`, RelWithDebInfo, Apple clang 21.0.0, Unix Makefiles,
  TBB, Python on, miso off; macOS 26.5.2 (Darwin 25.5.0), 18 cores. Baseline
  binaries rebuilt at HEAD before any edit and copied to the evidence
  directory: `PolyFEM_bin 782773fe…`, `unit_tests 3f02dc78…`
  (`baseline.txt`, `baseline-binaries/`).
- Evidence: `outputs/rb-06/20260915T141233Z/` in the parent workspace —
  `baseline.txt`, `build-logs/`, `r1-adhesion-baseline-repro/`,
  `injection-e2e-final/` (the E2E matrix; `injection-e2e-preformat/` an
  earlier identical run, `injection-e2e-stale-binary-partial/` a run
  against a stale link, kept), `ab-smokes-preformat/` (A/B smokes, with
  `timing.txt`), `rb04-endpoints-final/` (single-threaded RB-04 runner +
  checkers), `rb04-endpoints-final-threaded-failed/`, `…-threaded-repeat/`,
  `rb04-endpoints-baseline-threaded{,-2}/`, `threaded-friction-repeats/`
  (the threaded-friction attribution, see Validation), `tests-final/`
  (binaries, unit-test logs), `hda-tests/`.

### Inventory (code reading; the RB-02 state table extended)

The attempt of step n (`NonlinearElasticVarForm::solve_tensor_nonlinear`)
mutates, in this order: `init_lagging` (friction lag, lagged regularisation,
Rayleigh lag, tangential adhesion lag); `ALSolver::solve_al` — the
feasibility check's swept candidates, the AL weight (`set_initial_weight`),
the multipliers (`update_lagrangian` after every AL pass, accumulated across
steps and never reset), the caller's `sol` (adopted after each subsolve,
reverted to the step start on a grown BC error), the barrier refresh at
every subsolve start (`update_barrier_stiffness` → published-endpoint
refresh: frozen surface and Hessian, memo, continuation seeds, batch
statistics, trim controller, anchor), the stall restarts (retune + refresh +
trim bump; the second hard stall reverts the subsolve coordinates but keeps
the retuned coefficients — by design a warm-started correction, not a
rollback); `solve_reduced` — the same refresh, the in-solve post_step trim
controller and birth refreshes, PolySolve's iterate (the caller's `sol` is
assigned only on convergence); the lagging loop — `update_lagging` at each
returned iterate, `sol` assigned after each lagged minimize. Not mutated by
an attempt: the time-integration history (advanced between steps), the
body/pressure/inertia forms' time-dependent quantities (`update_quantities`
between steps), the AL targets, the mesh/basis/assembly caches.

| State / owner | Class | Restore action |
| --- | --- | --- |
| Caller's `sol` | authoritative | copied at solve start, assigned back |
| Time integrator `x/v/a_prevs`, `dt` | authoritative, not touched by an attempt | checked (fingerprint), not restored |
| `ContactForm`: `barrier_stiffness_` (trim/stiffness), `max_barrier_stiffness_`, `prev_distance_`, swept `candidates_` + flag, candidate statistics | trim/max/prev_distance authoritative; candidates a line-search cache; statistics diagnostic | all restored (`ContactForm::State`) |
| `BarrierContactForm`: `collision_set_`, `kappa_surface_`, `kappa_hessian_`, `kappa_hessian_max_`, `kappa_cache_`, `prev_kappa_cache_`, `endpoint_kappa_`, `continued_keys_`, batch cap/floor/median, fallback counters, `kappa_snapshot_had_contacts_`, `trim_solve_anchor_`, `iters_since_refresh_`/`_trim_`, `diagnostic_refresh_id_` | the snapshot content and the continuation/F2 history are authoritative between refreshes (a refresh at the same x with a different history assigns different coefficients; new stencils before the first refresh are priced from the frozen snapshot); the collision set is a cache restored for coherence | all restored (`BarrierContactForm::State`); the refresh id travels with the snapshot it identifies; `coefficient_event_id_` deliberately not restored (monotonic, so the stream keeps the failed attempt's rows distinguishable) |
| `SmoothContactForm::collision_set_`, `NormalAdhesionForm` (prev distance, swept cache, collision set), `TangentialAdhesionForm` lag | as above | restored |
| `FrictionForm`: `friction_collision_set_`, `lagged_trim_`, `lag_x_` | authoritative (the lag that acted; RB-10) | restored |
| `AugmentedLagrangianForm`: `k_al_`, `k_scale_`, `lagr_mults_` (every AL form; the stacked form delegates to its terms and rebuilds its constraint data) | authoritative, persisting across steps | restored |
| `LaggedRegForm::x_lagged_` + enabled flag, `RayleighDampingForm::lagged_stiffness_matrix_` | authoritative lag | restored |
| Every form's `weight_`, `scale_`, `enabled_`, `project_to_psd_` | `scale_` is re-derived by `normalize_forms` at every minimize; restored anyway | restored (`FormState` base) |
| `ElasticForm::pending_refinement_` (conservative check) | attempt candidate | dropped; committed refinements not undone (limit) |
| `NLProblem::current_size_` (full/reduced) | mode | restored |
| PolySolve solver instances, descent history | attempt-local (created per attempt) | nothing to restore |
| `stats.solver_info`, `save_subsolve` VTUs, the RB-04/RB-12 streams, event/refresh counters | failure artifacts, retained by design | kept |
| Output commit points: step callback → energy row → `step_N.vtm/.vtu` → PVD rewrite → history advance → forms' `update_quantities` → weights → between-steps refresh → restart state / stats row | publication of an accepted solve | a failed attempt reaches none of them; a publication failure of an accepted solve stops the run with the manifest step `accepted` and the PVD listing only completed frames |

### R1 — the adhesion form's caches (reproduced on the upstream code paths)

`tests/test_step_rollback.cpp`, "NormalAdhesionForm owns its collision set
…", run against the upstream code of `NormalAdhesionForm::init` /
`update_quantities` / `update_collision_set` temporarily restored in this
tree (`r1-adhesion-baseline-repro/`): (a) the function-static position
cache (the RB-01 defect, still present in this form and in
`SmoothContactForm`) let another instance's evaluation of the same
positions suppress a fresh form's first rebuild — **0 collisions instead of
1** on its first `init` at coordinates in contact
(`baseline-behaviour-run.log`); (b) with the static cache removed but the
swept cache left as upstream, an abandoned line search's candidates were
consumed by the retry `init` / the next step's `update_quantities` — **0
collisions instead of 1, energy 0 instead of −0.0375** (`no-discard-run.log`),
the RB-05 R1 mechanism on this form. Neither form is exercised by the public
smokes (adhesion and the smooth/GCP formulation are opt-in), so these are
unit reproductions of the mechanism, not observed production trajectories.

### R2 — the state a failed attempt leaves behind (baseline binary)

The RB-04 real-failure fixture (`quasistatic-semi`, restart budget
`soft_iteration_limit 1, max_restarts 1`; "Final reduced solve did not
converge" at step 1) on the baseline binary
(`rb04-endpoints-baseline-threaded/quasistatic-semi-failure-on/`): the
coefficient-event stream ends at event 4 with the contact form holding
**trim 4.0, refresh #3, 54 memoized stencils, 44 active collisions at the
failed iterate**, while the solve started with trim 1.0, refresh #0, no
memo and no contact (the first contact is born in-solve). The caller's
solution is unchanged here only because `solve_reduced` assigns it on
convergence alone; a failure inside the AL loop or the lagging loop leaves
the iterate in it (the in-process test below shows both). Nothing restores
any of it; RB-04's record documents this as "retained caller coordinates
with current attempt form state". With the process stopping at the failure
(RB-08) the contamination is invisible to `PolyFEM_bin` users but real for
any caller that keeps the objects — and it is the state RB-07's "restore
accepted state" would have to start from.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| A failed attempt leaves the caller's solution and every form's attempt state at the failed iterate; nothing restores the last accepted state | R2; the in-process test's `sol != before` before the fix (the after-AL and lagging phases assign `sol`) | **fixed**: the step transaction (below); restore verified per failure |
| The adhesion form skips its own rebuild on a function-static position cache shared by every instance (RB-01 invariant); the smooth-contact form has the same cache | R1(a); reading | **fixed** in both forms (rebuild on every notification, as `BarrierContactForm` since RB-01) |
| The adhesion form treats an abandoned swept cache as complete (RB-05 invariant) | R1(b) | **fixed**: `init`/`update_quantities` end the interval first; `line_search_begin` replaces any previous interval and is exception-safe |
| No way to fail a solve deterministically at a chosen point for tests | — | **added**: `solver/advanced/failure_injection` (test hook, off by default) |
| The RB-04 runner's on/off comparison of the friction fixture is not meaningful threaded (RB-12 stage 2) | Validation | **tool**: `--max-threads`, `--binary` options |

### What changed

- `Form` (`solver/forms/Form.hpp`): `FormState` (weight, scale, enabled,
  PSD flag) and the virtual pair `save_state()` / `restore_state(state, x)`;
  a state restored on a different form type is a `logic_error`. Overrides
  with their own `State` types: `ContactForm`, `BarrierContactForm`
  (emits a `rollback` coefficient event: before = the failed attempt's
  state at the restored coordinates, after = the restored state),
  `SmoothContactForm`, `NormalAdhesionForm`, `TangentialAdhesionForm`,
  `FrictionForm`, `AugmentedLagrangianForm` (+ `lagrange_multipliers()`
  accessor; `StackedAugmentedLagrangianForm` delegates to its terms and
  rebuilds its constraint data), `LaggedRegForm`, `RayleighDampingForm`,
  `StackedForm` (delegates), `ElasticForm` (drops a pending refinement).
- `FullNLProblem::save_state()` / `restore_state()` (`SavedState`: every
  form's state), `NLProblem` adds the penalty forms and the coordinate mode;
  `FullNLProblem::set_post_step_fault` (the test hook's Newton-iterate point).
- `NonlinearElasticVarForm::solve_tensor_nonlinear`: the rollback point
  (solution copy, problem state, fingerprint) is captured before the first
  mutation (23–38 µs on the 387-DOF public fixtures, logged at debug); on
  any exception the RB-04 failure record is written as before with a
  `rollback` announcement and the `solve_start` coordinates, then the
  restore runs, the fingerprint (`attempt_state_fingerprint`: solution
  norms, the barrier form's `diagnostic_state`, the friction lag size and
  lagged normal-force sum, the AL weights/multiplier norms, form weights and
  flags, the integrator history norms) is compared with the capture, the
  manifest's step record is amended (`rollback: {performed, verified}`), a
  warning (or an error with both fingerprints when they differ) is logged,
  and the exception is rethrown unchanged. `attempt_state_fingerprint` is
  exact json equality, not a tolerance.
- The transient loop is now three explicit stages
  (`begin_transient_run`, `solve_transient_step` = solve + publication,
  `advance_transient_step` = history advance and between-steps updates,
  `end_transient_run`); `solve_problem` calls them in order, bit-identical
  to the previous loop; tests drive them through `VarFormTestAccess`.
- `solver/advanced/failure_injection` (`solver/FailureInjection.{hpp,cpp}`,
  spec entries with the precise phase semantics): `after_al`,
  `after_stall_retune`, `al_subsolve` / `reduced` at an accepted Newton
  iterate, `lagging` after the lag update, `before_publication`,
  `after_publication`; `kind` `named` (an `InjectedFailure`, a
  `std::exception` that is not a `runtime_error`, like the RB-05 resource
  failure, so no in-step handler absorbs it) or `runtime_error`. A warning
  names the hook at startup; `main` reports an injected failure like any
  named failure (exit 1).
- `RunManifest::amend_last_step`; `tools/rb04/check_coefficient_events.py`
  accepts the `rollback` operation (last event of its step, `operation_threw`
  false, restored refresh id no newer than the failed one; exactly one in the
  failure run); `tools/rb04/run_endpoints.py` gains `--binary`,
  `--max-threads`; `tools/rb06/run_injection.py` (E2E matrix);
  `tests/test_step_rollback.cpp` (`[rollback]`).

Why this restores the contract: the attempt's mutations are exhaustively the
members listed in the inventory (every write path of the forms during an
attempt was traced: `init`, `init_lagging`, `line_search_begin/end`,
`solution_changed`, `post_step`, `update_lagging`, `update_lagrangian`,
`set_initial_weight`, the barrier refresh/retune/bump/calibrate paths,
`set_weight`/`set_scale`/`set_enabled`); each is copied by value at the
boundary and assigned back. Two independent checks guard the list: the
fingerprint equality after every restore (covering the diagnostic state of
the barrier form, the lag, the multipliers, the weights and the history)
and the scene-level equality of a re-solve from the restored state with a
fresh control (bit-identical, single-threaded) — a member missing from a
`State` would show as a differing endpoint or fingerprint. Restoring the
caches together with the authoritative state makes the restored forms
coherent without an extra rebuild (the test's re-solve starts with the
usual `init` anyway). No coefficient, tolerance, default or retry policy
changed; the transaction is inert on success (the capture is the only cost).

## Validation

Final binaries: `PolyFEM_bin 3eb5a50f…`, `unit_tests 1c8d8ae2…`
(`tests-final/binaries.txt`; sources = the committed ones, clang-format
clean). Single-threaded unless stated.

| Check | Input/configuration | Expected criterion | Measured result | Status |
| --- | --- | --- | --- | --- |
| R1 reproduction | upstream adhesion code paths temporarily restored | stale/shared caches consumed | 0 collisions instead of 1 (static cache); 0 instead of 1 and energy 0 instead of −0.0375 (stale sweep) | reproduced |
| R2 reproduction | RB-04 failure fixture, baseline binary | failed form state left behind | trim 4.0 / refresh 3 / 54 memo / 44 active at the end of the stream, start was 1.0 / 0 / 0 / 0 | reproduced |
| Form state round trips | `[rollback]`: barrier form through trial rebuild, post_step, bump, stall retune, published refresh, active sweep; friction lag; AL multipliers/weight; lagged regularisation; adhesion; typed-state refusal | restored diagnostic state, values, gradients, memo sizes, frozen surface/Hessian, trim equal the capture exactly; rollback event before/after | all equal; event operation `rollback`, before = mutated objective, after = start objective | pass |
| Recovery equals a fresh control (in-process) | `transient-semi` + friction 0.3, 3 steps, injection at step 2: `after_al`, `reduced` it 1 (named and `runtime_error`), `lagging` 1; and `after_stall_retune` under restart budget 4/8 (control with the same budget) | after the failure: `sol == start`, fingerprint equal, integrator unchanged, callbacks `{0,1}`, RB-04 record `failed_attempt` with the rollback and `solve_start == start`, manifest rollback verified; after the test's re-solve: every endpoint and fingerprint `==` the control's | all `==` (bit-identical), max difference 0 on every step of every scenario; callbacks `{0,1,2,3}` after the re-solve | pass |
| Publication failure (in-process) | `before_publication` at step 2 | solve accepted (`sol` moved), history not advanced, no callback for step 2, no `step_2` files, PVD without step 2, manifest step `accepted` without rollback | as expected | pass |
| E2E through `PolyFEM_bin` | `tools/rb06/run_injection.py`: 3 controls, 5 solve-phase injections, 2 publication injections, the real-failure fixture (`injection-e2e-final/`) | per run: exit 1, manifest completion failed / last step `failed_attempt` with rollback performed+verified, no frame/PVD entry/energy row of the failed step, RB-04 record announces the rollback with restored coordinates = previous endpoint, coefficient stream ends the step with `rollback`, frames before the failure identical to the control; publication cases as specified | 11 runs, 96/96 checks pass (`results.json`) | pass |
| Real (non-injected) failure | RB-04 fixture, candidate | rolled back and verified | rollback event trim 4.0 → 1.0, refresh 3 → 0, memo 54 → 0, active 44 → 0; manifest `performed: true, verified: true`; exit 1 | pass |
| Disabled-hook equivalence | six public scenes (A/B script), baseline `782773fe` vs final | proxies and solutions bit-identical | 6/6 exit 0/0, proxies identical, solutions identical, max diff 0.0 (`ab-smokes-preformat/identity.txt`) | pass |
| Overhead | same runs; debug capture timing | not resolvable / small | solve times within ±1 % (e.g. 1.649 vs 1.662 s, 1.764 vs 1.745 s); capture 23–38 µs per step (`timing.txt`, run logs) | pass (no large-scene measurement, see limits) |
| RB-04 endpoint runner + both checkers | `run_endpoints.py --max-threads 1`, final binary | all pairs and checks pass; failure pair equal exit codes | `passed: true` (three on/off pairs bit-identical to 1e-10, failure pair 1/1), events check exit 0 with the rollback event accepted, attempts check exit 0 (`rb04-endpoints-final/`) | pass |
| Affected unit selection | `[resource_containment],[contact_cache],[al_solver],[friction_lag],[run_manifest],[contact_form],[friction_form],[kappa_continuity],[form_derivatives]`; `[input_validation]` (spec change) | pass | 62 cases / 3,983 assertions; 18 / 168 | pass |
| Full unit suite | `unit_tests --rng-seed 1 --order decl`, default threads | only the known scene-group failures | 350 cases: 347 passed, 3 failed — `contact_2d` (`large-ratios/large-mass-ratio.json`, the RB-10 defaults flip against an unregenerated golden; `gcp-contact/cube-on-floor/run.json`, the open cube-on-floor case, whose authenticated error moves run to run: L2 .009817/.009809 in the RB-11 runs, .009822/.009826 on the RB-12 CI lanes, .009827 here), `contact_3d` (`large-mass-ratio.json`), `standard` (`multi-material/stretch-cubes.json`, RB-22); 5,158,588 / 5,158,592 assertions; 60.6 min (`tests-final/full-suite.log`) | pass (known failures only) |
| HDA end-to-end, Houdini 22.0.429 | all 13 `houdini_HDAs/tests/test_*.py` on the final binary (`hda-tests/`) | 13/13 exit 0 | 13/13 exit 0 (`summary.txt`) | pass |
| Formatting / whitespace / links | clang-format on every changed C++ file, `git diff --check`, record links | clean | clean | pass |

Threaded attribution (not an RB-06 effect): the RB-04 runner run with the
default threads failed its friction on/off comparison twice on the
candidate (3.93e-5 at step 4, the last lagged solve taking 6 or 7 Newton
iterations instead of 4) and passed twice on the baseline; six further
threaded repeats per binary against the serial endpoint gave baseline 6/6
within 2e-15 and candidate 5/6 within 2e-15, 1/6 at 3.93e-5
(`threaded-friction-repeats/results.json`). RB-12 stage 2 measured exactly
this cluster on the baseline (3.9e-5 and 1.7e-4 at step 4 of the friction
fixture under 18 threads, serial bit-reproducible): a discrete controller
decision at the tolerance edge on evaluation-order roundoff. The candidate
cannot change the arithmetic (the serial A/B smokes, including the
friction scene, are bit-identical); the threaded runs remain the RB-12
observation. The runner now takes `--max-threads 1` for the comparison.

- Numerical termination versus residuals: unchanged by construction (the
  A/B smokes are bit-identical; the transaction never alters an attempt).
- Physical quantities: none measured here; no physical claim.
- Missing quantities: no measurement of the capture cost on a large scene
  (the copy is the system Hessian's nonzeros plus the collision set and the
  memo; linear in both); no test of the FSI or legacy paths; no
  allocation-failure test (RB-05: impossible on this platform).
- Retained failed/partial runs: `injection-e2e-stale-binary-partial/` (a
  matrix run against a `PolyFEM_bin` that had not been relinked after the
  last source edit: 6 checks failed on the missing `solve_start` field —
  the binary, not the code); the two threaded RB-04 runner failures above;
  the first A/B attempt with a relative binary path (exit 127, removed and
  rerun).
- Tolerances: none changed; no golden regenerated. The coefficient-event
  checker was extended (a new operation with its own invariants), not
  relaxed.
- Not performed: private scenes; Teseo; other platforms (CI will run the
  new `[rollback]` cases on every lane).

## Publication and reproducibility

- Rebuilt targets: `polyfem`, `PolyFEM_bin`, `unit_tests` (hashes above).
- Committed files: see the implementation commit (recorded below).
- Companion pins unchanged (`c24d803e`, `bce32a39`); no HDA source or asset
  change (the hook is a JSON test option the asset does not expose).
- Remaining working-tree changes: none intended.
- Local evidence not distributed: the evidence directory (logs, VTUs,
  manifests, the failed/partial runs); the checked-in `tests/test_step_rollback.cpp`,
  `tools/rb06/` and this record are the reproducible part.

## Next session handoff

- Completed: inventory, R1/R2 reproductions, the transaction, the test
  hook, the in-process recovery test, the E2E matrix, the disabled-hook
  identity, publication.
- Status: validated within stated scope — every acceptance item of the plan
  (equality of authoritative state, equivalent subsequent solution/forces,
  coherent cache reconstruction, no stale success callback/output, error
  details preserved, success and exception paths tested) is measured on the
  public fixtures; the exclusions above are the scope.
- No model decision or external prerequisite is pending.
- Next eligible: RB-07 (bounded AL stagnation) can now build on
  `NLProblem::save_state/restore_state` and the fingerprint for its
  "restore accepted state" acceptance item; RB-23; RB-24. RB-08 stays
  closed; the transaction does not reopen it.
