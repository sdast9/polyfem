# RB-20 — Force-continuation κ (per-contact coefficient carried from the published endpoint)

Date: 2026-09-11
Status: **done — implemented, default on, validated with RB-21 on the public fixtures and ball-on-plate, published**. Companion: [RB-21](rb-21-parent-keyed-kappa.md). See the [progress log](#progress-log).

This file is both the plan and the running record, like [RB-18](rb-18-quick-fixes.md).
It is written so that a session that loses its context can resume from the log
alone. Its companion item is [RB-21 parent-keyed κ](rb-21-parent-keyed-kappa.md);
the sequencing decision between the two is in [Decisions](#decisions).

## Contract and authorization

- **User selection (2026-09-11):** after RB-18/RB-19 and the RB-17 retirement,
  implement the two remaining items from the RB-18 handoff — force-continuation
  κ (this item) and parent-keyed κ (RB-21) — with a written plan that is kept
  up to date so work can be recovered after a context loss. Both are
  implementation items with the design already characterized (RB-04 H5, RB-15,
  RB-16); the user's stated validation set is ball-on-plate, the five smokes and
  the RB-04 refinement matrix, with the criterion that the post-refresh residual
  actually shrinks.
- **Policy decisions still in force (2026-09-11):** keep IPC; keep per-contact
  Hessian scaling; keep the global gap band (trim controller) as the production
  controller; no per-contact band targeting (RB-17 retired); Fixed mode stays a
  reference; AL contact stays deferred.
- **Invariant:** at a between-steps refresh of the semi-implicit coefficients,
  the barrier force at the published endpoint must not change through
  re-estimation of a persisting contact's coefficient. Only (a) contacts new at
  the endpoint and (b) an explicit, logged global trim action may change the
  contact force at unchanged coordinates. Coefficient events remain accounted
  as parameter changes, never as displacement work (RB-04 convention).
- **Boundaries preserved:** CCD, the trial-displacement cap, the retired floor,
  the RB-18 coefficient law for *new* contacts (positive median, floor/cap,
  F2/F7 curvature fallbacks, F4 errors), the trim controller's rules, F5 stall
  handling, F6 lagged friction, the classic `adaptive` path (bit-identical), and
  PolySolve. No toolkit change in this item (RB-21 owns the builder).

## Baseline and reproduction

- PolyFEM `main` at `8846eb19f` (RB-19). IPC `af317a65` (local
  `ipc-toolkit-fork`), PolySolve `5afe3b5d` (local `polysolve-merged`), both
  clean. Build `polyfem/build`, macOS arm64, RelWithDebInfo, TBB.
- **The defect, already measured (RB-04 refinement, `lower-0.5-dt-0.25`,
  `coefficient-events.jsonl`, phase `between_steps_after_endpoint`):** at each of
  the four between-steps refreshes the active count (49/46/45/45) and the trim
  (2/4/4/8) are unchanged by the refresh, yet the contact-force vector norm at
  the unchanged endpoint moves 2.258e4→2.446e4, 5.912e4→6.744e4,
  1.014e5→1.172e5, 1.512e5→1.776e5 (+8% to +17%), and the barrier energy
  3.359→3.639, 9.930→11.330, 21.850→25.270, 25.338→29.764. The endpoint had
  ~1e-7 free residual under the old coefficients; under the new ones it is not
  an equilibrium. This is the RB-04 "post-publication force drift" (research log
  2026-09-09: +70.8 energy, 145746 force norm on the pilot). The metric is
  already recorded per event as `free_contact_force_change_norm` and
  `objective_change_at_fixed_coordinates`; no new instrumentation is needed to
  measure the fix.
- Evidence directory: `outputs/rb-20/<timestamp>/` in the parent workspace,
  created at the start of Stage 1.

## Design

### What "force continuation" is, precisely

The barrier force on contact i at coordinates x is
`f_i(x) = weight · trim · w_i · κ_i · ∇b(d_i(x)²)`, where `w_i` is the toolkit's
area/duplicate weight, `κ_i` the per-contact `stiffness_scale` and `trim` the
global multiplier. At fixed x and fixed trim, continuing the *realized force at
the realized gap* is exactly continuing `κ_i`. Therefore:

> At a refresh at coordinates x, every stencil active at x that already has a
> resolved coefficient keeps it. The Hessian estimate (with the full RB-18
> resolution law) is used only for stencils that have no coefficient at x.

No force extraction from the diagnostic snapshot is required; the snapshot stays
observational. "Realized" means the value that actually acted at x — the
resolved `stiffness_scale` (after floor/cap/`kappa_min`), not the raw memo.

### Decisions

Defaults are chosen here so work can proceed; each is a one-line change if the
user prefers otherwise.

- **D1 — Default on.** `solver.contact.semi_implicit.force_continuation: true`.
  This changes every semi-implicit result (the smokes' endpoints will move); the
  RB-04 drift is a measured defect, not an experimental candidate, so the repair
  is on by default as RB-18's fixes were. `false` restores the current behavior
  exactly and is the control in every comparison below.
- **D2 — Continue κ_i, not trim·κ_i.** The global trim keeps acting on every
  contact as today (collapse bump, band step, calibration). Continuing the
  *product* would silently neutralize the kept gap-band controller for every
  persisting contact. Consequence: at a converged endpoint the between-steps
  refresh becomes a no-op for persisting contacts *and* for the trim —
  `calibrate_trim` at equilibrium with unchanged ∇B returns the current trim
  (the drift measured above happened with the trim unchanged; under D2 it
  vanishes). The band step (`avg gap > trim_upper·d̂` → trim/2) and the collapse
  bump remain explicit, logged trim actions.
- **D3 — Optional bounded pull toward the Hessian estimate.**
  `continuation_max_ratio` (default `0` = pure continuation): when `r > 1`, a
  persisting contact's fresh Hessian estimate may move its coefficient within
  `[κ_old / r, κ_old · r]` per refresh. Pure continuation is the RB-15
  recommendation (frozen coefficients on stable interactions); the ratio option
  exists so the refinement matrix can measure what tracking a stiffening tangent
  buys, without making that a default.
- **D4 — Continued values are exempt from the batch floor/cap.** They were
  realized; clamping them to the *fresh* batch's median statistics would be a
  drift. Batch median/floor/cap (F1) are computed over the fresh estimates only
  (fallback: over all values, when there is no fresh estimate, only for the F2/F7
  sentinels). `kappa_min` still applies to everything.
- **D5 — Identity is the stencil key for this item.** `(type, v0..v3)` as
  today. A contact whose closest feature switches (EV↔VV, FV↔EV) at the
  endpoint is a *new* stencil and gets the Hessian estimate, i.e. exactly
  today's jump — no worse than now. RB-21 replaces the key with the parent
  candidate and makes continuation survive the switch; this item is built so
  that RB-21 only changes the key function and the lookup.
- **D6 — Separation forgets.** Only stencils *active at the refresh point*
  are continued. Stencils in the memo from line-search trial states but not
  active at x are fresh next time; a contact that separates and re-contacts
  gets a fresh estimate. Recorded as a limitation (RB-21's parent registry is
  the place to extend memory if wanted).
- **D7 (revised 2026-09-11) — Memory is captured at the published endpoint
  only.** The first version continued at *every* refresh, which at step 1
  "continued" values the birth refresh had estimated from the pre-contact
  snapshot — never realized anywhere — and produced 12 stall restarts on a run
  that has none. Now: the between-steps refresh (`update_barrier_stiffness`,
  `published_endpoint=true`) captures the resolved coefficients of every active
  stencil into `endpoint_kappa_`; every following refresh (birth, stall retune,
  interval) re-seeds exactly those and re-estimates everything else fresh, as
  today. A mid-solve refresh therefore still re-estimates contacts no endpoint
  has vetted, and F5's "unchanged" detection keeps its meaning.
- **D8 (amended 2026-09-11) — RB-20 code first, but RB-21 before RB-20 can be
  default-on.** See the 19:30Z log entry: with stencil identity, continuation
  makes the closest-feature-switch jump larger (a continued value from the old
  snapshot meets a fresh value from the new one) and Newton cost rises 7–20×.
  Original text: RB-20 first, RB-21 second.** RB-20 is fork-local PolyFEM
  and gives the drift removal immediately; RB-21 touches collision construction
  in the toolkit and is validated on its own (bit-identical `adaptive` smoke as
  the safety net). Doing RB-21 first would delay the largest payoff behind the
  riskier change. Alternative considered: RB-21 first so continuation is born
  parent-keyed — rejected for that reason; the D5 key abstraction keeps the
  later swap small.

### Code plan (PolyFEM only)

`BarrierContactForm.{hpp,cpp}`:

1. Options: `force_continuation_` (bool), `continuation_max_ratio_` (double)
   parsed from `semi_implicit` JSON; spec entries in `json-specs/input-spec.json`
   with doc strings; README defaults block updated.
2. In `refresh_semi_implicit_stiffness`, before the memo swap: build
   `continued_` = `{stencil_key(i) → collision_set_[i].stiffness_scale}` for
   every active i whose key is in `kappa_cache_` (i.e. has a resolved value that
   acted at x), when `force_continuation_`. Then swap/clear as today and seed
   `kappa_cache_` with the continued values (so `assign_collision_stiffness`
   hits the cache and never re-estimates them).
3. `assign_collision_stiffness`: unchanged for cache hits. When
   `continuation_max_ratio_ > 1`, on a continued key compute the fresh estimate
   too and clamp it into `[κ/r, κ·r]`, replacing the seeded value (done once,
   in the refresh's first pass; the memo then holds the clamped value).
4. `resolve_stiffness(kappa, continued)`: continued values skip floor/cap; the
   refresh's re-resolve loop and the batch median use the fresh set only (D4).
   `kappa_min` last, for all.
5. Diagnostics: `diagnostic_state()` gains `continued_count`, `fresh_count`,
   `continuation_max_ratio`; the refresh debug line reports them; the coefficient
   event stream is unchanged (it already carries the drift metric).
6. F2's `prev_kappa_cache_` fallback for nonpositive fresh curvature stays as is
   (it is a *new* stencil that happens to have a value from a trial state).

`tests/test_semi_implicit_coefficients.cpp`, new case `[semi_implicit_coefficients][continuation]`:

- (a) point-over-edge contact, refresh at x with provider H, then refresh again
  at the same x with provider 4H: continued κ unchanged; control with
  `force_continuation=false` gives 4× (the current behavior).
- (b) barrier gradient at x is bit-identical before/after the second refresh
  when the trim is unchanged (the invariant), and scales exactly with the trim
  when the trim is bumped between refreshes (D2).
- (c) a stencil not active at the refresh point is not continued (D6).
- (d) `continuation_max_ratio=2` with provider 100H gives 2κ; with provider H/100
  gives κ/2.
- (e) continued values bypass floor/cap: batch `{continued 1e-9, fresh 100, 100}`
  keeps 1e-9 (today's F1 floor would raise it to 100/spread).
- (f) new contact at the second refresh gets the fresh estimate.

### Validation (final, with RB-21's parent identity; evidence `outputs/rb-20/20260911T182152Z/`)

All runs use the final binaries (`tested-binaries.sha256`) unless marked
"stencil-key build". "Drift" is the maximum relative change of the contact
force vector at the unchanged published endpoint over the run's
between-steps refresh events with the trim unchanged.

| Check | Input/configuration | Expected criterion | Result |
| --- | --- | --- | --- |
| New regression | `tests/test_kappa_continuity.cpp` `[kappa_continuity]`, seeds 1–3 | pass | **3 cases / 66 assertions**, all three seeds |
| Affected suite | RB-18/19 tag list + `[kappa_continuity]`, seed 1, default-on build | no assertion failure | **55 cases / 3,415 assertions**, exit 0 (`tests-final/affected-tests.log`; also 55/3,415 on the default-off build, `tests/`) |
| Drift, quasistatic matrix | `tools/rb20/run_drift_matrix.py`, dt .25/.125/.0625 × band .1/.5/.8 | drift → 0; completion and Newton cost not worse than the re-estimating control | **on: 9/9 complete, drift ≤2.9e-16, 433 total Newton iterations, 0 restarts**; off (parent identity): 9/9, drift 17.4/8.5/4.2%, 431 iterations (`matrix-parent-on`, `matrix-parent-off`; baseline binary `baseline-quasistatic` 434) |
| Drift, stencil-key build (historical identity) | same matrix, `matrix-on-stencil` (opt-in build `0324ce096`) | measured | 8/9, drift ≤2.9e-16 but **5,274 Newton iterations, 45 restarts, 1 failure** — the RB-21 motivation |
| Drift, transient | `transient-semi`, band .5, dt .25/.0625 | same | on: drift 2.9e-16/3.3e-16, Newton 26/74; off: 17.4/4.2%, 26/73 |
| Drift, friction | `quasistatic-semi-friction`, band .5, dt .25/.0625 | same; F6 lag re-based at solve start as today | on: drift 1.9e-16/2.7e-16, Newton 49/152; off: 18.2/4.2%, 37/134 (+12/+18 iterations with continuation) |
| Optional pull | quasistatic, band .5, dt .125, `continuation_max_ratio=2` | measured | drift 8.53e-2 = the off value: every per-step estimate change on this fixture is < 2×, so the pull takes the fresh value; the ratio only bites on larger jumps (`matrix-ratio2`) |
| Five public smokes, new defaults | `run-smoke.sh`, default (parent + continuation) | four steps, zero error lines; `adaptive` bit-identical; semi-implicit endpoints differ by design | all exit 0, 5 saved steps; **`adaptive` 2.2e-16 vs RB-19** (Linf identical); `quasistatic-semi`/`-alhess`/`transient-semi` max endpoint diff **1.1e-4** (mostly normal, 32 of 1,540 nodes > 1e-4); `quasistatic-semi-friction` **1.57e-2** tangential on a .25 displacement (1,308 nodes > 1e-3; trim history 2,2,2,4,4,8,8,**16** vs …,8): a different stick/slip endpoint once coefficients stop growing with load (`final-smokes/`, `final-endpoints.json`) |
| Smokes, decomposition | same scenes with `coefficient_identity=stencil, force_continuation=false` and with `force_continuation=false` only | historical settings reproduce RB-19; identity-only effect isolated | historical: ≤1.2e-14 vs RB-19 (bit-level); parent identity alone: frictionless ≤2e-16, transient 7e-15, friction 4.0e-7 — the endpoint change is entirely continuation (`smokes-stencil-off`, `smokes-parent-off`) |
| Ball-on-plate | `tools/rb20/run_ball_on_plate.py`: user scene `test_cases/input/params.json` (E 1e6 plate / 1e11 ball, d̂ 1e-5, transient) bounded to 8 steps at its own dt .3 | both complete; drift and cost reported | **on: 8/8 steps, drift ≤6.5e-4 (only where the trim calibration moved; 2e-17 otherwise), 656 Newton iterations, 7 restarts, 40 s, min gap .31 d̂; off: 8/8, drift 13–53% per step, 628 iterations, 5 restarts, 38 s, min gap .35 d̂** (`ball-on-plate-on`, `-off`) |
| HDA E2E | `hython tests/test_polyfem_hda.py` | pass | `PASS: end-to-end PolyFEM 2.0 HDA test`, exit 0 (`hda-e2e.log`) |
| Toolkit tests | standalone fork build, `[parents]` | pass | 27 assertions (see RB-21) |
| Formatting / diff / links | changed files | clean | clang-format clean (PolyFEM and toolkit styles), `git diff --check` clean, links checked |

Physical accuracy, the full suite, Teseo and other private scenes are out of
scope. A completed scene is numerical termination, not an equilibrium
certificate; the drift metric is a fixed-coordinate force identity, not a
physical balance.

## Publication

- `0324ce096` — RB-20 opt-in implementation (stencil identity; the cost
  finding), `21f9fd592` — RB-21 PolyFEM side, then the final commit (default
  on, pin bump to toolkit `e3c8d3fe`, records) on `sdast9/polyfem:main`; hashes
  in the log.
- Toolkit `e3c8d3fe` on `sdast9/ipc-toolkit:semi-implicit-stiffness`.
- Parent README, plan rows, semi-implicit README updated.

## Next session handoff

- **Behavior changes to be aware of (both default on):** every semi-implicit
  run now keeps each contact's coefficient from its first pricing on; the
  global trim is the only between-step change. Endpoints differ from RB-19's
  (1.1e-4 frictionless, 1.6e-2 friction on the public smokes). Any
  semi-implicit golden recorded before RB-20 will differ;
  `force_continuation: false` (and `coefficient_identity: "stencil"`) restore
  the previous behavior bit-for-bit.
- **Friction:** the friction smoke's stick/slip endpoint moved by 6% of the
  prescribed displacement and its trim ended at 16 instead of 8. Neither
  endpoint is certified; RB-10 (friction coupling) is where that comparison
  belongs.
- **Limits:** separation forgets (re-contact is priced fresh); a contact born
  mid-step is priced by that step's start snapshot; `continuation_max_ratio`
  is unexercised below 2× on the public fixture; the improved-max
  (convergent) formulation's seam half-jump is analyzed but not measured;
  physical accuracy is not claimed anywhere here.
- Eligible next items: RB-05 (candidate/resource failure containment), RB-10
  (friction), RB-09 references.

## Progress log

Append-only. Newest entry last. Each entry: date/time (UTC), what was done,
what was measured, what is next.

- **2026-09-11 18:35Z** — Plan written after reading `BarrierContactForm`
  refresh/assign/resolve/post_step, the VarForm's endpoint diagnostics and
  `update_barrier_stiffness` between-steps entry, the RB-04 event stream (drift
  metric already recorded) and the RB-15 contract. Registered as RB-20 in
  `robustness-plan.md` and the parent README together with RB-21. Next: create
  the evidence directory, capture a baseline of the drift metric on the
  quasistatic refinement matrix (continuation off = current binary), then
  implement steps 1–6.

- **2026-09-11 18:50Z** — Evidence directory `outputs/rb-20/20260911T182152Z` (baseline binary
  `3b05266b…`, source `ffc83ac0d`). New runner `tools/rb20/run_drift_matrix.py`
  extracts the drift metric per between-steps refresh event. **Baseline
  (continuation absent), quasistatic 3 dt × 3 bands: 9/9 complete** (RB-19
  removed the step-1 stalls), every refresh event has unchanged trim and active
  count, and the relative contact-force change at the unchanged endpoint is
  **17.4% (dt .25), 8.5% (dt .125), 4.2% (dt .0625)** for all three bands —
  proportional to the load increment, the κ re-estimation signature
  (`baseline-quasistatic/results.json`). Next: implement code-plan steps 1–6.

- **2026-09-11 19:30Z** — Implemented (steps 1–6 of the code plan; `published_endpoint`
  flag on `refresh_semi_implicit_stiffness`, `endpoint_kappa_`/`continued_keys_`,
  `resolve_stiffness(kappa, continued)`, D4 fresh-only batch statistics, D3 pull
  at endpoint refreshes only, spec entries, diagnostics). **Two findings:**
  1. D7 as first written was wrong (birth refresh continued unrealized values →
     12 stall restarts on `lower-0.5-dt-0.25`); revised to endpoint capture,
     after which step 1 is identical to the baseline (9 iterations, 0 restarts).
  2. **Drift is removed: max relative contact-force change at unchanged trim is
     1e-16 on all nine quasistatic runs** (`matrix-on-stencil/`; baseline
     17.4/8.5/4.2%). But total Newton iterations rise from 26–76 to 174–1412,
     restarts appear (0 → up to 9) and `lower-0.8-dt-0.0625` fails after 20
     restarts. Trace (`trace-on/`): the slow solves cycle (‖∇f‖ 1 → 1000 → 1
     every few iterations) with **no** fresh stencil being created — a vertex
     toggles between two memoized face-vertex/edge-vertex stencils whose
     coefficients now differ by continued-vs-fresh snapshot, i.e. the RB-15
     subfeature-switch jump, amplified. This is the "the two go together"
     coupling the user anticipated. Decision: keep RB-20 **opt-in** (default
     `false`) so `main` is unaffected, do RB-21 now, then validate both
     together and flip the default. Control run (`matrix-off/`) reproduces the
     baseline (Newton counts within thread noise, identical drift).
  Evidence: `outputs/rb-20/20260911T182152Z/` (`baseline-quasistatic`, `probe-on-first`, `probe-on-second`,
  `matrix-on-stencil`, `matrix-off`, `trace-on`). Regression tests not yet
  written (they belong with the parent-keyed identity). Next: RB-21.

- **2026-09-11 20:40Z** — With RB-21's parent identity (see its log) the
  stencil-key cost problem is gone: parent identity + continuation gives
  **433 total Newton iterations vs 434 baseline, 0 restarts, drift 1e-16 on
  9/9** (`outputs/rb-20/20260911T182152Z/matrix-parent-on`). Semantics clarified by the regression
  tests: a contact born during a solve is priced by that solve's snapshot
  (birth refresh for a first contact, the step-start snapshot otherwise) and
  is continued from its publication on — at a published endpoint every active
  contact already has a value, so a fresh estimate at an endpoint refresh only
  occurs when continuation is off. Default flip to `force_continuation: true`
  is pending the remaining validation (RB-21 log lists the steps).

- **2026-09-11 22:10Z** — **Default flipped to on** after the full validation
  (table above): affected suite 55/3,415 on both defaults; smokes exit 0 with
  `adaptive` bit-identical and the semi-implicit endpoints moved by
  continuation alone (1.1e-4 frictionless, 1.57e-2 friction — decomposed
  against the historical settings, which reproduce RB-19 to 1e-14); transient
  and friction drift 1e-16; ball-on-plate drift 13–53% → ≤6.5e-4 at equal
  cost; HDA E2E pass. `continuation_max_ratio` measured (no effect below 2×).
  Toolkit `e3c8d3fe` pushed; PolyFEM pin bumped. Committed and pushed as the
  final RB-20/RB-21 commit (hash recorded below).
