# RB-20 — Force-continuation κ (per-contact coefficient carried from the published endpoint)

Date: 2026-09-11
Status: **planned — implementation not started**. See the [progress log](#progress-log); resume from the first stage not marked `done`.

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
- **D7 — Applies at every refresh, not only between steps.** Birth refresh,
  stall retune and `refresh_interval` refreshes continue active contacts too;
  a refresh then changes only the trim and new contacts. Consequence for F5: a
  stall retune with nothing new and an unmoved trim is correctly "unchanged"
  and the second one interrupts. The `[al_solver]`/stall tests must still pass.
- **D8 — Sequencing: RB-20 first, RB-21 second.** RB-20 is fork-local PolyFEM
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

### Validation plan

| Check | Input/configuration | Expected criterion | Status |
| --- | --- | --- | --- |
| New regression | `[continuation]` sections above, seeds 1–3 | pass | pending |
| Affected suite | RB-18/19 tag list, seed 1 | no assertion failure; F5 stall cases still pass under D7 | pending |
| Drift metric, public fixture | RB-04 refinement runner (`tools/rb04/run_refinement.py`), quasistatic × dt .25/.125/.0625 × band .1/.5/.8, continuation **on** vs **off** (off = today's 8–17%) | `free_contact_force_change_norm` at every `between_steps_after_endpoint` refresh event is 0 when the trim is unchanged, and equals `(trim_new/trim_old − 1)·‖f‖` when the band moved it; completion 9/9 both ways (RB-19 removed the step-1 stalls) | pending |
| Drift metric, transient and friction | same runner, `transient-semi` and `quasistatic-semi-friction`, dt .25/.0625, band .5 | same criterion; friction: lagged forces re-based at solve start as today (F6 covers the trim only) | pending |
| Optional pull | quasistatic, band .5, dt .125, `continuation_max_ratio` ∈ {0, 2} | measured: drift, final reaction, Newton steps, min gap; no promotion claim | pending |
| Five public smokes | on (default) and off | four steps to t=1, zero error lines both ways; `adaptive` bit-identical; semi-implicit endpoints **differ by design** — magnitude reported with the trim histories | pending |
| Ball-on-plate | `test_cases/input/params.json` (user scene, E=1e6 plate / 1e11 ball, d̂=1e-5), bounded to 8 steps as in the writeup, semi-implicit, on vs off | both complete; per-step Newton iterations, restarts, min gap/d̂, and the between-steps drift metric reported; wall time | pending |
| HDA E2E | `hython tests/test_polyfem_hda.py` | pass | pending |
| Formatting / diff / links | changed files | clean | pending |

Physical accuracy, the full suite, Teseo and other private scenes are out of
scope. A completed scene is numerical termination, not an equilibrium
certificate; the drift metric is a fixed-coordinate force identity, not a
physical balance.

## Publication

One implementation commit on `sdast9/polyfem:main` (code, spec, README, tests,
`tools/rb20/` runner/compact results, this file), then a hash-recording
documentation commit. Parent README and plan row updated. No companion change.

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
