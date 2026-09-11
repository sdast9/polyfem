# RB-18 — Quick-block robustness fixes (coefficient law, stall loop, friction lag)

Date: 2026-09-11
Status: **done — implemented, validated on the public fixtures, published** (`e3fa362e0`, `576b1d3d0`, `4a0df80a1` on `sdast9/polyfem:main`). See the [progress log](#progress-log) at the bottom.
Selected stage: all six fixes below, as one bounded implementation item.

This file is both the plan and the running record. Each fix has a status line
that is updated as work proceeds. A future session should read the progress log
first, then resume from the first fix not marked `done`.

## Contract and authorization

- **User selection (2026-09-11):** after reviewing RB-01–RB-17, the user asked for
  the "quick block" of fixes recommended in that review — the mechanical,
  fork-local repairs that each remove a failure class RB-02 reproduced — to be
  implemented now, with a plan file that tracks progress for handoff.
- **Explicit policy decisions taken by the user in the same session:** keep IPC;
  keep per-contact Hessian scaling; keep the gap band; Fixed mode stays a
  reference; AL contact stays deferred. The larger items (force-continuation κ,
  parent-keyed κ, per-contact band retirement) are **not** in this item.
- **Invariant:** the semi-implicit coefficient law must never silently delete a
  contact's barrier (κ=0), never silently substitute an invalid value, and must
  be unit-covariant; the stall handler must not repeat identical restarts; lagged
  friction must see the same normal-force scale as the barrier it was lagged
  against.
- **Boundaries preserved:** CCD, the trial-displacement cap, the retired floor,
  AL's feasibility-preparation role, PolySolve's configured convergence
  contract, friction lag *policy* (number of lag iterations, tolerance), and the
  classic `adaptive` path (must remain bit-identical on its smoke).
- **Dependencies read:** RB-02 contract/validation (all six defects and their
  probe fixtures), RB-04 (friction lag H6, post-publication drift — not fixed
  here), RB-13 (units of κ: force/length³), RB-16 (line-search roundoff — not
  fixed here, it is PolySolve), the semi-implicit writeup, and the current
  `BarrierContactForm`, `ALSolver`, `FrictionForm` sources.

## Baseline and reproduction

- PolyFEM `main` at `8f954f27d`. **Incoming uncommitted work from the RB-17
  session** is present and must be preserved untouched: modified
  `docs/rb-17-validation.md`, `docs/robustness-plan.md`; untracked `tools/rb17/*`.
  RB-18 commits must not include those paths unless the user says so.
- Effective IPC: local `ipc-toolkit-fork`, `semi-implicit-stiffness`, `af317a65`.
  Effective PolySolve: local `polysolve-merged`, `iteration-callback`, `4d372fa8`.
  Both clean. No companion change is planned for this item.
- Build: `polyfem/build`, macOS arm64, RelWithDebInfo, TBB, existing
  `Eigen::SimplicialLDLT` in the public smokes. No running solver/build at start.
- Evidence directory: `outputs/rb-18/20260911T164356Z/` in the parent workspace.
- Baseline reproduction of each defect is RB-02's standalone probe
  (`tools/rb02/coefficient_probe.cpp`), whose assertions currently *lock in* the
  defective behavior. Each fix flips the corresponding assertion to the repaired
  expectation; the pre-fix probe run is retained as the reproduction.

## The six fixes

Status vocabulary: `pending` → `in progress` → `implemented` (code written) →
`tested` (regression passes) → `done` (validated + committed).

### F1 — Positive-only median and relative κ floor

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-02):** `refresh_semi_implicit_stiffness` takes the median over *all*
per-contact κ including zeros; with a zero median the cap
`kappa_spread * median` is zero and every contact is clamped to zero
(`[0,0,100] → [0,0,0]`). A single contact with κ=0 is also possible (F2).

**Where:** `BarrierContactForm.cpp`, refresh, the `nth_element` block after
`assign_collision_stiffness`.

**Change:**
- `kappa_median_` = median of the **positive finite** κ in the batch. If there
  are none, `kappa_median_ = 0` and no cap/floor is applied (log a warning).
- Cap stays `kappa_spread * kappa_median_`.
- New relative floor `kappa_median_ / kappa_spread` applied to every contact in
  the batch (so a contact can never be more than `kappa_spread` below the
  batch median). Unit-covariant because it is relative. `kappa_spread ≤ 0` or
  non-finite disables both cap and floor, as today.
- Guard: if `kappa_spread * kappa_median_` overflows to `inf`, treat as "no
  cap" (already the effective behavior) and log once.

**Test:** RB-02 probe cases "zero median batch" (`[0,0,100]` must stay
`[·,·,100]` with the two zeros raised to the floor `100/spread`) and "positive
median / cap" (unchanged `[1,10,20]` at spread 2, floor 5 → `[5,10,20]`; the
probe's expected value updates accordingly). Also a Catch case in
`test_contact_cache.cpp` or a new `test_semi_implicit_coefficients.cpp`.

### F2 — Nonpositive curvature → previous κ, else batch floor; never zero

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-02):** `ipc::semi_implicit_stiffness` returns `wᵀHw` (mass term is
zero here); for indefinite or singular local curvature it is ≤ 0 and
`std::max(kappa, kappa_min_=0)` leaves κ=0: barrier deleted, CCD becomes the
only protection (the 1e-13 grind).

**Where:** `assign_collision_stiffness`, after the `kappa /= dhat²` line.

**Change:**
- Keep the previous snapshot's cache as `prev_kappa_cache_` (swap before
  `kappa_cache_.clear()` in refresh).
- If `kappa <= 0`: use `prev_kappa_cache_[key]` if present and positive; else mark
  the stencil as "needs floor" (store `-1` sentinel in the working value) and,
  after the batch median is known (F1), assign it the batch floor
  `kappa_median_ / kappa_spread` (or the median itself if spread is disabled).
  If the whole batch is nonpositive and `kappa_min_ > 0`, use `kappa_min_`;
  otherwise leave zero and warn — that case is a genuine model failure, not
  something to paper over.
- `kappa_min_` keeps its meaning as an absolute user floor applied last.

**Test:** RB-02 "indefinite curvature" and "zero / singular curvature" cases
(negative-definite `H` must now give the previous κ if any, else floor);
new case: a contact whose curvature turns negative on the second refresh keeps
its first-refresh κ.

### F3 — Normalize `conditioning_cap` by d̂²

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-02):** first-contact conditioning uses
`cap = conditioning_cap * max|H| / (weight * kappa_median)`. κ carries
force/length³ (after the d̂² division), `H` force/length, so the ratio has
units length² and the trim is length-unit dependent (measured trims .001/1/1
at L=.001/1/1000).

**Where:** refresh, first-contact branch.

**Change:** `cap = conditioning_cap * max|H| / (weight * kappa_median * dhat²)`.
Dimensionless: `trim·κ_median·weight·dhat²` is the barrier's effective interface
stiffness at gaps ~d̂ (force/length), compared to `max|H|`. Numerically
identical at d̂=1, so RB-02's L=1 probe values are unchanged. At d̂=1e-3 (public
smokes) the cap becomes 1e6× **tighter** than today, i.e. the cap will now
actually bind at first contact where before it never could. **This is a
behavior change on the public smokes**; the plan is to measure it, not hide it
(see validation). The doc string in `input-spec.json` and the scenes README are
updated to say the cap is in units of `max|H|`.

**Test:** RB-02 "controller converted scales" case: trims must now be equal
across L=.001/1/1000. Smoke endpoint comparison before/after recorded.

### F4 — Non-finite curvature: NaN is an error, overflow uses the cap; check after all arithmetic

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-02):** `if (!isfinite(kappa)) kappa = 1e30;` silently accepts NaN
and inf; the check precedes `kappa /= weight_`, so `weight_` = 0 or subnormal
produces a non-finite assigned scale that passes the check; finite `spread ×
median` can overflow to an infinite cap.

**Where:** `assign_collision_stiffness`, refresh.

**Change:**
- NaN from the Rayleigh quotient → `log_and_throw_error` naming the stencil
  key and the offending Hessian block. A NaN Hessian block means the Newton
  system is already NaN; failing loudly is the honest outcome.
- `+inf` (overflow of finite huge entries) → treated as "invalid" → previous
  κ if available, else the batch cap once it is known (it is the stiffest
  admissible value; RB-02 measured that the cap is exactly what such crushed
  elements need).
- Move the finiteness check to **after** `kappa /= weight_`, and add a
  constructor-time check that `weight_` is finite and positive (throw
  otherwise).
- Overflow of `kappa_spread * kappa_median_` → handled in F1.

**Test:** RB-02 "NaN / infinity / finite arithmetic overflow" cases: NaN must
throw; inf must give the cap (or previous κ); the "nonfinite after weight
division" case must throw at construction; "cap multiplication overflow" must
retain a finite κ and report no-cap.

### F5 — Do not repeat identical stall restarts

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-04 refinement, RB-16):** `ALSolver::minimize_with_stall_restarts`
calls `on_stall` and restarts up to `max_restarts` (20) times. When
`retune_on_stall` finds an empty collision set it returns without changing
anything; a mid-band stall with no calibration signal also leaves the trim
untouched. The restart then re-runs the identical problem from the identical
iterate; six RB-04 refinement runs burned 20 restarts this way at step 1 with
zero active contacts.

**Where:** `ALSolver.cpp` (`minimize_with_stall_restarts`),
`BarrierContactForm::retune_on_stall`, `NonlinearElasticVarForm.cpp` on_stall
lambda, `ALSolver.hpp` signature.

**Change:**
- `retune_on_stall` returns `bool changed`: true if the collision set was
  non-empty (the refresh re-evaluates κ_i at the current x, which is a change
  even if the trim is not moved) **or** the trim moved.
- `on_stall` becomes `std::function<bool(const VectorXd&)>`; a null/void
  adapter for existing callers (tests) returns `true` to preserve their
  behavior.
- In the loop: track `consecutive_unchanged`. One restart with no change is
  still allowed (a fresh solver instance resets L-BFGS/history, which can help
  on its own). A second consecutive unchanged stall from the same iterate
  interrupts immediately with `termination_reason =
  "stall persisted with no retunable contact state"` and
  `solve_info_["unchanged_restarts"]`. A hard-stall revert to the initial
  solution counts as a change of iterate and resets the counter.
- No change to `max_restarts` semantics otherwise; a scene that could
  legitimately be rescued by retunes still gets its full budget.

**Test:** `test_al_solver.cpp`: new case where `on_stall` returns false —
budget 20 must stop after 2 restarts with the new reason; existing cases with
`on_stall` returning true keep their counts (`retunes == budget`).

### F6 — Lagged friction follows the trim

**Status:** done (validated; committed in `e3fa362e0`).

**Defect (RB-02, RB-04 H6):** `FrictionForm::update_lagging` bakes
`barrier_stiffness()` (the trim) and per-contact `stiffness_scale` into the
lagged normal-force magnitudes. When the in-solve controller bumps the trim
(`bump_trim`, calibration, conditioning cap, stall retune), the barrier's
normal force changes but friction still uses the old magnitudes until the next
explicit lag update. RB-02 measured: tangential potential 11.688 before a trim
doubling, unchanged immediately after, 23.376 after re-lagging.

**Where:** `FrictionForm.hpp/.cpp`.

**Change:**
- Record `lagged_trim_ = contact_form_.barrier_stiffness()` in
  `update_lagging`.
- In `value_unweighted`, `first_derivative_unweighted`,
  `second_derivative_unweighted`, multiply by
  `s = contact_form_.barrier_stiffness() / lagged_trim_` **only when the
  contact form is a semi-implicit `BarrierContactForm`**. The friction potential
  is linear in the normal-force magnitude, so the scale is exact for value,
  gradient and Hessian. Outside semi-implicit mode `s ≡ 1` and the `adaptive`
  smoke stays bit-identical.
- Per-contact κ_i changes at a refresh are **not** covered by this scalar (the
  refresh already triggers a re-lag at solve start; mid-solve refreshes with
  `refresh_interval > 0` remain documented as inconsistent). Recorded as a
  limitation.

**Test:** RB-02 "lagged friction" case: tangential potential must be 23.376
immediately after the trim doubling. Existing friction-coupled-to-frozen-κ FD
derivative tests must still pass (they exercise the scaled derivatives when
run with a bumped trim — add one such section).

### F7 — Nonpositive curvature with no history: |wᵀHw|, then max|H|/d̂²

**Status:** done (validated; committed in `4a0df80a1`).

**Decision (user, 2026-09-11):** option B with E as fallback, chosen from the
five alternatives presented (A keep zero, B |wᵀHw|, C PSD-projected block,
D `kappa_min`, E global Hessian scale). This closes the one path to κ=0 that
F2 left open.

**Change** (`assign_collision_stiffness`, after the previous-value lookup):
- `q < 0` finite → κ = |q| (B). The magnitude of the local curvature is used
  as the barrier scale even though the elasticity is not providing it along
  the normal; heuristic, not a derivation, counted in
  `diagnostic_state()["curvature_abs_fallback_count"]`.
- `q == 0` → κ = `max|H| / (d̂² · weight)` (E). Same normalization as the
  conditioning cap; counted in `curvature_global_fallback_count`. Zero only if
  the system Hessian is identically zero (then `curvature_fallback_count`).
- Precedence: valid positive q → previous value → B → E → batch floor/cap →
  `kappa_min`. Previous value wins over B for continuity between refreshes.
- Overflow with no reference still throws (F4); E is not applied there
  because max|H| is then itself overflowing.

**Consequence for the fixtures:** the historical `{0,0,100}` batch is now
resolved by E to `{100,100,100}`, so it no longer exercises the F1 floor; the
F1 fixtures were changed to a tiny positive outlier `{100,100,1e-12}` →
`{.01,100,100}`. The RB-02 probe's `negative` and `singular_zero_normal`
curvature cases now expect 100; `zero` still expects 0; `floor_control` uses a
zero block. Both fixture changes are recorded in the probe comments.

**Tests:** three new sections in `[semi_implicit_coefficients]` (B, E with
d̂²/weight normalization, identically-zero, B over floor, previous over B).

## Validation plan

| Check | Input/configuration | Expected criterion | Status |
| --- | --- | --- | --- |
| Baseline RB-02 probe (pre-fix) | `tools/rb02` runner against unchanged build | 257 checks pass (locks in defects) | done: 257/257 at `8f954f27d`, `baseline-probe/` |
| RB-02 probe with flipped expectations (post-fix) | same runner, rebuilt | all checks pass with repaired expectations; flipped assertions listed in the progress log | done: **245/245** (`final-probe/`; the count changed because the arithmetic block was rewritten) |
| New Catch regressions | `[semi_implicit_coefficients]` (new file), `[al_solver]` (new case) | pass, across seeds | done: 5 cases / 54 assertions; 12 cases / 880 assertions; `[semi_implicit_coefficients]` also run with seeds 1–8 |
| Affected suite | `[contact_cache] [contact_stiffness_mapping] [semi_implicit_coefficients] [al_solver] [physical_diagnostics] [direction_filter] [restart] [form_derivatives] [floor_retirement] [bc_scale] [contact_form] [friction_form]`, seed 1 | no assertion failure | done: **52 cases / 3,338 assertions**, exit 0 (`affected-tests.log`) |
| Classic `adaptive` smoke | `quasistatic-adaptive.json` | Linf `0.24282997612162013`; endpoint within run-to-run noise of baseline | done: Linf identical; endpoint max abs diff 7.6e-16 vs baseline, run-to-run noise of the same binary 1.6e-16 (`final-smokes/`, `final-smokes-repeat/`) |
| Semi-implicit frictionless smokes | `quasistatic-semi`, `quasistatic-semi-alhess`, `transient-semi` | four steps to t=1, zero error lines; endpoints compared to baseline | done: all exit 0, 5 saved steps; max abs diff vs baseline 1.2e-16 / 1.4e-16 / 1.7e-16 (noise level); identical refresh-trim histories |
| Semi-implicit friction smoke | `quasistatic-semi-friction` | four steps to t=1, zero error lines; endpoint difference **expected** from F6 and reported | done: exit 0; max abs endpoint diff **4.8e-3** (max\|u\| .25); Linf .24247822746660083 → .2424763066234872; run-to-run noise 1.7e-14, so the change is real; post-lag-update residuals fell 3596.56 → 2646.22 (step 3) and 3717.85 → 3497.47 (step 4) |
| F3 binding on the smokes | `Conditioning cap on first contact` log count | measured, not assumed | done: **0 events before and after** — the plan's prediction that F3 would bind at d̂=1e-3 was wrong: the cap is only consulted when `calibrate_trim` fails (unloaded contact), and these scenes make contact under load, so calibration succeeds and the cap is never reached. F3 is verified by the probe/regression at L=.001/1/1000 (trim 1e3 at every scale), not by the smokes |
| HDA E2E | `houdini_HDAs/tests/test_polyfem_hda.py` via `hython` 22.0.429 | pass | done: `PASS: end-to-end PolyFEM 2.0 HDA test`, exit 0 (`hda-e2e.log`) |
| Formatting / links / diff | changed files | clean | done: clang-format clean, `git diff --check` clean |

Physical accuracy, full suite, private scenes and Teseo are **not** in scope.

## Publication

One implementation commit on `sdast9/polyfem:main` containing the C++ changes,
tests, probe updates, spec/README doc strings and this file; a following
documentation commit if this file needs the implementation hash. No companion
pin change. Parent-workspace README updated locally.

## Progress log

Append-only. Newest entry last. Each entry: date/time (UTC), what was done,
what was measured, what is next.

- **2026-09-11 16:44Z** — Plan written. Baseline state recorded above. Registered
  as RB-18 in `robustness-plan.md` and in the parent README. Evidence directory
  created. Next: capture pre-fix RB-02 probe run and a pre-fix set of the five
  smokes as the comparison baseline, then start F1.

- **2026-09-11 17:05Z** — Baseline captured on the unmodified build (binary
  sha `8c3e413d…`, source `8f954f27d`, incremental rebuild confirmed no-op).
  RB-02 probe: 257/257 (`baseline-probe/`; locks in the defects). Five smokes:
  all exit 0, zero error lines, five saved steps each (`baseline-smokes/`).
  Endpoint summary in `baseline-endpoints.json` via the new
  `tools/rb18/summarize_smokes.py` (solution-array SHA-256 + max|u|; adaptive
  Linf `0.24282997612162013`). `Conditioning cap on first contact` fired **0**
  times in every semi-implicit smoke, confirming the F3 prediction that the
  cap never binds at d̂=1e-3 today. Next: implement F1–F4.

- **2026-09-11 18:20Z** — F1–F4 implemented in `BarrierContactForm.{hpp,cpp}`:
  `prev_kappa_cache_`, `kappa_floor_`, `kappa_fallback_count_`,
  `batch_first_pass_`; new `resolve_stiffness()` and `stencil_key()` helpers;
  NaN throws; subnormal/zero weight throws at assignment (not construction —
  `set_weight` can change it later); overflow with no previous value and no
  batch cap **throws** (an infinite coefficient only fails later with an
  infinite objective — this is stricter than the plan's "else cap", because
  there is no cap in that case); `conditioning_cap` divided by d̂²;
  `diagnostic_state()` gains `batch_median/batch_floor/batch_cap/
  curvature_fallback_count`. Deviation from plan recorded: the one remaining
  path to κ=0 is a batch in which **no** contact has positive curvature and
  no previous value exists (single-contact negative/zero fixtures); it warns
  and is counted. Using |wᵀHw| there would be a driving-curvature model
  choice (RB-02 contract) and was not taken.
  New `tests/test_semi_implicit_coefficients.cpp` (4 cases / 41 assertions),
  order-independent — the parallel broad phase does not order the collision
  set, which made a first version flaky across seeds; the RB-02 probe's new
  batch check was made order-independent for the same reason.
  RB-02 probe expectations flipped: zero-median batch `[0,0,100]` →
  `[.01,.01,100]`; `[1,10,1000]` at spread 2 → `[5,10,20]`; converted-scale
  controller trims equal across L (unbound 1, bound 1e3); NaN/inf/finite-
  overflow/tiny-weight/zero-weight all throw with checked messages; new
  overflow-with-batch (→ cap 200) and negative-after-positive (→ keeps 100)
  cases; cap-overflow retains a finite coefficient. Probe: **245/245**
  (`probe-after-f1f4-e/`; earlier attempts a–d retained with their failures).
  `input-spec.json` and `scenes/semi-implicit/README.md` doc strings updated.
  Next: F5.

- **2026-09-11 18:50Z** — F5 implemented. `retune_on_stall` returns `bool`:
  false when not semi-implicit, when the collision set is empty, or when the
  trim did not move **and** the re-evaluated per-stencil values equal the
  previous snapshot's (`kappa_cache_ != prev_kappa_cache_`, so a stall at an
  unmoved iterate counts as "no change" even though a refresh ran). `on_stall`
  is now `std::function<bool(const VectorXd&)>` (both VarForm and legacy
  callers return the retune result; test lambdas return true). `ALSolver`
  counts `unchanged_restarts`/`consecutive_unchanged`; a hard-stall revert
  counts as a change only if the iterate had actually moved. Second
  consecutive unchanged stall → `Interrupted` with
  `termination_reason = "stall persisted with no retunable contact state"`.
  New `[al_solver]` case with three sections (soft unchanged → 2 retunes;
  changed-in-between resets the counter; hard stall at an unmoved iterate →
  2 retunes). `[al_solver]`: 12 cases / 880 assertions pass. Next: F6.

- **2026-09-11 19:15Z** — F6 implemented. `FrictionForm` records `lagged_trim_`
  in `update_lagging` and `trim_scale()` = `barrier_stiffness()/lagged_trim_`
  (1 unless the contact form is a semi-implicit `BarrierContactForm`)
  multiplies value, gradient and Hessian. First attempt put the scaling
  check in `test_form_derivatives.cpp`'s state fixture, where friction is
  identically zero at x=0; moved it to `test_semi_implicit_coefficients.cpp`
  on the real point-over-edge contact (value/gradient/Hessian scale exactly
  by 2 after `bump_trim(2)`, re-base to 1 after `update_lagging`, central FD
  of the rescaled gradient at trim 1.5). `[semi_implicit_coefficients]`:
  5 cases / 54 assertions. RB-02 probe friction case now measures 23.3761933
  immediately after the doubling (was 11.6880966); probe **245/245**
  (`final-probe/`). Next: affected suite, five smokes vs baseline, HDA E2E.

- **2026-09-11 19:45Z** — Validation complete on the fixed binary (hashes in
  `tested-binaries.sha256`). Affected suite 52 cases / 3,338 assertions.
  Smokes: adaptive Linf identical and endpoint within run-to-run noise;
  frictionless semi-implicit scenes unchanged to ~1e-16; friction scene
  endpoint moved by 4.8e-3 (F6 — the lagged friction now follows the three
  in-solve trim bumps each run makes; the post-lag residuals dropped, which is
  consistent with friction being closer to the barrier's state, but is not a
  physical certification). **F3 did not bind on any smoke** (0 events before
  and after); the plan's prediction was wrong because the cap is only reached
  when gradient-balance calibration fails, and these scenes contact under
  load. HDA E2E passes. Formatting and diff clean. Next: commit and push.

- **2026-09-11 20:05Z** — Implementation committed as **`e3fa362e0`** and
  pushed to `sdast9/polyfem:main`. Note: the push also carried the previously
  unpushed RB-17 commit `8f954f27d` (remote was at `29a18c3f1`). The RB-17
  session's *uncommitted* files were not touched. This documentation update
  follows as a separate commit.

- **2026-09-11 20:40Z** — **F7 added** after the user chose option B (|wᵀHw|)
  with E (max|H|/d̂²) as fallback for nonpositive curvature with no history.
  Implemented with two new diagnostic counters. `[semi_implicit_coefficients]`
  5 cases / 65 assertions across seeds 1–3; RB-02 probe **243/243**
  (`final-probe-f7/`; count changed with the fixture rework). Affected suite
  52 cases / 3,349 assertions (`affected-tests-f7.log`). Five smokes: all
  exit 0; endpoints within run-to-run noise of the pre-F7 fixed binary
  (≤3e-14; `final-smokes-f7/`, `final-endpoints-f7.json`); neither B nor E
  fired in any smoke (no fallback log lines), as expected — no public scene
  has a nonpositive local block at first contact. README doc string updated.
  The "one remaining path to κ=0" in the handoff is now only an identically
  zero system Hessian.

- **2026-09-11 20:55Z** — F7 committed as **`4a0df80a1`** and pushed. Documentation
  hash update follows.

## Next session handoff

- All six fixes are implemented, regression-tested, validated on the public
  fixtures and (see the final log entry) published. Nothing in this item
  remains pending.
- **Behavior changes to be aware of:**
  - F6 changes friction results when the in-solve controller moves the trim
    (the public friction smoke endpoint moved by 4.8e-3 on a .25 displacement).
    This is the intended consistency fix, not a regression, but any friction
    golden recorded before RB-18 will differ.
  - F3 changes the first-contact conditioning cap only at d̂≠1 and only when
    the contact is unloaded at first refresh. Knob: `conditioning_cap`
    (dimensionless; raise to loosen).
  - F4 turns three silent substitutions into errors (NaN curvature, overflow
    with no reference, zero/subnormal weight). A scene that previously "ran"
    through one of these will now stop with a message naming the stencil.
  - F5 makes a stall with nothing to retune fail after two restarts instead
    of `max_restarts`; `solve_info["unchanged_restarts"]` and the termination
    reason `stall persisted with no retunable contact state` identify it.
- **κ=0 is now only possible for an identically zero system Hessian** (F7,
  user choice B+E). `diagnostic_state()` reports how many stencils used
  |wᵀHw| (`curvature_abs_fallback_count`), max|H|/d̂²
  (`curvature_global_fallback_count`) or the batch floor/cap
  (`curvature_fallback_count`) at the last refresh; a debug log line lists
  the same. B is a heuristic borrowing of curvature magnitude, not a derived
  compliance — if a scene relies on it heavily, that is worth knowing.
- **Known limitations not in scope here:** post-publication force drift
  (force-continuation κ, proposed as the next implementation item); EV/VV
  coefficient jump (parent-keyed κ in the toolkit builder); mid-solve κ_i
  refresh vs. lagged friction (only the trim is followed); line-search
  roundoff near convergence (PolySolve) and the root cause of the no-contact
  step-1 failures — **both addressed by [RB-19](rb-19-line-search-roundoff.md)
  (2026-09-11)**: the step-1 stall is thread-order energy noise at the roundoff
  floor, and the line search now falls back to the gradient norm there.
- **Incoming RB-17 working-tree changes** (`docs/rb-17-validation.md`, the
  RB-17 status row in `robustness-plan.md`, `tools/rb17/*`) were left
  uncommitted and untouched by RB-18; they were committed with the RB-17
  retirement in the RB-19 session (per-contact band targeting withdrawn).
- Eligible next items: force-continuation κ (new RB item, to be written),
  parent-keyed κ, RB-05.
