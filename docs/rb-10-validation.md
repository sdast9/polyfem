# RB-10 — Friction coupling and dissipation

Date: 2026-09-13 (session started 2026-09-12)
Status: **characterized—decision pending** (the lagged-friction implementation
is validated within the stated scope; one production choice — which normal
force the lag carries after a trim action — is presented below and not taken)
Selected stage: Stages 1–4 complete — code audit and unit-level reproduction,
coupled trajectory fixtures, predeclared budget/smoothing sensitivity, and the
A/B of the opt-in `friction_lag: realized_force` mode against the RB-18 F6
default. See the [progress log](#progress-log).

## Contract and authorization

- **User selection (2026-09-12):** "Let's now address RB-10" — the friction
  coupling and dissipation item of [the robustness plan](robustness-plan.md#rb-10--friction-coupling-and-dissipation).
  No scope or policy decision was given with it; every friction-law, default
  smoothing (`contact/epsv`), lag-budget (`solver/contact/friction_iterations`)
  or lag-tolerance change is a decision to present, not to take.
- **Invariant:** the user's finite lagging policy is preserved while its
  accuracy and dissipation are measured. The baseline is the default budget
  (`friction_iterations = 1`: one frozen-lag solve per step, the lag rebuilt at
  the returned coordinates only to measure the updated-lag residual), default
  `friction_convergence_tol = 0.01` (× characteristic length) and default
  `epsv = 1e-3`. More lagging iterations are a comparison axis, not a policy.
- **Success criterion (the item's acceptance):** (1) nonnegative frictional
  dissipation under the declared convention — RB-04's right-endpoint
  `frictional_dissipation_increment = g_f,solved-lag · Δx` per accepted step;
  (2) balanced forces at every returned endpoint — support reaction, barrier
  force and friction force sum to the recorded residual; (3) constitutive
  checks — the lagged friction force has magnitude `μ · N_lag · trim_scale` in
  the sliding regime and `μ N f1(|u|)` inside the smoothing scale, opposes the
  slip used by the potential, and its derivatives pass finite differences with
  the trim scale active; (4) the normal force the friction lag carries is the
  one the barrier actually supplies after every retuning path (trim bump,
  calibration, refresh, stall retune, between-steps refresh); (5) quantified
  sensitivity to a predeclared set of lag budgets and smoothing scales, with
  every failure kept in the matrix. An implementation inconsistency is repaired
  only after reproduction and only within the existing contract; anything that
  changes the friction law is presented with alternatives and measured
  consequences.
- **Dependencies read:** RB-04 record version 2 (`6252a9119`, done): the
  `physical-diagnostics.jsonl` endpoint record supplies the per-form gradients
  in force units, the support force `AᵀA r`, the free residual, the two sides of
  the final lag update (`friction_before_update` / `friction_after_update`), the
  updated-lag residual and `frictional_dissipation_increment`. RB-18 F6 (lagged
  friction follows the trim), RB-20 (force continuation), RB-21 (parent-keyed κ),
  RB-02 (lagged-friction trim case), PF-08's frozen-lag constitutive check
  (μ = .3, `epsv = .001 L`, budget 2, prescribed slip). **RB-09's reference
  protocol does not exist** (RB-09 not started). Per the plan, this item does
  the characterization that stays valid without it: the reference used here is
  closed-form Coulomb statics of a body on a *flat* obstacle — in steady sliding
  the total tangential friction force equals μ times the total barrier normal
  force exactly, whatever the material — which checks the implementation, not
  engineering accuracy against a continuum model. No physical accuracy
  threshold is selected here (that stays RB-09's).
- **Exclusions / decisions not authorized:** no change of friction law, of the
  slip convention, of `epsv`, `friction_iterations`, `friction_convergence_tol`
  or their semantics without the user's explicit agreement; no golden
  regeneration; no Teseo or private scene; no impact or stick–slip
  certification from steady sliding alone; no claim that a returned finite-lag
  endpoint is a coupled equilibrium. The one implementation added here
  (`semi_implicit/friction_lag`) is **opt-in with the default unchanged**; its
  default is the pending decision.

## Baseline and reproduction

- Session start: PolyFEM `main` at `382c4c21e` (code `377a83fda`), clean. While
  the Stage 1 probe was being written another session published the
  closed-RB review corrections (`c93cdd9c5`, `91c7ff8ac` on `sdast9/polyfem:main`;
  PolySolve `ee5b296a` on `sdast9/polysolve:iteration-callback`) and
  deliberately left this workspace's checkouts alone. Both checkouts were
  fast-forwarded (no local commits, untracked RB-10 files only) and the build
  redone before any scene evidence was produced; **every scene and test result
  below is on `91c7ff8ac`** (+ the uncommitted RB-10 changes for Stage 4 and
  the final validation). IPC `bb795446` on `semi-implicit-stiffness` (local
  `ipc-toolkit-fork`); both companions are the effective sources in
  `build/CMakeCache.txt` (`IPCToolkit_SOURCE_DIR`, `PolySolve_SOURCE_DIR`).
- Build `polyfem/build`, RelWithDebInfo, AppleClang (`/usr/bin/c++`), macOS
  arm64 (Darwin 25.5.0, 18 cores), TBB. Stage 2/3 binary (`91c7ff8ac`, no
  RB-10 code): `PolyFEM_bin` sha256 `6dfd6cff…b875`. Final binary (RB-10
  changes): `PolyFEM_bin` `5782c5d3…`, `unit_tests` `0abbec82…`
  (`outputs/rb-10/20260912T232617Z/baseline.txt`, `build-logs/`). Scene runs
  use `--max_threads 1` (RB-19: thread order is roundoff noise; recorded); the
  five public smokes use the standard threaded `run-smoke.sh`.
- Evidence: parent `outputs/rb-10/20260912T232617Z/` — `stage1-probe/`,
  `stage2-fixtures/`, `stage3-sensitivity/`, `stage3-classic/`,
  `stage4-friction-lag-ab/`, `unit-tests/`, `smokes/`, `hda/`; every run
  directory holds its isolated `scene.json`, mesh copies, `run.log`, `run.json`
  (exit, wall time, error-line count, saved PVD times, input/binary hashes) and
  `output/physical-diagnostics.jsonl`. Small published measurements:
  [tools/rb10/results-20260913.json](../tools/rb10/results-20260913.json)
  (per-run summaries and per-step scalars). Reproduction:
  [tools/rb10/README.md](../tools/rb10/README.md). The public
  `scenes/semi-implicit` inputs are read, never modified.
- **Retained failed attempt (fixture design error):**
  `stage2-fixtures/corner_coupled-quasistatic-v1-infeasible-wall/` — the first
  corner fixture used a full-height wall at x = 1.02 that the prescribed top
  face (x up to 1.2) must pass through; from the first wall contact (step 6)
  every Newton trial was clamped by the trial-displacement cap against CCD and
  the run made no progress. Killed after 148 s at 5 accepted steps (exit −15,
  248 stall/restart log lines). The fixture was replaced by a half-height wall
  (z ≤ .5) that the top passes over; see its `NOTE.md`.

### Code audit — the force state that is actually solved

`FrictionForm` (`src/polyfem/solver/forms/FrictionForm.{hpp,cpp}`),
`FullNLProblem::{init,update}_lagging` (fan-out to every form),
`NLProblem::update_lagging` (reduced → full), the lag loop in
`NonlinearElasticVarForm::solve_tensor_nonlinear` (lines ~1911–1975), the
toolkit's `TangentialCollisions::build` / `TangentialCollision::init` /
`TangentialPotential` (`ipc-toolkit-fork/src/ipc/{collisions/tangential,potentials}`).

- The friction set is built at the lag coordinates `x_lag` from a fresh
  `NormalCollisions` (same `dhat`, `dmin = 0`, area/improved-max/shape-derivative
  flags copied from the barrier form) with the barrier form's own potential
  (barrier type, physical-barrier flag, current trim) and, in semi-implicit
  mode, the per-contact `stiffness_scale` from the frozen coefficient memo.
  Each lagged collision stores `N_lag = κ_i · trim · |b'(d²)| · 2d` at `x_lag`
  (the toolkit multiplies `stiffness_scale` into `force_magnitude`), the
  tangent basis and closest point at `x_lag`, and the toolkit weight `w`
  (area/duplicate) which the tangential potential applies. The barrier's own
  force on the same stencil is `weight · trim · w · κ_i · ∇b`, so the lagged
  magnitude is the barrier's normal force at `x_lag` — consistent by
  construction at lag time (Stage 1 P1: equal to 1e-16).
- Potential: `D = Σ w · N_lag · μ · f0(|τ|)`, `τ = Tᵀ v`, gradient
  `+w N μ f1(|τ|)/|τ| · T τ` (the toolkit's `force` is its negative), so per
  collision `g · v = w N μ f1(|τ|) |τ| ≥ 0` exactly for the **velocity the
  potential is evaluated on**. `f0/f1` are the IPC smooth-friction mollifier
  with scale `epsv`; sliding (`|τ| ≥ epsv`) gives `|g_t| = w μ N`.
- The velocity: with a time integrator `v = compute_velocity(x)`
  (`(x − x_prev)/dt` for implicit Euler); the form divides the value by
  `dv_dx` and multiplies the Hessian by it, so value/gradient/Hessian are a
  consistent energy/force/stiffness triple. **Quasistatic time-stepped problems
  (`time/quasistatic: true`) keep the ImplicitEuler integrator** — only the
  inertia form is disabled — so their friction slip is the per-step increment
  (`acceleration_scaling = dt²`, RB-04). Only a *static* problem (no `time`
  block, `is_time_dependent() = false`, `time_integrator = nullptr`) uses the
  total displacement `x` as the "velocity" (`compute_surface_velocities`,
  upstream since `908ce3a43`); a static problem is a single solve with
  `x_prev = 0`, where that is the increment. Consequence recorded (not a
  defect): `epsv` is a displacement in a static solve and a velocity
  (length/time) in every time-stepped one (Stage 1 P3).
- RB-18 F6 `trim_scale() = barrier_stiffness()/lagged_trim_` multiplies
  value, gradient and Hessian in semi-implicit mode, so every in-solve trim
  action (collapse bump, calibration, band step, stall retune) is followed
  exactly at fixed coordinates; κ_i changes are not followed by the scalar.
  With `force_continuation` (default on) every stencil active at a published
  endpoint keeps its κ through subsequent refreshes, and the friction set only
  ever contains stencils active at the lag coordinates, so under the defaults
  the lagged κ_i is the barrier's κ_i for the whole solve; stale κ_i needs
  `force_continuation: false` (Stage 1 P2b).
- The lag loop (default budget 1): `init_lagging(x_prev)` at solve start (after
  the between-steps refresh, so κ/trim are current) → AL + reduced solve →
  `update_lagging(sol)` → updated-lag free residual vs `friction_convergence_tol
  · L` → budget exhausted → "Lagging failed to converge" warning, `state = not
  converged`, endpoint returned. The rebuilt lag is discarded at the next
  solve start. With budget ≥ 2 the re-solve is preceded by
  `nl_problem.init(sol)` and `update_barrier_stiffness(sol)`; in semi-implicit
  mode that is a published-endpoint refresh (continuation memory captured at
  the lag iterate, one trim-controller step) followed exactly by F6; in
  classic adaptive mode it re-estimates the single stiffness **after** the
  friction lag was built (upstream ordering) — not exercised by any fixture
  here (Stage 1 P2e, Stage 3 classic: the classic rule returned the same value
  before and after on both fixtures).
- The convergence tolerance compares an objective-gradient norm (force ×
  form weight units) against `friction_convergence_tol × characteristic_length`
  (length units); inherited from upstream, recorded as an observation for
  RB-11/RB-12, not changed here. `delta_x_norm ≤ 1e-12` (`prev_sol` = the
  incoming solution) stops the loop early on a stationary step with a "tiny
  update" warning instead of the budget warning; cosmetic.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Lagged normal force equals the barrier force at lag time; Coulomb magnitude `μ N` beyond `epsv`, IPC mollifier `μ N f1` inside; `g·v ≥ 0` for the potential's velocity; point/edge forces balance; FD gradient/Hessian with the trim scale active | Stage 1 P1: magnitude error 8.1e-17, `g·v` min 6.7e-3 > 0, balance 0, FD 4.3e-9 / 4.3e-13 (inside), 2.1e-12 / 5.0e-6 (outside) | validated (regression `[friction_lag]`) |
| Normal-force transfer through trim bumps, calibration, stall retune and the between-steps refresh (continuation) is exact | Stage 1 P2a/c/d: lagged = barrier force to 1e-12 after ×2 and ×.25 bumps, after calibration (trim .5), after `retune_on_stall` (trim 3.54, κ continued), after a published refresh at a new endpoint with a ×4 driving Hessian (κ 100 kept) | validated (regression) |
| A mid-solve refresh with `force_continuation: false` re-estimates a lagged stencil (κ 100 → 400) and leaves the lag at the old force | Stage 1 P2b: lag/barrier ratio .25 | reproduced; documented limitation of the non-default setting (regression asserts it) |
| Static (no integrator) slip is the total displacement; a held point keeps `μ N`, a reversal has `g·Δx < 0`; every time-stepped run uses the increment (`(x − x_prev)/dt`), a held body has no force and both directions dissipate; BDF2 evaluates the BDF velocity (a held body after a +s step sees `v = −s/2` and a `μ N` force) | Stage 1 P3 | not a multi-step defect (a static problem is one solve); `epsv` unit semantics differ by problem type — recorded, no change |
| Steady sliding on a flat obstacle: `|F_t| = μ N` | Stage 2, slide_plus/minus/moving_obstacle, both models, steps 9–20: solved-lag ratio [.993, 1.001], updated-lag ratio [.9924, 1.00000001]; support work = frictional dissipation to .05 % per step; `frictional_dissipation_increment ≥ 0` at all 280 accepted steps of the 14 fixtures | validated within stated scope |
| Stick phase and reversal: creep below `epsv · dt`, elastic unloading with the contact stuck | slide_plus steps 5–7: contact slip / top increment .015, .030, .076, then .71, .97, 1.00; reversal steps 13–15: slip ~0, updated-lag ratio .62/.35/.06, support work −1024/−563/−108 (returned), dissipation 2.9/3.3/9.2 ≥ 0, sliding back at .995 μN by step 20 | validated (physically consistent; no impact/stick–slip certification) |
| Finite-lag artifacts inherent to budget 1: friction absent in the first contact step, overestimated during separation by `N_lag/N_end`, absent in the recontact step | separation_recontact q: lift-off steps 9–11 solved-lag ratio 1.62, 2.70, 107 (endpoint pressure vanishing) with D still ≥ 0; recontact step 16: ratio 0, contact slip .0129 > top .0125 (one free step), .97 by step 20 | characterized; quantified by the budget sweep |
| Coupled normals (floor + half-height wall): both interfaces stay at or below `μ N_i` | corner_coupled q/t: floor interface ratio .35 → .76 with creep ≈ 7 % of the top increment, wall normal force to 1.17e6 with wall friction ratio ≤ .73; every step D ≥ 0; the wall's top edge is a sharp feature, so its interface "normal" is a barrier-force direction, not a plane normal | characterized |
| Zero friction: no tangential support | zero_friction_slide q: max `|S_x|/|S_z|` 7e-5 (slab-diagonal asymmetry); transient 1.2e-2 = inertial (mass 1000 × acceleration at slide start), D ≡ 0 | control passes |
| Balance identity | Every run: `W_support − Σ_forms g·Δx` ≤ 2.1e-6 (moving obstacle ≤ 92 per step: the recorded reaction uses the post-update friction on the obstacle DOFs, which carry the friction directly — an RB-04 bookkeeping convention, not a friction defect) | validated; bookkeeping note for RB-04/RB-09 |
| **F6 trim-following after an in-solve trim bump changes the friction capacity by the trim ratio although the equilibrium normal force is load-determined** | slide_minus step 18 (both models): the controller doubled the trim (2 → 4) after a Newton trial collapsed the minimum gap to .1 d̂; endpoint N unchanged (377,686 vs 378,718) but the solved-lag friction reached **1.30 μ N** and the contact stuck for the step (slip 1e-4 vs .0125, D 5.7), then caught up (slip .0249, D 2797); corner_coupled step 13: solved/updated ratio .43/.18, D 6.4. In the public friction smoke the bump happens in every step (2 → 4 → 8 → 16) | reproduced; **opt-in `friction_lag: realized_force` implemented for the A/B; default unchanged; decision pending** |

### Stage 3 — predeclared budget and smoothing sensitivity

Baseline budget 1, `epsv` 1e-3; declared before the runs: budgets {2, 4, 8} and
`epsv` {1e-2, 1e-1, 1} on `slide_plus` and `reversal`, quasistatic and
transient (24 runs, all 20 steps, exit 0).

| axis | slide_plus q: lag error (max over steps) | D cumulative | accepted Newton iterations (all minimizes) | reversal q: lag error | D cumulative | iterations |
| --- | --- | --- | --- | --- | --- | --- |
| budget 1 (default) | .148 | 19,128 | 208 | .245 | 9,324 | 178 |
| budget 2 | .0101 | 19,415 | 275 | .118 | 9,604 | 281 |
| budget 4 | 9.2e-5 | 19,448 | 347 | .0024 | 9,642 | 376 |
| budget 8 | 1.1e-6 | 19,448 | 382 | 1.0e-5 | 9,643 | 435 |
| `epsv` 1e-2 | .131 | 19,112 | 202 | .131 | 9,345 | 161 |
| `epsv` 1e-1 | .054 | 18,807 | 176 | .079 | 9,280 | 131 |
| `epsv` 1 | .0033 | 8,722 | 94 | .0091 | 6,558 | 92 |

The lag fixed-point converges geometrically (≈ ×10–100 per iteration after
the first); the default budget's cumulative dissipation is 1.7 % (slide) and
3.4 % (reversal) below the converged value at 54 % / 41 % of the converged
cost. The updated-lag free residual falls 1.05e4 → 857 → 22 → .19; the lag
tolerance (5e-4) is reached only inside budget 8 (`lagging_states` all
`converged`). `epsv = 1` puts the sliding velocity .25 inside the smoothing
scale: the steady sliding ratio is **.4375 = f1(.25)** in both fixtures and
both models (the smoothing regime at trajectory level), with 54 % less
dissipation and 55 % fewer iterations; `epsv` 1e-2/1e-1 stay in sliding
(ratio 1.0008/1.0002) and change the dissipation by −.1 % / −1.7 %. Transient
values match the quasistatic ones to ≤ .3 % throughout (inertia small at
ρ = 1000, dt = .05).

### Stage 4 — `friction_lag` A/B (opt-in `realized_force` vs the F6 default)

`solver/contact/semi_implicit/friction_lag`: `"follow_stiffness"` (default,
RB-18 F6 — lag built with the current trim at the lag gap, scaled by every
in-solve trim ratio) or `"realized_force"` (lag built in
`FrictionForm::update_quantities` at the accepted endpoint **before** the
between-steps refresh, kept by `init_lagging` at the same coordinates, no
trim scaling — the friction carries the normal force that acted there).
Default path unchanged: the 8 fixture pairs (new binary, default mode) are
**bit-identical** to the Stage 2 binary's endpoints, and the friction smoke's
`D_cum = 10311.4` matches RB-04's record (10311.361).

| fixture | mode | solved-lag ratio range | max lag error | D cumulative | support work | accepted iterations | max updated-lag free residual | endpoint difference between modes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| slide_plus q / t | both | identical | | | | | | 0 (no trim action after contact) |
| slide_minus q | follow | [.278, **1.303**] | 1.06 | 18,024.5 | 30,033.8 | 250 | 1.73e4 | 1.3e-2 at step 18, 3e-4 by step 20 |
| slide_minus q | realized | [.278, 1.002] | .040 | 18,062.8 | 29,606.7 | 227 | 1.05e4 | |
| slide_minus t | follow / realized | [.270, 1.316] / [.270, 1.001] | 1.06 / .045 | 17,895.6 / 18,058.6 | 30,171.6 / 29,706.9 | 240 / 218 | 1.75e4 / 1.05e4 | 1.3e-2 at step 19 |
| corner_coupled q / t | follow / realized | (stick) | 1.44 / .21 | 1,418 / 1,412 | 122,648 / 122,619 | 193 / 176 | 2.42e4 / 1.05e4 | 1e-3 at step 13 |
| separation_recontact q | both | identical | | | | | | 0 (the trim actions fall on friction-free steps) |
| separation_recontact t | follow / realized | lift-off step 11: in-solve band step halved the trim; follow halves the lag, realized keeps it | | 5,562 / 5,641 | 23,840 / 23,647 | 160 / 153 | | 7.9e-3 at step 11 |
| public friction smoke, budget 1 | follow / realized | (compression; bulge friction) | | **10,311 / 13,465** | 551,226 / 527,518 | 49 / 33 | 7.46e4 / 8.41e4 | 4.0e-3, 1.1e-2, **2.1e-2** (steps 2–4, on a .25 press) |
| public friction smoke, budget 2 | follow / realized | | | 21,752 / 20,743 | 547,782 / 546,831 | 60 / 62 | 1.10e4 / 1.10e4 | 1.1e-3, 3.1e-3, 5.7e-4 |

Reading: at fixed coordinates F6 is exact (Stage 1 P2), but the equilibrium
normal force of a load-controlled contact does not change with the trim — the
solve absorbs a trim change by moving the gap (slide_minus step 18: gap .539 →
.662 d̂, N unchanged). When the bump is a reaction to a Newton-trial collapse
or a band step at constant load (slide_minus, corner, separation), following it
gives a spurious ×2 / ×.5 friction capacity for the rest of the step (a stick
hiccup, a 30 % support-force overshoot, 9–13 % more Newton iterations, a 65 %
larger updated-lag residual). In the friction smoke the bump in every step is a
reaction to the load itself doubling, so following the trim partly anticipates
the load growth: the follow mode's solved lag overshoots the endpoint normal
force by ≈ 25 % while the realized lag trails it by ≈ 35 %, and the two modes
differ by 30 % in dissipation and 8 % of the prescribed displacement at the
endpoint. **A second lag iteration removes most of both**: at budget 2 the
smoke's modes agree to 5.7e-4 and both dissipate ≈ 21,000 — twice the
budget-1 value of either mode. Neither endpoint is certified.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail / not run |
| --- | --- | --- | --- | --- |
| Stage 1 probe | `tools/rb10/run_probe.py` on the `91c7ff8ac` build | all sections pass; measurements as above | 23 checks, 9 sections, `passed: true` (`stage1-probe/probe.json`) | pass |
| New regression | `unit_tests "[friction_lag]"` | constitutive response, retuning transfer (incl. the documented `force_continuation: false` staleness), transient held/reversal | **3 cases / 85 assertions** | pass |
| Affected suite | `[contact_cache],[contact_stiffness_mapping],[semi_implicit_coefficients],[al_solver],[physical_diagnostics],[direction_filter],[restart],[form_derivatives],[floor_retirement],[bc_scale],[contact_form],[friction_form],[friction_lag],[kappa_continuity],[resource_containment],[bc_metric],[contact_floor_retired]`, seed 1, final binary | no assertion failure | **71 cases / 4,313 assertions**, exit 0 (`unit-tests/affected-suite.log`; expected negative-test error lines only) | pass |
| Stage 2 fixtures | 7 fixtures × {quasistatic, transient}, budget 1, `epsv` 1e-3 | 20 steps each, exit 0, zero error lines | 14/14 complete; plus the retained infeasible-wall attempt (5 steps, killed) | pass (1 retained failure, fixture error) |
| Coulomb identity | flat-obstacle sliding, updated lag | `|F_t| = μ N` | ratio [.9924, 1.00000001] over 12 sliding steps × 6 runs | pass |
| Dissipation sign | all 14 + 24 + 2 + 20 runs | `g_f,solved-lag · Δx ≥ 0` every accepted step | min 0 (frictionless/no-contact steps), no negative step in 1,141 accepted steps (61 runs incl. the retained failed attempt) | pass |
| Balance | all runs | `W_support − Σ g·Δx` at the free-residual level | ≤ 2.1e-6 (moving obstacle ≤ 92, convention note above) | pass |
| Stage 3 sensitivity | 24 runs | complete; monotone lag convergence; smoothing ratio `f1(v/epsv)` | 24/24 complete; `.4375 = f1(.25)` at `epsv = 1` | pass |
| Stage 4 A/B | 20 runs (18 fixture pairs + smoke budget 2 pair) | complete; default bit-identical | 20/20 complete; 8 pairs identical to Stage 2 to 0.0 | pass |
| Five public smokes | copied `run-smoke.sh`, final binary, threaded | four steps to t = 1, zero error lines | all 5 exit 0, PVD times [0, .25, .5, .75, 1], 0 error lines (`smokes/smokes.log`) | pass |
| HDA end-to-end | Houdini 22.0.429 `hython tests/test_polyfem_hda.py`, final binary | pass | 7/7 PASS, exit 0 (`hda/test_polyfem_hda.log`; one pre-existing non-fatal descent-direction log line, also in the RB-05 log) | pass |
| Formatting / diff / links | `git diff --check`, spec JSON parse, Python compile, local links | no introduced issue | checked before publication | pass |

- Numerical termination versus independently measured residual: every fixture
  step ends on the configured gradient criterion (solved-lag free residual ≤
  6.9e-5); the updated-lag free residual is the finite-lag mismatch, 40–50 in
  steady sliding (`μ ΔN` between the lag and the endpoint) and up to 1.05e4 in
  the first sliding steps at budget 1, `state = not converged` at every
  friction step of every budget-1 run (the policy, RB-04's label).
- Metrics measured: support force on the prescribed DOFs (x, z totals),
  barrier and friction force totals per body and per obstacle interface,
  contact-node slip against the prescribed increment, right-endpoint work
  increments (support, friction, elastic/barrier energy change, retuning
  energy), min det F (.9156 corner, .974 sliders, .829 smoke), min gap, trim
  history, Newton iterations and minimize calls. Units: internal force/length/
  energy of the record.
- Missing: no continuum/engineering reference (RB-09); no physical acceptance
  threshold; no impact, high-speed or stick–slip-cycle fixture; classic
  adaptive lag-loop ordering not exercised (stiffness constant on both
  fixtures); per-node friction directions (only totals and obstacle sums are
  recorded); rigid-obstacle-only geometry (no deformable–deformable
  interface); one mesh, one material, one dt per fixture.
- Retained failures: the infeasible full-height-wall attempt (design error).
  No solver failure, restart or stall in the 60 completed runs
  (`restarts_total` 0 everywhere).
- Tolerances: the constitutive checks use 1e-12 relative (probe/test), FD 1e-6
  gradient / 1e-4 Hessian declared before the runs; the trajectory "sliding"
  classification threshold .98 on the updated-lag ratio is a reporting choice;
  no test tolerance was changed.
- Not performed: the full unit-test suite; other platforms; private scenes;
  Teseo.

## Publication and reproducibility

- Rebuilt targets: `PolyFEM_bin` and `unit_tests` at `91c7ff8ac` + this item's
  changes (`FrictionForm` realized-lag mode, `BarrierContactForm` option and
  diagnostic field, `input-spec` entry, `tests/test_friction_lag.cpp`,
  `tools/rb10/`).
- Committed files / remote / commit: recorded in the progress log at
  publication.
- Companion pins unchanged (IPC `bb795446`, PolySolve `ee5b296a` as pinned by
  `91c7ff8ac`). No HDA source change (the new option is not exposed pending the
  default decision).
- Working tree after publication: clean; evidence stays in
  `outputs/rb-10/20260912T232617Z/` (not distributed: 60 run directories with
  logs and records, ~200 MB).

## Next session handoff

- Completed: Stages 1–4 (audit, unit probe/regression, fixture matrix, budget
  and smoothing sensitivity, `friction_lag` A/B on the fixtures and the public
  friction smoke).
- Status **characterized—decision pending**: the acceptance items (1)–(5) are
  measured and pass within the stated scope, but item (4) surfaced a production
  choice — whether the lagged friction should carry the trim-scaled force at
  fixed coordinates (RB-18 F6, current default) or the force that acted at the
  lag coordinates (`realized_force`). Alternatives, units and consequences are
  in Stage 4; the friction smoke's endpoint moves 2.1e-2 (on .25) between them
  at the default budget and 5.7e-4 at budget 2.
- Decisions for the user: (a) the `friction_lag` default; (b) whether the
  default lag budget stays 1 (its dissipation deficit is 1.7–3.4 % on the
  sliders and ≈ 50 % on the compression smoke, at 54–59 % of the budget-2 cost)
  — the plan's invariant keeps it at 1 unless the user says otherwise; (c)
  whether to expose `friction_lag` (and the RB-04 diagnostics) in the HDA.
- Observations for other items: RB-11 — `epsv` is a displacement in a static
  solve and a velocity in time-stepped ones; the lag tolerance compares an
  objective gradient with a length; RB-04/RB-09 — the recorded reaction uses the
  post-update friction on prescribed DOFs (moving obstacles carry it directly).
- Next eligible: RB-09 references (this item's fixtures and analyzer are a
  starting protocol), RB-23, RB-06/RB-08.

## Progress log

Append-only. Newest entry last.

- **2026-09-12 23:26Z** — Session start. Read the plan's RB-10/RB-09 sections,
  the RB-04/RB-18/RB-20/RB-21 records and PF-08's frozen-lag experiment,
  `FrictionForm`, the lag fan-out, the VarForm lag loop, the toolkit's
  tangential build/potential. Recorded the baseline identity in
  `outputs/rb-10/20260912T232617Z/baseline.txt`. Wrote the contract and the
  code audit above. Next: Stage 1 unit probe (`tools/rb10/friction_probe.cpp`).
- **2026-09-13 00:30Z** — `origin/main` had moved (`c93cdd9c5`, `91c7ff8ac`,
  PolySolve `ee5b296a`, another session's closed-RB review follow-up).
  Fast-forwarded both checkouts, rebuilt (`build-logs/build-after-ff.log`),
  reran the probe: 23 checks pass. The initial audit statement that
  `time/quasistatic: true` has no integrator was wrong (it has one; only a
  static solve uses the total displacement) — corrected above; P3 documents
  the static path.
- **2026-09-13 00:45Z** — Stage 2 (14 runs) and Stage 3 (24 + 2 runs) complete;
  analyzer and summary written; the infeasible full-height-wall attempt
  retained and replaced. Found the F6 trim-following artifact at slide_minus
  step 18 and corner step 13.
- **2026-09-13 00:55Z** — Implemented the opt-in `semi_implicit/friction_lag`
  (`realized_force`), rebuilt, `[friction_lag]` regression 3 cases / 85
  assertions, Stage 4 A/B (20 runs), affected suite 71 cases / 4,313
  assertions, five smokes, HDA E2E all pass. Record written; status
  characterized—decision pending.
