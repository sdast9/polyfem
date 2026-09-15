# RB-07 — Bounded AL stagnation handling

Date: 2026-09-15
Status: **validated within stated scope** (the opt-in mechanism, off by
default) — **production default: decision pending** (enabling an exit by
default needs the user's choice, see
[Decision](#decision-required-production-default))
Selected stage: reproduction → proposal of a stage budget and a stagnation
metric → opt-in implementation (default disabled) → validation → publication.
No load/timestep subdivision (RB-08 stays closed), no AL stationarity or
gradient requirement, no change to the weights, the ceiling, the multiplier
update or the geometric snap gates.

## Contract and authorization

- User-selected item: "Let's proceed with RB-07" (2026-09-15); RB-07 was
  `not started`. The plan's section authorizes an opt-in budget with the
  default disabled; an enabled production default requires the user's
  agreement and is **not** chosen here.
- Invariant (plan): geometrically stagnant continuation must have an explicit
  exit, while useful interrupted AL iterates remain allowed under PF-01.
  Success criterion of this stage: (1) a prescribed motion that can never be
  snapped is reproduced on public inputs and shown to keep the AL loop
  running with no exit; (2) a stage budget and a stagnation metric are
  proposed with units, windows and explicit failure reasons, grounded in the
  measured pass histories; (3) the opt-in implementation ends the reproduced
  cases under the configured limit with a named failure, restores the
  accepted state (RB-06) and publishes nothing of the step; (4) a useful
  multi-pass continuation (more than three interrupted passes) succeeds with
  and without the budget with a byte-identical endpoint; (5) the zero-initial-
  BC-error and ceiling cases of PF-07 keep their behavior; (6) the checks
  distinguish exhaustion from a solver error; (7) with the budget off every
  public scene is bit-identical to the previous binary.
- Dependencies: RB-04 (the attempt stream, the failure record and the
  manifest carry the new per-pass fields; the attempt-stream identities are
  re-checked with the budget on), RB-06 (the step transaction restores the
  accepted state when the stage fails: `NLProblem::save_state/restore_state`
  and the fingerprint, unchanged), PF-01/PF-07 (interrupted passes stay
  continuation; the ceiling and zero-error semantics are untouched).
  Effective IPC `c24d803e`, PolySolve `bce32a39` (local overrides copied
  beside the isolated worktree), unchanged.
- Exclusions: no production default (decision below); the remeshing
  `L2Projection` AL (its own hard-coded ALSolver parameters; the budget is
  not installed there); the FSI/fluid/thermo/differentiable/legacy paths get
  the same option, but only the nonlinear-elastic VarForm path has RB-06's
  transaction: on the others the failure still ends the run with exit
  status 1 and no rollback or publication claim is made (not exercised — the
  public set has no AL fixture for them); the Conservative inversion check's quadrature side effect
  of the snap probe (miso builds only — this build forces the Discrete
  check); Teseo and private scenes (not run); the Houdini asset (no control
  added until a default is chosen).

## Baseline and reproduction

- PolyFEM `4a422d177` on `main` (clean tree; the user's own `PolyFEM_bin` run
  from `polyfem/build` was in progress in `test_cases/inflation`, so this
  session built and tested in an isolated worktree,
  `outputs/rb-07/20260915T195548Z/polyfem`, with copies of the two companion
  checkouts and the saved configure command — `configure-command.txt`;
  RelWithDebInfo, Apple clang 21.0.0, TBB, Python on, miso off; macOS 26.5.2,
  Darwin 25.5.0). Baseline binaries built at `4a422d177` before any edit:
  `PolyFEM_bin d8aab86c…`, `unit_tests c08dde2b…` (`baseline.txt`,
  `baseline-binaries/`).
- Evidence: `outputs/rb-07/20260915T195548Z/` — `reproduce-baseline/`
  (the three scenes on the baseline binary, `results.json` with commands,
  input hashes and pass histories, `pass-analysis.json`),
  `probe-collision-sequence/` (per-pass solution frames of the collision
  scene, `save_solve_sequence_debug`), `try-candidate-{1,2,3-crush}/`
  (development runs, kept), `validate-final-{a,b,c}/` (the validation
  matrix on the pre-format candidate) and `validate-final-formatted/` (on
  the formatted one), `ab-smokes/` and `ab-smokes-final/` (six public scenes
  baseline vs candidate, before and after the format pass), `tests-1/`,
  `tests-full/` (a killed partial run, see below), `tests-final/` and
  `tests-full-final/` (the main-tree build), `run-smoke-main-tree.txt`,
  `hda-test-polyfem.log`, `build-*.log`, `session-log.md`.
- Fixtures (`tools/rb07/run_al_stagnation.py`; public `quasistatic-semi.json`
  cube on a fixed slab 0.02 below it, NeoHookean E = 1e7, ν = .45, d̂ = 1e-3,
  semi-implicit barrier, one quasistatic step, `--max_threads 1`, RB-04
  diagnostics and RB-12 manifest on; input hashes in `results.json`):

  | scene | prescribed motion | AL settings | what the snap meets |
  | --- | --- | --- | --- |
  | `compatible-multipass` | top face (25 nodes) pressed 0.3 in one step | initial weight 1e4, scaling 2, ceiling 1e8, AL subsolves interrupted after 4 iterations (`allow_out_of_iterations`) | first passes: the snap inverts the top layer (energy) then sweeps it through the interior (collision); feasible after 105 passes |
  | `incompatible-collision` | bottom face prescribed 0.05 below rest (slab 0.02 below) | defaults (1e6, 2, 1e8) | the straight snap crosses the slab: collision-blocked, finite energy, valid |
  | `incompatible-crush` | top face prescribed down by the cube height | defaults | the snap inverts the elements under the face: energy not finite |

- Reproduced symptom (baseline binary, `reproduce-baseline/`, 300 s wall
  clock per scene):
  - `compatible-multipass`: exit 0 after **105 AL passes** (44.8 s). The
    weight reaches the ceiling at pass 15; from there the BC residual
    e = √Σ(Au−b)² (length units) falls from 0.059 to 0.021 with one regression
    at pass 33 (0.0375 → 0.054) and jumps to 0.067/0.084 in passes 104–105,
    after which the snap is feasible. The blocking gate is inversion (energy)
    on the first pass, then collision; the collision-free fraction of the
    snap rises from 0.02 to 0.57 over the ceiling passes (candidate record).
  - `incompatible-collision`: **no exit; killed at 300 s after 98 passes**.
    Initial residual 0.25 (√25 nodes × 0.05). The AL cannot lower the bottom
    face through the slab; instead it drags the slab's own penalized DOFs
    (obstacle nodes are AL-penalized to zero displacement, not fixed, in the
    full-space stage): after pass 1 the cube's bottom face sits at z ≈ −0.045
    and the slab's vertices at z = −0.025…−0.050 (rest −0.02;
    `probe-collision-sequence/`). The weight
    reaches the ceiling at pass 8; the residual then sits at 0.0494 ± 0.5 %
    for 50 passes (subsolves converged in 3–8 iterations, min gap
    3–5e-4 wandering inside the trim band), the multipliers growing linearly;
    from pass ≈ 60 the subsolves degrade (21 stall restarts per pass,
    gradient-descent fallback with ‖∇f‖ ≈ 1e9, ≈ 7 s per pass) and the
    residual grows to 0.089. Nothing in the loop can end this.
  - `incompatible-crush`: **no exit; killed at 300 s after 59 passes**.
    Residual 5 → 0.53 by pass 7 (converged 46–97-iteration subsolves) → from
    pass 13 every subsolve is stall-interrupted (21 restarts) while the
    residual keeps creeping down (0.68 → 0.19 at pass 49, back to 0.25 at
    pass 58): the elements under the face get thinner and thinner; the snap
    always inverts them.
  - Deterministic no-progress case: the synthetic wall problem of
    `unit_tests "[al_budget]"` (a log-barrier obstacle in front of the
    constraint target; every pass converges to the same fixed point once the
    free coordinate has settled) and the PF-07 zero-initial-error fixture
    (residual 0 → 0 while only the free coordinate moves).
- Negative control: the six public scenes never enter the AL loop (0 passes
  in every step; their snap is feasible at once), so they cannot show the
  defect and are the off-path identity check only.

### BC error alone is not a stagnation metric (measured)

`tools/rb07/analyze_passes.py` replays two BC-residual-only rules on the
baseline logs — *window-relative*: the last W passes at the ceiling and
e_k > (1 − 0.01) e_{k−W}; *running-min patience*: the last W passes at the
ceiling improved nothing over the best residual before them — and reports the
first pass each would have ended the stage:

| scene (baseline) | W = 3 | W = 5 | W = 10 | W = 20 | W = 40 |
| --- | --- | --- | --- | --- | --- |
| `compatible-multipass` (succeeds at pass 105) | 33 / 35 | 33 / 37 | 33 / 42 | 41 / 52 | 104 / none |
| `incompatible-collision` (stuck) | 10 / 10 | 12 / 12 | 17 / 17 | 27 / 27 | 47 / 47 |
| `incompatible-crush` (stuck, creeping) | 12 / 14 | 12 / 16 | 17 / 21 | 59 / none | 59 / none |

(first trigger pass, window-relative / running-min). Every window up to 20
passes ends the compatible continuation before its snap: the residual of a
continuation under interrupted passes oscillates and even jumps by ×3 right
before the gate opens. This is the plan's warning ("do not invent a
distance-to-feasibility metric from BC error alone") measured; the stagnation
rule therefore needs the geometric signals below.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| The AL loop has no exit for a snap that can never become feasible; at the ceiling the multipliers grow without bound until the subsolves degrade | `reproduce-baseline/incompatible-{collision,crush}`: killed at 300 s after 98 / 59 passes | reproduced; fixed by the opt-in budget |
| In the full-space AL stage obstacle DOFs are penalized, not fixed: an incompatible motion drags the obstacle along | `probe-collision-sequence/` frames | reproduced; documented (unchanged — the snap restores the obstacle, which is exactly the blocked step) |
| A useful continuation can need > 100 interrupted passes with a non-monotone residual | `reproduce-baseline/compatible-multipass` | reproduced; the rule must not stop it (it does not) |
| BC-residual-only stagnation windows of 3–20 passes produce false positives on that continuation | `pass-analysis.json`, table above | reproduced; the production rule adds gate, snap-fraction and drift signals |
| The RB-04 attempt observer would misread a step-bound probe as a Newton trial | code reading (`flush_pending_proposal`) | avoided: the probe is unobserved (`probe_step_bound`) |
| The RB-04 checker expected every failed attempt's last internal iterate to differ from the retained coordinates; an AL stage ended between passes retains the last pass's iterate | `check_solver_attempts.py` on `try-candidate-1` | checker taught the new failure shape; identities pass on all three candidate runs |

### The proposal (implemented, opt-in, default off)

`solver/augmented_lagrangian/budget` (`ALBudgetOptions`, `ALSolver::set_budget`,
installed on every forward path that reads the AL block):

| key | default | units | meaning |
| --- | --- | --- | --- |
| `max_passes` | 0 (unbounded) | passes | hard cap on AL passes (subsolves) per nonlinear solve; reason `pass_budget` |
| `stagnation_window` W | 0 (never) | passes | end the stage after W consecutive passes at the weight ceiling with no progress signal over the window; reason `stagnation` |
| `progress_tolerance` | 0.01 | dimensionless | BC residual e = √Σ(Au−b)² (length) must fall by this fraction over the window to count as progress |
| `snap_tolerance` | 0.01 | fraction of the snap | the collision-free fraction of the straight snap (the forms' step bound, CCD and inversion check) must rise by this much to count |
| `drift_tolerance` | 0.01 | dimensionless | the full-space iterate must drift (L∞) by more than this fraction of the largest constrained-DOF residual over the window to count; with a zero residual any drift counts |

Progress over a window [k−W, k] is any of: the residual fell by
`progress_tolerance`; a snap gate (finite energy, validity, collision-free)
turned from blocked to passing; the CCD fraction rose by `snap_tolerance`;
the iterate drifted by more than `drift_tolerance` × max constrained
residual. The window must lie entirely at the weight ceiling — below it the
weight still grows and PF-07's continuation is by design still working — and
is judged against the state before it (pass 0 = the state the stage started
from). Neither exit reads a gradient norm, requires AL stationarity, changes
the weights, the ceiling, the multiplier update, the `eta < 0` rollback or
the snap checks: with the budget disabled the loop is the historical one
(the gates are evaluated with the historical short circuit and no probe;
bit-identical frames, see Validation). With a budget every gate is evaluated
for the record and, while the snap is blocked, the unobserved
`NLProblem::probe_step_bound` prices its collision-free fraction (one CCD
sweep per pass; the attempt stream is unchanged — the RB-04 observer reads a
proposal that carries a step bound as a Newton trial, so the probe emits no
`StepBound` observation).

Explicit failure reasons: `ALBudgetExhausted` (a `std::runtime_error` with
structured `details()`: reason, budget, passes, weight ceiling, the reference
and last pass records, the progress flags and the full pass history) is
thrown from the top of the AL loop with the swept candidates released; the
log message states the passes, the ceiling, the residual before/after the
window with the tolerance, the blocking gate with all three verdicts and the
CCD fraction before/after, the drift against the largest residual, and the
last subsolve's outcome. `NonlinearElasticVarForm` attaches the details to
the subsolve state at failure, so the RB-04 failure record
(`termination.subsolve_state_at_failure.al_stagnation`) and the manifest step
(`steps[].al_stagnation`, plus per-AL-subsolve `al_pass`, `al_at_ceiling`,
`al_bc_residual`, `al_relative_progress`, `al_rolled_back`, `al_moved`,
`al_gate`) carry them; the RB-06 transaction then restores the accepted state
(verified) and the run ends with exit status 1 (a named failure, RB-05).

Why the checks distinguish exhaustion from a solver error: a solver error is
a subsolve that threw (NaN energy, iteration limit, a line search that failed
three passes in a row) and is reported as before — the stage rethrows at
once, or after three absorbed retries whose passes carry `subsolve.outcome =
failed` in the history; a budget exit happens *between* passes with the last
subsolve converged or interrupted (usable, PF-01) and the failure record says
which gate blocked the snap. Within the budget exits, `pass_budget` with a falling
residual and changing gates means the cap was too small for the step (the
crush fixture: residual still moving, drift progress every window),
`stagnation` means the loop reached a fixed point (the collision fixture:
residual flat at −0.3 % over the window, fraction 0.008 → 0.011, drift 8e-5
against a 0.03 residual).

Per-pass record (`al_history`): pass, weight, at_ceiling, subsolve
{outcome, termination_reason, iterations, restarts, error}, bc_residual
(measured after the pass, PF-07's "before any rollback"),
bc_residual_carried (after the `eta < 0` rollback), relative_progress (η),
rolled_back, gate {finite_energy, valid, collision_free, feasible,
blocked_by, ccd_fraction}, snap_linf (largest constrained residual), moved
(L∞ motion of the carried state), drift_over_window, multiplier_norm,
wall_seconds.

Alternatives considered: (a) the pass cap only — unambiguous but it cannot
tell a slow success from a fixed point, so the record carries the signals the
user needs to choose a cap; (b) a BC-residual-only window — measured false
positives above; (c) multiplier growth as the divergence signature of an
unattainable constraint — it is the residual plateau in another guise (λ
grows by ρ√W·r per pass either way) and adds nothing to (b); (d) an absolute
iterate-motion tolerance in length units — fixture-dependent (the collision
fixture's gap wanders by 1e-4 per pass forever); the relative drift against
the snap length is what is implemented.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Status |
| --- | --- | --- | --- | --- |
| Baseline reproduction | `reproduce-baseline/`, baseline binary, 300 s each | incompatible scenes do not stop; compatible one succeeds after > 3 interrupted passes | collision: killed at 300 s / 98 passes; crush: killed at 300 s / 59 passes; compatible: exit 0 / 105 passes | reproduced |
| Off/on equivalence on a continuation the budget does not exhaust | `validate-final-a/compatible-multipass-{off,on}`, `{max_passes: 200, stagnation_window: 3}` | same passes, exit 0, byte-identical endpoint, no `al_stagnation` | 105 / 105 passes, exit 0 / 0, `step_1.vtu` byte-identical (also to the baseline binary's `reproduce-baseline` frame, `860cde46…`), manifest `al_stagnation` null | pass |
| Stagnation exit, collision-blocked | `validate-final-a/incompatible-collision`, same budget | named failure under the limit, reason `stagnation`, window at the ceiling with no progress signal, exit 1, rollback verified, nothing published | stopped at pass 10 in 8.2 s (passes 8–10 at 1e8: residual 0.04936 → 0.04952, fraction 0.0079 → 0.0110, drift 7.8e-5 vs 0.0297), exit 1, manifest `failed_attempt`/`augmented_lagrangian`, rollback performed + verified, no `step_1`, PVD `[0]`, RB-04 record announces the rollback; 20/20 checks | pass |
| Pass cap, energy-blocked crush | `validate-final-b/incompatible-crush`, `{max_passes: 60, stagnation_window: 3}` | named failure at the cap, reason `pass_budget`, rollback verified | stopped at pass 60 in 257 s, reason `pass_budget` (the window never fired: the drift over every 3-pass window, 0.03–0.3, exceeded 1 % of the 0.16–0.6 largest residual — the crush keeps moving), gate `energy` throughout (finite energy false, valid false, collision-free false, CCD fraction 0.05), residual 5 → 0.27, exit 1, rollback performed + verified, nothing published; 15/15 checks | pass |
| Pass cap alone, collision | `validate-final-c/incompatible-collision`, `{max_passes: 30}` | reason `pass_budget` at pass 30, rollback verified | stopped at pass 30 in 16 s, reason `pass_budget`, gate `collision`, residual 0.0494 at the ceiling (the flat plateau the message describes), exit 1, rollback verified; 15/15 checks | pass |
| Unit regressions | `unit_tests "[al_budget]"` | options parsing; pass cap with history; unexhausted budget on the PF-07 fixture (zero and nonzero initial error) identical to no budget, gates probed only under a budget; stagnation window on the synthetic wall (all four progress flags false, converged passes, plateau at 0.5 confirmed under a cap-only run); public transient fixture driven 1.5 below the slab at step 2: `ALBudgetExhausted`, accepted state and integrator history restored, no callback, manifest + RB-04 record | 5 cases / 194 assertions | pass |
| Affected selection | `[al_solver],[rollback],[bc_metric],[bc_scale],[direction_filter],[iteration_observer],[al_continuation],[input_validation]`, semi-implicit contact/friction derivatives | all pass | 55 cases / 3672 assertions (`tests-1/affected-suite.log`) | pass |
| Full suite | `unit_tests` on the main-tree build `7a8d0eab…` (`tests-full-final/`) | only the known failures (`large-mass-ratio` 2D+3D, `gcp-contact/cube-on-floor`, `stretch-cubes`) | 355 cases / 352 passed / 3 failed, 5,158,785 assertions / 4 failed: `standard` (`multi-material/stretch-cubes`, RB-22), `contact_3d` and `contact_2d` (`large-mass-ratio` 2D+3D at the RB-10 defaults, `gcp-contact/cube-on-floor`) — the same three cases and four assertions as RB-06's 347/350 run, plus the five new `[al_budget]` cases; 2 h 12 min wall | pass (known failures only) |
| RB-04 attempt-stream identities with the budget on | `check_solver_attempts.check_run` on `try-candidate-1/incompatible-collision`, `try-candidate-2/compatible-multipass-on`, `try-candidate-3-crush/incompatible-crush` | start/accepted/rejected rows, build-count identity, feasibility-check count = passes + 1 | 79 accepted / 11 feasibility checks (10 passes); 534 / 107 (105 passes + the reduced solve's check); 11041 / 61 (60 passes) | pass |
| Affected smokes (off path) | `ab-smokes/`: the five public scenes + `rb03-hex-q1`, baseline vs candidate, `--max_threads 1` | exit 0, 0 error lines, every frame byte-identical | 6/6 exit 0/0, 0/0 error lines, 5 frames each byte-identical | pass |
| Formatting / diff | clang-format 23.1.0 (`--style=file`) on the changed C++ files, `git diff --check` | clean | clean | pass |

- Numerical termination versus measured residual: a budget exit is never a
  termination of the reduced solve; the reduced solve's configured criteria
  (PF-01) are unchanged and the `[al_solver]` tests that pin them pass.
- BC/gap/inversion metrics: the BC residual e (length units, Euclidean over
  the constrained DOFs, obstacle DOFs included), its L∞ (`snap_linf`), the
  three snap gates, the CCD fraction of the snap and the min contact distance
  (log) per pass; no reaction or work balance is claimed for a failed stage.
- Missing: an AL stage on the private scenes (not run); the fluid/FSI/thermo
  paths with a budget (option installed, not exercised: those loops have no
  AL fixture in the public set).
- Retained partial runs: the two baseline timeouts (the reproduction), the
  development runs `try-candidate-1` (budget 12/3: the compatible scene
  correctly exhausted at 12 — a cap is a cap) and `try-candidate-3-crush`,
  and `tests-full/full-suite-killed-at-app-quit-partial.log` — a full-suite
  run on the pre-format worktree build that the desktop app's quit killed
  after ≈ 50 min without a summary (not counted; the suite was rerun on the
  final build, `tests-full-final/`).
- Not performed: full physical certification; a threaded repeat (the
  fixtures are single-threaded; RB-12's threaded-friction cluster applies to
  any threaded comparison); Windows/Linux lanes (CI on push).

## Decision required: production default

Both exits are **off by default**, which is the plan's authorized
implementation. Enabling one by default is the user's choice; the measured
consequences on the public fixtures are:

| default | compatible 105-pass continuation | collision-blocked motion | crushing motion |
| --- | --- | --- | --- |
| off (current) | 105 passes, success | runs until killed (98 passes / 300 s, degrading) | runs until killed |
| `max_passes: 200` | unchanged | stops at pass 200 (the baseline needed 300 s for 98 passes; the degraded passes cost ≈ 7 s each, so roughly 15 min) | stops at pass 200 (60 passes took 253 s: roughly 15 min) |
| `stagnation_window: 3` (+ any cap) | unchanged (never fires) | stops at pass 10 (8 s) | does not fire (drift progress); the cap ends it |
| `max_passes: 50` | **would fail this continuation at pass 50** | stops at 50 | stops at 50 |

A stagnation window of 3 with the default tolerances fired on nothing that
succeeds in this session's evidence, but the compatible fixture is one
scene; the rule's false-positive cost is one step (rolled back, reported),
not an accepted state. Recommendation to put to the user: keep both off
until a real scene shows the loop (the public smokes never enter it), or
enable `stagnation_window: 3` alone if the collision-blocked shape is what
the user's stalls look like.

## Publication and reproducibility

- Rebuilt targets: `PolyFEM_bin` and `unit_tests` in the isolated worktree
  build (`outputs/rb-07/20260915T195548Z/build`: candidate `PolyFEM_bin`
  `76df650e…` before the final clang-format pass — the A/B smokes, the full
  suite and `validate-final-{a,b,c}` ran on it — and `4a420e88…` after it,
  on which `validate-final-formatted/` (23/23 checks, endpoint `860cde46…`
  again) and `ab-smokes-final/` (6/6 byte-identical) ran), then the main
  checkout fast-forwarded to `6a553447b` and `polyfem/build` rebuilt from it
  (`PolyFEM_bin 62939e21…`, `unit_tests 7a8d0eab…`, `--build_info` polyfem
  `6a553447b` / IPC `c24d803e` / PolySolve `bce32a39`): `tests-final/`
  (`[al_budget]` 5 cases / 194 assertions, the affected selection 55 cases
  / 3672 assertions), `run-smoke-main-tree.txt` (five smokes exit 0, 0
  error lines) and the Houdini PolyFEM 2.0 end-to-end HDA test through the
  real binary (`hda-test-polyfem.log`, 8 PASS, exit 0).
- Committed files: `src/polyfem/solver/ALSolver.{hpp,cpp}` (budget options,
  `ALBudgetExhausted`, snap gate, pass records, budget checks),
  `FullNLProblem.{hpp,cpp}` / `NLProblem.{hpp,cpp}` (`probe_step_bound`),
  the six forward paths (`set_budget`), `NonlinearElasticVarForm.cpp`
  (details into the failure record and the manifest), `json-specs/input-spec.json`
  (`/solver/augmented_lagrangian/budget/*`), `tests/test_al_solver.cpp`,
  `tests/test_step_rollback.cpp`, `tools/rb04/check_solver_attempts.py`,
  `tools/rb07/`, `scenes/semi-implicit/README.md`, this record, the plan row.
  Implementation commit `6a553447b` on `sdast9/polyfem:main` (this record's
  publication details are in the follow-up documentation commit).
- Companion pins unchanged (IPC `c24d803e`, PolySolve `bce32a39`); no HDA
  change (no default chosen, so no control).
- Evidence stays local under `outputs/rb-07/20260915T195548Z/` (public
  inputs only; the frames and logs are not committed).

## Next session handoff

- Completed: reproduction (three public fixtures, two of them unbounded at
  the baseline), the measured case against BC-residual-only rules, the
  proposal, the opt-in implementation with per-pass records on every path,
  the failure record/manifest fields, unit + scene regressions, off-path
  identity, publication.
- Status: the mechanism is validated within the stated scope; the item's
  production default is **decision pending** (the plan's decision register:
  "Enabled production resource or AL budgets — user selects defaults").
- Decision to obtain: whether `solver/augmented_lagrangian/budget` gets an
  enabled default (`max_passes` and/or `stagnation_window`, with which
  numbers) and, if so, a Houdini *Solver* control for it.
- Next eligible: RB-23 (Q3+ hexahedral basis), RB-24 (per-thread contact
  memory). RB-08 stays closed; a budget exit is a named failure, not a retry
  trigger.
