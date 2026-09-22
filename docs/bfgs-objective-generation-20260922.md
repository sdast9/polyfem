# Objective-generation history reset — audit stage 2

Date: 2026-09-22. **Status: implemented and validated within the stated scope.**

Stage 2 of the [BFGS convergence audit](bfgs-convergence-audit-20260922.md):
a form that retunes itself during a solve now says so, the problem reports it,
and the nonlinear solver discards the quasi-Newton history before a secant pair
can span the change. This closes the gap RB-02 had recorded as a limit and the
audit reproduced as finding 2.

## Scope

No retuning law changed: not the semi-implicit coefficient estimate, the trim
controller, the classic adaptive barrier stiffness, the friction lag, the AL
continuation or the quadrature refinement. No stopping tolerance, restart
budget, line-search rule, CCD rule or trial cap changed. The retired constraint
floor stays retired, no automatic timestep retry was introduced and Teseo was
not run. Newton is unaffected by construction: it keeps no history.

Starting checkouts were clean: PolyFEM `eddf837ab`, PolySolve `30f3a3a8`,
IPC `482b9eab`; the configured build uses the local companion checkouts.

## The signal

`polysolve::nonlinear::Problem::objective_generation()` returns a number that
must change when the objective stops being the same function, and only then.
The default never changes, so a problem with a fixed objective costs one
comparison per iteration.

In PolyFEM every `Form` counts changes to its own function
(`Form::objective_generation`, incremented by `note_objective_change`), and the
problems sum their forms: `FullNLProblem` over the forms, `NLProblem` adding
the penalty (AL) forms, `NLHomoProblem` adding the homogenization forms. A sum
is monotone and needs no coordination between forms.

| counted as a change | not counted |
| --- | --- |
| a semi-implicit coefficient refresh (contact born mid-solve, periodic refresh, stall retune) | moving the iterate |
| a trim bump that moves the trim, up or down | a trim bump that leaves it where it was |
| the gradient-balance trim calibration when it raises the trim | rebuilding the collision set or any cache (`solution_changed`) |
| the classic adaptive barrier stiffness, in the barrier and smooth contact forms, when the value moves | evaluating the form (value, gradient, Hessian) |
| a committed quadrature refinement (`ElasticForm`), from `post_step` or from the immediate commit in `max_step_size` | the start-point `post_step` PolySolve emits before its first iteration |
| a friction lag rebuild | the caller's own weight ramp (`set_weight`), which happens between minimizations |
| a restored form state whose weight, scale or enabled flag moved (RB-06 rollback) | |

The trim also scales the friction potential in the follow-stiffness lag, so
that change rides along with the barrier form's own count.

## What the solver does with it

`Solver::minimize` reads the generation once before the loop and again at the
top of every iteration, before the energy and the gradient. When it differs:

- every strategy is told (`DescentStrategy::objective_changed`). `LBFGS` and
  `BFGS` discard the approximation **and** the stored iterate and gradient, so
  no pair is formed across the change at all, and the guard records the reason
  `objective_changed` alongside the stage 1 counters.
- the previous energy is dropped (`old_energy = NaN`), because the change in
  the objective value between two different objectives is not defined. That
  iteration's reported `Δf` is `nan`, which can only delay a convergence
  decision, never cause one: `fDelta < tol` is false for `nan` and the
  consecutive count resets.
- the count is reported as `solver_info["objective_changes"]`.

Checking at the top of the iteration catches every source, including a
quadrature refinement committed inside the previous iteration's line search,
because the comparison is between the generation the stored gradient belongs to
and the one the next gradient will be taken from.

What is deliberately **not** reset: the relative-gradient anchor
(`initial_grad_norm`), the iteration count, the restart budget, the AL weights
and the line-search state. Re-anchoring a convergence criterion mid-solve is a
stopping-criterion decision, which this stage does not make.

## Measurements

### The mechanism is live on the public scenes

The five public smokes retune themselves several times per solve, which the
solver now sees:

| scene (Newton) | objective changes seen by the solver | form-level change events |
| --- | --- | --- |
| `quasistatic-adaptive` | 0 | 0 |
| `quasistatic-semi` | 3 | 15 |
| `quasistatic-semi-alhess` | 3 | 15 |
| `quasistatic-semi-friction` | 4 | 35 |
| `transient-semi` | 3 | 15 |

All five still exit 0 with all four time steps, zero error lines and zero
restarts, and **every VTU frame is byte-identical** to the stage 1 executable's:
Newton holds no history, so being told changes nothing it does.

### The public contact scenes with L-BFGS complete one more time step

| scene, method | stage 1 | stage 2 |
| --- | --- | --- |
| `quasistatic-semi`, L-BFGS | exit 1 **at step 1**, 1 frame, 20 restarts, 12.5 s | exit 1 **at step 2**, 2 frames, 39 restarts, 22.0 s, 8 changes / 8 discards |
| `transient-semi`, L-BFGS | exit 1 **at step 1**, 1 frame, 20 restarts, 14.1 s | exit 1 **at step 2**, 2 frames, 39 restarts, 21.2 s, 4 changes / 4 discards |
| `quasistatic-semi`, dense BFGS | exit 1 at step 1, 1 frame, 74.7 s | unchanged outcome, 84.2 s, 8 changes / 8 discards |
| `transient-semi`, dense BFGS | exit 1 at step 1, 1 frame, 69.3 s | unchanged outcome, 84.6 s, 9 changes / 9 discards |

Both L-BFGS scenes now complete their first time step and fail at the second;
before this stage they failed at the first. That is the first movement on the
public scenes in this audit, and it is not convergence: both still end in a
named failure, and the dense form is unchanged in outcome and 10-20 % slower
for the rebuilding it now does.

The failing pass itself is not about curvature or objective mixing. The record
in the final `quasistatic-semi` failure reports 100 accepted pairs, no refused
pair, no reset and no objective change in that pass at the soft iteration
limit: the restart-limited plateau the audit separated out, which stage 4 owns.

### Suites

| check | result |
| --- | --- |
| PolySolve nonlinear suite | 24 cases / 1,706 assertions, exit 0 (20 / 1,632 before; four new `[bfgs][objective]` cases) |
| PolyFEM form and solver suites (`[al_solver]`, `[direction_filter]`, `[objective_generation]`, `[rollback]`, `[kappa_continuity]`, `[friction_lag]`, `[al_budget]`, `[coefficient_events]`, `[contact_form]`, `[elastic_form]`, `[form]`, `[form_derivatives]`, `[iteration_observer]`, `[contact_cache]`, `[fully_prescribed]`, `[contact_stiffness_mapping]`) | 88 cases / 10,310 assertions, exit 0 |
| RB-02 coefficient probe | 270 / 270 checks, exit 0 — the coefficient law is untouched |
| Five public Newton smokes, against the stage 1 frame hashes | 5/5 exit 0, all frames byte-identical |

## Acceptance

| check of the plan's stage 2 | result |
| --- | --- |
| A problem-level objective generation or equivalent explicit signal | `Problem::objective_generation`, summed from `Form::objective_generation` |
| Real stiffness / trim / friction-objective changes propagated through FullNLProblem / NLProblem | the table above; the penalty forms of the reduced problem and the homogenization forms are included |
| Quasi-Newton history invalidated before the next pair is formed | the strategies discard the approximation and the stored iterate/gradient; `[bfgs][objective]` pins that the next direction is exactly the steepest descent one, at the strategy and at the solver |
| Objective-difference bookkeeping reset where necessary | `old_energy` is dropped, so `fDelta` is undefined across the change instead of comparing two objectives |
| No reset for coordinates or an active collision list when the objective is unchanged | `[objective_generation]` pins the collision-set rebuild, the evaluation, the no-op trim bump, the start-point `post_step` and a full solve's worth of cache rebuilds |
| Tests for unchanged and retuned controls, same-position changes, AL / reduced boundaries and rollback | `[objective_generation]` (a refresh at fixed coordinates, a retuned and an unretuned solve, a reduced problem with a penalty form, a restored state) and `[bfgs][objective]` |
| All stored pairs belong to one objective generation | by construction: a pair is formed only from the stored iterate and gradient, and both are discarded when the generation changes before the next direction is computed |
| The current retuning law and Newton controls remain intact | RB-02 270/270 and five byte-identical Newton smokes |

## Limits

- This is a **report, not a detection**. A form that changes the function it
  evaluates without calling `note_objective_change` is still invisible, exactly
  as before. The known sites are instrumented and pinned by tests; a new one
  has to make the call, which `Form::objective_generation` documents.
- The generation is a version, not physical state: it is never saved, restored
  or reset, so it is monotone within a process and comparisons across a
  rollback stay meaningful.
- Nothing here establishes convergence on contact scenes, general failure
  rates, or physical accuracy. The extra time step on two public scenes is a
  measurement on those two scenes, and both still fail.
- Dense BFGS pays for the discards on those scenes (10-20 % more wall time)
  with no change in outcome.
- The stage 1 curvature safeguard remains the only thing standing between a
  *silent* objective change and the approximation, and it only catches the
  pairs whose curvature happens to be invalid; `[bfgs][objective]` pins a
  spanning pair with perfectly good curvature that only this signal prevents.
- A change that happens between minimizations needs no signal, because every
  `minimize` re-reads the generation; those sites are instrumented for the
  record, not for correctness.

## Published revisions

PolySolve [`440cd55cdaa0`](https://github.com/sdast9/polysolve/commit/440cd55cdaa09bef079578b128bfe7f6f1e869af), branch `iteration-callback`, pinned by
`cmake/recipes/polysolve.cmake`; PolyFEM this commit. Evidence, with the scene
runs, per-frame hashes, suite logs and the RB-02 probe, is in the parent
workspace's `outputs/bfgs-stage2/20260922T135940Z/`.
