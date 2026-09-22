# Feasibility-respecting Wolfe line search — audit stage 3

Date: 2026-09-22. **Status: implemented and validated within the stated scope,
as an opt-in (`solver/nonlinear/line_search/method: "Wolfe"`). No default
changed, and none is proposed: on the public contact scenes the search works as
specified and changes little.**

Stage 3 of the [BFGS convergence audit](bfgs-convergence-audit-20260922.md)
asked for an optional strong Wolfe search that respects feasibility — CCD,
validity, the trial-displacement cap — with explicit Armijo and slope tests, a
finite evaluation budget, a defined outcome where no Wolfe point is feasible,
and a restart when the objective changes mid-search; compared with the
safeguarded RobustArmijo before any default is discussed. [Stage 4](bfgs-contact-diagnostics-20260922.md)
had pointed it at the *growth* half. Both are now measured:

- The search is **inert where it is not selected**: the five public Newton
  smokes reproduce stage 4's frames byte for byte, with the diagnostics off and
  on.
- Under **Newton** it converges on all five smokes at Newton's solution, in
  26 instead of 31 iterations on `quasistatic-semi` and the same number
  elsewhere.
- Under **L-BFGS**, with stage 4's converging configuration, it never falls
  back, never exhausts its budget and reaches the same solutions — but moves
  the iteration count by between **−21 % and +11 %**, against the 220–940×
  that separate L-BFGS from Newton on these scenes. The plateau is not the line
  search's.
- At the textbook `c2 = 0.9` the growth half is **almost never reached** (8
  growth sweeps in 8,732 iterations of `quasistatic-semi`), as stage 4's own
  slope data predicts once it is read against `c2`. At `c2 = 0.1` it grows in
  84 % of iterations and saves at most a fifth of them, for about three energy
  and three gradient evaluations per iteration.
- At **production settings** nothing changes: the stall controller's soft
  budget still ends every L-BFGS run, and dense BFGS fails identically under
  all three searches.

Along the way the validation found and repaired two **pre-existing** RB-04
attempt-stream defects on the line-search-failure path (below).

## Scope

PolySolve gained the `Wolfe` search, a `line_search_extend` problem callback, a
`Solver::direction_admits_growth` hook (false for the box-constrained solver),
cleanup of the problem's state when a search returns NaN, and tests. PolyFEM
reports extensions (`IterationObservation::Kind::Extension`), carries them in
the attempt stream (schema `polyfem.solver-attempt` version 3), and repairs two
observer defects. No default, retuning law, stopping tolerance, restart budget,
contact law, CCD rule, trial cap or friction policy changed. The retired
constraint floor stays retired; no automatic retry was added; Teseo was not run.

Starting checkouts were clean: PolyFEM `16529cdd5`, PolySolve `fcab19f0`, IPC
`482b9eab`; the build uses the local companion checkouts. Raw evidence is in the
parent workspace's `outputs/bfgs-stage3/20260922T174512Z/` (compact:
`stage3-summary.json`; the pre-change binaries are kept in `baseline/` with
their SHA-256). The runner is [tools/bfgs_stage3](../tools/bfgs_stage3/README.md).

## What stage 4's data said before anything was built

Stage 4 counted 78 % of accepted L-BFGS iterations at `alpha = 1`, uncapped,
with the endpoint still descending — "the only place growth is admissible". A
Wolfe search, however, grows only where the curvature condition fails, i.e.
where the endpoint slope is still steeper than `c2` times the initial one.
Measured on stage 4's own converged runs, among those admissible iterations:

| run | admissible iterations | slope ratio, median | ratio > 0.9 | ratio > 0.5 | ratio > 0.1 |
| --- | --- | --- | --- | --- | --- |
| `quasistatic-semi`, no soft budget | 6,942 | 0.41 | **0.06 %** | 28 % | 94 % |
| `quasistatic-adaptive` | 24,479 | 0.40 | **0.3 %** | 28 % | 93 % |

So at `c2 = 0.9` the growth half has almost nothing to do here, and it only
becomes active at much smaller `c2`. That is why the matrix sweeps `c2` over
0.9, 0.5 and 0.1 rather than testing the default alone.

## The search

Bracket-and-zoom on `phi(alpha) = f(x + alpha p)` (Nocedal and Wright,
Algorithms 3.5/3.6) with the Armijo decrease (`Armijo/c`) and the strong
curvature condition `|phi'(alpha)| <= c2 |phi'(0)|`; cubic, quadratic or
bisection trial points kept at least 10 % inside the bracket.

**Feasibility.** The search starts from the step PolySolve's existing
finite-energy and CCD stages leave. It grows beyond it only when neither cap
shortened that step, the solver's direction admits growth, and the growth limit
allows. Every growth first calls `line_search_extend(x, x + target p)` — the
problem rebuilds its swept candidates for the whole longer interval — then
prices it with `max_step_size`, and only then evaluates anything there. The
separate trial-displacement cap is inside the contact form's `max_step_size`
and applies unchanged; a fraction below 1 makes that sweep's end a boundary.
No trial ever lies outside an interval the problem has priced (tested).

**Outcomes.** Each search reports how it ended
(`solver.line_search.wolfe.outcome` in the attempt stream, cumulative counts in
`solver_info["wolfe_outcomes"]`):

| outcome | meaning |
| --- | --- |
| `wolfe` | Armijo decrease and the strong curvature condition |
| `approximate_wolfe` | the energy could not be compared (below), the strong curvature condition held |
| `capped_feasibility`, `capped_start_capped`, `capped_growth_limit`, `capped_direction_admits_no_growth`, `capped_sweep_refused` | a decrease point whose slope is still steeper than `c2` allows, at a boundary growth cannot cross — the stated capped-step outcome |
| `armijo_*`, `flat_*` (`budget`, `interval_collapsed`) | the best decrease point, when the budget ran out or the bracket collapsed |
| `fallback_*` | no decrease point: RobustArmijo from the same start, on its own budget, decides exactly as it would alone |
| `objective_changed`, `objective_changed_not_descent` | the objective changed more often than `max_objective_restarts` allows (the search fails) |

**The safeguarded update rule for a capped step** is stage 1's: a step accepted
without the curvature condition carries no curvature guarantee, and the
strategies' curvature guard decides whether its pair enters the approximation.

**Objective changes.** A generation change observed after any trial's
`solution_changed` invalidates `phi(0)`, `phi'(0)` and every bracket value: the
search re-anchors at `x` under the new objective (re-evaluating energy, gradient
and the initial slope, failing if that is no longer a descent direction), at
most `max_objective_restarts` times. No PolyFEM form changes its objective
inside `solution_changed` today; the path is covered synthetically.

**Cleanup.** A NaN return leaves the problem's state at `x` and ends the
interval (the base search did neither on that path, which no earlier search
used); an exception ends the interval and propagates.

| parameter (`line_search/Wolfe/…`) | default | |
| --- | --- | --- |
| `c2` | 0.9 | must satisfy `Armijo/c < c2 < 1` |
| `growth_factor` | 2 | > 1 |
| `growth_limit` | 16 | largest `alpha` growth may reach |
| `max_evaluations` | 20 | further bounded by `max_step_size_iter` |
| `max_objective_restarts` | 2 | |
| `approximate_wolfe_epsilon` | 1e-6 | Hager and Zhang's default |

## The first design failed on the energy's own noise

The textbook zoom compares energies. Its first version converged under Newton
but spent 16–20 evaluations per search near convergence and accepted steps of
`alpha = 1e-7`; its second — trials judged on their slope when the energy was
within RobustArmijo's roundoff bound (RB-19) — made Newton **fail** on
`quasistatic-semi`. The trace showed why: at `f = 11972` the energy of the
contact problem is noisy at about `1e-11`, several times RB-19's bound
`eps (1 + |f|) = 2.7e-12`, because it is a sum of terms far larger than itself.
Every trial read as an increase, and the zoom chased the noise to zero.

The remedy is Hager and Zhang's approximate Wolfe condition (SIAM J. Optim.
16(1), 2005), whose slack exists for exactly this: a trial whose energy is
within `eps_k = approximate_wolfe_epsilon |phi(0)|` of `phi(0)` (or within the
roundoff bound) is judged on its slope alone — accepted on the strong curvature
condition, otherwise placed in the bracket by the sign of its slope, with
secant trial points on the slopes. The one-factor measurement on
`quasistatic-semi` under L-BFGS:

| `approximate_wolfe_epsilon` | exit | completed steps | accepted iterations | alpha restarts |
| --- | --- | --- | --- | --- |
| 1e-6 (default) | 0 | 4 of 4 | 8,732 | 0 |
| 1e-9 | 0 | 4 of 4 | 8,295 | 0 |
| 0 (roundoff bound only) | **1** | 2 of 4 | 6,752 | 59 |

Most accepted L-BFGS steps are judged this way at the default (80–89 %,
`approximate_wolfe`): late in a solve the per-iteration decrease is far below
`1e-6 |f|`. The slack admits an energy *increase* of at most `eps_k` at a point
whose slope satisfies the strong curvature condition; that is the trade Hager
and Zhang make, and this record measures it only on these scenes.

A final guard came from the same failing configuration: a budget exit accepted
a decrease point at `alpha = 9e-9` along a direction of norm `2e-10`, a step
that does not move the iterate in floating point, which the RB-04 checker
rejects. A decrease point that does not move the iterate is now not accepted;
RobustArmijo decides instead.

## Inert where not selected

| check | result |
| --- | --- |
| Five public Newton smokes, diagnostics off | frames byte-identical to stage 4's `*-newton-off` |
| The same, diagnostics on | frames identical to the off runs; attempt streams (version 3) pass the RB-04 checker |
| RobustArmijo L-BFGS in stage 4's converging configuration | 8,841 / 9,531 / 13,732 / 34,835 accepted iterations: stage 4's counts exactly |
| Final binary against the matrix binary | the 15 Newton rows and an L-BFGS Wolfe row reproduced byte for byte after the last source change |

## Measured: L-BFGS in stage 4's converging configuration

Soft iteration budget removed and `max_iterations: 20000` — a **diagnostic**
setting, not a production one; everything else at the scene settings. Four
single-threaded runs at a time, so the wall times are contended and comparable
only within this table. Every row converges on the configured
`‖∇f‖_rel < 1e-10` to the same solution (L2 within 2e-5 of RobustArmijo's).

| scene | line search | accepted iterations | wall s | energy / gradient evaluations in the search | growth sweeps | accepted `alpha > 1` |
| --- | --- | --- | --- | --- | --- | --- |
| `quasistatic-semi` | RobustArmijo | 8,841 | 27.1 | — | — | — |
| | Wolfe `c2 = 0.9` | 8,732 | 31.7 | 9,408 / 9,292 | 8 | 7 |
| | Wolfe `c2 = 0.5` | 8,644 | 26.3 | 12,525 / 12,456 | 3,104 | 2,724 |
| | Wolfe `c2 = 0.1` | **7,021** | 28.5 | 21,784 / 21,318 | 9,675 | 5,893 |
| `transient-semi` | RobustArmijo | 9,531 | 30.2 | — | — | — |
| | Wolfe `c2 = 0.9` | 8,433 | 31.9 | 9,034 / 8,933 | 4 | 3 |
| | Wolfe `c2 = 0.5` | **7,870** | 26.4 | 11,445 / 11,381 | 2,913 | 2,604 |
| | Wolfe `c2 = 0.1` | 8,337 | 31.7 | 25,940 / 25,480 | 11,543 | 7,087 |
| `quasistatic-semi-friction` | RobustArmijo | 13,732 | 46.5 | — | — | — |
| | Wolfe `c2 = 0.9` | 12,630 | 45.1 | 13,556 / 13,382 | 11 | 8 |
| | Wolfe `c2 = 0.5` | 12,917 | 40.4 | 18,779 / 18,615 | 4,584 | 4,034 |
| | Wolfe `c2 = 0.1` | **10,932** | 44.7 | 34,087 / 33,547 | 15,053 | 9,084 |
| `quasistatic-adaptive` | RobustArmijo | 34,835 | 240.3 | — | — | — |
| | Wolfe `c2 = 0.9` | 38,570 | 396.9 | 40,426 / 40,227 | 138 | 98 |
| | Wolfe `c2 = 0.5` | **32,121** | 263.3 | 44,203 / 43,999 | 9,973 | 8,600 |
| | Wolfe `c2 = 0.1` | 32,181 | 340.2 | 90,745 / 89,996 | 38,511 | 24,214 |

`quasistatic-semi-alhess` is identical to `quasistatic-semi` row for row, as in
stage 4. For reference, Newton needs 31 / 26 / 62 / 37 iterations.

Every search in these 16 Wolfe runs ended in `wolfe`, `approximate_wolfe` or a
`capped_*` outcome: **no fallback, no budget exhaustion, no collapsed bracket,
no objective restart.** RobustArmijo's column has no search evaluations because
it records none; a Wolfe search spends one gradient per accepted step that the
solver then evaluates again at the next iteration (a cache would remove it; not
done here).

## Measured: production settings, dense BFGS, Newton

| run | result |
| --- | --- |
| `quasistatic-semi`, L-BFGS, Wolfe 0.9 / 0.1 | exit 1 at step 2 (1 of 4 completed), 31 / 35 restarts, **all from the soft budget** — as RobustArmijo (39) |
| `transient-semi`, L-BFGS, Wolfe 0.9 / 0.1 | exit 1; 1 / 2 of 4 steps completed; 40 / 55 restarts, all from the soft budget |
| `quasistatic-semi`, dense BFGS, RobustArmijo / Wolfe 0.9 / Wolfe 0.1 | **identical failure**: 210 accepted iterations, 20 alpha restarts, no step completed. Under Wolfe 0.9 all 210 steps are Wolfe points (0.1: 209 and one capped start). Its plateau does not depend on the line search |
| five smokes, Newton, Wolfe 0.9 | all converge; 26 / 26 / 26 / 62 / 37 iterations against 31 / 31 / 26 / 62 / 37; L2 equal to Newton's to 15 digits on `quasistatic-semi`; outcomes `wolfe`, `approximate_wolfe` and 11 `capped_start_capped` on the adaptive scene |

## The attempt stream: extensions, and two older defects

A second `line_search_begin` inside one iteration would have filed the first
sweep as a `rejected` proposal. Growth therefore goes through the new
`line_search_extend` callback (by default the same as `line_search_begin`),
which PolyFEM observes as an `Extension`: once a `StepBound` prices it, the
row's trial becomes the longer sweep (`trial.extensions`,
`trial.extension_builds`, `trial.extensions_refused`,
`trial.unextended_norm`, schema version 3, documented in
[the RB-04 contract](rb-04-contract.md)). The checker adds extension rebuilds
to its broad-phase build identity.

Running the checker on every diagnostic row showed that the **stage 4 binary's
streams already failed it on every run with a `rejected` row** — three of
stage 4's L-BFGS rows, never checked there. Two defects, both on the
line-search-failure path and independent of this search:

1. A rejected proposal's validity checks were counted again in the next row,
   together with the next direction's finite-energy checks. The validity counts
   of a line search are now taken at its `LineSearchEnd`, and only the
   remainder carries over.
2. When a line search failed on its last strategy and the stall controller
   restarted the minimize, the pending proposal survived, and the new
   minimize's start point was filed as an accepted update of it (fraction 0 or
   negative, duplicate iteration numbers). PolySolve reports a start point with
   `status: NotStarted`; a proposal pending then is now filed as rejected.

Stage 4's three failing configurations are rows of this matrix
(`observer-*`); all pass now, as do the RB-04 endpoint runs
(`tools/rb04/run_endpoints.py --max-threads 1` plus the checker).

## Tests

PolySolve `tests/test_wolfe.cpp`, 15 cases / 103 assertions, `[wolfe]`: growth
beyond the direction with every growth priced first and never outside the
built sweep; a feasible bound below the curvature threshold (the capped
outcome); a zero CCD bound on the growth sweep, and on the unit step (fails as
before); a capped start does not grow; a box-constrained direction does not
grow; the growth limit; non-finite and invalid trials never accepted; zoom back
from an Armijo failure and from a positive slope; contacts that appear along
the line evaluated with their own contact set; unresolved energies judged on
slopes, including noise above the roundoff bound and the slack-0 fallback; an
objective change once (finishes under the new objective) and every time
(bounded, fails, restores the state); an exception mid-search (the interval
ends); inconsistent parameters refused; L-BFGS on Rosenbrock converging with no
refused pair and no history reset; L-BFGS-B staying inside its box. `Wolfe`
also joined `LineSearch::available_methods()`, so it now runs through the
generic solver × problem matrices, including `nonlinear-easier`, where an
exception is a failure.

| suite | result |
| --- | --- |
| PolySolve `[bfgs],[solver]` | 40 cases / 2,083 assertions (stage 4: 25 / 1,723) |
| PolyFEM solver, form and observer suites (stage 4's filter) | 89 cases / 10,328 assertions |
| RB-04 endpoints + attempt checker, single-threaded | pass |
| RB-02 coefficient probe | 270 / 270 |
| Houdini `test_polyfem_hda.py` | pass (strict JSON, solver-panel round trip, end to end) |

## Acceptance against the plan's item 3

| requirement | result |
| --- | --- |
| Bracket-and-zoom with explicit Armijo and slope tests and a finite evaluation budget | yes; `max_evaluations`, bounded by `max_step_size_iter` |
| Reuse `solution_changed`, validity, CCD and trial-cap handling | every trial through `solution_changed` and `is_step_valid`; every growth through `line_search_extend` and `max_step_size`, where CCD and the trial cap live |
| Growth beyond a swept interval rebuilds / revalidates it | yes, before any evaluation there (tested; `extension_builds` in the stream) |
| A capped-step outcome; a feasible decreasing step kept only under a stated safeguarded rule | `capped_*`, with stage 1's curvature guard as the rule |
| Entry/exit cleanup on exceptions; finite-energy prechecks | tested; growth trials pass validity and finiteness before use |
| Tests: alpha > 1, cap below the curvature threshold, non-finite trials, zero CCD step, new contacts, energy roundoff, objective-version changes | all present |
| Generation change: abort/restart under one objective, bounded, with cleanup | yes (tested synthetically; no PolyFEM form changes its objective mid-search today) |
| Compare with safeguarded RobustArmijo before proposing a default change | done; **no default change is proposed** |
| Positive accepted curvature alone does not establish Wolfe or rule out a benefit from growth | growth measured directly at three `c2`; its benefit is bounded above |

## Limits

- Five public scenes, one machine, single-threaded solves four at a time, one
  repetition each (the repeated rows reproduce bit for bit). This is a
  measurement on these scenes, not a benchmark or a claim about private scenes.
- The converging L-BFGS rows use stage 4's diagnostic setting (no soft
  budget); at production settings every L-BFGS row still fails.
- `approximate_wolfe_epsilon` is Hager and Zhang's default, measured only
  against 0 and 1e-9 on one scene. The slack admits small energy increases at
  points satisfying the curvature condition; that is a property of the method,
  not something this record establishes to be harmless in general.
- Wall times include the stage 4 diagnostics (one extra gradient per accepted
  iteration) and contention; iteration and evaluation counts are the primary
  measure.
- The objective-restart path is covered only synthetically.
- Dense BFGS's plateau remains undiagnosed; this record shows only that it is
  not the line search's.
- The search is not exposed in the Houdini asset; it is reachable through JSON.

## What this selects

Nothing to switch on. The measurement bounds what any line search can do for
L-BFGS here: even the growth-heavy configuration saves at most a fifth of the
iterations and pays for it in evaluations, while the gap to Newton is two to
three orders of magnitude. The levers that remain are the ones stage 4 named —
the stall controller's method-blind soft budget (the pending production
decision) — and the quasi-Newton model itself (conditioning, initial scaling,
a preconditioned L-BFGS), not the step length. Stage 5 remains as planned.

## Published revisions

PolySolve: the `Wolfe` search and `line_search_extend`,
[`fc62a679`](https://github.com/sdast9/polysolve/commit/fc62a6791f2136806fb36bd620459bca91bf5c89),
branch `iteration-callback`. PolyFEM: the extension observation, the attempt-stream
repairs, this record and the pin. Evidence in the parent workspace's
`outputs/bfgs-stage3/20260922T174512Z/`.
