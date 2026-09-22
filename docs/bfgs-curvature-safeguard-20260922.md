# BFGS curvature safeguard — audit stage 1

Date: 2026-09-22. **Status: implemented and validated within the stated scope.**

Stage 1 of the [BFGS convergence audit](bfgs-convergence-audit-20260922.md):
the BFGS and L-BFGS strategies no longer put a secant pair into their
approximation without validating it, and they report what they refused. The
fixed corpus the stage asked for decided the policy, the relative threshold and
one question the plan's text left open.

## Scope

No line-search acceptance rule, stopping tolerance, solver restart budget,
contact law, coefficient law, friction policy, CCD rule or trial-displacement
cap changed. The retired constraint floor stays retired. No automatic timestep
or load retry was introduced and Teseo was not run. The L-BFGS-B strategy keeps
its own filter, unchanged, for the reason given under *Limits*.

Stage 2 is **not** done here. The guard refuses the pairs a mid-solve retune
produces, but it does not know that a retune happened and does not reset the
history on one; that remains stage 2's objective-version signal.

Starting checkouts were clean: PolyFEM `c1d96c25f`, PolySolve `427e1458`,
IPC `482b9eab`. `polyfem/build/CMakeCache.txt` selects the local
`polysolve-merged` checkout; LBFGSpp is v0.2.0, Eigen 5.0.1.

## What the strategies do now

Both strategies form `s = x - x_prev` and `y = g - g_prev` as before and hand
them to a shared `CurvatureGuard`
(`src/polysolve/nonlinear/descent_strategies/CurvatureGuard.{hpp,cpp}`) before
anything is stored.

A pair is refused, with the reason recorded, when

| reason | test |
| --- | --- |
| `non_finite_pair` | `s` or `y` has a non-finite entry, or `‖s‖²`, `‖y‖²` or `s·y` overflows |
| `zero_displacement` | `‖s‖² = 0`: the pair carries no curvature |
| `zero_gradient_change` | `‖y‖² = 0`: the initial scale is undefined |
| `insufficient_curvature` | not `s·y > curvature_tolerance · ‖s‖ · ‖y‖`, which includes zero and negative curvature |
| `invalid_scale` | `‖y‖² / s·y` is not finite — the limited-memory initial scale and the magnitude of the dense rank-one term |
| `non_finite_update` | dense only: the updated matrix is not finite. It is formed aside and only assigned once checked, so a bad update never reaches the stored one |

The dense form additionally requires `B·s` to be finite and `s·B·s` finite and
positive before it divides by either denominator.

The threshold is **relative and scale aware**: `s·y > ε‖s‖‖y‖` is invariant
under a rescaling of the objective or of the variables, unlike an absolute
curvature bound.

After the direction is computed it is checked as well. A non-finite direction
or an ascent direction away from a stationary point discards the approximation
(`non_finite_direction`, `non_descent_direction`) and falls back to steepest
descent. A caught dense factorization/solve exception instead records
`factorization_failed`, resets the matrix and returns failure to the outer
strategy chain; a lone BFGS strategy then fails. The
[stage 1–2 review](bfgs-stage12-review-20260922.md) identified this discrepancy
with the originally stated fallback and added a contract/test follow-up.
A dense factorization of an indefinite matrix can succeed and return an ascent
direction, so the direction check is still needed beyond the pair tests.

Counts of accepted, damped and refused pairs, the reasons, and the discarded
approximations with their reasons are logged at debug level and reported in
`solver_info["curvature_guard"][<strategy>]`.

### Parameters

| key (under `/L-BFGS`, `/BFGS` or a `/solver` list entry) | default | meaning |
| --- | --- | --- |
| `curvature_policy` | `Skip` | `Skip` refuses a pair without positive curvature; `Damp` stores the nearest pair that has it, measured in the current approximation (Powell damping: `y ← φy + (1−φ)Bs` for the dense form, `s ← φs + (1−φ)Hy` for the limited-memory one) |
| `curvature_tolerance` | `1e-8` | the relative threshold `ε` above |
| `curvature_damping` | `0.2` | the fraction of the approximation's own curvature a damped pair gets; only used by `Damp` |
| `curvature_restart` | `1` | consecutive refused pairs after which the retained approximation is discarded; `0` keeps it for the whole solve |

## The fixed corpus

`polyfem/tools/bfgs_curvature/` builds the real strategies, line searches and
factory, and runs 520 rows: the two bounded polynomials of the audit, Rosenbrock
from `(-1.2, 1)`, and positive quadratics at curvature `1e-8 … 1e8` in three
variables, each as a lone strategy and as the public string-form configuration
that appends gradient descent as a fallback. It covers a policy × line-search
matrix (192 rows), a restart sweep (256), a threshold sweep (64) and eight
per-iteration traces. Evidence, with the build command and every source,
library and executable hash, is in the parent workspace's
`outputs/bfgs-stage1/20260922T113940Z/`: `corpus/` is the run the decisions
were taken from and `corpus-published/` its rerun on the published revision
with a clean tree, whose rows are identical to the byte.

### What it decided

**The relative threshold's value does not matter on this corpus.** Between
`0` (accept any positive curvature) and `1e-2`, every row has the same
outcome, the same iteration count and the same accepted/refused counts. The
defect was the absence of validation, not the choice of threshold. `1e-8` is
kept: it is the standard conservative value, it is what rejects a numerically
orthogonal pair, and it leaves the initial scale bounded by `‖y‖/(ε‖s‖)`.

**Skip converges on more of the corpus than Damp, in fewer iterations.**
At the defaults, over the 192-row policy matrix:

| policy | converged | total iterations |
| --- | --- | --- |
| `Skip` | 96 / 96 | 802 |
| `Damp` | 95 / 96 | 981 |

They differ on three objectives. Damping is better on Rosenbrock with L-BFGS
(37 against 42 iterations) and on the quartic with dense BFGS (8 against 10);
it is much worse on the quartic with L-BFGS (54 against 10) and is the only
configuration that still fails, under `Backtracking`, where 23 damped pairs
shrink the step until the line search cannot find a decrease.

`Skip` is the default. It converged everywhere, it costs fewer iterations
overall, it stores only measured curvature — which is what "no invalid pair in
history" means literally — and it matches the rule the L-BFGS-B strategy in
this repository already applies. `Damp` stays available and is covered by a
test; nothing here recommends it as a production setting.

## Where the plan's text was not sufficient

The plan says to skip invalid pairs and retain the last valid approximation,
clearing it only if it produces a non-finite or non-descent direction. Measured,
that is not enough, and it makes the **public** configuration worse:

All of these are the public string form, which appends gradient descent as a
fallback. "Before" is the audit's own measurement of the same chain; "retain
forever" is this stage with `curvature_restart: 0`, under the policy named.

| objective, strategy | before stage 1 | retaining forever |
| --- | --- | --- |
| Rosenbrock, L-BFGS, under `Skip` | converged in 32 / 24 / 32 iterations (Armijo / RobustArmijo / Backtracking), by escalating to the fallback | **iteration limit at 500**, `‖g‖ = 2.13` |
| `x⁴/4−x²/2+0.1x`, L-BFGS, under `Damp` | converged in 10 iterations | **iteration limit at 500**, `‖g‖ = 0.248` |

Both stalls have the same shape, and the traces
(`corpus/results.jsonl`, the rows carrying `trace`) show it directly. Once the
pairs stop being usable the retained approximation describes a stretch of the
solve that is over, and it keeps producing a *descent* direction that the line
search accepts — so neither the direction test nor the solver's own
non-descent escalation ever fires, and the fallback chain is never reached.
Rosenbrock creeps at `‖Δx‖ ≈ 1.8e-3` for 496 refused pairs; the quartic under
damping collapses to `‖Δx‖ = 4e-18` and then refuses 475 zero displacements.

Removing an invalid direction removed the error signal the fallback relied on
without putting anything in its place. `curvature_restart` is that signal: after
a bounded run of refused pairs the approximation is discarded and the strategy
restarts from steepest descent, which is the same "existing safe fallback" the
plan names, on a condition the plan did not state.

| `curvature_restart` | corpus rows that fail (Armijo sweep, 64 rows per value) |
| --- | --- |
| `0` | 4 — the two stalls above, alone and chained |
| `1` | 0 |
| `3` | 0 |
| `6` | 0 |

`1` is the default: it is the only value that leaves no pair of a refused step
in the approximation at all, it costs the fewest iterations of the three on the
rows where they differ (96 against 100 and 108), and it anticipates stage 2 —
the pair a mid-solve retune produces is exactly the one whose neighbours must
not be kept. On every other corpus row the value changes nothing.

## Acceptance

| check of the plan's stage 1 | result |
| --- | --- |
| Finite descent directions away from stationarity | every corpus row; the direction test discards the approximation instead of returning a non-finite or ascent direction, with a regression for each |
| No invalid pair in history | the six refusal reasons are validated before the approximation is touched; the dense update is formed aside and checked before assignment; `[bfgs][curvature]` asserts the exact counters for nine pairs, including both audited steps |
| Pure-strategy regression convergence at unchanged tolerances | both audited polynomials now converge with the strategy alone, for both strategies and all three energy line searches: 7 and 10 iterations, `‖g‖ ≤ 3e-13`. Before, each failed at iteration 1 — one with a NaN direction, one with an ascent direction |
| Unaffected valid-pair controls | quadratics at curvature `1e-8 … 1e8`, in one and three variables, converge with no pair refused and no approximation discarded; Rosenbrock with dense BFGS is unchanged at 35 iterations |
| Accepted / skipped pairs and reset reasons logged | debug lines per event and cumulative counts in `solver_info["curvature_guard"]` |
| Relative threshold compared with damping on the fixed corpus | the table above; `Skip` chosen, `Damp` retained as an option |

### Integration validation

| check | result |
| --- | --- |
| PolyFEM CMake build, macOS arm64 RelWithDebInfo | `PolyFEM_bin` and `unit_tests` rebuilt; no new compiler warnings |
| PolySolve nonlinear suite | **20 cases / 1,632 assertions**, exit 0 (13 / 1,254 before, plus the seven new cases) |
| PolyFEM `[al_solver],[direction_filter]` | 26 cases / 4,060 assertions, exit 0 |
| Five public Newton smokes, before and after | 5/5 exit 0, four time steps each, zero error lines, zero restarts, and **all five VTU frames byte-identical** to the pre-stage-1 executable |
| The two public scenes with L-BFGS and dense BFGS | unchanged: exit 1 at step 1 after 20 restarts, as the audit recorded. Dense BFGS takes 74.7 s and 69.3 s single-threaded |
| Guard activity on those four scene runs | **none**: no pair was refused and no approximation discarded |
| Rebuild after `CurvatureGuard.cpp` was clang-formatted | the relinked executable has a different hash and reproduces the recorded `quasistatic-semi` frames byte for byte; `[al_solver],[direction_filter]` and the PolySolve suite were rerun on it |

The last row matters. The public contact scenes do not exercise the defect this
stage repairs; their failure is the restart-limited plateau the audit already
separated from it. Stage 1 is not a claim about those scenes, and stage 4 still
owns their measurement.

## Limits

- Nothing here establishes convergence on contact scenes, general failure
  rates, or physical accuracy. The corpus is synthetic and one-dimensional
  apart from Rosenbrock and the three-variable quadratics.
- The remaining stalls are bounded by the corpus, not eliminated in principle:
  with the default the corpus has none, but `Damp` still fails one row, and no
  bound is proved for an objective outside the corpus.
- A pair that mixes two objective versions is refused because its curvature is
  usually invalid, not because the change was detected. When such a pair
  happens to have positive curvature it is still stored. That is stage 2.
- The absence of a Wolfe-capable line search is unchanged and is what makes the
  refusals common in the first place: a Wolfe curvature condition would make
  `s·y > 0` a property of the accepted step rather than something to test after
  the fact. That is stage 3.
- L-BFGS-B keeps its own `s·y > 1e-9‖y‖²` filter. It is not scale invariant in
  the variables, but it does bound the initial scale and it rejects every pair
  in the corpus that the new rule rejects. It is reached only through the
  box-constrained optimization path, which this stage has no measurements for,
  so it was left alone; stage 5 owns that strategy's support question.
- `curvature_restart` bounds how long an approximation outlives refused pairs.
  It is not a change to the solver's restart budget, the AL budget or the
  barrier-retune controller, and it says nothing about them.

## Published revision

PolySolve
[`30f3a3a8b0aa`](https://github.com/sdast9/polysolve/commit/30f3a3a8b0aa291da1c7f738e86bc134a8a269a1),
branch `iteration-callback`, pinned by `cmake/recipes/polysolve.cmake`. The
configured build uses the local `polysolve-merged` checkout, which is at that
revision with a clean tree.

## Evidence

`outputs/bfgs-stage1/20260922T113940Z/` in the parent workspace holds the
corpus build record and results, the test build/run records and logs, the
before/after scene runs with per-frame hashes, and the preserved pre-stage-1
executable. The reproducible corpus source and runner are in
[tools/bfgs_curvature](../tools/bfgs_curvature/README.md); the permanent
regressions are PolySolve `tests/test_bfgs.cpp`, tagged `[bfgs][curvature]`.
