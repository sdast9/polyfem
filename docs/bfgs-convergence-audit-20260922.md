# BFGS convergence audit and repair plan

Date: 2026-09-22. **Status: dense-BFGS update-order defect repaired; stages 1,
2 and 4 of the plan below (curvature safeguards, objective-generation history
reset, contact diagnostics) implemented and validated — see the
[stage 1](bfgs-curvature-safeguard-20260922.md),
[stage 2](bfgs-objective-generation-20260922.md) and
[stage 4](bfgs-contact-diagnostics-20260922.md) records; stages 3 and 5 remain
planned. Stage 4 found the public L-BFGS failures to be a controller/allowance
mismatch, not a solver defect: with only the semi-implicit stall controller's
100-iteration soft budget removed, all five public smokes converge under L-BFGS
on their configured criterion. Making that budget method-aware is a production
default decision and is pending the user's.**

The BFGS variants have implementation and integration issues that can impair
convergence. The line search is part of the problem, but replacing Armijo alone
would not repair unsafe curvature updates or history spanning changes to the
objective. The experiments below distinguish these defects from convergence
limits on actual contact scenes.

## Scope and provenance

Original audit scope (before stages 1 and 2): review the BFGS variants and their line searches; supply a plan for
issues that need more than a straightforward fix. One independent ordering
correction was made. No curvature policy, contact law, line-search acceptance,
stopping tolerance, restart budget, friction policy, CCD rule or trial cap changed.
The retired constraint floor stays retired; no automatic timestep retry or Teseo
run was introduced.

Starting checkouts were clean: PolyFEM `baac15c5f`, PolySolve
`bce32a39a2c8f0a64cb8ffa85b89f0ee773df0ec`, IPC `482b9eab`.
`polyfem/build/CMakeCache.txt` selects the local `polysolve-merged` checkout;
LBFGSpp is **v0.2.0**, Eigen **5.0.1**. The scene baseline executable predates
this audit and is preserved with its SHA-256. The strategy probe freshly compiles
BFGS, L-BFGS and L-BFGS-B against the configured headers and existing support
libraries; its commands and source/library/executable hashes are recorded.

Raw evidence is in the parent workspace's
`outputs/bfgs-audit/20260922T102120Z/`. The reproducible probe source and compact
results are under [tools/bfgs_audit](../tools/bfgs_audit/README.md).

## Findings

### 1. Repaired in stage 1: BFGS and L-BFGS accepted invalid curvature pairs

Sources in PolySolve:
`src/polysolve/nonlinear/descent_strategies/LBFGS.cpp`,
`descent_strategies/BFGS.cpp`, and the pinned LBFGSpp `BFGSMat.h`.

With `s = x_new - x_old` and `y = g_new - g_old`, L-BFGS unconditionally calls
`add_correction(s, y)`. That library routine divides by `s.dot(y)` and sets its
initial scale using `y.squaredNorm() / s.dot(y)`; it does not validate the pair.
Dense BFGS also divides by `s.dot(y)` and `s.dot(B*s)` without safeguards.
Zero curvature can poison the approximation; negative curvature can make it
indefinite and generate ascent. An energy-decreasing step alone does not
establish positive curvature.

Reproduced on smooth, bounded-below one-dimensional polynomials:

| Objective / start | Accepted first step | Curvature | Subsequent L-BFGS result |
| --- | --- | --- | --- |
| `f(x)=x+x^2(x+1)^2`, `x=0` | `x=-1`, energy `0 -> -1` | `s*y=0` | NaN direction; isolated solver fails at iteration 1 |
| `f(x)=x^4/4-x^2/2+0.1*x`, `x=0` | `x=-0.1`, energy `0 -> -0.014975` | `s*y=-0.0099` | `g*p=+0.0400010101`; isolated solver rejects ascent at iteration 1 |

**Armijo, RobustArmijo and Backtracking all accept those first steps with
alpha=1.** These are their ordinary energy branches, not their roundoff fallback.
Dense BFGS fails on both examples too; the ordering correction makes the invalid
pair take effect immediately, rather than one iteration later. This remaining
defect is explicitly not fixed by the ordering correction.

On Rosenbrock from `(-1.2,1)`, isolated L-BFGS with RobustArmijo rejects ascent
after 4 iterations with gradient norm `1.8220563`. The usual string-form solver
configuration adds gradient descent as a fallback; that combined solver does
converge in 24 iterations. A converged fallback chain therefore does not establish
that the BFGS strategy itself remained valid.

L-BFGS-B already filters pairs using `s.dot(y) > 1e-9*y.squaredNorm()` and passes
these synthetic cases. It should be reviewed for scaling and numerical edge
cases, but it does not have the same unconditional-update defect.

**Stage 1 repaired this**, in [its own record](bfgs-curvature-safeguard-20260922.md):
both strategies validate `s`, `y`, the curvature, the initial scale and the
dense denominators before storing anything, refuse a pair that fails, discard an
approximation that no longer reflects usable curvature or that produces a
non-finite or ascent direction, and report every accepted and refused pair. Both
objectives above now converge with the strategy alone. L-BFGS-B was left on its
own filter, for the reasons given there.

### 2. Repaired in stage 2: history could mix different contact objectives

PolyFEM `BarrierContactForm::post_step` can refresh a formerly empty coefficient
snapshot, bump global trim, perform an optional periodic refresh, or update
classic adaptive barrier stiffness. `FullNLProblem::post_step` forwards this
operation. PolySolve stores the old gradient in its descent strategy and then
calls `post_step` after an accepted displacement. There is no objective-version
notification or history reset when the coefficients change there.

Consequently the next `y` may subtract gradients of two different functions.
Even individually convex objectives can then supply negative apparent curvature.
The direct probe uses `f_old=x^2/8`, `x_old=1`, an accepted step to `x_new=.75`,
then `f_new=2*x^2`. It obtains `s*y=-.6875` and an L-BFGS uphill direction with
`g*p=.8181818182`; for the new objective alone the same displacement would have
positive curvature `.25`.

This synthetic test establishes the mechanism. The source trace establishes
that contact retuning has this lifecycle. It does **not** assign every observed
scene failure to a particular retune. RB-02 had already documented the missing
reset as a limit in [its validation record](rb-02-validation.md).
New `minimize` calls and return from fallback strategies do reset history; the
gap concerns changes inside a minimization call.

**Stage 2 repaired this**, in [its own record](bfgs-objective-generation-20260922.md):
every form counts the changes to its own objective, the problem sums them, and
the solver discards the approximation and the stored iterate and gradient
before the next pair is formed. The public scenes retune themselves three or
four times per Newton solve, and with L-BFGS they now complete their first time
step instead of failing at it. It is a report, not a detection: a form that
retunes without saying so is still invisible.

### 3. High priority integration gap: no Wolfe-capable line search

PolySolve exposes Armijo, RobustArmijo, Backtracking, ResidualBacktracking and
None. Its backtracking loop only reduces the starting alpha and has no endpoint
slope acceptance condition. RobustArmijo adds an energy estimate and bounded
roundoff fallback; neither is a Wolfe curvature condition.

Wolfe curvature, on a fixed smooth objective and a descent direction, gives
`s.dot(y)>0`. The pinned LBFGSpp default uses strong Wolfe, but PolySolve reuses
its matrix machinery with its own line searches. For a current primary-source
implementation reference, [Ceres' inverse-Hessian update](https://ceres-solver.googlesource.com/ceres-solver/+/master/internal/ceres/low_rank_inverse_hessian.cc)
still guards curvature even though its BFGS line search uses Wolfe.
[LBFGSpp's More-Thuente implementation](https://lbfgspp.statr.me/doc/LineSearchMoreThuente_8h_source.html)
also explicitly handles a maximum admissible step. These online references are
design references, not claims that this workspace uses their latest versions.

Simply adding a Wolfe rejection to the existing shrinking loop is insufficient:
some directions require a larger step, while CCD, inversion checks, bounds or
the separate displacement cap may make a Wolfe step unavailable. The algorithm
must preserve feasibility and explicitly handle such capped steps. Wolfe also
cannot make gradients from different objective versions into a valid secant.
Armijo with safeguarded updates remains a useful supported option; the absence
of Wolfe is not proof that every Armijo solve must fail.

### 4. Fixed: dense BFGS used its Hessian one update late

Previously `BFGS::compute_update_direction` solved `B*p=-g` first and only then
incorporated the newest `s,y`. The new order incorporates the pair before the
solve, matching the current-iterate secant contract.

For `f(x)=2*x^2`, after moving from `x=1` to `.5`, the old code returned `p=-2`;
the current secant gives the exact Hessian `4`, hence `p=-.5`. New tests repeat
this with curvatures `4`, `9`, and `.25`, including history reset, and require a
scalar quadratic to converge after its first secant.

- Before: both new tests fail, **4 failed assertions / 28**.
- After: all **13 nonlinear test cases / 1,254 assertions** pass, including the
  new tests, existing box-constrained cases, callback and roundoff regressions.
- Rosenbrock under Armijo and RobustArmijo: dense BFGS improves from **51 to 35
  iterations**, with the configured gradient convergence in both versions.
- Published PolySolve correction:
  [`427e1458123120044bb7dfed89e6673f2fb0dd12`](https://github.com/sdast9/polysolve/commit/427e1458123120044bb7dfed89e6673f2fb0dd12),
  branch `iteration-callback`. The companion PolyFEM recipe pins that revision.

This is an ordering repair, not a new safeguarded BFGS algorithm.

### 5. Separate availability issue: L-BFGS-B is not a forward FEM solver

The Houdini nonlinear menu/export accepts `L-BFGS-B`, but forward elastic and AL
paths use `polysolve::nonlinear::Solver::create`, which does not construct it.
The actual factory probe throws `Unrecognized solver type: L-BFGS-B`.
`BoxConstraintSolver` supports it, and PolyFEM's outer optimization factory
routes to that class. This is separate from convergence of ordinary L-BFGS and
should be reflected in the forward HDA's choices or explicit validation.

## Public-scene baseline

Single-thread runs used full copies of `quasistatic-semi.json` and
`transient-semi.json`, including the original geometry and four-step schedule.
Only the nonlinear method changed; dense BFGS additionally requires
`Eigen::LDLT` instead of the sparse linear solver. The inherited AL settings
therefore follow the chosen method too. Defaults, tolerances, restart budget,
contact settings and time/load increments were retained.

| Method | Quasistatic | Transient |
| --- | --- | --- |
| Newton control | Exit 0; all 4 steps, no restarts | Exit 0; all 4 steps, no restarts |
| L-BFGS | Exit 1 at step 1; 20 restarts | Exit 1 at step 1; 20 restarts |
| Dense BFGS before order fix | Stopped by 90-second probe limit; no completed time step | Stopped by 90-second probe limit; no completed time step |

L-BFGS repeatedly hits the soft iteration limit (101 accepted iterations in a
pass) and retunes/restarts. The final quasistatic pass has gradient norm
`1.7141246842e-5`, versus its rescaled absolute target `2.27951e-6`, and a direction
norm near `1.8e-14`. A restart-limited plateau is not a hard Armijo failure, nor
proof that relaxing a tolerance is the correct remedy. The existing restart
message refers to small alpha even when the soft iteration budget triggered it;
record the actual trigger during follow-up.

These measurements reproduce poor convergence on two public examples. They do
not establish the cause of any private scene, general failure rates or physical
accuracy. Existing generic nonlinear tests allow exceptions in their difficult
problem matrix; the easier matrix enforces success but permits fallback. The
focused no-fallback probe closes a diagnostic gap without making CI deliberately
fail on the still-open issues.

## Remaining implementation plan

Work one stage at a time, retain baseline evidence, and publish each validated
change with its dependency pin. Stages 1 and 2 are complete within their measured
scope. Next, close the bounded review follow-ups below and perform stage 4's
diagnosis before implementing stage 3. Keep the stage numbers stable for links
and existing validation records; they no longer describe execution order.

**Review follow-ups (2026-09-22).** Align the dense factorization-exception
path with the documented safe fallback: it currently resets the matrix and
returns false, which terminates a lone BFGS strategy or escalates a configured
chain. Either implement the promised finite `-grad` fallback with a current
iterate/gradient anchor, or explicitly retain and document escalation. Pin the
chosen behavior with fault injection for both lone and chained configurations.
This branch was not exercised by the recorded scene failures. Add stage 2
regressions for `fDelta`/consecutive-count behavior after retuning and for a
generation change during a failed line search before the next strategy runs.
(Stage 4 closed the third follow-up: the interruption trigger is now recorded,
and the inverse scaling, endpoint slopes and restart effects are measured.)
Add direct generation assertions for committed versus no-op quadrature
refinement and adaptive stiffness changes; existing broad suites do not assert
these new signals. Preserve the completed stages' scoped status, rather than
claiming that every exceptional path has been validated.

1. **Done (2026-09-22): safeguard curvature updates in PolySolve.**
   Published as PolySolve
   [`30f3a3a8b0aa`](https://github.com/sdast9/polysolve/commit/30f3a3a8b0aa291da1c7f738e86bc134a8a269a1) with the record in
   [bfgs-curvature-safeguard-20260922.md](bfgs-curvature-safeguard-20260922.md).
   The fixed corpus chose `curvature_policy: Skip` over damping and showed the
   relative threshold's value to be immaterial between 0 and 1e-2. It also
   showed the stage's own text to be insufficient: retaining the last valid
   approximation indefinitely leaves a stale *descent* direction that the line
   search accepts, so the solver's fallback is never reached, and the public
   chain stalled at the iteration limit on Rosenbrock under skipping and on the
   quartic under damping. A bounded `curvature_restart` (default 1) supplies the
   missing signal. The original text of this item: Add deterministic regressions
   for the bounded polynomials above, zero displacement, near-zero positive
   curvature, overflow/nonfinite intermediate values, and positive quadratics
   across scales. Validate `s`, `y`, curvature, the inverse scale, and dense
   denominators before mutating history. Start by skipping invalid pairs and
   retaining the last valid approximation; if that approximation produces a
   nonfinite or non-descent direction, clear it and use the existing safe
   fallback. Compare a relative, scale-aware threshold with damping on the
   fixed corpus before choosing either as the broader policy. Log accepted /
   skipped pairs and reset reasons. Acceptance: finite descent directions away
   from stationarity, no invalid pair in history, pure-strategy regression
   convergence at unchanged tolerances, and unaffected valid-pair controls.

2. **Done (2026-09-22): reset history when the objective actually changes.**
   Published as PolySolve
   [`440cd55cdaa0`](https://github.com/sdast9/polysolve/commit/440cd55cdaa09bef079578b128bfe7f6f1e869af)
   with the record in
   [bfgs-objective-generation-20260922.md](bfgs-objective-generation-20260922.md).
   The original text of this item: Add a problem-level
   objective generation counter or equivalent explicit signal. Propagate real
   stiffness/trim/friction-objective changes through FullNLProblem/NLProblem;
   invalidate quasi-Newton history before forming the next pair. Reset related
   objective-difference bookkeeping where necessary. Do not reset just because
   the coordinates or the active collision list changed when the mathematical
   objective is unchanged. Test both unchanged and retuned controls, same-
   position changes, AL/reduced boundaries and rollback. Acceptance: all stored
   pairs belong to one objective generation; the current retuning law and
   Newton controls remain intact.

3. **Next, informed by stage 4: add an optional Wolfe search that respects feasibility.**
   Stage 4 measured where its value is: target the bracket-and-grow half.
   Implement or
   adapt a bracket-and-zoom / More-Thuente search, with explicit Armijo and slope
   tests and a finite evaluation budget. Reuse `solution_changed`, validity,
   CCD and trial-cap handling. Growth beyond a previously swept interval must
   rebuild/revalidate that interval. Define a capped-step outcome for cases
   where no Wolfe point exists before the admissible boundary; retain a feasible
   decreasing step only under a stated safeguarded update rule. Cover entry /
   exit cleanup on exceptions and finite-energy prechecks. Test needing alpha
   greater than one, cap below the curvature threshold, nonfinite trials,
   zero CCD step, new contacts, energy roundoff and objective-version changes.
   A generation change invalidates stored endpoint energies and slopes as well
   as quasi-Newton history: abort/restart that search under one objective, with
   a bounded restart budget and cleanup. Compare with safeguarded RobustArmijo
   before proposing a default change. Positive accepted curvature alone does
   not establish the Wolfe curvature condition or rule out a benefit from growth.

4. **Done (2026-09-22): measure contact convergence with useful diagnostics.**
   Published with the record in
   [bfgs-contact-diagnostics-20260922.md](bfgs-contact-diagnostics-20260922.md).
   The plateau is not a line search, a curvature policy or a collapsed
   direction: on `quasistatic-semi` under L-BFGS the line search accepts
   `alpha = 1` without backtracking in 3,707 of 4,054 accepted iterations, no
   secant pair is refused, the gradient-descent fallback is never reached, and
   the direction-to-gradient ratio matches Newton's on the same scene. Every
   one of the 136 measured L-BFGS restarts came from the **soft iteration
   budget**, which the old message reported as an alpha collapse. Removing only
   that budget converges all five public smokes on the configured
   relative-gradient criterion, at the same solution Newton reaches; raising
   `max_iterations` alone does not. `quasistatic-adaptive` has the separate,
   simpler cause of PolySolve's default `max_iterations: 500`. The measurement
   also settles the open question under item 3: strong Wolfe already holds at
   the accepted alpha in 98 % of iterations, while 78 % sit at `alpha = 1` with
   no feasibility cap and a still-negative endpoint slope — so stage 3's value
   here is in its growth half, not its rejection half. The original text of
   this item: First reuse
   the stage 2 records as the baseline. The final quasistatic L-BFGS pass has
   100 accepted pairs, no skips or objective changes, and accepts alpha 1 with
   no search shrinkage, yet its reported gradient norm is 1.10001 against a
   2.27951e-6 target when the soft budget interrupts it. This establishes an
   unresolved plateau, not its cause. Capture accepted endpoint slopes, the
   initial inverse-Hessian scale and direction-to-gradient norm ratio alongside
   the metrics below, so tiny directions can be distinguished from rejected
   line-search steps. Repeat the public
   quasistatic/transient pair, then the other three public smokes and a fixed-
   objective contact control. Record final rescaled residuals, termination
   criteria, accepted displacement, alpha relative to the feasible cap, pair
   curvature/scale, objective versions, skip/reset/fallback counts and whether a
   restart came from alpha or the soft iteration budget. A diagnostic experiment
   can isolate the 100-iteration history reset, but production restart/controller
   defaults remain a separate decision. Compare a bounded uninterrupted pass
   against the existing restart policy from the same saved starting state;
   identify any simultaneous trim/objective changes so the comparison does not
   attribute their effect to history retention. Test conditioning/initial scaling
   one factor at a time at unchanged tolerances. Report failures, completed time
   steps, residuals and cost separately. Diagnostic completion requires evidence
   that selects the next experiment; a solver repair is accepted only on actual
   configured convergence, not an interrupted solve or an increased allowance
   alone. Do not change the production soft budget merely to make a run finish.

5. **Clarify forward-method support and strengthen test reporting.** Remove or
   explain the unsupported forward L-BFGS-B option while retaining legitimate
   box-constrained optimization support. Add an HDA JSON round-trip check and a
   clear forward validation message. Maintain deterministic BFGS tests that
   fail on exceptions and record which strategy converged; keep intentionally
   permissive stress tests separately labelled. Rebuild, run affected PolySolve
   and PolyFEM suites, all public smokes and HDA E2E before publication.

## Integration validation

| Check | Result |
| --- | --- |
| PolyFEM CMake build, macOS arm64 RelWithDebInfo | `PolyFEM_bin` and `unit_tests` rebuilt successfully with the pinned PolySolve correction |
| PolySolve nonlinear suite | 13 cases / 1,254 assertions, exit 0; fresh BFGS object and tests linked to configured support libraries |
| PolyFEM `[al_solver],[direction_filter]` | 26 cases / 4,060 assertions, exit 0 |
| Five public Newton smoke controls | 5/5 exit 0; all four time steps complete, zero error lines and zero restarts |
| Before/after Newton output comparison | All five VTU frames byte-identical for both quasistatic and transient baseline controls |
| Dense BFGS after the ordering fix | Both public scenes still fail at step 1 after 20 restarts, exits 1, approximately 71 / 69 seconds; the broader issues remain open |
| Houdini `test_polyfem_hda.py` | Exit 0; strict JSON / solver-panel round-trip / end-to-end checks pass against the rebuilt executable |

The initial sandboxed CMake attempt could not create its dependency-cache lock;
the authorized build with cache access succeeded. The first HDA attempt could
not import `gmsh`; the rerun exposed the already-installed module using
`PYTHONPATH=/opt/homebrew/lib` and passed. Both initial logs are retained.

No general BFGS convergence claim is implied by passing the affected suites.
The completed repair and published plan preserve all outstanding failures.
