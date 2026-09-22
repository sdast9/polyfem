# BFGS convergence audit and repair plan

Date: 2026-09-22. **Status: dense-BFGS update-order defect repaired;
broader convergence repairs remain planned.**

The BFGS variants have implementation and integration issues that can impair
convergence. The line search is part of the problem, but replacing Armijo alone
would not repair unsafe curvature updates or history spanning changes to the
objective. The experiments below distinguish these defects from convergence
limits on actual contact scenes.

## Scope and provenance

Requested: review the BFGS variants and their line searches; supply a plan for
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

### 1. High priority: BFGS and L-BFGS accept invalid curvature pairs

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

### 2. High priority: history can mix different contact objectives

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
change with its dependency pin. Prioritize stages 1 and 2 before evaluating a
new line-search default.

1. **Safeguard curvature updates in PolySolve.** Add deterministic regressions
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

2. **Reset history when the objective actually changes.** Add a problem-level
   objective generation counter or equivalent explicit signal. Propagate real
   stiffness/trim/friction-objective changes through FullNLProblem/NLProblem;
   invalidate quasi-Newton history before forming the next pair. Reset related
   objective-difference bookkeeping where necessary. Do not reset just because
   the coordinates or the active collision list changed when the mathematical
   objective is unchanged. Test both unchanged and retuned controls, same-
   position changes, AL/reduced boundaries and rollback. Acceptance: all stored
   pairs belong to one objective generation; the current retuning law and
   Newton controls remain intact.

3. **Add an optional Wolfe search that respects feasibility.** Implement or
   adapt a bracket-and-zoom / More-Thuente search, with explicit Armijo and slope
   tests and a finite evaluation budget. Reuse `solution_changed`, validity,
   CCD and trial-cap handling. Growth beyond a previously swept interval must
   rebuild/revalidate that interval. Define a capped-step outcome for cases
   where no Wolfe point exists before the admissible boundary; retain a feasible
   decreasing step only under a stated safeguarded update rule. Cover entry /
   exit cleanup on exceptions and finite-energy prechecks. Test needing alpha
   greater than one, cap below the curvature threshold, nonfinite trials,
   zero CCD step, new contacts, energy roundoff and objective-version changes.
   Compare with safeguarded RobustArmijo before proposing a default change.

4. **Measure contact convergence with useful diagnostics.** Repeat the public
   quasistatic/transient pair, then the other three public smokes and a fixed-
   objective contact control. Record final rescaled residuals, termination
   criteria, accepted displacement, alpha relative to the feasible cap, pair
   curvature/scale, objective versions, skip/reset/fallback counts and whether a
   restart came from alpha or the soft iteration budget. A diagnostic experiment
   can isolate the 100-iteration history reset, but production restart/controller
   defaults remain a separate decision. Test conditioning/initial scaling only
   after history correctness. Acceptance is actual configured convergence, not
   an interrupted solve or an increased iteration allowance alone.

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
