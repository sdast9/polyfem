# Review of BFGS audit stages 1 and 2

Date: 2026-09-22. Reviewed PolyFEM `5e9a06e8c` and PolySolve
`440cd55cdaa0`, both clean and synchronized with their remote branches at review
entry. This is a source/evidence review and plan update; no numerical policy or
production code changed.

## Decision

Keep both stages complete within their documented scope. Their core mechanisms
are supported by the implementation and regressions. Change the next action to
stage 4 contact diagnostics, then use those results to guide stage 3's optional
Wolfe implementation. Close the bounded coverage/contract follow-ups below.
Neither stage establishes contact convergence.

Stage 1 validates pairs before storing them, checks dense denominators and the
candidate matrix, and validates computed directions. Its fixed corpus supports
Skip with a bounded refusal restart over retaining stale history indefinitely.
Stage 2 resets both the approximation and the stored iterate/gradient before
forming another pair when a reported objective generation changes. Dropping the
old energy prevents an energy difference across two objectives from triggering
the energy-difference stopping criterion. The original post-step retune defect
is covered, including a spanning pair whose curvature would otherwise pass.

## Follow-ups found in the implementation

1. **Dense factorization failure differs from the stage 1 record.** In
   `polysolve/nonlinear/descent_strategies/BFGS.cpp`, the `runtime_error` catch
   records `factorization_failed`, resets the matrix and returns false before
   updating the stored iterate/gradient. `Solver::minimize` then escalates the
   strategy, or throws `UpdateDirectionFailed` for a lone BFGS strategy. The
   record says this path falls back to steepest descent. The nonfinite/ascent
   direction path does implement that fallback; the exception path does not.
   Resolve the contract and add an injected-failure test for lone and chained
   configurations. This is a source-level discrepancy, not a reproduced cause
   of the contact failures. The Eigen dense wrapper itself does not throw on
   every numerical factorization failure, so test numerical direction failures
   separately from exceptions.

2. **Stage 2's broader boundary claims need direct tests.** The new objective
   tests exercise post-step retuning, unchanged caches, contact refresh/trim,
   generation aggregation and restored base weights. They do not directly pin
   `fDelta` stopping across a retune, a generation change followed by failed
   line search and strategy escalation, or the generation increment at elastic
   quadrature commits and classic adaptive stiffness updates. The source adds
   these signals, and broad form suites pass, but those suites do not assert
   their generation behavior. Add focused tests, including no-op controls.
   Keep the known boundary explicit: callers' weight/enable changes between
   minimizations rely on the ordinary solve reset, not these counters.

3. **The next experiment should separate small directions from line-search
   shrinkage.** The final `quasistatic-semi` L-BFGS pass reports 100 accepted
   pairs, zero discarded pairs/history resets/objective changes, alpha 1 and no
   search shrinkage on its final step. Its gradient norm is 1.1000088878 against
   a 2.27951e-6 target, with direction norm about 7.14e-10; the iteration callback
   interrupts it at the soft budget. The log still calls this a line-search
   stall. Record the actual interruption trigger and inspect inverse scaling,
   endpoint slopes and restart effects before attributing it to Armijo. This
   does not rule out Wolfe growth helping: valid curvature is weaker than a
   Wolfe curvature condition.

The revised [implementation plan](bfgs-convergence-audit-20260922.md#remaining-implementation-plan)
incorporates these items without changing tolerances, CCD, trial caps, retuning
laws or production restart settings.

## Evidence checked

- Stage 1 corpus and published corpus are byte-identical. Its record reports
  Skip 96/96 versus Damp 95/96 on the policy matrix. This remains synthetic
  evidence, not a contact convergence claim.
- Stage 2's saved test executable hash matches its record. All six recorded
  test source hashes and all 16 library hashes match the current files. The
  record was made before the stage 2 commit and correctly records a dirty
  stage 1 parent; the matching hashes provide the additional content check.
- Rehashed all nine stage 2 scene inputs and every recorded VTU frame: no
  mismatch. The five Newton comparisons all report exit 0 and identical frames.
  The post-formatting check separately repeats those five Newton controls.
- Stage 2 L-BFGS completes step 1 and fails at step 2 in both public scenes;
  dense BFGS still fails at step 1. All four records have exit 1. Wall times are
  single-run measurements, not a benchmark establishing the cause of overhead.
- Inspected the saved full-suite summaries: PolySolve 24 cases / 1,706
  assertions; PolyFEM 88 cases / 10,310 assertions. Reran the focused tests:
  PolySolve `[bfgs]` **13 cases / 472 assertions**, PolyFEM
  `[objective_generation]` **3 cases / 43 assertions**, both passed. These runs
  use existing binaries; this documentation review did not require a rebuild
  or a repeat of the scene matrix.

Saved evidence roots in the parent workspace:
`outputs/bfgs-stage1/20260922T113940Z/`,
`outputs/bfgs-stage2/20260922T135940Z/`. Review checks and focused test logs:
`outputs/bfgs-stage12-review/20260922/` (`evidence-check.json`,
`polysolve-bfgs.log`, `polyfem-objective.log`).

## Dense solver clarification

The static linear-solver JSON specification is not the runtime option list.
`linear::Solver::apply_default_solver` replaces it with `available_solvers()`
before validation. Dense Eigen names, including `Eigen::LDLT`, therefore work
through JSON; the recorded dense BFGS scene inputs use that option. The earlier
conversational claim that the schema prevents their use was incorrect. HDA menu
exposure is a separate question; no HDA change is implied by this review.
