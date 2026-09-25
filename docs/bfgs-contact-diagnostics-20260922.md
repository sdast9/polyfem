# Contact convergence diagnostics — audit stage 4

Date: 2026-09-22. **Status: diagnosis complete within the stated scope. No
production default changed; the decision it selects is named at the end and is
the user's.**

Stage 4 of the [BFGS convergence audit](bfgs-convergence-audit-20260922.md)
asked what actually keeps L-BFGS from converging on the public contact scenes,
with diagnostics good enough to tell a collapsed direction from a rejected
line-search step. The answer is neither: on every public scene the solver's own
iteration is healthy, and the thing that ends it is a **controller setting**.
Removing only the semi-implicit stall controller's 100-iteration soft budget —
changing no tolerance, no line search, no curvature policy and no contact law —
makes **all five public smokes converge under L-BFGS on their configured
criterion**, to the same solution Newton finds.

## Scope

Nothing in the solver's numerical behaviour changed. The two source changes are
reporting: PolySolve gained an opt-in per-iteration diagnostic record, and
PolyFEM now names the condition that actually triggered a stall restart (it
named the wrong one before) and carries the PolySolve record into the RB-04
attempt stream. No retuning law, stopping tolerance, restart budget,
line-search rule, CCD rule, trial cap or friction policy changed. The retired
constraint floor stays retired; no automatic timestep retry was introduced and
Teseo was not run.

Starting checkouts were clean: PolyFEM `8fd6ef633`, PolySolve `440cd55c`,
IPC `482b9eab`; the build uses the local companion checkouts.

Raw evidence, with every input, log and diagnostic stream, is in the parent
workspace's `outputs/bfgs-stage4/20260922T162928Z/`; the compact reduction is
its `stage4-summary.json`. The runner is [tools/bfgs_stage4](../tools/bfgs_stage4/README.md).

## The instrumentation

`solver/nonlinear/advanced/iteration_diagnostics` (default **false**) makes
PolySolve record, for each accepted iteration: which strategy produced the
direction and from what — a limited-memory direction, the first/reset steepest
descent one, a curvature restart, or the separately configured
`GradientDescent` fallback — the direction's norm against the gradient's and
against the previous accepted direction's, the secant pair and the initial
inverse-Hessian scale, the line search's feasible bound with the accepted alpha
as a fraction of it, the accepted endpoint's gradient and slope, and every
strategy transition since the previous accepted iterate, with the rejected
direction that caused it.

With `output/physical_diagnostics` on, PolyFEM carries that record in the
`solver` field of each accepted row of `solver-attempts.jsonl`
(schema `polyfem.solver-attempt` version 2).

It costs one extra gradient evaluation per accepted iteration and **decides
nothing**. The control that makes the rest of this record admissible: all five
public Newton smokes produce **byte-identical VTU frames** with the option off
and on, and identical frames to the stage 2 record.

## The restart message named the wrong condition

`ALSolver`'s stall callback has two independent triggers — the small-alpha
patience and the soft iteration budget — and the restart message said
`alpha < {threshold} for {patience} iterations` whichever one fired. The
trigger is now recorded (`stall_trigger`, `stall_iteration`, `stall_alpha` in
the solve info, and named in the log). Measured on the four semi-implicit
scenes under L-BFGS:

| scene | restarts | from alpha | from the soft budget |
| --- | --- | --- | --- |
| `quasistatic-semi` | 39 | 0 | **39** |
| `quasistatic-semi-alhess` | 39 | 0 | **39** |
| `quasistatic-semi-friction` | 19 | 0 | **19** |
| `transient-semi` | 39 | 0 | **39** |

Not one restart came from a line-search collapse. The final `quasistatic-semi`
pass ends at `alpha = 1`. Reading the old message is what sent the audit
looking at Armijo.

## The iteration is healthy; there are just very many of them

The per-iteration record of `quasistatic-semi` under L-BFGS, over its 4,054
accepted iterations:

| measurement | value | reading |
| --- | --- | --- |
| accepted at `alpha = 1` | 3,701 / 4,054 | the line search almost never shrinks the step |
| line search with no backtracking at all | 3,707 / 4,054 | and almost never even tries |
| accepted below the feasible cap | 346 / 4,054 | CCD and the forms are rarely the binding constraint |
| direction norm / gradient norm, median | 2.0e-7 | **Newton's is 1.8e-7 on the same scene** |
| refused secant pairs, history resets | 0, 0 | stage 1's safeguard never fires here |
| escalations to `GradientDescent` | 0 | the fallback chain is never reached |
| strong Wolfe (`c2 = 0.9`) already satisfied | 3,976 / 4,054 | at the accepted alpha, measured not imposed |

The direction is not collapsed: it is scaled like Newton's on the same problem,
and the barrier's stiffness is why both are small against the gradient. No
invalid pair, no reset, no fallback, no line-search failure. This is an L-BFGS
iteration doing ordinary work — it simply needs hundreds of times more
iterations than Newton, and the controller stops it at 100.

## The same starting state, with and without the interruption

Step 1 of `quasistatic-semi` starts both runs from the identical iterate. They
are **bit-identical for the first 101 accepted iterations** and diverge exactly
at the first restart:

| accepted iteration | restart policy, ‖∇f‖ | uninterrupted, ‖∇f‖ |
| --- | --- | --- |
| 101 (the last shared one) | 1.18014159e+04 | 1.18014159e+04 |
| 102 | 1.20853621e+04 | 1.18107125e+04 |
| 501 | 3.71e+02 | 8.38e+01 |
| 1001 | 7.37e-01 | 5.03e-02 |

At step 2 the gap is three orders of magnitude at comparable cost: the restart
policy is at ‖∇f‖ = 1.10 after 2,121 accepted iterations, the uninterrupted
pass at 9.67e-4 after 2,000.

**The confound the plan asked to identify.** The two runs do not solve the same
sequence of objectives: at step 2 the restart run passes through 25 objective
generations against the uninterrupted run's 5, because each stall restart
re-freezes the per-contact stiffness snapshot. The barrier trim, however, is
**not** a confound: both runs follow the identical trim sequence
(1, 2, 4, 8, 16, 32, 64, 32), so the restarts did not make the problem stiffer
than the ordinary controller already does. Nor are history resets fatal in
themselves — the uninterrupted pass takes five of them and still converges.

## What removing only the soft budget does

| run (`quasistatic-semi`, L-BFGS) | exit | time steps | restarts |
| --- | --- | --- | --- |
| production settings | 1 | 1 of 4 | 39, all from the budget |
| `max_iterations: 20000` alone, controller untouched | **1** | 1 of 4 | 39, all from the budget |
| `soft_iteration_limit: -1` (restart machinery still enabled) | **0** | **4 of 4** | **0** |

Raising the iteration allowance alone changes nothing: the controller still
interrupts every 100 iterations. Removing only the budget — leaving the restart
machinery, its alpha threshold, its patience and its retune in place — converges,
and no alpha stall ever fires in the thousands of iterations that follow.

The result holds across the public set, every one of them stopping on the
**configured** relative-gradient criterion (`‖∇f‖_rel < 1e-10`), not on an
iteration limit and not on a relaxed tolerance:

| scene | exit | L-BFGS accepted iterations | Newton | L2 error, L-BFGS vs Newton |
| --- | --- | --- | --- | --- |
| `quasistatic-semi` | 0 | 8,841 (1304/2123/2594/2820) | 31 | 0.1693355 vs 0.1693757 |
| `quasistatic-semi-alhess` | 0 | 8,841 | 31 | 0.1693355 vs 0.1693757 |
| `transient-semi` | 0 | 9,531 | 26 | 0.1693232 vs 0.1693757 |
| `quasistatic-semi-friction` | 0 | 13,732 (with its lagging passes) | 62 | 0.1637441 vs 0.1637846 |
| `quasistatic-adaptive` | 0 | 34,835 | 37 | 0.1694713 vs 0.1693757 |

L-BFGS needs 220× to 940× Newton's iterations on these scenes and arrives at
the same solution. Wall times stay in the same order as Newton's on four of the
five (25–42 s against 5–10 s) because an L-BFGS iteration is far cheaper; the
adaptive scene costs 220 s.

`quasistatic-adaptive` has a **second, separate cause**: it carries no
semi-implicit restart block, so nothing interrupted it — its failure was purely
PolySolve's default `max_iterations: 500`. It is also this stage's
fixed-objective contact control, and it behaves the same way.

## One factor at a time

At unchanged tolerances, on `quasistatic-semi` under the production controller:

| factor | result |
| --- | --- |
| L-BFGS history size 3 / 6 / 12 / 24 | 1 / 2 / 2 / 2 time steps; every restart still from the budget. Not a history-length problem. |
| line search `Armijo` | worse: 1 time step, and 10 of its 20 restarts now really are alpha stalls |
| line search `Backtracking` | worse: 1 time step, 2 restarts from a line search that failed on every strategy |
| line search `RobustArmijo` (default) | the best of the three |
| dense `BFGS` | a **different** failure: direction norm ≈ gradient norm (median ratio 0.93, the approximation is near-identity), `alpha = 1` never accepted, 208 of 210 steps below the feasible cap, and all 20 restarts from alpha. Its plateau is not the L-BFGS one and this stage does not diagnose it. |

## What this says about stage 3

Measured, not assumed. On the converged `quasistatic-semi` run, of 8,841
accepted iterations:

- **8,665 already satisfy the strong Wolfe curvature condition** at `c2 = 0.9`
  at the accepted alpha. A Wolfe *rejection* test would change almost nothing
  here.
- **6,942 accept `alpha = 1` with no feasibility cap and an endpoint slope that
  is still negative** — the function is still descending where the search
  stops, which is exactly and only where a **growth** step (`alpha > 1`) is
  admissible. On the adaptive scene it is 24,479 of 34,835.

So stage 3's value on these scenes lies in the bracket-and-grow half of a Wolfe
search, not in its rejection half. That is a measurement on five public scenes,
not a prediction that growth will pay: the audit's own caution applies, and the
feasible cap, CCD and the trial-displacement cap still bound any growth.

## Acceptance

| check of the plan's stage 4 | result |
| --- | --- |
| Reuse the stage 2 records as the baseline | the five Newton smokes reproduce the stage 2 frame hashes byte for byte |
| Capture accepted endpoint slopes, initial inverse-Hessian scale, direction-to-gradient ratio | all three per accepted iteration, in `solver-attempts.jsonl` |
| Repeat the public pair, then the other three smokes and a fixed-objective contact control | all five, under Newton and L-BFGS; `quasistatic-adaptive` is the fixed-objective control |
| Record residuals, termination, accepted displacement, alpha against the feasible cap, pair curvature/scale, objective versions, skip/reset/fallback counts | `stage4-summary.json`, per run |
| Whether a restart came from alpha or the soft iteration budget | recorded and named; 136 of 136 L-BFGS restarts came from the budget |
| Bounded uninterrupted pass against the restart policy from the same saved starting state | step 1 of `quasistatic-semi`: identical for 101 iterations, then compared to convergence |
| Identify simultaneous trim/objective changes | trim sequence identical in both; the objective-generation counts differ (25 vs 5) and are reported as a limit, not explained away |
| Conditioning / initial scaling one factor at a time | history size and line-search method, each alone, at unchanged tolerances |
| Report failures, completed time steps, residuals and cost separately | the tables above |
| Evidence that selects the next experiment | it does; see below |
| A repair accepted only on actual configured convergence | the converging runs stop on `‖∇f‖_rel < 1e-10`, the configured criterion, unchanged |
| Do not change the production soft budget merely to make a run finish | **not changed.** The budget is removed only inside diagnostic runs |

## Limits

- Five public scenes, one machine, single-threaded, one repetition each. This
  establishes the mechanism on those scenes; it is not a failure rate, a
  benchmark or a claim about private scenes.
- The two compared runs differ in their objective-generation sequence (25 vs 5
  at step 2). The trim path is identical and the uninterrupted run survives its
  own resets, which is why the budget is named as the cause — but the
  comparison is not a controlled isolation of history retention alone, and is
  not presented as one.
- The convergence shown is of the **solver**, to the configured criterion, at
  the same solution Newton reaches on the same scenes. It is not a statement
  about physical accuracy, and the L2/H1 agreement quoted is the two methods
  agreeing with each other, not with an analytic answer.
- Dense BFGS's plateau is a different mechanism and remains undiagnosed.
- The wall times are single-run measurements.
- The diagnostics are a report of what the strategies say about themselves. A
  strategy that misreports its own direction source is invisible, exactly as
  stage 2's generation signal is a report and not a detection.

## What this selects, and the decision it is not allowed to make

The next experiment is **not** a solver change. The measurement says the
public L-BFGS failures are a controller/allowance mismatch: the semi-implicit
stall controller's `soft_iteration_limit` of 100 and PolySolve's
`max_iterations` of 500 are sized for a method that converges in tens of
iterations, and L-BFGS needs thousands. Whether those defaults should become
method-aware is a **production default decision and therefore the user's**, in
the same way RB-07's and RB-08's were. The audit's own rule — do not change the
production soft budget merely to make a run finish — is why this record stops
here rather than shipping the change. **Decided 2026-09-25 (user): leave the
soft budget method-independent** (L-BFGS is no longer pursued; on the real
scenes it fails on conditioning, see the
[quasi-Newton investigation](qn-contact-investigation-20260922.md)).

For the record, the shape of the option if it is wanted: leave the alpha-stall
threshold, patience and retune exactly as they are (they never misfired in any
measured run) and make only the soft iteration budget depend on the configured
nonlinear method, with the present value kept for Newton. That is the smallest
change consistent with every measurement above, and it would need its own
validation pass across the public set and the HDA before publication.

Stage 3's implementation, when it comes, should target the growth half of a
Wolfe search; its rejection half has nothing to reject on these scenes.

## Published revisions

PolySolve: the per-iteration diagnostics, branch `iteration-callback`.
PolyFEM: this commit. Evidence in the parent workspace's
`outputs/bfgs-stage4/20260922T162928Z/`.
