# Quasi-Newton and ADAM on real 3D contact: fundamental or fixable?

Date: 2026-09-22/23. **Status: investigation complete within the stated scope.
No production default, tolerance, controller setting or solver behaviour
changed.** The experimental code is opt-in and lives on the unpinned PolySolve
branch `qn-contact-experiment`; the pinned PolySolve rejects its options.

## The question

After the [BFGS audit](bfgs-convergence-audit-20260922.md) (stages 1–5) the
user observed that on real 3D contact scenes only Newton converges: ADAM and
the (L-)BFGS variants stop descending past some point. Is that a fundamental
limitation of these methods, given how the line search is truncated (CCD and
the 50·dhat trial-displacement cap), or can they be made a useful alternative
to Newton?

## Answer

1. **The line-search truncation is not what stops them.** On the real plate +
   ball-head scene (R1) plain L-BFGS accepts α = 1, with the feasible bound at
   1 and no backtracking, in almost every iteration; its steps (~1e-11 m against
   dhat = 1e-6) are too small to reach the trial cap (active in 1 % of its line
   searches) or CCD. Truncation binds **Newton**: on the large scene R4 its
   trial sweeps average ~800 barrier widths, the cap is active in 99 % of its
   line searches and CCD then keeps a median 5.7 % of the capped sweep. On real
   scenes truncation sets Newton's iteration count; it does not separate
   Newton from L-BFGS.
2. **Plain L-BFGS, dense BFGS and ADAM are fundamentally unsuitable here.**
   The Hessian's condition number on R1 is at least **3.5e13** (at least 9e9
   once the bodies are in contact) — a rigorous lower bound measured from the
   runs themselves. L-BFGS's only curvature scale is the scalar y·y/s·y, which
   sits at 1e7–4e8 on R1 and locks onto the stiffest barrier/material mode, so
   every softer mode moves ~1e-8 of its Newton step per iteration; methods of
   this kind need of the order of √κ to κ iterations. Uninterrupted, L-BFGS
   reached 9 % of Newton's step-1 energy decrease in 9,180 iterations. ADAM's
   normalised step (~1e-3, 1000× dhat) carries no curvature at all: the line
   search halves it 23–29 times every iteration and the solve fails. Dense BFGS
   starts from the identity and needs n² memory (1.4 GB at R1's 13k DOFs,
   ~4 TB at R4's 713k). No line search can supply the missing scale or
   coupling. The "cannot descend past a point" symptom is what a poor
   direction looks like once the energy decrease falls below the energy's
   resolution: the line search then accepts only steps that lower ‖∇f‖.
3. **Three Newton-tuned mechanisms make it worse, and one turns the failure
   into a silently wrong answer:**
   * **False convergence.** `ALSolver` (`src/polyfem/solver/ALSolver.cpp:254`,
     the `slope_tolerance` branch, from PF-01's `4b6e970c3`) accepts
     PolySolve's configured slope tolerance
     (`advanced/derivative_along_delta_x_tol`) as convergence for **every**
     method, whatever `allow_non_grad_convergence` says. For Newton |g·Δx| is
     the Newton decrement; for L-BFGS it is γ‖g‖² with γ ≈ 1e-8 and is met
     almost at once. L-BFGS "converged" R1 step 1 after one iteration with
     **100 %** error, and completed all six steps of the easy inflation scene R3
     with exit 0 and **4–7 %** error in every step.
   * **The stall controller reads truncation as a stall.** Its absolute trigger
     (α < 0.01 for five iterations) fires while every step sits exactly at the
     feasible bound — for Newton too — and exhausts a Newton-sized direction's
     restart budget (fixed-interval preconditioned runs fail R1 on it).
   * **The trim controller never lets a history form.** It changes the
     objective every ~30 iterations (`controller_interval`), and each change
     rightly discards the quasi-Newton history (audit stage 2): 297 objective
     changes in 9,180 L-BFGS iterations on R1.
4. **It is fixable — by giving the method Hessian information, which makes it
   a Newton variant.** L-BFGS whose initial matrix is a lagged factorization of
   the PSD-projected Hessian, refreshed every 10 iterations *and* after any
   truncated or backtracked step, with the secant pairs dropped at each refresh
   (variant **B** below), reaches Newton's answers on every scene with fewer
   factorizations, and where a factorization is expensive it is faster than
   Newton: **R3 1.6×** (fixed refresh), **R4 1.3–1.6×** on a common objective;
   break-even on R1 (75 s vs 71 s); slower only on the tiny public smoke. A
   fixed refresh interval alone is not enough (slower than Newton on R1 and
   R4), a diagonal initial matrix does nothing, and the secant pairs help in
   mild contact but poison the direction in dense contact.

Two side findings matter more for real scenes than the choice of minimizer:
most of Newton's 727 R4 step-1 iterations are the trim controller walking the
barrier trim down from 1 to 2⁻¹¹ one halving at a time (with the trim pinned at
that value Newton needs 111–134), and R4's final configuration differs by
3.5–6 % between identical Newton runs.

## Scenes

| id | scene | size | notes |
| --- | --- | --- | --- |
| smoke | `scenes/semi-implicit/quasistatic-semi.json` | 125-node cube on slab | the audit's public scene; 4 steps; stage 4's soft budget removed so every method can reach `‖∇f‖_rel < 1e-10`; `SimplicialLDLT` (bit-reproducible) |
| R1 | user's plate + ball head (`test_cases/input`) | 4,350 nodes, 13k DOFs | E 1e6 plate vs 1e11 ball, dhat 1e-6, dt 0.3, Neumann push; 6 steps |
| R3 | user's inflation against an obstacle (`test_cases/inflation`) | 8,148 + 4,398 obstacle nodes | Newton needs 3–5 iterations per step; 6 steps |
| R4 | user's uniaxial self-contact scene (`test_cases/uniax_mesh_constraintfloor_zero_b`) | 237,780 nodes, ~713k DOFs | frictionless; the scene of the user's own L-BFGS attempt; 18 threads |

R1, R3 and R4 run the user's exported settings, including their
`Eigen::AccelerateLDLT`, which threads internally and is **not bit-reproducible**:
two identical single-threaded R1 Newton runs diverge at the second iteration
(final difference 1.3e-6; 459 vs 480 iterations over six steps across two
builds). They stop on the directional-derivative test; on R1 not even Newton
can reach `‖∇f‖_rel < 1e-10` (with that test off it stalls at 0.16–0.23).
Accuracy is therefore the relative L2 difference of the `solution` field from a
Newton reference with the slope tolerance tightened to 1e-15.

## Measurements

### What limits each method on R1

| | plain L-BFGS | Newton |
| --- | --- | --- |
| α = 1, feasible bound 1, no backtracking | nearly every iteration | rare |
| trial cap active / CCD fraction of the capped sweep (median) | 1 % / 100 % | 70 % / 25 % |
| ‖p‖/‖g‖ | 1e-8–1e-7 (its scalar scale) | 3e-4 … 1e5 (the whole spectrum) |

Condition-number lower bound (`tools/qn_contact/kappa.py`): for any g,
‖H⁻¹g‖/‖g‖ ≤ 1/λ_min, so Newton's largest ratio bounds 1/λ_min from below;
y·y/s·y is a Rayleigh quotient of the path-averaged Hessian, so L-BFGS's
largest value bounds λ_max from below. Product on R1: **≥ 3.5e13** over six
steps, **≥ 9e9** over steps 2–6. (On the smoke the bound is only 5.9e3 — its
Newton ratios span less — and plain L-BFGS still needs 8,841 iterations.)

### The preconditioned L-BFGS experiment

`L-BFGS/preconditioner: "Hessian"` runs the two-loop recursion with
H0 = a factorization of the PSD-projected Hessian (with the linear solver
configured for Newton), refreshed on every reset and objective change and
every `preconditioner_refresh` iterations; `preconditioner_refresh_short_step:
0.5` also refreshes after a step shorter than half its direction (truncated or
backtracked); `preconditioner_clear_pairs` drops the secant pairs at each
refresh; `history_size: 0` gives the lagged-Hessian (modified Newton) control.
**B** = refresh 10 + short-step 0.5 + clear pairs.

Quiet, sequential runs (nothing else on the machine); wall / accepted
iterations / Hessian factorizations:

| scene | Newton | fixed refresh (5 or 10) | B | error vs tight Newton |
| --- | --- | --- | --- | --- |
| smoke, 4 steps, 1 thread | **5.6 s** / 31 / 31 | r10: 12.8 s / 105 / 15 | 8.7 s / 85 / 19 | all reach `‖∇f‖_rel < 1e-10` |
| R1, 6 steps, 1 thread | **71.2 s** / 480 / 480 | r5: 87.0 s / 786 / 170 | 74.9 s / 451 / 373 | Newton 4.5e-7…5.0e-4; r5 2.3e-6…5.1e-4; B 6.4e-6…5.0e-4 |
| R3, 6 steps, 1 thread | 68.6 s / 20 / 20 | r5: **43.8 s** / 25 / 7 | — | Newton 1.2e-4…6.4e-4; r5 1.5e-4…1.5e-3 |
| R4 step 1, trim pinned at 2⁻¹¹, 18 threads | 240–295 s / 111–134 / same (three runs) | r5: 596 s / 485 / 99 | **181 s / 80 / 71** | Newton vs Newton 3.5–6.1 %; r5 6.4 %, B 6.7 % from the first Newton run |
| R4, 2 steps, production controller, 18 threads | 1,763 s / 795 / 795 | r5: 2,113 s / 1,611 / 347 | 781 s / 327 / 276 | different trim paths (see below) |

What the rows say:

* **Hessian information is what matters.** On the smoke a diagonal (Jacobi)
  initial matrix does not help (9,290 iterations against plain L-BFGS's 8,841):
  the ill-conditioning is coupling, not scaling. A lagged Hessian refreshed
  only at the start of each solve is *worse* than none (>28,500 iterations,
  killed): it reaches the minimum's neighbourhood fast, then sits with the
  energy frozen at 723.931 for 27,000 iterations.
* **Secant pairs help in mild contact and hurt in dense contact.** Smoke,
  refresh 10: history 0 / 1 / 6 / 20 → 270 / 129 / 105 / 110 iterations. On R4
  the pure-H0 directions are Newton-like (‖p‖/‖g‖ ≈ 1e-5) but once pairs are
  added ‖p‖/‖g‖ jumps to 0.07–23 and CCD and the validity checks cut α to ~1e-8
  (13–27 halvings): pairs across the barrier's nonlinearity poison the
  recursion. Dropping them at every refresh (B) is what makes R4 work.
* **Where truncation drives every step, reuse saves little.** On R1 ~70 % of
  steps are truncated, the contact set changes every iteration, and B's
  short-step rule makes it Newton most of the time (373 factorizations vs 480).
  The saving appears where the model stays valid between refreshes: R3's
  stable contact (7 vs 20 factorizations) and R4 once the trim is fixed (71 vs
  111–134, with fewer iterations than Newton as well).
* **Fixed-interval refresh fights the controllers.** Refresh 10 or 20 with the
  production stall controller fails R1 outright (20 restarts exhausted); with
  200 restarts or the controller off it converges (975 / 125 and
  1,612 / 176).
* **R4 accuracy is bounded by R4 itself.** The scene is frictionless
  self-contact: tangential sliding is nearly free, and the per-contact
  coefficients of contacts that appear mid-solve depend on the path even with
  the trim pinned. Identical Newton runs end 5.3 % apart (another 3.5 % with
  the stall controller off), with final energies within 2 %; B ends at the
  lowest energy of the three pinned runs (1.5455e-9 against Newton's
  1.5748e-9). The preconditioned runs' 6–7 % differences are inside Newton's
  own spread.
* **Under the production controller R4 compares different problems.** The
  three two-step runs end on three trims (Newton 4.9e-4, r5 9.6e-4, B 2.1e-2),
  so B's 781 s against Newton's 1,763 s is not like-for-like; the pinned rows
  are the fair comparison.

### Controls

| control | result |
| --- | --- |
| `preconditioner: None` on the smoke | 8,841 iterations, the stage 4 count exactly: the untouched LBFGSpp path |
| smoke Newton, this build | 31 iterations, per step [9, 6, 11, 5] = stage 4 |
| Accepted-iteration counting | the manifest's `termination.iterations` counts only the final sub-solve; the tables count every accepted iteration across stall restarts (R1 step 1: the user's "20 iterations, 5 restarts" ≈ 190 here) |
| User's own L-BFGS + Wolfe settings on R4 (baseline binary, 20-min cap) | still in step 1 after 1,270 iterations; energy 5.5e-5 at 1,200 s where Newton was at 2.6e-7; 45 stall restarts |

## Recommendations (decisions for the user)

**Taken 2026-09-23:** 1 and 2 — repaired and published
([record](slope-tolerance-repair-20260923.md)): the slope tolerance and the
step-length tolerances end only a Hessian-based (Newton) solve, and the asset
warns about non-Newton methods on contact scenes instead of withdrawing them.
4 has a plan: [contact-efficiency-plan-20260923.md](contact-efficiency-plan-20260923.md).
3 **decided 2026-09-25 (user): not pursued** — L-BFGS is no longer of
interest; the slope-tolerance rule stays Newton-only, variant B stays on the
unpinned branch, and the plan's EF-06 is retired (its retest under the repaired
rule, [EF-04b](ef-04b-feasible-bound-retest.md) §5, found variant B no longer
converges R1).

1. **Stop the false convergence** — a correctness repair, independent of the
   rest: accept the slope tolerance as convergence only for strategies whose
   direction solves with a Hessian (the Newton family; a preconditioned L-BFGS
   right after a refresh) and require the gradient criterion otherwise. Today a
   user who picks L-BFGS or ADAM in the HDA can get exit 0 with a 5–100 %
   wrong solution.
2. **Say what plain L-BFGS, BFGS and ADAM are for.** They are not usable on
   contact scenes of this kind; the HDA could say so or withdraw them for
   contact.
3. **If a cheaper-than-Newton method is wanted, productionise variant B**:
   tests, a refresh before accepting convergence (its stopping test uses its own
   lagged direction — R3's errors reach 4× Newton's with fixed refresh), a stall
   trigger that ignores feasibility-capped steps, and a decision on how it
   should interact with the trim controller. Expected gain: ~1.3–1.6× where
   factorizations dominate (large meshes, stable contact), none on small or
   truncation-dominated scenes.
4. **Look at the trim controller's start first** (a separate item): on R4 it
   costs Newton ~5× (727 iterations against 111–134 at the final trim), and it
   is also what makes R4's answer path-dependent.

## Limits

* Four scenes, one machine, one repetition per configuration (plus the
  Newton repeats named above). R1/R3/R4 use a non-reproducible linear solver;
  iteration counts carry roughly ±5 % noise there.
* The R4 pinned trim was chosen with hindsight (Newton's final value); it
  isolates the minimizer, it is not a proposed setting.
* The experimental code is minimal: no tests, no clang-format pass, the
  diagonal fallback of a failed factorization never exercised, and
  factorizations are counted from accepted rows (a refresh in a sub-solve with
  no accepted iteration is missed).
* The condition-number bound is a lower bound along the path, not a spectrum.

## Evidence

Parent workspace `outputs/qn-contact/20260923T021710Z/`: `PROGRESS.md`
(findings F1–F28 in the order found, including the corrections made along the
way), every run under `runs/LABEL/` (input, log, `row.json` with the binary's
SHA-256, diagnostic streams, frames), binaries
`bin/PolyFEM_bin-{baseline,precond1..4}` with their build logs, the sequence
scripts, and the tools now in [tools/qn_contact](../tools/qn_contact/README.md).
PolySolve branch `qn-contact-experiment` (`b5c8861`, `c38ec15`, `65df8cf`,
`92ee746`) on top of the pinned `c874cd59`; PolyFEM and IPC at the pinned
`453ef2257` / `482b9eab`.
