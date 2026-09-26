# Contact-solve efficiency: barrier trim, stall trigger and AL weight — plan

Date: 2026-09-23. **Status: EF-01 done 2026-09-24**
([record](ef-01-trim-survey.md)); **EF-04 done 2026-09-24**
([record](ef-04-stall-trigger.md); retested 2026-09-25,
[EF-04b](ef-04b-feasible-bound-retest.md); default `absolute` decided by the
user 2026-09-25); **EF-06 retired 2026-09-25** (user: L-BFGS no longer
pursued); **EF-02/03 implemented and measured together 2026-09-26, opt-in; strict adoption gates unmet**
([record](ef-02-03-trim-controller.md)); EF-05 not pursued after the negative
EF-01 sensitivity result. Items are EF-01 …
EF-06; EF-01 is measurement only and is the prerequisite of the rest.

EF-01 outcome in brief: H-A and H-B hold, H-E holds on R4, H-C and H-D do not.
The endpoint gradient balance is an identity (it returns the trim in force), so
EF-02's estimate must come from the first stall's off-equilibrium κ_gb or from a
force-weighted gap target, which merges EF-02 with EF-03; EF-05 has no case on
these scenes; R3 fails at step 39 at every trim (published binary too) and needs
its own item. See the record's "Consequences for the plan".

EF-04 outcome in brief: H-E holds as a statement about the trigger (16 of 17
production R4 alpha restarts fired on steps accepted at the feasible bound),
but counting only steps that backtracked below the bound
(`restart/alpha_basis: feasible_bound`, opt-in) makes no scene cheaper — the
soft iteration budget takes over the retunes and R4's cost stays the trim
walk (852–879 against 726–828 iterations; R1, BBT, the smokes and the pinned
optima unchanged). Default kept absolute; the option is there for EF-02/03 and
EF-06 to revisit.

EF-04b (retest, 2026-09-25): no benefit in any regime left open. R4 step 1's
cost follows when the trim walk starts (iterations ≈ 299 + 1.26 × first
departure from 1, r = 0.91, 15 runs), which neither basis moves, at soft
budgets 100/200/400/off (absolute 780 ± 60, feasible bound 777 ± 70); with the
soft budget off, feasible bound leaves one 745-iteration attempt that the
exported AL cap (500) fails. Pinned multi-step R4 and BBT unchanged, held-out
BB −3–5 %, IT +13–17 % on a non-reproducible trajectory. EF-02/03 were not
landed at that retest; their subsequent experiment is recorded below.
EF-06's variant B no longer converges R1 under either basis: its
qn-contact convergence was the slope tolerance that `448f1b8`/`9f8f25881`
withdrew for non-Hessian directions — EF-06 must settle that criterion first.

## Why

The [quasi-Newton investigation](qn-contact-investigation-20260922.md) found
that on the user's large self-contact scene (R4, 237,780 nodes) Newton needs
727 iterations and 1,581 s for step 1 — and 111–134 iterations and 240–295 s
when the barrier trim is simply held at the value the controller eventually
reaches (2⁻¹¹). Most of Newton's cost on that scene is the controller finding
the barrier strength, not the minimization. The user expects that a better
choice of the AL weight and/or barrier stiffness, with more thought about the
trim, can improve efficiency. This plan turns that into bounded experiments.

## What is already established

All from the evidence of `outputs/qn-contact/20260923T021710Z/` (R4 = the uniax
scene, R1 = plate + ball head, R3 = inflation, smoke = `quasistatic-semi`).

| # | Observation |
| --- | --- |
| E1 | R4 step 1: 702 of 727 Newton iterations are the **single** augmented-Lagrangian pass (moving Dirichlet boundary); it converges ("feasible snap after 1 pass") once the trim is right. The trim stays at 1 for the first ~400 iterations (4 stall restarts), then halves 11 times to 2⁻¹¹. With the trim pinned at 2⁻¹¹: 111–134 iterations (three runs), 5.4–6.5× fewer. |
| E2 | The controller's statistic, √(mean d²)/d̂ over the active pairs, reads **0.82–0.96 both at trim 1 and at trim 2⁻¹¹** on R4 (1,000–2,500 active pairs). It sits at the band's upper edge (√0.9 = 0.949) whatever the trim, so the downward step fires only when noise crosses the edge. |
| E3 | Final trims span five decades across scenes — smoke 2 → 16 (rising step by step), R3 1–2, R1 2e-3 → 1.2e-4, R4 ≈ 5e-4 — and every run starts at 1. The gradient-balance calibration is **upward-only** by design (`BarrierContactForm.cpp`, "Upward-only: … letting it LOWER the trim starves exactly the contacts that are about to fail"); the only way down is ×½ per ≥ 30 iterations (`controller_interval`) while the mean gap is above √`trim_upper`·d̂. |
| E4 | The stall trigger (α < 0.01 for 5 iterations) is absolute: it fires while every step sits exactly at the feasible bound (R1 Newton's restarts, `iters.py` rows 0–9), and every restart refreshes the per-contact coefficients — an objective change. R4 Newton step 1: 8 restarts (4 soft budget, 4 alpha). |
| E5 | Newton is truncation-bound on real scenes: on R4 the 50·d̂ trial cap is active in 99 % of line searches and CCD then keeps a median 5.7 % of the capped sweep; on R1 70 % and 25 %. |
| E6 | The AL weight (`hessian_scaled`, ×`initial_weight_multiplier` of max\|H\|; 2.6e6 on R4) has not been varied on the real scenes. The design record's bisection on the ball-on-plate case found a multiplier of 1 → 100 changed the time by < 1.5 s ([writeup §5.3](../../SEMI_IMPLICIT_BARRIER_WRITEUP.md)); its effect on conditioning in dense contact is unmeasured. |
| E7 | The trim is path-dependent and so is R4's answer: three production runs ended on three trims (4.9e-4, 9.6e-4, 2.1e-2); identical Newton runs end 3.5–6 % apart (frictionless self-contact, `AccelerateLDLT` not bit-reproducible). Accuracy comparisons must use pinned-trim references and Newton-vs-Newton spreads. |

## Boundaries (from the robustness plan's decisions, unchanged here)

* The production per-contact coefficient law, Hessian-based scaling and the
  **global gap-band controller were retained by the user on 2026-09-11**
  (RB-13–RB-16 closures; RB-17's per-contact band targeting retired). EF-02,
  EF-03 and EF-05 propose changes to that controller and its inputs: each is
  opt-in until the user decides a default.
* CCD and the separate trial-displacement cap stay; the cap value is measured,
  not changed. The retired constraint floor stays retired. No automatic
  retry (RB-08). Teseo is not run.
* Every change keeps the five smokes byte-identical when off, reruns the RB-02
  probe (`tools/rb02`, 270/270) if anything touches the coefficient path, and
  publishes with its record.

## Hypotheses

* **H-A (start):** most of the dense-contact cost is the trim's slow descent
  from an uncalibrated start. An estimate at the step's first refresh recovers
  most of E1's 5×.
* **H-B (statistic):** the mean active gap is dominated by lightly loaded pairs
  near d̂; a load-aware statistic (force-weighted mean gap, or the gap
  quantile of the pairs carrying most of the contact force) moves with the
  trim where the mean does not.
* **H-C (normalization):** the per-contact coefficient is a local Rayleigh
  quotient that ignores how many pairs share a vertex. Dense self-contact puts
  tens of pairs on one vertex, so the barrier stiffness per vertex — and the
  right trim — scale with contact multiplicity (R4 ≈ 5e-4 vs smoke ≈ 2–16).
* **H-D (AL weight):** an AL weight far above the elastic stiffness stiffens the
  prescribed DOFs and worsens conditioning in the AL pass, where R4 spends 97 %
  of its iterations — or, as on the ball-on-plate case, it does not matter.
* **H-E (trigger):** counting feasibility-capped steps as stalls causes
  restarts that refresh the objective without helping.

## EF-01 — Cost surface and predictor survey (measurement only; do first)

**Scenes:** smoke `quasistatic-semi` and `transient-semi`; R1 steps 1–3; R3
steps 1–3; R4 step 1; `ballburst_thin_membrane` (d̂ 1e-5) step 1–2; hold out
`inertia_test` (d̂ 1e-3) and `ball_burst` for EF-02/03 acceptance only.

1. **Trim sweep.** Pin the trim (`trim_min = trim_max`, which all trim
   updates are clamped to — used for R4 on 2026-09-23) at 2^k for k = +4, +2,
   0, −2, …, −14; Newton, the scene's own settings otherwise. Record accepted
   iterations (all sub-solves), factorizations, wall, restarts by trigger, AL
   passes, the band statistic, the gap distribution (min, 10/50/90 %), active
   pairs, the trial-cap and CCD binding fractions, `physical_balance_pass`,
   and the solution against a tight pinned reference. Cap each run (15 min on
   R4). Result: cost-versus-trim curves, how flat the optimum is, and the
   accuracy cost of each trim (RB-09's `e_R = c·ḡ`).
2. **AL-weight sweep.** `initial_weight_multiplier` ∈ {0.1, 1, 10, 100} and
   the `scaling`/`eta` ratchet at defaults, at the best and the production trim,
   on R4 and on a Dirichlet-driven smoke. Result: whether H-D holds at all.
3. **Predictors.** Add opt-in debug logging (no behaviour change) at every
   refresh of: the two-sided gradient-balance value κ_gb and its cosine (today
   it is computed but only applied upward), mean/max contact multiplicity per
   vertex, the ratio of barrier to elastic Hessian diagonals on active DOFs,
   and the band statistic's force-weighted variant (H-B). Compare each with
   the EF-01.1 optimum across scenes.
4. **Controls:** Newton-vs-Newton repeats at the production setting and at
   the optimum on R4 and R1 (non-reproducible linear solver: E7); one
   production run per scene with the current binary.

**Acceptance:** the curves and predictor table for every scene, run records
with binary hashes, and a statement of which hypotheses survived. No default,
controller or law change.

## EF-04 — Stall trigger relative to the feasible bound (small, independent)

**Done 2026-09-24** — [record](ef-04-stall-trigger.md). Opt-in
`solver/contact/semi_implicit/restart/alpha_basis` / `feasible_ratio_threshold`;
no measured benefit; default unchanged (the user's decision). Retested
2026-09-25 at other soft budgets, pinned multi-step, held-out scenes and with
EF-06's preconditioned L-BFGS ([EF-04b](ef-04b-feasible-bound-retest.md)): no
benefit; **the user kept the `absolute` default (2026-09-25).**

Count a step toward the alpha patience only when the line search backtracked
below the feasible bound (accepted α / feasible α < threshold), not when α is
small because CCD or the trial cap bounded it. Measure on R1, R4 and the
smokes: restarts by cause, iterations, wall, byte-identity of scenes that never
cap. Opt-in first; the default is the user's. (Also removes the obstacle that
failed fixed-interval preconditioned L-BFGS on R1.)

## EF-02 — Step-start trim estimate (merged with EF-03; measured opt-in)

Implement the best EF-01 predictor at the first refresh of a step (and at
first contact) as a two-sided, bounded initialization — the band and collapse
feedback afterwards unchanged, the upward emergency bump unchanged. Guard the
case E3's comment names (a lowered trim starving contacts about to fail) with
the collapse statistic that already exists. **Acceptance:** on the EF-01
matrix and the held-out scenes, no new failures, iterations ≤ production on
every scene, accuracy within the Newton-vs-Newton envelope and RB-09's gap
error, `physical_balance_pass` unchanged, RB-02 probe 270/270.

**Outcome, 2026-09-26:** the guarded off-equilibrium seed reaches 2^-12 at
the first accepted R4 observation, replacing the production walk of 314–409
observations. R4 step 1 falls from 657–703 to 99–107 iterations; five steps
from 1050–1110 to 392–485. All matrix runs finish and every scene's total
iteration count is no worse. However, multi-step R4 exceeds the fresh
production repeat spread, and balance flags change on BBT and IT. Thus the
strict acceptance above is **not met**. The implementation remains an
experiment: `initial_trim_estimate: false` and `band_statistic: rms` stay the
defaults. See [the combined record](ef-02-03-trim-controller.md) for accuracy,
reference coverage, negative candidates, regression checks and recommendation.

## EF-03 — Force-weighted global band (merged with EF-02; measured opt-in)

If H-B survives: an opt-in load-aware statistic and a proportional downward
step (the upward branch already has `collapse_bump_factor`; the downward one is
a fixed ×½). Re-read RB-16's evidence first: it tested update factor and
hysteresis on springs and FEM fixtures and retained the global band. Same
acceptance as EF-02, plus band occupancy and oscillation counts.

**Outcome, 2026-09-26:** opt-in `band_statistic: force_weighted` uses a
[.35,.50] target, .025 dead zone, bounded factor-four proportional updates,
and the retained collapse feedback. A scalar-response veto prevents a band
softening from immediately undoing collapse protection, removing the v2
smoke cost regression. R4 occupancy improves markedly; R1 and BB remain
mostly outside the target, and IT reversals rise from 22 to 60. RB-02 is
270/270, the full suite has only the two known goldens, all 13 HDA scripts
pass, and all 25 option-off smoke VTUs are byte-identical. The accuracy and
unchanged-balance gates still prevent an adoption recommendation. No HDA
control, coefficient-law, stall-trigger or AL-budget default changed.

## EF-05 — AL weight (only if EF-01.2 shows sensitivity)

Candidate: tie the initial AL weight to the barrier-inclusive stiffness at the
prescribed DOFs rather than max\|H\|, or lower the multiplier. RB-07's budget
and the AL snap gates stay as they are. Acceptance as EF-02, plus AL passes and
snap residuals.

## EF-06 — Re-evaluate the preconditioned L-BFGS (optional, last)

**Retired 2026-09-25 (user decision):** L-BFGS is no longer pursued; the
slope-tolerance rule stays Newton-only and variant B is not productionized.

The experimental variant B (PolySolve branch `qn-contact-experiment`) was held
back mainly by the stall trigger and the trim walk. (EF-04b, 2026-09-25: merged
onto the current pin `448f1b8` it no longer converges R1 step 1 under either
trigger basis — its earlier R1 convergence was the slope tolerance, which the
repair now accepts only for Hessian directions. Decide first whether a
Hessian-preconditioned direction qualifies.) Repeat its R1/R4 comparison
after EF-02–04 before deciding whether to productionize it.

## Order and decisions

EF-01 → EF-04 → EF-02 → EF-03 → EF-05 (EF-06 retired 2026-09-25). EF-01 and EF-04 need no model
decision. EF-02, EF-03 and EF-05 change the retained controller or its inputs
and stay opt-in; making any of them a default is the user's decision, with the
EF-01 matrix as its evidence.
