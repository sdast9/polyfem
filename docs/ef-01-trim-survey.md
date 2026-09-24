# EF-01 — Barrier-trim cost surface, AL weight and predictor survey

Date: 2026-09-23/24. Item EF-01 of the
[contact-efficiency plan](contact-efficiency-plan-20260923.md) (measurement
only). **Status: done.** No default, controller or coefficient-law change; the
only code change is an opt-in, observational diagnostic stream.

Evidence: `outputs/ef-01/20260923T145726Z/` (not in the repository): 110 runs
under `runs/`, excluded runs with reasons under `runs-excluded/`, the sequence
scripts and logs, `tables.md` / `metrics.json` (reduced), and the two binaries
in `bin/` (`PolyFEM_bin-9f8f25881`, sha256 `39920e02…`, the published head;
`PolyFEM_bin-ef01`, sha256 `1df8e4cb…`, this change).

## What changed in the code

`output/trim_predictors` (bool, default false) writes
`trim-predictors.jsonl` (schema `polyfem.trim-predictors` v1). A record is
written after every semi-implicit refresh (`refresh`, `refresh_endpoint`),
after every stall retune (`stall_retune`) and after every post-step controller
update (`iteration`):

* the trim in force, the active pairs (distance ≤ d̂, the controller's filter),
  the gap distribution (min/p10/p50/p90/max/mean and the controller's
  statistic rms = √(mean d²)) in units of d̂;
* a force-weighted gap: weights = the norm of each active collision's local
  barrier gradient (proportional to its contact force): weighted mean and rms,
  the gap below which 50 % / 90 % of the force is carried, and the force share
  of the closest tenth of the pairs;
* contact multiplicity per surface vertex (active collisions incident to it);
* at refreshes and retunes only: the **two-sided** gradient-balance trim κ_gb
  with its cosine (the value `calibrate_trim` computes before its gate and its
  upward-only application), the barrier/elastic Hessian-diagonal ratio on
  contact DOFs with the trims that would make its median and its sum equal to
  one, and the unclamped first-contact conditioning-cap trim.

Implementation: `BarrierContactForm::trim_predictors` (const) and
`emit_trim_predictors`, wired by `NonlinearElasticVarForm` next to the
coefficient observer; the run manifest lists the flag, the schema and the file.
A failed record is logged and never reaches the solve.

**No behaviour change:** with the flag off, all five public smokes are
byte-identical to the published binary (every output file,
`byte-identity/`); with it on, the transient smoke's VTUs are byte-identical to
the flag off. The R3 step-39 failure below reproduces identically on the
published binary (901 iterations, 2,188 s vs 2,189 s).

## Method

`tools/ef01/` (new): `ef01_run.py` (one run: Newton, the scene's settings
otherwise, `--trim T` pins `trim_min = trim_max = T`, iteration and physical
diagnostics on, predictors on, coefficient events discarded to `/dev/null` —
17 GB per production R4 run otherwise), `ef01_reduce.py` (per-run metrics),
`ef01_tables.py` (per-scene tables + accuracy against `ref-SCENE`),
`ef01_predictors.py` (the full predictor records of a run), `ef01_best.py`
(cheapest completed pin). Restarts are counted by trigger from the log; the
trial-cap / CCD binding fractions use the qn-contact F17 method (each line
search's `trial_clamp` against its `collision_free_step_size`). References:
the cheapest pin with the slope tolerance at 1e-15. Runs strictly sequential;
R4 on all 18 threads, everything else single-threaded.

Deviations from the plan, all forced by the scenes:

* **R3 steps 1–3 have no contact** (0 active pairs; the 11 runs were
  identical). Contact starts at step 6 with 2–7 pairs and jumps to 150–210
  pairs at step 39; R3 runs cover steps 1–40.
* **BBT** (`ballburst_thin_membrane`) carries `space/pressure_discr_order`,
  which the current spec refuses; the driver removes that key (listed in
  `row.json`). Steps 1–2 have 1–3 pairs, so BBT covers steps 1–20 (1 → 29
  pairs, 32 s in production).
* The sweeps were extended below 2⁻¹⁴ where the cost was still falling (R1 to
  2⁻²², BBT to 2⁻²², R4 to 2⁻²⁰).
* 5-step R4 controls were added when the pinned 5-step AL runs showed the
  step-1 optimum degrading in later steps.
* Three runs segfaulted **during mesh loading** (before any solve) in
  `Selection::build`: a const `json::operator[]` read of the missing `"id"` key
  of a `{"file": …}` selection — upstream undefined behaviour, unrelated to
  the trim (reported separately). They were rerun; from then on the driver
  rewrites a lone `{"file": X}` selection to the string `X` (the same
  `FileSelection(resolve_path(X))`), recorded in `row.json`.
* The machine slept 12:26–13:49 on battery; run logs show no gaps and R4 kept
  2.2–2.5 s/iteration; the one run spanning the 13:47 sleep (`pin-R4-k-2`, a
  timeout) was rerun and agrees.

## Results

### 1. Cost versus pinned trim

Accepted Newton iterations over all sub-solves (= factorizations). "fail" = a
named failure (exit 1, the final solve stalls); "cap" = the run's time cap
(15 min on R4, 30 min on R3).

| trim | smoke-qs (4 steps) | R1 (3 steps) | BBT (20 steps) | R4 step 1 | R3 (40 steps) |
|---|---|---|---|---|---|
| 2⁴ | 27 | fail | 269 | cap | cap (step 39) |
| 2² | 26 | fail | fail | cap | cap (step 39) |
| 1 | 30 | fail | 205 | cap | cap (step 39) |
| 2⁻² | 26 | 1,044 | 195 | cap | — |
| 2⁻⁴ | 29 | 728 | 167 | cap | — |
| 2⁻⁶ | 31 | 629 | 157 | cap | cap (step 39) |
| 2⁻⁸ | 37 | 464 | 141 | 224 | — |
| 2⁻¹⁰ | 40 | 313 | 127 | 184 | — |
| 2⁻¹² | 43 | 242 | 121 | 109 | — |
| 2⁻¹⁴ | 48 | 204 | 115 | 87 | — |
| 2⁻¹⁶ | | 159 | 109 | 88 | |
| 2⁻¹⁸ | | **149** | **101** | 95 | |
| 2⁻²⁰ | | 189 | 117 | **83** / 90 (repeat) | |
| 2⁻²² | | 266 | 120 | | |
| production | 31 | 377 / 388 | 186 | **800 / 726** | **fail at step 39** |

R3 pins at 2¹⁰, 2⁸, 2⁶ and 2⁻³ also stall in step 39 at the cap (≈ 420
iterations into the step). Wall time follows the iterations (R4 2.2–2.5 s per
iteration; R4 production 1,854 / 1,589 s against 211–237 s at 2⁻²⁰).

* **Every scene with dense contact has a wide flat optimum 3–4 decades below
  where production starts** (trim 1): R1 flat over 2⁻¹⁶…2⁻²⁰ (149–189), BBT
  over 2⁻¹⁴…2⁻²² (101–120), R4 step 1 over 2⁻¹²…2⁻²⁰ (83–109). The smokes are
  flat over eight decades (25–48 iterations).
* **The controller's walk, not the trim value, is the cost (H-A holds).**
  Pinned at production's own final trim, R4 step 1 takes 184 iterations (2⁻¹⁰)
  against production's 800; R1 at its production final trim (≈2⁻¹³) ≈ 220
  against 377. Production stays at trim 1 for 354–426 of its R4 post-step iterations
  (both runs; 11–12 restarts in total) before the downward steps begin.
* **The single-step optimum is not the multi-step optimum.** Over R4 steps
  1–5, the pin at 2⁻¹² costs 377 iterations (101/66/51/80/79, 2 restarts, 883
  s) and the pin at 2⁻²⁰ 749 (94/143/132/151/229, 6 restarts, 1,653 s); the
  soft end of the step-1 plateau degrades as the compression accumulates.
  Production over 5 steps: 1,041 iterations (732/59/77/84/89, 10 restarts,
  2,299 s); from step 2 on its controller sits at 2.45e-4 = 2⁻¹², exactly the
  best multi-step pin, and its later steps cost what that pin's do. The
  multi-step optimum on R4 is the top of the step-1 plateau, where production
  ends; **production's entire excess is the step-1 walk (732 against 101).**
* **R3 fails at step 39 at every trim tried (2¹⁰ … 2⁻⁶), in production and on
  the published binary.** Contact jumps from 7 to 150–210 pairs in that step;
  the solve runs ≈ 400–650 iterations with 20–49 restarts. This is not a trim
  choice problem and is outside EF-02/03's reach; it needs its own
  investigation.
* On R1 and BBT the stiffest pins (1–16) fail by name; the failure is not
  monotone on BBT (2² fails, 2⁴ and 1 pass) — the trim is path-dependent (E7).

### 2. Accuracy

Relative L2 of `solution` against the tight reference at the cheapest pin:

* R1: every completed pin ≤ 8e-4 per step (≤ 1.4e-3 at 2⁻²⁰…2⁻²²), production 4–8e-4; two
  identical runs agree to 1e-6–1.6e-4 (production pair; the repeat at the
  reference trim vs the reference 0.5–1.6e-4), so the trim effect is real on
  R1 but below 0.1 %.
* BBT: 1–4e-3 early in the sequence falling to ≈1e-3 near the optimum;
  production 2.5e-3–1.3e-2.
* smoke-qs (d̂ = 1e-3): 1e-3–1e-2 across the whole range.
* R4 (frictionless self-contact, the flat valley of E7): two identical pinned
  runs differ by 3.6–4.8 %, the two production runs by 8.7 % (different final
  trims). Against the tight reference (300 iterations): production 11 %,
  2⁻⁸…2⁻¹² 9–11 %, the plateau 2⁻¹⁴…2⁻²⁰ 3.5–7 %; trim-to-trim differences
  (2⁻¹² vs 2⁻¹⁴: 4.6 %) are within the repeat spread. R4's accuracy is set
  by the stopping tolerance in the flat valley, not by the trim; the pinned
  optimum is no worse than production.

Lowering the trim toward the optimum does not cost accuracy on any scene: the
gaps shrink (RB-09's e_R = c·ḡ falls), R1/BBT/smoke stay within ≈1e-3 of the
reference and R4 within its own repeat spread. `physical_balance_pass` fails on every R1 step at the scene's
tolerances, pinned or not, and passes on every step of the tight reference at
the same trim — it tracks the stopping tolerance, not the trim. R4 fails it
even with the tight tolerance (300 iterations); BBT and the smokes pass.

### 3. Truncation and restarts at the optimum

* R4 remains truncation-bound at every trim: CCD bounds 94–99 % of the line
  searches; the trial cap binds in 39–87 % (52 % at 2⁻¹⁴, 84 % in production).
* H-E: in R4 production 317 of 354 small-α iterations (α < 0.01) sat at the
  feasible bound (a/feas ≥ 0.999) — the alpha trigger fires on steps that CCD
  or the cap bounded; on R1 production 23 of 174 (backtracking dominates).
  EF-04 targets a real effect on R4, not on R1.

### 4. AL weight (H-D does not hold)

* **R1 and the smokes never solve an AL problem**: the pass-0 snap gate of
  `ALSolver` finds the prescribed values feasible and the stage is skipped
  (every iteration is tagged `reduced`); the hessian-scaled weight is computed
  and logged but unused. Their multiplier sweep is a no-op by construction.
* **R4 (the uniaxial scene) spends every step in one AL pass** (step 1: 76–90
  of 76–90 iterations in the pass). The weight is applied (logged 2.58e4 …
  2.58e7 for multipliers 0.1 … 100, max|H| = 2.58e5).
* Step 1 at 2⁻²⁰: multipliers 0.1 / 1 / 10 / 100 → 76 / 77 / 83–90 / 87
  iterations, 211 / 210 / 228–237 / 276 s.
* Steps 1–5 at 2⁻²⁰: 772 / 751 / 749 / 759 iterations, 1,702 / 1,680 / 1,653 /
  1,665 s; per-step counts wander ±20 % with no ordering.
* Under the controller every multiplier hit the 15-minute cap (the trim walk
  dominates).

Across a factor 1,000 the AL weight changes nothing measurable: the Newton
steps are cut by collision truncation, not by the conditioning the weight
affects. EF-05 has nothing to act on for these scenes. (None of them needs
several AL passes; a scene whose snap is rejected repeatedly would be the one
to test if that case matters.)

### 5. Predictors (EF-01.3)

| predictor | finding |
|---|---|
| κ_gb at an endpoint | **Identically the trim in force** (R1, BBT, R4: e.g. 0.00196 = 0.00196, 0.5 = 0.5). At a stationary point the energy gradient balances the barrier force on every contact DOF and the AL/Dirichlet reactions sit on DOFs the barrier does not touch, so the least-squares balance returns the current trim whatever its cosine. It carries no information at a published endpoint. |
| κ_gb at a stall retune | Informative off equilibrium on R4: first retune 0.026 (cos 0.81) in one production run, 3.1e-4 (cos 0.94) in the repeat, with the trim at 1 — 1.5–3.5 decades toward the good region, discarded by the upward-only rule. Noise on R1 (1e-11…4e-3, cos < 0.2, gate fails). |
| Hessian-diagonal ratio (trim for unit median / sum) | Not a predictor: R4 10–36 (optimum ≈ 1e-6…2e-4), BBT 1–281, R1 1e-3–5 against optima near 4e-6. |
| Conditioning-cap trim | 1.7e3 – 1.5e6 everywhere; no relation to the optimum. |
| Contact multiplicity (H-C) | 1–2.4 (incidence-weighted) on R1, BBT and R4 at their optima, ≈5 on R3, 26 on the smoke (obstacle vertices shared by all pairs). The optima of R1/BBT/R4 coincide (≈1e-6–6e-5) at multiplicity 1–2; no scaling with multiplicity is visible. **H-C not supported.** |
| rms gap (the controller's band statistic, H-B) | Insensitive where it matters: R4 0.91–0.99 over 2⁻⁴…2⁻¹⁴, R1 0.91–1.0 over 1…2⁻¹⁴. It enters the band only near the optimum (R1 0.67–0.82, BBT 0.71–0.81, R4 0.72–0.91 at the plateau); production parks at the upper edge (0.94–0.99), where the downward step only fires on noise. On the smokes it tracks the trim (0.81 → 0.08 over 16 → 0.25). |
| force-weighted mean gap (H-B) | **Moves with the trim where rms does not** (R4 0.83 → 0.50 over 2⁻⁴…2⁻¹⁴; R1 0.99 → 0.48 over 1…2⁻¹⁸) and **sits in a narrow band at every scene's cheapest pin**: smoke 0.44, R1 0.48, BBT 0.33, R4 0.35–0.50 (d̂ units). **H-B supported.** |

## Hypotheses

* **H-A (start) — holds.** The cost is the descent from trim 1: R4 step 1
  7–9× (726–800 against 83–109 iterations; over 5 steps 2.8×, 1,041 against
  377, all of it in step 1), R1 ≈2.5×, BBT ≈1.8×, at equal accuracy. The step-start estimate must
  aim at the multi-step optimum (the upper part of the single-step plateau),
  not the lowest cheap single-step trim.
* **H-B (statistic) — holds.** The force-weighted mean gap is trim-sensitive
  on dense contact and ≈0.35–0.5 d̂ at every scene's cheapest pin; the rms
  statistic is not.
* **H-C (multiplicity) — not supported** on these scenes.
* **H-D (AL weight) — does not hold.** No measurable effect over ×1,000 on the
  one scene that runs the AL stage; the others never enter it.
* **H-E (trigger) — holds on R4** (90 % of small-α iterations at the feasible
  bound in production), not on R1.

## Consequences for the plan

* **EF-04** (stall trigger relative to the feasible bound) is supported by R4
  and stays next.
* **EF-02** (step-start estimate): the endpoint gradient balance is an
  identity and cannot seed it; the Hessian ratio, the conditioning cap and the
  multiplicity do not predict the optimum. Usable signals: the off-equilibrium
  κ_gb at the first stall (two-sided, cosine-gated) and the force-weighted gap
  measured against a target band. EF-02 and EF-03 therefore converge on one
  controller change: a force-weighted band statistic with a proportional
  two-sided step, possibly seeded by the first stall's κ_gb. Its acceptance
  matrix must include multi-step runs (R4 1–5, BBT 1–20).
* **EF-05** (AL weight): no case on these scenes; drop unless a scene needing
  several AL passes appears.
* **R3 step 39** is a separate failure (every trim, published binary) and
  needs its own item.
* All of this remains opt-in until the user decides defaults (retained
  controller, 2026-09-11).

## Reproduce

```
python3 tools/ef01/ef01_run.py LABEL --scene R4 --steps 1 --threads 0 --timeout 900 --trim 9.5367431640625e-07 --out EVIDENCE --binary BIN
python3 tools/ef01/ef01_tables.py EVIDENCE
python3 tools/ef01/ef01_predictors.py EVIDENCE/runs/prod-R4
```

The scenes R1/R3/R4/BBT are the user's Houdini exports under `test_cases/`
(not in this repository).
