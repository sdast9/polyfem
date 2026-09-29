# Default trim controller: should the EF-02 estimate or the EF-03 force-weighted band become the default? (2026-09-29)

Status: **assessment only. No default, solver, Houdini asset or scene was
changed.** Changing a default is the user's decision. This record gives the
evidence, proposes an accuracy standard for the user to agree to, and makes a
recommendation.

Question (user, 2026-09-29): should `solver/contact/semi_implicit/band_statistic:
"force_weighted"` (EF-03, with its enforced `collapse_guard_basis: pair`)
and/or `initial_trim_estimate: true` (EF-02) become defaults? What failures
could each cause, alone and together? The standing EF-07 condition applies:
*before the force-weighted mode can be considered for adoption it first needs
an agreed accuracy standard for trajectory-sensitive scenes, and repeat
evidence on dynamic scenes not used for tuning.*

## Answer in brief

**Recommendation: keep production (`rms`, no estimate) as the default for now.
Do not make the force-weighted band a default. The estimate alone is the only
candidate worth taking further, and it is not ready to adopt yet (see
*Recommendation*).**

* **Force-weighted band (F), alone or with the estimate (B): not a default.**
  * On `pup_push`, a held-out dynamic scene no EF item had used, it **fails
    step 1 in 2 of 4 runs (F) and 3 of 4 (B)**, against 1 of 4 for
    production. The runs are three deterministic realizations plus one
    multithreaded run per arm.
  * This is a new failure mode: a **softening cascade that exhausts the stall
    restarts**. Stalls come every ~9 iterations while the contact front is
    CCD-limited. The band softens the trim at almost every stall retune
    (64 → 0.085 in one run, ×750) while its force-weighted gap barely moves
    (0.68 → 0.58 d̂). All 20 restarts are gone by iteration 219–674, with
    the minimum gap at the collapse threshold.
  * It is not EF-07's pinched-pair loop, and the pair guard does not prevent
    it. That guard only vetoes a softening predicted to cross the collapse
    threshold.
  * On `fibers` (held out) the band **costs 12 % more** than production:
    809 vs 722 iterations in all three realizations, with 319 of its 340
    softenings vetoed. The solution stays within 1.1e-5 of production's.
  * On the looping ball-burst steps 31–32 it is now loop-free (F 264/266,
    B 174/340 iterations, 4 restarts each; production 251/217). This is the
    measured case the pair guard was built for.
  * On IT it changes the dynamic trajectory far beyond production's
    realization spread (ρ = 8.3). The IT trim ladder below shows that the
    change moves the result toward the low-trim limit, not away from it.
* **Initial estimate alone (E): inert on most scenes and a large win where it
  acts.**
  * It is never accepted on the smokes, R1, BBT, fibers, the first ~150
    steps of IT, or most of `pup_push`. There the runs are byte-identical to
    production (cosine < 0.8 every time).
  * On R4 it cuts step 1 from 583/510 to 89/133 iterations, and five steps
    from 832 to 426 (373 with scope `run`). The whole saving is in step 1:
    steps 2–5 cost 297 (E) and 282 (Er) against production's 255. Its
    one-step R4 solutions sit inside production's repeat spread (ρ = 0.75).
  * It completed all 4 `pup_push` runs and the looping ball-burst steps
    (234/196 iterations, 3 restarts; production 251/217, 4 restarts).
  * Open risks:
    * On IT (from step ~154) the step-scope seed re-fires almost every step,
      alternating up ×4–7 and down ×0.04–0.6. That is a step-to-step
      oscillation, although IT's ensemble statistics stay equal to
      production's (ρ = 1.00).
    * Upward seeds reach ×47. The 4096× bound is hit exactly on R4.
    * Scope `run` loses its "already used" flag across a restart: confirmed,
      the resumed run seeds again at step 169.
* **Both together (B)** combine the band's failure mode with the estimate's
  benefit. On R4 step 1, B (151/108) is not cheaper than E alone (89/133).
  Its R4 solutions sit outside production's spread (ρ = 1.51).
* **Accuracy standard (proposed, not declared met).** Agreement with
  production's realization ensemble is necessary but not sufficient.
  * On IT, production itself is biased. Its kinetic energy after impact is
    73–93 % above the pinned low-trim runs at step 124. The trim ladder
    (64 → 1 → 2⁻³ → 2⁻⁶) converges toward the force-weighted result.
  * The standard therefore has two parts: (A) agreement with a pinned-trim
    reference ladder, and (B) ensemble statistics against production's own
    realization spread, plus robustness gates. See *Proposed accuracy
    standard*.
  * Candidate held-out dynamic scenes: `pup_push`, `fibers`, IT and the
    friction variant ITF. `mesh_tissue` never makes contact in its 40 steps
    and cannot discriminate.

## Method

Evidence: parent workspace `default-controller-work/`. It is local and not in
`outputs/`, which is being archived to the Pitt share. Tools are copied to
`tools/dca/`:

* `dca_run.py` wraps `tools/ef02/run.py` and adds the arms, realizations and
  the extra scenes.
* `dca_reduce.py` and `dca_compare.py` reduce and compare the runs.
* `queue2.sh` runs the job files.
* `bb_run.sh` and `bb_summarize.py` run and summarize the ball-burst resumes.

**Binary.** `bin/PolyFEM_bin-main`, SHA-256 `f594ebff14c7f4b4330928c6341d5c73eb14fc6a409f986fa24fe12dd2ada60d`
(`bin/hashes.txt`). It is an immutable copy of
`fixed-contacts-work/bin/PolyFEM_bin-default4`, built from `39f26a429`.

* Its `src/` and `cmake/` are identical to `eb8286b7c` and to today's
  `origin/main` `d53b9e444`, except the test-data pin.
* IPC `f8dafef39e8`, PolySolve `43ca2e66`; RelWithDebInfo, Python on.
* It contains the pair default (`1cb1efc25`), the `exclude_statistics`
  default (`eb8286b7c`), the Accelerate thread cap (`ba3ea76b6`), and AL
  multipliers plus controller memory in state files (`48a5e16bf`,
  `c133948cf`).
* `main` moved on during the measurement: `ff4068968`/`9051aacb1` add an
  opt-in `gradient_balance_dofs` whose default (`all`) is unchanged, so these
  results describe current default behavior.

**Arms.** Every run sets the controller explicitly, whatever the scene file
says. The R4 export selects fw + estimate
([r4-production-speedup-20260928.md](r4-production-speedup-20260928.md)).

| arm | `band_statistic` | `initial_trim_estimate` | other |
|---|---|---|---|
| P, production | rms | false | |
| E, estimate only | rms | true | scope `step` (default) |
| Er | rms | true | `initial_trim_estimate_scope: run` |
| F, force-weighted only | force_weighted | false | `collapse_guard_basis: pair` |
| B, both | force_weighted | true | pair |

`clamped_contacts` stays at the binary default (`exclude_statistics`). The
coefficient law, CCD, trial cap, stall trigger, soft budget and
`max_restarts` (20) are as each scene exports them. There is no automatic
retry (RB-08).

**Realizations.** Single-threaded scenes run three deterministic realizations
that differ only in the linear solver: `AccelerateLDLT` capped at one thread
(`acc1`), `CholmodSupernodalLLT` and `SimplicialLDLT`. This is the ensemble
the [IT-reproducibility record](it-reproducibility-20260928.md) recommends.
R4 (6 threads), `pup_push` `r1` (4 threads) and the ball-burst steps
(8 threads) are multithreaded repeats and are not bit-reproducible.

**Scenes.**

* Five public smokes, 4 steps each.
* R1, 3 steps. BBT, 20 steps.
* IT, 200 steps, BDF3. It was held out of EF-02/03 tuning but used in its
  acceptance.
* **ITF**: IT with `friction_coefficient 0.3`, a synthetic friction variant
  using the defaults `friction_lag realized_force` and 2 iterations.
* R4: 1 step × 2 repeats; 5 steps × 1 run for E, Er, B and P.
* Ball-burst steps 31–32, resumed from EF-07 R0's `state_30`, which carries
  the controller memory. This is the regime where the EF-02/03 mode looped.
* Three of the user's dynamic scenes that **no earlier EF item used**:
  * **PP** `pup_push`: two bodies, per-element ActiveFiber material, contact
    from step 1, dt 0.3. Step 1 only.
  * **FB** `fibers`: an ActiveFiber body with self-contact from step 51 of
    200. Steps 1–50 were run once in production (4 threads, state files);
    every arm resumes from that `state_50` for steps 51–200.
  * **MT** `mesh_tissue`: biaxial stretch, 40 steps. There are 0 active
    pairs in every production step, so every option is inert. MT is not
    used below.

**Accuracy measures.** `d(a,b) = |ua−ub| / mean(|ua|,|ub|)` on the exported
`solution`. For arm X at step t:

* `E_P(t)` is the mean pairwise distance inside production's realizations.
* `D_X(t)` is the mean distance between X and P runs of different
  realizations.
* `ρ = Σ D_X / Σ E_P` over steps with `E_P > 1e-6`.
  * ρ = 1 for an arm statistically identical to production.
  * A systematic offset b raises D to about √(E_P² + b²).

Observables per step come from `physical-diagnostics.jsonl`: kinetic and
elastic energy, contact-force norm, endpoint gaps and trim. Each is compared
as the ensemble mean against production's mean, normalized by production's
range plus 1 % of the series' scale. The summed AL reaction is a balance
residual on two-sided Dirichlet scenes (R4), so it is not used as a load
quantity there. Robust scenes are also compared with a pinned-trim tight
reference (trim 2⁻¹⁸, `derivative_along_delta_x_tol 1e-15`, regenerated on
this binary: `runs/ref-R1`, `runs/ref-BBT`). IT gets a pinned-trim ladder
(trims 64, 1, 2⁻³, 2⁻⁶, 2⁻¹⁸).

Wall time is not compared. The host ran other sessions' jobs at load 35–60
throughout, so iterations are the cost measure. The app was quit once
(~13:00); every run in flight then kept going, and none was lost.

## Results

### Cost, failures and controller activity

Iterations are accepted Newton iterations over all attempts. `restarts` are
stall retunes. Per-realization values are listed as acc1 / cholmod /
simplicial, or r1 / r2 for multithreaded runs.

| scene | P | E | F | B |
|---|---|---|---|---|
| smokes (qs, tr, adaptive, alhess, friction) | 31, 26, 37, 31, 62 | identical to P (all 25 VTUs byte-equal) | identical | identical |
| R1, 3 steps | 348 / 280 / 373 | = P (byte-identical) | 196 / 143 / 187 | = F (byte-identical) |
| BBT, 20 steps | 186 / 178 / 186 | = P | 119 / 120 / 119 | = F |
| IT, 200 steps | 6385 / 3807 / 5087 | 6434 / 3807 / 4930 (Er 6245 / 3808 / 5238) | 4286 / 3274 / 3299 | 4313 / 3260 / 3284 |
| ITF, 200 steps | 4689 / 3599 / 4501 | 4666 / 3600 / 4490 | 3800 / 3388 / 3943 | 3803 / 3382 / 3947 |
| FB, steps 51–200 | 722 / 722 / 722 | = P (acc1, cholmod byte-identical) | 809 / 809 / 809 | = F (acc1 byte-identical) |
| **PP, step 1 (outcome its/restarts)** | OK 884/18, OK 602/6, OK 830/15, **FAIL** 780/20 (r1) | OK 849/18, OK 622/6, OK 879/15, OK 825/18 (r1) | **FAIL** 248/20, OK 534/7, **FAIL** 674/20, OK 680/20 (r1) | **FAIL** 248/20, OK 573/7, **FAIL** 674/20, **FAIL** 257/20 (r1) |
| R4, 1 step | 583 / 510 | 89 / 133 | 191 / 190 | 151 / 108 |
| R4, 5 steps | 832 [577, 68, 60, 63, 64] | 426 [129, 97, 37, 103, 60]; Er 373 [91, 69, 67, 78, 68] | — | 367 [113, 63, 44, 55, 92] |
| ball-burst steps 31 / 32 | 251 / 217, 4 restarts | 234 / 196, 3 restarts | 264 / 266, 4 restarts | 174 / 340, 4 restarts |

Every failure is the named *Final reduced solve did not converge* after the
20th stall restart. Every run not marked FAIL exited 0 with all requested
steps.

**When does each option act?**

* **Estimate.** It is accepted only where the barrier and driving gradients
  are nearly anti-parallel (cosine ≥ 0.8):
  * R4: every step. Step 1 seeds 1 → 2⁻¹² at cosine 0.948, exactly the
    4096× bound. Later steps give factors of 0.999–1.35 at cosine ≥ 0.99.
  * IT: from step 154 (acc1: 13 seeds in steps 154–197).
  * `pup_push`: one late seed in some realizations (×0.76).
  * The ball-burst steps.
  * Nowhere else: 0 acceptances in 187–572 evaluations per run on the smokes,
    R1, BBT, FB and IT steps 1–153.

  A rejected estimate changes nothing, which is why E and B are byte-identical
  to P and F there. It still costs one system-gradient evaluation per
  accepted iterate while pending.
* **Band softenings vetoed by the pair guard.** The share tracks how narrow
  the gap distribution is, measured as the median force-weighted/rms gap
  ratio:

  | scene | fw/rms | vetoed softenings |
  |---|---:|---:|
  | R4 | 0.66 | 0 % |
  | BBT | 0.64 | 0 % |
  | IT, ITF | 0.75–0.76 | 31–37 % |
  | PP | 0.79 | 59 % |
  | R1 | 0.85 | 50 % |
  | smokes | 0.88 | 100 % |
  | FB | 0.89 | 94 % |

  The band's target [0.35, 0.50] d̂ lies below the average-gap collapse
  threshold √`trim_lower` = 0.707 d̂. Where the load-carrying pairs are not
  much closer than the average, the band keeps asking to soften and the guard
  keeps refusing. This is the structural reason for EF-02/03's low occupancy
  on R1 and BB, and for the extra iterations on FB.
* **Trim reversals.** Force-weighted IT has 69–96 per run against 15–30 for
  production. ITF has 80–92 against 8–20.

### The new force-weighted failure mode on `pup_push`

Trim decisions for F and P on the same deterministic realization (acc1),
where F fails and P completes:

* **Iterations 1–89, both arms identical.** The collapse branch lifts the
  trim 1 → 64 by iteration 39. After that a stall fires every ~9 iterations
  in both arms (alpha < 0.01 for 5 iterations: the contact front is
  CCD-limited).
* **Production after iteration 89.** It does not move the trim at those
  stalls: the gap is inside the band and there is no balance signal. Its
  stalls then thin out to the 100-iteration soft budget (iterations 198,
  298, …). It halves the trim four times by iteration 478 and completes at
  iteration 884 with 18 of 20 restarts.
* **Force-weighted after iteration 89.** It moves the trim down at 14 of the
  stall retunes and iteration checks (×0.3–0.8 each, 64 → 0.085). The
  force-weighted gap barely responds (0.68 → 0.58 d̂) while the minimum gap
  falls from 0.47 to 0.07 d̂. Every trim change is an objective change that
  discards Newton's history. The stalls never thin out, and the 20th restart
  comes at iteration 219, followed by the failure.

The completed F/B runs end at trim 0.22–0.25 (production 4) with minimum gap
0.075–0.092 d̂, just above the pair collapse threshold 0.0707 d̂. Their step-1
solution differs from production's by 1.9–5.4 %; production's own spread is
0.02–0.04 %.

Production is itself marginal on this scene: 18/20, 6/20, 15/20 restarts and
one multithreaded failure. So `pup_push` is a scene near the restart budget
for every controller, and the force-weighted band pushes it over.

### Accuracy

**Robust scenes (R1, BBT): tight reference.** Production realizations agree to
3.3e-4 on R1 and 1.05e-5 on BBT. The force-weighted solutions differ from
production by 5.6e-4 and 3.8e-3, far outside those spreads (ρ = 2.1 and
348). Against the pinned-trim tight reference, however:

* **R1:** force-weighted max/last error 7.6–7.8e-4 / 2.0–3.8e-4, against
  production's 7.4–7.6e-4 / 5.0–7.4e-4.
* **BBT:** force-weighted is closer at every step after the first, ending at
  4.1e-4 against 2.6e-3. Both are 1.29e-2 at step 1.

The force-weighted band runs at a lower realized gap: the smaller operating
gap RB-09 ties to a smaller reaction error (`e_R = c·ḡ`). So on robust scenes
it is at least as accurate. It does not reproduce production.

**Trajectory-sensitive scenes: production ensemble.**

| scene | E_P max | E: ρ | F: ρ | B: ρ | observables (F/B; aggregate ratio) |
|---|---:|---:|---:|---:|---|
| IT | 0.207 | 1.00 (Er 1.00) | 8.33 | 8.33 | kinetic 2.0, elastic 1.1, contact force 1.0 |
| ITF | 0.156 | 1.00 | 1.13 | 1.13 | kinetic 0.54, elastic 0.50 |
| R4 step 1 (2 repeats) | 0.063 | 0.75 | 0.95 | 1.51 | — |
| FB (robust: E_P 8e-14) | 8.4e-14 | = P | max D 1.1e-5 | = F | all ≤ 0.01 |

`physical_balance_pass` counts false flags per run:

| scene | P | E | F | B |
|---|---|---|---|---|
| IT | 109 / 111 / 101 | 108 / 108 / 101 | 102 / 93 / 120 | 98 / 92 / 119 |
| ITF | 108 / 112 / 109 | 108 / 112 / 109 | 112 / 107 / 116 | 112 / 108 / 116 |
| R1 | all false | all false | all false | all false |
| BBT | 2 / 2 / 1 | 2 / 2 / 1 | 0 | 0 |
| R4 | all false | all false | all false | all false |
| FB | 22 / 22 / 22 | 22 / 22 | 19 / 19 / 19 | 19 |

The candidates' counts lie within or near production's realization range;
per-step equality is not a meaningful gate (IT-reproducibility record).

**IT: production is the biased arm.** Pinned-trim runs, all acc1 unless
noted:

* Trim 64, 1 (acc1 and cholmod) and 2⁻³ complete 200 steps.
* Trim 2⁻⁶ fails at step 137.
* Trim 2⁻¹⁸ fails at the first impact (step 12) in all three realizations.

Kinetic energy (internal units):

| step | P (3 real.) | F (3 real.) | pin 64 | pin 1 (acc1 / cholmod) | pin 2⁻³ | pin 2⁻⁶ |
|---:|---|---|---:|---|---:|---:|
| 80 | 2.95–3.15e3 | 2.16–2.23e3 | 3.75e3 | 2.09e3 / 2.08e3 | 2.01e3 | 2.13e3 |
| 100 | 6.36–6.80e3 | 3.91–4.42e3 | 7.49e3 | 3.74e3 / 3.65e3 | 3.48e3 | 4.07e3 |
| 124 | 1.34–1.50e4 | 7.38–8.04e3 | 2.21e4 | 7.98e3 / 7.95e3 | 7.76e3 | 7.27e3 |
| 150 | 5.39–5.94e4 | 1.64–2.71e4 | 9.42e4 | 2.76e4 / 2.41e4 | 2.28e4 | — |

The rebound energy depends on the trim level and converges as the trim
decreases (64 → 1 → 2⁻³ → 2⁻⁶).

* Production raises the trim to 20–750 during IT's impacts. Its kinetic
  energy sits 73–93 % (step 124) and 136–160 % (step 150) above the lowest
  complete pin.
* The force-weighted band runs at trim 0.5–2 with minimum gaps 0.08–0.2 d̂
  (production 0.7–0.9). Its kinetic energy is within −5…+4 % (step 124) and
  −28…+19 % (step 150) of that pin.

The solution distance to the 2⁻³ pin shows the same through step 100 (step 80:
F 0.10–0.15, P 0.40–0.45; `it-ladder.txt`). After about step 120 all arms
decorrelate, including two realizations of the same pinned trim (pin 1 acc1
vs cholmod), so late-step solution distances cannot rank the arms.

**Consequence for the standard.** On IT, "within production's realization
spread" would reject the arm that is closer to the low-trim (hard-contact)
limit. The estimate alone never changes IT materially, so it passes the
ensemble test trivially.

### Restart and resume

* **Controller memory round trip.** IT, 30 steps with state files, resumed
  from `state_20` for 10 steps. P, E, Er, B and Br are all bit-identical to
  the uninterrupted steps 21–30. The estimate never fired in steps 1–30, so
  this covers `force_band_age`, the trim and the coefficient caches, not the
  estimate's flags.
* **Scope `run` is not restart-safe (confirmed).** `trim_seed_used_` is not
  in `BarrierContactForm::write_restart_state` (23 scalars, version 1), so
  `update_quantities` re-arms the estimate after a resume.
  * Er on IT (acc1), 170 steps with state files: the only seed is at step 154
    (×4.03).
  * Resumed from `state_160`, steps 161–168 are bit-identical. The resumed
    run then seeds again at step 169 (×0.59, which the uninterrupted run does
    not do), and the solutions separate (1.7e-7 at step 169).
  * Scope `step`, the default, is unaffected: it re-arms every step anyway.
  * The asset does not export the scope key, so Houdini users cannot reach
    this.

### Ball-burst looping steps 31–32 (resumed from EF-07 R0 `state_30`)

| arm | step 31 | step 32 | stall restarts | trim moves (31 / 32) | outcome |
|---|---:|---:|---:|---|---|
| P | 251 | 217 | 4 | 2 / 0 | completed |
| E | 234 | 196 | 3 | 3 / 2 | completed |
| F | 264 | 266 | 4 | 1 / 0 | completed |
| B | 174 | 340 | 4 | 3 / 3 | completed |

EF-07 on `ef07-b`, for reference: EF-02/03 mode with the proxy guard, 609/347
iterations and 97/53 moves; with pair, 259/258 and 2/1 moves; production
297/198.

No arm loops on this state with the current binary. Every arm completes both
steps with 3–4 stall restarts and at most 3 trim moves per step, and the
minimum gap stays at 0.070–0.085 d̂. Costs are within about ±25 % of
production (B's step 32 is the dearest, at 340 against 217).
Coverage stays limited to the two pinned pairs EF-07 found; `pup_push` shows
the band has a second route to restart exhaustion.

## Failure-mode catalogue

Status: **observed** (with where), **not observed** (where tested),
**untested**.

| # | failure mode | option | status |
|---|---|---|---|
| F1 | Pinched-pair limit cycle, collapse bump vs band softening, restarts exhausted (EF-07) | fw | EF-07 observed with proxy. Not observed with pair here: ball-burst 31–32 F, B; PP and FB show no alternation of that kind. Other pinned-pair scenes: none found. |
| F2 | **Softening cascade → restart exhaustion** on a stall-heavy, CCD-limited scene (new) | fw | **Observed:** PP step 1, F 2/4 and B 3/4 failures (P 1/4, E 0/4). The pair guard does not stop it. |
| F3 | Pair guard vetoes softening before the target is reached (low occupancy; extra iterations where the band cannot act) | fw | **Observed:** FB 94 % vetoed and +12 % iterations; smokes 100 % (no cost change); R1 50 %; PP 59 %. Structural: target 0.35–0.50 < collapse threshold 0.707. |
| F4 | More trim reversals → trajectory changes | fw | **Observed:** IT 69–96 vs 15–30, ITF 80–92 vs 8–20. On IT the trajectory moves toward the low-trim limit (kinetic energy), so this is not shown to be an accuracy loss. |
| F5 | Cost regression where production is already cheap | fw | **Observed:** FB +12 %. Not observed: smokes (identical), R1, BBT, IT, ITF, R4 (cheaper). |
| F6 | Accuracy outside production's realization spread | fw, B | **Observed:** IT ρ 8.3, R1 2.1, BBT 348 (robust), R4 B 1.5. Against tight/low-trim references F is equal or closer (R1, BBT, IT kinetic energy). |
| E1 | Off-equilibrium seed over-softens → collapse bumps, CCD-limited steps, restart churn | est | **Not observed:** 0 collapse decisions in any seeded step (R4, IT, PP, ball-burst); R4 seed at the 4096× bound followed by 0–2 restarts. |
| E2 | Large upward seed (×4096 allowed) → over-stiff barrier | est | **Observed but benign so far:** IT seeds up ×4–7 (E) and ×47 (Er simplicial); no failure. |
| E3 | Step-scope re-seeding → step-to-step oscillation on dynamic scenes | est | **Observed:** IT steps 154–197, alternating ×4–7 up and ×0.04–0.6 down. Ensemble statistics unchanged (ρ = 1.00). R4 re-seeds every step at ≈1 (benign). |
| E4 | Seed is not a trajectory optimum; later steps not cheaper | est | **Partly observed:** R4 5 steps 426 (E) and 373 (Er); production 832 [577, 68, 60, 63, 64] here. After step 1, E costs 297 and Er 282 against production's 255: the seed saves step 1 only. |
| E5 | Scope `run` state lost on restart | est (run) | **Observed** (see *Restart and resume*). |
| E6 | Rejected estimate evaluated at every accepted iterate (one system-gradient evaluation each) | est | **Observed:** 187–2,158 rejected evaluations per run; cost only, results byte-identical. |
| X1 | Friction interaction (`realized_force` lag, 2 iterations) | both | **Tested on ITF:** no failure; E = P; F/B ρ 1.13 (within the tolerance proposed below), observables 0.5. Friction smoke: identical. |
| X2 | AL passes / AL budget (RB-07) | both | R4 (the AL-pass scene) completed in every arm. AL budget off (default); budget on is untested. |
| X3 | Stall trigger / restarts, no retry (RB-08) | both | F2 is exactly this interaction. |
| X4 | Restart/resume of controller memory | both | Bit-identical except E5. |
| X5 | Clamped contacts (`exclude_statistics`) | both | Default on in every run; no clamped-driven event. `keep` untested with fw. |
| X6 | Adhesion / smooth contact | both | **Untested.** |
| X7 | Per-element materials | both | PP and FB (ActiveFiber per element) tested: see F2 and F3. |
| X8 | Quasistatic vs dynamic, integrators | both | Quasistatic: smokes qs/alhess/adaptive/friction (identical); R1/BBT/R4 implicit Euler; IT/ITF BDF3. |
| X9 | Very small d̂ (BBT 1e-5, R4 ~1e-6, PP/FB/BB 1e-6) | both | Covered; no d̂-specific failure beyond F2. |
| X10 | `physical_balance_pass` | both | Counts within or near production's realization range. Per-step flips are realization noise. |

## Proposed accuracy standard (for the user to agree or amend)

"Trajectory-sensitive" means a scene whose production realizations differ by
more than 1e-3 in relative L2 at any step. By that measure IT, ITF, R4, and
PP near its restart limit are trajectory-sensitive; R1, BBT and the smokes are
not. A candidate controller is acceptable as a default only if every item
holds on every scene of the agreed set.

1. **Robustness (gate).**
   * No step fails in any realization where production completes it.
   * The failure count over all realizations is ≤ production's.
   * Stall restarts per step stay within the scene's `max_restarts` with the
     same margin as production (the maximum over realizations may exceed
     production's maximum by at most 2).
2. **Reference ladder (A).** Pin the trim at 2^k for k = 6, 0, −3, −6, … and
   run each to the end with production tolerances, down to the smallest trim
   that completes.
   * For each observable Q (solution; kinetic and elastic energy; one-sided
     support force where it exists), let `Q_ref` be the smallest complete
     pin.
   * Robust scenes: the candidate's distance to `Q_ref` must be ≤ production's
     + 10 % at every step.
   * Trajectory-sensitive scenes: time-average the distance over the contact
     steps. It must be ≤ production's + production's realization spread.
   * This rewards moving toward the hard-contact limit. It encodes RB-09
     (`e_R = c·ḡ`), which favors lower realized gaps.
3. **Ensemble (B).** Use 3 deterministic realizations single-threaded (or 3
   repeats multithreaded).
   * ρ ≤ 1.25 over the contact steps, **or** item 2 holds with a margin that
     explains the excess (the candidate is closer to `Q_ref` than
     production).
   * Observable aggregate ratio ≤ 1.
   * False `physical_balance_pass` counts within production's realization
     range ± 10 % of the step count.
4. **Gap safety.** The endpoint minimum gap stays ≥ the pair collapse
   threshold (0.0707 d̂) at every accepted endpoint. The median endpoint
   mean gap is ≤ production's + 10 %.
5. **Scene set.**
   * The tuning scenes: smokes, R1, BBT, R4 (1 step × 2 and 5 steps).
   * **Held-out dynamic scenes, which are not used to tune any parameter:**
     `pup_push` (steps 1–3), `fibers` (steps 51–200 from a shared state),
     IT and ITF.
   * The ball-burst looping state (31–32).
   * `mesh_tissue` is excluded (no contact).
   * The user should add at least one more dynamic self-contact scene of
     their own if one exists. The held-out set above has only light
     self-contact (FB) and one two-body push (PP).

**Measured against this proposal (not a declaration that it is met):**

* **Force-weighted (F, B) fails item 1** on PP: 2/4 and 3/4 failures against
  production's 1/4. It would pass item 2 on R1, BBT and IT (kinetic energy)
  and fail ρ on IT without the item-2 exemption.
* **Estimate alone (E) passes items 1 and 3** on every scene measured, and
  passes item 2 trivially where it is inert. Item 2 on R4 is not assessed:
  there is no current R4 ladder, and EF-01's single-step reference is from an
  older binary. On that reference P is at 9.9–10.4 % and E at 9.0–9.5 %.
  Item 4 holds in every accepted E run.

## Recommendation

1. **No default change today.** The standard above is a proposal; it is
   not yet agreed.
2. **Force-weighted band: do not adopt.** It stays experimental, as the user
   decided on 2026-09-28.
   * F2 is a failure on a held-out scene, and F3 is structural:
     the target band sits below the collapse threshold on narrow gap
     distributions.
   * Adopting it would need a design change, not a gate relaxation. Options:
     a target defined relative to the collapse threshold, no softening at
     stall retunes, or a cap on band moves per step. That is future work,
     only if the user wants it.
   * Its measured accuracy advantage (closer to the low-trim limit on IT,
     BBT and R1) is real. It argues for revisiting how production's trim
     rises on impact scenes, not for adopting this controller.
3. **Initial estimate alone: the only candidate. Adopt only after these
   steps:**
   1. Agree the standard.
   2. Run its item 2 on R4 (a current pinned ladder).
   3. Repeat PP for steps 1–3.
   4. Decide the scope. Scope `run` removes E3 and costs less on R4 five-step
      (373 vs 426), but it needs E5 fixed first: persist `trim_seed_used_` in
      the state file (a version-2 layout).
   5. Possibly lower the 4096× bound or cap upward seeds (E2). That is the
      user's decision; it changes behavior on R4, where the bound binds.

   Whoever adopts it also changes the HDA default
   (`si_initial_trim_estimate`) and its test.
4. **Both together: no.** On every measured scene, B costs more than E or
   fails more often than E.

## Consequences if a default were flipped (for the user's decision)

* **Five public smokes.** VTU outputs do not move under any arm: 25 files are
  byte-identical, because the estimate is always rejected and the band always
  vetoed. Only `run-manifest.json` changes, gaining the experiment block. So
  `run-smoke.sh` byte-identity would still hold.
* **Houdini asset.** PolyFEM 2.0 always exports `band_statistic` and
  `initial_trim_estimate` explicitly (defaults `rms` / off). A PolyFEM default
  flip alone would therefore not change Houdini-exported scenes; the asset's
  parm defaults would have to change too. `tests/test_polyfem_hda.py` asserts
  the export (`rms`, `False`, no `collapse_guard_basis`) at lines 119–122, and
  its fw round trip at 252–259.
* **Unit and golden tests.** No PolyFEM unit test scene sets these keys. A
  flip would reach every semi-implicit scene that omits them. The effect on
  those goldens is not measured here; it needs the full suite (≥ 2 h cap).
  `tests/test_contact_cache.cpp` and `tests/test_trim_loop_guard.cpp` set the
  options explicitly and are unaffected.
* **RB-02 probe.** It is not run: nothing changed. Rerun it (`tools/rb02`)
  with any default flip.
* **Tools.** `tools/ef02/sequence.py`'s `PRODUCTION_CONTROLLER` pins
  `clamped_contacts: "keep"`. That has been stale since `eb8286b7c` made
  `exclude_statistics` the default. It acts only when a scene file sets the
  key (none do today), but production mode would then pin the non-default
  value. It is recorded here, not changed.

## What remains unproven

* The standard is proposed, not agreed. No arm is declared to meet it.
* The held-out set is small: PP (one step, four runs per arm), FB (light
  contact), IT and ITF. The force-weighted accuracy advantage on IT rests on
  one observable (kinetic energy) and a ladder whose smallest complete pin is
  2⁻³.
* R4 has no current pinned-trim ladder. Five-step R4 has one run per arm.
  Steps 1→5 differ from production by 9.0→3.8 % (E), 5.5→3.4 % (Er) and
  8.1→2.4 % (B). E and Er are identical in step 1, yet they differ from each
  other by 7.3→3.8 %, which is realization noise of the same size. So
  five-step accuracy is not separable from noise with one run per arm.
* The ball-burst coverage is two steps from one state.
* Adhesion, smooth contact, AL budget on, `clamped_contacts: keep` with fw,
  and Windows/Linux are untested.
* Production itself is marginal on PP: it failed 1 of 4 runs and needed 18/20
  restarts in others. That is a separate robustness question for the
  production controller and scene.
