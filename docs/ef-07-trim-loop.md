# EF-07 — Force-weighted band and initial estimate without a controller loop

Date: 2026-09-27/28. Status: **diagnosed; one opt-in repair removes the loop
(`collapse_guard_basis: pair`); strict cost and accuracy gates unmet; defaults
unchanged.** Adoption is the user's decision.
Plan: [contact-efficiency-plan-20260923.md](contact-efficiency-plan-20260923.md),
section EF-07. Previous record: [ef-02-03-trim-controller.md](ef-02-03-trim-controller.md).

## Question

The user's ball-burst run with the EF-02/03 opt-in controller
(`band_statistic: force_weighted`, `initial_trim_estimate: true`) slowed to
~1,000 Newton iterations per step and looped at step 42 (156 stall restarts
under `max_restarts: 2000`, trim 3e-4 … 1.5e6). Can the opt-in controller keep
its benefit without that limit cycle, and with what change?

## Answer in brief

* The loop is two controller branches fighting over one **pinched pair that
  the global trim cannot move**: an edge-edge pair of the knit whose gap stays
  0.055–0.068 d̂ while the trim sweeps five decades. The collapse branch bumps
  the trim whenever the pair touches 0.0707 d̂; the force-weighted band (gap
  ≈ 0.8 d̂) softens it back. With the default 20 restarts the loop ends a run
  with the named failure *Final reduced solve did not converge*.
* The EF-02/03 guard meant to stop exactly that softening evaluates the
  scalar force law on the collapse *proxy*, 10 · g_min, instead of the pair's
  gap; near the threshold it never binds. Evaluating it on each collapse term's
  own gap (**`collapse_guard_basis: pair`**, opt-in) removes the loop on every
  looping step measured: 1–12 trim moves instead of 54–112, at most 5 stall
  restarts, no failed step. The other candidates (responsiveness veto, partial
  guard, per-step excursion bound, reversal lockout) do not.
* Cost and accuracy on the EF-01 matrix + BB + IT: no new failures; the pair
  basis costs **more than the EF-02/03 mode** on IT (+13–14 %), BB (120 vs 90)
  and one-step R4 (102/124 vs 81/98), less on R1/BBT, and its R4 solutions sit
  further outside production's repeat envelope. The strict EF-07 gates are not
  met.
* On the current base (`6a4e788bf` onward) the **production rms controller**
  is no longer the slow reference on R4: 102/116 iterations for R4 step 1
  (EF-02/03 recorded 657/703 on its 2026-09-26 base) and 401/348 for five
  steps, level with both force-weighted modes. The force-weighted modes still
  win clearly on BB (90–120 vs 385), IT, R1 and BBT. The cause of the R4
  change was not isolated (the weighted band statistic, the IPC upstream
  integration and Tight-Inclusion 1.1.0 all landed since EF-02/03).
  **Correction (2026-09-28,
  [r4-production-speedup-20260928.md](r4-production-speedup-20260928.md)):**
  the R4 scene file was re-exported on 2026-09-26 with `band_statistic
  force_weighted` and `initial_trim_estimate true`, and the matrix's
  production mode does not override them, so every R4 "production" run below
  ran the v3 configuration (their manifests record it). The true production
  controller on `6a4e788bf` still takes ≈ 600 iterations for R4 step 1. The
  R4 production numbers, the R4 part of the cost gate and the R4 "production
  repeat spread" in this record are v3 repeats; other scenes are unaffected.

## Evidence and method

Two evidence roots. `outputs/ef-07/20260927T231825Z/` holds the first half
(instrumented builds, R0/R1, control-state resumes c0–c4, step-31 base/pair,
the first matrix). It is part of the historical `outputs/` being moved to the
Pitt share (see AGENTS.md, *Archived test outputs*). `ef07-work/` in the parent
workspace holds the rebased build, the step-31 partial/reversal/excursion/rms
runs, the matrix reruns and a copy of the first matrix without its state
files (`matrix/runs/`), plus small summaries of the first half (`prev/`). The
original run's step-42 extract and per-step table are in
`outputs/ef-07/20260927-ball-burst-evidence/`; the user's rms control run's
per-step table (steps 1–40) is `outputs/ef-07/control-rms-steps.txt`.

On 2026-09-28 the disk filled while this matrix ran (each ball-burst run's
opt-in `coefficient-events.jsonl` grew past 10 GB and the pre-`c2a57e393`
binaries wrote 4 GB state files). Runs lost to that (production R4 five-step,
production BB, the partial-guard BB, the step-31 partial/reversal/excursion
runs) were rerun from scratch in `ef07-work/` on the same binaries; the
disk-full partial-guard BB run is kept as `diskfull-partial-candidate-BB-s1-r1`
and is not used.

Binaries (hashes in `ef07-work/bin/hashes.txt`): `c133948cf` (R0); `ef07-a`
(instrumented, R1); **`ef07-b`** (EF-07 on `6a4e788bf`, IPC `cf99893b`,
PolySolve `6a8c2cc9`; all matrix modes and most ball-burst runs); `ef07-c`
(= ef07-b + the partial guard); **`ef07-d`** (the published sources, EF-07
rebased on `99949c132`, PolySolve `43ca2e66`; regression checks). Runs
overlapped each other and other sessions' jobs; iteration counts, not wall
time, are compared.

Ball-burst runs (reconstructed looping params, default `max_restarts` 20,
`output/trim_predictors`):

* **R0** — from step 0 on `c133948cf`.
* **Control-state resumes** — the rms control run's step-40 state (t = 12 s,
  7,891 contacts) resumed for steps 41–43; the state predates `c133948cf`, so
  the controller starts fresh (trim 1, then the estimate). R1 on `ef07-a`,
  options off; c0–c4 on `ef07-b`.
* **Step-31 resumes** — R0's own `state_30` (controller memory included)
  resumed for steps 31–32. No resumed step ran augmented-Lagrangian passes, so
  the state files' missing AL multipliers (restored since `48a5e16bf`) do not
  affect them.

Instrumentation (trim-predictor record, observational): a
`controller_decision` for every trim move including collapse bumps; the
minimum-gap pair (`collapse_pairs.min_pair`: type, full vertex ids, gap, born
since the previous accepted iterate, in the last refresh set); the number of
pairs below the collapse pair threshold; the loop-guard counters.
`tools/ef07/reduce.py` attributes each move to its branch and reports
reversals and the excursion from the step's first trim;
`tools/ef02/sequence.py --set` passes EF-07 options to the matrix driver.

## Attribution: what the loop is

**1. A two-branch cycle at the collapse threshold.** Original step-42 log: the
collapse branch doubles the trim when the minimum gap touches 0.0707 d̂ (the
proxy `100·min d² < trim_lower·d̂²`); the gap rises to 0.080–0.099 d̂; the
force band (force-weighted gap ≈ 0.8 d̂, target [.35, .50]) softens ×0.80 and
×0.60; the gap drifts back; repeat. Every trim change is an objective change
that discards Newton's history. R0 shows the cycle growing over the run:
collapse-up vs band-down moves per step 3/4 at step 10, 8/16 at step 29, 10/17
at step 30, then 52/46 at step 31, which exhausted its 20 restarts (717
iterations, 112 moves, 50 reversals, trim 0.013 → 2.2e5) and stopped with the
named failure. (The original run reached step 42; the reproduction diverges
from step 1 on the newer binary: 77 instead of 94 iterations.)

**2. The EF-02/03 downward guard does not bind near the threshold.**
`ForceWeightedTrim::safe_band_step` compares the scalar barrier-force response
of `sqrt(severity)` with `sqrt(trim_lower)` = 0.707. When the minimum term
binds, `sqrt(severity)` = 10 · g_min, a number the force law says nothing
about: at g_min = 0.08 the guard admits factors ≥ 0.50, at g_min ≥ 0.0991 any
factor. On the pair's own gap against its own threshold `sqrt(trim_lower/100)`
= 0.0707 it requires ≥ 0.891 and ≥ 0.732, which vetoes both observed
softenings.

**3. The pinched pair does not respond to the trim.** In every control-state
resume the minimum gap is set by the same edge-edge pair A (PolyFEM node ids
42242–193974 × 56495–107873, both knit edges of ~40 µm; see point 3a). It
stayed the minimum across 44 of 47 trim moves (R1) and 73 of 77
(c0), and was born in the current iteration in only 2–12 % of records. Its gap
by trim decade in R1: 0.055–0.061 at trim 1, 0.061–0.065 at 10, 0.065–0.066 at
100, 0.066–0.068 at 1e3–1e5. A global trim scales every contact's force
alike, so a pair squeezed between other contacts is not opened by it. Across
stall restarts the in-solve 256× climb budget is re-anchored at each refresh,
which is how R1 reached 6e5 (1.1e6× the step start). On R0's trajectory the
pinned pair at steps 31–32 is a different one, pair B (90823–199920 ×
122379–166336), the minimum in 485 of 502 records of the rms resume and 981 of
1,471 of the partial-guard resume.

**3a. The pinned pairs are not held by boundary conditions.** One resumed
step 31 (`ef07-work/bc-probe/`, `ef07-b`, rms) exported PolyFEM's node
positions and displacement in its internal numbering, which the pair ids use
(the `.msh` vertex order is different: in it the same ids are millimetres
apart). Nodes with exactly zero displacement are the clamped ones (18,218, the
knit rim of sideset 10010101); the 988 ball nodes move with the prescribed cap.
All eight nodes of pairs A and B are free, about 4.8 mm from the nearest
clamped node, 0.55 mm (A) and 0.61 mm (B) from the nearest ball node, and
carried about 3.0–3.1 mm by the push (knit median 1.1 mm). After step 31 their
edge-edge gaps are 0.18 d̂ (A) and 0.12 d̂ (B). They are knit self-contact in
the region the ball drags furthest, consistent with a yarn squeezed between
neighbouring contacts; the surrounding geometry was not inspected.

PolyFEM's forward solve builds its collision mesh from the whole boundary
surface, without Dirichlet filtering, and leaves IPC's `can_collide` at its
default (every primitive may collide with every other); only remeshing and
shape optimization set a filter. So contacts between clamped primitives, or
between a clamped and a free one, are built and evaluated: their forces on
clamped degrees of freedom drop out of the solve, but they enter the trim
controller's gap statistics. They did not drive this loop: in all 18,069
trim-predictor records with the minimum-pair field (13 ball-burst runs), the
minimum-gap pair had no clamped node. A pair of clamped primitives cannot
respond to the trim, so one setting the minimum would cause the same kind of
loop; how much they shift the band statistics was not measured.

**4. Neither the estimate nor born-this-iteration pairs drive it.** The
estimate moved the trim at most once per step in every run (while pending it
is evaluated at every accepted iterate — R0 step 1: 71 rejections, one
system-gradient evaluation each). The minimum pair was born in the current
iteration in 2–12 % of records, so `collapse_exclude_born` was implemented but
not measured on the scene.

**5. Why late steps are slow at all (context, not EF-07's cause).** Per-iteration
cost is flat; the iteration count grows. In R0, 75–95 % of line searches are
cut by CCD at every step and the median fraction of the Newton step CCD allows
falls from 0.29 (step 2) to 0.03 (step 31); Armijo-limited searches stay at
~10–15 %. Active contacts grow ~20× by step 30. The rms control run slowed the
same way (20–50 iterations per step early, 300–450 by steps 33–40). The
elastic energy rises smoothly (barrier ≈ 1.4 % of it); nothing isolates it as
a cause.

## Candidates (opt-in; every option off reproduces EF-02/03 byte for byte)

| key under `/solver/contact/semi_implicit/` | what it does |
|---|---|
| `collapse_guard_basis: pair` | band veto on each collapse term's own gap and threshold (average vs `sqrt(trim_lower)`, minimum pair vs `sqrt(trim_lower/100)`) |
| `collapse_guard_partial: true` | a vetoed softening is clamped to the smallest factor the force law calls safe instead of cancelled |
| `collapse_responsiveness_veto: true` | within a step, skip a collapse bump when the previous one did not lift the collapse gap by 10 % and the gap has not fallen below half its value at that bump |
| `collapse_exclude_born: true` | in-solve minimum term ignores collisions not active at the previous accepted iterate |
| `trim_step_excursion: B` | trim within [start/B, start·B] for the whole step, across restarts |
| `trim_reversal_limit: K` | after K reversals in a step, band/estimate moves may not reverse again (collapse exempt) |
| `initial_trim_estimate_scope: run` | the estimate is accepted at most once per run |

Invalid values are named errors; the manifest lists the active options under
`coefficient_law.controller.ef07`. The per-step memory is part of the form's
rollback state and resets at each time step; it is not written to restart
state files (a resumed step starts it fresh, which equals its step-start
value). No Houdini control is added.

## Results on the looping regime

Iterations include every attempt of the step; excursion is relative to the
step's first trim.

| run | step | Newton its | stall restarts | outcome | trim moves (up/down) | reversals | max excursion |
|---|---|---:|---:|---|---|---:|---:|
| R0 (`c133948cf`, options off, from step 0) | 31 | 717 | 20 | **failed** | 112 (57/55) | 50 | 1.1e7× |
| step 31–32, options off | 31 / 32 | 609 / 347 | 7 / 3 | completed | 97 / 53 | 51 / 33 | 427× / 18× |
| step 31–32, **pair** | 31 / 32 | 259 / 258 | 2 / 2 | completed | 2 / 1 | 1 / 0 | 2× / 1× |
| step 31–32, production `rms`, no estimate | 31 / 32 | 297 / 198 | 2 / 1 | completed | 1 / 1 | 0 / 0 | 2× / 1× |
| step 31–32, pair + partial | 31 / 32 | 291 / 716 | 2 / 9 | completed | 42 / 107 | 20 / 37 | 4.8× / 282× |
| step 31–32, reversal lockout K = 4 | 31 / 32 | 214 / 396 | 2 / 20 | step 32 **failed** | 8 / 61 | 5 / 5 | 2.8× / 6.7e4× |
| step 31–32, excursion bound 256 | 31 / 32 | 980 / 1,041 | 16 / 14 | completed | 153 / 163 | 69 / 87 | at the bound |
| R1 (`ef07-a`, options off) | 41 | 441 | 20 | **failed** | 47 (30/17) | 9 | 1.1e6× |
| c0 options off | 41 / 42 | 678 / 387 | 18 / 20 | step 42 **failed** | 77 / 54 | 22 / 10 | 138× / 2,870× |
| c1 **pair** | 41 / 42 / 43 | 435 / 330 / 502 | 5 / 3 / 5 | completed | 12 / 8 / 10 | 6 / 5 / 5 | 11× / 2.1× / 4.1× |
| c2 responsiveness veto | 41 / 42 / 43 | 355 / 447 / 584 | 3 / 7 / 11 | completed | 8 / 7 / 10 | 3 / 2 / 6 | 23× / 4× / 4× |
| c3 pair + veto | 41 / 42 / 43 | 433 / 401 / 480 | 6 / 4 / 4 | completed | 11 / 6 / 3 | 4 / 5 / 1 | 12× / 2× / 2× |
| c4 excursion bound 256 | 41 (stopped at 297 its to free the machine) | 297 | 14 | — | 34 (19/15) | 4 | at the bound |

For scale, the rms control run took 294–453 iterations per step over steps
37–40 with 1–4 stall restarts, and the production controller resumed on R0's
looping step (same state, `ef07-b`, physical diagnostics off) completed steps
31–32 in 297 / 198 iterations with one trim move per step.

Reading: only the pair basis (alone or with the veto) removes the cycle —
moves and reversals drop by an order of magnitude, the trim stays within ~4×
of the step start and restarts stay ≤ 6. Softening partially to the "safe"
factor reintroduces a smaller cycle that grows (step 32: 107 moves, 9
restarts); the veto alone stops the futile collapse climbs but not the band's
softening, so its restarts grow 3 → 7 → 11; the reversal lockout lets collapse
ratchet the trim up once softening is locked out and fails; the excursion
bound only contains the swing. Collapse protection is retained by the pair
basis: it vetoes softening only; collapse bumps are unchanged.

## Acceptance (plan, EF-07 step 4)

Matrix on one binary (`ef07-b`); `v3` = EF-02/03 mode (fw + estimate), `pair`
= v3 + `collapse_guard_basis: pair`, `partial` = pair + `collapse_guard_partial`
(`ef07-c`). Iterations (repeat runs separated by /):

| scene | production (rms) | v3 | pair | partial |
|---|---:|---:|---:|---:|
| R1, 3 steps | 376 | 202 | 190 | 185 |
| BBT, 20 steps | 186 | 119 | 118 | 117 |
| IT, 200 steps | 6,027 / 5,789 | 3,546 / 3,196 | 4,039 / 3,622 | 3,702 |
| R4, 1 step | 102 / 116 † | 81 / 98 | 102 / 124 | 106 / 86 |
| R4, 5 steps | 401 / 348 † | 393 / 434 | 453 / 399 | — |
| BB, 1 step | 385 | 90 | 120 | 81 |
| five smokes | 31, 26, 37, 31, 62 | identical | identical | — |

† Not production: the R4 scene file selects `force_weighted` + the initial
estimate, so these runs are v3 repeats (see the correction under *Answer in
brief*). The production controller on R4 step 1 takes ≈ 600 iterations on
`6a4e788bf` ([r4-production-speedup-20260928.md](r4-production-speedup-20260928.md)).
The R4 "production" solution differences and repeat spread below are
likewise v3-vs-v3.

Solution differences to production (relative L2 per step; production's own
repeat spread in brackets): R4 one step v3 4.0–4.9 %, pair 3.4–5.5 % [4.7 %];
R4 five steps, step 1 → 5: v3 3.8–5.0 % → 1.3–2.2 %, pair 3.7–8.7 % → 2.3–3.6 %
[4.8 % → 1.8 %]; BB v3 2.4 %, pair 1.9 % (no production repeat); R1, BBT ≤ 0.4 %.
IT is not reproducible run to run on this host (identical-settings repeats
differ by up to 17.7 % in solution, 4–11 % in iterations and 45–53 balance
flags), so its 97–115 % cross differences and 50–54 flag changes do not
isolate a controller effect.

| check | result |
|---|---|
| Looping ball-burst step completes, restarts ≤ 20, bounded excursion | **Pass** with pair (steps 31–32 and 41–43: ≤ 5 restarts, ≤ 11×); every other candidate fails or loops |
| No new failures, EF-01 matrix + BB + IT | Pass: all matrix runs exit 0, no timeouts |
| Iterations ≤ EF-02/03 v3 (and ≤ production where v3 was) | **Not met**: pair > v3 on IT, BB and one-step R4; pair ≤ production everywhere except R4 (one step 102/124 vs 102/116, five steps 453/399 vs 401/348) |
| Accuracy within the Newton envelope and RB-09 gap error | **Not met** on R4 (pair's cross differences exceed production's repeat spread, more than v3's do); BB/R1/BBT small; IT not assessable |
| `physical_balance_pass` unchanged | R4 and BB unchanged for every pair; R1/BBT/smokes unchanged vs v3; IT not assessable (above) |
| RB-02 probe | Pass: 270/270 on `ef07-d` (`ef07-work/rb02/`) and `ef07-b` |
| Five smokes byte-identical, options off | Pass: 25 VTU files identical to clean `99949c132` (`ef07-d`) and to `6a4e788bf` (`ef07-b`) |
| Affected unit selection | Pass: 86 cases / 9,705 assertions on `ef07-d`; `[ef07]` 9 cases / 123 assertions |
| 13 HDA scripts | Pass: all 13 with Houdini 22.0.429 against `ef07-d` and `ef07-b` in isolated roots |

## Decision (user, 2026-09-28)

* The force-weighted band stays **experimental**; production `rms` is
  recommended and remains the default.
* It is exposed in the Houdini asset as an experimental choice
  (houdini-plugins, see the asset's help); when chosen, the asset always
  exports `collapse_guard_basis: pair`.
* PolyFEM enforces the pair basis for the force-weighted band: `pair` is the
  default of `collapse_guard_basis`, which acts only in that mode; an explicit
  `proxy` reproduces the EF-02/03 guard with a warning. Production (`rms`)
  runs and manifests are unchanged. `tools/ef02/sequence.py`'s `candidate`
  mode now requests `proxy` so it keeps measuring EF-02/03 v3.
* **Standing condition:** Before the force-weighted mode can be considered for adoption it first needs an agreed accuracy standard for trajectory-sensitive scenes, and repeat evidence on dynamic scenes not used for tuning.

## Conclusion and recommendation

The EF-02/03 loop is understood: a pair the global trim cannot open, a
collapse branch that keeps bumping for it, and a downward guard evaluated on a
proxy it was not derived for. `collapse_guard_basis: pair` is the only
measured change that removes the loop while keeping collapse protection; it is
a correction of the guard's own stated intent rather than a new heuristic.

**Recommendation: keep production `rms` and every EF-07 option off by
default.** If the force-weighted mode is used on ball-burst-like scenes (as the
user did), use it **with `collapse_guard_basis: pair`**: without it the mode
fails such scenes under the default restart budget; with it it completed every
looping step measured. Making that the default of the force-weighted mode, or
the force-weighted mode the default, is the user's decision; the evidence
above does not meet the plan's strict cost/accuracy gates (IT/BB/R4 cost versus
v3, R4 accuracy versus production's repeat spread). Note that on the current
base the rms controller does not loop on the ball-burst step that broke the
force-weighted mode (297 / 198 iterations vs pair's 259 / 258). (This record
also said the rms controller was no longer slow on R4; that was a scene-file
artefact — see the correction above. Production still takes ≈ 600
iterations on R4 step 1, so R4 stays among the scenes where the
force-weighted mode saves cost, with BB, IT, R1 and BBT.)

Out of scope and not changed: the per-contact coefficient law, CCD, the
trial-displacement cap, the stall trigger, RB-08's no-automatic-retry, the
retired floor. The scene-side causes of the slow late steps (d̂ = 1 µm against
~55 µm edges, CCD-limited steps as contact densifies, the E 1e12 ball driving
~1 % of coefficients to the `kappa_spread` cap) are recorded, not addressed.

## Open

* ~~The cause of the production controller's R4 improvement since EF-02/03 was
  not isolated.~~ **Attributed 2026-09-28**
  ([record](r4-production-speedup-20260928.md)): there was no ~6× production
  improvement. The R4 scene file re-exported on 2026-09-26 selects
  `force_weighted` + `initial_trim_estimate`, so this record's R4
  "production" runs were v3 runs. True production on `6a4e788bf` takes
  591 / 600 iterations for R4 step 1 (871 on `6a59cb387` rerun, 657 / 703 in
  EF-02/03). The only code effect is moderate: the collision-weighted band
  statistic (`6a4e788bf`) lets production leave trim 1 after 212–291 instead
  of 314–459 iterations (`c133948cf`, with the IPC/PolySolve adoptions,
  still 379 / 352); the ~350-iteration walk down from trim 1 is unchanged.
  Open: `tools/ef02/sequence.py` production mode does not pin the controller
  options, so it inherits whatever a re-exported scene selects.
* IT's run-to-run irreproducibility on this host (single-threaded): **cause
  found 2026-09-28** ([record](it-reproducibility-20260928.md)).
  `Eigen::AccelerateLDLT`'s internal threads ignore `--max_threads` and are
  not bitwise deterministic, so identical runs differ from step 1 at 1e-16.
  Loose descent-test stops then amplify this from step 12 (first stiff
  contact solve), and the contact trajectory carries it to 17.7 %. With
  `VECLIB_MAXIMUM_THREADS=1` or CHOLMOD, 200-step repeats are bit-identical,
  so nothing else in single-threaded runs is nondeterministic. **Fixed
  2026-09-28** (user decision): the binary now caps Accelerate at
  `max_threads` whenever a limit is set, so `--max_threads 1` runs are
  bit-reproducible (200-step IT repeats identical without the variable);
  earlier single-threaded Accelerate evidence stays a random realization. Even deterministic, two roundoff realizations differ by
  up to 23.5 % and 3,807 vs 6,385 iterations, so the IT cross comparisons
  above stay unassessable pairwise.
* Opt-in `coefficient-events.jsonl` (physical diagnostics) grows past 10 GB
  per ball-burst step; it filled the disk once during this work.
* ~~Contacts between clamped primitives are part of the collision set and of
  the controller's gap statistics (point 3a). Not a cause here; excluding them
  from the statistics (or from the collision set) would be a separate change.~~
  **Measured 2026-09-29** ([record](clamped-contacts-20260928.md)): fully
  clamped contacts occur only on R4 (≤ 2 active), BB and the ball-burst
  step-31 state (≤ 3, knit-rim self-contact among them); R1, BBT, IT and the
  smokes have none. They never set the minimum gap in production runs,
  flipped no collapse or calibration-gate decision, and shift the band, gap
  and batch statistics by ≤ 0.7 % (BB's first-refresh batch median 5.1 %,
  R4's gradient-balance trim 2.2 %). Opt-in `semi_implicit/clamped_contacts`
  (`exclude_statistics` / `exclude_collisions`, default `keep`) prototypes
  both exclusions: exact no-ops without a fully clamped contact; with them
  (R4, BB, ball-burst) no failure, and iterations and solutions not
  separable from the multithreaded run-to-run spread. Defaults unchanged;
  adoption is the user's decision. New open item there: the gradient balance
  counts Dirichlet rows (on IT the free-DOF balance trim is ~2.1×).
