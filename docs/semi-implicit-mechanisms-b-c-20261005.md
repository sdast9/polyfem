# Semi-implicit roundoff mechanisms B and C on the user's scenes

Date: 2026-10-05. Status: **measured; no code change** (publication: see the
end). Origin: the roundoff investigation of 2026-10-04 (parent workspace
`semi-implicit-roundoff-work/FINDINGS.md`) and the
[canonical-pair-keys record](canonical-pair-keys-20261005.md), which removed
mechanism A and left B and C reported but not measured. Every run here is on
top of canonical keys (`2a60cfa47`). The complete record, with every table and
the per-run evidence, is the parent workspace's `mechanism-bc-work/FINDINGS.md`;
this page keeps what the decisions rest on.

**Follow-up (2026-10-06):** *birth0g* is adopted as production behaviour,
without a historical setting; the adopted build reproduces the measured
*birth0g* runs byte for byte. See
[first-contact-refresh-20261006.md](first-contact-refresh-20261006.md).

**User decisions (2026-10-05), after this measurement:**

* **C: the 30-iteration in-solve trim cadence stays as it is.** No redesign and
  no per-step counter.
* **B: stale first-estimate stamps are not repaired; the guarded first-contact
  refresh (*birth0g*, below) is to be adopted** as separate work, with its own
  session and record (local branch `adopt-birth0g`, started from this record's
  commit). It changes the model at a contact onset, so the adoption carries the
  RB-02 probe, new smoke references and a check on a scene with a moving
  obstacle.
* The probe branch `mechanism-bc-probe` (local, not pushed, not for `main`) is
  kept for reference.

## What B and C are

**B.** A contact's coefficient is estimated once — a memo miss in
`memoized_stiffness`, evaluated by `estimate_stiffness` on the frozen snapshot
(surface and system Hessian) of the last refresh — and RB-20 continuation
carries that value from its first published capture on (between steps, AL pass
start, reduced-solve start, lagging). The repro's form of B is a first estimate
at a *degenerate* geometry: a gap below 0.05 d̂ at the snapshot, an edge pair
with sin θ < 1e-3, or a conditioning number above 100 (the largest |Δln κ| per
d̂ of random 1e-6 d̂ vertex motion). Measured here: the **stamp error**
|ln(κ_carried/κ_re-estimated)|, the re-estimate taken at the first capture's
state, and the contact's **gap shift** ½(1 − d/d̂)|Δln κ| (the investigation's
barrier-law estimate).

**C.** Inside a solve the trim steps down when `iters_since_trim` reaches
`controller_interval` (30) while the band statistic is pinned above
√trim_upper·d̂. The counter runs across solves and steps, so whether and where
a step fires depends on Newton iteration counts, which roundoff moves.

## Method

* **Probe** (local branch `mechanism-bc-probe`: `8e42824fc` probe, `5a21441ad`
  hybrid cadence, `1c4bd724b` *birth0g*, on `a7d39fc7b`). With no variable set
  it reproduces `2a60cfa47`: the five public smokes and the repro
  byte-identical, R1, BBT, pup_push and IT exactly (d = 0).
  * `POLYFEM_SI_BIRTHS=<file>` (observational) streams every fresh estimate
    that acted, with its snapshot's kind (step start, published mid-step,
    first-contact refresh, stall retune), whether the contact existed at the
    snapshot, the pair's gap there, the distance type, the edge-edge sin θ, the
    closest-point direction's tilt against the feature normal, the ratio to a
    feature-normal estimate and the conditioning number; a stamp row per key
    at its first capture (carried value, re-estimate, gap shift, prevalence);
    every trim move and step end.
  * `POLYFEM_SI_PROBE_B=regular,recapture,birth0,birth0g` (prototypes):
    *regular* estimates along the feature normal; *recapture* carries a
    re-estimate at a key's first capture unless the key was estimated in a
    published refresh's batch; *birth0* runs the first-contact refresh at the
    first accepted iterate instead of the second; *birth0g* = *birth0* without
    a collapse bump in a first-contact refresh.
  * `POLYFEM_SI_CADENCE=per_step|refresh|continuous|hybrid[,stall]`
    (prototypes): *per_step* resets `iters_since_trim` at every step;
    *refresh* turns the in-solve cadence off and halves the trim once per step
    at the step's first solve-start refresh when the band statistic there is
    pinned above √trim_upper·d̂; *continuous* the same with a factor
    ((1 − r)/(1 − trim_upper))² of the band ratio r, clamped to [2⁻¹², 1];
    *hybrid* = per_step + refresh; `,stall` adds stall retunes as decision
    points. (Deciding at the between-steps refresh does not work: see C,
    finding 2.)
* **Scenes** (single-threaded, Accelerate on one thread, physical diagnostics
  off): R1 3 steps, BBT 20, pup_push 3, IT 200 (BDF3), R4 5 steps with its
  scene's initial estimate and step 1 without it, ball-burst steps 31–32
  resumed from EF-07 R0 `state_30` (8 threads), fibers steps 51–200 resumed
  from the default-controller assessment's `state_50`, and the 1e-15 repro
  with six variants.
* **Ensembles** per arm: base, the second body (R4: the only body) translated
  by +1e-15 and −1e-15 in x, 1e-15 in y and 1e-15 in z, and the CHOLMOD and
  Simplicial realizations; d = time-averaged ‖u_a − u_b‖ / mean‖u‖ on body
  nodes, pooled over all pairs of completed runs. Accuracy against pinned-trim
  references (D1 item 2 as proposed): trim pinned at 2⁻¹⁸ for R1 and BBT; for
  IT the lowest pin of the ladder that completes (2⁻³; 2⁻⁶ fails at step 134);
  for pup_push only 2⁻² completes (at 20 restarts), so its reference is weak.
  Arms are compared with production's runs with the same inputs.

## B — measurements

First captures per scene:

| scene | estimates acted | born after their snapshot | degenerate | first captures | > 1 % | > 10 % | > ×2 | gap shift median / p99 / max (d̂) |
|---|---|---|---|---|---|---|---|---|
| R1 (3 steps) | 356 | 83 % | 0 | 105 | 87 % | 51 % | 8.6 % | 0.0011 / 0.024 / 0.026 |
| BBT (20) | 54 | 98 % | 0 | 47 | 45 % | 8.5 % | 0 | 5e-6 / 1e-4 / 1e-4 |
| pup_push (3) | 11,067 | 69 % | 0 | 458 | 41 % | 11 % | 0.2 % | 2e-4 / 0.011 / 0.012 |
| IT (200) | 25,745 | 99 % | 0 stamped (2 near-parallel estimates) | 4,756 | 98 % | 84 % | 23 % | 0.0085 / 0.11 / 0.83 |
| R4 (5, scene's estimate) | 8,201 | 89 % | 0 | 3,633 | 64 % | 27 % | 0.3 % | 4e-4 / 0.022 / 0.13 |
| ball-burst 31–32 | 18,998 | 53 % | 0 | 4,359 | 11 % | 2.3 % | 0 | 7e-6 / 0.0048 / 0.046 |
| fibers 51–200 | 25 | 96 % | 0 | 19 | 0 | 0 | 0 | 4e-6 / 1e-4 / 1e-4 |
| repro (d̂ 0.01) | 84 | 55 % | 20 tiny gap, 2 parallel | 28 | 43 % | 11 % | 0 | 5e-5 / 0.011 / 0.011 |

* **The repro's B does not occur in the user's scenes.** No first capture
  there was estimated at a degenerate geometry, and the conditioning number
  never exceeds 100 (maximum 11.6, IT). The first-contact refresh stamps few
  contacts (none in R1, pup_push, R4 and ball-burst, one in BBT and in fibers,
  ten at IT's re-contacts, at 0.10–0.83 d̂ with interior distance types).
* **What occurs is stale stamps.** A contact born during a solve is estimated
  on the snapshot of the solve start (or of the last stall retune), where the
  pair was still apart, and continuation carries that value. Pooled over the
  seven scenes: 8,060 such first captures, 63 % off by more than 10 % and 14 %
  by more than ×2; the 5,317 first captures estimated while the contact
  existed are off by more than 10 % in 1.7 % of cases. The error grows with
  the pair's distance at the snapshot (IT: median |ln| 0.040 below 1 d̂, 0.42
  at 5–20 d̂). In R1 and pup_push it is mostly the closest-point direction
  (correlation with the direction's ambiguity 0.97 and 0.93; the
  feature-normal direction lowers R1's share above 10 % from 51 % to 20 %); in
  IT the closest features change because the bodies move several d̂ per step.
  IT's stale stamps are on average 25 % softer than the regular-state
  estimate; in-contact stamps are unbiased. Share of all carried coefficients
  whose first capture was off by more than 10 % / ×2: R1 34 % / 5.1 %, BBT
  1.3 % / 0, pup_push 6.6 % / 0.1 %, IT 84 % / 21 %.
* **No measurable effect on results.** No B prototype reduces a scene's spread
  or moves it toward its reference by more than the spread (the five-run arms
  are base + nudges; production's spread over those runs is R1 8.8e-5, IT
  0.25, pup_push 4.5e-4):

| scene | arm | runs (failed) | iterates / stall retunes per run | pooled spread | distance to the reference (vs production, same inputs) |
|---|---|---|---|---|---|
| R1 | production | 7 (0) | 359 / 5.3 | 7.3e-5 | 6.5e-4 |
| R1 | birth0 / birth0g | 7 / 5 (0) | bit-identical to production | – | 1.00× |
| R1 | recapture | 7 (0) | 357 / 4.7 | 1.0e-4 | 0.98× |
| R1 | regular | 7 (0) | 352 / 5.7 | 1.2e-4 | 0.96× |
| BBT | production | 7 (0) | 205 / 0 | 9.6e-7 | 4.0e-3 |
| BBT | birth0 / birth0g | 7 / 5 (0) | bit-identical to production | – | 1.00× |
| BBT | recapture | 7 (0) | 202 / 0 | 8e-10 | 1.00× |
| BBT | regular | 7 (0) | 205 / 0 | 9.6e-7 | 1.00× |
| IT | production | 7 (0) | 5,422 / 13.6 | 0.21 | 0.43 |
| IT | production, base + nudges | 5 (0) | 5,469 / 14.4 (runs 3,971–7,280 / 4–27) | 0.25 | 0.42 |
| IT | birth0 | 5 (0) | 5,307 / 13.4 | 0.25 | 1.00× |
| IT | birth0g | 5 (0) | 5,703 / 18.8 | 0.23 | 1.01× |
| IT | recapture | 7 (0) | 6,526 / 27.7 | 0.19 | 1.08× |
| IT | regular | 5 (0) | 6,801 / 29.4 | 0.19 | 1.09× |
| pup_push | production | 4 (0) | 852 / 14.2 | 3.7e-4 | 1.9e-2 |
| pup_push | birth0 | 2 (0) | bit-identical to production | – | 1.00× |
| pup_push | recapture | 4 (0) | 857 / 14.2 | 3.7e-4 | 0.99× |

The stale stamps make a carried coefficient depend on how far a pair travelled
within the step (a time-step dependence of the model; inferred from the gap
correlation, not measured with a time-step study). Their gap shifts are
fractions of d̂, inside the barrier's own model error (RB-09).

## B — the repro's floor and the prototypes

At the repro's first contact `post_step` returns at iteration 0, and PolySolve
reports both its start point x₀ and the first accepted iterate x₁ with
iteration 0. x₁ is the CCD-limited state (every new contact at 0.010 d̂); the
first-contact refresh therefore runs at x₂, a Newton step taken with
coefficients estimated on the pre-contact snapshot and an uncalibrated trim
(gaps 0.0198–0.0201 d̂). x₂ is already the first untruncated iterate (CCD bound
1.0, the step before 0.99), and it is the ill-conditioned state: at d̂ = 0.02
the edge pairs `pEE[48,73]` / `pEE[60,73]` are at sin θ = 5e-4 (types
EA1_EB / EA0_EB) in the unperturbed run and 5e-3 (interior type, conditioning
50) in the 1e-15 run, and their stamps differ ×2.3. At x₁ the same edges are
exactly parallel in both runs and the toolkit's tie-break is the same.

Perturbation family: 7 repro variants × 8 perturbations of 1e-15–1e-14; a
"jump" is a largest body difference above 1e-9 (fixed stiffness responds with
≤ 0.29 δ):

| variant | production | birth0 | recapture | regular | birth0g (jumps, max) | iterations production / birth0g | final trim production / birth0g |
|---|---|---|---|---|---|---|---|
| base d̂ 0.01 | 8/8, 8.9e-9 | 0/8, 5e-15 | 0/8, 6.5e-11 | 0/8, 1.8e-10 | 0/8, 8.1e-15 | 46 / 41 | 3.6 / 2.0 |
| d̂ 0.005 | 0/8, 1.4e-10 | 0/8, 1.8e-12 | 0/8, 1.9e-12 | 0/8, 2.2e-13 | 0/8, 4.1e-13 | 52 / 43 | 37 / 4.2 |
| d̂ 0.02 | 8/8, 2.5e-4 | 0/8, 8.7e-11 | 8/8, 7.6e-6 | 8/8, 2.7e-4 | 8/8, 5.7e-8 | 51 / 43 | 1 / 1 |
| d̂ 0.04 | 8/8, ≤ 1.3e-5 | 7/8, ≤ 1.0e-7 | 8/8, ≤ 3e-6 | 8/8, ≤ 4e-6 | 7/8, ≤ 1.0e-7 | 49 / 49 | 1 / 1 |
| cube 4×4×4 | 8/8, 1.7e-4 | 0/8, 2.5e-11 | 8/8, 2.9e-5 | 8/8, 3.2e-4 | 0/8, 4.2e-15 | 72 / 42 | 89 / 2.0 |
| cube 8×8×8 | 8/8, 7.6e-8 | 7/8, **2.0e-4** | 8/8, 7.5e-8 | 0/8, 1.7e-12 | 0/8, 2.5e-15 | 81 / 52 | 25 / 2.0 |
| wide plate | 0/8, 7.7e-16 | 0/8 | 0/8 | 0/8 | 0/8, 1.9e-15 | 35 / 35 | 7.1 / 3.4 |
| **all (56)** | **40 jumps**, max 2.5e-4 | 14, max 2.0e-4 | 32, max 2.9e-5 | 24, max 3.2e-4 | **15, max 1.0e-7** | | |

The cadence prototypes leave the floor as it is (36–37 jumps). *birth0* fixes
base, d̂ 0.02 and cube4 but creates large jumps on cube8: at the CCD-limited x₁
every gap is 0.01 d̂, the collapse branch reads that as a collapse and bumps
the trim to 905 (production 99 at x₂), and the stiffer onset is itself
sensitive. *recapture* stops a bad stamp from persisting (d̂ 0.02: 2.5e-4 →
7.6e-6, onset step only) but not the onset; *regular* fixes cube8 and worsens
cube4. **birth0g** answers the cube8 regression: it removes every large
response of the family (median 4e-15), makes the onsets cheaper and lowers the
trim after the onset 2–45×.

On the user's scenes *birth0g* is bit-identical on R1 and BBT (and *birth0* on
pup_push; *birth0g* was not run there). Its guard applies to every first-contact
refresh, so it also acts at IT's re-contacts: the trajectories differ, and the
iterations, retunes, spread and distance to the reference stay within
production's run-to-run scatter (over the same five inputs 5,703 against 5,469
iterates and 18.8 against 14.4 stall retunes per run, where production's runs
range 3,971–7,280 and 4–27; distance to the reference 1.01×). The four
semi-implicit public smokes move by 6.1e-6–6.9e-6 relative (their cube is
pressed into the slab by a CCD-truncated first step); `quasistatic-adaptive`
is byte-identical. The other prototypes cost: *recapture* IT +20 % iterations
and twice the stall retunes, R1's spread 1.0e-4 against 7.3e-5, and it brings
back once per newborn contact the post-publication force change RB-20 removed;
*regular* IT +25 % iterations and twice the retunes.

### What adopting *birth0g* involves

(Done 2026-10-06: [first-contact-refresh-20261006.md](first-contact-refresh-20261006.md).)

* The first-contact refresh for the accepted iterate that `post_step` reports
  with iteration 0 (the solver also reports its start point with iteration 0;
  the probe tells them apart by comparing coordinates with the previous
  `post_step`), and no collapse bump in a first-contact refresh (the
  conditioning cap applies instead). About 20 lines; the probe's form is
  `BarrierContactForm::post_step` and `refresh` on `mechanism-bc-probe`
  (`8e42824fc` + `1c4bd724b`, behind the switch).
* A narrower variant — the guard only at the iteration-0 refresh — would leave
  IT's later first-contact refreshes unchanged; it was not measured.
* A `[kappa_continuity]` or scene case for the repro's linear response, the
  RB-02 probe (coefficient-law/controller change), new smoke references, and a
  check on a scene with a moving obstacle (none of the proposed D1 set has
  one).

## C — measurements

**Census** (production; δ = Newton iterations a roundoff perturbation could
shift a firing point by; a move is *cross-step marginal* when it fired within δ
iterations of a step start or end; a step end is marginal when the gap was
pinned above the band and the count was within δ of 30):

| scene (steps) | trim moves by source | cadence moves | cross-step marginal (δ = 2 / 5 / 10) | step ends pinned above the band | … within δ = 5 of firing | gap shift per cadence move, max (d̂) |
|---|---|---|---|---|---|---|
| R1 (3) | cadence 8, stall soften 3, refresh down 1 | 8 | 0 / 1 / 1 | 3 | 1 | 0.033 |
| BBT (20) | cadence 5, refresh down 1, calibration 1 | 5 | 0 / 4 / 5 | 20 | 4 | 0.008 |
| pup_push (3) | collapse 6, cadence 4, calibration 3 | 4 | 0 / 0 / 0 | 0 | 0 | 0.092 |
| IT (200) | collapse 62, refresh down 28, cadence 27, conditioning cap 1 | 27 | 1 / 4 / 6 | 15 | 2 | 0.20 |
| R4 (5, scene's estimate) | initial estimate 5, cadence 1 | 1 | 0 / 0 / 0 | 0 | 0 | 0.31 |
| R4 step 1, no estimate | cadence 11, calibration 1 | 11 | 0 / 0 / 0 | 1 | 1 | 0.21 |
| ball-burst 31–32 | collapse 1 | 0 | – | 0 | 0 | 0.32 |
| fibers 51–200 | calibration 116, collapse 3 | 0 | – | 0 | 0 | 0.16 |
| repro (8) | collapse 2, cadence 1 | 1 | 0 / 1 / 1 | 3 | 1 | 0.051 |

The realistic δ is large where the onset is chaotic: R1's step-1 iteration
count ranges 201–222 over four 1e-15 nudges (5 cadence halvings in step 1 in
the base run, 4 in one nudge, 6 in another). R1's largest nudge outlier
(4.9e-4 against ≤ 7.6e-5 for the other three) is one halving that slipped
from iteration 200 of step 1 to iteration 4 of step 2.

**Ensembles** (cost = accepted Newton iterates of all attempts per completed
run):

| arm | R1 pooled spread / iterates / retunes | BBT iterates (trim at the end) | IT failed / iterates / retunes | IT pooled spread / distance to the 2⁻³ pin (vs production, same inputs) | pup_push failed (step-1 restarts) | R4 step 1, no estimate |
|---|---|---|---|---|---|---|
| production | 7.3e-5 / 359 / 5.3 | 205 (0.016) | 0 of 7 / 5,422 / 13.6 | 0.21 / 0.43 | 0 of 4 (17, 6, 18, 16) | 575 iterates, 7 retunes, trim 0.00066 |
| no cadence (diagnostic) | 1.4e-5 / 510 / 10.6 | 224 (0.5) | 3 of 7 / 5,672 / 18.8 | 0.17 / 0.39 (1.02×) | 3 of 4 (20, 6, 20, 20) | 1,057 iterates, 15 retunes, trim 0.5 |
| per-step counter | 7.2e-5 / 355 / 5.0 | 224 (0.5) | 0 of 7 / 4,928 / 10.6 | 0.43 / 0.32 (0.75×) | 0 of 4 (17, 6, 18, 16) | = production |
| refresh step | 1.7e-5 / 475 / 10.0 | 174 (0.00024) | 1 of 7 / 6,681 / 38.7 | 0.22 / 0.43 (1.01×) | 3 of 4 (20, 6, 20, 20) | = no cadence |
| continuous law | 9.7e-5 / 372 / 7.4 | 150 (0.00029) | 2 of 7 / 5,973 / 31.0 | 0.22 / 0.44 (1.02×) | 3 of 4 (20, 6, 20, 20) | = no cadence |
| hybrid (per-step + refresh step) | 1.2e-4 / 348 / 4.7 | 174 (0.00024) | 0 of 7 / 4,791 / 10.4 | 0.44 / 0.32 (0.74×) | 0 of 4 (17, 6, 18, 16) | = production (inferred, not run) |

pup_push's runs in the order of the restart counts: acc1, CHOLMOD, +x nudge,
Simplicial. Every IT failure is the named *Final reduced solve did not
converge* after the scene's 20th stall restart (steps 175–198); every pup_push
failure is step 1 after 20 restarts, in the three runs where production needs
16–18. On R4 step 1 the refresh step and the continuous law never fire (one
step, one decision point): their probe streams are identical to the
no-cadence run's until they were stopped as duplicates at its 15th restart.
With stall retunes as extra decision points (`refresh,stall`, run on R1, BBT
and R4 only) R4 step 1 takes 844 iterates and 9 retunes (+47 %).

**Accuracy.** R1 and BBT (base runs, distance to the 2⁻¹⁸ reference; the D1
robust test is ≤ 1.1× production at every step):

| arm | R1 (vs production) | BBT (vs production) | D1 robust test |
|---|---|---|---|
| production | 6.8e-4 | 4.0e-3 | – |
| no cadence | 0.99× | 1.00× | pass / pass |
| per-step counter | 1.00× | 1.00× | pass / pass |
| refresh step | 1.00× | 0.94× | pass / pass |
| continuous law | 0.83× | 0.90× | pass / pass |

IT (distance to the 2⁻³ pin over steps 12–100, against production's runs with
the same inputs): per-step counter 0.75× (all seven runs below production's
mean), hybrid 0.74×, no cadence 1.02× (0.91× against all seven: its three
failed inputs are production's three farthest runs), refresh step 1.01×,
continuous law 1.02×. Kinetic-energy proxy at step 124 against the pin:
production +69 … +103 %, per-step counter +31 … +71 % (pins 2⁰ and 2⁶: +7 % and
+92 %). D1's ρ over three realizations: per-step counter 1.72, hybrid 1.75
(Simplicial diverges in both), no cadence 1.05, refresh step 1.04, continuous
law 1.03. Production is the high-trim-biased arm on IT; the per-step counter
runs at lower trims there (median log₂ trim over steps 50+: 6.5 against 7.2).

## C — structural findings

1. **The in-solve cadence is the only downward path inside a solve.** The
   gradient-balance calibration only raises the trim, collapse bumps raise it,
   and stall softening needs a stall without a balance signal. Without the
   cadence, stall retunes do the walk (R1: 13 instead of 5) at the price of
   restarts; on R4 step 1 without the estimate the walk does not happen at all.
   On IT the cadence is the counterweight to in-solve collapse bumps: in
   production's steps 193–200 it fires 1–7 times per step against 3–8 collapse
   bumps, with at most 3 stall retunes; without it the retunes per step climb
   (9, 3, 12, 18, 6, 10, 15) until step 197 exhausts the scene's 20.
2. **At a converged endpoint the calibration returns the trim in force.** Every
   minimize starts with its own published refresh, so the between-steps refresh
   and the solve-start refresh see the same state; a downward step at the first
   is undone by the calibration of the second (on R1 the refresh-point
   halvings and the calibration raises cancelled, −11.45 against +11.45 in
   log₂). A state-based step has to sit at the solve-start refresh.
3. **Production's cross-step accumulation is a feature on short-step scenes.**
   BBT's steps take 8–13 iterations; the count reaches 30 every ~4 steps and the
   trim drifts from 0.5 to 2⁻⁶ over 20 steps. A per-step counter removes that
   drift (the trim stays 0.5, +9 % iterations, the same distance to the
   reference).
4. **What no in-solve rule avoids.** The halving count inside a step follows
   the step's iteration count, which the contact onset makes roundoff-sensitive.
   A per-step counter removes the cross-step slip and moves the boundary effect
   to step ends: on R1 the outlier moves from the +x nudge to the Simplicial
   realization, where a step-2 halving fires two iterations before the end, and
   the pooled spread is unchanged (7.2e-5 against 7.3e-5).

## C — why it stays

The state-based replacements (once per step on the endpoint band statistic,
its continuous version) remove the only in-solve downward path and fail the
proposed D1 robustness gate: pup_push step 1 fails in 3 of 4 runs, IT fails late
steps in 1 and 2 of 7 runs, and R4 step 1 without the estimate costs 84 % more
(the stall form +47 %). The per-step counter is cheap and robust but relocates
the discrete event instead of removing it, doubles IT's spread and removes
BBT's drift; the hybrid is worse on R1. C is the price of walking the trim down
inside a solve; its effect is a fraction of d̂ per cadence move (first-order gap
shift ≤ 0.03 d̂ on R1, 0.2 d̂ on IT, 0.3 d̂ on R4). The IT observation that lower
trims on impacts sit closer to the low-trim reference belongs to the D1/D3
discussion of the default controller, not to C.

## Limits

* One deterministic realization per nudge; spreads from 5–7 runs per arm; IT's
  spreads are chaotic (single divergent members dominate).
* pup_push's ladder is nearly empty (only 2⁻² completes), so its reference is
  weak; *birth0g* was not run on pup_push.
* R4 ran single-threaded (the user's runs are threaded); R4 with its estimate
  was measured for B only. Ball-burst and fibers ran once (production only).
* The repro family is 7 variants of one geometry.
* Stamp errors compare with a re-estimate at the capture state, a reference
  rather than ground truth: the coefficient law has no exact value.

## Evidence

Parent workspace `mechanism-bc-work/`: `FINDINGS.md` (the full record),
`README.md` (layout); `polyfem/` (worktree on `mechanism-bc-probe`),
`build/`, `configure-command.txt`, `build.sh`; `bin/` (baseline and
`-probe1`…`-probe7` binaries, `hashes.txt`, source patches); `runs/verify/`
(identity checks, the candidates' smoke changes), `runs/obs/` and `runs/bb/`
(probe streams of production runs), `runs/B-*`, `runs/F-*`, `runs/G-*` (repro
prototypes and the perturbation family), `runs/ens/<scene>` (arms × runs),
`runs/ref/` (pinned-trim references), `runs/final-tables.json`,
`runs/record-tables.md`; `tools/` (`births_summary.py`, `b_tables.py`,
`cadence_census.py`, `ensemble.py`, `final_tables.py`, `record_tables.py`,
`d1eval.py`, `family.py`, `jobqueue.py`, and the investigation's tools,
adapted); `inputs/` (repro meshes, the fibers resume state, copied).

## Publication

Local commit on branch `semi-implicit-bc-record`, on top of
`canonical-pair-keys` (`a7d39fc7b`). **Not pushed**: it builds on canonical
keys, which the user has not pushed yet (2026-10-05: "not yet"); push it with
them. The probe branch `mechanism-bc-probe` stays local.
