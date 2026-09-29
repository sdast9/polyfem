# Gradient balance on free DOFs: characterization and opt-in prototype

Date: 2026-09-29. Status: **characterized; opt-in prototype
`semi_implicit/gradient_balance_dofs: free`. Decision (user, 2026-09-29):
the default stays `all`;** `free` remains an opt-in experiment.
Origin: [clamped contacts](clamped-contacts-20260928.md), *Open* (first
bullet) and its "Separate finding".

## Question

The semi-implicit trim controller's gradient balance —
`BarrierContactForm::calibrate_trim` (upward-only trim
κ_gb = −⟨gB,gE⟩ / (w‖gB‖²) after the cosine gate cos ≥ 0.1),
`estimate_initial_trim` (the force-weighted mode's initial estimate, the same
balance with `initial_trim_cosine`) and the trim-predictor record's
`gradient_balance` — is taken over all full DOFs. gE is the system gradient
provider's (elastic, inertia, body, pressure forms), gB the barrier gradient
mapped by `collision_mesh_.to_full_dof`. Both include the Dirichlet DOFs
(`boundary_nodes`, obstacle nodes included), while the reduced solve
balances free DOFs only. Which scenes does that affect? Is the full-DOF
balance diluted by the Dirichlet reactions in gE or by the clamped half of
gB? What trim does the reduced problem's own balance imply? And what
happens when the balance is restricted to free DOFs?

## Answer in brief

* **Mechanism (exact).** Split both gradients into free and Dirichlet rows.
  Then
  κ_all = κ_free · x · s_B² and cos_all = cos_free · x · s_B · s_E, with
  s_B = ‖gB_free‖/‖gB‖ (the clamped half of partly clamped contacts),
  s_E = ‖gE_free‖/‖gE‖ (Dirichlet reactions and any energy on clamped rows)
  and x = ⟨gB,gE⟩/⟨gB,gE⟩_free (correlation of the two gradients' Dirichlet
  rows). **The reactions dilute only the cosine; the balance itself is
  shifted only by the clamped half of the barrier gradient** (s_B) and by
  x. Against a rigid obstacle (no energy on its rows) x = 1 and s_E = 1, so
  κ_all = κ_free · s_B² exactly.
* **What the reduced balance implies.** At a converged reduced solve
  without friction or AL terms, gE_free = −w·trim·gB_free, so **the
  free-DOF balance equals the trim in force** (cos 1). Measured at the
  published endpoints: κ_free/trim median 1.000 on every smoke, BBT, D1–D4,
  Q1/Q2, 0.985–0.998 on IT (maximum 4.1–4.9 when the next step's inertia
  loads the contact). The full-DOF balance sits at trim · s_B² · x instead:
  0.11 of the trim on the smokes, 0.45 on IT.
* **Which scenes.** Only contacts with a clamped side move κ, and by how
  much depends on how the clamped side is meshed:

  | scene | clamped side | κ_free/κ_all (median) | s_B | s_E | cos all → free (median) |
  |---|---|---:|---:|---:|---|
  | five smokes, synthetic D1 | the 4-vertex slab obstacle: 25–50 contacts sum onto 4 nodes | 8.7–9.5 | 0.33–0.34 | 0.66–1 | 0.21–0.34 → 0.96–1 |
  | IT | sphere on the `mat_target` obstacle | 2.1 (max 3.0) | 0.69 | 1 | 0.064 → 0.094 |
  | D3 / Q1 | a clamped elastic block (cube.mesh), same resolution as the free cube | 2.3–2.9 | 0.66–0.69 | 0.66–0.85 | 0.42–0.44 → 1 |
  | D2 | a fine obstacle grid (spacing 0.1, finer than the cube's 0.25) | 1.24 | 0.90 | 1 | 0.90 → 1 |
  | R1, BBT, BB, D4, Q2 | none (Dirichlet rows away from the contacts) | 1 | 1 | 0.22–0.68 | diluted by reactions only |
  | R4 | grips, far from the self-contacts | 1 | 1 | 0.995 | 0.67 → 0.67 |

  The coarser the clamped side, the more the obstacle half of many contact
  forces concentrates on few nodes, and the smaller the full-DOF balance:
  the smokes' 9× is a property of the two-triangle slab, not of the physics
  (D1 vs D2 is the same drop on a coarse and a fine obstacle: 8.7× vs 1.24×).
* **Consequence under the default (`all`).** On obstacle scenes the
  upward-only calibration is systematically below the trim in force
  (0.11× on the smokes, ~0.45× on IT at endpoints), so it rarely acts: on
  IT it raised the trim once in 200 steps (Accelerate realization; 0 in the
  CHOLMOD and SimplicialLDLT realizations). Where no contact has a clamped
  side, it already equals the trim at equilibrium and behaves like the free
  balance.
* **Prototype.** `gradient_balance_dofs: free` zeroes the Dirichlet rows of
  both gradients before the balance in `calibrate_trim` and
  `estimate_initial_trim` (and reports the free balance in the record's
  top-level `gradient_balance`, with `dofs: free`). The gradients are
  unchanged IPC assemblies; with the option off (absent or `all`) the code
  path is the production one and every compared run is **byte-identical**
  (five smokes 25/25 VTU via `run_smoke.py`; R1, BBT, IT and four smokes
  through the matrix driver, 246 VTU; six synthetic scenes, key absent and
  `all`).
* **Effect of `free`.** Where no contact has a clamped side it changes
  roundoff only (BBT byte-identical, D4 6.5·10⁻¹⁶), except where it lifts a
  reaction-diluted cosine over the gate at a stall retune (R1: 2 more stall
  retunes, 348 → 375 iterations, 1.1·10⁻⁵; see below). Quasistatic obstacle scenes (smokes, Q1)
  change by ≤ 2.3·10⁻⁷: at equilibrium the free balance equals the trim, so
  calibration only makes raises below 1 %. On transient obstacle scenes it
  becomes active: IT 7–8 calibration raises per run instead of 0–1, and the
  cosine gate passes at 18–22 more full records per run. **But no cost or accuracy
  effect separates from IT's realization spread** (three deterministic
  realizations per mode: 4,339 / 4,489 / 6,002 Newton iterations with
  `free` vs 6,385 / 3,807 / 5,087 without; stall retunes 5 / 7 / 21 vs
  23 / 1 / 10; the modes differ by 6.5–21 % in solution, the realizations
  of one mode by up to 24 % and 33 %; realized minimum gaps median
  0.80–0.82 d̂ in both modes). R4 and BB (two multithreaded repeats per
  mode): no effect beyond their run-to-run spread; see *Multithreaded (R4, BB)*.
* **Force-weighted mode: the initial estimate is effectively disabled on
  all DOFs.** `estimate_initial_trim` gates at cosine 0.8. The full-DOF
  cosine never reaches it on the smokes (max 0.229), BBT (0.326) or R1
  (0.252), and on IT only in its last 24 steps; so the experimental
  force-weighted mode runs without its estimate there. With `free` the
  cosine reaches 0.93–1.0 and the estimate is accepted on every scene
  (IT 53–69 times per run instead of 3–12). Iterations: smokes −4 to +1,
  R1 196 → 175, BBT 119 → 121; IT's three realizations 3,163–3,258 vs
  3,260–4,313. The accepted estimates are mostly below the trim in force,
  so the barrier is softer and gaps tighter: IT's median per-step minimum
  gap 0.15–0.19 vs 0.18–0.20 d̂, 2 of 187 contact steps below the
  collapse-pair threshold (0.0707 d̂) against 0 of 194.
* **Side effect: near-zero raises.** Because the free balance equals the
  trim at equilibrium, calibration "raises" the trim by 10⁻⁶ or less at
  many endpoints (D1: 8 of 10 raises below 1 %, 4 below 10⁻⁵). Each raise
  resets `iters_since_trim_` and notes an objective change, which
  postpones the band's downward step (`controller_interval`): in D1 the
  default halves the trim at step 23 when the resting contact sits above
  the band (rms 0.95–0.97 d̂); `free` never does and ends at 1.07. The same
  happens today on scenes without a clamped side (D4: 8 of 11 raises below
  1 % under the default). A minimum relative raise would remove it; not
  prototyped.
* **Recommendation: keep `all` as the default for now.** The free balance
  is the consistent one (it is the balance the reduced solve satisfies,
  independent of how an obstacle is meshed), and nothing failed with it.
  For the production controller it buys nothing measurable on these
  scenes, and it extends the near-zero-raise interaction to every obstacle
  scene. For the experimental force-weighted mode it is what makes the
  initial estimate work at all, at the price of somewhat tighter gaps on
  IT. If adopted, pair it with a minimum relative raise. **Decided
  (user, 2026-09-29): `all` stays the default.**

## Mechanism

Let F be the free rows and D the Dirichlet rows, both gradients full-DOF
vectors. With ⟨·,·⟩_F, ⟨·,·⟩_D the partial inner products:

    ⟨gB,gE⟩ = ⟨gB,gE⟩_F + ⟨gB,gE⟩_D,   ‖gB‖² = ‖gB‖²_F + ‖gB‖²_D,   ‖gE‖² = ‖gE‖²_F + ‖gE‖²_D

    κ_all = −⟨gB,gE⟩ / (w‖gB‖²) = κ_free · x · s_B²
    cos_all = cos_free · x · s_B · s_E

with s_B = ‖gB‖_F/‖gB‖, s_E = ‖gE‖_F/‖gE‖, x = ⟨gB,gE⟩/⟨gB,gE⟩_F.

* **Dirichlet reactions** live in gE's Dirichlet rows (a prescribed face,
  a clamped grip). Where the barrier has no component on those rows they
  enter only ‖gE‖, so they lower the cosine (s_E) but not κ. BBT's
  reactions give s_E = 0.22 and cos 0.215 against 0.999 on free DOFs,
  without any change in κ.
* **The clamped half of a contact** is the barrier gradient on the clamped
  side's rows. Against an obstacle (no energy on its rows, so x = 1) it only
  adds to ‖gB‖², and κ_all = κ_free · s_B². For a single vertex over an
  edge midpoint the edge's two vertices carry half the force each, s_B² = 2/3
  (the unit test). When many contacts press on few obstacle nodes, the
  obstacle-side forces add up coherently on those nodes: the smokes' 25
  contact vertices on the slab's 4 corner nodes give s_B² ≈ 0.11.
* **Both** appear when the clamped side carries energy: D3's clamped block
  has free interior nodes, loaded by gravity, so its clamped surface rows
  carry elastic reactions that correlate with the barrier there (x = 0.71).
* **Equilibrium identity.** At a converged reduced solve, the free rows of
  the full gradient vanish: gE_F + w·trim·gB_F = 0 (without friction, AL
  or other forms outside the provider; the barrier gradient includes the
  per-contact coefficients). So κ_free = trim and cos_free = 1 there. The
  calibration is therefore at its margin at every converged endpoint; it
  raises the trim when the next step's load on the free DOFs (inertia
  predictor, body force, pressure) exceeds what the barrier carries at the
  current gaps. A quasistatic scene driven by Dirichlet motion adds its load
  on Dirichlet rows, which the free balance does not see at the endpoint.

## Measurement (option off)

Evidence `fable_polyfem/gradient-balance-work/` (outside `outputs/`).
Binaries (hashes in `bin/hashes.txt`): `base` = `main` at `d53b9e444` and
`free` = the prototype on the same base, both with IPC `f8dafef39e8` and
PolySolve `43ca2e661` (`main`'s pins), both with the Accelerate thread cap
(`ba3ea76b6`). Every run states the production controller explicitly
(`band_statistic: rms`, `initial_trim_estimate: false`,
`clamped_contacts: exclude_statistics`) through `tools/clamped/sequence.py`
(now with `--balance-dofs`). The `base` IT, R1 and BBT runs are
byte-identical to the clamped-contacts record's `default4` runs.

Reduction (`tools/balance/reduce.py`, `characterization.json`): the full
trim-predictor records' `clamped.gradient_balance` block, compared on the
controller's own balance (`excluding_fully` under the default) and
`free_dofs`. Medians [range] over the full records:

| scene | full records | κ_free/κ_all | cos all | cos free | s_B | s_E | x | gate passes: both / free only / neither | κ_free/trim | κ_all/trim |
|---|---:|---|---:|---:|---:|---:|---:|---|---|---|
| smoke-qs, -alhess, -tr | 8 | 9.1 [5.5, 9.2] | 0.219 | 1.000 | 0.334 | 0.670 | 1 | 7 / 0 / 1 | 1.000 | 0.110 |
| smoke-friction | 12 | 9.5 [5.5, 9.6] | 0.208 | 0.963 | 0.325 | 0.663 | 1 | 11 / 0 / 1 | 1.000 | 0.105 |
| IT (200 steps) | 159 | 2.11 [1, 2.96] | 0.064 | 0.094 | 0.688 | 1 | 1 | 54 / 16 / 89 | 0.985 [−0.03, 4.9] | 0.451 |
| R1 (3) | 11 | 1 | 0.181 | 0.211 | 1 | 0.323 | 1 | 6 / 1 / 4 | 0.017 | 0.017 |
| BBT (20) | 40 | 1 | 0.215 | 0.999 | 1 | 0.215 | 1 | 39 / 0 / 1 | 1.000 | 1.000 |
| R4 (1, 18 threads) | 9 | 1 | 0.668 | 0.674 | 1 | 0.995 | 1 | 9 / 0 / 0 | 0.124 | 0.124 |
| BB (1, 18 threads) | 5 | 1 | −10⁻⁶ | −10⁻⁶ | 1 | 0.671 | 1 | 1 / 0 / 4 | ≈ 0 | ≈ 0 |
| D1 drop, 4-vertex slab | 58 | 8.7 [5.4, 8.9] | 0.339 | 1.000 | 0.339 | 1 | 1 | 57 / 1 / 0 | 1.000 [.., 1.04] | 0.115 |
| D2 drop, 31×31 grid obstacle | 58 | 1.24 [1.24, 1.38] | 0.896 | 0.999 | 0.897 | 1 | 1 | 58 / 0 / 0 | 1.000 [.., 1.04] | 0.804 |
| D3 drop, clamped block | 62 | 2.95 [2.2, 6.4] | 0.417 | 1.000 | 0.690 | 0.849 | 0.712 | 61 / 1 / 0 | 1.000 [.., 1.05] | 0.339 |
| D4 drop, elastic support (control) | 62 | 1 | 0.561 | 0.996 | 1 | 0.564 | 1 | 62 / 0 / 0 | 1.000 | 1.000 |
| Q1 press, clamped block | 7 | 2.29 | 0.439 | 1.000 | 0.660 | 0.665 | 1 | 7 / 0 / 0 | 1.000 | 0.436 |
| Q2 press, elastic support (control) | 7 | 1 | 0.675 | 1.000 | 1 | 0.675 | 1 | 7 / 0 / 0 | 1.000 | 1.000 |

R4 and BB are from the clamped-contacts record's `exclude_statistics`
runs (`fixed-contacts-work/matrix2/`, the same code path); BB's balance is
almost orthogonal (cos ≈ 0) at most records in both variants. R1's
median κ/trim of 0.017 comes from its stall retunes (6 of 11 full records,
taken mid-solve); at its endpoint refreshes κ equals the trim to three
digits in both variants, and only the cosine differs (0.21–0.25 vs
0.84–0.97, the plate's reactions). No gate ever
passes on all DOFs and fails on free DOFs. The IT counts reproduce the
clamped-contacts record's (2.06–2.15× there).

Reading:

* **Which scenes.** Every scene where a free body touches an obstacle or a
  clamped body: the five public smokes (cube on the slab obstacle), IT
  (sphere on an obstacle), the synthetic D1–D3 and Q1. Not R1, BBT, BB or
  R4: their clamped DOFs (plate edges, the ball's prescribed components,
  the knit rim, grips) carry no contact force the controller sees, apart
  from the few fully clamped contacts that `exclude_statistics` already
  removes (R4 ≤ 2, BB ≤ 3).
* **Which dilution.** κ moves only through s_B (and x where the clamped
  side has energy). The cosine is diluted by both: on the smokes s_B 0.33
  and s_E 0.67 multiply to 0.22; on IT only s_B (0.69); on BBT, R1, BB, D4
  and Q2 only s_E. With the gate at 0.1 the dilution rarely matters except
  on IT, whose cosine is low throughout (median 0.064 all, 0.094 free).
* **What the reduced balance implies.** The trim in force (median ratio
  1.000 at endpoints), plus the next step's free-DOF load increment on
  transient scenes. The full-DOF balance implies a trim that is s_B²·x
  smaller: 1/9 of the trim on the smokes, 0.45 on IT, 0.34 on D3.

## Option and implementation

`/solver/contact/semi_implicit/gradient_balance_dofs` (spec entry; `all`
default, `free`; any other value is a named error):

| value | `calibrate_trim`, `estimate_initial_trim` | record `gradient_balance` (top level) | manifest |
|---|---|---|---|
| `all` (default) | unchanged: all full DOFs | unchanged | unchanged (the key is not listed) |
| `free` | the Dirichlet rows of gE and of the (IPC-assembled, `to_full_dof`) gB are set to zero before the norms, the cosine and the quotient | the same restriction, with `dofs: free` | `coefficient_law.controller.gradient_balance` and a status line naming the experiment |

* `BarrierContactForm::restrict_balance_rows` zeroes the rows of the
  Dirichlet list `SolveData::init_forms` already passes to the form
  (`set_dirichlet_dofs(boundary_nodes, dim)`, since the clamped-contacts
  work). It is a no-op when the option is off, when no Dirichlet list is
  installed, or when the sizes differ. The gradients are still assembled
  by `BarrierPotential::gradient` (through `controller_barrier_gradient`)
  and the provider; nothing is re-scattered, so the option-off path is
  bit-identical (the clamped-contacts record's *implementation error* was a
  serial scatter that changed roundoff).
* Fully clamped collisions act on Dirichlet rows only, so under `free` the
  `exclude_statistics` filter of the balance is implied (up to the
  assembly's roundoff).
* The observational `clamped.gradient_balance` block of full records
  (`all`, `excluding_fully`, `free_dofs`) is unchanged; `reduce.py`
  reconstructs the row split from its norms and cosines, so no new record
  field was needed.
* Augmented-Lagrangian passes solve the full-size problem, where clamped
  DOFs move toward their targets; the free balance still ignores their
  rows (the AL term is not in the provider under either option).
* The Houdini asset does not export the key; nothing it exports or reads
  changes.

## Prototype comparison

### Single-threaded, reproducible

`free` against the option-off `base` binary. Iterations are accepted
Newton iterations; raises are calibration raises (trim increases at a
refresh, endpoint refresh or stall retune without another controller
decision), with those below 1 % in parentheses; the solution difference is
the largest relative L2 difference over the exported steps.

| scene | option off: its / stalls / raises / trim max | `free`: its / stalls / raises / trim max | solution difference |
|---|---|---|---:|
| smoke-qs | 31 / 0 / 0 / 8 | 31 / 0 / 2 (2) / 8 | 1.6·10⁻¹⁶ |
| smoke-tr | 26 / 0 / 0 / 8 | 28 / 0 / 3 (3) / 8 | 2.3·10⁻⁷ |
| smoke-alhess | 31 / 0 / 0 / 8 | 31 / 0 / 2 (2) / 8 | 1.6·10⁻¹⁶ |
| smoke-friction | 62 / 0 / 0 / 8 | 66 / 0 / 6 (6) / 8 | 4.0·10⁻⁹ |
| R1 (3 steps) | 348 / 5 / 1 (1) / 1 | 375 / 7 / 0 / 1 | 1.1·10⁻⁵ |
| BBT (20) | 186 / 0 / 1 (1) / 1 | identical | 0 (byte-identical) |
| IT (200), Accelerate | 6,385 / 23 / 1 / 5.9·10⁴ | 4,339 / 5 / 8 (3) / 8.6·10³ | 15.8 % |
| D1 drop, slab | trim 1 → 0.5 at step 23 | 10 raises (8), trim 1 → 1.07 | 4.0·10⁻⁴ |
| D2 drop, grid | trim 1 → 0.5 | 13 raises (11), ends 0.53 | 1.7·10⁻⁴ |
| D3 drop, clamped block | trim 1 → 0.5 | 15 raises (12), ends 1.09 | 2.4·10⁻⁴ |
| D4 drop, support (control) | 11 raises (8), ends 0.615 | same path | 6.5·10⁻¹⁶ |
| Q1 press, clamped block | trim 2 → 4 | 2 raises (2), same path | 2.2·10⁻¹⁶ |
| Q2 press, support (control) | trim 2 → 4 | identical | 0 |

(The synthetic runs write no attempt stream; their trim paths and gaps come
from `tools/clamped/synthetic_reduce.py`.) The drops' impact gaps are
unchanged (D1's minimum 0.84 d̂ in both modes); at rest D1 sits at 0.92 d̂
with the default's halved trim and 0.94 d̂ with `free`.

**R1: a gate flip at a stall retune.** R1 has no clamped half (κ ratio 1),
but its plate's reactions dilute the cosine. At its fifth stall retune the
free cosine passes the gate (0.165 vs 0.053 on all DOFs) with a balance far
below the trim (10⁻⁵ vs 0.0039). `calibrate_trim` then returns "calibrated"
without raising, which skips the stall's fallback softening (halve the trim
when the band sits above its upper edge); the next stall softens instead.
Result: 2 more stall retunes, 27 more iterations and a final trim twice
the default's (2.4·10⁻⁴ vs 1.2·10⁻⁴). The same holds wherever `free` lifts a diluted cosine over
the gate at a stall: a passing gate with a balance below the trim is a
no-op that still blocks the fallback.

**IT realization ensemble** ([IT reproducibility](it-reproducibility-20260928.md):
judge an IT candidate against deterministic realizations that differ in
their roundoff source). Linear solver Accelerate (the matrix runs), CHOLMOD
and SimplicialLDLT, each with the option off and with `free`
(`it-ensemble/`, `it-ensemble.json`):

| realization | option off: its / stalls / raises / trim max / balance flags pass | `free`: its / stalls / raises (<1 %) / trim max / balance flags pass | off vs free | spread within off | spread within free |
|---|---|---|---:|---:|---:|
| AccelerateLDLT | 6,385 / 23 / 1 / 5.9·10⁴ / 91 | 4,339 / 5 / 8 (3) / 8.6·10³ / 93 | 15.8 % | | |
| CHOLMOD | 3,807 / 1 / 0 / 1.4·10³ / 89 | 4,489 / 7 / 8 (4) / 1.1·10⁴ / 90 | 6.5 % | 19–24 % | 18–33 % |
| SimplicialLDLT | 5,087 / 10 / 0 / 1.3·10⁵ / 99 | 6,002 / 21 / 7 (2) / 1.3·10⁴ / 92 | 20.6 % | | |

Realized gaps (last iteration of each contact step, `gaps.py`): minimum
gap median 0.76–0.82 d̂ in all six runs, p10 0.34–0.67, worst 0.246 d̂ in
both CHOLMOD runs, no step below 0.2 d̂; band rms median 0.93–0.95 in all.
With `free` the trim maximum is confined to 8.6·10³–1.3·10⁴ (option off
1.4·10³–1.3·10⁵); the mean iteration count is 4,943 vs 5,093.

### Multithreaded (R4, BB)

Both scenes have no contact with a clamped half the controller sees
(s_B = 1, κ ratio 1, identical gate decisions); `free` changes their
balance only at roundoff and through s_E ≈ 0.67 (BB) / 0.995 (R4) in the
cosine, which flips no gate decision on either (BB passes at 1 of 5 full
records in both variants, R4 at 9 of 9). They run with all
threads and are not reproducible run to run.

| scene | repeat | option off: its / stalls / AL passes / final trim | `free`: its / stalls / AL passes / final trim | off vs free |
|---|---|---|---|---:|
| BB (1 step) | r1 | 388 / 4 / 0 / 3.9·10⁻³ | 333 / 3 / 0 / 7.8·10⁻³ | 3.8 % |
| | r2 | 432 / 4 / 0 / 4.0·10⁻³ | 365 / 4 / 0 / 3.9·10⁻³ | 1.4 % |
| R4 (1 step) | r1 | 591 / 7 / 1 / 2.9·10⁻⁴ | 563 / 7 / 1 / 3.1·10⁻⁴ | 10.4 % |
| | r2 | 528 / 7 / 1 / 4.9·10⁻⁴ | 567 / 6 / 1 / 2.9·10⁻⁴ | 4.9 % |

Spread within a mode (r1 vs r2): BB 4.0 % off, 3.1 % `free`; R4 8.2 % off,
8.1 % `free`. Cross pairs (off r1 vs `free` r2 and the reverse): BB
3.3–3.8 %, R4 7.9–8.1 %. Every run exited 0 with its step accepted; the
free-contact minimum gap was 0.67–0.72 d̂ (BB) and 0.381–0.388 d̂ (R4) in
every run.

* **R4:** no effect. Iterations overlap (528–591 vs 563–567), the solution
  differences between the modes lie within each mode's own spread, and the
  controller took the same kinds of decisions (7 or 6 stall retunes, one AL
  pass, 11–12 trim decreases).
* **BB:** `free` needed fewer iterations in both repeats (333 and 365 vs
  388 and 432, −14 % and −16 %), with solutions within the modes' own
  spread. Nothing in the balance explains it: BB's balance ratio is 1 and
  its gate decisions are identical in both variants, so `free` differs only
  at roundoff there. Two multithreaded repeats per mode cannot separate this
  from run-to-run variation (the clamped-contacts record saw BB's own
  iterations vary by 343–368 and its modes by 368–386); it is not read as
  an effect.

R4 ran after an attempt that hit the driver's 2,700 s cap at a machine load
of ~140 on 18 cores (other sessions' runs; kept in `matrix/aborted-load/`);
these repeats used a 4 h cap (`sequence.py --timeout`, `queue-threaded2.sh`,
`threaded-compare.json`).

## Checks

| check | result |
|---|---|
| Production defaults unchanged | `gradient_balance_dofs` defaults to `all`; the manifest lists it only when `free` |
| Five public smokes, option off | Pass: 25/25 VTU byte-identical, `run_smoke.py`, `base` vs `free` binary (`smoke-base/`, `smoke-free/`) |
| Scenes, option off | Pass: R1 (4 VTU), BBT (21), IT (201), smoke-qs/-tr/-alhess/-friction (5 each) byte-identical, key absent; R1, BBT, smoke-qs, smoke-tr also with `all` explicit; synthetic D1–D4, Q1, Q2 key absent and `all` vs `base` |
| Unit tests | `[gradient_balance_dofs]` 2 cases (obstacle geometry: κ_free/κ_all = 1.5 and cos 1 vs 1/√1.5, applied by `calibrate_trim`, option-off bit identity, record/manifest, validation); affected selection (the clamped-contacts record's 28 tags plus the new one) 147 cases / 11,226 assertions |
| RB-02 probe | Pass: 270/270 (`rb02/`) |
| HDA scripts | Not run: the asset does not export or read the new key |

## Recommendation

Keep `gradient_balance_dofs: all` as the default. The full-DOF balance is
inconsistent with the reduced solve on every contact that has a clamped
side, and its bias depends on how the obstacle is meshed (9× on the
smokes' two-triangle slab, 1.24× on a fine grid). `free` removes that
dependence and is the balance the solve actually satisfies. But on every
measured scene the controller outcome is the same within noise: the
quasistatic obstacle scenes change at roundoff, the drops and IT gain
calibration raises without a separable change in iterations, stall
retunes, gaps or solution, and the scenes without a clamped side do not
change. The near-zero raises at equilibrium, which `free` extends to
obstacle scenes, also postpone the band's downward step. If the user
wants the consistent balance, adopt `free` together with a minimum
relative raise for the upward-only calibration (for example 1 %). That
second change would also affect scenes without a clamped side, so it
needs its own measurement.

## Decision (user, 2026-09-29)

**`gradient_balance_dofs` keeps `all` as its default**, as recommended.
`free` stays available as an opt-in experiment; the manifest names it when
set. No code change follows from the decision.

## Open

* Minimum relative raise for the upward-only calibration (near-zero raises
  at equilibrium reset the band's downward timer); not prototyped.
* A gate pass with a balance below the trim counts as "calibrated" and
  blocks a stall's fallback softening (R1); unchanged by this item.
* The force-weighted estimate with `free` accepts mostly downward
  estimates; whether its softer barrier (IT's tighter gaps) matters on
  other scenes is not measured.
