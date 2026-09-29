# Contacts between Dirichlet-clamped primitives: measurement and opt-in exclusion

Date: 2026-09-28/29. Status: **measured; two opt-in prototypes
(`semi_implicit/clamped_contacts`), default `keep` unchanged.** Adopting a
default is the user's decision.
Origin: [EF-07](ef-07-trim-loop.md), point 3a and its *Open* list.

## Question

PolyFEM builds the forward solve's collision mesh from the whole boundary
surface (`NonlinearElasticVarForm::build_collision_mesh`) and never sets IPC's
`CollisionMesh::can_collide`, so every primitive may collide with every other;
only remeshing (`LocalRelaxationData.cpp`) and shape optimization
(`BarrierForms.cpp`) install a filter. Contacts between two clamped primitives,
or between a clamped and a free one, are therefore built and evaluated. In the
reduced solve their forces on clamped DOFs drop out, but they enter the
semi-implicit trim controller's statistics. How many such contacts are there
on the EF-01 matrix scenes, IT and ball-burst? How much do they shift the
controller's statistics? And what does excluding them cost or change, either
from the statistics only or from the collision set?

## Answer in brief

* **They are rare, and none ever set a controller decision.** Fully clamped
  contacts (every stencil vertex's displacement prescribed) occur only on R4
  (≤ 2 of ~1,300–2,100 active), BB and the ball-burst step-31 state (≤ 3 of
  ~1,000 / ~4,000; the one identified is knit-rim self-contact). R1, BBT,
  IT and the smokes have none. In the keep runs the minimum-gap pair was never fully clamped, no
  collapse decision and no calibration-gate decision would change, and no
  refresh saw only clamped contacts (no masked first contact). The largest
  statistic shifts are the BB coefficient batch median (≤ 5.1 %, at the first
  refresh with few contacts), R4's gradient-balance trim (≤ 2.2 %), and band
  rms / force-weighted gap ≤ 0.7 %; on the ball-burst step-31 state every
  shift is ≤ 0.03 %.
* **Partly clamped contacts are common where a free body touches a clamped
  one or an obstacle.** On IT (sphere vs obstacle) up to 628 are active, and
  on the smokes about 50. Both prototypes leave them alone by definition.
* **Two opt-in prototypes, default off.** `clamped_contacts:
  exclude_statistics` keeps fully clamped collisions in the energy and CCD
  but hides them from every controller statistic. `exclude_collisions` drops
  candidates whose primitives are all clamped, through IPC's `can_collide`.
  With the option off, the five public smokes, R1, BBT and IT are
  byte-identical to `main`. Both options are byte-identical to `keep` on
  every scene without a fully clamped contact. On R4, BB and ball-burst
  (multithreaded, not reproducible), every mode completed every step, with
  iterations within about 10 % of keep's own range (R4 524–565 vs 566–586,
  BB 368–386 vs 343–368, ball-burst 459–556 vs 440–529 for two steps; no
  consistent direction between scenes), and the same trim decisions on
  ball-burst. Solution differences to keep are within keep's own
  spread except one `exclude_statistics` R4 run: 9.2–10.8 % against a
  4.5–7.3 % keep spread; its repeat is 4.1–7.9 %.
* **Recommendation: keep the default (`keep`).** There is no measured
  benefit, and neither exclusion is needed for correctness on these scenes.
  If one is wanted as a guard, `exclude_statistics` is the safe choice: it
  changes nothing the solver integrates. `exclude_collisions` also removes the
  pair's barrier and CCD, and that is unsafe wherever clamped DOFs move
  (augmented-Lagrangian passes, prescribed bodies approaching each other).
* **Separate finding (not prototyped): the gradient balance counts Dirichlet
  rows.** `calibrate_trim` / the initial estimate use the full-DOF barrier
  and energy gradients. On IT the obstacle half of every sphere–obstacle
  contact force is barrier gradient on clamped DOFs. Restricted to free DOFs,
  the balance trim is 2.06–2.15× larger (median), the cosine gate passes at
  18–20 more endpoint refreshes, and the trim would be raised at 16–19
  records instead of 5. R4, BB, R1 and BBT: ≤ 2.2 % in the trim; the gate
  (a cosine) flipped only at 1–2 R1 stall retunes. This
  concerns partly clamped contacts and the reaction rows, not the fully
  clamped ones this item is about; it is listed under *Open*.

## Definitions and implementation

**Clamped vertex.** A collision-mesh vertex is clamped when every node of its
displacement-map row has all `dim` components in the reduced solve's
Dirichlet DOF list (`boundary_nodes`, which includes obstacle nodes). A row
without entries cannot move either. Interpolated rows (high-order proxies)
are clamped only when all their parents are.
`BarrierContactForm::clamped_collision_vertices`; nodes with only some
components prescribed (for example the ball's `[true, false, true]` in R1/BBT)
are free.

**Collision classes** (`clamp_class`, by the collision's stencil vertices;
analytic planes count as clamped): free (no clamped vertex), partly, fully
(every vertex clamped). A fully clamped collision exerts no force on a free
DOF of the reduced solve, and the trim cannot move it.

**Plumbing.** `SolveData::init_forms` passes the Dirichlet DOFs to the barrier
form in semi-implicit mode (`set_dirichlet_dofs`). With `keep` this is
observational only.

**Option** `/solver/contact/semi_implicit/clamped_contacts` (spec entry,
invalid values are a named error, the manifest lists a non-default mode under
`coefficient_law.controller.clamped_contacts`):

| value | collision set, energy, CCD | trim controller |
|---|---|---|
| `keep` (default) | all candidates, as before | all collisions, as before |
| `exclude_statistics` | unchanged | ignores fully clamped collisions: band statistic, collapse minimum (with and without `collapse_exclude_born`), force-weighted gap, coefficient batch median/floor/cap (their coefficients are still resolved against it), gradient balance of `calibrate_trim` and the initial estimate (IPC's assembly over a filtered copy of the set), first-contact detection and the mid-solve contact-birth refresh |
| `exclude_collisions` | `NonlinearElasticVarForm::init_forms` sets `collision_mesh.can_collide &= clamped_collision_filter`: a candidate is kept when any vertex pair of its two primitives has a free vertex, so only candidates whose primitives are all clamped are dropped (the friction, adhesion and intersection checks use the same mesh) | unchanged; a collision whose stencil is fully clamped but whose parent primitives are not (an edge–edge candidate resolving to a clamped vertex–vertex pair) can remain |

**Energy.** In a reduced solve a fully clamped collision's barrier energy is
constant within the step, so `exclude_collisions` shifts the objective by a
per-step constant. In augmented-Lagrangian passes (`ALSolver::solve_al` uses
the full-size problem) clamped DOFs do move. There `exclude_collisions`
removes both the barrier and the CCD protection between clamped primitives,
so a prescribed body could be driven through a clamped one, and the snap
gate's CCD check no longer sees such pairs. `exclude_statistics` is also
inexact in AL passes, since the controller ignores contacts that move, but it
integrates the same energy. R4 (one grip fixed, the other prescribed) ran one
AL pass per step in every mode and completed; its fully clamped contacts were
not identified individually.

**Observational record.** When the trim-predictor stream is on
(`output/trim_predictors`), every record has a `clamped` block. It holds the
collision counts per class (in the set and active), the minimum pair's class,
and the controller statistics over all collisions and without the fully
clamped ones: band rms, minimum gap, collapse proxy and decision,
force-weighted mean gap. Full records add the coefficient batch (median /
cap / floor with and without the fully clamped keys) and the gradient
balance over all DOFs, without fully clamped collisions, and on free DOFs
only. Without a Dirichlet list the block is absent (missing, not zero).

**Implementation error found and repaired during the work.** The first build
assembled the filtered barrier gradient with a serial scatter
(`local_gradient_to_global_gradient`) instead of IPC's assembly. Its roundoff
differed, so `exclude_statistics` moved the trajectory on R1 and BBT, which
have no clamped contact at all. Those runs were discarded (kept in
`fixed-contacts-work/matrix/tainted-scatter-gradient/`). The repair routes
both branches through `BarrierPotential::gradient`: with nothing skipped the
production call itself, otherwise the same call on a filtered copy of the
set. A unit check and the byte-identity results below cover it.

## Evidence and method

Evidence: `fable_polyfem/fixed-contacts-work/` (outside `outputs/`, which is
being archived). Binaries and hashes are in `bin/hashes.txt`:

* `clamped` (first build, on `1cb1efc25`): keep and `exclude_collisions`
  runs in `matrix/`.
* `clamped2` (the repaired gradient, same base): `exclude_statistics`, repeats
  and the single-threaded `exclude_collisions` runs in `matrix2/`.
* `clamped3` (the published sources, rebased on `49d7753c0`, which caps
  Accelerate's threads so single-threaded runs reproduce,
  [it-reproducibility-20260928.md](it-reproducibility-20260928.md)) and
  `base3` (clean `49d7753c0`): the byte-identity matrix in `matrix3/`,
  `matrix3-base/` and `smoke3/`.

The published commit sits on `ec5d42862`; its two commits beyond `49d7753c0`
change only IPC's smooth-contact (GCP) collision builder, which semi-implicit
mode refuses, and documentation, so they were not re-measured.

The keep path is the same code in `clamped`, `clamped2` and `clamped3`. They
differ only in the excluded-mode gradient and the record's
`excluding_fully` gradient; `matrix-all/` links the valid runs of both
matrices.

Scenes: the EF-01 matrix on the EF-02 driver (`tools/clamped/sequence.py`
over `tools/ef02/run.py`): R4 one step, R1 three steps, BBT 20, IT 200, BB
one step, and the five public smokes. Added to these is the EF-07 ball-burst
step-31 state (R0's `state_30`, steps 31–32, `tools/clamped/bb_resume.py`).
Physical diagnostics' coefficient events go to /dev/null on the matrix and
are off on ball-burst. **Every run states the controller explicitly
(`band_statistic: rms`, `initial_trim_estimate: false`)**, because the R4
scene file selects the force-weighted mode since its 2026-09-26 re-export
([r4-production-speedup-20260928.md](r4-production-speedup-20260928.md)).
R4, BB and ball-burst use all threads and are not reproducible run to run.
The `clamped`/`clamped2` single-threaded runs predate the Accelerate cap and
are not reproducible either; `matrix3` is.
`tools/clamped/reduce.py` summarizes the `clamped` blocks,
`tools/clamped/spread.py` and `compare.py` the modes.

## Measurement (mode `keep`)

Shifts are the largest |statistic without fully clamped / statistic with all
− 1| over the records with a fully clamped active contact. "—" means no fully
clamped contact, so no shift.

| scene | clamped vertices | records | active free p50 / max | partly max | fully max | min pair fully clamped | band rms shift | force-weighted gap shift | batch median shift | kappa_gb shift (excl. fully) | collapse flips |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| R1 (3 steps) | 1182 | 388 | 35 / 93 | 0 | 0 | 0 | — | — | — | — | 0 |
| BBT (20) | 1182 | 207 | 13 / 29 | 0 | 0 | 0 | — | — | — | — | 0 |
| IT (200) r1 | 400 | 5678 | 0 / 119 | 599 | 0 | 0 | — | — | — | — | 0 |
| IT (200) r2 | 400 | 6625 | 0 / 65 | 628 | 0 | 0 | — | — | — | — | 0 |
| R4 (1) | 2659 | 588 | 1330.5 / 2082 | 0 | 2 | 0 | 0.44 % | 0.69 % | 0.23 % | 2.2 % | 0 |
| BB (1) | 16230 | 344 | 975 / 1337 | 1 | 3 | 0 | 0.65 % | 0.3 % | 5.1 % | 0.073 % | 0 |
| ball-burst steps 31–32 | 16230 | 537 | 4042 / 4427 | 1 | 3 | 0 | 0.0011 % | 0.013 % | 0.025 % | 0.0098 % | 0 |
| smoke-qs | 29 | 36 | 0 / 0 | 51 | 0 | 0 | — | — | — | — | 0 |
| smoke-tr | 29 | 31 | 0 / 0 | 51 | 0 | 0 | — | — | — | — | 0 |
| smoke-alhess | 29 | 36 | 0 / 0 | 51 | 0 | 0 | — | — | — | — | 0 |
| smoke-friction | 29 | 67 | 0 / 0 | 52 | 0 | 0 | — | — | — | — | 0 |

* **Where they are.** The record does not list the fully clamped pairs'
  ids; one was identified: the pair that briefly set the minimum gap in the
  `exclude_statistics` BB run (2 of 369 records, gap 0.73 d̂, far above the
  0.0707 d̂ collapse pair threshold) has all four nodes in the knit rim's
  zero-displacement set (EF-07 `bc-probe`). BB's ball is prescribed in all
  components, so ball–ball and ball–rim pairs would also be fully clamped; R4's
  clamped vertices are its two grip sidesets.
* **Share.** Fully clamped contacts are 0.07–0.3 % of the active set in
  steady contact. The larger shares (R4 up to 12 %, BB 6.7 %) occur only in
  the first records of a step, when a handful of contacts are active.
* **Decisions.** Without the fully clamped contacts, no collapse decision
  and no calibration-gate decision flips on any scene; no refresh had only
  fully clamped contacts active.

## Prototype comparison

Iterations are accepted Newton iterations over the run; solution difference
is the largest relative L2 difference over steps. `keep` spread is keep's own
run-to-run difference.

| scene | keep: iterations | keep: own spread | exclude_statistics: iterations | diff. to keep | exclude_collisions: iterations | diff. to keep |
|---|---|---:|---|---:|---|---:|
| BB (1 step) | 343 / 368 | 2.5 % | 368 | 2.7 % | 386 | 3 % |
| R4 (1 step) | 586 / 567 / 566 | 7.3 % | 565 / 539 | 11 % | 562 / 524 | 7.8 % |
| ball-burst steps 31+32 (resume) | 290+239 / 232+208 | no VTU | 266+290 | no VTU | 261+198 | no VTU |

R4 pairwise (relative L2 of the step-1 solution): keep–keep 4.5 / 6.6 / 7.3 %;
`exclude_statistics` r1 9.2–10.8 %, r2 4.1–7.9 %; `exclude_collisions`
6.1–7.8 %. BB: keep–keep 2.5 %, the modes 2.0–3.0 %. Ball-burst writes no VTU; its
trim decisions are compared instead: one trim move in step 31 (two in one keep
repeat and in `exclude_collisions`), none in step 32, final trim 0.03835 in
every run. Every run exited 0 with all steps accepted; `physical_balance_pass`
was false in every R4/BB run, in every mode.

Byte identity on the reproducible single-threaded path (`clamped3` and
`base3`, both with the Accelerate thread cap; every exported VTU file compared
with the clean `main` binary, option absent):

| scene | VTU files | `keep` | `exclude_statistics` | `exclude_collisions` | `keep` repeat |
|---|---:|---|---|---|---|
| R1 (3 steps) | 4 | identical | identical | identical | identical |
| BBT (20 steps) | 21 | identical | identical | identical | identical |
| IT (200 steps) | 201 | identical | identical | identical | — |
| five smokes (4 steps each) | 5 each | identical | identical | identical | — |

The earlier `clamped`/`clamped2` single-threaded runs of R1, BBT and IT
(before the Accelerate cap) differ run to run in every mode (IT 3,819–9,613
iterations for the same controller behaviour); they are kept in
`matrix2/` but carry no mode comparison.

Reading:

* **Option off is production.** The five public smokes (25 VTU files via
  `tools/smoke/run_smoke.py`) are byte-identical between `main`
  (`49d7753c0`) and the published sources, as are R1, BBT and IT through
  the matrix driver.
* **Without a fully clamped contact both options are exact no-ops.** They
  are byte-identical to `keep` on R1, BBT, IT and the smokes (IT's obstacle
  is entirely clamped, but it never had a collision with itself within d̂).
* **With fully clamped contacts (R4, BB, ball-burst)** every mode completed
  every step, with iterations within about 10 % of keep's own range and no
  consistent direction (`exclude_collisions` fewer on R4, more on BB). These runs
  are multithreaded, so each is one roundoff realization of a contact
  trajectory. R4 is trajectory-sensitive: EF-01 measured identical settings
  differing by 3.6–8.7 %, and keep differs from itself by up to 7.3 % here.
  One `exclude_statistics` R4 run lies 2–3 points beyond that spread and its
  repeat does not. With ≤ 2 of ~1,500 contacts excluded (band statistic
  ≤ 0.44 %, calibration trim ≤ 2.2 %), the samples do not separate an option
  effect from the spread. On the ball-burst state the controller took the
  same decisions in every mode.

## Checks

| check | result |
|---|---|
| Production defaults unchanged | `clamped_contacts` defaults to `keep`; manifests list the option only when it is not `keep` |
| Five public smokes byte-identical, option off | Pass: 25/25 VTU vs `1cb1efc25` (`smoke/`) and vs `49d7753c0` (`smoke3/`) |
| Scenes byte-identical, option off | Pass: R1, BBT, IT, smokes through the matrix driver vs `49d7753c0` (`matrix3-base/` vs `matrix3/`) |
| Unit tests | `[clamped_contacts]` 6 cases (clamped-vertex map incl. interpolated rows, classes, trim-predictor block, controller exclusion and exact no-op, validation/manifest, `can_collide` filter); affected selection (28 tags: controller, trim predictors, contact forms, rollback, restart, manifest, AL, input validation, form derivatives) 145 cases / 11,151 assertions on the published sources (144 / 11,127 on the first build) |
| RB-02 probe | Pass: 270/270 on the first build (`rb02/`) and on the published sources (`rb02-3/`) |
| HDA scripts | Not run: nothing the Houdini asset exports or reads changes (a new optional spec key, not exported by the asset) |

## Recommendation

Keep `clamped_contacts: keep` as the default. On every measured scene,
contacts between clamped primitives are too few to move a controller
decision, and neither exclusion changed cost or accuracy beyond run-to-run
noise. If the user wants the guard EF-07 point 3a described (a clamped pair
must never be allowed to set the collapse minimum), `exclude_statistics` is
the candidate: it is an exact no-op when no fully clamped contact exists, and
it leaves the energy and CCD untouched, including in AL passes.
`exclude_collisions` is not recommended as a default: its only extra effect is
removing barrier/CCD work for pairs that are few here, and it removes
protection wherever clamped DOFs move. Adopting either is the user's
decision.

## Open

* The gradient balance (`calibrate_trim`, the initial estimate and the
  trim-predictor `gradient_balance`) uses full-DOF gradients, so Dirichlet
  reactions and the clamped half of partly clamped contacts enter it. On IT
  the free-DOF balance trim is ~2.1× the full-DOF one, and the cosine gate
  would pass at 18–20 more refreshes. Restricting the balance to free DOFs
  is a separate controller change, not measured beyond this record.
* `tools/ef02/sequence.py` production mode still inherits controller options
  from re-exported scene files; `tools/clamped/sequence.py` pins them.
