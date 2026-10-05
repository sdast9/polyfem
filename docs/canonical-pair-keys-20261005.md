# Canonical pair keys and the history-sensitivity report

Date: 2026-10-05. Status: **implemented and verified** (PolyFEM `448fa8830`
report, `2a60cfa47` canonical keys; publication: see the end). Origin: the
roundoff investigation of 2026-10-04 (parent workspace
`semi-implicit-roundoff-work/FINDINGS.md`, the evidence for this record).

**User decision (2026-10-05):** adopt option 1 of the investigation —
canonical edge-edge keys — **as the default, without a historical switch**
(asked at the start of this work: "No switch"), and option 4 — an
observational sensitivity report in the run manifest. Options 2 (robust
estimates at degenerate geometry), 3 (state-based trim cadence) and 5
(continuation hysteresis) were not chosen and are not implemented. No other
default or model behaviour changed.

## Background

RB-21 keys a semi-implicit coefficient on the parent candidate that built a
collision, `(10 + type, id0, id1)` as the toolkit stores it, i.e. in the order
the broad phase emitted the pair. The LBVH emits an edge-edge pair as (query
leaf, target leaf) in Morton order, and a line search's cached swept
candidates are ordered differently from the static build, so one physical
edge-edge pair (or a pair of codimensional points) could carry two keys. RB-20
force continuation looks a carried coefficient up by key: after a flip it
missed and the contact was silently re-estimated at the current state
(mechanism A of the investigation). That was the whole 3e-4 response of the
1e-15 repro, and in the user's scenes 1.6–4.3 % of the continued keys per step
(R4, pup_push, IT; fresh/continued ratios up to 96) and 12–15 % of the
contacts at every ball-burst stall retune. Face-vertex and edge-vertex keys
are typed and were never affected; RB-21's design checked that parent *lists*
are order-insensitive, not the order inside a pair (correction note in
[rb-21-parent-keyed-kappa.md](rb-21-parent-keyed-kappa.md)).

## Change 1 — history-sensitivity report (observational, `448fa8830`)

Every semi-implicit step record of the run manifest gains
`steps[].contact.history_sensitivity`; the field reference is the table in
[scenes/semi-implicit/README.md](../scenes/semi-implicit/README.md#history-sensitivity-stepscontacthistory_sensitivity-2026-10-05).
In short:

* **continuation** — a coefficient key given a fresh estimate (memo miss) at
  an accepted Newton iterate or a mid-solve refresh is a *loss* when its pair
  held a value continuation carried at a capture of this step or the previous
  one. The captures are every published-endpoint refresh: between steps, at
  each augmented-Lagrangian pass start, at the reduced solve's start and at
  each lagging iteration; the two windows (this step's, the previous step's)
  shift at `update_quantities`. Causes: `orientation` (the pair was carried
  under its other order), `reentry` (the key itself was carried and a later
  capture dropped it: it was not active at the last endpoint and came back),
  `other` (active at a capture with a coefficient continuation could not
  carry). Each key counts once per record, with the largest
  |ln(fresh/carried)|.
  `split_pairs` counts edge-edge / vertex-vertex pairs memoized under both
  orders at an accepted iterate or refresh, carried or not (one contact, two
  estimates). Orientation losses and split pairs are 0 with canonical keys:
  together they are the regression guard.
* **trim** — start/end, count and net log2 change per source (collapse,
  calibration, conditioning cap, cadence step, refresh step, stall softening,
  force band, initial estimate, other); the log2 values add up to
  log2(end/start). Also the moves made inside stall retunes, and
  `iters_since_trim` against `controller_interval` (how close mechanism C's
  30-iteration cadence is to firing).
* **gap shifts** (units of d̂, at the accepted endpoint, fully clamped
  contacts excluded). The investigation's barrier-law estimate
  Δd ≈ ½(d̂ − d)·|Δln κ| (*Equilibria, not tolerance*) is evaluated twice. For
  the contacts whose coefficient contains a lost key, Δln κ is the collision
  coefficient against the same coefficient with the lost keys at their carried
  values. For a cadence step, Δln κ = ln(trim_factor) over all active contacts:
  how far one more or one fewer cadence step would move the equilibrium.
* `restored_from_state` flags a record whose accumulation began at a resumed
  state.

The accumulators are attempt state (`BarrierContactForm::State`, rolled back
with the form; the RB-06 fingerprint is unchanged) and start again after each
step record. On a resume the current window is rebuilt from the restored
endpoint, which is exact (a state file is written right after the
between-steps capture); the previous step's window is not saved, so the
first resumed record can miss a re-entry, and says `restored_from_state`. The
same-x energy check of the investigation's prototype (an extra barrier
evaluation per rebuild) was left out as too costly. Detection runs in
`post_step` and at the end of mid-solve refreshes: one `coefficient_keys`
pass and two map lookups per active key; measured below at +0.6 % of the
solve time on R1, within run-to-run noise.
Non-semi-implicit forms report `{value: null, unavailable_reason}`. The
manifest schema stays at version 1: a block was added, no field changed
meaning (`io/DiagnosticSchemas.hpp` says so).

## Change 2 — canonical pair keys (moves results, `2a60cfa47`)

* `BarrierContactForm::canonical_key`: a key whose two primitives have the
  same type — edge-edge or vertex-vertex parent (tags 12, 10), edge-edge or
  vertex-vertex stencil (tags 2, 0; an edge-edge stencil by its edges' vertex
  pairs) — is the smaller of the key and its reverse, i.e. the sorted pair.
  Typed keys are unchanged. `coefficient_keys` and `stencil_key` return
  canonical keys. One identity per pair therefore holds for continuation, the
  memo and the batch statistics, `coefficient_identity: "stencil"`, EF-07's
  birth tracking (`current_stencil_keys`, `previous_iterate_keys_`) and the
  contact-path diagnostic's signatures.
* `memoized_stiffness` rebuilds an edge-edge / vertex-vertex parent in sorted
  order whichever order the build emitted. The estimate is symmetric up to
  roundoff (a degenerate parallel pair may tie-break differently — mechanism
  B, not addressed).
* **Restart.** Layouts 1–2 store keys as emitted, and a table may hold both
  orders of one pair. On read every key is canonicalized; of two entries for
  one pair, the one stored under a continued or endpoint key wins (the value
  that acted at the saved endpoint), otherwise the one stored in sorted order
  (the orientation this binary estimates in). The rule is independent of row
  order, merges are logged, and the layout version is unchanged.
* The manifest's coefficient law names it
  (`coefficient_law.pair_key_orientation: canonical`, version string
  `…, canonical pair keys (2026-10-05)`).
* No historical switch (user decision): runs with orientation flips or split
  identities made before `2a60cfa47` are reproduced with an earlier binary
  (for example `49ad24b74`).

## Verification

All evidence is in the parent workspace's `canonical-keys-work/` (isolated
worktrees of PolyFEM, the IPC Toolkit `1f1b5dbf` and PolySolve `43ca2e6`;
binaries and hashes in `bin/`). "Production" below is `448fa8830`, that is
49ad24b74 plus the report; "canonical" is `2a60cfa47`. The two commits were
first built in an earlier form (the report without split pairs and with a
one-step window); every run of that form was redone with the final binaries,
and every single-threaded output is byte-identical between the two forms.

| Check | Result |
| --- | --- |
| New unit cases | `[kappa_continuity][history]` (re-entry with its ratio and gap shift; orientation, split and other classifier; trim accounting adding up; failed-attempt and no-cadence nulls; rollback and restart state), `[run_manifest][history]` (every field on the public quasistatic smoke, trim accounting closes across records), `[kappa_continuity][canonical]` (canonical_key; an edge-edge pair under parent and stencil identity and a vertex-vertex pair built in both orders keep their carried coefficient bit for bit; restart round trip with keys stored in the other order; the both-orders rule in both row orders) — 9 cases / 362 assertions at `448fa8830`, 11 cases / 444 assertions at `2a60cfa47` |
| Affected selection (28 tags: kappa_continuity, history, canonical, restart, rollback, trim_controller, trim_band, al_budget, fully_prescribed, output_kinematics, run_manifest, semi_implicit_coefficients, contact_stiffness_mapping, friction_lag, contact_cache(_parallel), coefficient_events, physical_diagnostics, form, direction_filter, contact_floor_retired, resource_containment, input_validation, clamped_contacts, gradient_balance_dofs, trim_predictors, stall_restart, matrix_io) | all pass: 158 cases / 13,066 assertions at `448fa8830`, 160 cases / 13,148 assertions at `2a60cfa47` |
| Five public smokes, one thread, against the 49ad24b74 binary | byte-identical at both commits (55 files, 25 VTUs) |
| Report is observational | production reproduces the investigation's 49ad24b74 runs byte for byte: repro pair (18 VTUs each), R1 (8 files), BBT (42 files), pup_push 3 steps (8 files), IT 200 steps (402 files); a failed ball-burst step's rollback verifies its fingerprint with the new attempt state |
| Restart conversion on a real pre-canonical state file | ball-burst `state_30` (layout 1): 1,644 both-order entries merged, 5,870 continued coefficients restored, no orientation loss in the resumed steps |
| Canonical keys equal the investigation's experimental switch | repro pair, pup_push and IT 200 steps byte-identical to its `POLYFEM_SI_CANON` runs |
| RB-02 probe (`tools/rb02`) | **270/270** (library built from the canonical-keys sources; rerun on every form of the change) |
| CTest `cli_contract` (`tools/rb12/cli_check.py`: build identity, a public smoke with its manifest, resource and refused-input exits) | 5 checks pass on `2a60cfa47` |
| Cost of the report (`tools/cost.sh`: 49ad24b74 and `448fa8830` interleaved, single-threaded, three repeats) | R1 (3 steps, 194 Newton iterations): solve wall 42.3 s against 42.6 s (+0.6 %, inside the ±0.4 s spread of the repeats); repro: 1.79 s both; outputs byte-identical |
| Repro sweep (`tools/sweep.py`, the investigation's deltas) | below; identical to the investigation's table to the printed digits |

Repro scene (unit cube pushed by a rotating plate, d̂ = 0.01), max body
difference over steps 1–8 against the unperturbed run, rotation centre moved
by δ:

| δ | production | canonical keys |
| --- | --- | --- |
| 1e-15 | 3.1e-4 | 6.9e-9 |
| −1e-15 | 3.1e-4 | 8.9e-9 |
| 1e-14 | 8.9e-6 | 6.9e-9 |
| 1e-13 | 4.5e-5 | 6.9e-9 |
| 1e-12 | 3.8e-4 | 6.9e-9 |
| 1e-11 | 3.1e-4 | 6.9e-9 |
| 1e-10 | 3.1e-4 | 6.9e-9 |
| 1e-9 | 4.5e-5 | 7.4e-9 |
| 1e-8 | 3.1e-4 | 1.2e-8 |
| 1e-7 | 3.1e-4 | 6.1e-8 |
| 1e-6 | 4.5e-5 | 4.2e-7 |

The report on the repro (production, unperturbed): orientation losses 0, 0,
0, 0, 0, 6, 12, 6 and split pairs 0, 0, 0, 0, 8, 6, 6, 12 at steps 1–8, the
largest ratio ×1.67, estimated gap shifts up to 0.016 d̂ (the investigation
measured up to 0.018 d̂ between branches); over the 12 runs of the sweep, 162
orientation losses and 299 split pairs in production and none with canonical
keys, where every count is 0 at every step. The sweep was run with both forms
of the binaries: all 432 VTUs are byte-identical between them. The 7e-9
floor that remains is mechanism B, outside this decision.

## What moved: real scenes

Production (P, `448fa8830`) against canonical keys (C, `2a60cfa47`) on the
user's scenes as the investigation ran them (`tools/realrun.py`: the scene
files with Newton and small volume output; R4's scene file opts into the
initial trim estimate, D2). Single-threaded runs are bit-reproducible, and
each was repeated with the binaries' first form with byte-identical outputs;
the threaded R4 runs are not reproducible run to run, and their two repeats
per mode give its spread. Counts are the history report summed over the
steps; "ratio" is the largest fresh/carried coefficient ratio, "gap" the
largest single-contact gap-shift estimate of a step. The distance d is the
investigation's ‖u_P − u_C‖ / mean‖u‖ on the body.

| scene (steps, threads) | orientation losses P → C | split pairs P → C | re-entries P / C | ratio P / C | gap (d̂) P / C | Newton its (stall retunes) P → C | P vs C | roundoff spread for comparison |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| IT (200, 1) | 1,165 (57 steps) → 0 | 7,297 (73 steps) → 0 | 343 / 423 | ×22.6 / ×30.6 | 0.088 / 0.115 | 4,291 (23) → 3,721 (4) | identical to step 10; from contact onset mean d 0.039, max 0.088 | three production linear-solver realizations: mean d 0.102; a 1e-15 obstacle nudge: 0.222 (investigation runs) |
| pup_push (3, 1) | 55 (steps 2–3) → 0 | 1,104 (1,020 in step 1) → 0 | 0 / 2 | ×2.08 / ×1.007 | 0.0075 / 0 | 129 (18) → 111 (17) | d 1.6e-4, 7.4e-4, 2.7e-4 at steps 1–3 | 1e-15 nudge: 1.3e-4, 1.6e-4, 2.6e-4 (P), 1.5e-4, 6.4e-4, 5.6e-4 (C) |
| R4 (5, 6) | 791 (steps 2–5) → 0 | 1,616 (all 5 steps) → 0 | 118 / 99 | ×2.28 / ×1.84 | 0.082 / 0.015 | two repeats each: 266, 389 → 332, 251 | d 0.036–0.069 per step | two repeats of one binary (threads): d 0.034–0.071 (P), 0.017–0.047 (C) |
| R1 (3, 1) | 0 → 0 | 4 (steps 1, 3) → 0 | 1 / 0 | ×1.49 / – | 0 / 0 | 194 (5) → 189 (5) | max 7.2e-7, d ≤ 8.5e-5 | 1e-15 nudge: max 5.5e-6, d up to 4.9e-4 in either mode |
| BBT (20, 1) | 0 → 0 | 0 → 0 | 0 / 0 | – | 0 / 0 | 186 (0) → 186 (0) | max 7.7e-16 | — |
| ball-burst (steps 31–32 resumed from `state_30`, 8) | 2,618 in step 31 (2,906 more in the failed step 32) → 0 | 3,814 (4,100) → 0 | 0 (175) / 188 | ×2.59 / ×1.54 | 0.081 / 0.021 | stall retunes 2, then 20 and a failed step 32 → 1, 2 | — (P stopped at step 32) | the investigation's production resume of the same computation completed step 32 with 3 retunes |
| five public smokes (4, 1) | 0 → 0 | 0 → 0 | 0 / 0 | – | 0 / 0 | unchanged | byte-identical | — |

Read:

* **IT** is chaotic at contact onset (step 12) under any perturbation. With
  canonical keys it stays inside production's own realization spread; on
  this realization it took 13 % fewer Newton iterations and 4 instead of 23
  stall retunes. The investigation's four-nudge ensembles showed no
  systematic cost difference (3,566–4,792 against 3,706–4,751 iterations),
  so no cost claim is made. The re-entry losses (423 with canonical keys,
  ratios up to ×30.6, gap shifts up to 0.115 d̂) are contacts that left the
  active set and returned. They are the history dependence option 5
  (hysteresis) would address; it was not chosen, and the report now shows it.
* **pup_push** already differs at step 1, where production has split pair
  identities (the same contact estimated in both orders) but no continuation
  loss. Its step 1 sits at 17–18 of 20 stall restarts in both modes.
  Production against canonical is of the same order as a 1e-15 nudge in
  either mode (step 2: 7.4e-4, against 6.4e-4 for a canonical and 1.6e-4 for
  a production nudge); one nudge pair per mode allows no finer statement.
* **R1** diverges at step 1, where production has 3 split pair identities
  and no continuation loss (one re-entry follows at step 3). It moves by less
  than a 1e-15 nudge moves it.
* **R4** runs threaded, and two runs of the same binary already differ by
  as much as production and canonical keys do. Its 791 orientation losses
  (distinct keys per step; the investigation counted 48–100 per event) go to
  0, and its largest estimated gap shift drops from 0.082 d̂ to 0.015 d̂, the
  remainder from re-entries.
* **ball-burst** resumes from a state file written before canonical keys
  (EF-07 R0 `state_30`, restart layout 1). Canonical keys convert it on read:
  1,644 entries stored under both orders of a pair were merged, the carried
  one kept, and the resumed step has no orientation loss. Production, reading
  the same file with emission-order keys, loses continuation on 2,618 keys in
  its first resumed step (ratios up to ×2.59, estimated gap shifts up to
  0.081 d̂ over 693 contacts) and splits 3,814 pairs. On this threaded
  realization production's step 32 then failed after its 20 stall restarts,
  each repeating the same crawl. The failed attempt was rolled back with a
  verified fingerprint, which exercises the new attempt state on a real
  scene. The investigation's production resume of the same computation
  completed that step with 3 retunes; one run per mode says nothing about
  robustness either way. Canonical keys completed both steps (1 and 2
  retunes; 188 re-entries in step 32).
* **BBT** differs at roundoff only: an edge-edge estimate is now taken in
  sorted order.

## Limits

* The report counts what it can attribute at accepted iterates and mid-solve
  refreshes. A fresh estimate made only in a line-search trial build that
  never becomes active at an accepted iterate is not a loss. The trial's
  objective was still affected — the investigation's two-valued objective —
  and with canonical keys it no longer occurs. The lookback spans this step
  and the previous one: a contact absent for a whole step more comes back as a
  new contact, not a re-entry.
* The gap shifts are first-order single-contact estimates, within a factor
  2–3 of the measured branch differences on the repro. Contacts stamped at a
  degenerate birth geometry (mechanism B) and the iteration-count dependence
  of the cadence itself (mechanism C) are only reported, not measured: the
  report shows how close the cadence is and what one step would do.
* Canonical keys remove mechanism A only. B (7e-9 on the repro, 2.5e-4 at
  d̂ = 0.02) and C remain. Chaotic scenes stay chaotic: the investigation's IT
  nudge ensembles and pup_push's step-1 stall marginality do not depend on
  key orientation.
* Not run: the full unit suite (the affected selection was); the Houdini
  asset's tests (the asset is unchanged and drives the shared `polyfem/build`,
  which is not rebuilt by this work); the native CI (runs on push).
  Surfacing the report in the Read PVD asset is a follow-up.

## Evidence

Parent workspace `canonical-keys-work/`: `bin/` (binaries, `hashes.txt`),
`runs/smokes/`, `runs/repro/` and `runs/repro-sweep/` (with
`summary.json`), `runs/real/` (scene runs, `scene-table.json`), `runs/bb/`,
`runs/rb02/`, `runs/unit/`, `tools/` (adapted from the investigation's:
`smokes.sh`, `cmpdirs.sh`, `runcase.py`, `sweep.py`, `realrun.py`,
`run_bb.sh`, `reportsum.py`, `scenecmp.py`, `scene_table.py`).

## Publication

Local commits on branch `canonical-pair-keys` of the isolated worktree, on top
of `49ad24b74`: `448fa8830` (report), `2a60cfa47` (canonical keys) and the
records. **Not pushed** (user, 2026-10-05: "not yet"; no Build was running at
the time). When it is pushed, the shared `polyfem/` checkout is to be
fast-forwarded and `polyfem/build` rebuilt at the new head in the same step
(user's choice), since the Houdini assets run that binary.
