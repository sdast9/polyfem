# Guarded first-contact refresh (*birth0g*) adopted

Date: 2026-10-06. Status: **adopted as production behaviour, no historical
setting** (user decisions of 2026-10-05 and 2026-10-06). Local branch
`adopt-birth0g`, on top of the canonical pair keys (`a7d39fc7b`) and the
mechanisms B/C record (`435c08a63`); **not pushed** (see *Publication*).
Measurement that led here: [semi-implicit-mechanisms-b-c-20261005.md](semi-implicit-mechanisms-b-c-20261005.md),
sections *B — the repro's floor and the prototypes* and *What adopting birth0g
involves*; the full record is the parent workspace's
`mechanism-bc-work/FINDINGS.md`.

## Decisions

* **2026-10-05**, after mechanisms B and C were measured: adopt the guarded
  first-contact refresh (*birth0g*) as production behaviour; keep the
  30-iteration in-solve trim cadence (C) as it is; do not repair stale
  first-estimate stamps (B); keep the probe branch `mechanism-bc-probe`
  (local, not for `main`).
* **2026-10-06**: **no historical setting** restores the old first-contact
  behaviour (as for canonical pair keys; earlier runs reproduce with an earlier
  binary). For the moving-obstacle check: the ball-burst scene with the ball
  head turned into an obstacle.

## What changed

Both parts are in `BarrierContactForm` (`src/polyfem/solver/forms/BarrierContactForm.cpp`)
and re-implement the measured prototype (`mechanism-bc-probe` `8e42824fc` +
`1c4bd724b`, switch `POLYFEM_SI_PROBE_B=birth0g`) without the probe scaffolding
or an environment variable.

1. **The first-contact refresh one iterate earlier.** PolySolve reports its
   start point x₀ and its first accepted iterate x₁ both with iteration 0
   (`post_step` is called before the iteration counter advances). Production
   returned from every iteration-0 `post_step`, so a contact born by x₁ after a
   contact-free snapshot was first refreshed at x₂, the first iterate the CCD
   line search does not truncate; on the 1e-15 repro that is where nearly
   parallel edge pairs make the fresh estimates roundoff-sensitive (the repro's
   floor). Now an iteration-0 `post_step` runs the first-contact refresh
   (`!kappa_snapshot_had_contacts_ && controller_has_contacts()`) when its
   coordinates differ from the previous `post_step`'s. That is the first
   accepted iterate, or a start point that the Dirichlet snap moved away from
   the solve-start refresh point (a contact the snap itself creates is then
   refreshed at the start point). A start point that repeats the previous
   coordinates (a solve continuing from the last iterate, a stall restart)
   never refreshes. Otherwise an iteration-0 `post_step` still does nothing:
   no in-solve controller step, as before.
2. **No collapse bump in a first-contact refresh.** In the refresh's
   controller branch a first-contact refresh from `post_step` (at iteration 0
   or later) does not read its gaps as a collapse: the barrier has not acted on
   those contacts yet, and their gaps are where the step that made them stopped
   (usually truncated by CCD). It makes no collapse bump, and the conditioning
   cap, which production skipped during a collapse, applies when the gradient
   balance gives no signal. Without this part (*birth0* alone) the repro's
   cube 8×8×8 variant regressed: at the CCD-limited x₁ every gap is 0.01 d̂,
   the collapse branch bumped the trim to 905, and the stiffer onset was itself
   sensitive (jumps up to 2.0e-4). The guard applies to **every** first-contact
   refresh in `post_step` (also those at iteration ≥ 1, e.g. IT's re-contacts):
   that is the measured form. The narrower variant (guard only at the
   iteration-0 refresh) was not measured and is not implemented.

Unchanged: the solve-start (published) refresh, including its collapse bump
when the run's first snapshot already has collapsed contacts; stall retunes;
the in-solve emergency bump and the 30-iteration cadence; RB-20 continuation;
every option and default; the run-manifest schema (a guarded first-contact
refresh shows up in `history_sensitivity.trim.moves` as a `conditioning_cap`
move, or none, instead of a `collapse` move); the restart file layout.

Implementation notes:

* The x₀/x₁ distinction is the probe's: `last_post_step_x_` holds the previous
  `post_step`'s coordinates (semi-implicit mode only; one copy of the full
  coordinate vector per `post_step`). At a start point that follows its own
  solve-start refresh the condition cannot hold anyway; the comparison keeps
  the adopted form identical to the measured one also in the corner cases (a
  snapshot taken on another collision set, then a solve continuing from the
  last iterate).
* `last_post_step_x_` is part of the form's RB-06 rollback state. It is not
  written to restart files: a resumed run's first `post_step` follows the
  solve-start refresh, where a missing previous point and the uninterrupted
  run's previous point lead to the same decision.
* `refresh_semi_implicit_stiffness` takes a fourth argument,
  `first_contact_refresh` (default `false`); only the two first-contact calls in
  `post_step` pass `true`.
* With `output/physical_diagnostics` on, the iteration-0 refresh is reported as
  a `post_step` coefficient event, like the first-contact refresh at a later
  iteration.

## Verification

All runs single-threaded with Accelerate on one thread (`VECLIB_MAXIMUM_THREADS=1`),
the isolated build `birth0g-work/build` (RelWithDebInfo, Python on), binary
`PolyFEM_bin-birth0g-c1`: the working tree later committed as `ca8435035`
(which only rewords one header comment). The build of the committed code,
`PolyFEM_bin-ca8435035`, repeats the smokes, the whole repro family and the
base runs of R1, BBT, IT and pup_push byte for byte (`runs/verify/smokes-final`,
`runs/Z-*`, `runs/ens/*/F-base`). The probe's runs are
`mechanism-bc-work/bin/PolyFEM_bin-probe7` with `POLYFEM_SI_PROBE_B=birth0g`.
Production is canonical keys: the B/C session's production arm (its probe
binary with no switch set, byte-identical to canonical keys), and for the
smokes, the unit-test baseline, the repro accuracy check and ball-burst this
build of `435c08a63` (docs only on top of `a7d39fc7b`).

| check | result |
| --- | --- |
| Repro perturbation family (7 variants × the unperturbed run and 8 perturbations of 1e-15–1e-14) | **63 runs, 1,764 output files byte-identical to the probe's** (`mechanism-bc-work/runs/G-*`), run manifests identical. 15 of 56 perturbations jump, max 1.0e-7 (production: 40, max 2.5e-4); iterations of the unperturbed runs 41 / 43 / 43 / 49 / 42 / 52 / 35 and final trims 2.0 / 4.2 / 1 / 1 / 2.0 / 2.0 / 3.4 (base, d̂ 0.005, 0.02, 0.04, cube 4×4×4, cube 8×8×8, wide plate), as the B/C record's table |
| Five public smokes | **byte-identical to the probe's** (`smokes-p7-Bdg`: 55 files, 25 VTUs). Against production the four semi-implicit smokes move by 6.1e-6 – 6.9e-6 (max \|Δu\| / max \|u\|) at step 1, 2.2e-6 – 2.5e-6 at step 4; `quasistatic-adaptive` is byte-identical. `tools/smoke/run_smoke.py`: all five exit 0 with no error lines, worst minimum distance 1.001e-4 (d̂ 1e-3), as before |
| R1, 3 steps (base, four 1e-15 nudges, CHOLMOD, Simplicial) | **byte-identical to production** (7 of 7 runs) |
| BBT, 20 steps (the same seven members) | **byte-identical to production** (7 of 7) |
| IT, 200 steps (base and four nudges) | **byte-identical to the probe's** (5 of 5, 403 files each); different from production, as measured |
| pup_push, 3 steps (base, CHOLMOD, +x nudge, Simplicial) — *birth0g* was not run there before | **byte-identical to production** (4 of 4; 111 / 82 / 98 / 90 subsolve iterations, 17 / 6 / 18 / 16 stall retunes, as production) |
| Unit tests | Affected selection (the canonical-keys record's 28 tags plus `[first_contact]`): **161 cases / 13,122 assertions pass** (production: 160 / 13,147). The difference is accounted for case by case: the new case (+22), the build-identity case (+1: the tree had uncommitted documentation) and the AL-budget rollback case (−48: its deliberately impossible motion now stagnates after 10 AL passes instead of 18, and the case asserts per pass; it accepts any count). Full CTest suite on the build of `ca8435035`: **438 of 438 pass** (serial as in CI, `OMP_NUM_THREADS=1`, 82 min, `cli_contract` included) |
| RB-02 probe (`tools/rb02`) | **270/270**, `passed: true` (on the working tree and again on the build of `ca8435035`); every measurement equals the canonical-keys session's runs up to the order of one batch's scales and the last bit of its energy, which also differ between those runs |
| RB-10 stage-1 friction probe (`tools/rb10/run_probe.py`) | 23 checks, `passed: true`, exit 0 (it never calls `post_step`). The runner itself reports "non-JSON output": the toolkit's `to_full_dof` deprecation warning is printed on stdout before the probe's JSON (since the IPC integration of 2026-09-27; not this change) |
| Ball-burst with the ball head as a moving obstacle, 10 steps | **byte-identical to production** (25 files; logs identical line for line; manifests identical); inert by construction (see *Moving obstacle*) |

Why R1, BBT and pup_push do not move: production makes one first-contact
refresh in each (probe streams of the B/C ensembles), at iteration 6–7 of the
first solve, for a single contact at 0.79–0.998 d̂. Neither part applies there:
it is not an iteration-0 report, and the gap is not below the band. The smokes
move because their cube is pressed into the slab by a CCD-truncated first
iterate: production refreshed one iterate later, at 44 contacts, with a
collapse bump (trim 1 → 2); now the refresh runs at x₁ (14 contacts at
0.1 d̂) without one. IT moves because both parts act at its re-contacts after
the bodies separate: production's base run makes 12 first-contact refreshes,
8 of them within the first two iterations of a solve and 10 with a collapse
bump (×2 to ×6, band rms 0.11–0.69 d̂).

### Smoke references

The repository stores no smoke outputs; a change is checked byte for byte
against the previous binary's outputs, single-threaded. The reference from
here on is this binary's set, identical to the probe's: `birth0g-work/runs/verify/smokes-c1`
(file list with SHA-256 in `birth0g-work/runs/verify/smokes-c1.sha256`; digest
of that list `b3eb8dce12fb4287c380e9027f96ec10fd6934c6ac545dcb6fb543f0aae18ef4`).
Twenty VTUs differ from the previous reference (steps 1–4 of the four
semi-implicit scenes); the 35 other files are unchanged. No stored reference changes:
the CTest data fixtures, whose references follow the CI-05/CI-06 policy, do not
use semi-implicit stiffness (no fixture in `sdast9/polyfem-data@aed03ab` names
`semi_implicit`).

### New unit case

`[kappa_continuity][first_contact]` (`tests/test_kappa_continuity.cpp`), on the
point-over-edge fixture with a synthetic Hessian: a solve starting contact-free
whose first accepted iterate (reported with iteration 0) brings a contact at
0.01 d̂ is refreshed there (the snapshot is x₁, the objective generation
advances), with a conditioning-cap move and no collapse move, and the next
iterate does not refresh again; a contact born at iteration 1 gets the same
guard; an iteration-0 report at the previous report's coordinates does not
refresh, the next new iterate does; the solve-start refresh still bumps for a
collapse. Against production's `BarrierContactForm` sources 11 of its 22
assertions fail (all but the last section's).

## Moving obstacle

### Ball-burst with the ball head as an obstacle (the user's choice)

`birth0g-work/tools/make_bb_obstacle.py` turns `ballhead.msh` into an obstacle
(the boundary of its tetrahedra is the collision surface) that moves rigidly
like the ball's prescribed face set, [0, −0.03/60·t, 0] m, and drops the
ball's two Dirichlet entries, its material and its discretization order. The
membrane, its clamp, the contact and solver settings and the production
controller are the scene's; output is a volume VTU per step, physical
diagnostics off.

**This scene cannot exercise *birth0g*.** The knit (Restorelle) membrane is in
self-contact from t = 0: the first refresh sees 45 contacts (smallest gap
0.12 d̂, band rms 0.79 d̂), and the same 45 with identical statistics when the
obstacle starts 0.1 mm higher. Every snapshot therefore has contacts, `!kappa_snapshot_had_contacts_`
never holds, and the ball's arrival (during step 1's augmented-Lagrangian
passes in the scene as is) is an ordinary mid-solve birth, estimated on the
existing snapshot — the stale-stamp mechanism B that is not repaired — never a
first contact. Both parts of *birth0g* act only in first-contact refreshes, so
the trajectory cannot change. Measured: production and the adopted binary,
single-threaded, 10 of the scene's 200 steps (t = 3 s; 187 contacts at the end
of step 1, 881 at step 10): all 25 output files byte-identical, the logs
identical line for line apart from timestamps and timings, the manifests'
step records identical, and each of the 34 refreshes had at least 45
contacts. The variant with the obstacle 0.1 mm higher was stopped once its
first refresh showed the same 45 contacts.

The same holds for the user's scenes of the B/C measurement: R1, BBT and
pup_push each have one first-contact refresh, at iteration 6–7 with a single
in-band contact, and stay byte-identical; only IT (re-contacts after the
bodies separate) and the synthetic scenes (smokes, repro family) start a
contact from a contact-free snapshot and move.

### The repro family as the measured moving-obstacle case

The 1e-15 repro is itself a moving obstacle (rotating by 90° and dropping) that
strikes a contact-free block: the onset *birth0g* changes. Its roundoff
response is in the verification table. Accuracy in the sense of the proposed
D1 standard (`birth0g-work/tools/repro_accuracy.py`): distance to a reference
with the trim pinned at 2⁻¹⁸ and `derivative_along_delta_x_tol` 1e-15 (the B/C
session's R1/BBT reference settings), with the metric of `tools/d1eval.py`
(‖u − u_ref‖ / mean(‖u‖, ‖u_ref‖) on body nodes, per step), for the
unperturbed run of each variant:

| variant | production | adopted | adopted / production | worst step (adopted / production) | D1 robust test (≤ 1.1× production at every step) |
| --- | --- | --- | --- | --- | --- |
| base (d̂ 0.01) | 0.815 | 0.805 | 0.99 | 1.00 | pass |
| d̂ 0.005 | 0.682 | 0.671 | 0.98 | 1.00 | pass |
| d̂ 0.02 | 1.195 | 1.195 | 1.00 | 1.00 | pass |
| d̂ 0.04 | 1.563 | 1.563 | 1.00 | 1.00 | pass |
| cube 4×4×4 | 0.827 | 0.809 | 0.98 | 1.00 | pass |
| cube 8×8×8 | 0.824 | 0.811 | 0.98 | 1.00 | pass |
| wide plate | 0.817 | 0.810 | 0.99 | 1.00 | pass |

The distances themselves are the barrier's gap offset at these d̂, which are
comparable to the block's motion per step, and are the same for both. The pin
acts only through a trim move (the trim starts at 1 and is clamped when it
moves): production never moves the trim on the d̂ 0.02 and 0.04 variants, so
its pinned run stays at trim 1 there, while the adopted binary's first-contact
cap engages the pin in every variant. The adopted binary's pinned runs are
therefore the reference; where both binaries' pinned runs are pinned they
agree to d ≤ 0.0026.

## Limits

* Where *birth0g* acts, the evidence is the probe's measurement repeated byte
  for byte: the repro family (7 variants of one synthetic geometry), the
  smokes, and IT's five runs (statistics within production's run-to-run
  scatter, one deterministic realization per nudge). Among the user's scenes
  only IT changes; the moving-obstacle scene the user chose cannot exercise it
  (above), so no realistic moving-obstacle onset from a contact-free snapshot
  has been run.
* The repro accuracy check is one unperturbed run per variant against one
  reference; the reference is the adopted binary's pinned run (production's
  does not pin on two variants).
* Ball-burst: single-threaded, 10 of 200 steps (step 1 alone took 30 min);
  the user's runs are threaded.
* The narrower variant (guard only at the iteration-0 refresh) was neither
  measured nor implemented.
* Not run: the Houdini asset tests. They drive the shared `polyfem/build`,
  which stays at `49ad24b74` until the push (the user's choice).

## Evidence

Parent workspace `birth0g-work/`: `BRIEF.md`; `build/`, `configure-command.txt`,
`build.sh` (isolated build; toolkit `1f1b5dbf`, PolySolve `43ca2e6` worktrees
beside it); `bin/` (`PolyFEM_bin-435c08a63-baseline`, `PolyFEM_bin-birth0g-c1`,
`PolyFEM_bin-ca8435035`, the matching `unit_tests`, the source patch,
`hashes.txt`); `runs/A-*` (repro family) and `runs/Z-*` (its repeat on the
committed code), `runs/verify/` (smokes, `run_smoke.py` outputs, the reference
hash list), `runs/ens/<scene>/A-*` and `F-base` (R1, BBT, IT, pup_push),
`runs/acc/` (repro accuracy), `runs/unit/` (selection logs and XML,
`ctest-full/`), `runs/rb02/`, `runs/rb10/`, `runs/bbo/` (ball-burst obstacle
runs); `inputs/` (repro inputs copied from `mechanism-bc-work`, the obstacle
variants); `tools/` (`cmp_family.py`, `cmp_ens.py`, `repro_accuracy.py`,
`make_bb_obstacle.py`, `run_bbo.sh`, `loglockstep.sh`, `run_final.sh`,
`run_ctest_full.sh`, and the B/C session's drivers, copied).

## Publication

Local commits on branch `adopt-birth0g` (worktree `birth0g-work/polyfem`):
the code and the new case (`ca8435035`) and this record with the documentation
updates, on top of `435c08a63` (the B/C record) and `a7d39fc7b` (canonical pair
keys). **Not pushed**: canonical keys are not pushed yet (user, 2026-10-05:
"not yet"), and the three go together. With that push the shared `polyfem/`
checkout is fast-forwarded and `polyfem/build` rebuilt (the Houdini assets run
that binary); then the asset tests run against it.
