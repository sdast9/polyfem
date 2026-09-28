# Restart JSON that resumes the run it came from — 2026-09-27

## Problem

Found while troubleshooting the ball-burst run, whose `restart.json` could not
have resumed it. Three defects in `VarForm`'s restart output, all reproduced
with the pre-fix binary (`3c40ae557`, `baseline-nocontact-IE/`):

1. **dt changed on resume.** The restart JSON overrode only `time.t0`. With a
   `tend` + `time_steps` scene (the Houdini asset's usual export), `State`
   re-derives `dt = (tend - t0) / time_steps` from the restart time and keeps
   the full step count: resuming a 6-step, dt 0.2 run at t = 0.6 ran six steps
   of dt 0.1.
2. **Launch-directory dependence.** `root_path`/`common` were written as given
   on the command line (`params.json`), so the resume only worked when started
   from the original input folder ("Unable to open common params").
3. **PVD times shifted.** After a restart `sim.pvd` was rewritten with frame
   `i` at `t0_restart + i*dt`, so every frame, including the ones before the
   restart, was offset by the restart time.

Separately, a restart JSON written without `output/data/state` points at an
empty state path; resuming from it starts from the rest configuration. That was
the ball-burst case (the HDA's State toggle was off).

## Change

`VarForm::save_restart_json` writes `time = {t0, dt, time_steps: remaining,
tend: null}` (the null removes the common params' `tend` in the merge, so the
`dt` + `time_steps` branch rebuilds the same end time) and absolute
`root_path`/`common`. `VarForm::save_timestep` passes the PVD writer the time
of frame 0 (`t0 - file_index_offset * dt`, with roundoff of a zero start
snapped to 0). `VarForm::save_step_state` warns once when a transient run
writes a restart JSON without a state file. The FSI form and the legacy
`State` output path keep their own restart/PVD code and are unchanged.

## Validation

Evidence: `outputs/restart-fix/20260927T200625Z/` in the parent workspace
(`run.sh`, `compare.py`, binaries with `binary.sha256`). Each case runs 6 steps
straight through (A) and 3 steps + a resume from the written
`restart_3.json`, launched from an unrelated folder (B), single-threaded, on
the semi-implicit transient smoke geometry with a `tend: 1.2, time_steps: 6`
time block.

| Case | Resume dt / tend | max \|A − B\| (final u) | PVD frame times |
| --- | --- | --- | --- |
| ImplicitEuler, no contact (gravity) | 0.2 / 1.2 | 1.0e-15 | 0 … 1.2, identical file |
| BDF2, no contact | 0.2 / 1.2 | 4.0e-17 | 0 … 1.2 |
| Quasistatic, no contact | 0.2 / 1.2 | 1.0e-15 | 0 … 1.2 |
| ImplicitEuler, contact | 0.2 / 1.2 | 1.08e-4 (3.6e-4 of max \|u\|) | 0 … 1.2 |
| Quasistatic, contact | 0.2 / 1.2 | 1.08e-4 | 0 … 1.2 |
| Pre-fix binary, no contact | **0.1** / 1.2, 6 more steps | — | **0.6 … 1.5** |

In-place resume (`PolyFEM_bin -j <output>/restart_3.json`, no `-o`, from
`/tmp`) reproduces the same numbers and leaves `sim.pvd` byte-identical to
the uninterrupted run's.

**Contact runs were not continued exactly** by that first change (`7dd45a606`):
the state file held only the integrator history (u, v, a), so the
semi-implicit contact form's memory — the global trim and the memoized
per-contact coefficients — started fresh (trim 1 → 4 in step 4 versus the
uninterrupted run's trim 4 carried in; first-refresh κ 1.89e12 versus 1.52e12).
The follow-up below serializes it.

Unit tests: new `[restart]` case "restart from restart json" resumes from the
solver's own `restart_3.json` of a `tend` + `time_steps` 2-cubes scene and
checks dt, remaining steps, `tend`, the absolute root path, the solution
(identical here, difference 0.0) and all seven PVD frame times. It caught the
`-0.000000` frame-0 roundoff that the snap now handles. Both `[restart]` cases
pass. The output/time-stepping selection
(`[output],[output_kinematics],[run_manifest],[rollback],[fully_prescribed],
[time_integrator],[restart],[varform],[state],[al_budget],
[resource_containment]`) passes: 53 cases, 9,407 assertions
(`unit-selection.log`). All 13 Houdini asset test scripts pass against this
binary, including the end-to-end run with Restart JSON on (`hda-*.log`). A missing-state
restart JSON produces the new warning once (`warn-check/`).

Scenes without `file_index_offset` (every fresh run) pass `t0 - 0` to the PVD
writer and write the same restart fields except `time`, so ordinary outputs
are unchanged.

## Follow-up: the contact controller's memory is part of the state

Each step's state file now also holds, after `u`/`v`/`a`:

* `contact_scalars` — the contact form's global stiffness (the trim in
  semi-implicit mode), its adaptive bound and previous distance (classic
  adaptive IPC stiffness carries these across steps too);
* `contact_si_*` (semi-implicit only, layout version 1) — the per-contact
  coefficient cache, the previous snapshot's cache, the endpoint
  (continuation) coefficients and continued keys, the batch cap/floor/median,
  the trim anchor, controller counters, the first-contact flag and the
  refresh id. Empty tables are not written; their counts are in
  `contact_si_scalars`.
* `friction_scalars` / `friction_normal_force` — the lagged trim and the
  lagged normal force of every friction collision. With the default
  `friction_lag: realized_force` the step-end lag carries the forces that
  acted during the step (built before the endpoint refresh), which a resumed
  run cannot recompute.

A restarted transient run reads them right after its own initialization
(`NonlinearElasticVarForm::restore_restart_form_state`). The snapshot surface,
frozen Hessian and collision set are functions of the saved displacement, so
they are rebuilt at the restored coordinates (the rebuilt max |H| is compared
with the saved one and a mismatch is reported); the friction set's structure
is rebuilt the same way and receives the saved force magnitudes (a different
collision count is reported and the rebuilt lag kept). A state file without
these datasets — every file written before this change — resumes with a fresh
controller and a warning. Fresh runs are unchanged: their `sol.txt` is
byte-identical to the previous binary's (contact and no-contact cases).

Remaining difference: the resumed run computes step times as
`t0_restart + k·dt`, the original as `t0 + (offset + k)·dt`, which can differ
by one ulp (step 5: 0.9999999999999999 versus 1). The first resumed step is
bit-identical; later steps differ at roundoff. Such a difference can still
flip a threshold decision (at the step-5 endpoint the uninterrupted contact
run refreshed over 51 contacts, the resumed one over 49; the final solutions
still agree to 1e-16).

Results (`m-*`, `state2-*`; 6 steps straight through versus 3 + resume):

| Case | max \|A − B\| (final u) | before this follow-up |
| --- | --- | --- |
| ImplicitEuler, semi-implicit contact | 1.1e-16 | 1.08e-4 |
| BDF2, semi-implicit contact | 1.0e-15 | — |
| Quasistatic, semi-implicit contact | 1.0e-15 | 1.08e-4 |
| ImplicitEuler, semi-implicit contact + friction 0.3 | 1.0e-15 | 9.2e-4 |
| BDF2, semi-implicit contact + friction 0.3 | 2.7e-15 | — |
| ImplicitEuler, classic adaptive stiffness | 1.0e-15 | — |
| ImplicitEuler, no contact | 1.0e-15 | 1.0e-15 |

Classic adaptive stiffness with friction (μ 0.3 and 0.1) could not be
compared: the uninterrupted run already fails at step 5 (Newton iteration
limit), identically with the previous binary and with `3c40ae557`.

The `[restart]` case "restart from restart json" now runs both classic
adaptive stiffness and semi-implicit contact with friction 0.3, checks that
the state file carries the new datasets, and requires the resumed solution to
match within 1e-10 (was 1e-3). With the restore call disabled it fails
(difference 6.0e-4); with it both cases pass.

Regression selection on this binary (tags in `unit-selection2-tags.txt`:
output, restart, rollback, time integrators, contact/friction forms, friction
lag, kappa continuity, semi-implicit coefficients, trim controller and
predictors, stall trigger, coefficient events, continuation, objective
generation, parent identity, EF-04 and the previous selection): 94 cases,
11,380 assertions pass (`unit-selection2.log`).

## Follow-up: state file size

On large meshes each state file was mostly unused chunk space (4.0 GB per step
on the 652,440-DOF ball-burst scene). The HDF5 matrix writer now sizes its
chunks to the data, and resume behavior is unchanged; see
[state-file-chunks-20260927.md](state-file-chunks-20260927.md).

## Follow-up: augmented-Lagrangian multipliers

The Dirichlet AL multipliers also carry across steps and were missing from
the state file; a resume lost them whenever the restart step's AL stage had
run passes (the refined smoke scene: 3.2e-4 at the end). They are saved and
restored since 2026-09-28; see
[restart-al-multipliers-20260928.md](restart-al-multipliers-20260928.md).
