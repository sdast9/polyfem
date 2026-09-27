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

**Contact runs are not continued exactly.** The state file holds only the
integrator history (u, v, a). The semi-implicit contact form's memory — the
global trim and the memoized per-contact coefficients — starts fresh, so the
resumed run re-derives them (trim 1 → 4 in step 4 versus the uninterrupted
run's trim 4 carried in; first-refresh κ 1.89e12 versus 1.52e12). Exact
continuation would need that form state serialized with the integrator state;
not attempted here.

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
