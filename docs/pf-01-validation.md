# PF-01 correction and validation — 2026-09-06

## What was corrected

The initial uncommitted PF-01 patch imposed a gradient-only check on both solve
phases, counted interrupted AL passes toward a three-failure limit, and changed
bounded friction-lagging exits into exceptions. The corrected patch preserves the
original AL continuation and lagging policies. The final reduced solve rejects
callback/budget interruption while honoring the configured convergence criteria.

Both user scenes also exposed a PolySolve status ambiguity: a sufficiently small
negative directional derivative returns `NotDescentDirection`. The correction
recognizes this configured tolerance exit separately from a genuinely
non-descending direction/hard failure. Diagnostics retain the raw status and
explicitly name the tolerance. This does not establish raw-gradient equilibrium
or validate the physical correctness of the remaining contact-floor/filter model.

There are no changes to contact-floor settings, penalty initialization, friction
iteration limits, timestep/load schedules or dependency pins. Actual inner-solver
status is now used for AL/reduced diagnostics instead of a separate solver object.
A failed reduced solve does not overwrite its caller's input solution or invoke
the success callback; this is not a general rollback of all simulation history.

## Focused validation

- Rebuilt `PolyFEM_bin` and `unit_tests` on macOS ARM64.
- Eight AL cases: 45 assertions passed, including a synthetic geometric gate that
  needs at least three interrupted AL passes before snapping and a subsequent
  converged reduced solve; final soft-budget/custom-stop rejection; configured
  non-gradient and negative-slope tolerances; shared-solver hard failure cleanup.
- Combined AL plus both semi-implicit contact/friction derivative cases:
  **10 cases, 205 assertions passed**.
- Five solver smokes exited 0: quasistatic adaptive, semi-implicit, Hessian-scaled
  AL, friction, and transient semi-implicit.
- clang-format 21.1.8 checks and `git diff --check` passed.
- Resume verification on September 6: rebuilt both targets and reran the same
  focused selection; 10 cases / 205 assertions passed again. No C++ changes were
  made during the handoff completion.
- No new full-suite claim: the separate historical cube-on-floor reference issue
  is outside this correction.

## User scene validation

Inputs were copied into `outputs/pf-01-correction/` in the parent workspace.
All 27 supporting files match the originals byte for byte. Parameter changes are
limited to isolated output paths and the ballburst schema translation below.
Original inputs and output directories were not modified.

| Scene | Result |
| --- | --- |
| ballburst_thin_membrane | All 200 steps completed; exit 0; about 304 seconds. |
| teseo_problem, first run | Saved step 11/20, then stopped in the next final reduced solve after 20 restarts; exit 134. |
| teseo_problem, isolated awake rerun | All 20 steps completed at 17:02:38 EDT; normal completion log and final output verified; about 1126 seconds. Original process exit code was not retained across the handoff. |

Ballburst's saved JSON has obsolete `space.pressure_discr_order`; the copied
input uses the current name `space.discr_orderq` with the same value. This input
compatibility correction is separate from PF-01 and was not applied to the user's
source file. Teseo uses its original numerical settings.

The first Teseo failure is a final-stage interruption, not an AL
stationarity requirement: the log shows a successful snap/reduced-space handoff,
then `ObjectiveCustomStop` after 20 restarts. The reported directional derivative
was approximately -0.00578, far outside its small-slope tolerance, and the solver
reported gradient norm about 1.04e7 (the callback criteria describe the solver's
iteration state, not an independently recomputed final residual). The old wrapper
would have returned normally on this exhaustion branch. The correction rejects
that result. The machine slept for about 30 seconds during this run (10:43:05 to
10:43:35 EDT); the failure occurred at 10:46:28. The isolated awake rerun
completed all 20 steps with the same numerical settings. Its log runs from 16:43:52 to 17:02:38 EDT and ends with error
computation and total timing; `step_20.vtu` and the final PVD entry are present.
The retained `pmset` log contains no Sleep/Wake/DarkWake events during this
window and records the caffeinate idle-sleep assertion. Evidence is saved in
`teseo_awake/sleep-evidence.txt`. The process is no longer running; its original
exit status could not be recovered, so completion is established from the log
and output rather than a newly observed exit code.

Both outcomes remain part of the record. The successful run followed a different
iteration path even before the first run slept; the first run also initially
overlapped ballburst. This is not a controlled test proving that sleep caused the
stall. Investigating the path-dependent final-stage exhaustion remains separate
work; do not relabel an interrupted state as convergence to pass this scene.

Before recognizing the negative-slope tolerance, the narrowed status check still
rejected ballburst with slope about -3.35e-16 and Teseo with slope about -3.88e-14,
both inside their configured thresholds after solver scaling. Those logs are
preserved as `rejected-legacy-tolerance.log`. This demonstrates why merely
whitelisting statuses labeled as success, or demanding raw gradient stationarity,
was an overbroad change. Passing these simulations means successful completion
under their configured criteria, not independent physical validation.

## Reproduction and handoff

From the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
```

The local evidence directory contains the incoming patch/test backup, isolated
inputs, user-scene logs/outputs, build/test logs and smoke summary. Do not publish
large private scene assets just to preserve these results.

Follow [the revised plan](correctness-remediation-plan.md) for PF-02 through
PF-09. They were not implemented by this correction; PF-02 starts as a model
comparison, and PF-09 requires a separate choice of contact formulation.
