# Constraint-floor retirement — 2026-09-07

## Decision and implementation

After preliminary testing with the floor disabled did not reproduce the earlier
candidate/memory blow-up, the user requested full removal if the floor was
physically inconsistent. PF-02's real-form counterexamples establish those
specific inconsistencies: discontinuous deletion of barrier energy/forces,
full-coordinate projection losing feasibility after prescribed DOFs are restored,
and stiffness refresh restoring the deleted barrier at unchanged coordinates.
They do not establish that every historical simulation was inaccurate.

The barrier mask, four-sweep `project_floor_pairs` implementation, member and
public method, and the production caller installing that filter are deleted.
There is no floor-dependent contact path left to opt into. The generic PolySolve
and ALSolver direction-filter API and its PF-06 derivative/cleanup regressions
remain; the retired contact-specific filter test is replaced with a barrier
retirement regression. This does not roll back PF-06.

The schema retains `constraint_floor` only as an ignored compatibility key.
A nonzero legacy value warns at semi-implicit form construction and cannot
reactivate the mechanism. New HDA exports omit it, the HDA control is removed,
and importing nonzero legacy JSON produces a compatibility note and drops the
setting on re-export. Even a saved/spare node parameter is not read on export.
Refresh Asset Libraries or restart Houdini to pick up the rebuilt definition.

CCD, `trial_displacement_cap=50`, the separate optional `gap_floor` force
saturation (still disabled by default), coefficient retuning, friction lagging,
restart budgets and tolerances are unchanged. The trial cap controls proposed
swept displacement, not a guaranteed memory or candidate-count budget.

## Reproduction and targeted validation

Started from clean tracked sources at `6279b3492`; IPC and PolySolve pins remain
`9da3094` and `4d372fa8`. Before production edits, the new regression failed
4/33 assertions against the positive legacy floor. After removal it passes all
33 assertions across omitted, zero and positive legacy options: energy derivative
continuity across the former threshold, retained below-threshold force, reaction
balance, refresh/rebuild consistency, CCD crossing rejection and limited-step
acceptance, and preservation of the trial cap.

- Rebuilt `PolyFEM_bin` and `unit_tests`.
- Focused suite: **22 cases / 1,189 assertions passed**. One retired real-floor
  derivative test was replaced; generic direction-filter tests remain.
- All five standard contact smokes completed four steps with exit zero and zero
  error log lines. Inputs and schedules were unchanged.
- A separate real-binary quasistatic smoke with an explicit positive legacy
  `constraint_floor=1e-4` completed and emitted the expected retirement warning.
- Four PF-02 probe configurations (zero/positive legacy setting, pre-existing/new
  contact) retain barrier energy through refresh and reject crossing via CCD.
  Scalar force-balance relative residuals are below 1.4e-12. The CLI legacy value
  is compatibility input only; results explicitly mark the floor retired.
- The updated PF-08 probe compiled; two targeted configurations passed: L=S=1,
  two load increments, fixed single contact and moving coupled contact. This is
  not a repeat of the full 108-configuration sweep.
- All **13 HDA test scripts passed** against the rebuilt solver. The updated
  end-to-end regression verifies absent control, ignored leftover positive
  spare parameter, legacy-positive import and omission on re-export.
- C++ formatting and `git diff --check` passed. No full solver suite, private
  scene, Ballburst or Teseo rerun is claimed.

Evidence is in the parent workspace's `outputs/pf-floor-removed/`, including the
baseline failure, build/focused/smoke logs, legacy-input run, four micro-probes,
two PF-08 results, and HDA build/test logs. Historical PF-02/PF-08 JSON measurements
are preserved unchanged. To reproduce historical active-floor behavior, use
commit `6279b3492` or earlier; current probe sources no longer project directions.

This retires the defective floor mechanism, not all physical-validation work.
Production coefficient-lifecycle consistency and PF-08 mesh, force/work balance
and robustness limitations remain open. PF-09 hard-contact work is deferred.
