# Floor-disabled default — 2026-09-07

The user selected disabling the constraint floor and will report any recurring
instabilities. The schema and direct C++ fallback now default to zero. The
Houdini asset also defaults to zero and exports that value. Explicit positive
values remain opt-in legacy behavior, including its known PF-02 defects.
Existing saved nodes/JSON must have positive settings changed to zero explicitly.
PF-09 hard-contact work is deferred.

This change bypasses floor-dependent barrier deletion and direction projection
in the default path. CCD, force-saturation default (gap_floor=0), coefficient
assignment/refresh, friction lagging, restart budgets and tolerances are unchanged.
It does not establish positive coefficients in all configurations, a fixed
objective across retunes, or general physical accuracy.

## Targeted validation

Started at PolyFEM a2f1811b9 with clean tracked sources. Dependencies remain IPC
9da3094 and PolySolve 4d372fa8. PolyFEM_bin and unit_tests rebuilt successfully.

- Focused solver/contact suite: 22 cases / 1,161 assertions passed.
- All five semi-implicit/adaptive smoke scenes completed their four steps,
  exit zero and zero error log lines. No input tolerances/schedules were changed.
- PF-02 real-form probe completed all four floor/snapshot configurations. With
  floor zero, barrier energy at gap 5e-5 stays approximately 1980.6975 through
  refresh; the scalar force-balance relative residual is below 1.34e-12. Explicit
  positive-floor runs still reproduce the known defects. This is characterization,
  not a new full-scene physical certificate.
- Installed HDA node default and exported end-to-end JSON both checked as zero.
- git diff --check passed. No new full-suite or Teseo run was performed.

Evidence is preserved in the parent workspace at outputs/pf-floor-disabled/:
build.log, focused.log, smoke-summary.log, smokes/, probe/, hda-default.log and
HDA build/test logs. The HDA publication uses assets built inside its publication
checkout and tested from a sibling verification copy against the rebuilt solver.
PF-08 physical-validation and robustness limitations remain unresolved.
