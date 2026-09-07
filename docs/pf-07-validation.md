# PF-07: AL initialization and continuation control — 2026-09-07

Implemented two bounded continuation corrections and documented the existing
initializer's limits. A new generalized-curvature initializer, continuation
budget, or early-snap restart optimization is not introduced.

## Reproduced and fixed

The new real `solve_al` regression failed **6 of 26 assertions** before the
production changes:

- Initial weight 3, multiplier 2, ceiling 5 produced weight 6. Growth now clips
  to the ceiling. An initial weight already above the ceiling is still preserved;
  the ceiling controls subsequent increases, not the caller's initializer.
- With zero initial and current BC error, `1-sqrt(current/initial)` was NaN.
  This skipped weight continuation even while the synthetic geometric gate
  required further preparation. Zero-to-zero now reports zero relative BC
  progress; zero-to-positive reports negative progress and retains rollback.
  For positive initial error, separate square roots avoid overflow in the ratio.

The last inner-solve diagnostic now includes `al_initial_error`,
`al_current_error`, `al_relative_progress`, and `al_next_weight`. These describe
BC residual progress and the next penalty, not a geometric feasibility metric
or an equilibrium certificate. The current error is measured before any rollback.

The regression's synthetic snap gate requires the free quartic coordinate to
fall below 0.5. Multiple interrupted AL passes are accepted, followed by a
configured converged reduced solve. The gate is explicitly not a collision test.
PF-01's hard-error handling and reduced convergence contract are preserved.
No interruption counter, new BC stopping tolerance, or mandatory AL stationarity
condition has been added. The existing outer loop still has no general budget
for geometrically stagnant but usable iterates; designing such a budget remains
optional work requiring geometric progress and explicit failure diagnostics.

## Initializer characterization; policy unchanged

Both nonlinear-elastic forward implementations use
`rho0 = max(multiplier * max_abs(H_elastic), 1)` with the **weighted elastic
form Hessian**. The old comment claimed curvature dominance; that claim is
removed. The schema now states the actual formula, including the numeric floor.
This option is not newly enabled for other VarForms.

For normalized BC metric W, the quadratic penalty is
`rho/2 * (u-b)^T W (u-b)`. Thus rho has internal objective/displacement-squared
units. With a physical energy objective these are stiffness units; with the
transient acceleration-scaled objective they need not be force/length. The
current transient setup scales elastic energy by the integrator's acceleration
factor and has an inertial Hessian M. The initializer omits that M, contact and
other forms. Uniform FEM mass scaling cancels from W, not from inertia.

A reproducible two-coordinate algebraic characterization uses
`H_elastic = [[2,1],[1,2]]`, multiplier 10, and rho0=20:

| Case | Largest curvature relative to the BC metric | Initial rho |
| --- | ---: | ---: |
| W=I, elastic only | 3 | 20 |
| W=diag(0.001,1.999), elastic only | 2000.25021893 | 20 |
| W=I, add inertia diag(100,300) | 302.004999875 | 20 |

These values are the largest eigenvalues of `W^(-1/2) H W^(-1/2)`.
They show why maximum entry alone is not a generalized-curvature bound. For
partially constrained systems, coupling to free coordinates needs additional
analysis; this two-constrained-coordinate example does not prescribe a universal
penalty or establish that curvature dominance is required for safe snapping.

Under objective rescaling by 1e-6 with unchanged coordinates, the equivalent
rho is 2e-5. The initializer's numeric floor instead gives 1, a factor 50,000
larger. A characteristic scale could remove this dependence; selecting that
scale and accounting for inertial/coupled curvature are optional numerical
changes, separate from the corrections above. No production failure is attributed
to these synthetic examples and no initializer default changes here.

## Equivalent-unit and density regression

A coupled quadratic uses dimensionless elastic Hessian `[[2,-1],[-1,2]]`
plus inertial `density * diag(1,3)`, prescribed coordinate 0.3 and zero inertial
predictor. All **27 combinations** of coordinate scale L in {1e-3,1,1e3},
objective scale S in {1e-4,1,1e4}, and density in {0,1,100} run actual AL and
reduced solves. Density changes the physical reference solution; L and S are
converted representations of each reference problem.

Coordinates/targets scale by L, objective Hessian and initial/maximum penalty
by S/L^2, and absolute gradient tolerance by S/L. Uniform mass conversion leaves
the BC metric [0.5,1.5] unchanged. Multipliers start at zero and evolve through
the real BC form. Each final solution is compared with
`u_fixed/L=0.3`, `u_free/L=0.3/(2+3*density)` and a converted free-gradient residual
below 1e-9. The synthetic snap gate checks prescribed-coordinate distance below
0.05 L; iteration counts are not an acceptance criterion. This validates the
converted numeric-penalty path, not unit covariance of `hessian_scaled` or a
full FEM scene's physical accuracy.

## Validation and provenance

Started at PolyFEM main `2ee367b37` with clean tracked sources. Existing untracked
simulation outputs were preserved. Effective dependencies remain IPC `9da3094`
and PolySolve `4d372fa8` (local source override matching the pinned revision).
Local evidence is in the parent workspace's `outputs/pf-07/`.

Both `PolyFEM_bin` and `unit_tests` rebuilt. The affected suite passed
**22 cases / 1,161 assertions**, including the two new cases (809 assertions),
AL phase/cleanup tests, BC metric and scale tests, direction-filter tests, and
semi-implicit contact/friction derivatives. Expected negative-test error logs
are not failures. All five solver smokes exited zero with zero error log lines;
the real-binary Houdini PolyFEM 2.0 end-to-end test and JSON round-trip passed.
Formatting checks for the changed solver/test regions and `git diff --check`
passed. No full unit-suite, Ballburst, or Teseo run is claimed.

Reproduction from the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
zsh outputs/pf-07/run-smoke.sh quasistatic-semi.json quasistatic-adaptive.json transient-semi.json quasistatic-semi-friction.json quasistatic-semi-alhess.json
/Applications/Houdini/Current/Frameworks/Houdini.framework/Versions/Current/Resources/bin/hython houdini_HDAs/tests/test_polyfem_hda.py
```

The local smoke runner is a copy of the workspace runner with only its output
directory redirected into `outputs/pf-07/smokes/`. The source changes, tests,
schema documentation and this record are published together on the PolyFEM
fork's `main` branch; companion revisions are unchanged.
