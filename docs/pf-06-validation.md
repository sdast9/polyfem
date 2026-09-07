# PF-06: direction-filter derivative contract — 2026-09-07

The solver now reports `g.dot(p)` for the filtered search direction `p`.
The direction-filter callback signature and its application to computed search
steps are preserved. PolySolve no longer calls that filter on `-g` to manufacture
a substitute slope. PolyFEM pins the corrected companion revision.

## Objective and scope

The forward caller lifts a direction to full coordinates, applies
`BarrierContactForm::project_floor_pairs`, then restricts it to reduced
coordinates. The filter removes closing components at the numerical floor;
separation and sliding remain free. It is a one-sided, four-sweep direction
heuristic, not a definition of a new objective or inequality merit function.
PolySolve's Backtracking line search evaluates the supplied objective, and its
Armijo search already computes `g.dot(p)`. The descent check and slope stopping
criterion must use that same derivative wherever the objective is smooth:

```
d/dalpha E(x + alpha p) at alpha=0 = g(x)^T p
```

The old `-p.dot(P(-g))` is generally different. Equality for a linear orthogonal
projector with `p` in its range does not extend to one-sided filters or to
lift/project/restrict operators that eliminate fixed DOFs.

This change does not fix the floor's discontinuous objective, fixed-DOF
projection feasibility, coefficient refresh policy, or establish constrained
stationarity. Those remain PF-02/PF-09 concerns. CCD, floor defaults, restart
policy, friction lagging, and configured convergence tolerances are unchanged.
PF-01 still accepts a genuinely negative objective slope inside its configured
tolerance; an ascent direction must not acquire that status through a negative
surrogate. There is no claim that all production floor states have a smooth
objective or that this correction defines a complete hard-contact solver.

## Reproduced defects

The initial focused regression run failed **7 of 33 assertions** across four
of five cases, before changing production code. The cleanup case passed.

- A coupled quadratic has gradient `(1,1)` and Newton step `(1,-3)`.
  The one-sided filter `p[0]=max(0,p[0])` preserves that separating step.
  Finite differences give slope **-2**, while the old solver reports **-3**.
  With slope tolerance 2.5, the old code incorrectly skips that configured
  stopping criterion and takes a step.
- A synthetic fixed-coordinate lift/project/restrict filter changes the second
  component of closing directions by a factor of one half. The old surrogate
  rejects the original descending direction and switches to a regularized
  strategy. The corrected original slope agrees with finite differences.
- A deliberately invalid caller filter returns `(1,1)` for every input.
  The actual slope is **+2**, but filtering `-g` produces the surrogate **-2**.
  With the test's deliberately large slope tolerance, the old solver returns
  normally under its negative-slope tolerance. The correction rejects ascent
  on all strategies before line search. This is a synthetic API counterexample,
  not a claim that the production contact filter returns this direction.
- The **real** `BarrierContactForm` filter is exercised on a point at gap
  `5e-5` above a fixed edge, with floor `1e-4` and `dhat=1`. A coupled smooth
  quadratic on the point's free coordinates gives separating step `(-3,1)`.
  The exact production projection plus fixed-edge elimination reports an old
  slope of **-2.6666667** versus finite-difference **-2**. The objective in this
  regression is explicitly the quadratic; this isolates the filter contract
  from the discontinuous contact energy characterized in PF-02.

Central differences use step `1e-6` and absolute tolerance `1e-9`. Additional
controls cover no filter, an identity filter, and a linear orthogonal projector.
AL tests count filter invocations and reuse the same PolySolve instance after
convergence, interruption, and exception to verify cleanup on every exit.

## Validation

Both `PolyFEM_bin` and `unit_tests` rebuilt. The affected suite passed
**20 cases / 352 assertions**, covering direction filters, AL termination and
cleanup, BC metric/scaling, and semi-implicit contact/friction derivatives.
Expected negative-test and slope-tolerance error logs are not test failures.

All five solver smokes exited zero with zero error log lines. The real-binary
Houdini PolyFEM 2.0 end-to-end test passed, including JSON round-trip import.
Formatting checks on the changed solver files and new test, and `git diff
--check`, passed. No full unit-suite, Ballburst, or Teseo run is claimed.
These checks validate the derivative contract and regressions, not whole-scene
physical accuracy or robustness with every active contact-floor configuration.

## Provenance and reproduction

Started with clean tracked sources on PolyFEM main `5a6c68d07` and the PolySolve
`012658e` local override. Existing untracked simulation outputs were preserved.
Fetched companion remotes and fast-forwarded PolySolve to `713220f`, which has
an identical source tree and is also the inspected upstream main revision.
The defect is present there; no shared API signature change was needed.
Effective IPC remains at `9da3094a46bcc054cc19024a5c748557c5bb6b9e`.
The parent workspace's `outputs/pf-06/` contains incoming status, baseline
failure evidence, build/test logs, and smokes in a fresh output directory.

From the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
```

The correction is PolySolve `4d372fa8a73f42bc224e31d464f1a308e1159ba8`, published to the fork's
`iteration-callback` branch;
PolyFEM's dependency recipe records its exact revision. The tests and this
record are published with that pin on the PolyFEM fork's `main` branch.
