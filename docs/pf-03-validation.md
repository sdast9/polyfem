# PF-03: obstacle-independent BC normalization — 2026-09-07

The BC AL metric now uses the mean of the FEM lumped DOFs, before constraint
masking and excluding the trailing obstacle placeholders. Obstacle weights are
filled from the normalized FEM mean afterward. The metric is constructed once;
penalty/multiplier updates and target updates do not recompute it. The existing
row-sum, automatic HRZ and identity/no-mass fallback policies are retained.
With no FEM reference, obstacle weights default to one.

The required inhomogeneous-target test also reproduced an aliased dense slice
in the fixed-boundary constructor: its output resized/overwrote its input.
That single line now assigns the returned slice, as the time-dependent target
update already does. This is necessary to test the intended nonzero constraints;
it does not implement PF-05's nonunit form-scale derivative correction.

## Reproduction and results

Before the production edits, the two new regression cases failed (34 of 37
assertions). For FEM masses [1,3], the original FEM weights were [0.5,1.5]
without obstacles, [0.75,2.25] with one, and [1.25,3.75] with three. Now they
remain [0.5,1.5] with 0, 1, 3 or 20 obstacle DOFs, and obstacle weights are one.

After rebuilding `PolyFEM_bin` and `unit_tests`, the focused selection passed:
**14 cases, 289 assertions**. This includes four new BC metric cases plus the
existing AL termination and semi-implicit contact/friction derivative cases.
Coverage includes off-diagonal row sums, HRZ fallback, nonpositive-metric and
no-mass identity fallbacks, unconstrained FEM DOFs in the reference population,
nonzero prescribed values, and multiplier updates with changing penalties.
An initial test-fixture mistake used Catch sections inside an obstacle-count
loop and skipped mass assignments; replacing that with explicit nested loops
resolved the fixture failures without further solver changes.

For the upstream mass metric M and fork metric M/mbar, equivalent parameters
are rho_f = mbar*rho_u and lambda_f = sqrt(mbar)*lambda_u. Unit tests check the
independent mass-metric equations with nonzero initial multipliers, inhomogeneous
targets, a free FEM DOF, a constrained obstacle DOF, and mass scales 1e-9, 1 and
1e9. They use the existing form scale of one; PF-05 remains separate.

An additional local executable links the actual fork form and a renamed copy
of upstream BCLagrangianForm from commit
`6c9e7a39063e844d86e2d6938329efab98dd506b`. The upstream fixture receives the same
single target-slicing correction (the uncorrected constructor can assert before
the comparison). Across converted penalties, multiplier updates and coupled
quadratic subsolves, the maximum absolute energy/gradient/Hessian/solution
difference was **4.44089e-16**. This checks algebraic equivalence, not the
production AL feasibility controller.

All five solver smokes exited zero: quasistatic adaptive, semi-implicit,
Hessian-scaled AL, friction, and transient semi-implicit. A local copy of
`run-smoke.sh` directs results to `outputs/pf-03/smokes/`, preserving prior smoke
outputs. The real-binary Houdini PolyFEM 2.0 end-to-end test also passed,
including JSON round-trip import, with its own temporary output directory.
`clang-format --dry-run --Werror` and `git diff --check` passed.

## Scope and provenance

Started from PolyFEM `c27f9552d` on main with no tracked incoming edits. Existing
untracked simulation outputs were left in place. The effective IPC checkout is
`9da3094a46bcc054cc19024a5c748557c5bb6b9e`; the local PolySolve override is clean
at `012658e5d95b086f20c8e0e5f247473a2a229825`, matching the recipe pin. Dependency
pins and solver settings are unchanged.

This validates PF-03's normalization and parameter-conversion contract. It does
not establish mesh independence or physical accuracy, reinterpret the penalty
as inertia, change contact-floor behavior, or require intermediate AL energy
convergence. No new Ballburst or Teseo runs or full unit-suite pass are claimed.
Teseo remains excluded unless the user explicitly requests it.

Local baseline/build/test/smoke logs, upstream fixture and build script are in
`outputs/pf-03/` in the parent workspace. Reproduce the committed focused tests
from the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
```
