# PF-05: nonunit BC AL form-scale derivatives — 2026-09-07

The BC augmented-Lagrangian gradient now applies the inverse form scale once
to each term, matching the existing objective and Hessian. For constraint
residual r = Ax - b, normalized metric M, penalty rho and form scale s:

```
E = (-lambda^T sqrt(M) r + (rho/2) r^T M r) / s
g = A^T (-sqrt(M) lambda + rho M r) / s
H = (rho/s) A^T M A
```

Previously the quadratic part of g had an extra factor 1/s. The same expression
is present in upstream commit `6c9e7a390`; this is an inherited defect. Current
`NLProblem::normalize_forms()` returns one, so this correction is not evidence
that the defect caused the recorded production scene failures. Normalization,
multiplier updates, BC metric construction and solver/contact policies are unchanged.

## Reproduction and validation

The new `[bc_scale]` regression uses scales 0.5, 1 and 2, nonzero multipliers,
nonzero prescribed targets, unequal metric weights and a free DOF. It explicitly
checks the sliced target values before testing derivatives; PF-03's existing
constructor correction preserves them. Central differences of the objective
check the gradient, and central differences of the gradient check the Hessian,
using step 1e-5 and absolute norm tolerance 1e-8.

Before the production edit, four of 24 assertions failed: gradient and Hessian
consistency at both nonunit scales. All scale-one checks passed. The gradient
error norms were 9.12414 at scale 0.5 and 1.14052 at scale 2; corresponding
Hessian consistency error norms were 9.48683 and 1.18585.

After rebuilding `PolyFEM_bin` and `unit_tests`, the affected suite passed:
**15 cases, 313 assertions**, including the new scale test, BC metric tests,
AL termination tests and semi-implicit contact/friction derivatives. Expected
failure diagnostics in the AL negative tests are not suite failures.

All five solver smokes exited zero with zero error log lines. The real-binary
Houdini PolyFEM 2.0 end-to-end test passed, including JSON round-trip import.
`clang-format --dry-run --Werror` and `git diff --check` passed.
No full unit suite, Ballburst or Teseo run is claimed. These checks validate
the derivative correction and affected regressions, not physical accuracy.

## Provenance and reproduction

Started on main at `3716f099a`, with no tracked incoming edits. Existing
untracked simulation outputs were preserved. Effective IPC is clean at
`9da3094a46bcc054cc19024a5c748557c5bb6b9e`; the clean PolySolve override is
`012658e5d95b086f20c8e0e5f247473a2a229825`. Dependency pins are unchanged.
The parent workspace's `outputs/pf-05/` contains incoming status, baseline
failure evidence, build/test logs and smokes directed to a fresh output folder.

From the workspace root:

```sh
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
polyfem/build/tests/unit_tests '[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
```
