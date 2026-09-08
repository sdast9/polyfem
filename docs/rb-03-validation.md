# RB-03 — Collision/FEM coordinate mapping

Date: 2026-09-08
Status: **characterized—decision pending**
Selected stage: mapping contract, bounded counterexamples and comparative
curvature definitions. No production repair; exact indexing repair remains pending.

## Contract and authorization

The user selected RB-03 from [the robustness plan](robustness-plan.md).
Read RB-01/RB-02 records, PF invariants, floor retirement and workspace/HDA
instructions. RB-01's production repair is present; the live same-process tests
pass. RB-02's coefficient law/lifecycle decisions remain unresolved.

[The coordinate contract](rb-03-contract.md) derives dimensions, ordering,
chain rules, prescribed offsets, the meaning of proxy IDs, and curvature
alternatives. The invariant is compatibility of geometry, derivatives and driving
curvature. Matching frozen contact derivatives does not prove stiffness validity.

No coefficient or friction law, retune policy, stopping criterion, CCD, trial
cap, default resource limit, dependency or HDA source was changed. The constraint
floor remains retired. No private scene, Ballburst or Teseo was run.

## Baseline and reproduction

Started with clean `polyfem:main` at `b37d9f745`. The latest production-source
change is RB-01 `3abfcfe25`; subsequent commits are characterization/docs.
Effective IPC is clean detached `9da3094a46bcc054cc19024a5c748557c5bb6b9e` in
`~/.cache/CPM/ipc-toolkit/0c20`, matching the recipe. The companion checkout is
clean on `semi-implicit-stiffness`. PolySolve is clean
`4d372fa8a73f42bc224e31d464f1a308e1159ba8`, `iteration-callback`, via the existing
`CPM_polysolve_SOURCE` override to `../polysolve-merged`, matching its recipe.

A pre-existing `PolyFEM_bin -j params.json -o ../output/ --log_level 1` process
(PID 84378) was running. It was left untouched. All new probe objects/executables
and scene outputs went to isolated evidence directories. No shared binary,
library, dependency or build configuration was rebuilt or replaced. The new
C++ characterization executable was compiled and linked against the existing
libraries before testing; no production C++ changed.

Build: macOS arm64 Apple Clang, RelWithDebInfo, TBB and Accelerate. The runner
saves the exact compiler/flags/link recipe from `build/tests/unit_tests`.
The probe is serial at the driver level; library TBB behavior is retained.
Public smokes retain `Eigen::SimplicialLDLT` and their original schedules.
HDA installation identity was not remeasured (no HDA changes).

Evidence relative to the parent workspace:
`outputs/rb-03/20260908-123929/`. It contains baseline repo/process and binary
hashes, CMake cache, copied per-run probe sources and exact commands, compiled
executables, JSON/stderr/exits, linked-library hashes, fixture hashes, unit-test
logs, copied smoke script, VTK outputs and independently extracted PVD summaries.
The baseline/final library and solver/test binary hashes match. Only the small
[final JSON](../tools/rb03/results-20260908.json) is published with the probe.

## Findings and changes

| Finding | Measurement | Outcome |
| --- | --- | --- |
| Frozen contact gradient/Hessian pullback | Identity, selection, permutation, interpolation; actual external builder | Verified within fixtures |
| Virtual work | `g_u dot du = g_y dot B du`; error at most 5.56e-17 | Verified |
| Permutation curvature | Current 71.6667; exact coordinate transform requires 26.6667 | Reproduced indexing defect; **unrepaired** |
| Interpolation curvature ambiguity | Current 71.6667; transpose direction 88.3333; minimum-energy lift 218.3333; minimum-norm lift 338.3333 | Characterized; model decision pending |
| Appended obstacle index spaces | FEM-only Hessian produces 83.3333 because one obstacle proxy ID aliases a FEM node | Same indexing concern reproduced; no unique interpolation reference selected |
| Actual prescribed motion | Obstacle edge moves +.05 in y at t=1; FEM input displacement remains zero | Builder/Obstacle behavior verified |
| Actual reduced `NLProblem` | Four free DOFs, four prescribed DOFs; exact offset and projected derivatives | Verified |
| Contact action–reaction | Moving fixture FEM y force +1496.54050756; obstacle −1496.54050756 | Full contact force balance verified |
| Prescribed contact objective rate | Analytical dE/dt=74.8270253780; FD=74.8270254007 | Verified; force work has opposite sign |

The probe tests actual forms without overriding their derivative, collision or
stiffness lifecycle methods. It supplies a constant synthetic Hessian. Most
mapping fixtures use 100 I across all full DOFs, including obstacle placeholders;
this isolates chain-rule behavior and is not a physically assembled obstacle
Hessian. A separate FEM-only Hessian fixture exposes the index mismatch.
The external builder reads real generated OBJ/HDF5 files; no FEM elements are
assembled or solved in those microfixtures. The supporting empty mesh object is
not used by the selected external-file branch.

For the same interpolated geometry, current sampling gives E=212.60031 and
full force norm=837.64693. The transpose-direction candidate gives E=262.04224,
force norm=1032.44854; minimum-energy lift gives E=647.68932, force norm=2551.90110;
minimum-norm lift gives E=1003.67123, force norm=3954.47269. These are measured
unit-barrier energy/gradient scaled by the candidate kappa at fixed geometry,
not nonlinear trajectory comparisons. Units are consistent objective units and
objective/length; dhat=1 and trim=form weight=1. No candidate was installed.

## Validation

| Check | Expected criterion | Measured result | Exit |
| --- | --- | --- | --- |
| Final compiled mapping probe | Explicit analytical/FD checks and expected counterexamples | 97 checks pass | 0 |
| Full energy-gradient FD | Relative error <1e-6 | Maximum 1.06e-10 | 0 |
| Energy-only mixed second differences | Relative error <1e-5 | Maximum 3.77e-7 | 0 |
| Actual reduced problem derivatives | Gradient <1e-6, Hessian <1e-5 | 7.63e-11, 3.46e-11 | 0 |
| Rigid translation/rotation | Frozen energy error <1e-9(1+E) | Max rotation error 5.69e-14; translation zero | 0 |
| Affected baseline plus `[contact_cache]`, seed 1 | No assertion failure | 26 cases / 1,892 assertions pass | 0 |
| quasistatic-adaptive | Four steps to t=1 | Four steps; zero error lines; 3.5820 s | 0 |
| quasistatic-semi-alhess | Four steps to t=1 | Four steps; zero error lines; 1.1459 s | 0 |
| quasistatic-semi-friction | Four steps to t=1 | Four steps; zero error lines; 1.3166 s | 0 |
| quasistatic-semi | Four steps to t=1 | Four steps; zero error lines; 1.4166 s | 0 |
| transient-semi | Four steps to t=1 | Four steps; zero error lines; 1.2342 s | 0 |
| C++ formatting, Python syntax, diff whitespace, local Markdown links | Changed-file checks | Pass | 0 |

All scene PVD times are `[0,.25,.5,.75,1]`; each process exit was inspected.
Times are solver log totals during a concurrent user simulation, not controlled
performance comparisons. Expected negative-test solver error messages did not
fail assertions. No baseline measurements/goldens were regenerated.

Tolerances were encoded before each run: direct map comparisons 1e-10 or tighter,
central energy gradients with step 1e-6 and error/(1+norm) <1e-6; mixed energy
second differences with step 1e-4 and error/(1+norm) <1e-5. Geometry gap .2,
support 1 and moderate stiffness avoid near-contact singular conditioning.
No threshold was relaxed. Finite rotations are at frozen isotropic coefficients,
not evidence of retune covariance for arbitrary anisotropic material laws.

The first probe passed 80 checks; the expanded probe passed 91. The next
`final-probe/` stopped after 55 checks with an invalid 2D rotation error. Its
external transformation object omitted defaults normally supplied by schema
processing. The probe was corrected to specify dimensions=null, scale=1,
rotation=0 and translation=[0,0]. This is a fixture correction, not a production
repair. All earlier logs remain; their external-builder results are superseded
by `schema-complete-probe/`, which passes 97 checks. That final source/result is
published. The failure is not attributed to threads or the concurrent simulation.

Measured physical quantities are contact forces/reactions, local energy and its
prescribed-motion derivative at specified coordinates. Full continuum residuals,
solved BC accuracy, det(F), accepted-step gap histories, total work balance,
friction dissipation and mesh/time refinement were **not measured**. Smokes are
numerical completion evidence, not an independent physical-accuracy certificate.
No whole suite, HDA end-to-end, high-order, periodic, 3D, other-platform or private
scene validation is claimed. No HDA workflow/production change requires a new
HDA build in this stage.

## Publication and next stage

Publish `tools/rb03/`, the coordinate contract, this record and the RB-03 plan row
to `https://github.com/sdast9/polyfem`, branch `main`; the task completion reports
the commit. Update the parent workspace README locally. No companion or asset
publication is required. Generated libraries/binaries/large evidence stay local.

This stage completes the bounded characterization and real-map derivative
checks, **not an implementation repair**. RB-03 remains characterized—decision
pending. An exact selector/permutation indexing fix remains authorized and can
be isolated from the unresolved interpolation choice. It should expose/use a
sparse map with explicit index spaces, preserve identity behavior and obstacle
handling, and demonstrate baseline failure then corrected equivalence before
publication. The current probe deliberately asserts the existing mismatch;
convert that assertion to the reference expectation when implementing the fix.

For interpolation, select an admissible displacement/curvature definition from
[the comparison](rb-03-contract.md), specifying fixed DOFs, rank deficiency and
indefiniteness, before introducing a lift/inverse/projection. No such choice is
implied by this record. RB-04 can reuse the established derivative maps and
prescribed-work convention; physical stiffness support remains limited. No
arbitrary high-order or remeshed support is advertised.
