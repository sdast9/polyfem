# RB-04 — Accepted-step physical accounting and diagnostics

Dates: 2026-09-08–2026-09-09
Status: **in progress — endpoint instrumentation validated; trajectory accounting pending**
Selected stage: inventory and opt-in version 1 returned/failed endpoint records,
with independent public-fixture measurements. This does not close all RB-04
acceptance stages.

## Contract and authorization

The user selected RB-04 from [the robustness plan](robustness-plan.md), then
requested continuation. The [version 1 contract](rb-04-contract.md) defines
record timing, coordinates, units, signs, supported forms and unavailable fields.
Read PF invariants, floor retirement, RB-02 coefficient inventory and RB-03
mapping/repair records. Exact-selector stiffness indexing is repaired at the
starting commit; interpolation curvature and RB-02 model decisions remain open.

No coefficient/friction law, lag budget, tolerance, CCD, constraint-floor behavior,
trial cap, production restart policy, time schedule or dependency pin changed.
No physical convergence gate was added. No private scene, Ballburst or Teseo ran.
No HDA source or asset changed.

## Baseline and reproduction

PolyFEM began clean on `main` at `245592c02`. IPC recipe and effective local source
are `af317a65d69d0ac7c5efa4bf103bf75e280c323b`,
`../ipc-toolkit-fork`, branch `semi-implicit-stiffness`; this supersedes the older
CPM-cache baseline in the plan. PolySolve recipe and effective local override are
`4d372fa8a73f42bc224e31d464f1a308e1159ba8`, `../polysolve-merged`, branch
`iteration-callback`. Both companion trees were clean. No pre-existing solver or
build process was found at the start. Incoming data were not moved or deleted.

Configured macOS arm64, Apple Clang, RelWithDebInfo, TBB/Accelerate; build with
six jobs. Fixture linear solver is unchanged `Eigen::SimplicialLDLT`.
The new Python runner used Homebrew Python 3.14 and NumPy 2.5.1.

Evidence directory in the parent workspace:
`outputs/rb-04/20260908-accounting/`. Baseline repo/remote and executable identities
are in `baseline.json`; effective cache in `CMakeCache.txt`; final source/library/
executable hashes in `tested-manifest.json`, patch in `tested.patch`. Commands,
input SHA-256s, logs and exits are in each runner's results, build logs, focused
logs and the copied smoke script. Original fixture schedules/materials were
preserved. Output stats were enabled for baseline inventory. Paired checks add
only output options (stats, velocity, diagnostics) and isolated output paths.
The deliberate failure alone changes existing restart test options to
`soft_iteration_limit=1`, `max_restarts=1`, `min_iterations=0`.

Before production edits, all three public baseline fixtures completed four steps.
They emitted energy/runtime CSVs but no versioned physical-attempt record. Source
inventory confirmed that `EnergyCSVWriter` divides all form objectives by the
acceleration scaling; its inertia column is not kinetic energy. Existing output
owners are nonlinear VarForm solve/save paths, `SolverCSVWriter`, and
`ElasticVarForm::build_elastic_output_fields`.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| No versioned accepted/failed endpoint record | Three baseline output inventories and source trace | Implemented opt-in JSONL writer |
| Diagnostic contact rebuild can need new stencil coefficients | Real form test evaluates a different closest feature | Private snapshot/broad phase leaves production memoization and derivatives unchanged |
| Quasistatic steps still have acceleration scaling | Measured `s=.0625` despite disabled inertia | Explicitly divide weighted forces/energies by actual `s`; quasistatic kinetic energy unavailable |
| Finite friction lag returns without updated-lag equilibrium | All four friction endpoints have `state=not converged`; final updated-lag objective residual 3717.8484223013 versus existing .0025 lag tolerance | Reproduced and labeled, not changed into rejection |
| Final full free residual differs from earlier numerical stopping data | Final friction free force residual 59485.5747568211 after lag update | Recomputed/labeled; no physical threshold selected |
| Failed reduced subsolve leaves caller coordinates, not failed trial | Existing `ALSolver::solve_reduced` contract and exhausted run | Record explicitly labels retained caller/current-attempt state; captures interrupted subsolve and one restart |
| Nonlinear VTU velocity lags the saved displacement | Step 1 velocity field is zero; nonzero displacement-derived endpoint kinetic energy is 16.7842271784; exporter uses `v_prev()` before update | Reproduced existing output mismatch; unresolved, displacement-history reference used |
| Trajectory work and every coefficient jump are not retained | Source inventory | Explicit unavailable reasons; remaining RB-04 stage, not inferred balance |

The snapshot does not refresh stiffness, call the provider, update lagging or
advance integrator history. A monotonic per-form completed refresh ID is exposed.
The full residual sums supported enabled physical form gradients, excluding AL
terms, then uses the existing free-coordinate map. Other enabled forms make the
full residual explicitly unavailable. Pressure loading beyond empty boundaries/
cavities is not covered by this stage.

## Validation

Final source build: `build-release.log`, exit 0, both `PolyFEM_bin` and `unit_tests`.
Small published measurements: [results-20260909.json](../tools/rb04/results-20260909.json).
Reproduction: [tool README](../tools/rb04/README.md).

| Check | Expected criterion | Measured result | Outcome |
| --- | --- | --- | --- |
| New `[physical_diagnostics]` real-form snapshot test | Production state unchanged, independent reference/FD match, empty fields unavailable | 27 assertions / 1 case | Pass |
| Focused affected suite plus cache/mapping/diagnostics | No assertion failure, seed 1 | 2007 assertions / 30 cases | Pass |
| Three public fixture types, off/on | Four steps per run; displacement max difference <1e-10 | Six runs exit 0; max 2.19755e-14 over 12 endpoints | Pass |
| Independent P1 Neo-Hookean elastic energy | Relative error <1e-9 | Max 4.60077e-15 | Pass |
| Independent det(F) and BC reconstruction | Absolute difference <1e-10 | All 12 endpoints agree | Pass |
| Quasistatic support reaction from independent P1 stress assembly | Error/E <1e-9; top boundary has no contact/traction | Both quasistatic fixtures pass all steps | Pass |
| Transient kinetic energy from displacement history and exact P1 mass | Relative error <1e-9 | Max 1.06636e-15 | Pass |
| Residual equals recorded gradient component sum | Norm discrepancy <1e-8 | All 12 endpoints pass | Pass; not an independent full trajectory oracle |
| Failure off/on | Same failure; failed record; no accepted step 1 | Both CLI exits −6 (SIGABRT from uncaught solver exception); one restart; interrupted reduced status retained | Expected failure verified |
| Five standard public smokes | Four steps through t=1, no error lines | All exits 0; PVD times [0,.25,.5,.75,1] | Pass |
| HDA end-to-end test against final binary | Strict schema/export/solver/round-trip checks | `test_polyfem_hda.py` passes | Pass |
| Existing independent reconstruction tests | Rigid rotation and energy/reaction finite difference | 2 tests | Pass |
| C++ formatting, Python syntax, diff whitespace and local links | No introduced issue | Checked before publication | Pass |

All final BC errors are zero. Final sampled min det(F) is about .857117 for the
frictionless quasistatic/transient fixtures and .840574 for friction. These are
assembly-quadrature measurements; no continuous inversion or engineering-accuracy
certificate follows. Frictionless final free residuals are about 1.06e-7 internal
force units; the friction residual quoted above is after an unconverged lag update.
Reported numerical termination in these public runs is the existing gradient or
relative-gradient criterion. The recorder also preserves configured directional-
derivative exits when they occur; it adds no gradient-only acceptance gate.

Tolerances were declared in the test/runner before the comparisons. Contact FD
uses central h=1e-6 at a finite, fixed-feature gap with relative norm tolerance
1e-6; production/reference comparisons use 1e-10 absolute-plus-relative. The P1
reference reconstructs F and Neo-Hookean stress/energy from saved coordinates.
Kinetic integration uses `integral(N_i*N_j)=V*(1+delta_ij)/20` and the fixture's
ImplicitEuler velocity `(u_n-u_(n-1))/.25`; it does not use the stale VTU velocity.
No numerical tolerance was relaxed to get a pass.

Retained development results:

- `endpoint-checks/`: failed the new completeness assertion because the active
  empty pressure form was initially outside the supported list. Added support
  for empty pressure boundaries/cavities; nonempty loads remain unavailable.
- `endpoint-reviewed/`: quasistatic checks passed; kinetic comparison failed
  because it used the lagged exported velocity. This established the output
  mismatch above. `endpoint-displacement-reference/` then passed all eight runs
  using the independently derived endpoint velocity, without changing tolerances.
- `reference-tests.log`: the directory-specific `python3` resolved to system
  Python 3.9 without NumPy. `reference-tests-explicit.log` uses the same explicit
  Homebrew interpreter as the scene runner and passes both tests.
- `final-endpoints/`: passes all eight runs with final refresh/failure identity
  fields. All failed/development logs remain; none overwrite the earlier evidence.

Final validation jobs overlapped on this host; recorded wall times are run
provenance, **not** a controlled overhead benchmark. HDA installed asset identity
was not remeasured, since no asset was modified. No full solver/HDA suite, other
platform, mesh/time refinement or private-scene check is claimed.

## Publication and remaining work

Publish only this endpoint implementation, new test, runner/compact results,
contract and validation record, and the RB-04 status row to
`https://github.com/sdast9/polyfem`, branch `main`. Companion source/pins and HDA
assets stay unchanged. Parent workspace README is updated locally. The completion
message identifies the published implementation commit and synchronization state.

RB-04 remains **in progress**. Completed: baseline inventory, endpoint schema,
observational contact reconstruction, accepted/failure labeling, lag residual,
BC/reaction/energy/kinetic/det(F) measurements, and targeted equivalence checks.
Pending before closing RB-04:

1. Per-trial proposals/candidate history and per-subsolve attempt events, including
   failed internal coordinates rather than only retained caller state.
2. Every fixed-coordinate retune/refresh energy jump, kept distinct from mechanical
   work and RB-02 feature-transition discontinuities.
3. Initial-state and trajectory external/prescribed-boundary work, friction
   dissipation, declared quadrature and refinement/truncation-error measurements.
4. Complete residual/work reference and nonlinear output kinematics alignment;
   additional supported forms/mappings must carry their own evidence.

The next session should resume RB-04's **coefficient-event and work-integration
stage** from the checked-in endpoint contract. It must not infer a new coefficient
law, friction budget, interpolation curvature or physical acceptance threshold.
This endpoint stage alone does not satisfy the complete RB-04 prerequisite for
physical-balance/recovery certification in later RB items.
