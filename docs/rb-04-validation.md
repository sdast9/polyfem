# RB-04 — Accepted-step physical accounting and diagnostics

Dates: 2026-09-08–2026-09-09
Status: **characterized—limits documented; opt-in endpoint and path instrumentation validated**
Continuation: [candidate research log](rb-04-research-log.md) records the user's
coherent-model direction, trim-band hypothesis, 15-run pilot, and exact remaining
event/work-accounting tasks. The pilot does not close trajectory accounting.
Historical initial stage: inventory and opt-in version 1 returned/failed endpoint records,
with independent public-fixture measurements. See the final dated disposition for the completed bounded accounting scope
and remaining accuracy limitations.

Current user-selected direction: improve the existing adaptive barrier; Fixed
mode remains a reference and AL is deferred. See the
[current strategy](rb-04-candidate-comparison.md#current-agreed-direction-2026-09-09).
No production model change follows from the comparison results alone.

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

The tested implementation/source is committed as `8f67bc191c5e3fa4861e32b6552e895611ecdb75`.
It includes the endpoint implementation, new test, runner/compact results,
contract and validation record, and the RB-04 status row, for publication to
`https://github.com/sdast9/polyfem`, branch `main`. Final source and binary hashes
were rechecked after committing; they match the tested manifest. A subsequent
documentation-only commit records this implementation identity. Companion source/pins and HDA
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

## Coefficient-event continuation — 2026-09-09

The user chose the coherent barrier-model direction and requested continued
RB-04 testing plus durable hypotheses/results. The
[research log](rb-04-research-log.md) contains the prior 15-run trim-band pilot
and exact interpretation/continuation. No new coefficient law or production
default has been selected. This stage instruments existing behavior only.

Started clean at `06545b81a`; effective IPC local `af317a65`, PolySolve local
`4d372fa8`, same compiler/build settings. No incoming solver/build process.
Evidence: parent `outputs/rb-04/20260909-coefficient-events/`. Final runs are
`force-endpoints/`, `affected-force-tests.log`, `force-smokes/`, and `force-hda.log`.
Final manifest/patch are `tested-force-manifest.json` and `tested-force.patch`.
Earlier energy-only development runs remain separate. First build failed only
because a new Catch matcher header was missing; final build completed both targets.

The opt-in event stream measures before/after objective energy and gradient at
identical coordinates with private snapshots, physical-unit energy change and
free contact-force change. It observes outer refresh/calibration/stall/post-step
operations once, including refreshes after endpoint publication. Initial prior
snapshot, direct external setters, embedding, time-weight changes and
coordinate-only feature transitions remain explicitly outside the stated scope.
The [contract](rb-04-contract.md) specifies those limits and phase semantics.

| Final validation | Result |
| --- | --- |
| Build PolyFEM_bin/unit_tests | Exit 0 |
| New real-form event checks within affected selection | 171 assertions across weights .25, 1, 4 |
| Full affected contact/cache/mapping/AL/BC/diagnostic selection | 2178 assertions / 31 cases, exit 0 |
| Three public off/on pairs | All six runs complete four steps; maximum displacement difference 1.565e-14 |
| Independent endpoint energy/det(F)/BC/reaction/kinetic reference checks | Original thresholds pass |
| Deliberate failure off/on | Both exits -6; interrupted reduced solve, one restart, no accepted step 1 |
| Event stream checker | Contiguous IDs, arithmetic/scaling, phase coverage, endpoint contact energy/force agreement and failure retune pass |
| Five standard smokes | All exit 0, zero error lines, four steps through t=1 |
| HDA end-to-end, Houdini 22.0.429 | Pass against final solver binary |

The new test checks linear energy scaling under doubled driving H against an
independent unobserved control, no-op refresh, nested retune counted once, changed
closest feature, provider-call counts, observation failure isolation, operation
exception retention, and identical subsequent energy/gradient/Hessian. No
production tolerance was relaxed. Observation jobs overlapped; wall times are
not controlled overhead comparisons. No assets/dependencies were modified.

Compact [event results](../tools/rb04/coefficient-events-results-20260909.json):
30 events in each frictionless run, 35 in friction, 4 in the deliberate failure.
Each has one unavailable initial before-state. The earlier friction development
run had 36 events; endpoint equivalence does not imply identical iteration count.
Tiny raw no-op energy differences are screened for reporting at
1e-10*(1+abs(before)+abs(after)) in objective units, not counted as model changes.

At the final frictionless quasistatic saved coordinates, the subsequent refresh
changes barrier energy by +70.8181 and free contact force by norm 145746.710 in
internal energy/force units. Earlier steps have force changes 10042.349,
44707.295 and 86186.957. Before-refresh contact energy/forces agree with the saved
endpoint diagnostics, whose free residual is approximately 1e-7. This establishes
a difference between saved and post-refresh contact models; it does not quantify
engineering displacement error or prove a cause for a historical scene failure.
Transient/friction counterparts are recorded separately; contact-force change
is not the total residual after an integrator or friction-state update.

RB-04 remains in progress. Optimization-event energy sums are not a physical
work budget. Continue with an explicit physical-path/contact-state convention,
initial-state accounting, external and friction work, and load/time refinement
of the selected bands. Per-trial proposals, every internal subsolve identity,
feature-switch accounting and the existing VTU velocity mismatch also remain
open. The new evidence supports a candidate lifecycle requirement: report a
returned equilibrium with the coefficient state under which it was solved;
any changed coefficient state must be explicitly labeled and, if used as the
claimed equilibrium model, solved/assessed accordingly. It does not preselect
the new coefficient scale, interpolation law or numerical stopping policy.

## Load/time refinement continuation — 2026-09-09

At unchanged production `9e7c3b59f`, the three-band/three-increment/three-fixture
matrix completed 23/27 runs; two additional repeats preserved the quasistatic
failures. All six failures occur before contact activation at step 1 after
20 restarts, with observed trim=1. No timeout, tolerance change or failed-run
replacement. This does not establish a causal trim-band failure boundary.

The default band's frictionless quasistatic final reaction varies ~.012% across
dt=.25,.125,.0625, while the friction reaction varies ~1.93% and updated-lag
residuals remain nonzero. Independent P1 energy/det(F)/BC reconstruction passes
at 188 accepted endpoints; transient inertia-work/kinetic/implicit-Euler
dissipation reconstruction passes at 84 endpoints with max absolute inertia
work discrepancy 7.82e-14. These are targeted comparisons, not an accuracy
envelope, converged mesh study or complete contact/friction work budget.

See the [research log](rb-04-research-log.md) for full numerical findings and
retained postprocessor correction, [work convention](rb-04-work-convention.md)
for signs/force-state limitations, and
[compact data](../tools/rb04/refinement-results-20260909.json). Exact runner
sources/input hashes/commands/exits and raw outputs remain in the four dated
refinement evidence folders. Only standalone analysis tools/docs changed;
the real solver was exercised without rebuilding unchanged production sources.
The next measurements are pre/post lag force pairs and a common-coefficient
energy decomposition at physical endpoints. RB-04 and candidate selection remain
in progress; no new production coefficient or friction policy is selected.

## Physical coefficient/friction-state pairs — 2026-09-09

Starting clean source `798cd4bd6` and unchanged effective dependencies. Only
the opt-in VarForm writer/observations change production C++; no coefficient,
friction, lag budget, acceptance or default changes. New fields are specified
in [the contract](rb-04-contract.md) and [work convention](rb-04-work-convention.md).
Evidence: parent `outputs/rb-04/20260909-physical-state-pairs/`.

| Validation | Result |
| --- | --- |
| Rebuild solver/unit_tests | Exit 0 |
| Affected contact/cache/mapping/AL/BC/diagnostic selection | 2178 assertions / 31 cases, exit 0 |
| Three public diagnostic off/on pairs | All six complete four steps; max displacement difference 2.004e-14 |
| Independent baseline endpoint measurements | Original energy/det(F)/BC/reaction/kinetic tolerances pass |
| Deliberate restart failure off/on | Both -6, no accepted step 1; original failure contract preserved |
| Five public smokes | Exit 0, zero error lines, all four steps through t=1 |
| HDA E2E with final binary | Pass |
| Common-snapshot alternate reconstruction | 75 applicable endpoints; max energy discrepancy 1.137e-13 |
| Paired-state refined runs | 9 complete trajectories / 84 endpoints across three types and increments, plus one retained failed fine friction attempt |

The alternate common-snapshot reconstruction uses the independent solve-start
event energy and final/start global trim ratio when the refresh ID stays fixed;
the writer separately rebuilds/evaluates the endpoint snapshot at start coordinates.
Paired post-update friction gradients agree with the separately recorded endpoint
friction component under the original 1e-9 relative-plus-absolute reference screen.
The energy decomposition is a bookkeeping identity, not a proof of physical balance.

The fine default-band friction attempt failed at step 1 before contact, with
20 restarts; one identical new run completed all 16 steps. The failure is retained
alongside the repeat, not replaced or attributed to a band, thread or sleep cause.
No test tolerance changed. Jobs overlapped during validation; timing is not an
overhead benchmark. A misspelled initial output-path command failed directory
creation before launching a solver and was corrected using cwd-derived paths.

Key result: the coarse friction endpoint has pre-update-friction residual
1.236e-8 and updated-lag residual 59485.575. The difference between accumulated
pre/post friction resistance-work estimates (9289.886) accounts for the earlier
updated-lag virtual-work discrepancy in that fixture. This identifies a force-state
distinction, not a failure of the completed frozen-lag minimization or a physical
accuracy certification. The finite-lag approximation remains increment-dependent.
The three-increment table and coefficient-state energy terms are in the work
convention and [paired results](../tools/rb04/physical-state-pairs-results-20260909.json).

Remaining for candidate determination: fixed-snapshot gradient-path integration
versus feature jumps, a concrete coherent-coefficient comparison, and an informative
upper-band fixture. These observations make the comparison interpretable but do
not repair RB-02's discontinuity/zero-median/unit counterexamples or RB-03's
unresolved interpolation law. RB-04 remains in progress; do not replace the
user's finite-lag policy with a global coupled-gradient requirement.


## Coherent-candidate boundary comparison (2026-09-09)

Standalone tool stage against `e652fae53`; no production edits. Both the original
66-check and extended 68-check probes passed. Energy/gradient jumps, independent
finite differences, fixed/semi uniform-curvature normalization, unit conversions
and split midpoint path integration support advancing the fixed-positive-scalar
barrier as a comparison candidate. The existing feature-dependent law retains
an approximately 44.49774 energy/work discrepancy across the RB-02 transition;
the Fixed k=70 mismatch decreases to 7.98e-7 with quadrature refinement.
See [candidate comparison](rb-04-candidate-comparison.md) and its linked data for
all measurements and limitations. No scene coefficient or default was changed;
full trajectory/model validation remains pending.


## Sequencing after roadmap revision (2026-09-09)

RB-04 remains in progress. RB-05 resource containment can proceed using available
diagnostics; completing RB-04 is not its prerequisite. The next mechanical-scaling
research item is RB-13 stage 1, which can also start with current evidence.
Remaining trajectory work must define/validate discrete work accounting and its
integration-error limits for the required fixtures, retaining coefficient and
friction-lag state identity. Supply the relevant measurements for RB-17/RB-09
physical conclusions. Do not claim full friction certification or rerun completed
endpoint/transition probes merely because the plan was revised.


## Discrete trajectory budget — completed postprocessing stage (2026-09-09)

Existing `e652fae53` binary evidence only, 9 full trajectories / 84 endpoints,
plus the retained failed fine-friction attempt with unavailable totals. No new
solver run, source/default change, or tolerance relaxation. The new analyzer
checks the complete right-endpoint algebraic budget including elastic/contact
remainders, parameter-state changes, solved-lag friction and independent inertia.
Independent elastic stress-path integration errors fall from .8932054 (2-point)
to 4.248e-7 (4-point) to 1.783e-10 (8-point); all endpoint comparisons pass.
Three analytical/negative controls pass. Maximum budget arithmetic error is
2.911e-11; maximum step equilibrium-work defect 7.305e-7. These are targeted
checks, not complete physical energy certification. No C++ rebuild/smokes needed
for standalone postprocessing of unchanged solver artifacts.

See [work convention and exact next stage](rb-04-work-convention.md#discrete-trajectory-budget-and-elastic-path-reference-2026-09-09)
for equations, inputs, commands, detailed results and restrictions. The contact
remainder still needs actual scene-path gradient quadrature to distinguish
integration error from frozen-coefficient feature jumps. RB-04 stays in progress.


## Actual contact-path measurements and RB-04 disposition (2026-09-09)

**Status: characterized—limits documented; opt-in instrumentation validated.**
This closes the current bounded RB-04 accounting investigation, not physical
certification of the contact model. Later model comparisons may require tighter
quadrature or more fixtures; the limitations below must travel with the data.

The new `output.physical_diagnostics_contact_path` option defaults false and only
acts with `physical_diagnostics=true` at returned accepted endpoints. It evaluates
1025 private snapshots on x(t)=x_start+t*(x_end-x_start), retaining endpoint
coefficient/trim state. New stencil coefficients use the same frozen snapshot
rule; no stiffness refresh or production cache update occurs. Energies and
path-directional gradients are recorded in objective units; divide by the saved
acceleration scaling for the physical-energy convention. Failures are explicitly
unavailable and do not invalidate the rest of the endpoint record.

Nested trapezoids use 16/64/256/1024 panels. The postprocessor also applies
composite Simpson integration to the same samples. Sorted collision-key signatures
identify changed grid intervals, then 24 bisections narrow a detected transition.
Store both sides, energy difference, and whether multiple signatures appeared.
A bracket is no wider than 2^-34 in normalized path coordinate. This is bounded
sampling, not an exhaustive event detector, collision-path certificate or proof
of continuous energy for arbitrary contacts. The straight segment is an accounting
path between time endpoints, not the solver's Newton path or a resolved physical
trajectory within the timestep.

**Validation:** final formatted build passed; affected suite 2191 assertions /
31 cases passed. All three public off/on pairs completed four steps with maximum
displacement difference 1.424e-14, within the unchanged 1e-10 screen. Paired
intentional failures both returned -6 without an accepted step 1. All five public
smokes had exit 0, zero error lines and PVD times 0,.25,.5,.75,1. Real Houdini
PolyFEM HDA E2E passed. No dependency, coefficient, friction-lag, stopping rule,
CCD, trial-cap or default behavior changed. No private scene ran. Diagnostics are
expensive when enabled; overlapping validation jobs are not timing benchmarks.

The separate detector control reuses the exact RB-02/04 transition geometry:
4 assertions passed, recovering the Semi energy jump -44.497739402978 and zero
for Fixed k=70, with unchanged production snapshot state. This tests sensitivity
to the known jump, not reliability for every degenerate multi-feature event.

| Public path | Step-1 energy minus Simpson work, 1024 panels | Maximum absolute same error, steps 2–4 |
| --- | ---: | ---: |
| Quasistatic frictionless | .001310237 | 3.980e-13 |
| Transient frictionless | .001308053 | 5.685e-14 |
| Quasistatic friction | .001310237 | 3.837e-13 |

First-contact activation occupies a short part of the coarse displacement
segment. In quasistatics its trapezoidal mismatch decreases from -270.916 at
16 panels to -.119055 at 1024. Simpson mismatch at 16/64/256/1024 panels is
-162.694 / -2.48885 / -.0105961 / +.00131024; its sign change means no monotonic
error/extrapolation guarantee is claimed. The last value is about 2.44e-5 of that
segment's contact-energy change (~53.75). These are measured discrepancies from
the actual energy difference, not a preselected engineering acceptance tolerance.
Do not claim the earlier 1e-9 energy screen is met by first-contact quadrature.

Detected feature-bracket energy differences after subtracting the tiny smooth
bracket work have maximum magnitude below 5.0e-12 on these 12 segments. Many
intervals contain multiple signatures (including nearly all later-segment
brackets), so the detector cannot isolate every geometric tie/switch. Agreement
of integrated work and endpoint energy supports a small net discrepancy on the
sampled later segments; it does not rule out unobserved cancelling jumps. These
public trajectories do not exhibit the large net jump in the synthetic RB-02
counterexample. Keep that counterexample open for RB-15.

**Accounting conclusion:** the large right-endpoint normal-contact remainder in
these coarse public trajectories is predominantly integration error. Independent
elastic path work and implicit-Euler mass work were verified in the preceding
stage; parameter-state changes and solved-/updated-lag friction work remain
separate. Actual contact-path integration now quantifies its residual error and
detected jump limits. No universal energy conservation, converged friction law,
mesh accuracy or optimal trim band is established.

Evidence: parent `outputs/rb-04/20260909-contact-path/`, with build/test logs,
input/binary/source hashes in `tested-manifest.json`, raw on/off/failure runs,
full path samples and signature brackets, `path-summary.json`, and
`detector-control/` source/executable/commands/results. The copied helper
`candidate_probe.cpp` is required to reproduce the detector probe. Small portable
results are [contact-path-results-20260909.json](../tools/rb04/contact-path-results-20260909.json);
full signature data stay in the local evidence. Sources:
[analyzer](../tools/rb04/analyze_contact_path.py) and
[detector control](../tools/rb04/contact_path_probe.cpp).

**Handoff:** no more RB-04 work is required before RB-05 or RB-13. Start RB-13
stage 1 for the agreed mechanically informed adaptation research. Tighten actual
path quadrature near activation if a later RB-09/17 comparison needs error below
.00131 on these coarse segments; use exhaustive event isolation if making stronger
continuity claims. Friction reversal/stick-slip and coupled-lag accuracy remain
RB-10, and unsupported maps remain subject to RB-03. Existing missing diagnostic
fields retain unavailable reasons; a populated arithmetic budget is not an
automatic `physical_balance_pass`.
