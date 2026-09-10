# RB-13 — Mechanical coefficient estimate and conditional bounds

Updated: 2026-09-10
Status: **characterized — decision pending; stages 1–3 completed**
Current selected stage: stage 3 mechanical scope tests; analytical investigation complete.

## Stage 1 record — 2026-09-09

The following stage 1 evidence and handoff are retained as historical records.
The stage 2 and stage 3 continuations and current handoff appear below.

## Contract and authorization

The user selected “RB-13 stage 1.” The selected invariant is that an isolated,
stable, locally linear frictionless contact has a dimensionally consistent
compliance/predicted-demand reference, with the actual squared-distance barrier
converted to physical force and tangent stiffness correctly.
[The contract](rb-13-contract.md) supplies the derivation, integrator objective
scales, map/prescribed-motion requirements, source trace and exclusions.

Read the robustness plan, PF invariants/floor retirement, RB-02 coefficient
contract/validation, RB-03 coordinate contract/validation and RB-04 work convention,
candidate comparison and current handoff. Those records are prerequisites and
historical evidence, not newly repeated tests. RB-02 model choices, RB-03
interpolated stiffness and RB-04 broader physical accuracy remain unresolved.

No production coefficient/provider, defaults, schema, friction, solve timing,
stopping criterion, CCD, trial cap, resource limit, retry or HDA behavior changed.
The constraint floor remains retired. Conditional intervals and the full stage 3
matrix are outside this stage; no physical model choice is requested or made.

## Baseline and source identity

PolyFEM `main` started clean at `f9de2eb07`. Effective IPC is the clean local
`ipc-toolkit-fork` override on `semi-implicit-stiffness`,
`af317a65d69d0ac7c5efa4bf103bf75e280c323b`. Effective PolySolve is the clean local
`polysolve-merged` override on `iteration-callback`,
`4d372fa8a73f42bc224e31d464f1a308e1159ba8`. Both match their recipe pins.
No running solver/build was found in the initial process inspection. No incoming
user files, dependencies, branches or processes were modified.

The configured solver build is macOS arm64, RelWithDebInfo, `/usr/bin/c++`, TBB.
This stage instead compiles a tiny C++17/O2 executable with the same compiler and
the actual effective IPC barrier primitive source. It does not link/run the
PolyFEM solver, use a FEM linear solver or rebuild shared libraries. The mechanical
reference uses a serial Python standard-library Cholesky solve and bisection.
HDA identity was not remeasured because there is no HDA or workflow change.

Local parent-workspace evidence:
`outputs/rb-13/20260909-223711-stage1/`. It preserves initial repository history,
branches/remotes/status, process list, CMake cache and solver/test binary hashes;
`run.json` and `final-run.json` record exact runner commands, cwd, stdout/stderr
and exits. The `probe/` and `final-probe/` directories preserve compiler command/log, bridge input/output, source hashes,
platform/Python identity and result. `source-manifest.json` records the audited
production source hashes and verifies unchanged solver/test binaries. Checked-in
fixtures and [results](../tools/rb13/results-20260909-stage1.json) are portable;
the local evidence has absolute paths and additional host provenance.

There was no claimed new production defect to repair. The negative control
reproduces the error from **omitting 2 B_s in a proposed physical-gap tangent**,
while the compiled IPC derivative/chain-rule control agrees with independent
energy finite differences. No production source was edited before or after it.

## Findings

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Free-space compliance yields K_eff=1/(J H^-1 J^T) | Constrained quadratic derivation, solve and minimized-energy identity | Characterized within SPD/affine-gap model |
| Prescribed motion belongs in the predictor | Full 2-DOF quadratic with second DOF fixed at .1; d_free=.3 | Reproduced analytical reference |
| Coefficient, force and stiffness differ | K_eff=6, demand=1.2, k=.277172006700521, barrier tangent=4.18909612060939 | Characterized, not a production recommendation |
| Independent full free-force balance recovers target .5 | Bisection root .4999999999999999 | Verified within fixture |
| Physical barrier derivatives need both chain-rule terms | Correct unit tangent 15.1137056388801 versus 23.7725887222398 when 2 B_s is omitted | Incorrect derivation rejected; production primitive agrees |
| Provider already includes enabled inertia | SolveData, InertiaForm, assignment source trace; three integrator algebra controls | Double-counting risk identified; no provider change |
| Controller band is squared-gap/RMS based | Actual average/minimum/severity code and scalar controls | Source-traced; defaults map to .7071–.9487 dhat for one contact |
| Bounds, nonlinear accuracy, coupled contact and estimator quality | Not executed in this selected stage | Unresolved; stages 2–3 and downstream items |

The probe's fixed H, J, k and force evaluations share one state; it has no lagged
friction or coefficient updates. The spring example uses one consistent internal
length/force unit system and dhat=1. Its full quadratic is
1/2 [z,c]^T [[6,2],[2,5]] [z,c] - .8 z, c=.1, with d=.3+z-c.
The free equilibrium z=.1 gives d_free=.3. Demand at d=.5 is 1.2.
The second free-space fixture uses H=[[6,2],[2,5]] and J=[1,-.5]; it checks the
gap constraint and condensed energy, not stage 3's multi-contact coupling.

No measured uncertainty enclosure or rigorous coefficient interval is claimed.
The contract distinguishes zero compressive demand from permission to remove
contact protection and identifies singular/indefinite/zero-J exclusions. The
probe rejects scalar zero/negative H; broader rigid-mode treatment is pending.

## Validation

Predeclared in the script before its first run: direct identities use scaled
error abs(actual-reference)/(1+abs(reference)) <= 1e-11. Energy finite differences
use 60-digit Decimal arithmetic, central step 1e-5*dhat, force threshold 2e-7
after dhat^3 normalization and curvature threshold 2e-6 after dhat^2 normalization.
The 15 barrier points have d/dhat=.1,.25,.5,.8,.95 and dhat=.01,1,100. Moderate
gap ratios and the small well-conditioned SPD fixtures justify this bounded
truncation/roundoff screen; it is not a near-singular or arbitrary-scale bound.
No tolerance or fixture was changed to obtain a pass.

| Check | Measurement | Result |
| --- | --- | --- |
| Compile actual IPC primitive and bridge | C++17/O2, `/usr/bin/c++`; compiler exit 0 | Pass |
| Standalone analytical probe | 101/101 explicit checks, runner exit 0 | Pass |
| Physical-distance formula versus compiled IPC force | Maximum scaled error 5.44e-16 | Pass |
| Physical-distance formula versus compiled IPC curvature | Maximum scaled error 3.31e-16 | Pass |
| Independent energy-only FD force | Maximum scaled error 2.91e-9 | Pass |
| Independent energy-only FD curvature | Maximum scaled error 5.10e-9 | Pass |
| Spring equilibrium root | .4999999999999999, target .5 | Pass |
| Euler/Newmark/BDF2 scaling | Physical H=57,207,119.5; weighted/physical residual controls | Algebra verified, integrator classes not executed |
| Python syntax, C++ formatting, Markdown links, diff whitespace | Selected-file checks | Pass |
| Production solver build, unit suite, public/HDA smokes, private scenes | No production/workflow edit; not required for this standalone primitive/analytical stage | Not run |

The initial probe and a final rebuild/run after C++ formatting both passed all
101 checks, with byte-identical result JSON. There are no failed or partial
simulations to classify. No scene was run, including Teseo or Ballburst. No goldens or prior
measurement files were regenerated. Numerical bisection verifies a scalar
reference equilibrium only. Mesh det(F), continuum/free residuals, full reactions,
trajectory work, friction dissipation, timestep/mesh refinement, general mappings,
nonlinear prediction errors and other platforms were **not measured**.

## Publication and next session

Publish the bridge, Python reference probe, small result/README, contract,
validation record and updated RB-13 plan row to `sdast9/polyfem:main`. The task
completion reports the exact resulting commit; it is not inserted recursively
into this document. No production implementation, dependency or HDA publication
is involved. The parent workspace README is updated locally (not a repository).
The compiled bridge and large/local provenance stay in the isolated evidence
directory. Shared production binaries retain their baseline hashes.

**Next: RB-13 stage 2.** Extend the standalone reference with monotonic roots,
conditional bands and truly enclosing uncertainty inputs; test inactive demand
and empty intervals without choosing a fallback coefficient. Keep the stage 1
result as historical evidence. Stage 3 still owes spring-series/rigid limits,
differing approach speeds, full mechanical unit conversion, nonlinear errors,
Rayleigh comparison, multi-contact coupling and invalid-tangent scope tests.
The full RB-13 acceptance is not complete. RB-14/RB-15 now have stage 1's reference
equations/units, but any dependent implementation must retain its own prerequisite
and model-decision boundaries. RB-16's bounds prerequisite is still pending.

## Stage 2 continuation — 2026-09-10

### Authorization, baseline and invariant

The user selected “continue to stage 2.” This authorizes the conditional-band
derivation and standalone tests in RB-13, not stage 3 or production model changes.
Stage 1 is published at `2103108f3579a40c5ff8a3e37cecd75689d87501` on `main`;
the checkout was clean at that revision when this stage began. IPC remains
`af317a65d69d0ac7c5efa4bf103bf75e280c323b` and PolySolve remains
`4d372fa8a73f42bc224e31d464f1a308e1159ba8`, both clean and both matching their
configured local source overrides and recipe pins. No solver/build was running.
The stage 1 probe/result, production code, shared build and dependencies are
unchanged. Its earlier tests were not rerun as if they were new measurements.

Re-read the current RB-13 plan/contract/validation, workspace instructions and PF
invariants; the prerequisite/source audit from stage 1 remains applicable at this
unchanged production revision. Rechecked actual IPC barrier derivatives, recipe
pins and effective overrides. The selected invariant is that a reported band
interval covers the unique scalar force-balance root for its declared admissible
inputs, with inactive/mixed/empty cases reported explicitly.

The [extended contract](rb-13-contract.md#7-stage-2--conditional-bands-2026-09-10)
proves positive decreasing ordinary-barrier force, uniqueness and monotonicity,
the exact-input interval and the enclosing-rectangle theorem. This is the same
isolated SPD, affine-gap, frictionless model as stage 1. K, p, h, weights and
coefficient state stay fixed within each force evaluation/root solve; no lagged
friction or optimization-event state is present. No production k or model repair
is selected. Active-contact protection, CCD, the trial cap, stopping and all
production defaults remain unchanged.

### Evidence and protocol

New local parent-workspace evidence:
`outputs/rb-13/20260910-094044-stage2/`. It contains initial repository/process
state, exact revisions, build/binary/recipe/stage-1-source hashes, copied
predeclared protocol, runner command/cwd/exit/stdout/stderr, compiler provenance,
raw requests/responses, complete query metadata and summarized results.
`final-identity.json` verifies that every baseline-hashed file is unchanged.
The parent README is updated locally; it is outside the Git repositories.

The [protocol](../tools/rb13/stage2-protocol.md) and numerical tolerances were
written before the first run. The new C++ bridge was formatted, then compiled
with `/usr/bin/c++ -std=c++17 -O2` and effective IPC's actual `barrier.cpp`.
This is a serial scalar executable, independent of the shared FEM build.
Python uses standard-library Decimal at 60 digits for coefficient endpoints;
the bridge uses double-precision IPC forces and up to 120 bisections. Fourteen
selected roots have independent 120-step Decimal comparisons. Root gap/band
tolerance is 2e-11*dhat; residual tolerance is 2e-10 after dividing by
K*dhat+abs(k*f); normalized curvature identity tolerance is 1e-11.
No tolerance, input fixture or protocol changed after seeing results.

The finite numerical checks are not a formal floating-point certificate. The
rigorous statement is the exact-arithmetic theorem conditional on the declared
model and true input enclosure. These manufactured boxes define their entire
admissible sets by construction; they are not measured estimator error bars.
Printed Decimal endpoints approximate the exact logarithmic expressions.

### Findings and measured results

The new [probe/result](../tools/rb13/results-20260910-stage2.json) passed
**23,396/23,396 checks**. Its 2,286 bridge queries comprise **2,282 scalar
equilibria** and four expected rejections: k=0 with p=-.2 or p=0 has no
positive-gap equilibrium; K=0 and k=-1 are invalid reference inputs.
Six additional Python input checks reject invalid/reversed/nonfinite intervals.
The check total includes several assertions per root and 4,968 adjacent-pair
monotonicity comparisons; it is not a count of independent FEM tests or scenes.

| Check | Measured result | Outcome |
| --- | --- | --- |
| Exact inputs, three physical bands and three K values | 81 classifications; 270 in-band root samples | Pass |
| Three applicable nonempty/zero-only boxes | 11x11 K,p grid, five coefficient samples: 1,815 roots | Pass |
| Known band endpoints/manufactured targets | 133 checks; max gap error 2.23e-16 | Pass |
| Independent Decimal roots | 14 comparisons; max gap error 1.12e-16 | Pass |
| Scalar force residual | Maximum normalized magnitude 1.38e-15 | Pass |
| Compiled curvature versus Decimal expression | 2,228 checks; max normalized error 8.43e-16 | Pass |
| Positive-curvature proof bound | 2,228 interior-root checks | Pass |
| Monotonicity in k, p, K | 4,968 comparisons; expected directions retained | Pass |
| Tightness beyond positive bounds / zero-only upper bound | Exact and rectangle worst-corner roots leave the band as predicted | Reproduced |
| Fresh C++ build, Python syntax, C++ formatting, Markdown links, diff whitespace | Selected-artifact checks | Pass |

All quantities below use dhat=1, band [.4,.7], K in force/length and k in
force/length^3. Rounded bounds are displayed for readability; the result retains
the 60-digit values.

| K enclosure | p enclosure | Result |
| --- | --- | --- |
| [4,6] | [.1,.2] | Nonempty: [.3004512032,1.1351984474] |
| [4,6] | [.45,.55] | Zero lower bound: [0,.3405595342] |
| [4,6] | [.45,.7] | Formal zero-only interval [0,0]; not active-barrier deletion permission |
| [1,100] | [-.2,.65] | Empty: lower 10.0150401077 > upper .0283799612; no fallback chosen |
| [4,6] | [.8,.9] | Inactive band demand; no applicable interval |
| [4,6] | [.5,.9] | Mixed band applicability; no all-box certificate |
| [4,6] | [-.2,.9] | Mixed applicability with positive raw lower bound; no all-box certificate |

The empty box's upper coefficient gives d=.0027992943 at its lower-gap worst
corner; its lower coefficient gives d=.9634436237 at the upper-gap worst corner.
Neither result satisfies [.4,.7]. Both are retained counterexamples, not failed
production simulations or instructions to use those coefficients.

The raw [0,0] interpretation fails explicitly: p=.9,k=0 gives d=.9, above the
band. The code labels inactive/mixed applicability instead of certifying that
raw pair. This reproduces a **misinterpretation of a proposed formula**, not a
defect in the current production controller. No new production repair is claimed.

Enclosure is essential: using the positive box's lower coefficient with actual
K=100 outside [4,6] gives d=.1460237013, below .4. Empirical error bars cannot
be advertised as guaranteed bounds without evidence that they enclose the inputs.

Empty boxes also lose information about correlation. For the manufactured
family K in [1,10], p(K)=.5-f(.5)/K, one fixed reference k=1 gives d=.5 for
the entire family analytically; all 25 sampled members reproduce that root.
Its Cartesian bounding box instead has lower 7.0596711122 > upper .3592586105.
This establishes that an empty conservative rectangle does not prove the
correlated physical family is infeasible. k=1 is manufactured before the run,
not chosen as a fallback from the empty interval.

Finally, the mathematical increasing-force counterexample f=d^2 with K=k=1,
p=.1 has two roots, .1127016654 and .8872983346. It demonstrates the need for
the decreasing-force/stability assumptions; that force is not installed anywhere.

### Limits, publication and current handoff

The initial compiled probe and final rebuild/run both passed with identical root
outputs, counts and numerical metrics. The final result uses null for unmeasured
error magnitudes in Boolean-only groups, replacing the initial default zero;
no fixture, formula or tolerance changed. There were no failed runs, hidden
partial runs, changed thresholds or reruns replacing failures. Expected invalid/domain cases
are reported above. No production solver build, full unit suite, scene, private
input, Teseo, HDA test or asset rebuild was run. They are not required for this
isolated analytical/primitive stage; the shared solver and test binaries retain
their baseline hashes. No stage 1 result or golden was regenerated.

These are scalar equilibrium and conditional interval measurements. They do not
measure full-system residuals/reactions, mesh det(F), energy/work trajectories,
friction, nonlinear predictor accuracy, coupled contacts, timestep/mesh refinement,
real estimator enclosure, arbitrary weighted/interpolated geometry or other
platforms. Fixed k in each probe is a mathematical test state, not a replacement
of the agreed adaptive-barrier direction with production Fixed mode.

Publish the new bridge/probe/protocol/result, extended contract/record and roadmap
to `sdast9/polyfem:main`; the task completion supplies the exact commit. No source
under `src/`, dependency pin or HDA file changes. All local logs and executable
stay in the new evidence directory. Final publication state is recorded there.

**Next: RB-13 stage 3.** Verify spring series and rigid/deformable limits,
implicit-Euler masses with differing speeds, mechanical unit conversion,
nonlinear local-tangent error, compliance versus Rayleigh curvature, two-contact
coupling and invalid-H behavior. The full RB-13 acceptance remains incomplete
until that scope matrix is characterized. RB-14/RB-16 can use the conditional
interval semantics, but no practical enclosing estimator or controller policy
has been selected or validated by this stage.

## Stage 3 continuation — 2026-09-10

### Authorization, baseline and invariant

The user selected “continue to stage 3.” This stage completes RB-13's requested
mechanical scope matrix, not a production coefficient/default implementation.
Started clean on `main` at `052b0acb609d4222d7b879441b18100dff414a59`; effective
IPC and PolySolve remain the clean local overrides `af317a65` and `4d372fa8`,
matching the unchanged recipe pins. No running solver/build was found. The
workspace instructions, PF invariants, stage 3 requirements and existing record
were read; current IPC stiffness/derivative code, PolyFEM's provider/assignment
and implicit-Euler history/scaling were inspected. Earlier source/record context
continues to apply because no production source changed after stage 1.

The invariant is to establish the declared reference's behavior on supported
springs, dynamics, unit conversions and coupled contacts, while explicitly
reproducing nonlinear/local-model and invalid-tangent limits. The
[contract's section 8](rb-13-contract.md#8-stage-3--mechanical-scope-and-counterexamples-2026-09-10)
gives the derivations and distinguishes source-traced formulas from executed code.

No production model, provider, mapping, coefficient timing, friction, stopping,
resource/retry policy, CCD, trial cap or asset changes. The constraint floor
remains retired. No negative force is clamped into a proposed coefficient and
no pseudoinverse, PSD projection or positive stiffness minimum is selected.

### Protocol and reproducibility

New parent-workspace evidence:
`outputs/rb-13/20260910-104453-stage3/`. `baseline.json` records repository/process
state, revisions/remotes and hashes of the shared binaries/cache, audited source,
recipes and stage 1/2 probes/results. `run.json` and `final-run.json` record exact
commands, cwd, exits and output. `probe/` and `final-probe/` retain compiler command/log, library, source/protocol hashes
and the full result. `final-identity.json` verifies all baseline-hashed files
remain unchanged. The parent workspace README is updated locally outside Git.

The [protocol](../tools/rb13/stage3-protocol.md) was written before execution.
The bridge was formatted and compiled as a C++17/O2 shared library with
`/usr/bin/c++ -shared -fPIC`, using the effective IPC barrier primitive directly.
Python ctypes calls its energy/force/curvature functions. No shared PolyFEM/IPC
library or binary is rebuilt. Stage 1's unchanged tiny Cholesky solver is reused;
the earlier complete probes are not rerun as new evidence.

Direct identities use abs(actual-reference)/(1+abs(reference)) <=1e-10.
Scalar target checks use absolute gap error <=2e-11*dhat, normalizing converted
gaps by dhat before comparison.
All 78 scalar roots have normalized residual <=2e-10 and brackets of width
<=2e-11*dhat. Scalar residual normalization is
abs(g(d)-k*f(d))/(abs(g(dhat))+abs(k*f(d))); the unit-conversion check additionally
uses K*dhat+abs(k*f). Coupled Newton uses the predeclared 80-iteration limit,
60 backtracking halvings, positive-gap domain, Armijo 1e-4 and normalized residual
<=1e-10. Its target errors also satisfy the absolute 2e-9 gap screen. These are
standalone test controls, not changes to production stopping/CCD behavior.

The initial and final compiled runs passed, with no failed/partial runs. Final
review tightened the target-gap check implementation to use the protocol's
absolute error, rather than the generic relative-plus-absolute identity helper;
all original roots already satisfy it. No model input or declared tolerance
changed. An editorial repeated sentence in the protocol was removed; the
original pre-run copy remains in the evidence directory.
The checked-in [result](../tools/rb13/results-20260910-stage3.json) preserves the
manufactured inputs, all scalar roots, both coupled histories, errors and rejected
cases. Boolean checks report null for an unmeasured error magnitude. No fitted
estimator or held-out application validation is claimed.

### Results: 654 checks passed

| Scope | Measurement | Outcome |
| --- | --- | --- |
| Two supported springs | Six A,B pairs; K_eff=AB/(A+B), target roots, displacements and support/contact balance | Verified within fixture |
| Rigid/deformable limit | Four stiffness ratios; relative error 1/(1+B/A), reaching about 1e-6; explicit prescribed endpoint control | Verified within fixture |
| Rayleigh versus compliance | A=1,B=100: raw r=50.5, restricted gap stiffness=25.25, relaxed K_eff=.9900990099 | Difference characterized, not a production defect/fix |
| Inertia and approach speed | 12 dt/speed cases; eight positive-demand target roots, four zero-demand classifications | Verified reference; C++ integrator class not executed |
| Equivalent units | 27 length/force/time conversions; root, stiffness, force, displacement and objective checks | Pass; production controller not tested |
| Nonlinear prediction | 12 alpha/anchor cases; exact-demand controls recover .5; local predictions have measured errors | Approximation limits reproduced |
| Nonlinear band | Local upper k=.7491934949 predicts .55, actual stable nonlinear root=.3997523752 outside [.45,.55] | Counterexample reproduced |
| Two-contact coupling | Full W yields [.5,.55]; diagonal-only estimate yields [.3859795050,.4724886438] | Counterexample reproduced despite convergence of both solves |
| Joint target compatibility | SPD W=[[2,1],[1,2]], delta=[.1,.5] needs F=[-.1,.3] | Repulsive-target incompatibility recorded; no clamp |
| Unsupported H/J | Six singular/indefinite/zero-J/nonsymmetric/nonfinite cases rejected; explicit support control passes | Reference domain handled explicitly |
| Small positive stiffness | H=diag(1e-10,1) gives K_eff=1.0000000000000002e-10, ratio check passes | Preserved, no floor |

All **654/654 assertions** pass; these are multiple checks of manufactured
systems, not independent FEM scene counts. There are **78 scalar root solves**
and **two full three-displacement/two-contact reference solves**. The latter
converge in five and four Newton updates with normalized residuals 8.76e-17 and
3.45e-12 respectively. The diagonal-only result therefore demonstrates model
error even when the reference nonlinear solver has converged.

Across scalar roots the maximum normalized residual is 2.46e-16 and maximum
normalized bracket width is 1.12e-16. Maximum normalized unit-conversion errors
are 1.12e-16 in gap/dhat, 1.41e-16 in force and 3.08e-16 in physical incremental
energy. Both coupled target gaps are reproduced to the displayed precision.
Direct spring support balance has absolute error <=4.45e-16; the implicit-Euler
full free-coordinate residual controls have absolute error <=3.56e-15.
Python syntax, C++ formatting, local links/anchors, result consistency and
staged diff whitespace checks pass. The actual scalar roots/target errors are
recorded, not inferred from successful process exit alone.

### Interpretation and limits

The source-traced raw Rayleigh value represents a normalized prescribed
displacement direction. Even converting it to gap stiffness leaves different
relaxation constraints from compliance. This is a mathematical contrast, not a
claim that the existing coefficient code promised equivalence or that a local
compliance estimate is ready to replace it.

At dt=.2 the same K_eff=21.75 accompanies speeds 1,2,4 with demands
1.575,5.325,12.825. At speed 4, reusing the speed-2 coefficient produces gap
.3299897579 rather than .5. History/prediction matters even when H is unchanged.
Negative free-gap predictions are extrapolations only; no invalid geometry is
passed to the barrier. Zero target demand does not remove active protection.

For the stable quartic with alpha=100, linearizing at .1 predicts target .5
but reaches .3713309397 (gap error -.1286690603). Its target-force mismatch is
6.4. Moving the anchor to .45 reduces the gap error to -.0032954536; using the
target tangent/demand is exact here. This is an observed local-model error trend,
not a production refresh rule or a general guarantee of improvement.
The band counterexample establishes why RB-14 needs approximation/enclosure
limits beyond a single tangent value. Stage 2's theorem is not contradicted:
the nonlinear spring is outside its fixed-quadratic model.

Rigid modes deserve a distinction: relative J can annihilate a null mode and
admit a quotient-based compliance, while a gap sensitive to that mode has zero
elastic resistance along it. The strict-SPD helper rejects both unsupported
full matrices. Its separately prescribed support fixture is explicit and not an
automatic pseudoinverse/gauge selection. Positive selected curvature alone does
not make an indefinite full energy stable. The tiny diagonal SPD case validates
its own arithmetic, not general ill-conditioned solve robustness.

Units are physical force/length for K, force/length^3 for k, and force for k*f.
Weights are fixed and unit unless converted in the stated dimensional control.
Reported physical incremental energy includes the implicit-Euler inertial
potential; it is not stored energy or a trajectory work budget. Each root holds
its coefficient fixed; no retuning event or friction-lag state is present.

No production solver, full unit suite, public/private scene, Teseo, HDA test or
asset rebuild was run: no production/workflow change requires them in this
standalone stage. There is no measured continuum accuracy, mesh det(F), contact
feature transition, remeshed/interpolated coefficient support, friction
dissipation, trajectory/work balance, mesh/time convergence, real estimator
enclosure, resource containment or other-platform verification. The converted
unit fixtures do not erase the earlier controller unit-dependence evidence.

### Publication and final RB-13 handoff

Publish the new probe/bridge/protocol/result, extended contract, validation
record and roadmap to `sdast9/polyfem:main`; the completion message identifies
the exact commit. Shared production binaries and all earlier probe/result hashes
remain unchanged. No dependency pin or HDA publication is needed. The compiled
library and full local provenance remain in the isolated evidence directory.

All three planned RB-13 analytical stages are characterized within their stated
scopes. The item's status is **characterized—decision pending**, because this
investigation has not selected a production estimator, coefficient assignment,
controller or default. The stage 1/2 results and historical handoffs above remain
intact; this section supersedes their outstanding stage-3 instructions.

**Recommended next: RB-14**, practical compliance and force-demand estimates
against these references. RB-15 is independently eligible for feature-consistent
assignment. RB-16 still needs estimator/assignment decisions before integrated
controller work, and RB-17 must evaluate the selected candidate before default
promotion. No additional user model decision is necessary to finish this
characterization; downstream model changes retain their own decision boundaries.
