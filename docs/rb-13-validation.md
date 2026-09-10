# RB-13 — Mechanical coefficient estimate and conditional bounds

Date: 2026-09-09
Status: **in progress — stage 1 characterized; stages 2–3 pending**
Selected stage: reference derivation and standalone analytical fixture only.

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
