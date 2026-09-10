# RB-14 — Practical compliance and force-demand estimators

Date: 2026-09-10
Status: **in progress — stage 1 standalone comparison complete; stage 2 pending**
Selected stage: supported quadratic networks, estimator errors, held-out coverage
and fallback controls. Production estimator selection remains pending.

## Authorization and baseline

The user requested “please start RB-14.” The first bounded stage establishes
comparative evidence before assembled-FEM extraction or production changes.
Read the RB plan/item, PF invariants and floor-retirement record, RB-03 map
contract and RB-13 reference/handoff. The invariant is a consistent comparison
of candidate K and unconstrained-gap predictor p against full SPD solves, with
physical-distance coefficient and realized-gap checks kept separate.

Started clean on PolyFEM main at `5b64d316fc68e94ba1ae8b3492af31f8ed8f19f5`.
The clean effective IPC override is `../ipc-toolkit-fork` at `af317a65d69d0ac7c5efa4bf103bf75e280c323b`
and clean PolySolve override `../polysolve-merged` at `4d372fa8a73f42bc224e31d464f1a308e1159ba8`.
CMake overrides and recipe pins agree. No incoming solver/build process was found.
Source inspection confirmed current selector extraction and provider scaling;
H contains elastic/inertia contributions while the gradient includes body/pressure
forms too. Actual nonlinear load-tangent consistency remains unmeasured.

Evidence: `outputs/rb-14/20260910-151657-stage1/` in the parent workspace.
`baseline.json` retains repository/process state and shared binary/cache, recipe,
source and RB-13 artifact hashes. The pre-run protocol is retained unchanged.
`run.json`, `final-run.json`, `reviewed-run.json`, `publication-run.json` retain commands/cwd/output/exits;
`probe/`, `final-probe/`, `reviewed-probe/`, `publication-probe/` hold independent compiled IPC bridges,
provenance and full results. macOS arm64, system Python and `/usr/bin/c++` C++17/O2;
these tiny serial solves do not exercise configured production threading/solvers.

No production source, dependency, HDA, scene input, coefficient law, stopping
criterion, friction state, CCD, trial cap or retired floor changed. No private
scene/Teseo or full solver run was needed for this standalone probe.

## Comparison and outcomes

See the [contract](rb-14-contract.md) for definitions, units and exact scope;
[protocol](../tools/rb14/stage1-protocol.md) for predeclared parameters/tolerances;
[comparison](../tools/rb14/results-20260910-stage1.json) for all measurements.

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Fixed-boundary neighborhood stiffness is an upper estimate for these SPD references | Nested full/radius-1/radius-0 inequalities on all 28 fixtures; restricted-energy derivation | Characterized within quadratic scope |
| Good stiffness alone does not predict demand | Baseline radius-1 K error .2863%, gap .04874483 versus .05; remote load 10 leaves its estimate unchanged but gap becomes .03936870 | Limitation reproduced |
| Empirical ranges do not enclose held-out behavior | Radius-1 joint coverage 18/18 calibration by construction, 6/10 held out | Failed generalization retained |
| Extreme remote response is missed | Combined holdout radius 1 gives zero demand despite reference p=-37.5869 | Unprotected candidate outcome recorded, no invalid barrier evaluation |
| Diagonal K and local p have no general error sign | Independent two-DOF positive/negative off-diagonal and remote-load controls | Both error signs reproduced |
| Shared raw coefficient cannot meet heterogeneous bands | Same dt/dhat independent blocks require disjoint k intervals | Control incompatibility reproduced; global trim is a different question |
| Prescribed motion belongs in both residual and gap | Fixed/moving obstacle force, free balance and reaction; finite-rigid-limit controls | Verified algebraically |
| Stale dt reuse changes realized gap | Old k gives .07839541; fresh full recomputation gives .05 | Proposal measured, no production fallback installed |

## Validation results

**806/806 checks passed**, 18 calibration + 10 held-out fixtures, 224 positive-k
scalar solves. The compiled IPC force is used for each positive-k predicted and
realized root. The full-system endpoint is reconstructed as v=-H^-1 r+H^-1 J^T F;
its free equilibrium residual and gap are checked independently of bisection.
This is exact condensation of a frozen quadratic, not a nonlinear continuum solve.

Predeclared solve/identity/root tolerances are 1e-8 with the protocol's force/gap
normalizations. Maximum positive-k normalized root residual is 6.10e-16.
No tolerance was relaxed. Initial run passed 789 checks; review added explicit
prescribed reactions, finite-rigid-limit and stale-dt consequence checks (805),
then the independent predictor-error sign control (806). The final publication run also passed 806 checks after tightening target-gap
assertions from the generic identity helper to the protocol's absolute normalized
gap check; all earlier target results already met it. All four runs remain
in evidence, with no failed/partial runs. Additional controls did not change the
28 comparison fixtures, empirical ranges or any held-out result.

| Candidate | Max calibration relative K error | Max held-out relative K error | Max calibration p error / dhat | Max held-out p error / dhat |
| --- | ---: | ---: | ---: | ---: |
| Radius 0 | 1.3573 | 45.7774 | 1.0398 | 376.6682 |
| Radius 1 | .2216 | 26.9682 | .3436 | 376.6670 |
| Diagonal | 1.3573 | 1651.6921 | 1.0398 | 376.6690 |
| Gap direction | 7.7173 | 109036.9222 | 1.0398 | 376.6690 |
| Raw Rayleigh comparison | 16.4347 | 218074.8444 | 1.0398 | 376.6690 |

Errors above are absolute magnitudes; 1.0 means 100% relative K error. Large
holdout values are manufactured approximation failures, not arithmetic failures.
Full-reference self-error columns in JSON are zero by definition; reference
credibility instead uses checked linear residuals and full equilibrium balance.

Held-out empirical coverage is K 7/10, p 8/10, jointly 6/10. There are only two
active nonempty empirical bands across both sets, one per set; both endpoint
pairs fall in the band. All remaining intervals are empty or inactive/mixed,
reported separately. No coverage claim is drawn from training cases alone.

Cost records use 15 repeats of baseline estimator evaluation and a separate
tracemalloc call. Final median full/radius-1 costs are 98.5/43.5 us; peak traced
Python allocations 4672/2240 bytes. The earlier full median was 65.1 us and second
run 98.8 us, illustrating timing noise. This is neither production performance
nor a memory bound: no sparse factor reuse, contact-count scaling, native-memory
accounting or cache lifecycle is measured.

## Verification and publication

The isolated bridge was rebuilt for each run. Production C++ and shared binaries
are unchanged, so no production rebuild, HDA suite or scene smoke was run.
Syntax, local documentation links, result consistency, whitespace and baseline
hash-preservation checks are recorded in `artifact-checks.json` and
`final-identity.json`. The parent workspace README is updated locally outside Git.
The probe, protocol, compact comparison, contract, this record and roadmap are
published to `sdast9/polyfem:main`; the completion message and local publication
record identify the exact commit. No dependency pin or HDA publication is needed.

## Stage 2 handoff and remaining acceptance

RB-14 is **not complete**. Stage 1 establishes a reproducible candidate comparison
and shows why a neighborhood estimate or empirical envelope cannot yet be used
as a production guarantee. It does not choose a preferred production estimator.

Next, capture small assembled FEM snapshots on RB-03's supported selector maps,
with correct free reduction, prescribed offsets, current noncontact residual and
physical Hessian scaling. Compare the same neighborhood candidates against full
solves. Declare missing load-tangent contributions, indefinite/stale outcomes
and neighborhood BCs. Add genuine mesh refinement, a held-out assembled fixture,
nonlinear predicted-versus-realized gaps and realistic extraction/solve/memory
cost. Measure fallback and uncertainty again before any production decision.
The first stage's geometric-size sweep is not mesh refinement; the algebraic
permutation is not execution of the production map/extractor.

No new user model choice is needed to begin that independent comparison.
Production estimator, invalid-state continuation, uncertainty enclosure and
coefficient lifecycle choices remain pending. RB-15 remains independently
eligible; RB-16/17 integration is not authorized by this stage.
