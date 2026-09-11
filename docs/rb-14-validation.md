# RB-14 — Practical compliance and force-demand estimators

Date: 2026-09-10
Status: **in progress — stages 1–2 complete; further estimator comparison pending**
Current handoff: stage 2 assembled-FEM comparison complete; practical estimator
and protection choices remain pending. Earlier stage-1 handoff is retained below
as history and superseded by the stage-2 continuation.

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

## Stage 2 continuation — 2026-09-10

### Scope and incoming state

The user selected “start stage 2” and then “continue.” This stage runs assembled
FEM comparisons and publishes their limits; it does not choose a production law.
Started clean on main at `94597b12a87f954816fadce9233dc977d6d5aee7`. Effective IPC
`af317a65` and PolySolve `4d372fa8` overrides/pins remain unchanged and clean.
No incoming build/solver was found. The item/handoff, current PF invariants,
floor retirement, RB-03 map contract, current embedding/provider/form APIs and
CMake effective overrides were inspected before the comparison.

Evidence is in `outputs/rb-14/20260910-153034-stage2/`. `baseline.json` records
revisions, processes and shared library/binary/cache, recipes, provider sources
and previous RB-13/14 artifact hashes. The isolated executable links the existing
Makefiles build's libraries using its exact unit-test flags and link recipe.
No production source or library was rebuilt or changed; only the standalone
C++ probe was compiled. Library hashes recorded after the run are explicitly
named `linked-archive-hashes-postrun.json`; the main library and binary identities
were also captured at baseline. Actual source, executable and input identities
are preserved with the build commands and per-case inputs/outputs/exits.

See [contract stage 2](rb-14-contract.md#stage-2--assembled-fem-comparison-2026-09-10)
for physical versus weighted units, reduction, prescribed motion, load sign,
coefficient lifecycle and scope. The [protocol](../tools/rb14/stage2-protocol.md)
and [case list](../tools/rb14/stage2-cases.json) precede estimator comparisons.
The in-memory mesh setup uses the same non-strict file-schema initialization as
existing assembler tests, then supplies explicit vertices/cells. It does not
bypass mechanical, derivative, determinant or contact checks.

### Setup failures and final execution

All setup artifacts remain, including failures:

- `setup/`: compile failure from a malformed JSON initializer.
- `setup2/` and `setup3/`: missing required geometry entry; the second run exposed
  the validator message after enabling logging.
- `setup4/`: an empty geometry list was rejected by the minimum-one-entry rule.
- `setup5/`: successful in-memory assembly after using an empty-path placeholder
  and non-strict file validation, as in the existing assembler fixture.
- `comparison-build/`: successful initial baseline comparison.
- `sweep/`: compile failure from attempting the private InertiaForm predictor
  accessor. The probe now uses the public integrator predictor and independently
  checks the actual inertia gradient against it; no visibility/API change.
- `sweep2/`: final formatted C++ source, all 18 declared cases complete. No case
  was dropped, no tolerance relaxed and no held-out input changed.

The runner was subsequently extended to save archive hashes and compile/link
exit records for future reproductions. That logging-only change does not alter
the tested C++ source, binary, input cases or numerical results. The summary
script strips redundant full matrices and endpoint vectors from the published
comparison; full local records retain them.

### Results and checks

**2,772 structural/numerical checks passed across 18 FEM fixtures** (15 calibration,
three held out). There are 126 candidate records: 54 converged positive-k solves
and 72 zero-demand candidates for which no unprotected solve was attempted.
The separate stale-dt reuse solve also converged: **55/55 attempted nonlinear
solves converged**. Zero-demand classifications are not successful contact solves.
The [checked-in comparison](../tools/rb14/results-20260910-stage2.json) retains
candidate errors, all nonlinear histories, cost, coverage and classifications.

| Check / finding | Measurement | Outcome |
| --- | --- | --- |
| Actual selector extraction | Assembled FEM H through BarrierContactForm, appended obstacle IDs, centered EV normalization | Pass within supported exact-selector fixture |
| Physical scaling | Actual dt^2, inertia gradient/predictor identities | Pass; no mass double counting |
| Residual/tangent consistency | Maximum gradient/Hessian FD errors 1.03e-9 / 5.60e-11 | Pass for enabled constant-load cases; pressure derivative unmeasured |
| Full reference | SPD eigenspectrum, maximum condition number 407.388; dense/sparse LLT agreement and residual screens | Pass on all declared snapshots |
| Neighborhood locality | n=2/4/8, full K 11.2215/7.8694/5.9819; radius-1 dimension remains 10 | Shrinking physical support and missed demand reproduced |
| Nonlinear target error | Held-out speed: full .04801919, radius 1 .01219297, radius 2 .03251305 versus .05 | Approximation errors retained despite convergence |
| Empirical coverage | FEM-calibrated held-out K 2/3, p 0/3, joint 0/3 | Failed generalization reproduced |
| Stage-1 transfer | Prior empirical ranges cover 9/15 FEM calibration cases jointly and 0/3 held out | Transfer failure retained |
| Conditional interval | 13 inactive/mixed rectangles, five empty intervals; zero active nonempty intervals | No coefficient chosen from them |
| Geometry/forces | Minimum endpoint det(F)=.470555; actual proxy CCD/elastic step limits; mapped contact action/reaction | Pass within represented pair; no whole-surface collision claim |
| Fresh/stale dt | Old k gives .06057172; fresh reference p=.05493047 gives zero target demand | Stale consequence measured; protection policy remains unresolved |
| Relevant existing regressions | Mapping tests and Neo-Hookean nonlinear branches | 11 cases / 7,994 assertions passed |

The existing suite command is preserved under `existing-tests/`, including RNG
seed, exit zero and full log. No full scene smoke, HDA suite, private input or
Teseo was run. No AL/reduced-production stopping, friction, retuning, timestep
schedule, CCD, trial cap, constraint floor or dependency setting changed.

Positive load values in this probe's negative-y RHS syntax produce upward body
force in the actual assembled objective; load cases are unloading comparisons.
This was inspected and recorded, not relabeled as a compressive test or changed
post hoc. The full nonlinear reference uses the same actual objective, and its
finite-difference checks pass. No production load-sign defect is alleged.

The sparse n=8 full matrix/factor payload is 21,580/23,128 bytes, with median
factorization 34.375 us and two solves reusing that factor 5.583 us. Dense full
extraction/two-solve cost is 97.916 us; radius-1 cost .625 us. These local timings
are descriptive and noisy. They exclude global cache lifetime, allocator/work
buffers, process RSS, throughput and multiple-contact scheduling. Reported
payloads are not total peak memory or resource guarantees.

### Publication, status and next work

The C++ probe, runner, analysis script, protocol, case list, compact results and
updated contract/record/roadmap are published to `sdast9/polyfem:main`. The final
message and local `publication.json` identify the commit. Formatting, Python
syntax, links/anchors, JSON consistency, final residuals/BCs and identity checks
are retained in `artifact-checks.json`; shared-file checks are in
`final-identity.json`. Stage-1 executable source/protocol/results remain unchanged;
its tools README is deliberately extended. The parent workspace README remains
a local update outside Git. No HDA/dependency publication is needed.

**Stage 2 is complete within the declared scope; RB-14 remains in progress.**
The comparison supplies actual assembly/extraction and nonlinear evidence, but
fixed graph-radius candidates and empirical envelopes are not ready for a
production guarantee. Further candidate work is needed before asking for a
production selection: compare physical-size neighborhoods and remote residual
influence, alongside sparse-factor reuse and a declared zero-demand protection
contract. No alternative is silently selected by this handoff.

Remaining coverage includes assembled material contrast/anisotropy, loaded
pressure tangents, full contact patches and 3D, general interpolation, nonlinear
force-range enclosures and multi-contact resource behavior. Stage-1 network
controls remain useful but do not fill those FEM gaps. RB-15 is independently
eligible; RB-16/17 integration still requires explicit estimator/assignment
choices and comparison.
