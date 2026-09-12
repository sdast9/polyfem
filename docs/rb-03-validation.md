# RB-03 — Collision/FEM coordinate mapping

Date: 2026-09-08 (characterization, indexing repair); 2026-09-11 (interpolated stiffness)
Status: **closed 2026-09-11 — validated within stated scope**: exact selector indexing (2026-09-08) and the selected interpolated-stencil stiffness (2026-09-11, see the [last stage](#interpolated-stencil-stiffness--2026-09-11)); no decision pending — the interpolation curvature choice this item was held open for was made and implemented, not left unselected

The characterization below is historical. The indexing repair stage and the
interpolated-stiffness stage are recorded at the end of this document; their
targeted validation is complete.

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


## Exact selector indexing repair — 2026-09-08

The user selected the recommended first implementation step: repair exact
selector/permutation and obstacle indexing, then validate and publish. No new
interpolation curvature definition, coefficient law or controller policy was
selected. Starting PolyFEM commit was `eae30966b`, with clean tracked sources;
IPC was `9da3094` and PolySolve `4d372fa8`. No active solver/build was found.

The original 97-check mapping probe passed its historical defect assertions.
New `[contact_stiffness_mapping]` tests then ran against unchanged production
code: 2 cases / 80 assertions, 48 failures (process exit 42); identity/rectangular
identity controls passed. Tests compare against an independently constructed
small surface-coordinate `P H P^T` reference, including cross-node and
cross-component terms, 2D/3D edge-point contact, inclusion plus permutation,
FEM-only and full Hessians, stationary/prescribed obstacle positions, mapped
energy/gradient/Hessian, repeated assignment and refresh. Tolerances were
specified before the baseline: absolute-plus-relative 1e-10 at moderate stiffness
and finite gap. No tolerance was relaxed.

The implementation exposes IPC's existing sparse `S*T` displacement map through
a const accessor, and records the exact unit-selector system-node ID of each
collision vertex when a semi-implicit form is constructed. Stencils whose rows
select distinct system nodes extract the Hessian blocks using those IDs.
Out-of-Hessian obstacle nodes contribute zero blocks, as in the identity contract.
Other contact modes do not build or use this lookup. Storage is linear in surface
vertex count; there is no dense production map/inverse or per-contact map scan.
CollisionMesh topology and its displacement map must remain unchanged during a
form's lifetime, consistent with the existing form ownership contract.

If any stencil row is interpolated, scaled, empty, or selects a duplicate system
node, the **entire stencil** retains the legacy proxy-ID sampling. This is an
explicit unresolved limitation, not a claim that such stiffness is correct.
Mixed interpolated FEM/obstacle stencils are therefore not repaired by this stage.
The new regression suite locks down that scope boundary. The standalone probe
now requires the corrected permutation coefficient 26.6666667 instead of the
historical 71.6666667, and adds an actual external selector/obstacle builder case
with a FEM-only Hessian and expected coefficient 66.6666667.

IPC companion commit `af317a65d69d0ac7c5efa4bf103bf75e280c323b` adds only the
const map accessor. PolyFEM pins that revision. For verification the existing
build now uses an explicit `CPM_ipc-toolkit_SOURCE` pointing at the companion
checkout, whose committed revision must match the pin. The original CPM cache
checkout is untouched; all other build choices and the PolySolve override remain.

Evidence: `outputs/rb-03/20260908-145544-indexing/` relative to the parent workspace.
Baseline states/hashes and original CMake cache, build logs, baseline failures,
probe copies/results, and subsequent verification are retained there. One early
attempt to change the probe used an incorrect relative path and did not edit it;
its following run simply repeated the historical 97-check probe. The decisive
baseline failure is the new Catch regression above.

### Repair validation and publication

| Check | Result | Exit |
| --- | --- | --- |
| Configure and rebuild `PolyFEM_bin unit_tests -j 6` | Complete with the pinned IPC companion source | 0 |
| `[contact_stiffness_mapping]`, seed 1 | 3 cases / 88 assertions pass; includes 8 additional assertions preserving unresolved map behavior | 0 |
| Affected contact/cache/AL/BC/filter/derivative selection, seed 1 | 26 cases / 1,892 assertions pass | 0 |
| Recompiled standalone mapping probe | 110 checks pass; original tolerances retained | 0 |
| quasistatic-adaptive | Four steps to t=1, zero error lines | 0 |
| quasistatic-semi-alhess | Four steps to t=1, zero error lines | 0 |
| quasistatic-semi-friction | Four steps to t=1, zero error lines | 0 |
| quasistatic-semi | Four steps to t=1, zero error lines | 0 |
| transient-semi | Four steps to t=1, zero error lines | 0 |
| HDA `test_polyfem_hda.py`, Houdini 22.0.429 | Real-binary scene/export/import round trip passes | 0 |
| C++ formatting, local document links, diff whitespace | Checked | pass |

Each smoke's process exit was independently extracted from the copied runner's
log. Every PVD has times `[0,.25,.5,.75,1]`. Only the runner's OUT variable changed.
Expected negative-test error messages in the affected suite did not fail tests.
No goldens or prior measurement files were regenerated. The original
`results-20260908.json` remains historical; the new
[repair result](../tools/rb03/results-20260908-indexing.json) is separate.

The permutation fixture now measures 26.6666667, matching its independent
reference; identity remains 71.6666667. The actual external selector/obstacle
fixture measures 66.6666667 with stationary and moving obstacles and a FEM-only
Hessian. Newly appearing and reappearing permuted contacts retain the mapped
coefficient after an empty snapshot. The original interpolated-contact and
interpolated-obstacle values remain 71.6666667 and 83.3333333 respectively.
The 2D standalone probe's maximum gradient relative error is 1.051e-10 and
maximum energy-only Hessian relative error is 3.761e-7. Its original 1e-6 and
1e-5 acceptance thresholds are unchanged. 3D coverage is the unit suite's tiny
edge-point reference comparison, not general 3D contact certification.

No new coefficient, friction, stopping, CCD, trial-cap, resource/recovery or
HDA asset policy was introduced. The constraint floor remains retired. No
Teseo, Ballburst, private scene, complete solver/HDA suite, other platform,
continuum accuracy, total work balance, mesh/time refinement or arbitrary
high-order validation was run. The existing HDA assets were hashed, not rebuilt.

Publish the accessor to `sdast9/ipc-toolkit:semi-implicit-stiffness` at
`af317a65d69d0ac7c5efa4bf103bf75e280c323b`, and this repair, dependency pin, tests,
probe/result and records to `sdast9/polyfem:main`. The completion message reports
the resulting PolyFEM commit. The local parent README is updated separately.
`tested-manifest.json` records binary/library/input/HDA and modified-source hashes;
the source patch and new test source are retained with the evidence.

RB-04 can proceed using the established derivative maps and exact-selector
stiffness support. It must keep interpolated/mixed-stencil stiffness and RB-02's
coefficient/lifecycle decisions explicitly unresolved. This closes the bounded
indexing repair, not the entire RB-03 model decision.


## Interpolated-stencil stiffness — 2026-09-11

After the 2026-09-11 decision to retain the production adaptive-barrier law,
the user selected, from [the contract's alternatives](rb-03-contract.md#selected-interpolated-stiffness--2026-09-11),
**local condensation of the parent block with the gap-normalized force
direction as fallback** for stencils whose map rows are interpolated, scaled,
empty or duplicated. This stage implements that definition in
`BarrierContactForm::interpolated_stiffness`, replaces the legacy proxy-ID
sampling those stencils used, and validates it. No coefficient law,
controller, friction, CCD, trial-cap, dependency or HDA change; exact selector
stencils keep the 2026-09-08 extraction unchanged.

### Baseline and reproduction

PolyFEM started clean on `main` at `a0883f608` (the RB-02 closure over the
RB-20/21 code `beb6ef641`); companion IPC `e3c8d3fe` on
`semi-implicit-stiffness` and PolySolve `5afe3b5d` on `iteration-callback`,
both clean and matching the pins, used through the existing
`CPM_*_SOURCE` overrides. No build or simulation was running. Evidence relative
to the parent workspace: `outputs/rb-03/20260912T025302Z-interpolation/`
(`baseline/baseline.txt` with repo states, pins, effective sources and
pre-change binary hashes; `baseline/CMakeCache.txt`; `tested-manifest.txt`
with post-build binary and changed-source hashes; `source.patch`).

The defect was reproduced three ways on the unchanged library before any
production edit: the existing `[contact_stiffness_mapping]` tests pass with
their legacy assertions (3 cases / 88 assertions, `baseline/mapping-tests/`);
the probe passes 110 checks reading 71.6667 for the interpolated contact and
83.3333 for the mixed FEM/obstacle proxy (`baseline/probe/`); and the new
tests of this stage, built against the unchanged library, fail decisively
(`baseline/new-tests-pre-fix/`: 4 cases, 2 failed, 31 of 169 assertions):
every synthetic variant reads the legacy 71.6667, and on a real Q1 hex column
the centroid contacts read **101.0 = max|H|/d̂²** — the RB-18 F7 global
fallback, because a centroid's proxy ID lies outside the FE node range and
its block was read as zero (`curvature_global_fallback_count == 2`).

Where interpolated rows arise in production (source audit): Q1 hexahedral
boundary faces on the default path (face centroid with four .25 weights,
`io/OutData.cpp`), `max_order` sampled proxies and spline bases,
`max_edge_length` collision proxies, and external `linear_map` weights.
Simplicial P1/P2 meshes and the HDA pipeline produce only selectors.

### Change

`BarrierContactForm` keeps, per non-selector map row, its (node, weight)
parents (built once from the sparse map). `estimate_stiffness` routes any
stencil that is not an exact distinct selection to `interpolated_stiffness`,
which assembles the dense parent block `H_PP` and the kept rows `B` from the
frozen Hessian, excludes fixed parents (outside the Hessian or with a zero
diagonal block) and drops rows without a movable parent, then returns
`wᵀ (B H_PP⁻¹ Bᵀ)⁻¹ w` through the toolkit's own `semi_implicit_stiffness`
(LLT of `H_PP`, column-pivoted QR rank check on `B` at 1e-10, LLT of the
compliance, finiteness check), or the fallback
`|w_K|⁴ (wᵀ B H_PP Bᵀ w)/(wᵀ B Bᵀ w)²` when any of those fail. The result
enters the unchanged d̂² normalization, weight check and RB-18 F2/F4/F7
chain. Two diagnostic counters (`interpolated_condensed_count`,
`interpolated_direction_count`) and a per-refresh debug line were added.
IPC and PolySolve are unchanged.

### Validation

| Check | Expected criterion | Measured result | Exit |
| --- | --- | --- | --- |
| Rebuild `PolyFEM_bin unit_tests -j 6` | complete; no new warnings | complete (`build.log`; only pre-existing deprecation/libigl warnings) | 0 |
| `[contact_stiffness_mapping]`, seeds 1–3 | all assertions pass | **4 cases / 169 assertions** on each seed (`mapping-tests-post-fix/`): exact-selector cases unchanged (88 assertions); six synthetic variants match hand-derived values (94.2424, 86.6667, 630, 70, 123.75, 213.75 via the F7 \|·\| chain) and an independent dense oracle; a real Q1 hex column against an obstacle plane through the production builder (18 centroid rows) matches the oracle on every collision with all interpolated stencils condensed and zero fallbacks | 0 |
| `[semi_implicit_coefficients] [kappa_continuity] [contact_cache]`, seed 1 | unchanged | 12 cases / 834 assertions (`semi-implicit-suites/`) | 0 |
| Affected suite, RB-18/19/20 tag list + `[kappa_continuity]`, seed 1 | no assertion failure | **56 cases / 3,496 assertions** (`affected-suite-rb20-list/`; RB-20 had 55 / 3,415 — the difference is exactly this stage's new mapping case and 81 assertions) | 0 |
| Plan's affected selection + semi-implicit tags, seed 1 | no assertion failure | 39 cases / 2,206 assertions (`affected-suite/`) | 0 |
| Standalone probe (`tools/rb03/run_probe.py`) | flipped and new checks pass | **120 checks** (`probe-post-fix/`, published as [results-20260911-interpolation.json](../tools/rb03/results-20260911-interpolation.json)): interpolated contact 218.3333 = minimum-energy lift, obstacle proxy 133.3333, dependent rows 45, indefinite block 123.75, counters as expected; all 2026-09-08 derivative/FD/virtual-work/NLProblem checks retained | 0 |
| Five public smokes (`run-smoke-final.sh`, OUT only) | exit 0, four steps, endpoints at run-to-run noise of the same-session baseline | all exit 0, zero error lines, 5 saved steps; max endpoint difference vs `baseline/smokes` **2.2e-16 / 1.4e-16 / 1.2e-16 / 4.5e-16 / 2.0e-16** (adaptive / semi / alhess / friction / transient), identical trim histories; the baseline itself differs from RB-20's published endpoints by ≤6.0e-16 (thread summation order, RB-19); zero interpolated rows logged, as every smoke mesh is simplicial | 0 |
| Hex scene `tools/rb03/hex-scene/quasistatic-semi-hex.json` (4×4×4 Q1 hex unit cube, otherwise `quasistatic-semi.json`) | completes; interpolated path exercised | exit 0, zero error lines, four steps; first refresh **42 of 61 contacts have interpolated map rows: 42 condensed, 0 direction fallback**, no invalid-curvature fallbacks; coefficients 8.1e11–2.2e13 (tet smoke 1.5e12–8.2e12), trims 2,2,2,2,2,4,4,4 (tet 2,2,2,4,4,8,8,8), Linf 0.24272 (tet 0.24282), 0.86 s (`hex-scene/`) — completion and diagnostic evidence, not a physical comparison | 0 |
| HDA `test_polyfem_hda.py`, Houdini 22.0.429 hython | round trip and end-to-end pass on the rebuilt binary | PASS / PASS (`hda-e2e.log`) | 0 |
| clang-format (four C++ files), `git diff --check`, local links | clean | clean | — |

Tolerances were fixed before the runs: unit-test comparisons 1e-10 relative
(1e-9 on the hex oracle, whose condensation goes through a dense 60×60
coupled block), probe `close` at 1e-9 relative. No threshold was relaxed and
no golden regenerated. The hex scene has no pre-change counterpart in this
session (the pre-change binary was rebuilt in place); its pre-change behavior
is the unit test's measured global-fallback reading.

Not measured: physical accuracy of the condensed stiffness on hex or proxy
meshes, mesh/time refinement, friction on interpolated stencils, `max_order` /
`max_edge_length` / external-map scenes beyond the probe's builder fixture,
other platforms, private scenes, Teseo or Ballburst. Q2+ hexahedral faces are
skipped by the default boundary extraction itself and remain outside this item.

### Publication

Publish this stage — `BarrierContactForm.{hpp,cpp}`, the mapping tests, the
probe and its dated result, the hex scene under `tools/rb03/hex-scene/`, the
contract, this record and the plan/README rows — to `sdast9/polyfem:main`.
No companion (IPC, PolySolve) or HDA change; the pins are unchanged.
Published as `1ea71d93c` on `sdast9/polyfem:main` (this hash note follows in
a separate documentation commit).
