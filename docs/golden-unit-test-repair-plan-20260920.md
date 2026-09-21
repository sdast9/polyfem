# Implementation plan for the four failing golden-test groups

Investigation date: **2026-09-20**. Status: **plan only; no repair, reference
regeneration, tolerance change, or production-default change applied**.

The four CTest groups contain **six failing inputs**. They need separate
treatment. Two friction mismatches now have a controlled cause; an existing
collision proxy recovers the mixed-order scene exactly; the microstructure
scene needs a supported resource profile; and the GCP scene needs an ordering
investigation before selecting its reference policy.

This document makes [CI-03–CI-06](ci-portability-plan.md#4-scene-failures-requiring-distinct-treatment)
actionable. It incorporates newer experiments than the September 13 audit.
The [machine-readable investigation record](golden-unit-test-investigation-20260920.json)
contains the measured values, binary identity, effective settings, and limits.

## 1. Verified failures and what is known

Both Release jobs at `c30d215ebd62fa7ef23322d0b6230a7d1e657231`,
[Build run 35537426575](https://github.com/sdast9/polyfem/actions/runs/35537426575),
fail the same four groups and six inputs:

- [macOS Release, job 106148868200](https://github.com/sdast9/polyfem/actions/runs/35537426575/job/106148868200): **361/365 pass**.
- [Linux Release, job 106148868109](https://github.com/sdast9/polyfem/actions/runs/35537426575/job/106148868109): **362/366 pass**.

All paths in this table are relative to the pinned `polyfem-data` root.
The numerical discrepancy is the largest of the six harness comparisons,
not a percentage error against physical ground truth.

| Group | Input | Current failure | Diagnosis and confidence |
| --- | --- | --- | --- |
| `contact_2d` | `contact/examples/2D/large-ratios/large-mass-ratio.json` | `err_h1_semi` relative difference 0.00622455 | **Confirmed historical friction-budget mismatch.** Explicit budget 1 recovers all six references; budget 2 reproduces CI. |
| `contact_3d` | `contact/examples/3D/large-ratios/large-mass-ratio.json` | `err_h1_semi` relative difference 0.31665698 | **Confirmed same cause.** Explicit budget 1 recovers all six references; budget 2 reproduces CI. |
| `triangle_data` | `contact/examples/3D/higher-order/ball-bounce/P4-dt=0.01.json` | `err_h1_semi` relative difference 0.44076346 | **Strong friction-budget hypothesis, still requires A/B.** It does not pin the budget and first failed after the approved default change. |
| `standard` | `multi-material/stretch-cubes.json` | 437 of 7,610 boundary faces skipped; a P2 face has five owned nodes | **Confirmed extraction incompatibility; tested configuration remedy.** Both explicit `dof` and `max_order` proxies complete and recover all six references exactly. |
| `triangle_data` | `contact/examples/3D/higher-order/microstructure.json` | HashGrid requests 65,153,532 candidate emissions; limit 50,000,000 | **Confirmed protective refusal.** A memory/runtime characterization must select its fixture-specific execution profile. |
| `contact_2d` | `gcp-contact/cube-on-floor/run.json` | H1-seminorm discrepancy 0.00426405 on macOS, 0.00348178 on Linux | **Confirmed fresh-process variation and reference mismatch.** Hash-dependent collision ordering is a concrete, unproven end-to-end cause. |

`authenticate_json()` reports internal result **3** for `SOLVE_FAILED` and
**4** for `AUTHETICATION_FAILED`. These are not standalone process exit codes.
The executable uses exit 1 for an ordinary failure and exit 3 for a resource
failure. Keep these contracts separate when parsing evidence.

There is a separate current failure: [Linux Debug, job 106148868118](https://github.com/sdast9/polyfem/actions/runs/35537426575/job/106148868118)
passes 330/334 tests, with SIGSEGV in four rollback/publication-failure tests.
Those are outside this plan. The golden repairs alone cannot establish an
all-green matrix. Debug and Windows normally hide the `[run]` scenes; their
green status cannot substitute for Release scene coverage.

## 2. Evidence from this investigation

Local evidence is retained under the parent workspace:

```text
outputs/golden-investigation/20260920T221525Z/
```

The copied executable is frozen at SHA-256
`fa7837fd16d6ab0bc321ddaa9798d02606f0feab2e09f4713906a76bb5b6eaae`.
Its build record says AppleClang 21, RelWithDebInfo/TBB, Triangle **OFF**,
PolyFEM `92e0d8c20` plus the then-uncommitted RBR-01 patch, IPC `c24d803e`,
and PolySolve `bce32a39`. The recorded PolyFEM patch SHA-256 is
`b9a992fad0000f4a7c7d33ba8af04ab6c014171cd98b7326a502ec4645e1f971`.
The shared checkout subsequently recorded RBR-01 as `babc3b355`. **These local
experiments are not tests of the exact CI binary at `c30d215e`.** Reconfirm
the chosen repairs against the implementation checkout before publication.

The diagnostic driver resolves the fixture's `common` chain and JSON Patch,
makes mesh paths absolute, applies the same time horizon and Eigen solver as
`verify_run.cpp`, and requests one thread. It adds isolated JSON/solution
outputs. All original inputs, reference values, and common files remain
unchanged. Each run saves input/binary hashes and an exit record before
parsing results. This is a CLI reproduction of the harness settings, not a
fresh execution of the complete CTest groups.

| Local experiment | Horizon | Maximum relative reference discrepancy | Outcome |
| --- | --- | ---: | --- |
| 2D mass ratio, friction budget 1 | 120 steps, dt 0.025 | 5.25e-12 | all six pass at 1e-5 |
| 2D mass ratio, friction budget 2 | same | 0.00622455 | same six-metric mismatch as macOS CI |
| 3D mass ratio, friction budget 1 | 120 steps, dt 0.025 | 7.74e-10 | all six pass at 1e-5 |
| 3D mass ratio, friction budget 2 | same | 0.31665698 | same six-metric mismatch as macOS CI |
| Stretch cubes, default extraction | 1 step, dt 0.0025 | no solution | named incomplete-surface refusal |
| Stretch cubes, `max_order` proxy | same | 0 | all six pass exactly |
| Stretch cubes, `dof` proxy | same | 0 | all six pass exactly |
| GCP cube, omitted friction budget, process 1 | 20 steps, dt 0.0005 | 0.00345668 | completes; reference fails |
| GCP cube, omitted friction budget, process 2 | same | 0.00417345 | completes; reference fails |
| GCP cube, input friction budget 1 | same | 0.00397196 | effective budget becomes 0; reference fails |

Additional findings:

- The DOF proxy emits 7,025 used vertices, 21,069 edges, and 14,046 triangles.
  Every edge belongs to two triangles with opposite directed orientation;
  there is one connected component, Euler characteristic 2, and no zero-area
  triangle. Its total undeformed area equals the original 7,610-face tetrahedral
  boundary, **10.430232935572167**. This is useful coverage/topology evidence;
  it is not a self-intersection or displacement-map proof.
- The two identical-input GCP processes differ by **8.860650379579598e-4**
  in the largest final displacement component. Each records 20 accepted
  steps and completion with exit 0. Its solve records use configured gradient
  or directional-derivative stopping; this does not independently establish
  a physical accuracy bound.
- GCP's resolved `contact/friction_coefficient` is zero. `State::init`
  therefore sets `solver/contact/friction_iterations` to zero even when the
  input requests 1. Do not try to fix this scene by applying the mass-ratio
  fixture's friction override.
- The active IPC build has Abseil and robin-map enabled. A small program
  linked to the same Abseil libraries produced **five different key iteration
  orders in five fresh processes** for the same inserted integer pairs.
  `SmoothCollisionsBuilder<2>::merge` emits collisions by iterating these maps;
  `SmoothContactPotential` accumulates energy/gradient/Hessian from that list.
  This demonstrates an ordering mechanism, not yet its complete contribution
  to the scene's final error. TBB reduction order may add further variation.
- The P4 ball and microstructure require Triangle for their **irregular
  collision proxies**. They were not rerun using the Triangle-OFF local binary.
  Replacing irregular tessellation to get a local run would change the fixture.

## 3. Scope and execution rules

Read root `AGENTS.md`, the current [CI plan](ci-portability-plan.md),
[PF invariants](correctness-remediation-plan.md), and the linked RB record
for the selected work package. Read current source, branch status, effective
dependency overrides, and incoming changes. Work on **one package per session**
in an isolated checkout/build; other work is using the developer checkout.

Preserve the approved budget-2/realized-force production defaults, CCD, the
trial-displacement cap, retired constraint floor, and retry-off policy.
No Teseo runs, global memory-limit increase, dropped collision faces,
blanket scene exclusions, or bulk reference regeneration belong to this plan.
The use of an existing proxy in a named test fixture does not authorize
changing the automatic production proxy for every mixed-order mesh.

Use fresh evidence directories and record failures as well as successful
runs. A cache entry is reusable only with matching input/binary hashes and a
completed exit-0 run. Preserve source/data pins, compiler/options, thread
count, command, exit status, complete log, resolved input, six metrics,
solver termination records, and peak RSS. Do not cache an interrupted or
unparsed run as a successful reproduction.

## 4. G0 — Provide a versioned fixture-policy mechanism

**Purpose:** clean CI currently downloads upstream `polyfem-data` at
`e0efb6ba291e3acfc8a5e12e66486a7246849b30` using
[`polyfem_data.cmake`](../cmake/recipes/polyfem_data.cmake). Editing the local
`data/` directory will not publish a repair. Prefer a small, explicit
**test-only profile file in this fork** so these repairs do not require
creating or administering another data repository. A reviewed data-fork
change plus a new recipe pin is also valid, but choose one source of truth.

Files to change for the preferred approach:

- `tests/verify_run.cpp`: apply the selected profile in `authenticate_json`;
  pass the data-source identity and relative manifest path from `run_data`.
- New `tests/data_profiles.json`: exact fixture path, rationale, source-data
  revision, input merge patch, and separately represented reference policy.
- `tests/README.md`: explain that a historical profile is visible and local
  to this harness, and document reproduction/regeneration behavior.
- A focused profile/margin test source, added to `tests/CMakeLists.txt` if
  it is separate from the harness.

Implementation requirements:

1. Scope profiles to `POLYFEM_DATA_DIR`; pref/polyspline/optimization roots
   must not match by an incidental filename. Match the complete relative
   path, never a substring or basename.
2. Resolve `common` and its JSON Patch first, then apply the profile's
   explicit input patch, then retain the harness's time-horizon and linear
   solver rules. Do not let a common file overwrite the profile.
3. Print the profile name, rationale, and actual applied settings before the
   solve. Preserve the resolved settings in evidence. Keep profile metadata
   outside the strictly validated production input schema.
4. Leave the stored six golden values unchanged. Keep optional comparison
   policy distinct from solver input; currently authentication reads
   `in_args["tests"]`, so merely changing `args["tests"]` will not change
   the effective margin. Centralize that policy for comparison and generation.
5. Replace unconditional `out["margin"] = 1e-5` with preservation of the
   existing effective, explicitly stored scene margin, defaulting to 1e-5
   only when absent. Keep `time_steps` unchanged. Test the actual metadata
   generation path on temporary files, without executing a large simulation.
6. Validate profile entries against active manifests and the documented
   data revision. Unknown/stale paths and invalid profile structure fail
   clearly. Ordinary unprofiled fixtures must retain their current behavior.

Focused checks: explicit margin survives regeneration; absent margin gets
1e-5; numeric and `"all"` horizons survive; merge order is correct; only the
intended data root/path matches; unknown profile paths fail; resolved default
behavior remains budget 2 for a frictional case outside the profile list.
Do not use the `*` manifest convention against the production dataset during
these tests: it rewrites the input JSON.

## 5. G1 / CI-03 — Pin historical friction settings where demonstrated

Coordinate with [RB-10](rb-10-validation.md). The approved change
`756070f44` moved the general friction budget from 1 to 2. These are ordinary
adaptive-IPC scenes; the semi-implicit `friction_lag` option is not their fix.

For both large-mass-ratio paths, the recommended fixture input patch is:

```json
{"solver": {"contact": {"friction_iterations": 1}}}
```

Steps:

1. Reconfirm the two observed A/B results on the implementation binary.
   Run omitted budget, explicit 1, and explicit 2 with the unchanged
   120-step horizon, materials, mesh, and Eigen::SimplicialLDLT. Verify
   omitted equals explicit 2 within numerical repeatability.
2. Build with `POLYFEM_WITH_TRIANGLE=ON` and run the same three-way comparison
   for `P4-dt=0.01.json`, preserving **20 steps at dt 0.01**, BDF2, its
   `P4.json -> P1.json -> higher-order/common.json -> examples/common.json`
   chain, and the irregular proxy at max edge length 0.01.
3. Add the P4 profile **only if** budget 1 recovers all six stored metrics at
   the existing 1e-5 margin. If it does not, keep that failure open; compare
   `91c7ff8a` with `756070f44` using the same data/dependencies and bisect the
   remaining behavior difference. Do not assume that all frictional scenes
   need the same profile.
4. Retain the existing `[friction_lag]` current-default regressions. Add a
   check that the historical profile does not affect an otherwise identical
   unprofiled input. Keep a current-default smoke with convergence/friction
   diagnostics, so golden preservation does not hide future default regressions.
5. Run `contact_2d`, `contact_3d`, and `triangle_data` and classify any remaining
   failures by scene; G2/G3/G4 may still be open at this stage.

Acceptance: the three demonstrated historical fixtures pass all six original
values at 1e-5; current production defaults remain budget 2/realized-force;
the profile is applied and logged on clean Linux/macOS Release checkouts.
No reference numbers change. Record partial acceptance if P4 is still unresolved.

## 6. G2 / CI-04 — Use and verify the existing DOF proxy for stretch cubes

The default simplex extractor in
[`OutData.cpp`](../src/polyfem/io/OutData.cpp) omits local basis nodes whose
`global().size() != 1`. At the mixed P1/P2/P1 interfaces this leaves five
owned nodes on 437 P2 boundary faces, which the default triangulation cannot
represent. The completeness check correctly refuses those missing faces.

**Recommended bounded repair:** explicitly select the existing DOF proxy for
this fixture through G0, retaining its FE orders and physics:

```json
{"contact": {"collision_mesh": {"enabled": true, "tessellation_type": "dof"}}}
```

`max_order` also reproduced the golden exactly, but `dof` is the first choice
because the emitted vertices use exact selector rows and the measured proxy
uses the existing boundary DOFs. This is an explicit fixture profile, not
a new automatic proxy policy. If the implementation instead proposes
automatic routing of mixed-order tetrahedra, present that separately as a
production-behavior decision with broader validation.

Steps:

1. Reproduce the default refusal and both observed proxy controls on the
   current implementation. Keep contact enabled, the P1/P2/P1 regions,
   `lump_mass_matrix: true`, and the one-step dt 0.0025 test horizon unchanged.
   The higher-order mass-lumping warning remains a separate physical limitation;
   changing the mass matrix is not this golden repair.
2. Add a small conforming mixed-order tetrahedral regression near
   `tests/test_hex_collision_surface.cpp` or a separate
   `tests/test_tet_collision_surface.cpp`. Use real element bases from a mesh
   sharing a P1/P2 interface, including an exterior edge with a constrained
   midpoint. A uniform P2 fixture would miss the defect.
3. Exercise the production `NonlinearElasticVarForm::build_collision_mesh`
   route with the explicit profile. Check zero skipped boundary faces,
   connectedness/topology against the source boundary, oriented edge incidence,
   positive triangle areas, and no duplicate/degenerate faces. For the full
   fixture, compare the emitted surface with the measured counts and area
   above; use a scale-aware geometric tolerance, not exact cross-platform area
   summation equality.
4. Validate the compacted `ipc::CollisionMesh` map dimensions and indices,
   finite coefficients, and row sums. For the `dof` path, each used proxy
   vertex must have one unit selector. Test constant translation, affine
   displacement, and an admissible nonuniform FE displacement against
   evaluation of the constrained basis on representative boundary points.
   For a `max_order` alternative, test weighted rows rather than insisting
   that all rows are selectors.
5. Check self-intersection and mapping, which the current Python topology
   check did not establish. Retain the existing named refusal for truly
   unsupported extraction and its `[hex_collision_surface]` tests.
6. Run the complete one-step `stretch-cubes` fixture and require the original
   six metrics at 1e-5; then `standard` plus the affected proxy/basis tests.

If any map/topology check fails, repair `extract_boundary_mesh_nodal` (and
its minimal reproducer) before shipping the profile. Do not bypass the
completeness check. Scene completion alone does not prove the map is correct.

## 7. G3 / CI-05 — Give microstructure a measured resource profile

Coordinate with [RB-05](rb-05-validation.md). The refusal happens during the
first test step: trial Linf 0.98; HashGrid has 37,732 vertex, 126,302 edge, and
89,025 face **items** over a 9x8x9 grid. Its mesh has 19,240 collision vertices,
57,768 edges, and 38,512 faces. The limit is on pre-filter pair emissions,
not physical contacts, retained candidates, or allocated bytes.

Steps:

1. Reproduce with Triangle ON, the original irregular proxy with max edge
   length 0.1, dt 0.01, **one step**, and automatic resource limits. Save the
   named refusal and verify standalone exit 3. Keep a small independent
   negative regression proving the default resource protection still works.
2. In an isolated characterization run with a recorded host memory ceiling,
   try a **fixture-only candidate-emission limit of 75,000,000**, retaining
   the automatic cell-item limit. This is a diagnostic starting point above
   the observed demand, **not an approved final budget**. Keep a timeout and
   RSS supervision; stop and record if it exceeds the host's safe envelope.
   Do not escalate the budget repeatedly without reviewing measurements.
3. Record the maximum raw demand across **every detection call** in the step,
   surviving/deduplicated candidates, actual peak process RSS, and runtime.
   `BroadPhaseBuildStatistics::candidate_emissions` can aggregate multiple
   detection passes, while the cap is checked per pass. Label and distinguish
   these quantities. Capture pre-refusal values as well as successful calls.
   Inspect `ContactForm.cpp`, `ipc/broad_phase/hash_grid.cpp`, and
   `ipc/broad_phase/broad_phase.hpp` for existing counters before adding probes.
4. Repeat the successful candidate on Linux and macOS runner classes, first
   alone and then at their normal CTest concurrency (4 and 3 respectively).
   The suite's other jobs contribute to memory pressure. If necessary give
   `triangle_data` `RUN_SERIAL` or suitable CTest resource scheduling rather
   than raising a global solver limit.
5. If a bounded fixture-specific budget fits both runner envelopes, record
   the measured headroom and put only that justified cap in its G0 profile.
   If it does not, keep the default refusal regression in the normal suite
   and move the original full scene to an explicitly required job with
   adequate resources. Preserve the actual one-step scene and its six
   reference assertions there; merely hiding it in a disabled nightly job
   would remove coverage.
6. After unblocking the solve, compare all six original metrics. This case
   has friction coefficient 0.3 and also inherits the new budget default;
   a newly exposed numerical mismatch must get the G1 A/B treatment before
   it can be called repaired.

The **diagnostic-only** patch for step 2 is:

```json
{"solver": {"contact": {"CCD": {"resource_limits": {
  "max_cell_items": -1,
  "max_candidate_emissions": 75000000
}}}}}
```

It changes no source default and does not disable the limits. Use it only
on the copied microstructure input during characterization; do not publish
it as the final profile until the resource and numerical checks above pass.

A BVH/sweep-and-prune experiment is a separate alternative, not a silent
substitute: current automatic count limits are not enforceable there, and
the implementation logs that fact. A switch needs explicit memory/CCD
coverage and a support-policy decision. Likewise, changing the broad-phase
algorithm to reduce conservative enumeration requires collision-pair
equivalence tests. Neither route is necessary if a modest, measured
fixture-specific cap works.

Acceptance: the original physical/discretization problem completes under a
documented, bounded resource profile; its golden passes; tiny/default-limit
refusals still fail cleanly; no partial candidate set is used; required CI
coverage remains present. The final cap and scheduling remain open decisions
until those measurements exist.

## 8. G4 / CI-06 — Resolve GCP ordering before choosing reference tolerance

The reference history is real but insufficient by itself: `polyfem-data`
commit `f52db28` changed this scene's margin to 0.01; bulk regeneration
`6f8f569` replaced it with 1e-5. G0 fixes the overwrite mechanism. Do not
immediately restore 0.01: that would hide the presently demonstrated
fresh-process variation without establishing its numerical acceptability.

Start at these sources:

- PolyFEM `solver/forms/SmoothContactForm.cpp`:
  `update_collision_set`, `value_unweighted`, and derivative methods.
- IPC `src/ipc/smooth_contact/smooth_collisions_builder.cpp`:
  `SmoothCollisionsBuilder<2>::merge` and then the 3D counterpart.
- IPC `src/ipc/utils/unordered_map_and_set.hpp`: active aliases use
  `tsl::robin_map` with `absl::Hash` in this build.
- IPC `src/ipc/smooth_contact/smooth_contact_potential.cpp`: collision-list
  traversal, thread-local sums, and sparse-Hessian triplet assembly.
- PolyFEM `utils/par_for.hpp`: requested thread count installs TBB's global
  parallelism control. Single-thread work alone does not stabilize a
  process-randomized hash iteration order.

Steps:

1. Reproduce **at least five fresh processes** with identical resolved input,
   binary, one thread, and all 20 steps. Record final solution arrays, all
   six metrics, accepted endpoints, solver stopping reasons, and energy/
   gradient information. Run the same scene twice in one process as a
   separate control for process-dependent versus state-lifetime behavior.
2. At identical saved geometry/iterates, fingerprint both the **ordered**
   collision sequence and its canonically sorted semantic membership. Include
   collision type, primitive IDs, weights, and any per-collision parameters.
   Do not use pointer addresses or hash values as stable identifiers.
3. If membership/weights agree and order differs, evaluate E/g/H in fixed
   semantic order at those same iterates. Locate the first difference in
   the solve trajectory. This is the missing causal experiment; the hash
   probe alone does not authorize claiming the full root cause.
4. If confirmed, make the narrow IPC correction: construct a canonical
   collision sequence after merging, using stable semantic identifiers;
   preserve primitive-local orientation and all weights. Check duplicate-key
   merging as well—choosing different records on a first-insert-wins path
   would not be repaired merely by sorting the final vector. Extend to 3D
   only with its corresponding invariants and tests. Avoid changing global
   hashing or friction/contact models to fix this one path.
5. Add insertion-permutation and fresh-process regressions, plus E/g/H
   agreement checks at fixed inputs. Run the relevant IPC smooth-potential
   tests in `tests/src/tests/potential/test_smooth_potential.cpp`, including
   derivative tests, and PolyFEM's affected GCP/contact/cache tests. Then
   repeat the scene on Linux and macOS. One-thread exact-repeatability and
   multi-thread numerical-repeatability are separate acceptance claims.
6. If fixed ordering does not resolve the variation, retain the negative
   result and inspect contact membership, duplicate accumulation, and
   thread-local Hessian reduction at the first divergent iterate. Do not
   sort away a real difference in the contact set. If evidence points to
   invalid arithmetic or initialization instead, isolate and repair that
   implementation defect with a minimal failing control.
7. Once repeatability is characterized, choose the reference policy from
   independent residual/solution comparisons and a diagnostic tighter-solve
   or timestep-refinement control. Keep that control distinct from the
   original 20-step acceptance scene. A deterministic correction may still
   differ from an old reference sampled under a different summation order.
   Such a remaining offset needs review; determinism does not make a new
   endpoint automatically correct.
8. If the justified outcome is a scene-specific margin, document the
   measured platform/process envelope, comparison metric, chosen bound,
   and sensitivity to an intentionally perturbed control. The historical
   0.01 is a candidate to evaluate, not an already validated threshold.
   Change only this scene's policy through G0 (or the selected versioned
   data route), keeping all other margins at their intended values.

Acceptance: explained/repaired implementation defects, documented residual
and repeatability evidence, an explicit defensible reference policy,
preserved margin generation, and repeated passing `contact_2d` on both
Release platforms. Do not label a configured directional-derivative
termination as a crash; do not relabel every configured convergence as
physical equilibrium.

## 9. G5 — Reproduction commands, publication, and final acceptance

Use an isolated checkout of the current implementation branch. Record the
chosen commits before running these commands; they intentionally do not
reuse the shared developer build. The following is the common Release
configuration; retain required native dependency setup from the workflow:

```sh
cmake -S . -B build-golden -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOLYFEM_WITH_TESTS=ON \
  -DPOLYFEM_WITH_TRIANGLE=ON \
  -DPOLYFEM_THREADING=TBB
cmake --build build-golden --target unit_tests PolyFEM_bin --parallel 3
build-golden/PolyFEM_bin --build_info
ctest --test-dir build-golden -N -R '^(standard|contact_2d|contact_3d|triangle_data)$'
ctest --test-dir build-golden --output-on-failure --parallel 1 \
  -R '^(standard|contact_2d|contact_3d|triangle_data)$'
```

Linux CI additionally sets `POLYFEM_PORTABLE_BUILD=ON` and `OMP_NUM_THREADS=1`.
Use those in its acceptance build. Verify effective IPC/PolySolve pins and
source overrides with `--build_info`; an old cached dependency is not a
test of a newly published IPC correction. Existing native CI sets up the
remaining options; compare its actual build record rather than assuming
these few flags describe the whole environment.

Catch's four group names do not select one manifest entry. For a focused
scene, either use an isolated manifest copy in the isolated checkout or
introduce a documented exact-scene selector in G0. Never rewrite manifests
in the shared checkout. The retained `probe.py` is an investigation aid;
it supports these particular common/patch chains and is not a general JSON
resolver to promote without additional coverage.

Mirror the harness exactly when using the CLI: resolve common files and
patches, preserve geometry paths, select Eigen::SimplicialLDLT for these
six fixtures, request one thread, and use the table's horizon. Run each in
a fresh directory. Compute comparisons as the harness does:

```text
abs(current - stored) / max(abs(stored), 1e-5) <= effective_scene_margin
```

Require all six finite metrics (`err_l2`, `err_h1`, `err_h1_semi`, `err_linf`,
`err_linf_grad`, `err_lp`), process success, and the intended completed
horizon. A partial output JSON is not a pass. Retain the original reference
values during the G1/G2/G3 investigations.

Deliver each package as a bounded commit with its negative control,
implementation/configuration change, focused test, and updated validation
record. Publish IPC fixes to `sdast9/ipc-toolkit` on its maintained branch
before advancing PolyFEM's recipe pin. Commit/push PolyFEM source and
profiles to `sdast9/polyfem` under the standing project instructions; stage
only named files. Publish a changed data pin only after the data commit is
available from the repository in the recipe. Update the CI plan and the
relevant RB records without marking other packages complete.

Final gate:

1. Rebuild after every source/dependency change and run the focused affected
   regressions plus all four scene groups. Keep exception/refusal regressions.
2. Run the complete Release suite on Linux and macOS, and the appropriate
   Debug/Windows suites. Confirm the four groups are registered and actually
   ran. Resolve or explicitly retain the separate Linux Debug rollback
   segfaults; report them independently.
3. Repeat the GCP scene in fresh processes on both Release platforms; run
   the chosen resource profile at normal suite concurrency or its explicit
   resource schedule. Run the pinned formatter and existing required checks.
4. Publish the commit IDs, dependency/data pins, run/job links, per-group and
   per-scene results, preserved failures, and reference/default differences.
   Make no overall portability or physical-accuracy claim beyond that evidence.

Suggested next-model starting request:

> Read this plan and current AGENTS.md. Implement G0 and the confirmed
> large-mass-ratio portion of G1 in isolation; reproduce the Triangle-enabled
> P4 A/B before adding its profile. Preserve production defaults and original
> golden values. Run focused checks, publish the bounded change, update the
> record, and report remaining G2/G3/G4 and separate Debug failures.
