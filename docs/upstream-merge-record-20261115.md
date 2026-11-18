# Upstream merge execution record

## 2026-11-16 — PF-1: conforming tetrahedral refinement

Merged upstream `d7525f554162f95307d587ac012bd870a14fc40c` (#547) into the isolated `upstream-merge` branch from fork `7aa161108daa87e53c2a0c2d93a3cd4a2bf3c8d2`.

Test data merged first: upstream `b125252765d8b3bae72e876743d57263f2ba705e` into fork `aed03ab9abf9eb54ca21dcbc3df3682dc8176e7f`, published as `feba1481682285a7febf896211a980a45a5c5e3b` on `sdast9/polyfem-data:fable-fixtures-upstream-merge`. All nine fork fixture files remained byte-identical; the isolated data checkout remains clean after testing.

### Resolution and preservation

Only expected conflict: `cmake/recipes/polyfem_data.cmake`. Kept the fork repository/recipe, changed only its data pin to the published data merge above. No files deleted. Upstream mesh-refinement changes and new tests retained; no fork changes were dropped. Upstream also adjusts the fixed-corotational derivative test's finite-difference step (not a production tolerance). No solver tolerance, controller default, CCD safeguard or trial-displacement cap was altered.

The plan's global configuration note says to point at the IPC integration worktree once available, while IPC-1/PF-4 specifically place PolyFEM consumption in PF-4. To isolate PF-1 and avoid adopting IPC behavior prematurely, this stage uses an isolated detached `ipc-pre-pf4` worktree at the unchanged fork pin `7b8fcac7`. PF-4 will switch to accepted IPC-1. PolySolve remains `f175cabe11deea2d3cafb74a6dec21a9e680ef69`. ITR is OFF because RM-0 has not passed; MISO remains OFF. No remeshing functionality is claimed, removed, or replaced by this task.

The old integration binary had dirty/older source provenance and was not used as an oracle. Built a clean pre-merge baseline at `7aa161108d` first, preserving its binary, build info and hash. All comparisons below use that clean baseline and identical dependency commits/settings.

### Ladder results

- **V1:** `PolyFEM_bin` and `unit_tests` compile successfully (AppleClang 21, macOS arm64, RelWithDebInfo, four build jobs).
- Additional focused refinement check: **2 cases, 83 assertions passed**.
- **V3:** all five one-thread smoke scenes exited 0 with no error lines. `quasistatic-semi`, `quasistatic-semi-friction`, `transient-semi`, `quasistatic-adaptive`, and `quasistatic-semi-alhess` each produced **five byte-identical VTUs**, 25/25 total, versus the clean baseline.
- **V5:** entire default `unit_tests` invocation with seed 1, no tag filter, under `caffeinate -s` and a 7,200-second watchdog: **443 cases, 5,179,481 assertions passed**, exit 0, 1,155.54 seconds. No shortened test selection, timeout, accepted failure or tolerance adjustment. This native result does not validate other platforms or CUDA.

Evidence is in sibling `PF-1/`: configure/build logs and exit receipts, `fork-fixtures-preserved.json`, focused-refinement log/receipt, both smoke directories and receipts, `smoke-comparison.json`, and full-suite log/start/receipt. Baseline and tested binaries/build-info snapshots are under `bin/PF-1-pre/` and `bin/PF-1/`.

PF-1 meets its specified native gate. As already stated in the plan, #547 changes refined tetrahedral results for `n_refs > 0`; no smoke uses refinement. Keep this behavior change on the REL-1 disclosure/confirmation list. No release, shared-checkout change, main-branch push or Houdini publication is authorized by this result.

### Related independent IPC-1 status

IPC-1 completed separately: native toolkit default suite passed 453 cases / 5,802,589 assertions, new interaction checks and Python flag bindings passed. Integration tip is `eec0ff82d2b7a0a9ce27fff0afd7de2c18e57d3a` after whitespace-only CI fixes. Remote formatting workflow 43910976196 succeeded. Its consumer validation remains at PF-4. RM-0 evidence/experimental branch is unchanged and still separate.

## 2026-11-17 — PF-2a/PF-2b: upstream IO and fork checkpoint continuation

Combined acceptance unit, as authorized by Steven: upstream checkpoint is the baseline, with fork continuation capabilities added on top. No old `restart.json` compatibility adapter is required. Merge parent is PF-1 `c1bfa2cf1742f5d2e47c5b0d9714648e6b64e034`; upstream parent is `946c0b372941a97a936aead4a116a45b131f0212` (#541). No intermediate checkpoint implementation without fork history is published.

Data prerequisite: local data merge `270ece1f73fd10891ae1ee8f592a1a2942e50688` adds only upstream `io-tests/checkpoint-quick.json` and `io-tests/hdf5-quick.json`. Existing fixtures are unchanged. Publish that dependency before the source pin; neither PF-2 source nor this data merge has been pushed at the time of this record. IPC remains frozen at `7b8fcac7`, PolySolve at `f175cabe`, MISO and ITR remain OFF. Shared checkout/build and RM-0 are untouched.

### Conflict resolutions and replacements

The 39 conflicts are accounted for below. Nonconflicting upstream changes are retained; fork-only call sites were migrated to the new APIs without deleting their tests.

- `cmake/recipes/polyfem_data.cmake`: retain fork recipe and pin the isolated merged fixture commit above.
- `src/polyfem/State.cpp`, `State.hpp`, `main.cpp`: upstream ResourceIO/checkpoint entry points plus fork run manifest, named failure/resource exit codes, thread capping and zero-time-step validation. Include the complete checkpoint type for public overload resolution. Retired `/output/restart_json` now produces a named migration error pointing to `/output/checkpoint/path`, in strict and non-strict input modes.
- `src/polyfem/assembler/Assembler.cpp`, `Assembler.hpp`, `GenericElastic.cpp`, `GenericFiber.cpp`, `HookeLinearElasticity.cpp`, `MatParams.cpp`, `MatParams.hpp`, `OgdenElasticity.cpp`, `SumModel.cpp`: upstream resource-aware signatures with fork element-wise materials, fiber/HGO laws and domain validation. Preserve immutable material snapshots and bindings; resource cache identity replaces the old filesystem-only key. Growth-NeoHookean template instantiation follows upstream's split layout.
- `src/polyfem/io/CMakeLists.txt`, `MatrixIO.cpp`: retain fork row-bounded compression and upstream checkpoint IO; share the row-chunk calculation with CheckpointWriter so both writers preserve payload-size bounds.
- `src/polyfem/legacy/state/StateInit.cpp`, `src/polyfem/mesh/remesh/wild_remesh/LocalRelaxationData.cpp`: retain fork behavior atop the upstream compile-level API changes. This is not remeshing validation or activation; RM-0 remains open.
- `src/polyfem/mesh/GeometryLoader.cpp`, `Mesh.cpp`, `MeshUtils.cpp`, `MeshUtils.hpp`, `Obstacle.cpp`: upstream resource loader/mesh-data interface with fork malformed-input, repeated-cell, rest/surface, obstacle and topology checks retained at their new ownership boundaries. Checks precede unsafe indexing. Existing MSH parser and patched dependency remain; their malformed-number and stride tests pass.
- `src/polyfem/utils/ExpressionValue.cpp`, `JSONUtils.cpp`, `JSONUtils.hpp`, `Selection.cpp`: preserve named input diagnostics, material cache and selection repair under ResourceIO. Missing-file diagnostics use resource descriptions; cached immutable expressions remain usable after source deletion.
- `src/polyfem/varforms/ElasticVarForm.cpp`, `IncompressibleElasticVarForm.cpp`, `LinearElasticVarForm.cpp`, `NavierStokesFSIVarForm.cpp`, `NonlinearElasticVarForm.cpp`, `ThermoElasticVarForm.cpp`, `VarForm.cpp`, `VarForm.hpp`: retain fork staged steps, rollback, fully prescribed bodies, linear force normalization and output-time phase. Replace the retired JSON restart writer with upstream checkpoint serialization; preserve integrator restoration before form construction and defer AL/contact/friction restore until initialized forms exist. Output frame zero uses upstream's restored output-index interface. The transient ResourceIO constructor is out-of-line to retain ownership of its private run state.
- `tests/CMakeLists.txt`, `test_assembler.cpp`, `test_hdf5.cpp`, `test_restart.cpp`, `verify_run.cpp`: retain upstream and fork tests, port resource arguments and mesh APIs, register all four original restart cases under their existing tags. Preserve numeric margins, exact first resumed state, nonzero AL and run-scope trim checks. The old reference copy was archived outside the source tree after the port passed, then removed from the tree. New tests cover migration diagnostics, checkpoint chunking/size/readback and malformed/identity-sensitive owner state.

### Checkpoint semantics and safety

Versioned `contact`, `friction` and `al` owners under `/checkpoint/state/fork/` store the fork's existing state, not a duplicate solution or integrator history. Missing owners retain upstream-only fallback behavior; unknown versions, contradictory shapes/counts, nonfinite values or missing required datasets fail by name. AL vectors are all validated before assignment. Controller canonical keys, counts, run-scope flags and existing SI v1/v2 scalar interpretations are retained.

Friction state now carries typed primitive identities, orientation, weight sign and plane coefficients where applicable. Restore requires a unique matching bijection, reorders rebuilt typed collision objects (preserving their auxiliary data), then applies saved forces before committing state. Ambiguous or missing identity fails instead of silently replacing history. Regression tests change both rebuilt order and force values and check atomic failure for invalid identity/trim data.

Checkpoint node reordering with contact/friction history is explicitly refused. AL reordering is permitted only for empty or finite exactly-zero multipliers; even a tiny nonzero value is refused. No tolerance-based history deletion was introduced. The general friction reconstruction still relies on deterministic rebuilding of geometry/tangent data rather than serializing every internal field; tested single-threaded scenarios match exactly, not a promise of cross-platform bitwise identity.

Thermoelastic/FSI form-history limitations were traced to PF-1's actual save/restore call paths and are preexisting, not a PF-2 capability removal. They are not newly repaired or claimed as exact restart coverage here. No solver tolerance, controller default, CCD safeguard or trial-displacement cap was changed to obtain parity.

### Validation

- **V1:** final build (seventeenth recorded pass) successfully compiled PolyFEM_bin and unit_tests. Earlier API migration failures and interrupted attempts remain archived, not counted as passes.
- **V2:** prescribed combined tags passed **82 cases / 20,612 assertions**, exit 0. Additional progressively strengthened restart/checkpoint passes and all four formerly failing focused cases passed. Final retired-JSON diagnostic passed strict/non-strict checks.
- **V3:** all five one-thread smoke scenes exit 0, no error lines, **25/25 VTUs byte-identical** to PF-1. No altered solver history was accepted.
- **V4:** RB02 **270/270 checks**; RB05 count estimates equal measured/published representative counts; scene-limit refusals and aborted-attempt checks match expectations, default/generous outputs byte-identical to unlimited.
- **V5:** final snapshot's entire default suite, seed 1, four OpenMP threads, isolated Python with h5py/numpy, MKL_CBWR=COMPATIBLE, under caffeinate -s with a 7,200-second watchdog: **469 cases / 5,180,097 assertions passed**, exit 0, 2026-11-17 01:34:32 UTC (about 19 minutes). The earlier interrupted V5 has no terminal result and is not a pass. This is native macOS arm64 coverage, not remote/cross-platform CI.
- Extra checkpoint parity: quasistatic-semi-friction and transient-semi write midpoint checkpoints and resume in fresh processes; **6/6 remaining VTUs byte-identical** to uninterrupted outputs. The original writer runs to completion after writing the midpoint checkpoint; the comparison does not simulate power loss during checkpoint writes.
- Pinned clang-format 21.1.8 passed all changed extant source files, including the three new headers; diff whitespace check passed.

Evidence is in sibling `PF-2a/`: chronological CHECKPOINT.md, all build/test exit receipts and logs, `v2-first/`, `smoke-first/`, `smoke-comparison.json`, `rb02/`, `rb05/`, `rb05-scene/`, `checkpoint-smokes/`, and `full-suite-second/`. Final executable/build-info/hash snapshots are in `bin/PF-2-validation-final/`; earlier snapshots remain preserved. Publication and later Houdini/merge-plan stages remain separate gates; no main-branch or shared-build update is implied.

## 2026-11-17 — PF-3: corrected upstream AMIPS

Merged upstream `008a1aeae97b277c8bcf644adf0673749b7889ef` (#487) onto PF-2 `c1066a57ca3a35450e30d424a281f7fc6a21ac06` in the isolated integration worktree. Data prerequisite is local merge `f92cfdcff8a7567e392d82d93c3433bba14af478`, merging upstream `4fa1e3e6674253e57477b65730713cc168a4184c` onto PF-2 data `270ece1f73fd10891ae1ee8f592a1a2942e50688`. Only `standard/amips.json` reference values changed; no fork fixture was removed. Neither PF-3 merge has been published at the time of this record. PF-2 data/source were separately approved, pushed in that order and verified remotely; integration pushes generated no CI runs.

### Preservation decision and resolutions

Inspection found additional restrictions beyond the plan's brief energy summary. Work stopped before compilation, and Steven explicitly chose **“Accept upstream”** after the alternatives were explained. Adopt the corrected regular-element map, determinant exponent 2/d, per-element normalization, corrected regular tetrahedron and per-point material parameters. Absolute mode (`use_rest_pose=false`) now supports straight simplices with linear displacement, emits NaN rather than misleading physical stress, and refuses stress derivatives. Deformation-gradient output remains available. Rest-pose AMIPS, including the existing FSI scenes and optimization AMIPSForm, is retained. WMTK remeshing's AMIPS implementation is separate and was not changed.

The removed stress calculation differentiated the wrong map; retaining its old numbers would preserve reproducibility rather than correctness. No legacy compatibility mode, silent rest-pose substitution or bypass of upstream guards was introduced. A bounded source/JSON review found no existing fork-specific use of the affected paths, but did not cover binary HIP files or archived/external projects. This is an accepted upstream scope change, not a claim that nobody could have used those APIs.

**Future goal:** a separately designed and tested broader AMIPS extension, if needed: define higher-order and non-simplex reference-cell distortion functionals and correctly derived energy-conjugate outputs/adjoints. Do not restore incorrect legacy numbers under the corrected model's name. No such extension is implemented or promised as validated here.

Two textual conflicts (rather than the plan's predicted zero): `cmake/recipes/polyfem_data.cmake` retains the fork repository/recipe with the merged data pin; `tests/test_assembler.cpp` retains both the fork string-matcher include and upstream generator include. No file deletions. Automatic merges retain the fork material-cache/validation behavior; upstream adds boolean ExpressionValue input support. IPC remains `7b8fcac7`, PolySolve unchanged, MISO/ITR OFF. Shared checkout/build and RM-0 remain untouched.

### Validation

- **V1:** PolyFEM_bin and unit_tests built successfully, exit 0 (macOS arm64, RelWithDebInfo, four build jobs).
- Additional focused `[amips],[amips_form],[material_cache]`: **5 cases / 187 assertions passed**, including upstream 2D energy/gradient/Hessian consistency, high-order refusal, absolute stress/F behavior, and fork material snapshots.
- **V3:** five one-thread smoke scenes, **25/25 VTUs byte-identical to PF-2**, all exit 0. No changed solver history accepted or tolerance adjusted.
- **V5:** default full suite, seed 1, four OpenMP threads, isolated Python, MKL_CBWR=COMPATIBLE, caffeinate -s and 7,200-second timeout: **470 cases / 5,180,153 assertions passed**, exit 0, **1,150.73 seconds**, completed approximately 02:26:59 UTC. No test filtering or accepted failure. These results are native coverage, not remote/cross-platform CI.

Evidence: `PF-3/build-first.*`, `focused-first/`, `smoke-first/`, `smoke-output/`, `smoke-comparison.json`, `suite-first/`, `post-gates.*`, source/usage review and data patch. Tested binary/build-info/hash snapshot: `bin/PF-3-validation/`. Dedicated new 3D vertex-permutation/scale-invariance property tests were suggested during review but not added; the existing standard 3D AMIPS scene and full suite passed. Later publication and REL-1 remain separate; main and installed Houdini assets are unchanged.

## 2026-11-17 — PF-4: ESP and IPC-1 consumer integration

Merge upstream `cc1eac01e1` onto accepted PF-3 `040429ca0aedc0444f506912c0f80bc3b2871f83`, consuming the separately accepted IPC-1 `eec0ff82d2b7a0a9ce27fff0afd7de2c18e57d3a`. PolySolve remains the fork pin `f175cabe11deea2d3cafb74a6dec21a9e680ef69`. Data prerequisite is local merge `efb3095ee97e490bb65edfba922fc9de689bc68c` (upstream `c103d197b7f4d5474208c9b0a470c81349e75dd7` onto PF-3 data): the only fixture change adds upstream's two Newton PSD-projection settings to `gcp-contact/parallel-edge/run.json`. No fixture is deleted or reference regenerated.

### Conflict resolutions and preservation

Seventeen textual conflicts, rather than the plan's predicted thirteen:

- `.github/workflows/continuous.yml`: retain the fork's MKL-compatible, single-thread test environment while adding upstream's job-level OpenMP setting.
- `cmake/recipes/ipc_toolkit.cmake`, `polyfem_data.cmake`, `polysolve.cmake`: retain fork recipes/repositories, update IPC and data pins to the verified integration prerequisites, preserve PolySolve pin. The isolated configuration now points at `ipc-toolkit` instead of the frozen pre-PF4 copy. MISO/ITR remain OFF, Python ON.
- `json-specs/input-spec.json`: keep all fork settings and add ESP, barrier, collision-set type, obstacle skipping and adaptive-update scale. JSON parses and no previous pointer is lost. The preexisting schema deliberately repeats 26 pointers for alternative types; rather than delete valid alternatives to satisfy the plan's literal uniqueness wording, verify no additional repeated pointer was introduced (`spec-preservation.json`).
- `legacy/state/StateHomogenization.cpp`, `StateSolveNonlinear.cpp`, `mesh/remesh/wild_remesh/LocalRelaxationData.cpp`, `varforms/NonlinearElasticVarForm.cpp`, `varforms/diff/DifferentiableNonlinearElasticVarForm.cpp`: combine new contact arguments with all existing semi-implicit options; retain legacy/remeshing code and deferred checkpoint restoration. No remeshing activation or claim follows.
- `mesh/GeometryLoader.cpp`: retain fork surface validation/resource diagnostics and upstream obstacle-volume loading.
- `solver/SolveData.cpp`, `SolveData.hpp`: keep fork stiffness-mode selection, lumped mass, initial trim, clamped/Dirichlet contact mapping, gradient/Hessian providers, budget/restart/failure hooks and rollback wiring. Add ESP as an opt-in branch; refuse semi-implicit ESP/GCP/periodic combinations by name.
- `solver/forms/BarrierContactForm.cpp`, `BarrierContactForm.hpp`: start from the complete fork implementation and replay the small upstream API changes. Retain fork caches, experimental gap-floor handling, controller and checkpoint state. Explicit non-IPC collision type takes precedence over the legacy improved-max boolean; its effective type controls the signed-parent requirement. Semi-implicit mode refuses OGC, custom barriers and obstacle skipping, in addition to existing unsupported shape/physical modes. Adaptive stiffness keeps upstream's squared-distance guard and update scale (default 1e-9). No obsolete cache was reintroduced.
- `solver/forms/FrictionForm.cpp`: retain realized lag, coefficient assignment, objective-change notifications and checkpoint methods. Rebuild copies collision type, signed-contribution and obstacle-skipping policies. ESP friction now explicitly uses the same normalization policy as its normal potential rather than silently taking the toolkit default.
- `tests/test_diff.cpp`: retain the fork's documented forward-solve accuracy settings and assertions, not upstream's competing replacement. No test tolerances were loosened to make results pass.

Smooth-contact classes/APIs migrate to GCP, including the fork rollback snapshot's collision set. Existing contact/friction/AL state remains retained. No source file deletion beyond the upstream SmoothContactForm-to-GCPContactForm rename; shared checkout/build and RM-0 are untouched.

### ESP checkpoint safety boundary

The existing checkpoint keys identify typed primitive pairs, orientation and sign. ESP can generate multiple quadrature contributions with identical such keys and distinct forces. The new actual 3D fixture demonstrates this ambiguity. Assigning occurrence indices from rebuilt order or aggregating forces would silently lose correspondence.

Therefore **ESP with nonzero friction and checkpoint output/resume is refused by name before geometry loading**, with defensive serialization/deserialization checks as well. This new combination was not present in the pre-PF4 fork; ordinary ESP simulations and existing barrier/GCP restart remain available. ESP-specific quadrature history serialization is future work. A zero-friction ESP configuration is not covered by this refusal, but exact ESP-only continuation was not independently certified by these tests. Do not advertise general ESP restart equivalence.

Tests cover strict/non-strict output/resume refusal without loaded geometry, supported neighboring configurations, explicit improved-max selection with the old boolean false, signed-parent requirements, and unvalidated-option refusals. The normalization fixture compares actual ESP friction forces to explicitly configured toolkit results for both policies and confirms duplicated primitive identities.

### Validation and retained failures

- **V1:** PolyFEM_bin and unit_tests built successfully. Initial compile failures were only test-constructor migration errors; their logs and corrections are retained. Final compiled-source snapshot is `bin/PF-4-validation/` with binary hashes/build info.
- Added focused checkpoint/contact checks: **25 cases / 412 assertions passed**. The first attempt's one failed new test omitted required geometry/materials; only its fixture was corrected, with all assertions retained.
- **V2:** prescribed fork tags, contact-form/ESP tests and all ESP scene cases. First pass: **65/66 cases passed, 3,921/3,922 assertions**. The sole failure was dolphin-funnel's relative `source/data` path, not a numerical mismatch. Both meshes existed in the isolated external data checkout. Add an ignored `polyfem/data -> ../polyfem-data` link inside the integration worktree; rerun that exact scene unchanged: **1 case / 3 assertions passed**. The ESP runner also resolves the tracked `esp_data` folder from the source test directory instead of assuming the data directory is nested in the source. No test skip, input/reference edit or tolerance change. Full V5 below includes all these cases again.
- **V3:** all five one-thread smokes exit 0, **25/25 VTUs byte-identical to PF-3**. The potentially different IPC tie/controller history did not occur in these comparisons; no difference was accepted or tuned away.
- **V4:** RB02 exits 0; RB05 probe and scene-limit runs exit 0. Strict comparison against accepted PF-2 evidence confirms identical probe estimates, measurements, candidate counts, expected exceptions/build outcomes and process exits. Scene default/generous results equal unlimited, and all expected refusal outcomes match. `v4-comparison.json` reports passed.
- **V5:** full default suite, seed 1, four OpenMP threads, isolated Python, MKL_CBWR=COMPATIBLE, caffeinate -s and 7,200-second timeout: **480 cases / 5,181,433 assertions passed**, exit 0, **1,147.83 seconds**, completed about 03:48:57 UTC. Native macOS arm64 coverage, not cross-platform CI.

Evidence: sibling `PF-4/` chronological checkpoint, all build/failure logs, focused attempts, `v2-first/`, `esp-path-recheck/`, `smoke-output/`, comparison JSON, RB02/RB05 results and `suite-first/`. No PF-4 source/data publication or main/shared-build update is implied by this local acceptance record. HDA-1 and later release gates remain separate.

### Final audit repairs: acceptance still pending

The first full-suite pass above predates two final-review repairs and is not final PF-4 acceptance. Two-dimensional obstacle-volume cells were labeled tetrahedra; OutData now chooses triangles for three-node connectivity and tetrahedra for four-node connectivity. ESP rollback previously inherited only base metadata, leaving trial collision/cache state behind. ESP now builds a fresh immutable shared collision container on geometry changes and snapshots/restores that container together with its matching displaced-surface cache and base state. This preserves exact ordering without cloning noncopyable IPC internals. Independent read-only review found no blocking concern.

New repeated A-to-B-to-restored-A checks require exact value, gradient, Hessian and stiffness recovery. The 2D export regression requires all emitted cells to be triangles. Its initial fixture failures are retained: unsupported Dimension-2 Medit input, omitted explicit static export call, and a Medit surface import that did not supply obstacle triangle faces. The corrected fixture uses supported planar Dimension-3 Medit for the 2D body and explicit OBJ triangle faces for the obstacle, followed by the same export API as main. The triangle assertions and production tolerances were not relaxed.

The ninth incremental build succeeded, and final focused checks passed **27 cases / 432 assertions** (`focused-sixth/`). A new binary/source snapshot is in `bin/PF-4-final-validation/` and `PF-4/final-tested-source.patch`. Final V2–V5 verification is still running under a stop-on-failure continuation; earlier passes remain evidence, not a substitute for final-source gates. No PF-4 publication has occurred.

### PF-4 final local acceptance, 2026-11-17

Final-source verification completed successfully after the audit repairs:
- V1 incremental build passed; focused checks **27 cases / 432 assertions**.
- V2 **68 cases / 3,942 assertions**, exit 0.
- V3 all five smoke scenes exit 0 and **25/25 VTUs byte-identical to PF-3**.
- V4 RB02, RB05 and scene limits exit 0; strict count/outcome and default/generous versus unlimited comparisons pass.
- V5 **482 cases / 5,181,453 assertions**, exit 0 in **1,149.74 seconds**, completed approximately 04:38:51 UTC. Same native arm64, four-thread, seed-1 environment as the earlier pass.

Final evidence: `PF-4/v2-final`, `smoke-final-output`, `smoke-final-comparison.json`, `v4-final-comparison.json`, `suite-final/receipt.json`, and `FINAL-ALL-GATES-PASSED`. `final-source-audit.json` verifies no unresolved conflicts/deleted paths and no changes to compiled sources after the final binary snapshot (only this record changed). PF-4 is locally accepted within the documented ESP checkpoint boundary; publication and release/HDA gates remain pending. No cross-platform CI claim.

## HDA-1 preservation follow-up (isolated preparation, 2026-11-17)

V6 exposed two native regressions not covered by the earlier solver ladder: State diagnostic output had no nonlegacy writer, and ResourceIO input loading no longer supplied the original CLI file identity to run manifests. Both are repaired independently of checkpoint restart readers.

- `/output/data/diagnostic_state` writes the historical HDF5 `u/v/a` integrator histories, newest-column-first, including the former AL/contact/semi-implicit/friction diagnostic datasets. Writers are copied from the accepted pre-PF2 fork and renamed as diagnostics. It remains write-only; a diagnostic file is rejected by CheckpointReader. Existing overwrite/chunking behavior is retained. The isolated HDA `state_file` setting now uses this key; its name and `state.txt` output layout remain unchanged.
- LoadedInput carries original absolute JSON/YAML/HDF5 source identity separately from ResourceIO roots. Forward CLI manifests hash that source; direct checkpoint and JSON-wrapper resumes identify their respective CLI inputs. Library checkpoint initialization uses checkpoint identity without leaking it after exceptions. No provenance field enters simulation JSON, and resource resolution is unchanged. Optimization source-identity propagation is outside this repair's scope.
- Exact dataset checks cover all State/checkpoint output-toggle combinations and adaptive/semi-implicit contact. Additional fixtures require nonempty AL multipliers/friction forces and exact comparison with checkpoint data. Output toggles leave final solutions identical. Failed initialization/reuse, diagnostic rejection, size/history layout and all five forward CLI provenance routes pass.

Native validation: focused **24 cases / 521 assertions**; all five CLI provenance routes; full V2/V3/V4/V5 ladder, with V3 byte-identical smoke outputs and V4 unchanged resource results. Final V5 **483 cases / 5,181,577 assertions**, exit 0 in **1,219.83 seconds**. Snapshot `bin/HDA-preservation-validation`; evidence in `HDA-preservation/`. Pinned formatting passed. Earlier failures are retained.

The HDA-1 copy maps the existing Restart JSON toggle to self-contained checkpoint output, keeps old params.json import behavior, and adds an optional Resume Checkpoint field. Resume writes an absolute minimal input/checkpoint wrapper; saved checkpoint physics/time govern continuation. Working directory, executable, thread and log CLI controls still apply. Active-run and overwrite guards remain; resume requires no reconstructed geometry. Null checkpoint import and stale resume selection are handled. Actual HDA-generated launch-command execution and midpoint parity pass. Assets were built only in the isolated copy, never installed; installed-asset hashes are checked.

Additional V6 fixture adaptations preserve stronger assertions: static volumetric obstacles retain an interior output node, so the animated-obstacle test now checks exact exterior connectivity (26 nodes/48 faces), bijective coordinates and nonzero contacting-body response parity instead of equating all output nodes with surface vertices. Relocated reader scenes now explicitly name their original resource root. Reader tests support an isolated smoke root, and material-reader validation forces unique fresh output. Fresh manifests verify bound solver and input hashes. No collision behavior or tolerance was changed for these adaptations. Full HDA suite final status is recorded separately upon completion. HDA-1 remains prepare-only until REL-1; no publication or installed-asset update is implied.

HDA final verification: **31/31 test scripts passed** across the retained first 20 passes and the repaired remaining 11, all with installed-asset hashes unchanged. Independent read-only review found no blocker. The volume/surface body-response comparison uses fixed stiffness; it is not a claim of new SI controller equivalence (native SI gates passed separately). Source/asset hashes and combined results are saved in `HDA-1/validation/accepted-source-assets-sha256.json` and `all-accepted-results.json`. HDA changes remain isolated and uninstalled/unpublished.

## PF-5 — macro strain and Eigen mirror (2026-11-17)

Merged upstream `50628af3d7` (#546 macro strain, #554 Eigen mirror) onto
accepted `82ccc940388ab6bd37eb0bb5261bf0cdbb4f0fa8`, using a non-squashed
merge. Shared checkout, installed Houdini assets and remeshing baseline were
not changed. No deleted files. Fork IPC, PolySolve and test-data pins retained;
MISO and ITR remain OFF, Python ON. Upstream inflator remains opt-in OFF.

### Conflicts and preservation

Five conflicts, rather than the plan's predicted zero: retained fork PolySolve
recipe/pin per plan; retained both AL `initial_weight_multiplier` and upstream
`error` schema entries; retained fork collision rebuilding rather than
upstream's position-only cache in `BarrierContactForm.{cpp,hpp}`; combined
macro routing with existing ordinary static publication guard and clamped
contact filtering in `NonlinearElasticVarForm.cpp`. The new macro time loop
must be outside the fork's staged ordinary time-loop setup, otherwise the
merge runs both loops. Existing non-macro paths remain intact.

Upstream migrates the specialized static macro solver into the common elastic
VarForm and adds forward quasistatic macro loading. Historical audit of the
fork legacy loop and pre-PF-5 differentiable path found no disk restoration of
macro gradient or AL history. Legacy writes only generic restart JSON; its
macro state writer is commented out. This is not evidence of a working exact
macro restart. No old JSON adapter was reintroduced.

Added explicit macro checkpoint ownership: versioned gradient, macro AL
multipliers and periodic-contact state. Restore after form initialization,
reconstruct fluctuation as total displacement minus the linear macro field,
validate prescribed components at saved time, and start at the next step
without re-solving or publishing the completed checkpoint step. Missing,
invalid, nonfinite, incorrectly shaped and owner-mismatched histories fail;
macro history requires `input.checkpoint.reorder=false`.

Added complete macro problem snapshots for macro-owned forms, target values,
fixed projection and base constraint projection, and restore the saved full
state directly without rebuilding collisions during rollback. A macro step
failure restores extended solution, full solution and gradient before
rethrowing; publication follows acceptance. Existing AL/target snapshots now
also retain constraint values. Added the missing macro `line_search_end`
forwarding. The new upstream callback test exposed double projection:
`FullNLProblem::max_step_size` dispatched virtually back through a reduced
coordinate adapter. It now calls its full-coordinate probe explicitly; macro
probes include macro forms and observations report the final bound.

Periodic-contact friction previously dereferenced a null ordinary contact
owner. It now fails by name instead of crashing. Nonperiodic friction remains
available. Historical macro paths did not implement the ordinary outer
friction-lag convergence policy or ordinary AL budget/stagnation handling;
this task does not claim to add them, nor introduce arbitrary caps, tolerances,
normalization or controller changes. These remain separate extension goals.

### Validation

Evidence: `upstream-merge-work/PF-5/`; accepted binary snapshot
`bin/PF-5-validation`. All exploratory failures retained, including the first
callback failure, an oversized interrupted fixture, a test namespace typo and
an empty periodic-surface fixture. The bounded perforated-cell fixture now
requires a nonempty active contact set, not merely successful contact setup.

- V1: CLI and unit tests compile, seventh pass, pinned formatter clean.
- Macro focused: 4 cases / 265 assertions pass. Prescribed/free macro
  components, noncontact/active periodic contact, uninterrupted versus resumed
  solution and per-step macro histories within `1e-10`, exact injected-AL-failure
  solution/multiplier rollback, no duplicated/failed-step checkpoint, final
  step-bound observation and no collision rebuild during rollback.
- Ten malformed/reordered checkpoint CLI cases reject without publishing
  continuation (`malformed-second/results.json`). Reusable script:
  `tests/test_macro_checkpoint_validation.py` (h5py/numpy required).
- V2: 73 cases / 4,331 assertions pass, including ordinary restart, contact,
  AL budget, trim, optimization-gradient and macro tests.
- V3: all 25 VTUs across all five smokes byte-identical to the immediately
  preceding accepted State/provenance binary.
- V4: RB-02 and RB-05 probes and scene limits pass with unchanged expected
  outcomes/counts.
- V5: 486 cases / 5,181,838 assertions pass, 1,232.34 seconds; seed 1,
  OMP_NUM_THREADS=4, MKL_CBWR=COMPATIBLE, macOS arm64. Completed 16:15 UTC.
- Source patch matched the tested snapshot exactly before adding this record
  and the already-executed standalone negative-test script. Independent
  read-only preservation review found no remaining blocking issue.

No HDA rebuild or installation is needed for PF-5; HDA-1 remains prepare-only.
PF-5 is locally accepted; publication is to the integration branch only and
must be reported separately after remote verification. Next plan step RM-1
still depends on resolving the preserved RM-0 remeshing goal/gates; this record
does not declare remeshing functional or enable ITR.

## PF-6: upstream legacy removal and remeshing alignment (2026-11-17)

Status: local verification recorded below; no PF-6 publication yet.

The fork owner superseded remeshing decision 1: use upstream MMG shape-optimization
remeshing, not a revival of deprecated in-timestep dynamics remeshing. The plan
amendment records that distinction. RM-0 failures, binaries and exploratory
patches remain archived. RM-1/RM-2 are superseded, not falsely marked passing.
Merge target is `ddadb5398f`; legacy State and ITR are removed as authorized.
There is no dynamic displacement/velocity/acceleration/contact-history transfer
across remeshing. Old forward `space/remesh/enabled` inputs fail explicitly.

Preservation resolutions:
- Retain CLI named exits, exception wrapper, run manifest and input provenance.
- Retain checkpoint extensions and VarForm transaction/resource/contact safeguards.
- Retain `ImplicitTimeIntegrator::save_state` as write-only u/v/a diagnostics,
  despite upstream deleting it; it is not an obsolete restart reader.
- Merge upstream typed boundary setup and pressure/cavity selection; prior
  combined dispatch also skipped nonpositive boundary tags, so do not broaden
  that behavior speculatively.
- Port material-envelope and fork SI derivative fixtures from removed legacy
  State to ElasticVarForm, retaining numerical assertions. Remove only the
  redundant legacy cache-reinitialization section; current VarForm and explicit
  assembler snapshot/cache tests remain.
- Legacy restart/time stepping/validation replacements are the accepted PF-2
  through PF-5 paths. Legacy-only solver combinations are intentionally absent,
  per decision 2; unsupported configuration errors include configured features.
- Keep MISO off and fork PolySolve/IPC pins. MMG optimization remains enabled.

The isolated data merge from `8fdd300` changes only upstream's FSI reference
values in `standard/navier_stokes_fsi_simple_square.json`. Both full-suite logs
show the fixture authenticating; an independent focused rerun against the baked
binary passed **1 case / 3 assertions**, exit 0, and confirmed all six values to
roundoff (`fsi-authenticated-receipt-20261117/`). The fixture margin and time
steps are unchanged, and the new git pin is `ff5d77a7d216298cff19e5da4b61aacae2d6df12`. The local
build used the isolated existing data checkout with those same file bytes.

Pre-repair snapshot: V2 104 cases/5,761 assertions; V3 all 25 VTUs in five scenes
byte-identical to PF-5; V4 resource outcomes/counts unchanged; V5 489 cases /
5,181,989 assertions, 1,257.55 seconds. Binary and source patch are preserved
under PF-6. This snapshot is not proof of correct transformed MMG restarts.

### Separate bounded MMG restart correction

Actual CLI experiments found upstream preprocessing replay: MMG writes current
internal-coordinate vertices, but restart reapplies input transforms, units,
normalization and refinement. A separate correction resets only these settings
in the next-round working configuration for one enabled ordinary FEM mesh with
uniform actual body IDs. It retains selectors/materials/BCs, obstacles, disabled
entries and original input provenance. It adds no blanket refusal or tolerance
change and does not pretend to fix other geometry layouts.

An independent manual regression (`tests/test_opt_remesh_restart.py`, fixture
`tests/fixtures/opt-remesh`) executes two periodic MMG restarts and compares every
emitted mesh vertex to the next-round loaded geometry. All ten 2D cases pass:
plain, nonuniform scale, rotation/translation, mesh unit, normalization,
refinement, dimensions, explicit uniform body selector, disabled entry and
obstacle entry. The six preprocessing variants fail against the pre-repair
binary. Initial malformed rotation fixture and all failed evidence are retained.
### Verification on the corrected binary

- **V1:** isolated arm64 native build of `PolyFEM_bin` and `unit_tests` exited 0.
  Immutable binary digests and the build/source snapshot are under
  `bin/PF-6-baked-validation/` and `PF-6/baked-tested-source.patch`.
- **V2:** 104 cases / 5,761 assertions, exit 0. The independent MMG restart
  oracle passed all ten cases described above.
- **V3:** all 25 VTUs (five scenes) byte-identical to the accepted PF-5 binary's
  saved outputs, with identical file sets. The initial PF-6 comparator had
  accidentally used each candidate output as both operands; those comparisons
  are not evidence. An independent exact byte comparison against the retained
  PF-5 artifacts verified both PF-6 candidates; receipt and paired hashes are in
  `PF-6/smoke-v3-independent-pf5-comparison.json`.
- **V4:** RB-02 passed all 270 checks; all 24 RB-05 probe rows and the scene
  checks matched approved resource outcomes/counts. All gate runners exited 0.
- **V5:** the first corrected-binary run was interrupted by the power loss, not
  counted as passing. The clean, unfiltered rerun (seed 1, four OpenMP threads,
  isolated Python, MKL_CBWR=COMPATIBLE, 7,200-second bound) emitted Catch2's
  complete **489 cases / 5,181,989 assertions passed** summary. The separate
  supervisor's process-exit receipt and aggregate marker were not recovered;
  both test and supervisor had already exited when checked. The test report and
  its immutable binary/source identity were verified, but no numeric process
  exit is claimed. See `PF-6/v5-console-evidence-20261117.json`. This is local
  native evidence, not cross-platform CI.

Limits: multi-FEM-entry and mesh-array layouts retain pre-existing upstream
ownership bugs. Multiple body/interface labels and stored/indexed selectors are
not generally transported by the MMG wrapper; MshWriter does not currently emit
physical groups from its body_ids argument. Explicit geometric/uniform selectors
are retained, not proof of general label preservation. Object/array
volume_selection parsing and strict nested optimization-functional schema
limitations predate this merge. The manual workflow tests use existing upstream
non-strict optimization convention. No 3D optimization-workflow or general
boundary-response parity is claimed.

HDA-1 remains prepare-only. Its legacy remesh values stay readable/importable;
enabled controls explicitly block forward export, disabled exports omit the
retired key, and no remesh toggle changes contact weighting. Checkpoint resume
remains authoritative. Optimization input import refuses before mutating the
scene. Read PVD topology controls remain. Isolated build, legacy-remesh/import
reconstruction/main HDA tests passed; installed asset hashes stayed unchanged.
Evidence: `HDA-1/validation-pf6-remesh`. Each of the same three scripts was
also rerun against the exact frozen corrected `PolyFEM_bin`, with exit 0 and
unchanged installed asset hashes after each script; see
`HDA-1/validation-pf6-baked-native-20261117/receipts.json`. The sandboxed
Houdini invocation that could not detect NEON is retained as environment
failure evidence, not a failed solver assertion. No HDA publication or
installation.
## PF-7: upstream mesh readers and FEM input formats (2026-11-17)

Merge target `188bd09fc805a04cad687c65c280d9c46b191e57` from PF-6
`635ea6cf22a9f0297bab1218c70179491997f68c`. The three conflict resolutions
retain MISO OFF and upstream opt-in OpenMP in `CMakeLists.txt`, preserve the
fork PolySolve pin and Eigen handling in `cmake/recipes/polysolve.cmake`, and
keep the fork's rest-element and surface validation declarations with
upstream matrix-based `is_planar` in `MeshUtils.hpp`. The pinned PolySolve
`f175cabe` contains upstream `621e8fb3`. The published PF-6 data pin
`ff5d77a7d216298cff19e5da4b61aacae2d6df12` carries forward.

The owner approved alignment with upstream FEM dispatch. The Geogram fallback
and the `State::set_mesh(GEO::Mesh &)` overload were present in the previous
upstream revision `2b302cb749`. PF-7 accepts `.msh`, `.hybrid` and `.obj` and
retires that shared fallback, including `.off`,
`.mesh`, `.meshb`, `.ply` and `.stl`. CLI controls show that PF-6 solved all
five retired inputs and PF-7 refuses each by extension. The OBJ control
produced the same VTU bytes. Obstacle and surface inputs use the separate
upstream surface reader. PF-6's remeshing decision carries forward as MMG
shape-optimization restarts.

The smoke tetrahedral and hexahedral assets use MSH 2.2 with exactly the
same decoded coordinates and element corner order. Their Medit originals
remain available as historical fixtures. Dependent scenes, regression
runners and test-generated inputs use the migrated meshes. The RB-11
out-of-range connectivity probe uses Gmsh 2.2 and checks the named
missing-node diagnostic. The duplicate-rest-element test exercises the
actual MSH loader. The existing HDF5 polygon checkpoint test uses the
upstream geometry implementation.

Verification on the isolated macOS arm64 build with MMG, optimization and
Python enabled and MISO and OpenMP disabled:

- **V1:** `PolyFEM_bin` and `unit_tests` built with exit 0. The pinned
  clang-format 21.1.8 check passed for all 50 staged C-family files.
- **V2:** 107 focused cases and 8,946 assertions passed, including MSH,
  mesh geometry, MMG utilities and checkpoint/restart. Both selected
  RB-11 cases passed. All 82 malformed-MSH outcomes, named diagnostics and
  output file sets matched the retained corpus reference: 10 completions
  and 72 named refusals. Ten MMG workflow cases passed both restarts with
  zero coordinate difference between the emitted and reloaded geometry.
- **V3:** Five single-threaded public smokes produced the same solver
  histories and all 25 VTUs matched the retained PF-6 outputs byte for
  byte. The independent Medit-to-MSH hex smoke comparison matched all
  five VTUs.
- **V4:** RB-02 passed 270 checks. All 24 RB-05 probe rows matched PF-6,
  and both resource-limit scenes passed.
- **V5:** The full default native suite passed 503 cases and 5,184,202
  assertions on seed 1 in 20 minutes 22 seconds. Both the test process and
  its bounded supervisor recorded exit 0.
- **V6:** The isolated `test_msh_end_to_end.py`, `test_msh_parser.py` and
  `test_msh_reader_topology.py` scripts each exited 0. Installed asset
  digests matched their pre-run values after every script.

Evidence is in sibling `PF-7/`, including the source snapshot
`upstream-format-build-source.patch`, `fixture-migration-audit.json`,
`build-upstream-formats/receipt.json`, and the per-gate logs, receipts and
independent comparisons under `final-validation/`. The frozen executables,
their digests and the generated build identity are under
`bin/PF-7-upstream-format-validation/`.
PF-7 publication requires owner authorization.

## PF-8: static exact problems and full regression (2026-11-17)

PF-7 was published with owner approval. The verified remote source ref is
`8092c35d87aa160b8baf0149678dcf53c79bfddf`. The remote and local `main`
refs and all 31 installed Houdini asset digests matched their pre-push
values. Evidence is in `PF-7/publication-20261117.json`.

Merge upstream `f974f45ed12acbf0e824a3fa480991d9ec7a7f59` into that base.
Resolve the data recipe to the fork's merged fixture commit
`386006a6faee31b1a82e8ae145349bc229a8664e` (upstream data `ab3246ce`).
The 44 upstream data changes leave the nine fork-specific fixture paths
intact. In `State.cpp`, insert upstream's time-dependence check before
the existing run-manifest registration. In `test_problem.cpp`, add the
upstream static exact-solution case beside the fork's problem tests.
The unchanged fork policies include checkpoint continuation, the existing
FEM formats and MMG shape-optimization remeshing.

Adopt upstream's time-independent `Franke`, `FrankeOld`, and `ElasticExact`
behavior and the corresponding regression references. The build follows the
recorded production configuration with the accepted IPC-1 source and
isolated auxiliary fixture paths. Data pin checks include upstream's updated
pref and polyspline revisions. MMG, optimization, Python, and TBB remain
enabled. MISO and OpenMP remain off.

- **V1:** Both production targets built with exit 0. The 18 staged C/C++
  files passed the pinned formatter.
- **V2:** 142 focused cases and 10,198 assertions passed. The 82-case MSH
  corpus matches its original outcomes and diagnostics. Both targeted RB-11
  cases passed. All ten MMG workflow cases passed two restart comparisons
  each with zero coordinate difference.
- **V3:** The five one-thread public smokes match PF-7's solver histories.
  All 25 VTUs are byte-identical.
- **V4:** All 270 RB-02 checks passed. The 24 RB-05 probe rows and both
  resource-limit scenes passed against PF-7.
- **RB-12:** All 30 runs completed. Reanalysis of the retained VTUs extracted
  30 solutions and compared 75 cross-thread pairs. Within each thread
  setting each fixture had one solver history, with maximum displacement
  difference `3.08e-15` and zero numerical violations.
  Fifty cross-thread pairs had the same history and agreed to within
  `2.93e-15`. The other 25 pairs, from the quasistatic-semi fixture, had
  different histories at step 3 (six default-thread iterations and nine
  serial iterations) and a maximum solution difference of `3.68e-16`.
  The initial report contained zero displacement comparisons because the
  isolated runner missed its decoder. The reanalysis checksum matches the
  frozen binary, and the original solver outputs are retained.
- **V5:** The full native suite passed 509 cases and 5,184,365 assertions
  on seed 1 with exit 0. The same process resumed on AC with the lid open
  after its recorded power pause. The complete Catch2 log and supervisor
  receipt are in `PF-8/native-suite/`.
- **V6:** All 31 isolated Houdini scripts passed with the same frozen solver
  and HDA definitions. The first 20 passes were retained. The next reader test
  stopped at an absent staged VTU. The continuation copied the verified
  PF-8 quasistatic smoke to the reader's expected layout and passed all 11
  remaining tests. Installed asset digests matched their pre-run values
  throughout. See `hda-complete/receipts.json` and
  `hda-resume-run/receipt.json`.

See `PF-8/production-build-source.patch`, `production-build-pins.json`,
`validation/`, `validation-run/`, and `repeat-analysis-receipt.json`.
The frozen executables and SHA-256 digests are under
`bin/PF-8-production-validation/`. PF-8 publication requires owner
authorization for the data ref followed by its consuming source ref.

## IPC-2: compact candidates and PolyFEM stencil adapters (2026-11-17)

PF-8 publication was verified with data
`386006a6faee31b1a82e8ae145349bc229a8664e` published before source
`6ea60259a33c8c3275fb4a003073db445de1f3c8`. Both remote integration refs,
both repositories' main refs, and the installed Houdini asset digests
were read back. See `PF-8/publication-20261117.json`.

Toolkit integration commit `8854a705370d3cdd933748268955ca611f0c9491`
merges upstream `c7ada28baa1ed875ad361f9404c2f7a0ee950efb` into IPC-1.
The two conflict resolutions retain checked broad-phase accounting and the
BruteForce pre-allocation budget with upstream's candidate storage.
The existing budget tests were ported to the new container and typed visitor.
The toolkit native suite passed 453 cases and 5,802,571 assertions with
exit 0. The frozen Python binding passed all four independent fork flag
combinations and candidate-to-stencil numerical parity. The toolkit's
`docs/upstream-integration-20261117-ipc2.md` records its source audit.

PolyFEM pins the accepted local toolkit revision and uses upstream's
`VertexVertexStencil`, `EdgeVertexStencil`, `EdgeEdgeStencil`, and
`FaceVertexStencil` for the four parent coefficient estimates.
These preserve AUTO classification and existing canonical parent IDs.

- **V1:** Both isolated production targets built with exit 0. The
  consumer's pinned formatting check passed.
- **V2:** 63 focused cases and 5,630 assertions passed, including the
  contact and checkpoint/restart gates.
- **V3:** All five single-threaded solver histories and all 25 VTUs
  matched PF-8 exactly.
- **V4:** RB-02 passed 270 checks. RB-05 matched PF-8 in all 24 probe
  outcomes and in both resource-limit scenes. All 17 candidate reports
  kept the same per-type counts. Reported storage changed from 16 to
  8 bytes per candidate on this build as expected from the upstream
  representation. The first exact comparison flagged that field.
  The independent verification checks the exact size change as well as
  the preserved estimates, measured counts, statistics, and refusals.
- **V5:** The full native PolyFEM suite passed 509 cases and 5,184,365
  assertions on seed 1 in 20 minutes 23 seconds with exit 0.

Evidence is under `IPC-2/`, including `consumer-build-source.patch`,
`consumer-validation/`, `rb05-compact-layout-verification.json`, and
`consumer-suite/`. The executable digests are in
`bin/IPC-2-polyfem-validation/`. IPC-2 publication requires owner approval
for the toolkit ref followed by the consuming PolyFEM ref.

## IPC-3: per-primitive thickness and zero-thickness parity (2026-11-17)

IPC-2 publication was verified with toolkit
`8854a705370d3cdd933748268955ca611f0c9491` published before PolyFEM
`b76ecd237ac95cdd569a797673376f40eaff47e5`. Both `main` refs and all
31 installed Houdini digests matched their pre-push values.
See `IPC-2/publication-20261117.json`.

Toolkit commit `50041d99f90dea65a1f37f4a492c4380fcb54c44` merges upstream
`54d58ebcf1f6dd0cf95bcc58afd79839ca603a99` into IPC-2. The candidate
resolution keeps the fork's budget checks, cleanup and aggregate statistics
alongside upstream thickness-aware static and swept broad-phase builds.
The normal builder keeps signed parent construction alongside upstream
thickness helpers. The consumer pins that toolkit revision and continues
passing its existing scalar minimum distance.

- **Direct parity:** Both frozen toolkit bindings match all 60 collision
  records exactly across absent and explicit-zero thickness, including
  vertex-vertex, edge-vertex, edge-edge, face-vertex and plane-vertex
  stencils. Omitted, explicit-zero and positive legacy scalar minima
  and the corresponding collision weights agree.
- **Toolkit:** 462 cases and 5,808,918 assertions passed with exit 0.
  The Python smoke passed the two fork collision flags and candidate
  and stencil parity. See `minimum-distance-parity.json` and
  `toolkit-tests-run/` under `IPC-3/`.
- **V1:** Both isolated PolyFEM production targets built with exit 0.
- **V2:** 43 focused cases and 1,574 assertions passed, including
  contact and checkpoint/restart.
- **V3:** All five single-threaded solver histories and all 25 VTUs
  matched the frozen IPC-2 baseline exactly.
- **V4:** RB-02 passed 270 checks. All 24 RB-05 rows and both
  resource-limit scenes matched the IPC-2 outcomes and accounting.
  An exact raw-report comparison found unordered scale samples and
  one-ULP energy differences. Eight replays per frozen probe reproduced
  the same energy pairs on both versions. With the scale multisets
  compared, every other report value matched. See
  `rb02-repeat-evidence/verification.json`.
- **V5:** The full native PolyFEM suite passed 509 cases and 5,184,365
  assertions on seed 1. Both the suite and its outer supervisor recorded
  exit 0.

Evidence is under `IPC-3/` in `consumer-build-source.patch`,
`consumer-validation/`, and `consumer-validation-run/`. The executable
digests are in `bin/IPC-3-polyfem-validation/`. Publication requires
owner approval for the toolkit ref followed by PolyFEM.

IPC-3 was published with owner approval on November 18. GitHub verified
toolkit `50041d99f90dea65a1f37f4a492c4380fcb54c44` before the PolyFEM
consumer `c7adfffd8207be6b467d3c7c9449a1b3b00bdcf4` on their
`upstream-merge` branches. Both remote main refs, the four local main
refs and all 31 installed Houdini asset hashes matched their preflight
values. See `IPC-3/publication-20261118.json`.

## IPC-4 — HashGrid cell traversal (2026-11-18)

Pin toolkit `4f99f337357b366daf0984ea9c775273403e7e46` with upstream
`282d4830f0f2a48f741e46ecfc8bfb72ec2b50d5`. The toolkit keeps its checked
budget immediately before each upstream `detect_cell_pairs` call. Each
common key is queued once, and the parallel tiles partition its index
pairs. Two lists emit at most `n0 * n1` pairs per key. One list emits at
most `n * (n - 1) / 2` because only pairs with the second index greater
than the first are visited. The filter, AABB test and deduplication can
only reduce the emissions. The cross-list budget counts checked products
directly from the original sorted items. The same-list checked sum and
pre-insertion cell-item budget remain in place.

The isolated macOS arm64 toolkit build, pinned formatter and Python
binding smoke passed. The focused run passed 20
cases. The full default toolkit suite passed 466
cases and 5,811,140 assertions. Six private CCD
fixtures were unavailable in both IPC-3 and IPC-4.

- **V1:** The isolated PolyFEM build recompiled HashGrid and linked both
  `PolyFEM_bin` and `unit_tests` with exit 0.
- **V2:** 50 focused cases and 3,569
  assertions passed, including MSH, MMG and HDF5 checkpoint/restart.
- **V3:** All five single-threaded solver histories and 25 VTU files
  matched IPC-3 exactly.
- **V4:** RB-02 passed all 270 checks. The 24
  RB-05 probes matched IPC-3 exactly for exits, estimates, measured
  counts, candidates, statistics and exceptions. Both resource-limit
  scenes passed.
- **V5:** The full default PolyFEM suite passed 509 cases and
  5,184,365 assertions with seed 1 in 20 minutes
  24 seconds. The suite and supervisor receipts both record exit 0.

The consumer code change is the toolkit pin. See sibling `IPC-4/` for
the captured build source, toolkit and consumer reports, gate
comparisons and receipts. The frozen native executables are under
`bin/IPC-4-toolkit-validation/` and `bin/IPC-4-polyfem-validation/`.
Publication requires owner authorization.
