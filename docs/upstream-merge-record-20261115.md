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
