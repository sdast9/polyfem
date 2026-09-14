# RB-12 — Repeatability, provenance and release discipline

Date: 2026-09-14
Status: in progress — stage 1 (run identity) implemented—validation pending
publication; stage 2 (controlled repeatability) and stage 3 (CI/release
integration) not started
Selected stage: stage 1, the run manifest, on the plan's own scope; no
dependency upgrade, upstream merge, release tag or feature promotion.

## Contract and authorization

- User-selected item: RB-12 ("let's continue with RB-12", 2026-09-14). The
  plan's three stages are taken in order: run identity, controlled
  repeatability, CI/release integration. Nothing else is authorized by this
  selection; the four CI-01–CI-06-class repairs of the CI plan are separate
  bounded items and are only touched where they block the acceptance of this
  stage (see "CI baseline" below).
- Invariant and success criterion (stage 1): every `PolyFEM_bin` run leaves a
  manifest that identifies the effective PolyFEM / IPC Toolkit / PolySolve
  sources (commit, dirty state or patch hash), the executable's own hash,
  the build, compiler, linear-solver and thread settings, the producer's
  asset when the input carries one, the effective expanded input and every
  referenced file with their hashes, the platform, the actual step/attempt
  history, the completion status and the diagnostics schemas — with source
  identity and binary identity recorded separately and the declared
  dependency pins recorded next to the effective sources, never inferred
  from the recipe alone. The revised integration scope's items (estimator /
  controller version, reference state and coefficient-update sequence, gap
  convention, uncertainty method, model-selection status, fallbacks used)
  are the contact form's `model` record plus the per-step contact state.
- Dependencies read: the plan (`robustness-plan.md` at `73227670c`), the PF
  invariants, the CI portability plan and its evidence
  (`ci-portability-plan.md`, `ci-portability-evidence-2026-09-13.md`), the
  RB-04 contract (record version 3 and the three streams), the RB-09/RB-11
  handoff notes addressed to RB-12 (unit system and
  `characteristic_force_density` belong in the manifest — done). No
  prerequisite is incomplete for stage 1.
- Exclusions: no production coefficient, friction, material, tolerance,
  resource-limit or retry default changed (the manifest is observational);
  no CI workflow edited yet (stage 3); the legacy `polyfem::legacy::State`
  path (Stokes and the other non-VarForm formulations) writes no manifest —
  the fork's contact work runs through the VarForm state; a run refused
  inside `State::init` itself (invalid JSON, `dhat`, broad-phase name, time
  schedule) stops before the effective input is final and has no manifest —
  its record is the log's `PolyFEM stopped:` line and the exit status; every
  later refusal (mesh, materials, collision surface, solve) has one; no HDA change yet
  (the `provenance` input block is specified and consumed, the asset fills
  it in a later publication).

## Baseline and reproduction

- PolyFEM `a23ac34d9` on `main` at session start (clean); a concurrent
  session committed the documentation-only `73227670c` (RB-11 follow-up
  review) at 09:44 local while this stage was being built — the working tree
  is shared, files are added by name. Evidence directory
  `outputs/rb-12/20260914T133427Z-identity/` (`baseline-identity.txt`).
- Effective IPC Toolkit source: the local override
  `ipc-toolkit-fork` at `bb795446` (`semi-implicit-stiffness`, clean) =
  the recipe pin; effective PolySolve: the local override `polysolve-merged`
  at `ee5b296a` (`iteration-callback`, clean) = the recipe pin; both via
  `CPM_<name>_SOURCE` (`CMakeCache.txt`), which the manifest now records as
  `build.sources.*.source_override` with `matches_declared_pin`.
- Build: AppleClang 21.0.0.21000101, RelWithDebInfo, Unix Makefiles, TBB,
  macOS 26.5.2 arm64, CMake 4.4.3, git 2.55.0.
- Binary at session start `a134f6c8…` (the RB-11 candidate B, 2026-09-13
  23:14); the stage-1 binary is recorded in `binary-identity-stage1.txt`
  and inside every evidence manifest (`process.executable.sha256`).
- Reproduced starting point: no run identity existed. The output JSON
  (`output/json`, off by default) carries the effective `args`, thread count
  and peak RSS only at a successful end; the RB-04 streams carry a run id
  that no other artefact shared; nothing recorded the sources, the binary,
  the input files' hashes, the platform or a failed run's identity. No
  version plumbing existed in the code base (`rg git_sha|GIT_COMMIT` empty).
- CI baseline at session start (read-only snapshot, `ci-baseline/`): Build
  run 34803837989 and pre-commit run 34803838027 at `a23ac34d9` both fail —
  Linux (both configurations) at `tests/test_linear_elastic_time.cpp:52,62`
  with `-Werror=missing-braces`; macOS Debug with 8 aborted RB-11 tests
  (Eigen resize / `Mesh.cpp:207` / `MatParams.cpp:534` assertions);
  Windows Debug 9 failures (4 `linear_elastic` "Failed", 5 `input_validation`
  fail-fast exits `0xc0000409`); Windows Release 1 failure (`elastic laws:
  finite-difference derivatives`); macOS Release the four known CI-03–CI-06
  scene groups; pre-commit: clang-format drift in `MatrixUtils.hpp`,
  `test_input_validation.cpp`, `test_linear_elastic_time.cpp`,
  `test_material_envelope.cpp`. Classification: the Linux compile error, the
  Debug/Windows assertion failures and the formatting drift are new
  regressions of the RB-11 stages (validated locally on RelWithDebInfo /
  AppleClang only); the four macOS Release groups are the known baseline
  failures. `ci-baseline/summary.md` and the seven job logs are retained.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| No run identity: sources, binary, input files, platform and a failed run's fate were unrecorded; the RB-04 run id was private to its streams | code inspection (`main.cpp`, `OutData.cpp::save_json`, `NonlinearElasticVarForm.cpp`), `rg` for version plumbing | fixed — `run-manifest.json` (below) |
| Source identity must be measured at build time and kept apart from the declared recipe pins | the developer build takes both dependencies from local overrides; the recipe pins happen to match today | fixed — `polyfem_build_info` target regenerates `BuildInfo.cpp` before every build (git commit / branch / dirty / patch SHA-256 of tracked diff + untracked file hashes, capped at 1000 files; `source_override`, `declared_pin`, `matches_declared_pin`); unchanged identity does not relink (`copy_if_different`; a no-op build runs only the refresh) |
| Binary identity | `process.executable.sha256` = the executable's own SHA-256 (`_NSGetExecutablePath` / `/proc/self/exe` / `GetModuleFileName`), size, mtime | fixed — verified equal to `shasum` of the binary in every evidence run |
| Effective input and referenced files | `input.effective` (= `State::args`, the same object `output.json` writes), `effective_sha256` (canonical sorted-key serialization), `referenced_files` = every string outside `/output` and `/root_path` that resolves — as the solver resolves it — to an existing regular file, with pointer, path, size, SHA-256; `input.file` and `common_chain` hashed | fixed — `[run_manifest][referenced_files]` pins the walk (mesh, per-element value file, `discr_order` file listed; an absent mesh, an output name that exists in the directory and a multi-line string are not) |
| Unit system and `characteristic_force_density` (RB-09/RB-11 handoff) | `input.units`, `input.characteristic_force_density.{setting, effective}` | fixed |
| Model description (revised scope) | `solver.model` from `BarrierContactForm::model_description()`: stiffness mode, coefficient-law version and lineage (RB-18/RB-20/RB-21), curvature and interpolated-stencil fallbacks, coefficient identity, continuation, friction lag, controller constants, reference-state statement, gap convention, model-selection status, uncertainty method (`null`, reason given); per step the form's `diagnostic_state()` subset (trim, refresh id, batch median / floor / cap, fallback and continuation counts, candidate counts) | fixed — `[run_manifest][model]` |
| Step/attempt history for every outcome | `steps[]` written at the RB-04 recording boundary of `solve_tensor_nonlinear`, diagnostics enabled or not: outcome, phase, wall time, termination (restarts, iterations, reason), every subsolve of the step from `stats.solver_info`, stall retunes, lagging state, error | fixed — a mid-solve resource failure leaves `steps[0].outcome = failed_attempt` with the exception (`failure-run-2/`); a failure before the first solve leaves no step (`failure-run/`) |
| Completion for every process exit | `main` finalizes `completed` (0) on success; the top-level handler finalizes `failed` (1) / `resource_failure` (3) with the message before logging; the registry holds the manifest strongly because the `State` is gone by then | fixed — `failure-run/` exit 3 → `resource_failure`, message, `steps_recorded: 0` |
| Diagnostics schema identity | `io::schemas` constants shared by the three RB-04 writers and the manifest (`diagnostics.schemas`); the streams adopt the manifest's `run_id` | fixed — `diag-run/`: one id across `run-manifest.json` and the three `.jsonl` streams; `output.json` `args` equals `input.effective` |
| Producer provenance | new spec block `/provenance` (producer, producer_version, asset, asset_version, asset_sha256, scene, exported_at), copied to `producer` when any field is filled (the spec injects an all-empty object otherwise) | fixed — `failure-run/`, `[run_manifest][switch]`; the Houdini asset does not fill it yet |
| Opt-in for the library, default for the executable | `output/manifest` (spec default `""`); `State::default_manifest` = `run-manifest.json` set by `PolyFEM_bin`, applied after the `common` chain so a common file's explicit `""` still wins | fixed — `[run_manifest][switch]` |
| Formatting drift and a GCC aggregate-initializer error left by RB-11 block the CI acceptance of this stage | `ci-baseline/` (pre-commit 103851665920; Linux 103851686556) | fixed here as the mechanical CI-01/CI-02-class repair: nested braces at `test_linear_elastic_time.cpp:52,62`, the pinned clang-format 21.1.8 applied to the four drifted files; no behaviour change. The Debug/Windows assertion failures of the RB-11 tests are **not** touched: they are RB-11's follow-up (a concurrent session holds it) and are recorded for stage 3 |
| Threaded `quasistatic-semi-friction` differs from the Stage 1 fingerprint by 1.7e-4 while the serial run is bit-identical | `smokes/summary.json`; the two manifests' step 4: threaded 13 iterations ("Gradient vector norm too small"), trim 8; serial 12 iterations ("Relative gradient vector too small"), trim 16 | reproduced — an evaluation-order sensitivity amplified by a discrete controller decision (trim band step); stage 2 characterizes it; not attributed to threading as a cause without the isolated comparison |

Design notes. The manifest is written when `State::init` finishes (before
the mesh is read, so a later failure still leaves the identity), rewritten
atomically (temporary file + rename) after the model is known, after every
step and at completion; a write failure is logged and never stops the solve.
No environment variable is recorded (secrets); absolute paths are recorded
(the output lives next to the user's other outputs; a public CI artefact
policy is stage 3's). SHA-256 is a self-contained implementation
(`utils/Sha256`) with FIPS known-answer tests, so the identity never depends
on an optional system library. Nothing the manifest reads changes the solve:
the five smokes are within roundoff of the RB-11 Stage 1 fingerprints
(single-threaded friction bit-identical), see Validation.

Changed settings: none of the production defaults. New input keys:
`output/manifest` (string, default `""`; `PolyFEM_bin` defaults it to
`run-manifest.json`), `/provenance` (object, all fields optional strings).
New output: `run-manifest.json` in the output directory of every executable
run.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail |
| --- | --- | --- | --- | --- |
| New regression | `unit_tests "[run_manifest]"` (7 cases: SHA-256 known answers and streaming, build identity, canonical hash, referenced files, model description, a 2-step NeoHookean beam run, the opt-in/default switches) | all pass | 156 assertions / 7 cases | pass (`unit/run_manifest.log`) |
| Affected baseline selection | `[contact_floor_retired],[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives` | pass | 1,573 assertions / 24 cases | pass (`unit/affected.log`) |
| Smokes vs RB-11 Stage 1 fingerprints | the five public scenes, default threads, plus `quasistatic-semi-friction --max_threads 1` | all 4/4 steps, exit 0; last-step solution within roundoff | adaptive 1.5e-16, semi 1.9e-16, alhess 1.7e-16, transient 2.4e-16, friction single-threaded 0.0 (bit-identical); friction threaded 1.7e-4 (see finding) | pass except the threaded friction observation (`smokes/summary.json`) |
| Manifest per smoke | same runs | `completed`, exit 0, `steps_recorded` = 4 = saved steps, `executable.sha256` = `shasum` of the binary, model `semi_implicit`/`adaptive` | all six | pass |
| Failure paths | `quasistatic-semi` with `max_candidate_emissions` 10 (fails in `init`) and 3000 (fails in the first line search) | exit 3, `resource_failure`, message; 0 / 1 step, the latter `failed_attempt` with the exception | as expected | pass (`failure-run/`, `failure-run-2/`) |
| Shared run id and `output.json` agreement | `quasistatic-semi` with `physical_diagnostics` and `output/json`, single thread | one `run_id` across the manifest and the three streams; `output.json.args == input.effective` | both hold | pass (`diag-run/`) |
| Build hygiene | `polyfem_build_info` custom target | a no-op build refreshes the identity and relinks nothing | only "Refreshing the PolyFEM build identity" runs (`build-2.log` … ) | pass |
| Formatting / diff / links | pinned clang-format 21.1.8 dry run over every tracked C-family file plus the new files; `git diff --check` | clean | 0 violations; clean | pass |
| Native GCC / MSVC compilation of the new code | GitHub matrix | compiles and `[run_manifest]` passes on the three platforms | **not run yet** — triggered by the publication; the outcome belongs to stage 3 | not run |
| HDA end-to-end | `houdini_HDAs/tests/test_polyfem_hda.py` with the new binary | the pipeline is unaffected by the extra output file (the tests filter by extension) | **not run** in this stage (no HDA change; the manifest run of the asset is the later provenance publication) | not run |

- Numerical termination versus measured residual: unchanged by this stage;
  the manifest copies the solver's own termination record.
- Missing quantities: `uncertainty_method` is `null` with the reason (no
  estimator in production); the legacy state path has no manifest; Windows
  `platform.release/version` are not queried (`unavailable_reason`).
- Retained failed/partial runs: `failure-run/`, `failure-run-2/` (deliberate
  resource failures), the threaded friction smoke (a run that completed
  with a different endpoint than the fingerprint, kept as a stage-2 input).
- Test tolerances: `[run_manifest]` compares exact hashes and JSON equality;
  no numerical tolerance was introduced or changed.
- Not performed: the full suite, private scenes, Teseo, the HDA tests, native
  Linux/Windows builds (GitHub).

## Publication and reproducibility

- Rebuilt targets: `PolyFEM_bin`, `unit_tests` (RelWithDebInfo, AppleClang
  21, TBB); tested source identity: the working tree of `73227670c` plus
  this stage's changes (the manifests embed the patch hash of that tree).
- Committed files: see the publication commit recorded below.
- Companion pins: unchanged (IPC `bb795446`, PolySolve `ee5b296a`).
- Remaining working-tree changes: none of this stage's after publication;
  the concurrent RB-11 session's files are theirs.
- Evidence (local, not committed): `outputs/rb-12/20260914T133427Z-identity/`.

## Next session handoff

- Completed: stage 1 implementation and local validation. Pending: the
  GitHub matrix outcome for the new code (stage 3 reads it), stage 2
  (five repeats of a small public fixture and of the PF-08 fine-load case
  `quasistatic-semi` at `dt = .0625`, then the declared serial/threaded
  comparison — convergence, restarts, reactions, energies, candidate counts,
  peak memory), stage 3 (bounded CI checks, workflow verification, the
  defaults table, README claims), the Houdini `provenance` block.
- Status: `in progress` — stage 1 validated locally; the item's acceptance
  (manifest identifies tested artifacts; repeat matrix and tolerances saved;
  CI run and linked; README claims match) is not yet met.
- Decision required: none for stage 1. For stage 3 the user will need to
  choose the CI scope (which lanes are required, what to do with the RB-11
  Debug/Windows failures and the four known scene groups).
- Next command: `python3 tools/rb12/repeat.py …` (stage 2 runner, to be
  written) on `quasistatic-semi` and the `dt = .0625` variant.
