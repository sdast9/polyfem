# RB-12 — Repeatability, provenance and release discipline

Date: 2026-09-14 (closed 2026-09-15)
Status: **closed — validated within stated scope** (2026-09-15, [closure](#closure-2026-09-15)) — stage 1 (run identity)
published `1f6f826fa`; stage 2 (controlled repeatability)
characterized—limits documented; stage 3 (CI/release integration)
published `f5f59db26`, `4f5acc773`, `fffc722b9`: the required lanes
(pre-commit, Linux Release, macOS Release) are at their known baseline —
green apart from the four CI-03–CI-06 scene groups, which stay tracked
exceptions — the dependency forks' maintained branches are under CI, and
the Houdini asset carries provenance. No dependency upgrade, upstream
merge, release tag or feature promotion was made.
Selected stage: all three, in order, on the plan's own scope.

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
  later refusal (mesh, materials, collision surface, solve) has one. The
  Houdini asset fills the `provenance` block since its publication
  `6f5fcdc` (2026-09-14, see stage 3).

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

## Stage 2 — controlled repeatability (2026-09-14)

**Status: characterized—limits documented.** Runner `tools/rb12/repeat.py`
(see its README); evidence `outputs/rb-12/20260914T141159Z-repeat/`
(matrix A, binary `2c4658eb…`, re-analysed after the comparison rule was
refined) and `outputs/rb-12/20260914T142106Z-repeat/` (matrix B, the
stage-2 binary `4595aa53…`, which only adds `effective_sha256_without_paths`
to the manifest). Fixtures: `quasistatic-semi` (the small public fixture,
dt .25, 4 steps), `quasistatic-semi-friction` (the same with friction,
because stage 1 had already seen it diverge), and `quasistatic-semi` at
`dt = .0625` (16 steps: the PF-08 fine-load case whose first run exhausted
20 restarts and whose repeat passed). Five repeats each at the default
thread count (18 on this Mac) and at `--max_threads 1`, every run from its
own directory with `output/json` and `physical_diagnostics` on. Every
manifest names the same executable hash and the same path-independent
input hash within a cell; peak RSS is the manifest's `completion.peak_rss_mb`.

Declared comparison rule (both matrices, 0 violations): two repeats with an
identical *solver path* — the manifest's per-step subsolve iteration counts,
termination reasons, restarts, stall retunes, lagging state, trim and
refresh id — must agree on the last saved solution to `‖du‖∞ ≤ 1e-12`
(internal length) and on every endpoint scalar to 1e-12 relative to the
run-level magnitude of that quantity (energies and work increments over the
largest step value, the residual norm over the peak external force, the FE
body's summed support reaction over its largest component). Repeats whose
paths differ are *branch divergences*, reported with the first differing
step and their endpoint difference, never averaged. The roundoff-sensitive
contact state (active collisions, continued / fresh coefficients, candidate
counts) is reported as a per-step spread, not as part of the path.

| Cell (5 repeats) | Completed | Distinct paths | Same-path max ‖du‖ | Branch divergence | Peak RSS MB | Wall s |
| --- | --- | --- | --- | --- | --- | --- |
| `quasistatic-semi`, 18 threads | 5/5 (A) 5/5 (B) | 1 / 1 | 2.1e-16 / 2.0e-16 | none | 2079–2474 / 2330–2460 | 4.4–5.3 |
| `quasistatic-semi`, 1 thread | 5/5 / 5/5 | 1 / 1 | **0** (bit-identical) | none | 222–226 | 5.3–6.2 |
| `quasistatic-semi-friction`, 18 threads | 5/5 / 5/5 | **3** / **2** | 3.5e-15 / 2.3e-15 | at step 4: A 1.7e-4 (one run: 13 reduced iterations ending "Gradient vector norm too small", trim stays 8) and 3.9e-5 (two clusters at trim 16, 12 iterations); B 3.9e-5 (iteration totals 16–20, trim 16) | 2066–2417 | 4.7–5.4 |
| `quasistatic-semi-friction`, 1 thread | 5/5 / 5/5 | 1 / 1 | **0** | none | 222 | 5.5–5.8 |
| `quasistatic-semi` dt .0625, 18 threads | 5/5 / 5/5 | 2 / 2 | 6.5e-16 / 6.9e-16 | at step 11 (6 vs 7 iterations) with ‖du‖ ≤ 6.9e-16: a path difference at the roundoff floor, no endpoint effect | 2210–2546 | 5.4–5.6 |
| `quasistatic-semi` dt .0625, 1 thread | 5/5 / 5/5 | 1 / 1 | **0** | none | 212 | 5.7–5.9 |

Serial versus threaded (25 pairs per fixture): `quasistatic-semi` 2.9e-16
(no pair shares a path — the serial run's step 3 takes 11 reduced
iterations where the threaded runs take 6, both ending "Gradient vector
norm too small", i.e. iterations at the roundoff floor, RB-19's regime);
`quasistatic-semi-friction` 2.3e-15 for the 10 (A) / 20 (B) pairs that
share the serial path, 1.7e-4 (A) / 3.9e-5 (B) otherwise;
`quasistatic-semi` dt .0625 5.6e-16.

Findings.

- Single-threaded runs are bit-reproducible on this platform for all three
  fixtures (15/15 runs per matrix identical to the last bit, including the
  16-step fine-load case), with identical solver paths and contact states.
- Threaded frictionless runs reach the same endpoint to roundoff
  (≤ 6.9e-16) even when their paths differ by an iteration at the
  tolerance floor and their active-collision counts differ by ±2 (contacts
  within roundoff of the `d̂` band edge, gaps clustered near `.9–.99 d̂` as
  RB-09 measured): the frictionless endpoint is a well-posed minimiser.
- Threaded friction runs are reproducible only to the contact-model scale:
  the endpoint spread over 10 threaded runs is up to 1.7e-4 (`d̂ = 1e-3`;
  7e-4 of the .25 prescribed displacement; support reaction 7.9e-4
  relative; frictional dissipation increment .5 %). The cause chain is
  visible in the manifests, not inferred: evaluation-order roundoff moves
  the last reduced solve of step 4 across its stopping criterion (12
  iterations "Relative gradient vector too small" versus 13 "Gradient
  vector norm too small"), the global trim controller then takes a
  different decision (16 versus 8), and because the friction lag is not
  converged after the default 2 lagging iterations (RB-10: every friction
  step reads `physical_balance_pass` false, residual ratio 3.3e-3) the
  endpoint depends on that path. Threading is the source of the roundoff
  noise (the serial runs are exact), but the amplifier is the pair
  "discrete controller decision + unconverged lag", which stage 2 records
  and does not change (RB-10's defaults were a user decision; RB-16 closed
  the controller question). The spread stays below the RB-09 realized-gap
  model error (`c·ḡ ≈ d̂`), so it does not change any accuracy claim, but a
  threaded friction run cannot serve as a golden.
- The PF-08 first-run failure (exhausted restarts at dt .0625) did not
  reproduce: 10/10 runs completed 16/16 steps with 0 restarts and 0
  retunes; the RB-19 fallback and the RB-18 stall repairs have been in
  production since. The original failure stays recorded in the PF-08 record;
  it is not labelled a concurrency effect.
- `physical_balance_pass` is true on every frictionless dt .25 step (max
  residual ratio 4.9e-11) and false on the dt .0625 run's **step 1**
  (ratio 1.1): the cube has not touched the slab yet, the peak external
  force is ~1e-9 and the RB-09 normalisation is degenerate before first
  contact. Observation for RB-09's record; nothing gated on it.
- Peak RSS grows linearly with the thread count: 222 / 327 / 613 / 1081 /
  2392 MB at 1 / 2 / 4 / 8 / 18 threads on the 384-tetrahedron smoke
  (`rss-probe/` in matrix A), about 125 MB per thread independent of the
  mesh, while the wall time only falls from 1.9 s to 1.0 s. A second probe
  (`rss-probe-2/` in matrix B) locates it: with contact disabled the run
  takes 37 MB at 1 thread and 43 MB at 18; with contact on, 222 MB at 1
  thread and 2.3–2.7 GB at 18 whatever the stiffness law (classic adaptive
  or semi-implicit) and whatever the broad phase (hash grid, BVH, brute
  force). So the contact path carries a ~185 MB fixed cost plus ~125 MB per
  thread on a 387-DOF system — not the coefficient machinery, not the broad
  phase; the per-thread storage of the contact potential/Hessian assembly
  (PolyFEM's or the toolkit's) is the first place to instrument. An
  RB-05-adjacent resource item for the user; recorded in the manifest of
  every threaded run.

Limits: one platform (macOS 26.5 arm64, AppleClang 21, TBB), one build
configuration, the public fixtures only, five repeats per cell (the 1.7e-4
branch appeared once in ten threaded friction runs — the sampling is
small); no cross-platform repeat comparison (the GitHub runners run the
CLI contract, not this matrix).

## Stage 3 — CI and release integration (2026-09-14)

**Status: published (`f5f59db26`, `4f5acc773`, `fffc722b9`); closed
2026-09-15 (see [Closure](#closure-2026-09-15)).** Bounded additions, no
workflow lane removed, no required check turned into a warning:

- `PolyFEM_bin --build_info` prints the compiled-in identity as JSON.
- CTest `cli_contract` (`tools/rb12/cli_check.py`, standard-library
  Python, ~17 s): through the real executable on every platform —
  `--build_info` is valid and names the three sources; the public
  `quasistatic-semi` smoke completes single-threaded with its 4 saved
  steps and a `completed` manifest that names this very binary, the input
  file and the two meshes by hash and equals `--build_info`; a broad-phase
  resource limit exits 3 with a `resource_failure` manifest, the
  `PolyFEM stopped:` line and no accepted step (RB-05's contract); an input
  refused at init (`dhat = 0`) exits 1 with no output and no manifest;
  `output/manifest = ""` writes none. Registered when a Python 3
  interpreter is found (the CI configurations find one).
- `continuous.yml`: `LastTest.log`, `LastTestsFailed.log` and the CLI
  contract's manifests/logs are uploaded `if: always()` on all three
  platforms; the Windows sccache size is the documented
  `SCCACHE_CACHE_SIZE=1G` instead of the rejected `--max-size` option.
- Dependency forks: their `Build` workflows triggered on `main` only while
  the maintained branches are `semi-implicit-stiffness` (IPC) and
  `iteration-callback` (PolySolve); `sdast9/ipc-toolkit` had never run a
  workflow (0 runs), `sdast9/polysolve` last ran on `main` on 2026-07-18.
  Companion commits (workflow files only, no source):
  `sdast9/ipc-toolkit@c24d803e` and `sdast9/polysolve@bce32a39` add the
  maintained branch and `workflow_dispatch` to the `Build` trigger. The
  PolySolve push started Build run
  [34889533898](https://github.com/sdast9/polysolve/actions/runs/34889533898)
  on `iteration-callback`; the IPC push started nothing (a fork with no run
  history), a manual dispatch did — Build run
  [34889554146](https://github.com/sdast9/ipc-toolkit/actions/runs/34889554146)
  on `semi-implicit-stiffness`. Outcomes (`ci-stage3/*-jobs.txt`):
  **PolySolve 6/6 green** (Linux, macOS, Windows × Debug, Release) — the
  first native check of the fork's PF-06 and RB-19 changes on every
  platform. **IPC Toolkit 4/6**: Linux and macOS Debug/Release green, so
  the fork's `stiffness_scale`, parent contributions and broad-phase budget
  compile and pass their tests there; the two Windows lanes fail on
  **upstream** problems — Windows Debug cannot compile oneTBB under the
  workflow's toolchain (`profiling.h:148: cannot convert 'const char*' to
  'const wchar_t*'`, a dependency build error before any toolkit source),
  Windows Release builds and passes 284/285 with the upstream `Smooth
  barrier potential real sim 2D C^2` test segfaulting — a GCP/smooth-contact
  test the fork does not touch. Recorded as known upstream lane failures,
  not fork regressions; they were invisible before because the branch had
  never been built by CI. PolyFEM's recipe pins follow the two commits
  (identical sources; `--build_info` reports `matches_declared_pin: true`
  for both against the local overrides), so the declared and effective
  sources agree again.

CI outcome at `1f6f826fa` (stage 1 publication; logs in
`outputs/rb-12/20260914T133427Z-identity/ci-stage1/`): pre-commit
**passes** (run 34854316144, the first green formatter run since
2026-09-13's `a327e2932`). Build run 34854316022, every lane still red:

| Lane | Outcome | Cause |
| --- | --- | --- |
| Linux GCC Debug / Release | build fails at `tests/test_run_manifest.cpp:47` | `-Werror=missing-braces`: my own beam writer copied the unbraced `std::array` initializer — the very pattern this session had just repaired in RB-11's test. Fixed here (`{{…}}`). |
| macOS Release | 336/340; the four known CI-03–CI-06 scene groups | unchanged baseline; **every `[run_manifest]` case passes on the Release lanes**, so the manifest, its hashing and the build-time identity generator compile and work under Ninja/AppleClang there |
| Windows Release (MSVC 19.44, CPP threading) | 324/325; only RB-11's `elastic laws: finite-difference derivatives` | the manifest code compiles and passes on MSVC, `_getpid`/`GetModuleFileNameA` included |
| macOS Debug | 299/308; `[run_manifest][run]` aborts plus the 8 RB-11 tests | `Mesh.cpp:207` `assert(in_ordered_edges_.size() > 0)` in `Mesh::create(GEO::Mesh &)` — see the Debug repairs below; the RB-11 `input_validation` aborts (Eigen resize, `MatParams.cpp:534`) are theirs |
| Windows Debug | 296/306; `[run_manifest][run]` "Failed" plus 9 RB-11 tests | the same `Mesh.cpp:207` assertion (`Assertion failed: mesh->in_ordered_edges_.size() > 0`), which MSVC reports as a failed test rather than an abort |

### Debug-lane repairs (2026-09-14, this publication)

Reproduced in an isolated Debug build of the same sources
(`polyfem/build-debug`, AppleClang 21, `-DCMAKE_BUILD_TYPE=Debug`, the
same local dependency overrides; evidence
`outputs/rb-12/20260914T142106Z-repeat/debug-build/`). Two Debug-only
defects on the public inputs, both fixed as bounded repairs:

1. `Mesh::create(GEO::Mesh &)` asserted `in_ordered_edges_.size() > 0` and
   `in_ordered_faces_.size() > 0` for volume meshes read through geogram.
   A MEDIT tetrahedral mesh — the smokes' `cube.mesh`, the RB-11 and RB-12
   test beams — lists no edges and no facets; the only consumer
   (`VarForm::build_node_mapping`) already handles the empty case by
   disabling the node ordering with a warning, so the invariant was false
   and the Release binary ran these inputs with empty connectivity all
   along. The asserts are gone, the facet copy is guarded (it also read
   `facets.nb_vertices(0)` of an empty facet store and looped over
   `edges.nb()` instead of `facets.nb()`), Release behaviour is unchanged
   (empty edges/faces stay empty). `[linear_elastic]` (111 assertions / 3
   cases) and `[run_manifest]` (158 / 7) now pass in Debug; four of the
   nine macOS-Debug failures at `1f6f826fa` were this.
2. `ElasticVarForm::elastic_output_fields` appended the obstacle vertices'
   rows of a sampled DOF field by slicing `obstacle->ndof()` values, which
   is the right count only for vector fields; for the averaged scalar and
   tensor fields (`von_mises_avg`, `*_stress_avg`) it sliced 12 values into
   4 rows — an Eigen `DenseBase::resize` assertion in Debug (the CLI
   contract's smoke aborted while saving `step_0.vtu`) and, in Release, a
   silently wrong obstacle row: the slab's four vertices carried copied FE
   von Mises values up to 2.9e6 in every smoke's VTU. The slice is now
   `n_vertices × field_dim`; the obstacle rows read 0 (no element touches
   them). Verified on `quasistatic-semi-friction --max_threads 1` before /
   after: `solution` and every other field bit-identical, only
   `von_mises_avg` rows 1536–1539 changed (`smokes-stage3/` vs
   `smokes-stage3b/`). The CLI contract passes against the Debug executable
   (5/5 checks, the Debug smoke in 42 s); the HDA end-to-end tests
   (`test_polyfem_hda`, `test_readpvd_materials`, `test_polyfem_materials`)
   pass with the repaired Release binary.

After both repairs, with the Release binary `c56aa4c0…` (Debug `bcce1870…`) recorded in
`binary-identity-stage3.txt`: five smokes within 7e-16 of the RB-11 Stage 1
fingerprints, single-threaded friction bit-identical (`smokes-stage3b/`);
`[run_manifest]` 158 / 7, the affected baseline selection 1,573 / 24,
`[output],[rb22],[input_validation],[linear_elastic]` 992 / 30, CTest
`cli_contract` passes (16 s); in Debug `[run_manifest],[linear_elastic],[output]`
278 / 11.

### Defaults table: historical, opt-in, selected

The plan asks for one table that tells the three apart. Effective defaults
of this fork at `1f6f826fa` (`json-specs/input-spec.json`; every value is
also in a run's manifest under `input.effective`):

| Setting | Historical behaviour | Opt-in / candidate | Explicitly selected production behaviour |
| --- | --- | --- | --- |
| `solver/contact/barrier_stiffness` | upstream adaptive IPC stiffness (Li 2020) or a fixed number | `"semi_implicit"` per-contact stiffness (Ando 2024) — selected per scene; the smokes and the Houdini asset select it | not a global default: the spec default stays the upstream adaptive mode |
| Constraint floor (`semi_implicit/constraint_floor`) | positive floor with barrier deletion/projection (PF-02) | — | **retired** (2026-09-07); a positive value is ignored with a warning and cannot reactivate it |
| Per-contact coefficient law | median with zeros, zero κ at nonpositive curvature, stencil-keyed | — | RB-18 law (positive-only median, relative floor `median/kappa_spread`, `|wᵀHw|` then `max|H|/d̂²` fallbacks, `d̂²`-normalised `conditioning_cap` 1e3), 2026-09-11 |
| `semi_implicit/coefficient_identity` | `"stencil"` (EV/VV jump) | `"stencil"` remains selectable | `"parent"` (RB-21, 2026-09-12) |
| `semi_implicit/force_continuation` | off (re-estimation at every refresh, 17–53 % drift) | `false` restores it; `continuation_max_ratio > 1` bounded pull | `true`, `continuation_max_ratio 0` (RB-20, 2026-09-11) |
| `semi_implicit/friction_lag` | `"follow_stiffness"` (RB-18 F6: friction follows the in-solve trim) | `"follow_stiffness"` remains selectable | `"realized_force"` (RB-10 user decision 2026-09-13) |
| `solver/contact/friction_iterations` | upstream 1 | explicit 1 for historical goldens (CI-03) | 2 (RB-10 user decision 2026-09-13) |
| `semi_implicit/trial_displacement_cap` | uncapped trial sweeps | — | 50 barrier supports, semi-implicit mode only |
| `semi_implicit/gap_floor` | — | experimental force saturation (`> 0`) | 0 (off) |
| `solver/contact/CCD/resource_limits` | none (the kernel killed the process) | `0` disables; explicit custom limits | `-1` automatic: 1e8 hash-grid items / 5e7 emissions, enforced before allocation, exit 3 (RB-05 user decision 2026-09-12) |
| Automatic timestep / load retry | none | — | **none** (RB-08 user decision 2026-09-13: a failed step reports and stops) |
| Line-search roundoff fallback (`line_search/use_grad_norm_tol`, `Armijo/roundoff_tolerance`) | none (stalls at the energy floor) | `0` restores the previous behaviour | gradient-norm fallback at machine epsilon (RB-19) |
| Q2+/serendipity hexahedral collision surface (`tessellation_type`) | faces silently skipped, crash or absent body | `"max_order"` lattice | `"dof"` proxy selected automatically for Q2+ hex boundaries; Q3+ blocked (RB-22 user decision 2026-09-12) |
| Input validation | late context-free failures, silent acceptance | — | named early failures, exit 1, no output (RB-11, 2026-09-13); lumping warning; unit-system notice |
| `output/physical_diagnostics`, `physical_balance_tolerance` | no endpoint record | the RB-04 streams (off by default) | `physical_balance_pass` = force residual at 1e-6 of the peak external force (RB-09 user decision 2026-09-13), observational |
| `output/manifest` | no run identity | `""` disables | `run-manifest.json` for every `PolyFEM_bin` run (RB-12, 2026-09-14); library default off |
| Exit statuses | abort on every uncaught failure | — | 0 / 1 named failure / 3 resource failure (RB-05 follow-up 2026-09-12) |

A successful microfixture does not authorize promotion: nothing in this
table was promoted by RB-12; the rows cite the decision that set them.

### CI classification at the stage-1 publication

| Lane | Known baseline failure (kept) | New regression at session start | Fixed by this session | Unavailable |
| --- | --- | --- | --- | --- |
| pre-commit | — | clang-format drift in four RB-11 files | yes (`1f6f826fa`, run 34854316144 green) | — |
| Linux GCC 13 Debug/Release | — | `-Werror=missing-braces` in `test_linear_elastic_time.cpp` (build-blocking); then the same in `test_run_manifest.cpp` at `1f6f826fa` | yes (`1f6f826fa`, then this publication); the lanes' test outcome is first read from this publication's run | — |
| macOS Release | CI-03–CI-06 scene groups (`contact_2d`, `triangle_data`, `standard`, `contact_3d`) | — | not in scope | — |
| macOS Debug | — | 8 RB-11 tests abort on Debug assertions; `[run_manifest][run]` at `1f6f826fa` | partly: the `Mesh.cpp:207` assertion (4 RB-11 `linear_elastic` tests and the manifest run test) and the output-field assertion are repaired here; the RB-11 `input_validation` aborts (Eigen resize in the material validation, `MatParams.cpp:534`) belong to RB-11's follow-up (a concurrent session holds it) and are not touched | — |
| Windows Debug / Release | — | 9 / 1 RB-11 test failures (fail-fast `0xc0000409`, `elastic laws` derivatives); `[run_manifest][run]` on Debug at `1f6f826fa` | the `Mesh.cpp:207` share (4 `linear_elastic` + the manifest run test) is repaired here; the rest has the same owner | — |
| Cross-compiler repeatability matrix | — | — | — | needs another host; `cli_contract` is the portable check |

### Houdini provenance (2026-09-14, `sdast9/houdini-plugins@6f5fcdc`)

`build_params` appends the `provenance` block (producer `houdini`, the
Houdini version, the asset type name and `.hdanc` file with its SHA-256,
the asset version, the scene file, the export time); `read_params` treats
the key as represented (it is regenerated at the next export); the
end-to-end test `test_polyfem_hda.py` asserts the block and its round trip
into the solver's manifest (`producer` equals the exported block, the
manifest `completed`, the input file hashed). The export now needs PolyFEM
`1f6f826fa` or later — an earlier strict build refuses the unknown key —
which the HDA README states. Published through the separate checkout
procedure (assets rebuilt inside the clone, installed copies verified equal
to the published files, no `backup/`); the full HDA suite passes (13/13,
`outputs/rb-12/20260914T142106Z-repeat/hda-publication/`).

Found on the way, and a stage-3 finding in its own right:
`test_readpvd_hda.py` compared a cached *minimal-fields* run from
2026-07-17 with the *full* smoke run regenerated on 2026-09-13 by a later
solver — 4.4e-4 of displacement apart after the RB-18/20/21 coefficient
repairs — and failed its PK2 comparison (max error 1.2e4) on that staleness
alone; it had passed on 2026-09-12 only because the full run was then the
old one. The test now regenerates its derived runs when they are older
than the full run or the binary (the manifest would make this exact —
compare `process.executable.sha256` — but the pre-RB-12 full run has none).
The rule the failure illustrates is RB-12's: a comparison is only as good
as the identity of both sides.

Rebased publication check: after rebasing onto the concurrent RB-11
follow-up (`eaa624098`/`7aaf53f8e`), the combined Release selection passes
(2,600 assertions / 55 cases: `[run_manifest],[linear_elastic],[output],[input_validation]`
and the affected baseline), `cli_contract` passes, and the Debug selection
`[run_manifest],[linear_elastic],[output]` passes (857 / 13) — RB-11's new
BDF tests included.

### The RB-11 leftovers on the Debug and Windows lanes (2026-09-14, this publication)

RB-11 is complete as an item; what its tests left on the CI lanes had no
owner once its follow-up session finished, and they were the difference
between "classified" and "green apart from the known scene groups", so
they are repaired here as the bounded fix stage the plan provides for. All
reproduced in the local Debug tree first (`debug-build/rb11-leftovers/`):

| Failure (macOS/Windows/Linux Debug; Windows/Linux Release) | Cause | Repair |
| --- | --- | --- |
| `elastic parameters are validated at the element barycenters`, `shear-only laws accept the incompressible limit` — abort at `MatParams.cpp:534` | `LameParameters::lambda_mu` asserted finite λ/μ, but `validate_material_parameters` evaluates the parameters at the barycentres precisely to *report* a non-finite pair (ν = ½ → λ = ∞), and the shear-only laws accept ν = ½ by design | the asserts are removed; the validation is the authority (Release unchanged: the asserts were compiled out) |
| `fibre directions: constant, expression and dimension`, `density, fibre and dispersion parameters are validated` — Eigen "Invalid sizes when resizing" | `GenericFiber::parameters()` copied the fibre functor's result into a fixed `Vector3d`: a 2-D direction (2×1) asserts in Debug and reads past its end in Release; the tensor form (size×size) likewise | the components are read at the functor's own size; for the tensor form the diagonal entry of each axis |
| `duplicate rest elements are refused` — `assert(false)` at `CMesh3D.cpp:460` | a duplicated cell in a geogram-loaded mesh reached geogram's adjacency construction, whose `assert(false)` fired before `validate_rest_elements` could refuse it | the RB-11 topological cell check (`check_cells`) now runs on the geogram path before the connectivity is built, shared with the matrix path; the test expects the named error from `Mesh::create` |
| `elastic laws: finite-difference derivatives` — GCC 13 and MSVC 19.44 Release, `NeoHookean-nearly-incompressible` Hessian | finite-diff's entrywise rule holds analytically-zero entries to 1e-5 absolutely, while the central-difference noise of a Hessian with 1e7 entries (ν = .4999, λ ≈ 3e7) is ~1e-1 on those zeros — under 1e-5 on AppleClang by luck, not on GCC/MSVC | that law's Hessian is compared entrywise at the matrix's scale (`|x − y| ≤ 1e-5 · max(1, max|x|, max|y|)`), the same relative precision on the entries that carry the law; every other law keeps finite-diff's rule |

Validation: Release `[input_validation],[rb11_envelope],[linear_elastic],[run_manifest],[output]`
1,461 assertions / 36 cases; the affected baseline plus
`[assembler],[material_cache],[form_derivatives]` 4,771,486 / 83;
`cli_contract`; five smokes within 3e-15 of the Stage 1 fingerprints,
serial friction bit-identical (`smokes-stage3c/`); the 93-case RB-11 input
matrix in verify mode (`rb11-matrix/`, 93/93); Debug `[input_validation]`
168 / 18 and the full Debug selection
`[input_validation],[linear_elastic],[rb11_envelope],[run_manifest],[output]`
1,461 / 36 (`debug-build/rb11-leftovers/debug-final.txt`). `von_mises_avg`
and the other outputs are unchanged by these repairs. Published as
`fffc722b9`; the isolated Debug tree was deleted afterwards (user decision).

CI at `fffc722b9` (Build run 34903453744; pre-commit run 34903453923
green; logs in `ci-stage3/polyfem-fffc722b9/`): **Linux GCC Release
340/344 and macOS Release 339/343 — only the four known CI-03–CI-06
scene groups** (`standard`, `contact_2d`, `contact_3d`, `triangle_data`);
the derivative test now passes on GCC. The two required build lanes are
therefore at their known baseline. **Linux Debug, macOS Debug and Windows
Release: green** (every test, `[run_manifest]` and `cli_contract`
included) — the first green Debug lanes since the RB-11 tests were added
and the first green Windows lanes of the fork; **Windows Debug green as
well**, so every lane is either green or at the four known scene groups. (At `20d7c360a` Linux Debug had exactly the six leftovers, macOS
Debug the five aborts, Windows Release the one derivative failure; Windows
Debug was cancelled by this push.)

| Lane at `fffc722b9` | Result |
| --- | --- |
| pre-commit | green |
| Linux GCC 13 Release | 340/344 — the four known scene groups |
| Linux GCC 13 Debug | green (312/312) |
| macOS AppleClang Release | 339/343 — the four known scene groups |
| macOS AppleClang Debug | green (311/311) |
| Windows MSVC 19.44 Release | green (328/328) |
| Windows MSVC 19.44 Debug | green (309/309) |

### User decisions (2026-09-14)

- **CI scope:** the required set is pre-commit, Linux Release and macOS
  Release; the four CI-03–CI-06 scene groups stay tracked exceptions with
  owners; the Windows lanes are informational until every remaining
  failure is repaired (after this publication the expected Windows state
  is green — to be read from its run).
- **Per-thread memory growth:** its own bounded item, **RB-24** (plan
  section added), starting from `rss-probe-2/`; until then, a lower
  *Max Threads* on large scenes is the workaround.
- **`polyfem/build-debug`:** deleted once this stage's Debug validation is
  complete (rebuild takes ~20 minutes when needed).

## Next session handoff

- Completed: stage 1 (`1f6f826fa`), stage 2 characterization, the stage-3
  checks (`f5f59db26`), the dependency forks' CI triggers and the pin bump
  (`4f5acc773`), the Houdini provenance block (`6f5fcdc`), the RB-11
  leftovers on the Debug/Windows lanes (`fffc722b9`), the user's decisions
  (required lanes, RB-24, Debug tree deleted). Every stage's GitHub run is
  read and linked in the record.
- Acceptance: the manifest identifies the tested artifacts (yes — every
  evidence run's `run-manifest.json`, the `cli_contract` check on every
  platform); the repeat matrix and its declared tolerances are saved (yes —
  matrices A and B with the rule in `tools/rb12/README.md`); appropriate CI
  is run and linked (yes — the required lanes at their known baseline, the
  exceptions named with owners, the dependency forks' first native runs);
  README claims match the measured scope (yes). Status `validated within
  stated scope`; stage 2's findings are limits, not promises: threaded
  friction runs reproduce only to the contact-model scale, one platform
  and configuration were sampled.
- Open, with owners (checked against the CI plan at closure, see
  [Closure](#closure-2026-09-15)): the four CI-03–CI-06 scene groups (the
  CI plan's items); the IPC fork's two upstream Windows lane failures
  (CI-07 item 8); the cross-platform repeat matrix (CI-07 item 9); RB-24
  (per-thread memory). Nothing is left under RB-12.
- Next command: `ctest --test-dir build -R cli_contract -V` after any
  change to `main.cpp`, the manifest or the exit statuses;
  `python3 tools/rb12/repeat.py --binary build/PolyFEM_bin --output <fresh>`
  for a new platform or build configuration. Next eligible items: RB-23,
  RB-24, RB-06.

## Closure (2026-09-15)

The user asked whether RB-12 can close and, if anything remained, that it
be checked against the CI plan's items. Every open item of the handoff has
a home outside RB-12:

| Left open by RB-12 | Home | Where it is recorded |
| --- | --- | --- |
| The four scene groups failing on the Release lanes (`contact_2d`, `standard`, `triangle_data`, `contact_3d`) | CI-03, CI-04, CI-05, CI-06 | already the CI plan's items since 2026-09-13; tracked exceptions of the required lanes by the user's decision |
| IPC Toolkit fork: Windows Debug stops in oneTBB 2022.3.0 under MinGW GCC 15.2 (`profiling.h:148`), Windows Release 284/285 with the upstream `Smooth barrier potential real sim 2D C^2` segfault | CI-07 (dependency-fork CI) — **added today as item 8** | [ci-portability-plan.md](ci-portability-plan.md#ci-07--workflow-triggers-test-selection-and-reproducible-builds), update 2026-09-15 |
| Cross-platform / cross-compiler repeat matrix (stage 2 ran on macOS arm64 only; the plan says record it as pending) | CI-07 (platform coverage) — **added today as item 9** | same section |
| Per-thread memory of the contact path | RB-24 | plan section, opened 2026-09-14 on the user's decision |

What RB-12 completed of the CI plan is now written there as well (CI-01
and CI-02 complete on the native lanes at `fffc722b9`; of CI-07 the
dependency triggers, the sccache fix, the always-on test-log uploads, the
build identity on every lane and a first public integration run on all
three OSes), so the next CI session starts from the measured state rather
than the 2026-09-13 audit.

README claims: the fork section of `README.md` still named the
2026-09-08 baseline, the pre-RB-12 dependency pins (`9da3094`,
`4d372fa8`) and "all RB items initially remain unimplemented", and said
nothing about the fork's own CI — corrected today to the current pins,
the plan's status table and the measured lane state (the upstream badges
stay, labelled as upstream's).

Acceptance re-read at closure: manifest identifies the tested artifacts —
yes; repeat matrix and declared tolerances saved — yes; appropriate CI run
and linked — yes, on the user's required-lane decision; README claims match
the measured scope — yes after today's correction. No dependency upgrade,
upstream merge, release tag or feature promotion was made. **Status:
closed — validated within stated scope**; the stage-2 limits (threaded
friction to the contact-model scale; one platform sampled) stand as
documented, with the cross-platform matrix now a CI-07 item.
