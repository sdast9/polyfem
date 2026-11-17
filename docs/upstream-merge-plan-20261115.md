# Upstream merge plan: IPC Toolkit and PolyFEM (2026-11-15)

Status: implementation underway; see [the dated execution record](upstream-merge-record-20261115.md).
PolySolve is merged and published (section 1). Written for
implementation sessions that have not seen the review; each task below is meant
to be handed to one session on its own. Read sections 1–4 before any task.

A task is finished only when its acceptance checks pass and its record is
written. If a "stop" condition is met, stop, write down what you found in the
task's record, and report to the user. Do not work around a stop condition.

## Amendment: upstream remeshing alignment (user, 2026-11-17)

The user superseded decision 1: align with upstream MMG-based shape-optimization
remeshing, rather than revive deprecated in-timestep dynamics remeshing. RM-0
failure evidence and exploratory work remain archived. RM-1 and RM-2 are no
longer implementation prerequisites. PF-6 takes upstream removal of ITR and
legacy State; do not restore the remesh tree/build option. PF-6 through PF-8
use the upstream build with no ITR switch. ITR-specific gates are replaced by
MMG utility and optimization-workflow coverage plus explicit rejection of old
`space/remesh/enabled` inputs. Run the baked-coordinate restart oracle as well
as the malformed-MSH corpus in PF-7, then retain those coverage checks in PF-8.
Other preservation/validation gates remain. HDA-1 must clearly mark retired in-timestep controls/imports,
not silently reinterpret them as optimization settings, and remains uninstalled.

This intentionally means no in-timestep dynamic displacement/velocity/
acceleration/contact-history transfer support. Optimization remeshing restarts
shape optimization, not the ongoing dynamics timestep. Keep all other fork
capabilities, including write-only State diagnostics and checkpoint extensions.
The original task descriptions below are historical where this amendment differs.

## 1. State on 2026-11-15

| Repository | Fork branch and tip | Upstream tip | Merge base |
| --- | --- | --- | --- |
| PolySolve | `iteration-callback` `f175cabe`: **merged with upstream `623a7ea` and published 2026-11-15** (record: `docs/upstream-integration-20261115.md` in the PolySolve repository) | `623a7ea` | — |
| IPC Toolkit | `semi-implicit-stiffness` `7b8fcac7` | `7d668025` (64 commits ahead of the base) | `869e489e` |
| PolyFEM | `main` `320a08c1b` plus the PolySolve pin commit | `f974f45ed` (77 commits ahead of the base) | `591b08bd5` |
| Test data | `sdast9/polyfem-data` branch `fable-fixtures`, pinned `aed03ab9` | `polyfem/polyfem-data` `ab3246ce` at the PolyFEM tip | — |

Upstream PolyFEM `f974f45ed` pins IPC Toolkit `dff05854` (2026-10-21) and
PolySolve `621e8fb3`. IPC Toolkit's later commits have not been run under any
PolyFEM, upstream's included.

Suite baseline for task ladder V5, measured 2026-11-15 on fork `main`
`320a08c1b` with the merged PolySolve: 441 cases / 5,179,398 assertions pass,
none fail, 19 minutes (`unit_tests` run directly, macOS arm64,
RelWithDebInfo). The same build passes all 31 Houdini asset test files.

The numbers in this plan come from in-memory trial merges
(`git merge-tree --write-tree`) of the fork tips above. Nothing in IPC Toolkit
or PolyFEM has been built or run against upstream yet.

## 2. Decisions (user, 2026-11-15)

1. **Remeshing is kept.** The fork keeps its copy of the in-timestep remeshing
   code (`src/polyfem/mesh/remesh/`) and moves it onto upstream's VarForm
   structure, because upstream deleted it (#555).
2. **The legacy solver path is dropped.** Inputs that only
   `polyfem::legacy::State` handled are lost, as upstream intends. After the
   merge they must fail with a named error, not run differently.
3. **Restart moves to upstream's HDF5 checkpoint** (#541, `/output/checkpoint`,
   `/input/checkpoint`, `--checkpoint`). The fork's restart extensions are
   ported into it. `restart_json` and `output/data/state` restart are retired.

Standing decisions that this work must not change: ESP contact (#542) stays
opt-in and is not offered in the Houdini asset
([esp-assessment-20261025.md](esp-assessment-20261025.md)); no solver default,
tolerance, trim rule or friction policy changes; `POLYFEM_WITH_MISO` stays
`option(... OFF)` (upstream re-adds `set(... ON)` in merges, undo it each time).

Facts found during the review that the decisions rest on:

* **Remeshing is not in the binary people run.** `POLYFEM_WITH_ITR` defaults to
  OFF and is OFF in `polyfem/build`. With remeshing ticked in the Houdini
  asset, the run goes to the legacy path and logs "built without ITR support;
  remeshing will be disabled". No CI lane (fork or upstream) builds ITR, the
  test suite has no remeshing case, and `polyfem/data` has no scene with
  remeshing enabled. Whether the remeshing code compiles and runs at fork
  `main` is **unknown**; task RM-0 finds out.
* Upstream's own `VarFormFactory::supports` still returns false for
  `/space/remesh/enabled`, and upstream removed the `POLYFEM_WITH_ITR` option
  from `CMakeLists.txt`. It kept the `wmtk.cmake` and `lagrange.cmake` recipes.
* The Houdini asset writes `/space/remesh`, `output/restart_json` and
  `output/data/state` (`RESTART_STATE_FILE`).

## 3. Rules for every task

**Where to work**

* Work only in worktrees under
  `/Users/stevenabramowitch/Downloads/fable_polyfem/upstream-merge-work/`.
  Other sessions use the shared checkouts (`polyfem/`, `ipc-toolkit-fork/`,
  `polysolve-merged/`) and the shared `polyfem/build`. Do not edit, switch,
  rebuild or reconfigure them. A failed link deletes `build/PolyFEM_bin`.
* Give `git worktree add` an absolute path.
* Integration branches, one per repository, all named `upstream-merge`:
  * PolyFEM: `upstream-merge-work/polyfem` (switch that worktree to a new
    branch `upstream-merge` created from `origin/main`).
  * IPC Toolkit: `upstream-merge-work/ipc-toolkit` (new worktree, branch
    `upstream-merge` from `semi-implicit-stiffness`).
  * Test data: a clone of `sdast9/polyfem-data`, branch
    `fable-fixtures-upstream-merge` from `fable-fixtures`.
* Push the integration branches to `origin` after every finished task so the
  work is not only on disk. **Do not push to `main`,
  `semi-implicit-stiffness` or `fable-fixtures`, and do not publish Houdini
  assets, before task REL-1.** REL-1 needs the user's go-ahead.
* The PolyFEM integration build is `upstream-merge-work/build`, configured by
  `upstream-merge-work/configure-command.txt`. Point `CPM_ipc-toolkit_SOURCE`
  at `upstream-merge-work/ipc-toolkit` once that worktree exists, and add
  `-DPOLYFEM_WITH_ITR=ON` once RM-0 has passed. Keep
  `-DPOLYFEM_WITH_PYTHON=ON` (the `standard` test set needs it) and the two
  `Accelerate_LIBRARY` settings (the Xcode 27.2 SDK does not link).

**Git**

* Merge with `git merge --no-ff --no-commit <upstream commit>`; one merge
  commit per task. Never squash an upstream merge and never rebase the
  integration branch after it is pushed.
* Stage files by name. Never `git add -A` or `git add .`: tests write
  artefacts into the working directory.
* PolyFEM commits go through the installed pre-commit hook (clang-format
  21.1.8). If it reports "files were modified by this hook", the commit did
  not happen: `git add` the files again and commit again. Never use
  `--no-verify`. Homebrew's clang-format is a different version; do not run it.
* The shell is zsh. Write `"${rev}:path"`, not `$rev:path` (zsh reads `:c`,
  `:s` and others as modifiers and mangles the argument).
* A file upstream deleted and the fork never touched is deleted by the merge
  **without a conflict**. After every merge, compare
  `git diff --name-status --diff-filter=D HEAD` against what the task expects
  to lose.

**Resolving conflicts**

Section 6 lists every conflicted file with a rule letter:

* **F** – the fork's change is large and upstream's is small. Take the fork's
  side (`git checkout --ours -- <file>`), then apply upstream's change by hand
  from `git diff <previous upstream commit> <this upstream commit> -- <file>`.
* **U** – the reverse. Take upstream's side (`--theirs`), then re-apply the
  fork's change from `git diff 591b08bd5 <fork commit before the merge> -- <file>`.
* **B** – both sides are substantial. Resolve hunk by hunk with both diffs
  open. Keep every fork behaviour and every upstream behaviour; when they
  cannot both hold, stop.
* **P**, **L**, **R**, **C** – explained in the tasks named in the table.

For every resolved file, check afterwards that no fork change was lost:
`git diff 591b08bd5 <fork commit before the merge> -- <file>` lists the fork's
additions; each one must still be present or deliberately replaced, and a
replacement goes into the task's record.

**Validation ladder**

Each task names the levels it must pass.

* **V1 build.** The integration build compiles `PolyFEM_bin` and `unit_tests`
  with no errors.
* **V2 focused tests.** `unit_tests "<tags>"` for the tags the task names.
* **V3 smokes.** The five scenes in `polyfem/scenes/semi-implicit/`
  (`quasistatic-semi`, `quasistatic-semi-friction`, `transient-semi`,
  `quasistatic-adaptive`, `quasistatic-semi-alhess`), run with
  `--max_threads 1` (`tools/smoke/run_smoke.py` runs them and gives one
  status), compared with the same scenes from the previous task's
  binary (keep each task's binary under `upstream-merge-work/bin/<task>/`).
  Report each scene as: byte-identical VTUs; same solver history with
  differences at roundoff (largest normalized difference at most `1e-12`); or
  different solver history. A different history is not a failure by itself,
  but it is a **stop**: record the first step and Newton iteration where the
  histories part and report. Do not change a tolerance, default or controller
  rule to make outputs agree.
* **V4 probes.** `tools/rb02/run_probe.py` (270 of 270 checks) and
  `tools/rb05/run_probe.py` plus `run_scene_limits.py` (outcomes equal to the
  accepted record), as each tool's README describes.
* **V5 full suite.** All of `unit_tests`, with a time limit of at least two
  hours (`timeout` is not installed; use `perl -e 'alarm N; exec @ARGV'`).
  Baseline: the suite is green on fork `main` (see `CLAUDE.md` and section 1
  for the last measured run). Any failure is new and is a stop, unless
  [open-items-20260929.md](open-items-20260929.md) lists it.
* **V6 Houdini.** The asset tests in `houdini_HDAs/tests/`, run with `hython`
  and `POLYFEM_BIN` set to the integration binary.

Long builds and suites: run them under `caffeinate -s` (never `-i`), in the
background, with a log that has timestamps. The laptop must be on AC power
with the lid open. Never run `pmset`.

**Records**

Each task appends a dated section to
`docs/upstream-merge-record-20261115.md` on the PolyFEM integration branch:
the commits merged, each conflicted file and how it was resolved, the ladder
results with numbers, anything that stopped or was left open. Evidence (logs,
compared outputs) goes in `upstream-merge-work/<task>/`, not in `outputs/`.

## 4. Order

```
RM-0 ─────────────────────────────────────────────┐
IPC-1 ───────────────────────┐                    │
PF-1 → PF-2a → PF-2b → PF-3 → PF-4 → PF-5 → RM-1 → RM-2 → PF-6 → PF-7 → PF-8
                 └→ HDA-1 (prepared, not published)
PF-8 → IPC-2 → IPC-3 → IPC-4 → IPC-5 → REL-1
```

* RM-0 and IPC-1 have no prerequisites and can run at any time, in
  parallel with the PF tasks. PS-1 waits for upstream (see the task) and
  does not hold up REL-1.
* PF-4 needs IPC-1. PF-6 needs RM-2. IPC-2 to IPC-5 come after PF-8 because
  upstream PolyFEM itself stops at IPC Toolkit `dff05854`.
* Only one session at a time writes to a given integration branch.

## 5. Tasks

### RM-0 — Find out whether remeshing works today

Goal: a measured baseline for remeshing at fork `main`, before anything moves.

1. New worktree of PolyFEM `main` (not the integration branch) and a separate
   build directory configured as `configure-command.txt` plus
   `-DPOLYFEM_WITH_ITR=ON`.
2. Build. If it does not compile, record every error. Fix only errors that
   are plainly bit-rot (a renamed member, a missing include); commit the fixes
   on a branch `remesh-baseline`. If a fix needs a design choice, stop.
3. Write three small transient scenes under
   `upstream-merge-work/RM-0/scenes/`: 2D and 3D elastic bodies with contact
   and `/space/remesh/enabled: true` (`type: physics`), and one 2D scene with
   `type: sizing_field`. Start from the remeshing options in
   `json-specs/input-spec.json` and from the parameters the Houdini asset
   writes (`houdini_HDAs/tests/test_remesh_hda.py`, `EXPECTED_REMESH`).
4. Run each scene with `--max_threads 1`. Record exit code, whether the mesh
   changed (element counts per step), wall time, and the outputs. Run each
   twice to see whether the result is repeatable.

Accept: a record stating, per scene, compiles / runs / remeshes / repeatable.
Keep the scenes, binary and outputs; RM-2 compares against them.

Stop: the code does not compile without design choices, or no scene remeshes.
The user then decides whether decision 1 still stands.

### PS-1 — Block size and Hypre ranks after the PolySolve merge

Background. Upstream PolySolve (#124) moved the block size (unknowns per
node) out of the solver options: `linear::Solver::create(params, logger,
strict, dimension)` and `nonlinear::Solver::create(..., norm_type, dimension)`
now take it and call `set_block_size`. PolyFEM, the fork's and upstream's,
passes nothing, so the block size is 1 unless the scene says otherwise. The
Houdini asset writes `Hypre/dimension: 3` on purpose (it switches BoomerAMG to
its elasticity settings); the fork's PolySolve keeps accepting that option
(`f91643e`) so those scenes still run as before, until step 3 removes it.
Upstream PolySolve (#125) also made Hypre split its rows across rank threads,
by default one per core.

1. PolyFEM has to pass the problem dimension when it creates solvers.
   **Upstream has this in progress; do not write a fork-only version**
   (found 2026-11-16). Draft pull request polyfem/polyfem#545 (branch
   `max-dev`, by the author of PolySolve #124 and #125, last updated
   2026-10-05, based on `591b08bd5`, pins that author's PolySolve fork and
   also carries unrelated preconditioner work) contains:
   * `2dfde3e459`: the dimension at the solver-creation calls where the
     system is uniformly interleaved per node (`LinearElasticVarForm`,
     `ScalarVarForm`, `BilaplacianVarForm`, the mass solve in `RhsAssembler`,
     the elastic and contact nonlinear solves, `make_nl_solver`), left at 1
     on purpose for the solver inside `NLProblem`, mixed formulations and
     homogenization; and `NLProblem::block_mapping()`, which gives the solver
     the component of each row of the reduced system when the constraints
     are an axis-aligned selection (Dirichlet conditions), and nothing for
     QR-projected constraints.
   * `08b51cd7b7`: the same for the homogenization and differentiable paths.

   **Decided 2026-11-16: wait for upstream.** Do not port the draft. When
   upstream merges this work it arrives with an ordinary upstream merge, and
   steps 2, 3 and 5 follow that merge. Until then the asset keeps writing
   `Hypre/dimension` and the fork's PolySolve keeps accepting it (`f91643e`),
   so Hypre runs as it does today; that acceptance is the fork's one
   deliberate difference from upstream PolySolve's spec and has no end date
   of its own. At each later upstream review, check whether #545 or a
   successor has merged
   (`git grep -n "block_mapping" upstream/main -- src/polyfem/solver/NLProblem.hpp`
   finds it once it has). Only steps 4 (already decided, nothing to
   implement) and the record apply before that.
2. Check with the Hypre scene of `houdini_HDAs/tests/test_polyfem_hda.py`
   that a run with `Hypre/dimension: 3` in the scene and a run without the
   option (block size from step 1) give the same iteration counts.
3. `Hypre/dimension` goes, to match upstream's spec (**decided 2026-11-16**:
   stay consistent with upstream, as part of this task and not before it).
   The fork's acceptance of the option (`f91643e`) is temporary. Do these in
   order, after step 2 passes:
   1. Asset: in a copy (`upstream-merge-work/PS-1/`, not `houdini_HDAs/`),
      stop writing `"dimension": 3` in the Hypre block of
      `src/polyfem/PythonModule.py`, and change the check in
      `tests/test_polyfem_hda.py` from `linear["Hypre"]["dimension"] == 3` to
      the key being absent. The export-and-run Hypre case must still pass and
      must give the iteration counts step 2 recorded.
   2. PolySolve: on `iteration-callback`, remove what `f91643e` added (the
      `Hypre/dimension` block in `HypreSolver::set_parameters`, the spec
      entry in `linear-solver-spec.json`, the `hypre_dimension_option` test,
      the `block_size` line in `get_info` if upstream's file does not have
      it). Compare the result with upstream's `HypreSolver.cpp` and spec: the
      only differences left should be ones the fork had before the merge.
      Push, then move PolyFEM's pin.
   3. A scene that still carries the key then fails validation the way it
      does upstream (`No rule matched for ".../Hypre"`). The asset writes a
      fresh scene on every run, so saved Houdini scenes are unaffected; only
      scene files kept by hand need the key deleted. Say so in the record and
      in the fork's README.
   4. Publish the asset change only together with a binary that contains
      step 1. An asset without the key on a binary without step 1 runs Hypre
      at block size 1.

   Why step 1 has to come first (measured 2026-11-16,
   `upstream-merge-work/hypre-dimension/`, a 4,913-node NeoHookean cube, one
   rank, one thread): with block size 3 the run took 6.0 s, 19 Newton
   iterations and 5 rejected linear solves; with block size 1, which is what
   upstream PolyFEM runs today, 16.8 s, 44 iterations and 54 rejected linear
   solves. The end results agree.
4. Hypre rank count: **decided 2026-11-16, leave upstream's behaviour.** Hypre
   runs on one nano-mpi rank per core unless `NANOMPI_NUM_RANKS` is set, and
   `--max_threads` does not limit it. Do not add a cap or a default in
   PolyFEM. Consequence to state in the record and in the asset's Hypre help:
   results depend on the rank count (the 2026-11-15 check on a 32³ Laplacian
   gave relative residuals `4.19e-11` at 1 rank, `4.57e-11` at 4 and
   `9.14e-11` at the default on an 18-core machine, each repeatable), so a
   Hypre scene gives different numbers on machines with different core
   counts; set `NANOMPI_NUM_RANKS` to the same value on both to compare runs.
5. Rows of the reduced system. The block size alone assumes row `i` belongs
   to component `i % dimension`; PolyFEM's reduced system drops the
   constrained rows, so that fails when a node is constrained in only some
   components. Upstream's draft answers this with `NLProblem::block_mapping()`
   (step 1). The asset's `Hypre/dimension: 3` has never supplied a mapping,
   so today's Hypre runs on such scenes use a misaligned assignment. Not
   measured in isolation: on the cube of step 3 with the bottom held only in
   z, the option still helped (12.4 s against 19.5 s), but that is also a
   different problem. When step 1 arrives, check on that scene that the
   mapping is passed and compare iteration counts with and without it.

Ladder: V1; V2 `[solver]`-style cases that use Hypre, if any; V3 (expect
byte-identical: no smoke uses Hypre); PolySolve's own suite after step 3.2;
V6 `test_polyfem_hda.py` from the step 3.1 copy against the final binary.

### IPC-1 — IPC Toolkit to `dff05854`

Goal: merge upstream IPC Toolkit up to the commit upstream PolyFEM pins.
Brings ESP (#259), the `smooth_contact` → `gcp` rename, user vertex and edge
areas (#273), CMake option fixes (#268), the Eigen download fallback (#277).

1. Worktree `upstream-merge-work/ipc-toolkit`, branch `upstream-merge` from
   `semi-implicit-stiffness`. `git merge --no-ff --no-commit dff05854`.
2. Nine files conflict. The ones that carry fork behaviour:
   * `normal_collisions.{hpp,cpp}`, `normal_collisions_builder.{hpp,cpp}`:
     upstream adds a `skip_obstacles` flag where the fork added
     `separate_signed_contributions`. Keep **both** flags, as two members and
     two constructor arguments (fork's first, then upstream's; update every
     call site and the Python bindings).
   * `candidates.cpp`: upstream adds an `all_types` argument where the fork
     records broad-phase statistics (`accumulate_statistics`). Keep both.
   * `gcp/gcp_collisions_builder.cpp` (renamed from
     `smooth_contact/smooth_collisions_builder.cpp`): the fork emits smooth
     collisions in a canonical order (`f8dafef3`). Keep that order on the
     renamed builders.
   * `tests/src/tests/collisions/CMakeLists.txt`,
     `tests/src/tests/potential/CMakeLists.txt`,
     `tests/src/tests/collisions/test_normal_collisions.cpp`: keep both
     sides' entries and test cases.
3. Build the toolkit's own tests (clone `ipc-sim/ipc-toolkit-tests-data` into
   `tests/data` first; the download step is not a dependency of the test
   target) and run the whole suite.

Accept: toolkit suite passes with the fork's tests included (the last fork
record, `ipc-toolkit-fork/docs/upstream-integration-20260927.md`, had 339
cases passing and 5 assertion-build skips; the count will grow with
upstream's new tests, no fork test may disappear). The fork's stiffness,
parent-contribution, resource-budget and canonical-order tests all still run.
PolyFEM is not built here; that happens in PF-4.

Stop: a fork test fails and the cause is not a missed conflict resolution.

### PF-1 — PolyFEM to `d7525f5541` (#547 tet refinement)

1. Test data first: merge `polyfem/polyfem-data` at `b1252527` into
   `fable-fixtures-upstream-merge`, push it, and use its commit as the pin.
   Every later PF task repeats this with the data commit upstream pins at that
   task's upstream commit (`git show "${rev}:cmake/recipes/polyfem_data.cmake"`).
   The October review found no overlap between upstream's data changes and the
   fork's nine fixture files; if a fixture file conflicts, stop.
2. Merge `d7525f5541`. One conflict, `cmake/recipes/polyfem_data.cmake`: keep
   the fork recipe with the new fork data pin.

Note for the record: #547 changes results on tet meshes refined with
`n_refs > 0`. No semi-implicit smoke uses refinement. The user has not ruled
on this change separately (it was part of open item D10); the plan takes it
with the merge, and REL-1 lists it among the points to confirm.

Ladder: V1, V3 (expect byte-identical), V5.

### PF-2a — PolyFEM to `946c0b3729` (#541 new IO), merge only

Goal: the new IO layer and upstream's checkpoint are in, the build is green,
and every test that does not concern restart passes. The fork's restart
extensions are **not yet** in the checkpoint; PF-2b does that.

Upstream's change: a resource layer (`io::ResourceIO`), a rewritten mesh
loader (`GeometryReader` becomes `GeometryLoader`), VarForm constructors that
take the resources, HDF5 checkpoint (`io/Checkpoint.{hpp,cpp}`,
`VarForm::serialize_checkpoint` / `deserialize_checkpoint`,
`ImplicitTimeIntegrator::serialize_checkpoint`), `restart_json` and
`tests/test_restart.cpp` removed.

1. Merge. 39 files conflict (section 6).
2. Restart-specific resolutions:
   * `tests/test_restart.cpp` (deleted upstream, extended by the fork): do not
     delete it. Move it to `tests/fork_restart_reference.cpp`, leave it out of
     `tests/CMakeLists.txt`, and note in its first line that PF-2b ports it.
   * Keep the fork's per-form state functions compiling but uncalled:
     `ContactForm::write_restart_state` / `read_restart_state`, the
     `BarrierContactForm` and `FrictionForm` overrides, and
     `NonlinearElasticVarForm::save_restart_form_state`,
     `restore_restart_form_state`, `restore_restart_multipliers`. PF-2b
     rewrites them.
   * Remove the fork's `VarForm::save_restart_json` and its spec entries
     (`/output/restart_json`); take upstream's spec for `/output/checkpoint`
     and `/input/checkpoint`.
3. Fork behaviour that must survive in files upstream rewrote (check each one
   against the fork diff, rule in section 3):
   * `mesh/Mesh.cpp`, `mesh/MeshUtils.{hpp,cpp}`, the MSH reader and its
     `mshio` patch: named errors for malformed MSH numbers, the parametric
     stride repair, positive tets (`[msh]`-style tests in
     `tests/test_input_validation.cpp` and the reader tests must still pass).
   * `assembler/MatParams.{hpp,cpp}`, `Assembler.{hpp,cpp}`,
     `GenericFiber.cpp`, `OgdenElasticity.cpp`, `SumModel.cpp`: per-element
     materials, HGO and fiber parameters, material domain validation.
   * `utils/ExpressionValue.cpp`, `utils/Selection.cpp`, `JSONUtils.*`:
     RB-11 input validation, the `Selection::build` repair.
   * `main.cpp`, `State.{hpp,cpp}`: run manifest, exit codes (3 resource,
     1 named failure), `--max_threads` capping Accelerate, the time-step
     count rule from `320a08c1b`.
   * `NonlinearElasticVarForm.cpp`, `ElasticVarForm.cpp`,
     `LinearElasticVarForm.cpp`, `VarForm.{hpp,cpp}`: the step transaction and
     rollback (RB-06), output time phase (RBR-01), fully prescribed bodies
     (RBR-05), linear-elastic force normalisation, staged transient loop.
4. The legacy State and the remeshing files still exist at this upstream
   commit; resolve their conflicts so they compile (rule **L** / **R**: keep
   the fork's edits on top of upstream's version).

Ladder: V1; V2 on `[input_validation]`, `[linear_elastic]`,
`[output_kinematics]`, `[fully_prescribed]`, `[al_budget]`, `[trim_band]`,
`[rb23]`, `[rbr04]` and upstream's new checkpoint tests in `test_hdf5.cpp`;
V3; V4; V5 (the `[restart]` cases are absent until PF-2b; say so in the
record).

Stop: a fork behaviour in step 3 has no place in upstream's new structure.

### PF-2b — Port the restart extensions into the checkpoint

Goal: a run resumed from an upstream checkpoint continues exactly as the fork
resumes from `restart.json` today.

What the fork stores today (HDF5 state file, flat dataset names):

| Dataset | Written by | Holds |
| --- | --- | --- |
| `u` and the time-integrator history | `NonlinearElasticVarForm`, `VarForm::save_step_state` | solution and history (upstream's checkpoint already stores these; do not duplicate them) |
| `contact_scalars` | `ContactForm::write_restart_state` | contact form scalars |
| `contact_si_scalars` (layout version 2; a 23-entry version 1 is still read) | `BarrierContactForm::write_restart_state` | semi-implicit controller memory |
| `friction_scalars`, `friction_normal_force` | `FrictionForm::write_restart_state` | friction lag state |
| `al_scalars`, `al_multipliers_<i>` | `NonlinearElasticVarForm::save_restart_form_state` | augmented-Lagrangian multipliers, which carry across steps |

Read each of those functions in full before designing; the table is an index,
not a specification.

1. Add `serialize_checkpoint(io::CheckpointWriter &, const std::string &group)`
   and `deserialize_checkpoint(const io::CheckpointReader &, const std::string
   &group, const Eigen::VectorXd &x)` to `ContactForm` (virtual),
   `BarrierContactForm`, `FrictionForm`, replacing the path-based functions.
   Use the writer's typed calls (`write_matrix`, `write_vector`,
   `write_long`). Put everything under `/checkpoint/state/fork/` with one
   group per owner (`contact`, `friction`, `al`) and a `layout_version`
   attribute per group, starting at 1.
2. Call them from `NonlinearElasticVarForm::serialize_checkpoint` /
   `deserialize_checkpoint`, after the base class call.
3. A checkpoint without the `fork` group (one written by upstream PolyFEM)
   must load, with the same log message and behaviour the fork has today for a
   state file without the extension datasets. A group with an unknown
   `layout_version` is a named error. The old `restart.json` format is not
   read; a scene that still sets `/output/restart_json` gets a named error
   that names `/output/checkpoint/path`.
4. `--checkpoint-reorder` / `/input/checkpoint/reorder`: per-node and
   per-collision fork data must be reordered with the solution, or the
   combination refused by name. Decide by reading
   `VarForm::reorder_checkpoint_block`; if neither is clean, stop.
5. Port every case of `tests/fork_restart_reference.cpp` to the checkpoint
   (tag `[restart]` kept), then delete that file. The cases to keep: resume
   equals an uninterrupted run; resume keeps the run-scope initial trim
   estimate; resume keeps AL multipliers; friction state; the file-size bound
   ("state file size stays near its payload").

Ladder: V1; V2 `[restart]`, `[matrix_io]` and the checkpoint tests; V3; V5.

Accept, in addition: for `quasistatic-semi-friction` and `transient-semi`,
running all steps at once and running half, checkpointing, and resuming give
byte-identical VTUs for the remaining steps with `--max_threads 1`.

### HDA-1 — Houdini asset: restart through checkpoints (prepare only)

Goal: the asset writes and resumes from checkpoints. Not published until
REL-1.

1. In `houdini_HDAs/src/polyfem/PythonModule.py`, the restart toggle
   (`restart_json`, `RESTART_STATE_FILE`, the import path around the
   `"restart_json" in output` check) writes `/output/checkpoint/path` and
   resumes with `/input/checkpoint/path` instead. Keep the parameter's name
   and label so saved scenes keep their setting; importing an old
   `params.json` that has `restart_json` turns the toggle on.
2. Update the tests in `houdini_HDAs/tests/` that mention `restart_json` or
   the state file, and add one end-to-end resume test.
3. Work in a copy (`upstream-merge-work/HDA-1/`), not in `houdini_HDAs/`;
   other sessions and the installed assets use that folder.

Ladder: V6 against the PF-2b binary.

### PF-3 — PolyFEM to `008a1aeae9` (#487 AMIPS)

No new conflicted files. Changes the AMIPS energy (absolute mode,
regular-tetrahedron reference, per-element `use_rest_pose`). Ladder: V1, V3
(expect byte-identical), V5.

### PF-4 — PolyFEM to `cc1eac01e1` (#542 ESP) with IPC-1

Needs IPC-1. Point the PolyFEM recipe at the IPC Toolkit `upstream-merge`
commit and at the fork's PolySolve (`4d498266` content is already in it).

1. Merge. Thirteen more files conflict.
2. `BarrierContactForm.{hpp,cpp}` are rule **F**: the fork's change is about
   3,000 lines and upstream's is 27. Upstream's change, to replay by hand:
   four new constructor arguments (`dhat_epsilon_scale`,
   `collision_set_type`, `skip_obstacles`, `barrier`); the
   `collision_set_type` / `use_improved_max_operator` precedence; a guard that
   adaptive stiffness needs the squared-distance barrier; the static
   `cached_displaced_surface` becoming a member (the fork's file no longer
   contains `cached_displaced_surface` at all; keep the fork's mechanism and
   do not add upstream's member); `dhat_epsilon_scale` passed to
   `ipc::update_barrier_stiffness`.
3. Semi-implicit stiffness with a non-default `barrier` or with
   `collision_set_type` other than the ones the fork validated: refuse by
   name in the constructor, as the fork already does for unsupported
   combinations (see `[rbr04]`). Do not let a new option reach the
   semi-implicit law silently.
4. `SmoothContactForm` becomes `GCPContactForm` upstream; the eight fork
   files that use the toolkit's smooth-contact API
   (`grep -rl "ipc/smooth_contact\|SmoothCollisions\|SmoothContactParameters" src tests`)
   follow the rename.
5. `json-specs/input-spec.json` (rule **B**): keep all of the fork's entries
   and add upstream's `contact/barrier`, `collision_set_type`,
   `skip_obstacles`, `dhat_epsilon_scale` and the ESP entries. After
   resolving, check that the file parses and that every `pointer` is unique.
6. `SolveData.{hpp,cpp}`: upstream restructured form construction for ESP;
   the fork's additions (clamped contacts, gradient-balance DOFs, AL budget,
   stall restarts, failure injection hooks) must all still be wired.

Ladder: V1; V2 with `[rbr04]`, `[trim_band]`, `[al_budget]`, `[restart]`,
`[opt_gradient]` and upstream's ESP tests; V3; V4; V5.

Expect in V3: the IPC Toolkit merge of 2026-09-27 changed a friction-scene
controller decision through a distance-type tie. If histories differ here,
that is a stop like any other; record where.

### PF-5 — PolyFEM to `50628af3d7` (#546 macro strain, #554 Eigen mirror)

No new conflicted files. #554 adds an Eigen download line to upstream's
PolySolve recipe; the fork's recipe keeps its own form. Ladder: V1, V3, V5.

### RM-1 — Separate the remesher from the legacy State

Needs PF-5 and RM-0. Still no behaviour change: the legacy path keeps
driving remeshing; this task only removes the type dependency.

The remeshing code reads these members of `legacy::State` (counted in the
fork sources): `args` (68 uses), `resolve_output_path` (14), `formulation`
(7), `solve_data` (6), `is_contact_enabled` (6), `problem` (5), `mesh` (5),
`units` (3), `root_path` (3), `make_nl_solver` (3),
`starting_min_edge_length` (2), `starting_max_edge_length`, `obstacle` (2),
`min_boundary_edge_length` (2), `material_file_cache` (2), `avg_mass` (2),
`out_geom`, `n_boundary_samples`, `is_adhesion_enabled`, `in_node_to_node`,
`boundary_conditions_ids`.

1. Add `src/polyfem/mesh/remesh/RemeshHost.hpp`: an abstract class with one
   accessor per member above (const references; `make_nl_solver` as a virtual
   function), and nothing else.
2. `Remesher`, `PhysicsRemesher`, `SizingFieldRemesher`, `WildRemesher`,
   `LocalRelaxationData` and the `wild_remesh/` files take `const RemeshHost
   &` instead of `const State &`. Remove their includes of
   `polyfem/legacy/State.hpp`.
3. `legacy::State` implements `RemeshHost`. Move the driver
   (`legacy/state/StateRemesh.cpp`: `build_boundary_to_id`,
   `build_edge_energy_maps`, `create_wild_remeshing`, the body of
   `State::remesh`) into `mesh/remesh/RemeshDriver.{hpp,cpp}` as free
   functions over a second small interface, `RemeshTarget`, for the mutating
   steps the driver performs: replace the mesh, set body and boundary ids,
   `load_mesh`, `build_basis`, `assemble_rhs`, `assemble_mass_mat`, rebuild
   the right-hand-side assembler, re-initialise the time integrator from the
   projected history, re-initialise the nonlinear problem.

Accept: with ITR on, the RM-0 scenes give byte-identical outputs before and
after this task. With ITR off, V3 is byte-identical and V5 passes.

Stop: `LocalRelaxationData` needs something from the legacy State that has no
counterpart in the list above.

### RM-2 — Run remeshing on the nonlinear elastic VarForm

Goal: remeshing no longer needs the legacy State.

1. `NonlinearElasticVarForm` implements `RemeshHost` and `RemeshTarget` with
   upstream's own members (`set_mesh`, `load_mesh`, `build_basis`,
   `assemble_rhs`, `assemble_mass_mat`, `build_rhs_assembler`, `solve_data_`).
2. Its transient loop gets the remesh step the legacy loop has
   (`legacy/state/StateSolveNonlinear.cpp`, the `remesh_enabled` block): after
   a converged step, remesh; if the mesh changed, save the remeshed state and
   solve the step again (global relaxation); write `stats.csv` as the legacy
   loop does. The fork's step transaction (RB-06) must wrap this correctly:
   a remesh happens only after the step is committed, and the snapshot taken
   for the next step is of the new mesh.
3. `VarFormFactory::supports` stops returning false for
   `/space/remesh/enabled` when the formulation is a nonlinear elastic one.
   For every other formulation, remeshing enabled is a named error.
4. Built without ITR, a scene with remeshing enabled keeps today's behaviour:
   a warning that remeshing is disabled, then a normal run (now on the
   VarForm path).
5. Refuse by name the combinations the fork cannot carry across a remesh
   rather than running them wrongly. Known ones: per-element material files
   with remeshing (already refused since RB-11); semi-implicit controller
   memory, whose per-collision state is keyed by mesh ids. For the second,
   read how the controller resets at a restart without state and reuse that
   reset at each remesh; if that is not sound, refuse the combination.
6. Checkpoints after a remesh: the checkpoint must store the current mesh
   (upstream's writer has `write_mesh`). If resume after a remesh cannot be
   made to work in this task, refuse `/output/checkpoint` together with
   remeshing by name and record it as open.
7. Add `[remesh]` tests (compiled only with ITR): the RM-0 scenes at reduced
   size, checking that the mesh changes, the run completes, and energy and
   momentum stay within the bounds measured in RM-0.

Accept: on the RM-0 scenes, the VarForm path and the legacy path (same
binary, selected by a temporary test-only switch) agree: identical mesh
sequence, and final displacements within `1e-9` of the scene size. If the
legacy path itself was not repeatable in RM-0, compare against its spread
instead and say so. Ladder: V1 with ITR on and with ITR off; V2 `[remesh]`,
`[restart]`; V3; V5.

Stop: the two paths give different mesh sequences and the cause is not found
in a day of work.

### PF-6 — PolyFEM to `ddadb5398f` (#555 legacy removed)

Needs RM-2.

1. Before merging, list the files upstream deletes:
   `git diff --name-status --diff-filter=D 50628af3d7 ddadb5398f`.
2. Merge. Resolve the legacy modify/delete conflicts by deleting
   (`src/polyfem/legacy/**`): decision 2. The fork's edits to those files
   (about 110 lines: restart, time-step count, input checks) are either
   already present on the VarForm path or lost with the legacy path; list
   each in the record with which of the two it is.
3. Restore the whole remeshing tree from the commit before the merge, since
   the merge deletes the files the fork never touched without a conflict:
   `git checkout <pre-merge commit> -- src/polyfem/mesh/remesh`. Upstream
   moved `MMGRemesh.{hpp,cpp}` up to `src/polyfem/mesh/`; use upstream's copy
   there and drop the fork's copy from `remesh/`. Restore the build wiring
   upstream removed: the `POLYFEM_WITH_ITR` option and its block in
   `CMakeLists.txt`, the `remesh` subdirectory in
   `src/polyfem/mesh/CMakeLists.txt`. Also restore `L2Projection.{hpp,cpp}`
   if the remesher uses it.
4. Legacy-only inputs must fail by name. After upstream's own extensions the
   VarForm path still rejects, among others: periodic contact without
   homogenization, contact and periodic constraints together; homogenization
   on a non-elastic formulation or in a non-quasistatic transient; file,
   periodic or zero-mean constraints on thermo-elastic and fluid
   formulations. Check what `main.cpp` does when `VarFormFactory::create`
   returns null and make sure the message names the unsupported feature.
   Add one `[input_validation]` case per class.
5. `app/app.cpp` (the GUI app, `POLYFEM_WITH_APP`, off in the fork's build)
   lost its legacy branch upstream; take upstream's.

Ladder: V1 with ITR on and off; V2 `[remesh]`, `[input_validation]`,
`[restart]`; V3 (expect byte-identical: the smokes never used the legacy
path); V5; V6 with the HDA-1 copy (the asset's remeshing round-trip test and
its legacy-import test).

### PF-7 — PolyFEM to `188bd09fc8` (#557 PolySolve pin, #559 geogram removed)

1. Merge. `cmake/recipes/polysolve.cmake` keeps the fork pin (the fork's
   PolySolve already contains `621e8fb3`). One new conflict, `CMakeLists.txt`.
2. #559 rewrites `MeshReader`, `MeshUtils`, `CMesh2D`, `CMesh3D` and
   `MMGRemesh` without geogram. The fork's MSH reader work lives in the same
   files and merged without textual conflict in the trial; that is not
   evidence that it still works. Run every MSH and mesh-validation test and
   the malformed-MSH corpus (`msh-malformed-work/`).
3. The remeshing tree mentions geogram only in comments, but build it with
   ITR on to confirm nothing in it or in the `wmtk` recipe needed geogram.

Ladder: V1 with ITR on and off; V2 `[input_validation]`, `[remesh]`, the mesh
reader tests; V3; V5; V6 (the asset's MSH tests:
`test_msh_end_to_end.py`, `test_msh_parser.py`, `test_msh_reader_topology.py`).

### PF-8 — PolyFEM to `f974f45ed` (#558), full validation

Merge (no new conflicted files expected), then the whole ladder V1–V6, with
both ITR settings, and the RB-12 repeat matrix (`tools/rb12`). Rebuild once
with the production configuration (`configure-command.txt` exactly, ITR off)
and use that binary for V3–V6.

### IPC-2 — IPC Toolkit to `c7ada28b` (#286 candidates trivially copyable)

Upstream's candidates no longer derive from `ipc::CollisionStencil`; the
polymorphic interface comes from `candidates/stencil_adapter.hpp`
(`StencilAdapter`, `DTypeStencilAdapter`). 93 upstream files change; two more
toolkit files conflict:

* `broad_phase/broad_phase.hpp`: keep the fork's `checked_count.hpp` include
  and upstream's `candidate_vector.hpp` include.
* `broad_phase/brute_force.cpp`: upstream changes the thread-local storage to
  `CandidateVector<Candidate>`; the fork has its resource-budget check
  (RB-05) in front of the loop. Keep the budget check and take upstream's
  storage type.

The fork's PolyFEM passes candidates where a `CollisionStencil` is expected:
`BarrierContactForm::estimate_stiffness(const ipc::CollisionStencil &, ...)`
is called with `ipc::VertexVertexCandidate(id0, id1)` and the three other
candidate types, and `interpolated_stiffness` takes a stencil too. Those call
sites need the adapter (or the collision types, which still are stencils).
The fork's toolkit code that records parent contributions and builds the
exact displacement map must be checked the same way.

Ladder: toolkit suite; then PolyFEM V1, V2 (`[trim_band]`, `[rbr04]`,
`[restart]`), V3, V4, V5.

### IPC-3 — IPC Toolkit to `54d58ebc` (#287 per-primitive thickness)

Upstream sets each collision's `dmin` to its pair's thickness and inflates
broad-phase boxes by thickness. Conflicts in `candidates.cpp` (the fork's
statistics beside upstream's thickness boxes; keep both, and keep upstream's
restoration of `can_vertices_collide`) and in
`normal_collisions_builder.cpp` (the fork's `parent_of` helper beside
upstream's thickness helpers; keep both).

Check, and write the answer in the record: with no thickness set, is every
collision's `dmin` exactly what it was before? The fork's semi-implicit law
and its probes assume the scene-wide `dmin`. If a zero-thickness mesh changes
any number, stop.

Ladder: toolkit suite; PolyFEM V1, V3 (expect byte-identical), V4, V5.

### IPC-4 — IPC Toolkit to `282d4830` (#289 hash-grid traversal)

Upstream replaced both candidate loops in `hash_grid.cpp` with
`detect_cell_pairs`. The fork had added, in front of each loop, the opt-in
resource-budget check that counts candidate emissions from the sorted items
before anything is allocated (RB-05, checked arithmetic from RBR-02).

Take upstream's `detect_cell_pairs` and put the fork's budget check in front
of each call. Then verify the count still bounds what the new traversal
emits: read `detect_cell_pairs`; if it can emit more pairs for a key than
`n0 * n1` (two sets) or `n(n-1)/2` (one set), the bound is wrong and the
check must be rewritten, which is a stop.

Ladder: toolkit suite including the budget tests; PolyFEM V1, V3, V4 (the
RB-05 probe must give the same 11 outcomes and candidate counts), V5.

### IPC-5 — IPC Toolkit to `7d668025` (tip)

Brings, among others: the Eigen ABI pin (#297, #300), the collision-filter
fix for codimensional passes (#303), the default logger no longer registered
globally (#294), Python packaging changes.

The Eigen pin matters to PolyFEM: with `IPC_TOOLKIT_WITH_SIMD` on (it is on
in the fork's build), upstream now defines `EIGEN_DONT_VECTORIZE` on the
shared `Eigen3::Eigen` target, so PolySolve and every other library that
links Eigen is compiled without Eigen vectorization too. Today only targets
that link `ipc_toolkit` get the definition. Expect last-digit changes and
possibly different solver histories in V3, and a speed change. Measure both:
V3 as usual, plus wall time of the five smokes and of
`test_cases`-scale runs the user names. Report; do not decide. The
alternative to report alongside is `IPC_TOOLKIT_WITH_SIMD=OFF`.

Ladder: toolkit suite; PolyFEM full ladder V1–V6.

### REL-1 — Release (needs the user's go-ahead)

Present the record and the open points to the user first. Then, in this
order: push IPC Toolkit `upstream-merge` to `semi-implicit-stiffness`; fast
forward `fable-fixtures`; set the PolyFEM recipe pins to those commits; merge
PolyFEM `upstream-merge` into `main`; rebuild the shared `polyfem/build`
(back up `PolyFEM_bin` to `bin-backups/` first); publish the Houdini asset
from HDA-1 by the procedure in `houdini_HDAs/AGENTS.md`; update
`docs/open-items-20260929.md` and the workspace `README.md`.

## 6. Conflicted files in PolyFEM

"Task" is the first upstream commit at which the file conflicts in a direct
trial merge with fork `main`. The change sizes are lines added and removed
from the merge base to each side's tip, so they show which side carries more,
not the size of the conflict at that task. 58 files in all; the trial merge
at the tip had 91 conflict hunks.

| Task | File | Upstream change | Fork change | Rule |
| --- | --- | --- | --- | --- |
| PF-1 | `cmake/recipes/polyfem_data.cmake` | +1/-1 | +17/-3 | P (pins: keep the fork recipe and its fork pin) |
| PF-2 | `src/polyfem/State.cpp` | +113/-85 | +106/-9 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/State.hpp` | +26/-19 | +13/-0 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/assembler/Assembler.cpp` | +4/-3 | +268/-11 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/assembler/Assembler.hpp` | +3/-2 | +33/-1 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/assembler/GenericElastic.cpp` | +15/-25 | +2/-0 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/assembler/GenericFiber.cpp` | +2/-2 | +18/-10 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/assembler/HookeLinearElasticity.cpp` | +6/-6 | +1/-1 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/assembler/MatParams.cpp` | +43/-44 | +191/-31 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/assembler/MatParams.hpp` | +15/-15 | +47/-5 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/assembler/OgdenElasticity.cpp` | +8/-8 | +16/-4 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/assembler/SumModel.cpp` | +2/-2 | +16/-3 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/io/CMakeLists.txt` | +5/-0 | +4/-0 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/io/MatrixIO.cpp` | +30/-5 | +34/-1 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/legacy/state/StateInit.cpp` | +0/-444 | +23/-3 | L (legacy: see PF-6) |
| PF-2 | `src/polyfem/main.cpp` | +110/-123 | +86/-2 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/mesh/GeometryLoader.cpp` | +788/-0 | (old name) | U (renamed upstream; take upstream, re-apply the fork edit from the old file) |
| PF-2 | `src/polyfem/mesh/Mesh.cpp` | +239/-288 | +81/-8 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/mesh/MeshUtils.cpp` | +413/-633 | +91/-0 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/mesh/MeshUtils.hpp` | +26/-76 | +20/-0 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/mesh/Obstacle.cpp` | +31/-16 | +6/-4 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/mesh/remesh/wild_remesh/LocalRelaxationData.cpp` | +0/-336 | +15/-2 | R (remeshing: keep the fork file, see RM tasks) |
| PF-2 | `src/polyfem/utils/ExpressionValue.cpp` | +13/-12 | +71/-12 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/utils/JSONUtils.cpp` | +99/-10 | +4/-2 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/utils/JSONUtils.hpp` | +8/-0 | +7/-1 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/utils/Selection.cpp` | +15/-14 | +24/-22 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/varforms/ElasticVarForm.cpp` | +64/-67 | +65/-14 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/varforms/IncompressibleElasticVarForm.cpp` | +36/-23 | +6/-0 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/varforms/LinearElasticVarForm.cpp` | +4/-1 | +61/-8 | F (keep the fork side, replay upstream's diff) |
| PF-2 | `src/polyfem/varforms/NavierStokesFSIVarForm.cpp` | +89/-103 | +5/-3 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/varforms/NonlinearElasticVarForm.cpp` | +353/-82 | +1671/-133 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/varforms/ThermoElasticVarForm.cpp` | +59/-39 | +12/-5 | U (take upstream, replay the fork's diff) |
| PF-2 | `src/polyfem/varforms/VarForm.cpp` | +225/-212 | +38/-5 | B (both sides substantial: hunk by hunk) |
| PF-2 | `src/polyfem/varforms/VarForm.hpp` | +61/-13 | +16/-0 | U (take upstream, replay the fork's diff) |
| PF-2 | `tests/CMakeLists.txt` | +11/-1 | +35/-0 | B (both sides substantial: hunk by hunk) |
| PF-2 | `tests/test_assembler.cpp` | +177/-48 | +191/-1 | B (both sides substantial: hunk by hunk) |
| PF-2 | `tests/test_hdf5.cpp` | +771/-1 | +44/-1 | B (both sides substantial: hunk by hunk) |
| PF-2 | `tests/test_restart.cpp` | +0/-127 | +275/-0 | C (checkpoint port: see PF-2) |
| PF-2 | `tests/verify_run.cpp` | +599/-93 | +53/-4 | B (both sides substantial: hunk by hunk) |
| PF-4 | `.github/workflows/continuous.yml` | +84/-21 | +55/-4 | B (both sides substantial: hunk by hunk) |
| PF-4 | `cmake/recipes/ipc_toolkit.cmake` | +6/-1 | +32/-2 | P (pins: keep the fork recipe and its fork pin) |
| PF-4 | `cmake/recipes/polysolve.cmake` | +3/-1 | +28/-2 | P (pins: keep the fork recipe and its fork pin) |
| PF-4 | `json-specs/input-spec.json` | +112/-36 | +698/-16 | B (both sides substantial: hunk by hunk) |
| PF-4 | `src/polyfem/legacy/state/StateHomogenization.cpp` | +0/-361 | +1/-0 | L (legacy: see PF-6) |
| PF-4 | `src/polyfem/legacy/state/StateSolveNonlinear.cpp` | +0/-490 | +68/-3 | L (legacy: see PF-6) |
| PF-4 | `src/polyfem/solver/SolveData.cpp` | +150/-103 | +111/-3 | B (both sides substantial: hunk by hunk) |
| PF-4 | `src/polyfem/solver/SolveData.hpp` | +10/-0 | +11/-0 | B (both sides substantial: hunk by hunk) |
| PF-4 | `src/polyfem/solver/forms/BarrierContactForm.cpp` | +27/-6 | +2979/-70 | F (keep the fork side, replay upstream's diff) |
| PF-4 | `src/polyfem/solver/forms/BarrierContactForm.hpp` | +11/-1 | +653/-4 | F (keep the fork side, replay upstream's diff) |
| PF-4 | `src/polyfem/solver/forms/FrictionForm.cpp` | +27/-10 | +111/-3 | F (keep the fork side, replay upstream's diff) |
| PF-4 | `src/polyfem/varforms/diff/DifferentiableNonlinearElasticVarForm.cpp` | +39/-164 | +17/-1 | U (take upstream, replay the fork's diff) |
| PF-4 | `tests/test_diff.cpp` | +23/-3 | +18/-2 | B (both sides substantial: hunk by hunk) |
| PF-6 | `docs/varform-design.md` | +23/-34 | +35/-28 | B (both sides substantial: hunk by hunk) |
| PF-6 | `src/polyfem/legacy/State.cpp` | +0/-1746 | +3/-4 | L (legacy: see PF-6) |
| PF-6 | `src/polyfem/legacy/State.hpp` | +0/-805 | +5/-0 | L (legacy: see PF-6) |
| PF-6 | `tests/VarFormTestAccess.hpp` | +65/-9 | +41/-0 | B (both sides substantial: hunk by hunk) |
| PF-6 | `tests/test_problem.cpp` | +160/-28 | +47/-0 | B (both sides substantial: hunk by hunk) |
| PF-7 | `CMakeLists.txt` | +24/-16 | +24/-1 | B (both sides substantial: hunk by hunk) |

IPC Toolkit, cumulative conflicted files in direct trial merges with
`semi-implicit-stiffness`: 9 at `dff05854`, 11 at `c7ada28b` and `54d58ebc`,
12 at `282d4830` and at the tip (23 hunks at the tip).

## 7. Limits of this plan

* It is built from trial merges and reading, not from builds. Compile-level
  breaks that are not textual conflicts (changed signatures, moved headers)
  will appear in V1 and are part of each task.
* Upstream's change sizes for remeshing and legacy files are deletions; how
  much of the remeshing code still compiles against the new VarForm structure
  is unknown until RM-0 and RM-1.
* The staged conflict counts assume each stage is merged into fork `main` as
  it is today. Stages after the first are merged into the result of the
  previous one, so later counts will be somewhat lower.
* Upstream keeps moving. The plan stops at the commits named here; anything
  newer is a later sync.
