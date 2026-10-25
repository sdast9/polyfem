# Upstream ESP contact and the 2026-10-25 upstream state

Date: 2026-10-25. Status: **measured, no source change.** Decision for the
user: D10 in [open-items-20260929.md](open-items-20260929.md).

## Summary

* **Upstream since the fork's last sync** (PolyFEM `591b08bd5`, PolySolve
  `da4e7fe`, IPC Toolkit `869e489e`):
  - PolyFEM `main` `cc1eac01e1` has four merges:
    - #547, a tet red-refinement fix;
    - #541, a new IO layer whose HDF5 checkpoint replaces `restart_json` and
      `output/data/state`;
    - #487, an AMIPS energy fix;
    - #542, ESP contact.
  - IPC Toolkit `main` `33fc60f` adds ESP, renames `smooth_contact` to `gcp`,
    and brings CMake option, Eigen-download and CUDA fixes.
  - PolySolve `main` `5f4903c` adds one opt-in line-search option.
* **ESP is not a clear gain for the fork.** Measured on upstream's own ESP
  scenes and on the fork's semi-implicit scenes:
  - On scenes all three models solve, ESP gives the same outcome as upstream
    IPC and the fork's semi-implicit stiffness. Newton iterations are similar,
    except on `sliding-wedge`, where the fork needs 282 against 73–76.
  - On the deformable `spike-in-crack`, ESP aborts at the scene's own stiffness
    and pins the spike's tip on a convex corner at the other two stiffnesses
    tried. Upstream IPC and the fork carry the tip over the corner.
  - ESP has no adaptive stiffness, so every scene needs a hand-set
    `barrier_stiffness`.
  - Its form derives from `ContactForm`, not `BarrierContactForm`, so the
    fork's semi-implicit stiffness and trim controller would not apply to it.
* **Upstream spends 130–150 ms per time step validating PolySolve parameters.**
  - PolySolve checks its JSON parameters with jse every time a solver is
    created, and upstream PolyFEM creates solvers at every step.
  - The fork's `jse-rule-index.patch` (`8c1150ab3`) removes this cost. With
    only that patch added, upstream runs the ESP scenes 14–24× faster (for
    example 10.8 s → 0.44 s), with byte-identical output.
  - The patch is worth offering upstream (json-spec-engine), alongside D7.

## 1. Upstream changes since the fork's sync

Trial merges were run in scratch checkouts; nothing was committed.

| Repository | Change | Trial merge into the fork | Effect on default behaviour |
| --- | --- | --- | --- |
| PolySolve | #122 `line_search/accept_energy_decrease_in_grad_mode` (default off) | clean | none |
| polyfem-data | AMIPS, tet-refinement and new-IO references; PSD projection for `gcp-contact/parallel-edge` (the D6 remedy) | none of the 50 files overlap the data fork's 9 | references only |
| PolyFEM | #547 tet red refinement cuts the inner octahedron along its shortest diagonal | conflicts only in the data pin | moves results of tet meshes refined with `n_refs > 0`; no semi-implicit smoke uses refinement |
| IPC Toolkit | ESP (#259); `smooth_contact` → `gcp`; user vertex/edge areas (#273); CMake options no longer forced into the cache (#268); Eigen download fallback (#277); CUDA CI | 9 conflicted files, about 12 hunks (`skip_obstacles` beside `separate_signed_contributions`, candidate `all_types` beside the broad-phase statistics, the canonical GCP order on renamed builders) | squared-distance barrier path unchanged by inspection; the merge still needs the full suite and smoke comparison |
| PolyFEM | #541 new IO (228 files): resource layer, mesh IO, HDF5 checkpoint (`/output/checkpoint`, `/input/checkpoint`) replacing `restart_json` / `output/data/state`; `tests/test_restart.cpp` removed | 39 conflicted files | the fork's restart work (controller memory, AL multipliers, friction state, layout version 2) is built on the old format |
| PolyFEM | #487 AMIPS: absolute mode, regular-tetrahedron 3D reference, per-element `use_rest_pose` | no conflicts beyond #541's | changes AMIPS energy |
| PolyFEM | #542 ESP; `SmoothContactForm` → `GCPContactForm` (`smooth_contact_force_norm` kept as alias); `contact/barrier`, `collision_set_type`, `skip_obstacles`, `dhat_epsilon_scale`; volumetric obstacles keep tets; friction no longer downgrades OGC collision sets to IPC | 52 files / 82 hunks for the whole PolyFEM merge | ESP and the new options are opt-in |

## 2. What ESP is

ESP (Extremum Sum Potential) is described here from the upstream code; neither
repository names a paper, though the code cites "supplemental" sections S2 and
S4.

* **Inclusion–exclusion weights.** Each mesh element gets a weight chosen so
  that the elements containing any surface point sum to one. On a closed mesh
  faces count +1, edges −1 and vertices +1. Boundary edges and vertices get 0;
  free edges and isolated vertices get 1.
* **The potential.** A barrier summed over nearby elements with those weights
  approximates a barrier of the distance to the nearest feature, without
  counting shared features twice.
* **Exact classification.** The closest feature is classified with exact
  predicates, so the cancellation stays free of roundoff.
* **Evaluation.** The potential is integrated with face (3D) or edge (2D)
  quadrature. Edge–edge contact uses a separate near/far split (`dbar_factor`).
* **Default barrier.** PolyFEM's default for ESP is an inverse-power barrier;
  upstream's ESP scenes select `normalized_log`.
* **Supported features.** Friction and PSD projection are supported; adaptive
  stiffness is not (`ESPContactForm::update_barrier_stiffness` throws).

## 3. Comparison

**Builds.** Both were built on Linux x86-64 with GCC 13.3, Release,
`POLYFEM_PORTABLE_BUILD=ON` and MISO off:
- upstream PolyFEM `cc1eac01e1`, which pins IPC Toolkit `dff05854` and
  PolySolve `4d498266`;
- fork `main` `3f4a90b43`.

**Run settings.** Every run used one thread (`--max_threads 1`,
`OMP_NUM_THREADS=1`) and `MKL_CBWR=COMPATIBLE`. Pardiso was the solver unless
noted; the "LDLT" rows force `Eigen::SimplicialLDLT`, as upstream's
`tests/verify_run.cpp` does. Each configuration ran once, except the ESP abort,
which was run three times.

**Models compared.**
- **U-ESP:** upstream ESP as shipped, κ = 1e4.
- **U-IPC:** upstream standard IPC with adaptive stiffness.
- **Fork:** `barrier_stiffness: semi_implicit`.

On the ESP scenes, U-ESP and U-IPC use the scenes' ESP-tuned Newton settings
(`iterations_per_strategy: 1`, `use_grad_norm_tol: 500`, PSD projection,
`grad_norm_tol: 2.5e-5`). The fork is shown with its own defaults, and with the
ESP settings where noted.

**Upstream ESP scenes** (steps completed, Newton iterations):

| Scene | U-ESP | U-IPC | Fork (defaults) | Fork (ESP settings) |
| --- | --- | --- | --- | --- |
| `sliding-wedge` | 40/40, 76 | 40/40, 73 | 40/40, 282 | 40/40, 259 |
| `spikes` | 50/50, 483 | 50/50, 556 | 50/50, 484 | 50/50, 897, 37 stall restarts |
| `spike-in-crack-rigid` (in upstream's `tests/esp.txt`), LDLT | 200/200, 615 | 200/200, 607 | 200/200, 797 | — |
| `spike-in-crack` (not in `tests/esp.txt`) | **SIGABRT at 78/200** ("Reached iteration limit in AL"), Pardiso and LDLT, 3/3 runs | 200/200, 1,369 | 200/200, 759 | named failure at 95/200 (200 stall restarts) |
| `spike-in-crack`, U-ESP κ = 1e3 / 1e5 | 200/200, 646 / 721 | | | |
| `boxes` | not runnable: `unit-plane-sub.obj` is not in `esp_data` | | | |

* **`sliding-wedge`:** final states agree to 3.5e-8.
* **`spikes`:** every model lets the spike slide off the obstacle's tip and
  fall about 3.2. This tip-on-tip balance is unstable: ESP falls along +z and
  the others along −x, a roundoff choice.
* **`spike-in-crack-rigid`:** U-ESP reproduces upstream's stored reference
  exactly (`err_l2` 5.120197096133011). U-IPC differs from it by 3.6e-4 and the
  fork by 2.8e-5.
* **`spike-in-crack`:** the base is dragged 5 units across a 45° V-groove,
  frictionless.
  - All models climb the groove wall identically and reach the block's top
    corner (0.5, −0.5) at about step 132.
  - The fork passes the corner at once: the tip is off the block by step 134.
  - U-IPC holds the tip at the corner from step 134 to step 154 (spike
    stretched to 4.18, 39 %) and frees it at step 156.
  - Both the fork and U-IPC end with the tip off the block and the spike back
    at rest length (3.01).
  - U-ESP at κ 1e3 and 1e5 holds the tip at the corner from step 134 to the
    end, with the spike stretched to 5.15 (72 %) while its base sits at the
    prescribed x = 5.
  - At κ 1e4, the scene's own value, U-ESP aborts mid-climb.
  - The scene's stored reference (`err_l2` 4.958) matches none of the models
    (U-IPC 5.002, fork 5.000, ESP 3.877).

**Fork semi-implicit scenes** (cube pressed 0.25 into a slab, 4 steps; Newton
iterations):

| Scene | Fork | U-IPC | U-ESP κ 1e4 |
| --- | --- | --- | --- |
| `quasistatic-semi` | 26 | 37 | 27 (κ 1e3: 41; κ 1e5: 28) |
| `transient-semi` | 27 | 39 | 35 |
| `quasistatic-semi-friction` (μ 0.3) | 62 (two lag passes per step, `friction_iterations: 2`) | 43 | 31 |

* **Frictionless scenes:** final states agree within 8.5e-4 (d̂ = 1e-3). U-IPC
  lets the gap close to 4e-8; the fork and U-ESP keep it at or above 1e-4 (U-ESP
  at κ 1e3: 5e-5).
* **Friction scene:** mean outward slide of the cube's bottom face, against
  0.058 for every model without friction:
  - fork 0.031;
  - U-IPC 0.040;
  - U-ESP 0.052.

  None of the three converged its friction lagging (lag gradient about 1e3
  against a tolerance of 2.5e-3), and there is no reference solution. Not
  adjudicated.

## 4. Upstream's per-step cost

**Symptom.** On these scenes upstream took about 10× the fork's wall time, for
example 35–41 s against 4.3 s for `spike-in-crack`'s 200 steps. Its trace log
shows two gaps of about 65–70 ms at each step before Newton starts.

**Cause.** Sampling stacks with gdb during upstream `sliding-wedge` (U-IPC, 24
samples) put 23 inside `polysolve::nonlinear::Solver::create`, with most frames
in `jse::JSE::verify_json` / `collect_default_rules`. The call chain is
`ALSolver::solve_reduced` → `Newton::create_solver` →
`polysolve::linear::Solver::create`. PolySolve validates its parameters against
its spec whenever a solver is created, at the fork's pin `43ca2e66` and at
upstream `4d498266` alike.

**Why the fork doesn't pay it.** PolyFEM's jse recipe is the one PolySolve
uses, so the fork's rule-index patch (`8c1150ab3`) removes this cost in the
fork.

**Confirmation.** Upstream was rebuilt with only `cmake/recipes/jse.cmake` and
`cmake/recipes/patches/jse-rule-index.patch` copied from the fork; the CPM
cache shows the patched jse source in use. The runs below were sequential, one
at a time, on identical inputs. Every VTU of the patched binary is
byte-identical to the unpatched binary's: 540 files across the six cases.

| Case | Upstream | Patched upstream | Fork (`semi_implicit`, defaults) |
| --- | --- | --- | --- |
| `sliding-wedge`, U-IPC / U-ESP | 10.8 s / 10.6 s (73 / 76 it.) | 0.44 s / 0.53 s | 1.64 s (282 it.) |
| `spikes`, U-ESP | 12.0 s (483 it.) | 0.79 s | 1.50 s (484 it.) |
| `spike-in-crack-rigid`, U-ESP, LDLT | 33.8 s (615 it.) | 2.28 s | 4.00 s (797 it.) |
| `spike-in-crack`, U-IPC | 39.5 s (1,369 it.) | 2.92 s | 4.10 s (759 it.) |
| fork `quasistatic-semi`, U-IPC | 14.7 s (37 it.) | 9.23 s | 3.60 s (26 it.) |

**What this means for speed.** With equal jse, neither side is uniformly
faster.
- On the few-vertex ESP scenes, patched upstream is 1.4–3.7× faster in wall
  time. The fork costs more per Newton iteration there: about 5.4 ms against
  2.1 ms on `spike-in-crack`, where both run Pardiso.
- On the fork's 1,536-vertex cube scene, the fork is 2.6× faster.
- These are single runs on tiny meshes; production meshes were not timed.

## 5. Recommendation

1. **Don't adopt ESP now.** Revisit if upstream gives it an automatic stiffness
   and it handles the deformable `spike-in-crack`.
2. **Before any upstream merge:** the user decides whether the fork moves to
   upstream's HDF5 checkpoint, porting the restart extensions into it, or keeps
   `restart_json`. That decision governs the PolyFEM #541/#487/#542 merge.
3. **Independent of that:**
   - the PolySolve option, the data-fork references and #547 (if its result
     change is accepted) can be taken separately;
   - the IPC Toolkit merge is a separate integration with the full suite and a
     smoke comparison;
   - the jse patch can be offered to json-spec-engine.

## Limits

* **Scene size.** The meshes are tiny: the spike is two tetrahedra.
* **Repetition.** One run per configuration (the ESP abort three times).
* **Timing.** In the first matrix, three runs shared four cores, so its wall
  times are indicative only. Section 4's timings are sequential.
* **Coverage.** No friction reference solution; Linux only; Houdini assets not
  run.

## Evidence

**Committed tools** in `tools/esp/`:
- `run_matrix.py` (standard library) builds every run's input from upstream's
  `esp_data/` or the fork's `scenes/semi-implicit/` and runs it;
- `summarize.py` writes the per-run tables;
- `tip_track.py` prints section 3's tip trajectories.

The last two need numpy and meshio to read the VTU outputs. The command lines
are in each run's `run.json`.

**Raw outputs** were kept in this cloud session's scratchpad and are not
committed. They are lost when the container is reclaimed:
- `esp-compare/20261025T230151Z/` (the matrix), `…-followup/`, `…-ldlt/`, with
  `summary.json`, and per-run `log.txt`, `jsons/input.json` and `out/`;
- `…-jse/`, the sequential timing set, with `timings.txt`;
- the overhead trace and the gdb stacks.

Rerunning `run_matrix.py` with the two binaries named in section 3 regenerates
them.
