# Upstream or fork? Attribution of the 2026-09-30 CI findings

Date: 2026-09-30. Companion to [ci-cross-platform-findings-20260930.md](ci-cross-platform-findings-20260930.md)
and [parallel-edge-regression-20260930.md](parallel-edge-regression-20260930.md). Status: **in progress**
(written as the work goes; the "Experiments on the upstream build" section is filled in when they finish).

## What "upstream" means here

| Component | Upstream reference point | Fork (this repository's `main`) |
| --- | --- | --- |
| PolyFEM | `polyfem/polyfem` HEAD `591b08bd5` (2026-09-23, "Add elastic material smoothing form") | `sdast9/polyfem` `main` |
| IPC Toolkit used by PolyFEM | upstream PolyFEM pins `ipc-sim/ipc-toolkit` **`b40e9c07`** | fork pins `sdast9/ipc-toolkit` `f8dafef3` (= upstream `869e489e` merged at `cf99893b` + fork commits) |
| IPC Toolkit HEAD | `ipc-sim/ipc-toolkit` `869e489e` | |
| PolySolve | upstream PolyFEM pins `polyfem/polysolve` `a7727e33` | fork `43ca2e66` (`iteration-callback`) |
| Test data | `polyfem/polyfem-data` `e0efb6b` | `sdast9/polyfem-data` `5d76dcb` (`fable-fixtures`) |

`b40e9c07` is exactly the merge base of the fork's toolkit merge: upstream PolyFEM has **not** adopted
the 21 upstream toolkit commits (SIMD geometry, Cramer 2x2 solve, MeshFEMSparse, LBVH) that the fork
merged on 2026-09-27. Upstream sources were read by anonymous `git fetch` of the public repositories.
Upstream's own GitHub Actions history could not be read from this session (only `sdast9/*` repositories
are attached), so every "does upstream fail" statement below comes from code comparison and local
reproduction, not from upstream CI logs.

Shared infrastructure (identical upstream and fork): the scene harness (`tests/verify_run.cpp` forces
`Eigen::SimplicialLDLT`, one thread; scene tests are `[run]` in Release and hidden `[.][run]` in Debug);
the CI matrix (Linux/macOS, `DebugNoSymbols` + `Release`, TBB, `POLYFEM_PORTABLE_BUILD=ON`,
`OMP_NUM_THREADS=1 ctest`); PolySolve's MKL setup (`POLYSOLVE_WITH_MKL` on except Apple Silicon,
`EIGEN_WITH_MKL` → `EIGEN_USE_MKL_ALL`); the toolkit's `PUBLIC EIGEN_DONT_VECTORIZE=1`.

## Attribution table

| # | Finding | Upstream? | Fork-specific? | Why |
| --- | --- | --- | --- | --- |
| 1 | `parallel-edge` crawl, 500-iteration limit on some Linux runners | **Mechanism upstream** (scene, mesh and reference byte-identical in upstream data; GCP smooth contact, MKL dispatch, harness all upstream) | **The failing draw is fork-specific** (the fork's toolkit merge re-rolled it for MKL's AMD path) | see §1; upstream build under emulated AMD pending |
| 2 | `cube-on-floor` off by up to 6.5e-4 on macOS | **Yes, and worse upstream**: scene tolerance-limited (upstream scene), and GCP smooth contact is run-to-run non-deterministic in upstream toolkit (`absl::Hash`-seeded `robin_map` iteration in `SmoothCollisionsBuilder`, present in `b40e9c07` and `869e489e`) | The fork *removed* the non-determinism (`f8dafef3` canonical order) and regenerated the reference with margin 1e-3; the residual macOS gap is platform roundoff on a tolerance-limited solve | see §2 |
| 3 | Windows `restart from restart json` cleanup | No | **Yes** | the test was added by the fork (`7dd45a606`); upstream's only restart test (`"restart"`) keeps no file open at `remove_all` |
| 4 | Linux Debug SIGSEGV (Eigen `EIGEN_DONT_VECTORIZE` ODR mismatch) | **Yes, latent**: reproduced with an **upstream scene** on the fork's Debug binary; crash site is upstream code identical in upstream's pin | Only fork tests *reach* it in CI: upstream's Debug CI runs no friction scene (scenes are hidden in Debug) | see §4 |
| 5 | Cramer `solve_spd_2x2` accuracy loss (near-parallel edges) | **Yes, in upstream ipc-toolkit HEAD** (`b778f64`, 2026-09-06, "Add SIMD batch support…"; `closest_point.hpp` identical upstream and fork). **Not** in upstream PolyFEM (its pin `b40e9c07` predates it and uses pivoted LDLT) | The Debug assertion was reached through the fork's semi-implicit stiffness path | see §5 |
| 6 | AL-budget test ~10x slower (`max_restarts` 20 → 200) | No | **Yes** | semi-implicit stall restarts and the default change (`e20ec8781`) exist only in the fork; upstream's input spec has no `semi_implicit` block |

## 1. parallel-edge

* The scene (`gcp-contact/parallel-edge/run.json`), its mesh (`tet-perp-edges.msh`) and stored reference
  (margin 1.2e-5) are byte-identical in upstream data `e0efb6b` and the fork's `5d76dcb`; the scene is in
  upstream's `tests/contact_3d.txt`.
* Everything that makes the scene fragile is upstream: GCP `SmoothContactForm`, the Newton/backtracking
  solver, `use_psd_projection: false` in `gcp-contact/common.json` (upstream data), and the
  MKL-routed 3x3 `DGETRF` calls whose kernel MKL picks by CPU vendor.
* What the fork changed: the toolkit merge (SIMD paths) moved roundoff so that the AMD-path draw of the
  fork crawls. On the pre-merge toolkit (which is ≈ upstream PolyFEM's pin for this code) the AMD path
  solves the scene. Expected consequence: upstream PolyFEM passes on all runners *today*, but would
  inherit the same 1–3 % chance of a crawling draw with any roundoff change, including adopting upstream
  toolkit `869e489e`. To be confirmed on the upstream build (native, one-ulp ensemble, emulated EPYC-Milan).

## 2. cube-on-floor

* Upstream's stored reference (`err_h1` 0.098813…, margin **1e-5**) is the one the fork replaced; CI-06
  measured the upstream-equivalent code (fork toolkit `cf99893b`, before the canonical-order commit)
  0.40–0.52 % away from it in every process, with 0.13 % spread *between processes of the same binary*
  ([ci-06-validation.md](ci-06-validation.md)). The source of that spread, `absl::Hash`'s per-process
  seed driving `tsl::robin_map` iteration order in `SmoothCollisionsBuilder`'s merge, is unchanged in
  upstream `b40e9c07` and `869e489e` (`ipc/utils/unordered_map_and_set.hpp`,
  `smooth_collisions_builder.cpp` lines 195–283). So upstream's `contact_2d` scene test can only pass
  by luck with a 1e-5 margin. To be confirmed on the upstream build (repeats against upstream's reference).
* Fork-specific: the canonical order (`f8dafef3`) made the scene deterministic per platform; the fork then
  regenerated the reference and set margin 1e-3. The remaining macOS/Linux gap (6.5e-4) is the
  tolerance-limited solve (`grad_norm_tol` 1e-7, an upstream scene setting) reacting to platform roundoff.

## 3. Windows restart test — fork only

`restart from restart json` (`tests/test_restart.cpp`) was added by fork commit `7dd45a606` ("Restart JSON
resumes the run it came from"). Upstream `591b08bd5`'s `test_restart.cpp` has only `TEST_CASE("restart")`,
which opens no stream at its `remove_all`.

## 4. Linux Debug segfault — upstream defect, reached by fork tests

* Upstream toolkit `b40e9c07` and `869e489e` both define `EIGEN_DONT_VECTORIZE=1` **PUBLIC** when SIMD is on;
  upstream PolySolve compiles Eigen vectorized. The mismatch is structural in upstream's build.
* The crash site, `TangentialPotential::hessian`'s 3D sliding branch
  (`T_aniso * ((scale * mu_f1_over_norm_u / (norm_u*norm_u)) * u_perp)`), is byte-identical in upstream's
  pin (`b40e9c07`, lines 301–305). The misaligned temporary is at a fixed offset in that frame, so every
  Debug evaluation of the branch faults, whoever calls it.
* **Reproduced with an upstream scene:** `contact/examples/3D/friction/high-school-physics-slopetest-mu=0.49.json`
  (upstream data; cut to 10 steps) run through `run_manifest_env` on the fork's ODR-affected Debug binary
  crashes with SIGSEGV after 54 s in `TangentialPotential::hessian` ← `FrictionForm::second_derivative_unweighted`
  (gdb). The fork's upstream-origin friction tests that run in Debug CI (`friction-contact`,
  `friction form derivatives 2d/3d`, `shape-transient-friction`) never reach the 3D sliding branch and pass.
* Why upstream CI does not see it: upstream's Debug lane runs no scene test (scenes are `[.][run]` in
  Debug), and none of its Debug unit tests evaluates a 3D sliding friction Hessian. The fork's four
  rollback/AL-budget scene tests are ordinary unit tests (not `[run]`), so the fork's Debug lane runs them.

## 5. Cramer 2x2 solve — upstream ipc-toolkit, not upstream PolyFEM

* Introduced upstream by `b778f64` (2026-09-06, PR #251). `ipc/tangent/closest_point.hpp` is identical in
  upstream `869e489e` and the fork's `f8dafef3`, so the fork inherited it unchanged; the fork's fix is
  `sdast9/ipc-toolkit` `cloud/parallel-edge-fix` `dacf5ea7`.
* Upstream PolyFEM's pin `b40e9c07` still solves the system with `A.ldlt().solve()`, so upstream PolyFEM is
  unaffected until it adopts a newer toolkit.
* Upstream callers of `edge_edge_closest_point` in `869e489e` include `EdgeEdgeCandidate::compute_coefficients`,
  `EdgeEdgeTangentialCollision` (friction tangent basis) and the GCP smooth-contact edge-edge distance, and
  `semi_implicit_stiffness` exists upstream in `ipc/barrier/adaptive_stiffness.cpp`. The inaccuracy is
  therefore reachable by any upstream toolkit user with nearly parallel edges; the fork reached the Debug
  assertion through its own semi-implicit barrier form, which upstream PolyFEM does not have.

## 6. max_restarts — fork only

Semi-implicit stall restarts (`/solver/contact/semi_implicit/restart/max_restarts`) and the 2026-09-29 default
change are fork features; upstream's `json-specs/input-spec.json` has no `semi_implicit` block.

## Experiments on the upstream build

Upstream PolyFEM `591b08bd5` with its own pins, Release, same flags as CI; results added below as they finish.
