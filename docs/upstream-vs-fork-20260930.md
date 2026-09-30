# Upstream or fork? Attribution of the 2026-09-30 CI findings

Date: 2026-09-30. Companion to [ci-cross-platform-findings-20260930.md](ci-cross-platform-findings-20260930.md)
and [parallel-edge-regression-20260930.md](parallel-edge-regression-20260930.md). Status: **done** (measured on
upstream builds, Release and Debug, where a claim needed it).

## Summary

* **Upstream defects** (present in upstream code, independent of the fork):
  - the Debug-only friction-Hessian segfault (toolkit's `PUBLIC EIGEN_DONT_VECTORIZE` against vectorized
    PolySolve) — reproduced on upstream's own Debug build and scene; upstream CI misses it only because its
    Debug lane runs no friction scene;
  - the Cramer 2x2 closest-point solve's accuracy loss for nearly parallel edges — upstream ipc-toolkit HEAD
    (not yet in upstream PolyFEM's pin);
  - the `parallel-edge` fragility (2/64 roundoff draws crawl past the 500-iteration limit on upstream code) and
    its runner-CPU dependence through MKL dispatch;
  - process-to-process non-determinism of GCP smooth contact (hash-seeded map order), harmless at the 1e-7 level.
* **Fork-specific** (caused by fork changes):
  - which `parallel-edge` draw fails: the fork's toolkit merge turned the AMD-path draw into a crawl
    (upstream passes on both paths today);
  - the whole `cube-on-floor` problem, including the macOS gap: fork commit `63e06378e`'s AL mass
    normalization; with only that line off, fork `main` reproduces upstream's reference to 5e-8;
  - the Windows restart-test failure (a fork-added test);
  - the AL-budget test slowdown (fork-only semi-implicit restart default);
  - the fork's Debug lane *reaching* the upstream segfault (its rollback scene tests are plain unit tests).

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
| 1 | `parallel-edge` crawl, 500-iteration limit on some Linux runners | **Yes — the fragility.** Upstream code crawls past 500 iterations in 2/64 one-ulp draws (same rate as the fork); scene, mesh, reference, solver settings and MKL dispatch are all upstream | **The failing draw.** Upstream solves the scene on both Intel and emulated-AMD dispatch today; the fork's toolkit merge (upstream toolkit SIMD code) re-rolled the AMD-path draw into a crawl | see §1 |
| 2 | `cube-on-floor` off by up to 6.5e-4 on macOS (and 0.4–0.7 % off upstream's reference on every fork build since at least 2026-09-13) | **No.** Upstream code passes upstream's reference (margin 1e-5) in 6/6 processes, deviation ≤ 8e-7, and one-ulp perturbations stay ≤ 8.4e-7 | **Yes: fork commit `63e06378e` (2026-07-02, "AL mass normalization")** divides the Dirichlet augmented-Lagrangian penalty metric by the mean lumped mass (here ×1/0.33205 = ×3.0116), which changes where the AL stage stops and makes the scene roundoff-sensitive (one-ulp spread 2e-4..3e-3). Switching only that line off makes fork `main` match upstream's reference to 5e-8 | see §2 (rewritten after the experiment) |
| 3 | Windows `restart from restart json` cleanup | No | **Yes** | the test was added by the fork (`7dd45a606`); upstream's only restart test (`"restart"`) keeps no file open at `remove_all` |
| 4 | Linux Debug SIGSEGV (Eigen `EIGEN_DONT_VECTORIZE` ODR mismatch) | **Yes, latent — reproduced on pure upstream**: upstream `591b08bd5`'s own `DebugNoSymbols` `PolyFEM_bin` segfaults (exit 139) on upstream's own 3D friction scene, same frame | Only fork tests *reach* it in CI: upstream's Debug CI runs no friction scene (scenes are hidden in Debug) | see §4 |
| 5 | Cramer `solve_spd_2x2` accuracy loss (near-parallel edges) | **Yes, in upstream ipc-toolkit HEAD** (`b778f64`, 2026-09-06, "Add SIMD batch support…"; `closest_point.hpp` identical upstream and fork). **Not** in upstream PolyFEM (its pin `b40e9c07` predates it and uses pivoted LDLT) | The Debug assertion was reached through the fork's semi-implicit stiffness path | see §5 |
| 6 | AL-budget test ~10x slower (`max_restarts` 20 → 200) | No | **Yes** | semi-implicit stall restarts and the default change (`e20ec8781`) exist only in the fork; upstream's input spec has no `semi_implicit` block |

## 1. parallel-edge — upstream fragility; the failing draw is the fork's

* The scene (`gcp-contact/parallel-edge/run.json`), its mesh (`tet-perp-edges.msh`) and stored reference
  (margin 1.2e-5) are byte-identical in upstream data `e0efb6b` and the fork's `5d76dcb`; the scene is in
  upstream's `tests/contact_3d.txt`. The AL mass normalization of §2 does not enter (the scene has no
  Dirichlet boundary).
* Measured on the upstream build:

| Build | native (Intel dispatch) | `qemu -cpu EPYC-Milan` (MKL's AMD path) | one-ulp height ensemble, native, limit lifted to 5000: draws over 500 iterations |
| --- | --- | --- | --- |
| upstream `591b08bd5` (toolkit `b40e9c07`) | solves, 56 at the hard step, deviation 2.8e-7 | **solves**, 359 / 56 | **2 / 64** (897, 2039) |
| fork `3c40ae557` (toolkit `75600955`, pre-merge) | solves, 56 | solves, 359 / 56 | 2 / 64 (897, 2112) |
| fork `6570e0410` / `7dd45a606` / `main` (toolkit merge) | solves, 56 | **500-iteration limit at step 40** | 1 / 64 (809) |
| fork `main`, `IPC_TOOLKIT_WITH_SIMD=OFF` | solves, 59 | solves, 358 / 55 | |

* So the crawl — a 2–3 % chance per roundoff draw that step 39 takes 800–2100 Newton iterations — is an
  **upstream property of the scene with upstream code** (GCP smooth contact, backtracking Newton with
  `use_psd_projection: false` from upstream's `gcp-contact/common.json`, the 500-iteration default). The
  runner-CPU dependence is also upstream infrastructure (PolySolve's MKL with `EIGEN_USE_MKL_ALL`, whose
  3x3 `DGETRF` kernel depends on the CPU vendor).
* Upstream is green on these runners only because its current draw does not crawl on either dispatch path.
  The fork's toolkit merge (SIMD geometry from upstream `ipc-sim/ipc-toolkit` `b778f64`…`869e489e`)
  re-rolled the AMD-path draw into a crawl. Upstream PolyFEM will face the same re-roll when it adopts
  the newer toolkit (or any other roundoff change); whether that particular draw crawls cannot be known
  in advance — the chance is ~3 % per dispatch path.
* Fork-specific part: only *which* draw the fork currently has. The remedies in the parallel-edge record
  (`MKL_CBWR=COMPATIBLE` in CI; a scene-level `max_iterations` or PSD projection) apply to upstream equally.

## 2. cube-on-floor — fork-specific (AL mass normalization, `63e06378e`)

My first reading (in the cross-platform record) called this a tolerance-limited *upstream* scene. The
upstream build shows that is wrong:

| Build (Release, one thread, `SimplicialLDLT`) | Reference compared against | max relative deviation (6 metrics) |
| --- | --- | --- |
| upstream `591b08bd5`, 6 separate processes | upstream (margin 1e-5) | 1.5e-8 … 8.0e-7 (passes every time) |
| upstream, Young's modulus + 1..12 ulp | upstream | 4.6e-9 … 8.4e-7 |
| fork `main`, AL mass normalization **on** (as shipped), 2 processes + 6 one-ulp draws | upstream | 3.9e-3 … 4.8e-3 |
| fork `main`, AL mass normalization **off** (only that line switched off), 2 processes + 6 one-ulp draws | upstream | 1.2e-8 … 2.2e-7 |
| fork `main` as shipped, + 1..23 ulp | fork's regenerated reference (margin 1e-3) | 2.4e-4 … 3.0e-3 |

Trace: the first solve of step 1 is the Dirichlet AL stage (the top face is pushed down 0.02). Its
initial energy and gradient are ×3.0116 in the fork (f₀ 475.32 vs 157.83, ‖∇f‖ 535 067 vs 177 668) with
*identical* Newton steps — the objective is uniformly scaled. 3.0116 = 1 / 0.3320479, the scene's average
nodal mass: fork `BCLagrangianForm::init_masked_lumped_mass` normalizes the penalty metric by its mean
diagonal (`masked_lumped_mass_ /= mean_diag`, introduced by fork commit `63e06378e`, 2026-07-02,
"Semi-implicit stiffness: unit fix, load-following trim control, AL mass normalization"); upstream uses the
raw lumped mass. Newton's stopping test is an absolute ‖∇f‖ tolerance, so the ×3 objective stops at a
different, later point (94 iterations with regularization in the fork against 26 upstream), and the
rest of the run follows from there. The fork's run lands 0.4 % from upstream's answer and, because
that AL stage ends on a regularized crawl, its end point depends on roundoff — hence the one-ulp spread,
the process-to-process spread CI-06 saw before the canonical order, and the macOS/Linux gap.

Upstream-side contribution: the `absl::Hash`-seeded `robin_map` iteration order in
`SmoothCollisionsBuilder` (upstream `b40e9c07`, `869e489e`) does make upstream non-deterministic across
processes, but only at the 1e-7 level on this scene (6 processes: 1.5e-8 … 8e-7), which the 1e-5 margin
absorbs. The fork's canonical-order commit (`f8dafef3`) is still worth upstreaming as a determinism fix.

Consequences: the fork's regenerated reference and 1e-3 margin (CI-06) paper over a fork behaviour
change. Options for the user: (a) keep the normalization and accept a sensitive scene (margin ≥ 5e-3), or
(b) decide whether `63e06378e`'s normalization should apply to scenes without semi-implicit contact
(it was introduced for the semi-implicit work), which would restore upstream's reference and margin.

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
* **Reproduced on pure upstream:** upstream `591b08bd5` built `DebugNoSymbols` exactly as its CI does (toolkit
  `b40e9c07`, PolySolve `a7727e33`, TBB, portable) crashes with SIGSEGV (exit 139) after 46 s on the same 10-step
  slope scene from upstream data, in `ipc::TangentialPotential::hessian` ← `Potential<TangentialCollisions>::hessian`
  ← `polyfem::solver::FrictionForm::second_derivative_unweighted` (gdb). The weak-symbol split is the same as the
  fork's: the vectorized `Eigen::internal::pstore<double, __vector(2)>` is defined 17 times in upstream PolySolve's
  archives and not at all in the toolkit's.
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

## Other differences between the data fork and upstream data

`sdast9/polyfem-data` `5d76dcb` differs from upstream `e0efb6b` in 9 files, all adaptations to fork behaviour,
none to an upstream defect: CI-03 (`e6ed5cf`) pins `friction_iterations: 1` on three historical fixtures and adds
current-default twins because the fork changed that default to 2 (the only changed pre-existing default under
`/solver` and `/contact` in `json-specs/input-spec.json`); CI-05 (`8e88613`, `d1c54f0`, `b7ae0d9`) sets an explicit
broad-phase budget (a fork-only `CCD/resource_limits` feature) and a friction-defaults twin for the microstructure
fixture; CI-06 (`72d3076`, `5d76dcb`) regenerates `cube-on-floor` and widens its margin (§2).

## Experiments on the upstream build

Upstream PolyFEM `591b08bd5` with its own pins (toolkit `b40e9c07`, PolySolve `a7727e33`, data `e0efb6b`),
Release, TBB, `POLYFEM_PORTABLE_BUILD=ON`, Triangle on, built in `wt-upstream` on the cloud host (Intel,
flags md5 `90d38fdb…`). Upstream has no `run_manifest_env` hook, so scenes are run with `PolyFEM_bin` exactly
as the harness would (`Eigen::SimplicialLDLT`, one thread) and the six printed metrics are compared with the
stored reference using the harness's normalisation (`tools/parallel-edge/direct_ref_check.py`; it reproduces
the fork harness bit for bit: deviation 0 on fork `main` against the fork's reference).

* `cube-on-floor`: see §2.
* `parallel-edge`, native: 3 processes, deviation 2.85e-7 each, hard step 56 iterations, no limit hit
  (same as the fork on this host).
