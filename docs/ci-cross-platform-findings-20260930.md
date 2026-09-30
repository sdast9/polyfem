# CI failures across platforms: likely sources (2026-09-30)

Written as a checkpoint so the work survives a lost session. Companion to
[parallel-edge-regression-20260930.md](parallel-edge-regression-20260930.md). Evidence: GitHub Build run
[36576988097](https://github.com/sdast9/polyfem/actions/runs/36576988097) (d53b9e444) job logs,
reproduced on a cloud Linux host (Ubuntu 24.04, GCC 13.3).

| # | Failure | Platform | Source (confidence) | Status |
| --- | --- | --- | --- | --- |
| 1 | `gcp-contact/parallel-edge` hits the 500-iteration limit | Linux Release, some runner CPUs | Chaotic crawl at step 39, selected by MKL run-time dispatch (AMD path); reproduced under `qemu -cpu EPYC-Milan` (high) | diagnosed; `MKL_CBWR=COMPATIBLE` adopted in the test lanes (section 6) |
| 2 | `gcp-contact/cube-on-floor` differs by up to 6.5e-4 | macOS arm64 Release | **Corrected 2026-09-30:** the fork's AL mass normalization (`63e06378e`) makes the scene roundoff-sensitive; upstream code is not (see [upstream-vs-fork-20260930.md](upstream-vs-fork-20260930.md) §2) (high, measured) | fork-specific; decision needed |
| 3 | `restart from restart json` (`test_restart.cpp:244`) | Windows Release | Test bug: `remove_all(outdir)` runs while the test's `std::ifstream` on `sim.pvd` is still open; Windows cannot delete open files (high, from the exception text) | fixed in this commit, unverified on Windows |
| 4 | Four rollback/AL-budget scene tests SIGSEGV at `test_step_rollback.cpp:407` | Linux DebugNoSymbols only | Eigen `EIGEN_DONT_VECTORIZE` ODR mismatch (3 tests, verified: pass with a uniform setting); the 4th then trips a real toolkit assertion in the new Cramer 2x2 solve (nearly parallel edges) | ODR: Debug-only uniform `EIGEN_DONT_VECTORIZE` adopted; 2x2 solve: fixed on toolkit branch `cloud/parallel-edge-fix`, pinned; AL-budget test hidden in Debug (section 6) |

## 2. macOS arm64: solver-tolerance-limited references

> **Correction (same day):** the measurements below are right, the attribution is not. On an upstream
> build the scene passes upstream's reference (1e-5 margin) with one-ulp spread ≤ 1e-6. The sensitivity
> comes from fork commit `63e06378e` (AL mass normalization, ×3.0116 on this scene's Dirichlet AL stage):
> with only that line switched off, fork `main` matches upstream's reference to 5e-8. Details in
> [upstream-vs-fork-20260930.md](upstream-vs-fork-20260930.md) §2.

`cube-on-floor` (2D, NeoHookean E=1e4, barrier stiffness 1e5, 20 steps) stores six metrics generated
with `solver/nonlinear/grad_norm_tol` 1e-7. On the cloud Linux host the harness reproduces the stored
values exactly (deviation 0). Changing Young's modulus by *one ulp* (1e4 + k·ulp, k = 1..23, no
other change) moves the metrics by 2.4e-4 to 3.0e-3 relative to the reference; 8 of the 23 draws
exceed the new margin of 1e-3. macOS's 6.52e-4 is inside that spread (arm64 clang contracts
multiply-adds into FMAs, x86 GCC here does not, and there is no MKL), so the macOS deviation is
platform roundoff, not a bug.

Why roundoff is amplified to 1e-3: the run stops when ‖∇f‖ < 1e-7, and the point where it stops
depends on the iteration path. With `grad_norm_tol` 1e-9 or 1e-11 the same 12 one-ulp draws agree
to ~1e-4 among themselves (deviations from the stored reference 5.88e-3..5.99e-3, spread 8e-5): the
scene *converges*, and the stored reference is 0.6 % away from the converged solution. So the
reference is tolerance-limited, not converged, and any platform or compiler difference shows up at
the level of the tolerance.

Consequences and options (user decisions; nothing changed in the tree):
* The 1e-3 margin set for CI-06 will fail intermittently on other platform/compiler/CPU
  combinations (35 % of the one-ulp draws on Linux alone). A margin of ~5e-3 would cover the
  measured spread; a principled alternative is to regenerate the reference with a converged
  tolerance (1e-9) and keep the tight default margin. That rewrites a stored reference, which the
  working rules forbid without a decision.
* The same holds for other contact scenes with loose tolerances: their references are only as
  reproducible as the tolerance.

## 3. Windows restart test

Log of the failing job (109435475785): both `GENERATE` sections pass all 29 checks and then fail on
the same line, 244 (`std::filesystem::remove_all(outdir)`), with
`remove_all: The process cannot access the file because it is being used by another process`.
In the test, `std::ifstream pvd_file(restart_outdir / "sim.pvd")` was declared at function scope
and still open at `remove_all`. Fix: read the PVD in an inner scope so the handle is closed first
(this commit). On Linux the test passes with the change (58 assertions). Not verified on Windows
(no Windows runner here); the exception text identifies the open handle but not which one, so if it
still fails on Windows the next suspects are an HDF5 state file kept open by `State` and the
`sim.pvd` writer.

## 4. Linux Debug rollback segfaults: Eigen ODR violation (reproduced locally)

Reproduced with a local `DebugNoSymbols` build (same flags as CI, `build-dbg`): the same four tests
crash with SIGSEGV; under gdb the first frame is
`Eigen::internal::pstore<double, double __vector(2)>` (an aligned SSE store), called from
`assign_op::assignPacket` <- ... <- `Eigen::Matrix<double,-1,1,0,12,1>` constructed from a product
<- `ipc::TangentialPotential::hessian` <- `Potential<TangentialCollisions>::assemble_hessian` (TBB
`parallel_for`) <- `polyfem::solver::FrictionForm::second_derivative_unweighted`. All four tests
use the friction scene (`friction_coefficient` .3), which is why only they crash.

Cause (high confidence): `ipc_toolkit` defines `EIGEN_DONT_VECTORIZE=1` **PUBLIC** when
`IPC_TOOLKIT_WITH_SIMD` is on (`CMakeLists.txt`, "Disable vectorization in Eigen since I've found it
to have alignment issues"; present at the old pin 75600955 too, which is why the crash predates the
merge). That definition reaches only the targets that link `ipc_toolkit` (the toolkit, PolyFEM, the
tests: 341 of 419 compile commands). PolySolve and the other dependencies do not link it, so they
compile Eigen vectorized. The mangled names of Eigen's template instantiations do not contain the
macro, so at `-O0` (no inlining) the same weak symbol exists twice, a scalar copy in the toolkit's
objects and a packet copy in `libpolysolve*.a`, and the linker keeps one. Here the packet copy wins
(the toolkit archive has no `pstore<double, __vector(2)>`; `libpolysolve_linear.a` and
`libpolysolve.a` define it 20 times), so the toolkit's friction Hessian, whose stack temporaries
are only 8-byte aligned in its non-vectorized layout, executes `movapd` on them and faults.
Release builds inline these functions and never resolve the symbol, so they run; the mismatch is
still there (a class with a `Vector2d`/`Vector4d` member has a different alignment and padding in the
two kinds of translation unit, so it is a latent ABI hazard in Release too, unobserved so far).

Options (a build-configuration decision for the user, because vectorization changes the summation
order of Eigen reductions and therefore re-rolls chaotic scenes such as parallel-edge and
cube-on-floor):
1. Make the setting uniform: `-DCMAKE_CXX_FLAGS=-DEIGEN_DONT_VECTORIZE=1` (or PolySolve's
   `-DEIGEN_DONT_VECTORIZE=ON`, which defines it on `Eigen3_Eigen`) so PolySolve matches. Release
   PolySolve would lose vectorization; results of Release runs can change at roundoff level.
2. Remove the toolkit's PUBLIC definition (make PolyFEM build the toolkit vectorized). The comment
   says the toolkit saw alignment problems, so this needs the toolkit's tests.
3. Keep Release as is and only make Debug consistent (option 1 in Debug lanes only), accepting that
   Debug and Release then run different arithmetic.
Verification (local Debug build `build-dbg2` with `-DEIGEN_DONT_VECTORIZE=1` on every translation
unit): the three friction rollback tests that segfaulted now pass ("A failed step attempt is rolled
back..." 1676 assertions, "A publication failure...", "A stall retune followed by a failure..."). Option 1
therefore removes the Debug segfaults.

### 4b. The fourth test then hits a real numerical assertion

With the ODR fixed, "An AL stage ended by its budget is rolled back..." aborts on a Debug-only
`assert` in the toolkit, `solve_spd_2x2` (`ipc/tangent/closest_point.hpp:203`), reached from
`ipc::semi_implicit_stiffness` -> `EdgeEdgeCandidate::compute_coefficients` -> `edge_edge_closest_point`.
Instrumented values of the failing call:

    A = [0.0678967655905255 1.10374224378703; 1.10374224378703 17.9426433062929]   det = 5.06e-7
    b = [0.620822036653334 10.0922008015598]
    Cramer residual = 4.29e-9, debug bound tol*scale = 2.02e-9

`det/(a00 a11)` = 4e-7, i.e. the edges are ~6e-4 rad from parallel and cond(A) ~ 4e7. Before the
toolkit merge this system was solved with Eigen's pivoted `A.ldlt().solve()` (asserted residual
< 1e-10); the merge replaced it by the branchless Cramer rule ("comparable accuracy" in the
comment), whose residual grows with cond(A). This is the only accuracy regression I found that
matches the parallel-edge brief ("a robust scalar path for near-parallel edges"): residual 4.3e-9
against 1.1e-16 for LDLT on the same system (`tools/parallel-edge/spd2x2_check.cpp`). Release
builds compile the assertion out, so the inexact `x` is used silently in
`semi_implicit_stiffness` and friction tangents.

Fix (toolkit branch `sdast9/ipc-toolkit:cloud/parallel-edge-fix`, dacf5ea7, patch also in
`tools/parallel-edge/toolkit-solve-spd-2x2-refinement.patch`): one step of iterative refinement
in `solve_spd_2x2`, applied lane-wise only where the relative residual exceeds 1e-12 (rounding-level
residuals are ~1e-16, so well-conditioned systems keep their bits). On the system above: residual
1.1e-16 (as LDLT), forward error 4.8e-10 against 3.2e-9 before. Checks on a Release build with the
fix: the five semi-implicit smoke scenes (single-threaded) are byte-identical to `main` (25 VTU, sim
files), and `parallel-edge` is bit-identical (the refinement never triggers there), so the fix does
**not** change the parallel-edge CI outcome; that is the crawl described in the other record. PolyFEM
branch `cloud/parallel-edge-fix` moves only the pin (main's pin is untouched).

## 5. A latent CI-time regression: `max_restarts` 20 -> 200

The AL-budget scene test drives the prescribed top face through the slab, which stalls the line
search; every stall triggers a barrier-stiffness retune and restart. Its last passing Linux Release
run took 304 s (run 36576988097) under `max_restarts` 20. Since the 2026-09-29 default change to 200
(e20ec8781) it needs ~10x more restarts (175 of 200 used after 20 minutes in Debug; over 10 minutes
without finishing in Release), so the next full CI run would hit its time budget with no code
defect. The test does not measure stall restarts, so it now pins `max_restarts` 20 (the value it
was written under): 2 min 22 s and 129 assertions pass in Release. The same holds for any other test
that relies on the old default; `test_al_solver.cpp:368` and `test_step_rollback.cpp:540` already
pin theirs.

## Open items after this pass

* Debug lane: the ODR mismatch needs a decision (options in section 4); the three friction tests
  pass with a uniform `EIGEN_DONT_VECTORIZE`; the AL-budget test now **passes in Debug** (129 assertions, ~1 h wall time
  in `build-dbg2` with the toolkit fix, the `max_restarts` pin and a uniform `EIGEN_DONT_VECTORIZE`), so
  expect that lane's wall time to grow once the segfaults stop ending these tests early.
* Windows: cannot be verified here; the cause is read from the exception text.
* macOS: the stored `cube-on-floor` reference is roundoff-sensitive because of the fork's AL mass
  normalization (section 2 correction); a decision on the normalization, or on margin
  versus regenerating with a converged tolerance is needed before the next native run.

## 6. Resolution (2026-09-30, cloud session; brief `tasks/ci-fixes-20260930.md`)

Adopted on branch `ci/fixes-20260930` (user decisions of 2026-09-30, see the brief). Host: Ubuntu 24.04,
GCC 13.3, 4 vCPU, Sapphire-Rapids-class Xeon; every PolyFEM run single-threaded (`OMP_NUM_THREADS=1`).

**What was adopted**

| Item | Change | Commit |
| --- | --- | --- |
| Workflow | Linux/Windows test steps `MKL_CBWR=COMPATIBLE`; `EIGEN_DONT_VECTORIZE=1` in every translation unit of the DebugNoSymbols lanes (section 4, option 1, Debug only); `cube-on-floor` margin 5e-3 (data pin `aed03ab`) | `05d7354be` (earlier) |
| Toolkit | `solve_spd_2x2` iterative refinement (section 4b), with a regression test, on `sdast9/ipc-toolkit` branch `cloud/parallel-edge-fix` = `1f1b5dbf` (`dacf5ea7` + test `8a353ad9` + a comment-only correction; `f8dafef3` = `main`'s pin is the parent of `dacf5ea7`); `cmake/recipes/ipc_toolkit.cmake` pins `1f1b5dbf` (first `8a353ad9`, `5c7a1288`) | toolkit `1f1b5dbf`; PolyFEM `5c7a1288` and the repin below |
| Debug runtime | "An AL stage ended by its budget is rolled back to the accepted state and recorded" (`test_step_rollback.cpp`) is registered in release builds only and hidden (`[.]`) in debug builds, as in `tests/verify_run.cpp` (`tagsrun`) and `test_diff.cpp`; its body is unchanged. Catch2 does not list hidden tests, so CTest does not register them (Debug: 391 tests; Release: 425) | `3495bb41` |

The toolkit's work branch `semi-implicit-stiffness` and `main` were not touched; fast-forwarding
`semi-implicit-stiffness` to `1f1b5dbf` is left to the user.

**Evidence**

1. *Toolkit suite* (`ipc_toolkit_tests`, Release, `ipc-toolkit-tests-data` `c7eba549`, `--rng-seed 1`, default
   selection: the 27 hidden benchmark/sweep cases are not part of it):

   | toolkit | SIMD flags | result |
   | --- | --- | --- |
   | `f8dafef3` | default (`-march=native`) | all pass: 346 cases, 5 685 663 assertions |
   | `8a353ad9` | default | all pass: 348 cases, 5 687 679 assertions (346 existing + the 2 new) |
   | `8a353ad9` | `-DSIMD_CXX_FLAGS=-msse4.2` (PolyFEM's portable build, no FMA) | all pass: 348 cases, 5 641 917 assertions |

   No test fails only with the fix. (A first run at the fix with a draft of the new test failed that one
   test, an over-strict forward-error check that I dropped: refinement brings the *residual* to rounding
   level, the forward error stays at cond(A) eps.) The new test
   (`[solve_spd_2x2]`, `tests/src/tests/tangent/test_closest_point.cpp`) uses the system from section 4b:
   at `f8dafef3` its relative residual is above the 1e-10 debug bound (1.4e-10 with hardware FMA; 2.1e-10
   without) and two assertions fail; at `8a353ad9` it is below 1e-15. It also checks that four fixed and 1000
   random well-conditioned Gram systems equal the unrefined Cramer result bit for bit.
2. *Release, new pin* (`tools/cloud/compile.sh`, toolkit `8a353ad9`; `1f1b5dbf` differs only in a test comment, data `aed03ab`):
   `MKL_CBWR=COMPATIBLE OMP_NUM_THREADS=1 ctest -R '^(contact_2d|contact_3d|triangle_data|standard)$'`:
   4/4 pass (557 s, 867 s, 1072 s, 470 s).
3. *Smoke scenes*: `tools/smoke/run_smoke.py --threads 1` on all five `scenes/semi-implicit` scenes exits 0
   with 0 error lines for the new pin and for a build at the previous pin (`f8dafef3`, commit `05d7354be`):
   the 55 output files (VTM/VTU/PVD) have identical SHA-256, and the logs are identical after masking
   timings and run ids. The refinement does not move them.
4. *parallel-edge under emulated AMD*: `tools/parallel-edge/run_qemu.sh EPYC-Milan` on
   `gcp-contact/parallel-edge` (scene copy with `Eigen::SimplicialLDLT`): with `MKL_CBWR=COMPATIBLE` it
   solves all 60 steps (362 iterations, hard step 58, no limit hit), log identical (timings masked) to the
   build at the previous toolkit pin; without the variable it stops at step 40 with
   `Reached iteration limit (limit=500)`, the CI symptom, so the emulation still reproduces it.
5. *DebugNoSymbols, CI flags* (`-DCMAKE_BUILD_TYPE=DebugNoSymbols -DCMAKE_CXX_FLAGS_DEBUGNOSYMBOLS=-DEIGEN_DONT_VECTORIZE=1
   -DPOLYFEM_THREADING=TBB -DPOLYFEM_WITH_TRIANGLE=ON -DPOLYFEM_PORTABLE_BUILD=ON`):
   `MKL_CBWR=COMPATIBLE OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=4 ctest`: **391/391 pass, 5399 s wall**.
   The three friction rollback tests that segfaulted pass (837 s, 211 s, 99 s). No test exceeds 1500 s; the
   longest are "linear elastic quasistatic schedule equals the static solve for every integrator" at
   1231 s (82 % of the limit, with four tests running in parallel on 4 vCPUs: a slower runner could push it
   over), "A failed step attempt is rolled back ..." 837 s, "Hexahedral interpolation converges at order
   q + 1" 787 s and "linear elastic transient forces follow the solved step for every integrator" 738 s.
   The `[.][run]` scene groups (`contact_2d`, `contact_3d`, `standard`, ...) are hidden in Debug by
   design, so the Debug lane does not run them.
6. *The hidden AL-budget test in Debug with the new pin* (explicit name, single process):
   a first run was killed after 75 minutes by a restart of my shell (289 stall restarts, no assertion or error
   other than the test's own expected stall messages); a complete rerun is listed below when finished.

**Left for the user**: fast-forward `sdast9/ipc-toolkit` `semi-implicit-stiffness` to `1f1b5dbf`; a completed
native GitHub Build of the branch (Linux lanes with `MKL_CBWR=COMPATIBLE`, Windows restart test, macOS
`cube-on-floor` at margin 5e-3); a CI run with `MKL_VERBOSE=1` would still confirm the inferred AMD CPU model.
The Windows Release lane hides the `[run]` scene groups (`tests/verify_run.cpp`), which this change does not alter;
the new AL-budget gating deliberately follows `NDEBUG` alone (no `WIN32` exclusion), so it runs in Windows
Release as before.
