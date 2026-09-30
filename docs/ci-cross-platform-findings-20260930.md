# CI failures across platforms: likely sources (2026-09-30)

Written as a checkpoint so the work survives a lost session. Companion to
[parallel-edge-regression-20260930.md](parallel-edge-regression-20260930.md). Evidence: GitHub Build run
[36576988097](https://github.com/sdast9/polyfem/actions/runs/36576988097) (d53b9e444) job logs,
reproduced on a cloud Linux host (Ubuntu 24.04, GCC 13.3).

| # | Failure | Platform | Source (confidence) | Status |
| --- | --- | --- | --- | --- |
| 1 | `gcp-contact/parallel-edge` hits the 500-iteration limit | Linux Release, some runner CPUs | Chaotic crawl at step 39, selected by MKL run-time dispatch (AMD path); reproduced under `qemu -cpu EPYC-Milan` (high) | diagnosed; CI env fix proposed |
| 2 | `gcp-contact/cube-on-floor` differs by up to 6.5e-4 | macOS arm64 Release | Stored metrics are limited by the solver tolerance (`grad_norm_tol` 1e-7), so any roundoff change moves them by 1e-4..3e-3 (high, measured on Linux) | margin 1e-3 is too tight; see below |
| 3 | `restart from restart json` (`test_restart.cpp:244`) | Windows Release | Test bug: `remove_all(outdir)` runs while the test's `std::ifstream` on `sim.pvd` is still open; Windows cannot delete open files (high, from the exception text) | fixed in this commit, unverified on Windows |
| 4 | Four rollback/AL-budget scene tests SIGSEGV at `test_step_rollback.cpp:407` | Linux DebugNoSymbols only | See the last section | in progress |

## 2. macOS arm64: solver-tolerance-limited references

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
See the result of the verification build at the end of this section.
