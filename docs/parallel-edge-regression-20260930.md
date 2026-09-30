# parallel-edge regression (contact_3d), bisect record — WORK IN PROGRESS

Date: 2026-09-30. Brief: [tasks/parallel-edge-bisect-20260930.md](tasks/parallel-edge-bisect-20260930.md).
Branch `cloud/parallel-edge`. This is the phase-1/2 checkpoint; the final record replaces it.

## Cloud host

* Session 1 host: Intel Xeon @ 2.80 GHz, AVX-512, flags-line md5 `9cfaedc8a90cafc6f531637e3a8c8418`.
* After a container restart (same session): Intel Xeon model 207 (Emerald Rapids), AVX-512 + AMX,
  md5 `90d38fdb848dfba6e140bdef814c10fc`. Neither matches a CI group (`0c21a325`, `9d95d5af`,
  `bf2d68ba` fail; `8e8b1783`, `1176f78c` pass). Ubuntu 24.04, GCC 13.3, glibc 2.39, MKL 2022.2.1.
* Every measurement below is deterministic on a host (12 concurrent repeats bit-identical).

## Proxy metric (tools/parallel-edge)

`step_metrics.py` reads a `--log_level debug` log: per solve the Newton iterations, the final
||grad|| and `Reached iteration limit` hits. The scene has one hard step, index 39 (log line
"Step 40"): 56 iterations at 3c40ae557 and at 6570e0410/7dd45a606/main on this host; the other 59
solves need 1..19. Totals: 358 (3c40ae557), 360 (6570e0410, 7dd45a606, main).

## Findings so far

1. On native Intel hosts every build (3c40ae557, 6570e0410, 7dd45a606, main, main with
   `IPC_TOOLKIT_WITH_SIMD=OFF`) solves the scene and authenticates.
2. MKL is entered only through 2 440 `DGETRF` calls on 3x3 matrices (`MKL_VERBOSE`); `libm`
   `log`/`pow` are called ~19 000 times and their FMA/non-FMA ifunc variants differ in the last
   bit, without changing the run. `MKL_CBWR`/`MKL_ENABLE_INSTRUCTIONS` change the trajectory
   (56..65 iterations at the hard step) but none fails on an Intel host.
3. Emulating the CPU with `qemu-x86_64 -cpu EPYC-Milan` (AMD vendor, so MKL takes its generic
   path) reproduces the failure: the 6570e0410, 7dd45a606 and main builds hit
   `Reached iteration limit (limit=500)` at step 40; the 3c40ae557 build solves all 60 steps
   (359 iterations). `-cpu Haswell` / `Skylake-Server` (Intel AVX2) pass for both.
   `MKL_CBWR=COMPATIBLE` under the AMD CPU passes (362 iterations, the same numbers as native
   COMPATIBLE). `tools/parallel-edge/run_qemu.sh` runs one CPU model.
