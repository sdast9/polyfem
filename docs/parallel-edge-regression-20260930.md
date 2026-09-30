# parallel-edge regression (contact_3d): bisect record

Date: 2026-09-30. Brief: [tasks/parallel-edge-bisect-20260930.md](tasks/parallel-edge-bisect-20260930.md).
Branch `cloud/parallel-edge`. Status: **cause established; the parallel-edge failure has no code fix** (decision boundary,
rule 3). A *different* accuracy regression of the merge (nearly parallel edge-edge closest-point
solve) was found afterwards while chasing the Linux Debug failures and is fixed on toolkit branch
`cloud/parallel-edge-fix`; it does not affect this scene. See
[ci-cross-platform-findings-20260930.md](ci-cross-platform-findings-20260930.md) section 4b.

## Result in five lines

1. The scene has one fragile time step (index 39, "Step 40" in the log). After an ordinary start
   it either converges in ~50-68 Newton iterations or *crawls* along a barrier wall at
   `‖∇f‖ ≈ 0.0164`, taking ~530-2100 iterations (measured with `max_iterations` 5000). The
   default limit is 500, so a crawl is a failure.
2. Which of the two happens is decided by 1e-16-level differences. 64 one-ulp geometry
   perturbations per toolkit, both toolkits, native Intel dispatch, limit lifted to 5000: pre-merge
   3c40ae557 needs 897 and 2112 iterations in two draws (and 143 in a third), post-merge 6570e0410
   809 in one; the other 61-62 draws need 49-68. The merge did not change how often the scene
   crawls.
3. The runner-CPU dependence is MKL's run-time dispatch. The only MKL routine the scene calls is
   `DGETRF` on 3x3 matrices (2 440 calls); 120 of them differ between the Intel default and the
   generic kernel, by at most 1.9e-16 relative. That is enough to select the trajectory.
4. Under `qemu-x86_64 -cpu EPYC-Milan` (AMD vendor, so MKL's non-Intel path) the post-merge
   builds 6570e0410, 7dd45a606 and `main` crawl (528 iterations with the limit lifted) and stop
   at `Reached iteration limit (limit=500)` at step 40, exactly the CI symptom; the pre-merge
   3c40ae557 build solves the scene. Intel AVX2 models solve it with every build.
5. The merge changed the outcome only by re-rolling this 1-3 % draw for the AMD path: with
   `-DIPC_TOOLKIT_WITH_SIMD=OFF` `main` passes under the same emulated CPU; disabling the
   MeshFEMSparse block assembly changes nothing. The smooth-contact gradient and Hessian match
   finite differences to ~1e-8 at every iterate of the failing run. (A separate near-parallel-edge
   accuracy loss in `solve_spd_2x2` exists but is not exercised by this scene; see the link above.)

## Established from GitHub CI logs (from the brief, not re-derived)

* Scene: GCP/SmoothContact, NeoHookean E=1e5, 60 steps, grad_norm_tol and rel_grad_norm_tol 1e-8,
  reference margin 1.2e-5. The harness forces `Eigen::SimplicialLDLT` and one thread.
* Linux Release passes through PolyFEM 3c40ae557; from 7dd45a606 it fails (only
  6570e0410 `Adopt IPC Toolkit upstream integration (cf99893b)` and 7dd45a606, a restart-JSON
  change, lie between). It fails on runners whose flags-line md5 starts `0c21a325`, `9d95d5af`,
  `bf2d68ba` and passes on `8e8b1783`, `1176f78c`. I read two more facts from the logs of
  runs 36347827022 (7dd45a606, CPU `0c21a325`) and 36576988097 (d53b9e444, CPU `0c21a325`):
  both fail the same way, `Reached iteration limit (limit=500)` after ~19 s of scene time; the
  d53b9e444 job also built from a cold cache (no stale objects).

## Work

### a) Cloud host

| | host 1 (start of the session) | host 2 (after a container restart) |
| --- | --- | --- |
| CPU | Intel Xeon @ 2.80 GHz, AVX-512 | Intel Xeon family 6 model 207, AVX-512 + AMX |
| flags md5 | `9cfaedc8a90cafc6f531637e3a8c8418` | `90d38fdb848dfba6e140bdef814c10fc` |
| CI group | none | none |

Ubuntu 24.04, GCC 13.3, glibc 2.39, MKL 2022.2.1 (static, TBB threading), 4 vCPU. All measurements
below were taken on host 2. A run is bit-reproducible: 12 concurrent repeats per build give
identical logs, and thread counts 1-4 (`taskset`) and padding the environment change nothing.

### b) Proxy metric, native host (Intel dispatch)

Tools: [`tools/parallel-edge`](../tools/parallel-edge). `run_direct.sh` runs `PolyFEM_bin`
single-threaded with `--log_level debug` on a scene copy with `Eigen::SimplicialLDLT`;
`step_metrics.py` extracts per solve the iterations, the final ‖∇f‖ and limit hits;
`run_harness.sh` / `harness_sweep.py` run `run_manifest_env` with a one-line manifest.

| build | toolkit | total iterations (60 solves) | hard step | harness | max relative deviation from stored reference |
| --- | --- | ---: | ---: | --- | ---: |
| 3c40ae557 | 75600955 | 358 | 56 | authenticated | 2.85e-7 |
| 6570e0410 | cf99893b | 360 | 56 | authenticated | 2.91e-7 |
| 7dd45a606 | cf99893b | 360 | 56 | bit-identical log to 6570e0410 | |
| `main` (db563f82 tree) | f8dafef3 | 360 | 56 | authenticated | 2.91e-7 |
| `main`, `IPC_TOOLKIT_WITH_SIMD=OFF` | f8dafef3 | 363 | 59 | authenticated | 2.95e-7 |
| `main`, `IPC_TOOLKIT_WITH_MESHFEM_SPARSE=OFF` | f8dafef3 | 360 | 56 | log bit-identical to `main` | |

The merge does not move the scene towards the 500 limit or the 1e-8 floor on this host; the
full `contact_3d` case (50 scenes, one process) also passes on 6570e0410.

### c) Run-time dispatch

* `MKL_VERBOSE=1`: the run makes 2 440 `DGETRF(3,3,...)` calls and no other MKL call. `libm`
  `log` (7 590 calls) and `pow` (11 382) are the only dispatched libm functions; their FMA and
  non-FMA `ifunc` variants differ in the last bit (`GLIBC_TUNABLES=glibc.cpu.hwcaps=-AVX2,-FMA`),
  but no run changes (identical logs), so libm is not the knob.
* MKL knobs on the native host (harness deviation from the stored reference; all authenticate
  except 3c40ae557 with AVX, 3.3e-5, and SIMD-off `main` with COMPATIBLE, 2.0e-5): `MKL_CBWR`
  COMPATIBLE 2.9e-7, SSE4_2 2.8e-7, AVX 2.9e-7, AVX2 1.6e-7, AVX512 2.9e-7 (6570e0410 values); `MKL_ENABLE_INSTRUCTIONS` gives the same values;
  `MKL_DEBUG_CPU_TYPE` has no effect. Hard step 56-65 iterations. None crawls: the Intel paths
  cannot reproduce the failure.
* Emulating the runner CPU with qemu user mode is faithful for the whole stack (an emulated
  Haswell run equals a native `MKL_CBWR=AVX2` run of the same binary iteration for iteration, an
  emulated COMPATIBLE run equals the native one). `tools/parallel-edge/run_qemu.sh`. TCG has no
  AVX-512, so AVX-512 models run as AVX2.

| emulated CPU | 3c40ae557 | 6570e0410 | 7dd45a606 | `main` | `main`, SIMD off | `main`, sparse off |
| --- | --- | --- | --- | --- | --- | --- |
| EPYC-Milan (AMD) | solves, 359 / 56 | **limit at step 40** | **limit** (log identical to 6570e0410) | **limit** (identical) | solves, 358 / 55 | **limit** |
| Haswell (Intel AVX2) | solves, 364 / 59 | solves, 364 / 59 | | solves, 364 / 59 | solves, 364 / 60 | |
| Skylake-Server (as AVX2) | | solves, 364 / 59 | | | | |

`MKL_CBWR=COMPATIBLE` makes the outcome vendor independent: 6570e0410 gives 362 / 58 iterations
and an identical log on EPYC-Milan, on Haswell and on the native host; `main` passes on
EPYC-Milan, 3c40ae557 on Skylake-Server with the same setting. `MKL_ENABLE_INSTRUCTIONS` and
`MKL_CBWR=AVX2/AUTO` do not rescue the AMD path (it ignores them).

With `max_iterations` 5000 the failing AMD trajectory finishes step 39 in 528 iterations: it is
the crawl, 5 % past the limit, not a divergence.

### d) Bisect

The brief's premise (the merge is responsible) does not survive b): the merge changes no
nominal quantity. The reproduction c) is bit-deterministic, so I bisected the available points
instead of the upstream range:

| point | AMD path |
| --- | --- |
| fork tip 75600955 (3c40ae557) | solves |
| merge cf99893b (6570e0410) | crawls |
| + restart JSON (7dd45a606) | crawls, log identical |
| + canonical smooth-contact order f8dafef3 (`main`) | crawls, log identical |
| `main`, SIMD off | solves |
| `main`, MeshFEMSparse off | crawls |

So among the merge's changes only the SIMD paths move the AMD draw. A finer bisect of the 21
upstream commits is not meaningful: upstream intermediate commits do not build against the
fork's PolyFEM (the API renames come with the merge), and, more to the point, the statistics
below show that *any* roundoff change re-rolls the outcome, so the first commit that flips a
single draw is arbitrary. The SIMD first differs from the scalar build already in step 0
(‖∇f‖ 1.69e-16 against 1.64e-16), where no contact is active.

### e) Mechanism

* Draw statistics (native host, `perturb.py`, one-ulp shifts of the dynamic tet's start height
  0.5, `max_iterations` 5000, 64 draws per build; hard step iterations): 3c40ae557: 61 draws in
  50-67, then 143, 897, 2112; 6570e0410: 62 draws in 49-68, then 809. With the harness at the
  default limit (`harness_sweep.py`, height ulps 8-71 and Young's modulus ulps 1-96, 160 draws
  per build): 3c40ae557 has 3 limit failures and 1 authentication deviation above the 1.2e-5
  margin, 6570e0410 has 1 limit failure and 8 deviations above the margin (Fisher p ≈ 0.16 for
  4 against 9 bad draws of 160: not distinguishable).
* The crawl: at the failing iterate the minimum barrier distance is 0.0099827 for dhat 0.01 and
  `barrier_stiffness` is 1e6. The Newton direction is ~1e-3 long but the line search accepts
  α = 2^-14 every iteration (Δf ≈ 7e-10 = α·|Δx·∇f|, the first-order prediction), so the
  iterate crawls through the barrier onset by ~1e-7 per iteration. `use_psd_projection` is false
  in `gcp-contact/common.json`, so the indefinite Hessian is used and the regularized fallback
  alternates with plain Newton; which branch it lands on is chaotic (the gradient jumps between
  0.02 and 1.5 within a few iterations in the passing trajectory too).
* Derivative check (`fdcheck.patch`, not part of the tree): at every Newton iterate of both the
  passing and the failing (emulated AMD) trajectory, the unweighted smooth-contact gradient
  agrees with finite differences of the value and the Hessian with finite differences of the
  gradient to ~1e-8 (best over h), with 2-7 active pairs. No inaccurate near-parallel-edge
  formula is involved, and the toolkit needs no robust scalar path.
* Scene variants on the failing AMD path (measurement only, no input changed in the tree):
  `use_psd_projection: true` 332 iterations / hard step 37; `barrier_stiffness` 1e5 300 / 33;
  `max_iterations` 5000 831 / 528.

## Recommendations (user decisions)

1. **CI environment:** set `MKL_CBWR=COMPATIBLE` for the test lanes (workflow `env`, or the CTest
   `ENVIRONMENT` property). It makes the MKL dispatch, and with it the scene trajectory, the
   same on every x86-64 runner (verified across three emulated CPUs and the native host), and
   the scene passes. It is not a model or tolerance change, and it does not need the reference
   regenerated (deviation 2.9e-7 against a 1.2e-5 margin). Cost: 2 440 DGETRF calls of ~15 µs on
   3x3 matrices per scene run. Caveat: it removes the CPU dependence, not
   the fragility; the next roundoff-changing commit re-rolls the draw once for all runners.
2. **Scene robustness:** the underlying fragility is the scene's, not the toolkit's. Options, each
   changes a stored input and needs a decision: raise this scene's `max_iterations` to 1000-2000
   (the crawl needs ~530-2100); or `use_psd_projection: true` (37 iterations on the failing
   path); or lower `barrier_stiffness`. I did not change any of them.
3. **Toolkit:** for this scene, no change: `IPC_TOOLKIT_WITH_SIMD` is not the defect (turning it off
   only re-rolls the draw). The toolkit branch `cloud/parallel-edge-fix` (dacf5ea7) fixes the
   unrelated `solve_spd_2x2` accuracy loss; adopting it is optional for this scene (bit-identical
   here).

## Open

* Native MKL `DGETRF` on the AMD path: replayed on the scene's 2 440 real inputs (dumped with
  gdb), the emulated AMD result equals `MKL_CBWR=COMPATIBLE` bit for bit (TBB and sequential
  layers), yet the whole-scene AMD trajectory differs from the COMPATIBLE one from the first
  iteration of step 0. I could not explain that residue (it may be another dispatched routine
  that `MKL_VERBOSE` does not list); it does not change the conclusions, since COMPATIBLE is
  measured directly.
* Emulation is not silicon: the CPU model behind flags hashes `0c21a325`, `9d95d5af`,
  `bf2d68ba` is inferred (AMD) from the reproduction, not read from a runner. A CI run with
  `MKL_CBWR=COMPATIBLE` and `MKL_VERBOSE=1` would confirm it.
* The other CI-only failures (Linux Debug rollback segfaults, Windows restart test) are not
  touched by this work.

## Files

`tools/parallel-edge/`: `run_direct.sh`, `run_harness.sh`, `run_qemu.sh`, `step_metrics.py`,
`perturb.py`, `harness_sweep.py`, `summarize_sweep.py`, `dgetrf_probe.c`, `dgetrf_replay.c`,
`fdcheck.patch`. Evidence stays in `outputs/parallel-edge/20260929T234546Z/` (not committed).
