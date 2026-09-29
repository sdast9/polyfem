# IT run-to-run irreproducibility — where identical runs diverge and why

Date: 2026-09-28. Follow-up to the open item of
[EF-07](ef-07-trim-loop.md#open) ("IT's run-to-run irreproducibility on this
host (single-threaded)"), also noted in [EF-04b](ef-04b-feasible-bound-retest.md).
**Status: cause found, fix decided and implemented (2026-09-28).** The user
chose option (a) plus (c) below: the binary caps Accelerate's threads at
`max_threads` whenever a limit is set, which makes `--max_threads 1` runs
bit-reproducible. See "Implemented fix" for the code and acceptance results.

## Result

* **Source.** On macOS the default linear solver is `Eigen::AccelerateLDLT`.
  Apple's Accelerate (vecLib) runs its sparse factorization with its own
  threads, and those threads do not honor `--max_threads`. That limit caps
  only `tbb::global_control` and Eigen. The multithreaded factorization is not
  bitwise deterministic, so every run's first solve (step 1, Newton iteration 1)
  ends at a different roundoff. The run manifest's statement that "the linear
  solver and the IPC toolkit follow it" (`process.threads.scope`,
  `src/polyfem/io/RunManifest.cpp:359`) is wrong for Accelerate.
* **Nothing else is nondeterministic in single-threaded runs.** With
  Accelerate limited to one thread (`VECLIB_MAXIMUM_THREADS=1`), or with the
  linear solver switched to CHOLMOD, two 200-step IT runs are bit-identical:
  all 201 solutions, iteration counts, trim predictors, solver attempts and
  balance flags. The other candidates listed for this investigation are ruled
  out for `--max_threads 1`: TBB, unordered-container or hash order, IPC
  Toolkit broad-phase reductions, timing or resource limits, uninitialised
  memory, and ASLR-dependent ordering. The four 200-step runs ran at the same
  time on a machine at load ≈ 24–30 (18 cores) and still matched.
* **Amplifier.** The roundoff seed (relative 8e-16) stays at roundoff
  through free fall (steps 1–11: 6.5e-16 → 2.4e-15). It becomes visible in the
  first stiff contact solve (step 12: 4.9e-5) because IT's contact steps stop
  on the directional-derivative (descent) test, not on the gradient tolerance.
  In EF-07's production repeats, 189 and 188 of 200 steps stop this way, at
  ‖∇f‖_rel with median 2.6e-5 and 3.8e-5 (maximum 8.2e-2 and 2.0e-2). Within
  that loose region the roundoff chooses the endpoint. Discrete branches then
  flip (projected-Newton fallback, soft-iteration restarts, trim moves), and
  the contact trajectory carries the difference to 1e-3 (step 20), 1e-2
  (step 27), 1e-1 (step 90) and at most 17.7 % (step 124).
* **The run can be made deterministic, but its result is not robust.** Two
  deterministic runs that differ only in their roundoff source (Accelerate
  with one thread vs CHOLMOD) differ by up to **23.5 %** in solution, need
  **6,385 vs 3,807** Newton iterations and have 51 of 200 balance flags
  different. A deterministic IT run fixes one sample from this spread. Any
  change that perturbs roundoff moves the result within the spread, and that
  includes a controller option that is "inert" to 1e-16. So even after the
  fix, a single pair of IT runs cannot isolate a controller effect (see
  [Consequences for acceptance](#consequences-for-acceptance)).

## Evidence

`/Users/stevenabramowitch/Downloads/fable_polyfem/it-repro-work/` is local
and not in the repository. The Pitt-share `outputs/` move is in progress,
so nothing was written to `outputs/`. It contains:

* `PolyFEM_bin-ef07-b`, sha256 `64d79a15…eeb68b`. This is EF-07's matrix
  binary (published `03ebea845`), byte-identical to the one that produced the
  EF-07 IT repeats.
* `runs/<label>/` from `tools/ef02/run.py --scene IT --threads 1` (the EF-07
  driver: `--max_threads 1`, `physical_diagnostics` on with the contact path
  off, coefficient events to `/dev/null`, solution-only VTU). The 200-step
  runs keep `vtu-sha256.txt` (hashes of all 201 step files), steps 0, 11, 12,
  13, 124 and 200, gzipped jsonl files and `run.log`.
* `runs/ef07-prod-r{1,2}` are symbolic links to the copied EF-07 production
  repeats in `ef07-work/matrix/runs/production-IT-s200-r{1,2}`.
* `cmp.py` (per-step relative L2 of the solution; uses
  `tools/qn_contact/compare_vtu.py`), `flags.py` (balance flags; differing
  jsonl keys), `iters.sh` (total Newton iterations from `run.log`; reproduces
  EF-07's 6,027 / 5,789) and `inject/` (the two injected libraries below).

### 1. First differing value (EF-07's production repeats)

The logs are identical up to their timings until **step 1, Newton iteration 1**.
The endpoint gradient norm is ‖∇f‖ = 2.41343e-13 in one run and 2.29944e-13 in
the other, with Δf identical to 6 digits. Every later step differs at the same
roundoff level until contact:

| step | rel. L2 solution difference, r1 vs r2 |
|---|---|
| 1 | 6.5e-16 |
| 11 | 2.4e-15 |
| 12 | 4.9e-5 |
| 20 | 1.3e-3 |
| 27 | > 1e-2 |
| 90 | > 1e-1 |
| 124 | 0.177 (maximum) |
| 200 | 0.109 |

At 4 significant digits the first logged difference is step 12, iteration 14
(‖Δx‖ 0.002876 vs 0.002877). The first discrete difference is step 13 after
iteration 29, where r1 takes the "not a descent direction → SparseProjectedNewton"
fallback and r2 does not. Soft-iteration restart stops: 14 vs 20.

### 2. Short repeats (3 steps, 6 s each)

Each configuration is repeated with identical input on the same binary. The
comparison is the sha256 of `step_1.vtu` / `step_4.vtu`.

| configuration | repeats | distinct results |
|---|---:|---:|
| default (`AccelerateLDLT`, Accelerate chooses its threads) | 6 | **6** (step 1 rel. 7.9e-16, max abs 2.3e-17) |
| `VECLIB_MAXIMUM_THREADS=1` (environment) | 6 | 1 |
| `VECLIB_MAXIMUM_THREADS=4` | 3 | 3 |
| `BLASSetThreading(BLAS_THREADING_SINGLE_THREADED)` on the main thread (injected constructor) | 3 | 3 |
| `setenv("VECLIB_MAXIMUM_THREADS","1")` in-process: injected constructor | 3 | 1 (= the environment result) |
| `setenv(...)` in-process at `main` (lldb breakpoint, after all library initialisers) | 3 | 1 (= the environment result) |
| `Eigen::SimplicialLDLT` | 2 | 1 |
| `Eigen::CholmodSupernodalLLT` | 2 | 1 |

What this shows:

* Accelerate is nondeterministic whenever it uses more than one thread; a
  fixed thread count > 1 is not enough.
* The per-thread `BLASSetThreading` API (macOS 15+) does not reach the sparse
  factorization's threads.
* vecLib reads `VECLIB_MAXIMUM_THREADS` lazily, so setting it inside the
  process before the first factorization works. A code fix does not need the
  user's shell environment.

### 3. 200-step repeats (identical settings, four runs at the same time)

| pair | step VTUs differing | Newton iterations | solves | balance flags false / differing | jsonl differences |
|---|---:|---|---:|---|---|
| EF-07 production r1 vs r2 (default) | 200 of 200 | 6,027 / 5,789 | 214 / 220 | 101 / 112; **45** differ | — |
| `VECLIB_MAXIMUM_THREADS=1` r1 vs r2 | **0** of 201 | 6,385 / 6,385 | 223 / 223 | 109 / 109; 0 differ | timings, RSS, `run_id` only |
| CHOLMOD supernodal r1 vs r2 | **0** of 201 | 3,807 / 3,807 | 201 / 201 | 110 / 110; 0 differ | timings, RSS, `run_id` only |

Spread between realizations (one sample each):

| pair | max rel. solution difference (step) | step 200 | balance flags differing |
|---|---|---|---:|
| EF-07 production r1 vs r2 | 17.7 % (124) | 10.9 % | 45 |
| VECLIB=1 vs CHOLMOD | 23.5 % (124) | 7.2 % | 51 |
| VECLIB=1 vs EF-07 production r1 | 33.7 % (140) | 16.1 % | 54 |
| VECLIB=1 vs EF-07 production r2 | 27.7 % (132) | 11.2 % | — |

In every pair the first step above 1e-9 is step 12. Iterations range from
3,807 to 6,385 across the four realizations. Most of that range comes from
soft-iteration restarts: 23 (VECLIB=1), 14 and 20 (EF-07), 1 (CHOLMOD).

### 4. Cost of single-threaded Accelerate (rough; host under load)

Linear-solve time per Newton iteration, projected fallback included:

| run | ms per iteration |
|---|---:|
| `VECLIB_MAXIMUM_THREADS=1` | 20.3 |
| EF-07 default | 15.7 and 11.0 |
| CHOLMOD | 13.6 |

Wall time per iteration: 0.136 s (VECLIB=1), 0.138 and 0.109 s (EF-07), 0.146 s
(CHOLMOD). The VECLIB=1 and CHOLMOD runs shared the machine with each other
and with other sessions' runs (load ≈ 24–30), so these numbers are not a
clean comparison. At IT's size (5,280 DOF) the linear solve is ≈ 11–15 % of
wall time. Single-threading it adds ≈ 5–9 ms per iteration here, ≈ 3–7 % of
wall time; a clean measurement belongs to the implementation step.
The cost for R4-sized scenes, which run multithreaded, is not affected by the
proposal below.

## Consequences for acceptance

* EF-07's statement stands, and is now explained: the IT repeat spread is a
  roundoff-realization spread, not a controller or measurement artefact.
* With the fix, repeats of the same binary and settings become identical,
  which makes regressions detectable (a byte-identity check on IT becomes
  possible, as for the five smokes). It does **not** make IT usable as a
  pairwise accuracy or iteration gate for changes that perturb roundoff. For
  such a gate, estimate the spread from several deterministic realizations,
  for example the Accelerate one-thread, CHOLMOD and SimplicialLDLT runs,
  optionally plus small seeded perturbations of the initial state. A
  candidate is then judged against that ensemble, not against one production
  run.
* The five public smokes' byte-identity is unaffected by this issue: their
  scenes use `Eigen::SimplicialLDLT`.
* The same mechanism (a loose descent-test stop plus an Accelerate roundoff
  seed) probably underlies R4's 4.7 % production repeat spread. That was not
  tested here: R4 runs multithreaded, where TBB reductions could also
  contribute.

## Proposed fix (decided: (a) and (c))

1. **Pin Accelerate's threads for single-threaded runs.** In
   `NThread::set_num_threads` (proposed there as `State::set_max_threads`; the
   former is what every entry point calls), on Apple builds,
   when `max_threads == 1` and `VECLIB_MAXIMUM_THREADS` is not already set:
   `setenv("VECLIB_MAXIMUM_THREADS", "1", /*overwrite=*/0)`. This runs before
   the first factorization (verified above: setting it at `main` works). It
   gives bit-reproducible `--max_threads 1` runs on this Mac.
2. **Make the manifest truthful.** Record Accelerate's thread setting in
   `process.threads` (the variable's value, or "Accelerate decides"). Correct
   the `scope` sentence: Accelerate follows the limit only through this
   setting, and only at one thread.
3. **Test.** A unit test that `set_max_threads(1)` leaves
   `VECLIB_MAXIMUM_THREADS == "1"` on Apple and respects a preset value. The
   acceptance check is two 3-step IT runs with byte-identical step files
   (6 s), plus the usual smokes, RB-02 and the HDA scripts.

Effect on results:

* `--max_threads 1` runs change once, from a random realization to a fixed one
  of the same distribution. Every single-threaded Accelerate trajectory that
  is past first contact (IT, the single-threaded EF matrix scenes) will
  therefore differ from the recorded evidence by the amounts in §3.
* Unaffected: the five public smokes (their scenes set
  `Eigen::SimplicialLDLT`, which is why they are already byte-identical run
  to run), every scene that names a non-Accelerate solver, and every
  non-Apple build.
* Multithreaded runs are unchanged.

Decision (user, 2026-09-28): (a) apply the fix to the binary, and (c) cap
Accelerate at `max_threads` for multithreaded runs too (that changes
performance, not determinism: 4 threads is still nondeterministic). Option (b),
a harness-only export in `tools/ef01/ef01_run.py`, was not chosen.

## Implemented fix

`NThread::set_num_threads` (`src/polyfem/utils/par_for.cpp`), which every
solver entry (`State`, legacy `State`, `OptState`) calls, now sets
`VECLIB_MAXIMUM_THREADS` on Apple builds before the first factorization:

* `max_threads > 0`: the variable is set to the effective thread count
  (1 for `--max_threads 1`, 4 for `--max_threads 4`).
* `max_threads <= 0` (unlimited, the default): the variable is not set and
  Accelerate decides, so default multithreaded runs behave as before.
* A `VECLIB_MAXIMUM_THREADS` the user exported is never overridden. The
  variable PolyFEM itself exported is replaced when the limit changes.
* Non-Apple builds are unchanged.

The run manifest records it in `process.threads.accelerate`
(`VECLIB_MAXIMUM_THREADS` value or null, and `source` = `max_threads` |
`environment` | `unlimited` | `not_applicable`), and the `scope` sentence now
says that Accelerate follows the limit through this variable and is bitwise
reproducible only at one thread. The manifest schema stays at version 1 (a
field was added, none changed). Test: `[run_manifest][threads]` in
`tests/test_run_manifest.cpp` (cap 1, cap 3, unlimited, preset value kept,
manifest fields, environment restored).

Acceptance (binary built from `5143c15a9` plus this change; the unrelated
upstream commits since then were not rebuilt):

| Check | Result |
| --- | --- |
| `unit_tests "[run_manifest]"` | 185 assertions in 8 cases pass; `[threads]` also with `VECLIB_MAXIMUM_THREADS=7` preset |
| IT 3 steps, no environment variable, two runs | step files byte-identical to each other and to the `VECLIB_MAXIMUM_THREADS=1` reference (`step_1` `2ca1e5fc`, `step_4` `12c55752`); the old binary without the variable gives `c9375856` |
| IT 200 steps, no environment variable, two runs at the same time | all 201 VTU files byte-identical; `step_1/12/124/200` equal the `VECLIB_MAXIMUM_THREADS=1` reference hashes |
| Five public smokes, `--max_threads 1` | exit 0, no `[error]` lines, all field outputs byte-identical to the pre-change binary (only `run-manifest.json`, which gains the new field and the binary hash, and timings in `run.log` differ) |
| `--max_threads 4` / `0` manifest | `accelerate` = "4"/`max_threads` and null/`unlimited` |
| 13 Houdini HDA scripts against the fixed binary | all exit 0 |

Not run: the full unit suite (about 2 h; the change is confined to thread
setup and the manifest) and the RB-02 probe (a standalone coefficient probe
that does not touch threading). Single-threaded Accelerate trajectories
recorded before this change (IT, EF matrix scenes) came from random
realizations and cannot be reproduced bit for bit; the spreads in §3 apply to
them.

Not proposed: changing the default linear solver (CHOLMOD is deterministic
and was faster here, but that is a solver-default change), or tightening the
descent-test stop, which is the amplifier (a model and solver behaviour
change; see the cube-on-floor "descent check at roundoff" lead).

## Reproduce

```bash
W=/Users/stevenabramowitch/Downloads/fable_polyfem; O=$W/it-repro-work
# nondeterministic (default): repeat and compare step_1.vtu hashes
python3 $W/polyfem/tools/ef02/run.py --workspace $W acc-s3-rN --scene IT --steps 3 --threads 1 --out $O --binary $O/PolyFEM_bin-ef07-b
# deterministic
VECLIB_MAXIMUM_THREADS=1 python3 $W/polyfem/tools/ef02/run.py --workspace $W veclib1-s3-rN --scene IT --steps 3 --threads 1 --out $O --binary $O/PolyFEM_bin-ef07-b
# per-step divergence of two runs
cd $O && python3 cmp.py RUN_A RUN_B 1,2,...,200
```
