# Open items, decisions and questions — 2026-09-29

A reconciliation of the project records as of PolyFEM `main` at `2a619cd38`
(IPC Toolkit pin `f8dafef39e8`, PolySolve pin `43ca2e66`, data pin
`sdast9/polyfem-data@b7ae0d9`). It adds no evidence: every line points at the
record that owns the claim. When a record and this page disagree, the record
governs; fix this page.

## 1. Decisions waiting on the user

| # | Decision | What it blocks | Owner record |
| --- | --- | --- | --- |
| D1 | Agree the **accuracy standard** for trajectory-sensitive scenes (a pinned-trim reference ladder plus a production realization ensemble) and the **held-out scene set** | Any change to the default trim controller; standing condition of 2026-09-28 for the force-weighted band | [default-controller-assessment-20260929.md](default-controller-assessment-20260929.md) |
| D2 | Whether the **initial trim estimate alone** becomes the default, and its scope: `step` (re-arms every step) or `run` (needs `trim_seed_used_` persisted in the state file, a version-2 layout); whether to lower or cap the 4096× seed bound | Flipping `initial_trim_estimate`; the HDA default `si_initial_trim_estimate` and its test change with it | same, *Recommendation* 3 |
| D3 | Whether to pursue a **force-weighted redesign** (a target relative to the collapse threshold, no softening at stall retunes, or a cap on band moves per step) | Nothing today: the mode stays experimental | same, *Recommendation* 2; [ef-07-trim-loop.md](ef-07-trim-loop.md) |
| D4 | **Distribution licensing** for the Linux CLI package: GPL-2.0 SuiteSparse components (UMFPACK, SPQR, parts of CHOLMOD) and the Intel MKL redistribution notice; the glibc ≥ 2.38 and `-msse4.2` baselines were measured, not decided; which macOS/Windows targets to support | Wider distribution; CI-08 macOS/Windows work | [ci-08-validation.md](ci-08-validation.md) |
| D5 | Whether the `*-work/` evidence directories at the workspace root (`default-controller-work` is 51 GB) move to the Pitt share as `outputs/` did | Only disk hygiene | `AGENTS.md`, *Archived test outputs* |

Decided and closed (do not reopen without new evidence): production trim
controller stays `rms` with no estimate; force-weighted is experimental and
always runs with `collapse_guard_basis: pair`; `clamped_contacts` defaults to
`exclude_statistics` (2026-09-29); `gradient_balance_dofs` defaults to `all`
(2026-09-29); AL budget default off; stall-trigger default `absolute`; soft
budget method-independent; L-BFGS not pursued (EF-06 retired); automatic
timestep retry off (RB-08); mixed hexahedral orders refused (RB-23);
`friction_iterations: 2` with `realized_force` lag (RB-10); the CI-05 and
CI-06 reference policies (2026-09-29); the default stall-restart budget
`max_restarts` is 200 (2026-09-29, R3).

## 2. Open work

### R3 inflation (NeoHookean vs Ogden)

[r3-inflation-diagnosis-20260926.md](r3-inflation-diagnosis-20260926.md). The
NeoHookean run stops at step 39 because the snap-through past the pressure
maximum needs about **131 stall retunes** and the stall-retune budget the run
used was 50. With restarts on and the budget raised to 1,000
(`probe-nh-restarts1000`) step 39 passes (169.0 mL, 815 contacts), every later
step converges within the ordinary budget, and the 4 h cap ended the run at
step 168. **The user decided on 2026-09-29 to raise the default
`solver/contact/semi_implicit/restart/max_restarts` from 20 to 200** (scenes
that set it explicitly are unchanged); the change is PolyFEM `8bb9f936e` plus
the Houdini asset's parameter (default 200, range 0–500). Verify both are
published before relying on them, and when they are, update the `max_restarts`
value in [scenes/semi-implicit/README.md](../scenes/semi-implicit/README.md).
Still unestablished:

- whether the NeoHookean completes all 200 steps (the 10 h repeat,
  `probe-nh-restarts1000-long`, was started 2026-09-29);
- whether the Ogden run has its own pressure maximum without the obstacle;
- mesh independence of either path;
- a constrained eigenanalysis of the tangent at the limit point;
- the physical adequacy of either completed state (the snapped NeoHookean
  state has min J 0.016–0.021 and the membrane pressed on the obstacle; whether
  that is the intended physics is the user's question).

### Contact-efficiency plan

[contact-efficiency-plan-20260923.md](contact-efficiency-plan-20260923.md).
EF-01, EF-04, EF-02/03 and EF-07 are done; EF-05 was not pursued; EF-06 is
retired. What is left is the decision list above and the measurements it
needs: a current pinned-trim ladder on R4 (no current R4 ladder exists;
five-step R4 has one run per arm, which cannot separate accuracy from
realization noise), and a repeat of `pup_push` for steps 1–3.

### CI and portability

[ci-portability-plan.md](ci-portability-plan.md). CI-01, CI-02, CI-03, CI-05
and CI-06 are complete. Open:

- **Native acceptance — a first native result exists and is not green.**
  CI-04, CI-05 and CI-06 were validated on one cloud Linux GCC host. The only
  GitHub Build that covered them is [run 36576988097](https://github.com/sdast9/polyfem/actions/runs/36576988097)
  at `d53b9e444` (2026-09-29); every later push cancelled its successors, so no
  Build has completed since. Its job results, **none of them diagnosed yet**:
  Linux Release fails `contact_3d` (green natively at `0c129dcfb`, before the
  CI-04/05/06 changes); macOS Release fails `contact_2d` (the cube-on-floor
  reference regenerated on Linux is the obvious suspect; the cloud session
  planned to measure a platform gap and set that scene's margin from it);
  Linux DebugNoSymbols fails four rollback/AL-budget scene tests with a
  SEGFAULT; Windows Release fails the `restart from restart json` test;
  macOS and Windows Debug were cancelled. Reproduce each before treating any
  of them as a regression or as platform noise. (The `pre-commit` failure that
  accompanied these runs was formatting in `tests/test_trim_loop_guard.cpp`,
  repaired in `bd2f4db44`.)
- **CI-07:** item 2 (presets, pinned runner, cold-cache build), item 3
  (named common subset, expected counts, `[.][run]` gating), item 4 (portable
  Eigen lane; TBB/CPP decision), item 5 (bounded concurrency), item 6
  remainder (JUnit, configure metadata), item 7 (path filters), item 8 (IPC
  fork Windows lanes), item 9 (Windows half of the repeatability matrix; the
  Linux half is done), item 10 (IPC `-march=native` cache key).
- **CI-08:** macOS arm64/x86_64 and Windows packages. Linux x86_64 is done.
- **CI-09:** Houdini-side integration (path quoting in `write_params()`,
  launcher tests, packaged-solver discovery). The portable smoke runner is
  done.

### Verification gaps

- The last **complete** PolyFEM unit suite: 395 of 398 cases pass (92 minutes,
  2026-09-27; the three failures were the two known `verify_run` scenes,
  `gcp-contact/cube-on-floor` (CI-06) and `multi-material/stretch-cubes`
  (CI-04), and `shape-transient-friction`, whose forward tolerance was then
  tightened). A cloud Linux run on 2026-09-28 gave
  403/405 (`contact_2d`, `triangle_data`); CI-05/CI-06 then repaired both
  groups, confirmed locally on Linux only. No full suite has run
  since `clamped_contacts` became a default or since the Accelerate thread cap
  (`ba3ea76b6`); RB-02 was not rerun for the cap either. A full run needs a
  cap of at least two hours.
- `tools/ef02/sequence.py` `PRODUCTION_CONTROLLER` pins
  `clamped_contacts: "keep"`, stale since `eb8286b7c`. It matters only if a
  scene file sets the key; recorded, not changed.
- BFGS audit review follow-ups ([bfgs-convergence-audit-20260922.md](bfgs-convergence-audit-20260922.md)).

### Known limits (documented, not defects)

- Threaded friction runs reproduce only to about 1.7e-4 (RB-12); single-thread
  runs are bit-reproducible, including on macOS since `ba3ea76b6`.
- Two deterministic linear-solver realizations of IT differ by up to 23.5 %
  ([it-reproducibility-20260928.md](it-reproducibility-20260928.md)).
- The ν = .49999 cube needs 24 Newton iterations against a declared target of
  20 (RB-11 envelope stage).
- Physical accuracy beyond the recorded envelopes is **not** established.

### To investigate later: the `merge-cloud-branches` memory

On 2026-09-28 the "Cloud environment setup" session (transcript
`f5907031-1f9e-417a-bdcb-4fc96ad7c1c2`) had a fast-forward of `cloud/ci-04` into
`main` blocked by the auto-mode classifier ("Merge Without Review"), the user
replied "merge without asking", and the session then wrote a memory file
`merge-cloud-branches.md` (its `Write` succeeded) and tried to index it. That
index and `cloud-routines-20260928.md` edit was denied ("Self-Modification").
Today the file is no longer in the memory directory and the index has no entry.
A later attempt to recreate it was denied ("Instruction Poisoning"), and it was
deliberately **not** retried. Open questions: what removed the file, whether the
user wants a standing merge permission recorded at all and with what scope, and
how a durable record should be made without bypassing the classifier. Until that
is settled, treat merges of cloud branches as needing the user's go-ahead.

## 3. Repository state

- `polyfem` `main` = `origin/main` = `2a619cd38` (fast-forwarded 2026-09-29).
- The `cloud/*` branches on `origin` (`ci-04`, `ci-05-06-refs`, `ci-06-order`,
  `ci-08-linux`, `linux-evidence`, `locale-fix`) were cherry-picked onto `main`; `git cherry`
  finds no unmerged work except the cloud full-suite log, which `main` carries
  as `09e95bd83`. The branches are safe to delete.
- `ipc-toolkit-fork` and `polysolve-merged` match their `origin` branches.
- The HDA sources are not a repository; see `houdini_HDAs/AGENTS.md`.
