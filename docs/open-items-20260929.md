# Open items, decisions and questions — 2026-09-29

A reconciliation of the project records, written 2026-09-29 and brought up to
date on 2026-09-30 (end of day) at PolyFEM `main` `0a367634a` (IPC Toolkit pin
`1f1b5dbf`, PolySolve pin `43ca2e66`, data pin `sdast9/polyfem-data@aed03ab`);
the canonical-pair-keys and mechanisms B/C entries were added on 2026-10-05, the
first-contact refresh entries on 2026-10-06. It adds no evidence: every line points at the
record that owns the claim. When a record and this page disagree, the record
governs; fix this page.

## 1. Decisions waiting on the user

| # | Decision | What it blocks | Owner record |
| --- | --- | --- | --- |
| D1 | Agree the **accuracy standard** for trajectory-sensitive scenes (a pinned-trim reference ladder plus a production realization ensemble) and the **held-out scene set** | Any change to the default trim controller; standing condition of 2026-09-28 for the force-weighted band | [default-controller-assessment-20260929.md](default-controller-assessment-20260929.md) |
| D2 | ~~Whether the initial trim estimate alone becomes the default, and its scope~~ **Decided 2026-10-02: no default change** (`initial_trim_estimate` stays off in PolyFEM and the asset; R4's scene opts in, switched to `rms` + estimate); scope `run` preferred if adopted later; E5 (scope `run` lost on restart) repaired with restart layout version 2. **Still open:** whether to lower or cap the 4096× seed bound (recommendation: keep it downward, R4 binds there) | Only the bound question; a default flip waits on D1 | [default-controller-assessment-20260929.md](default-controller-assessment-20260929.md), *Decision on D2* |
| D3 | Whether to pursue a **force-weighted redesign** (a target relative to the collapse threshold, no softening at stall retunes, or a cap on band moves per step) | Nothing today: the mode stays experimental | same, *Recommendation* 2; [ef-07-trim-loop.md](ef-07-trim-loop.md) |
| D4 | **Distribution licensing** for the Linux CLI package: GPL-2.0 SuiteSparse components (UMFPACK, SPQR, parts of CHOLMOD) and the Intel MKL redistribution notice; the glibc ≥ 2.38 and `-msse4.2` baselines were measured, not decided; which macOS/Windows targets to support | Wider distribution; CI-08 macOS/Windows work | [ci-08-validation.md](ci-08-validation.md) |
| D5 | ~~Whether the `*-work/` evidence directories move to the Pitt share~~ **Done 2026-09-29 for the finished ones** (39,347 files, 108 GB, SHA-256-verified, log `work-evidence-transfer-20260929.log`; isolated builds deleted). Still local and undecided: `test_cases/*/output` (about 580 GB of Houdini output; the `.hipnc` scenes reference `/output` paths), `r3-ogden-work`, `retune-default-work` | Only disk hygiene | `AGENTS.md`, *Archived test outputs* |
| D6 | What to do about the `gcp-contact/parallel-edge` scene's own fragility: raise `max_iterations`, `use_psd_projection`, or lower `barrier_stiffness`. The CI side is handled (`MKL_CBWR=COMPATIBLE`), the scene is not | Nothing in CI; robustness of that scene on other CPUs | [parallel-edge-regression-20260930.md](parallel-edge-regression-20260930.md) |
| D7 | Whether to report the upstream defects found 2026-09-30 (Debug friction-Hessian segfault, the nearly-parallel-edge 2x2 solve, `parallel-edge` fragility) to the upstream projects | Nothing local | [upstream-vs-fork-20260930.md](upstream-vs-fork-20260930.md) |
| D8 | ~~Whether small-step stall restarts apply in every mode~~ **Decided 2026-10-01: on by default** (`solver/advanced/stall_restart`: `feasible_bound` trigger, soft limit off, forced-PSD remedy, at most 20 restarts, kept for now by the user and revisited if real-world scenes hit it; semi-implicit stays at 200) | — | [stall-recovery-generalization-20261001.md](stall-recovery-generalization-20261001.md) |

Decided and closed (do not reopen without new evidence): production trim
controller stays `rms` with no estimate; force-weighted is experimental and
always runs with `collapse_guard_basis: pair`; `clamped_contacts` defaults to
`exclude_statistics` (2026-09-29); `gradient_balance_dofs` defaults to `all`
(2026-09-29); AL budget default off; stall-trigger default `absolute`; soft
budget method-independent; L-BFGS not pursued (EF-06 retired); automatic
timestep retry off (RB-08); mixed hexahedral orders refused (RB-23);
`friction_iterations: 2` with `realized_force` lag (RB-10); the CI-05 and
CI-06 reference policies (2026-09-29); the default stall-restart budget
`max_restarts` is 200 (2026-09-29, R3); a line search that fails on every strategy restarts the subsolve once with a fresh solver in every mode without stall restarts (`solver/advanced/line_search_failure_restarts` 1, 2026-10-01, [record](stall-recovery-generalization-20261001.md)); semi-implicit edge-edge / vertex-vertex coefficients are keyed on the sorted pair, **without a historical switch**, and the run manifest reports per step what depended on discrete history (2026-10-05, options 1 and 4 of the roundoff investigation; options 2, 3 and 5 — robust degenerate estimates, a state-based trim cadence, continuation hysteresis — not chosen; [record](canonical-pair-keys-20261005.md)); after mechanisms B and C were measured on the user's scenes, the 30-iteration in-solve trim cadence stays as it is and stale first-estimate stamps are not repaired, while the guarded first-contact refresh (*birth0g*) is adopted as production behaviour **without a historical setting** (2026-10-05 and 2026-10-06, [record](semi-implicit-mechanisms-b-c-20261005.md), [adoption](first-contact-refresh-20261006.md)).

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
that set it explicitly are unchanged); the change is PolyFEM `e20ec8781` plus
the Houdini asset's parameter (default 200, range 0–500), both published
2026-09-29 with the README defaults block updated. The R3 scene
(`kristin_sim.hipnc`) sets 50 explicitly and keeps it until the scene is
changed. Still unestablished:

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

### Semi-implicit roundoff sensitivity (2026-10-04/05)

[canonical-pair-keys-20261005.md](canonical-pair-keys-20261005.md) and
[semi-implicit-mechanisms-b-c-20261005.md](semi-implicit-mechanisms-b-c-20261005.md)
(evidence: the parent workspace's `semi-implicit-roundoff-work/FINDINGS.md`,
`canonical-keys-work/` and `mechanism-bc-work/`). Mechanism A (pair-key orientation) is removed and the
manifest reports continuation losses, split pair identities, trim moves by
source and gap-shift estimates per step. Open:

- the Read PVD asset does not show the report yet (follow-up; the asset was
  not touched);
- mechanisms B and C were measured on the user's scenes (2026-10-05): the
  repro's degenerate first estimates do not occur there; stale first-estimate
  stamps are frequent but without a measurable effect and are not repaired, and
  the cadence stays (user decisions). The guarded first-contact refresh
  (*birth0g*) is adopted (2026-10-06, local branch `adopt-birth0g` on top of
  the B/C record's branch `semi-implicit-bc-record`;
  [record](first-contact-refresh-20261006.md)): the adopted build reproduces
  the measured prototype's runs byte for byte, RB-02 270/270, new smoke
  references. The moving-obstacle check on ball-burst (ball head as an
  obstacle) cannot exercise it: the knit membrane is in self-contact from
  t = 0, so no snapshot is ever contact-free. The probe branch
  `mechanism-bc-probe` stays local for reference;
- canonical keys, the B/C record and *birth0g* were pushed together on
  2026-10-06 (fast-forward `49ad24b74..db2a8bb12`, user decision); the shared
  `polyfem/` checkout was fast-forwarded and `polyfem/build` rebuilt at
  `db2a8bb12` ([publication](first-contact-refresh-20261006.md#publication)).

### Gmsh input (found 2026-10-03)

Nothing open. The malformed-number hang is repaired ([RB-11 record](rb-11-validation.md#gmsh-files-with-malformed-numbers-stop-by-name-instead-of-hanging-2026-10-03)).
The defect found there is repaired the same day ([record](rb-11-validation.md#gmsh-parametric-node-blocks-keep-their-positions-2026-10-03)):
`MshReader` read a 4.1 parametric node block (`parametric` 1, written by Gmsh
only with `Mesh.SaveParametric`) with a stride of 3, so such a file loaded
with wrong vertex positions, silently or as a misleading `element N is
flipped`; each block is now read with its own stride. No mesh on disk outside
MshIO's own test data had such a block.

### CI and portability

[ci-portability-plan.md](ci-portability-plan.md). CI-01, CI-02, CI-03, CI-05
and CI-06 are complete. Open:

- **Native acceptance — diagnosed and green (2026-09-30).** [Build run
  36755168744](https://github.com/sdast9/polyfem/actions/runs/36755168744) at
  `3495bb41e` passed **all six lanes** (Linux, macOS and Windows; Debug and
  Release), the first completed fully green native Build since the CI-04/05/06
  merges. The Build on the merge commit `0a367634a` (run 36799322242) passed Linux
  Release and macOS Release and was then **cancelled** by the next push to `main`
  (every push cancels the running Build), so a completed Build of the merged
  head is still wanted: it needs about 2.5 hours without a push to `main`, or a
  manual run on a fixed commit. Caveats that still apply: Windows does not run the `[run]` scene
  groups, and the Debug lanes hide the AL-budget rollback test (about an hour
  in Debug). What was diagnosed, for the record: CI-04, CI-05
  and CI-06 were validated on one cloud Linux GCC host; the GitHub Builds that
  covered them are [run 36576988097](https://github.com/sdast9/polyfem/actions/runs/36576988097)
  (`d53b9e444`) and the partly completed [run 36609059244](https://github.com/sdast9/polyfem/actions/runs/36609059244)
  (`9051aacb1`); later pushes cancelled every other run. Diagnosis from their
  logs and earlier runs:
  - **CI-05 microstructure pair:** green on Linux and macOS Release.
  - **CI-06 cube-on-floor:** green on Linux Release; macOS Release is
    deterministic but differs from the Linux reference by up to 6.52e-4.
    The scene is roundoff-sensitive because of the fork's AL mass
    normalization (one-ulp spread up to 3e-3); margin now `5e-3` (data
    `aed03ab`, user decision 2026-09-30); green in Build 36755168744.
  - **Linux Release `contact_3d`:** `gcp-contact/parallel-edge/run.json` hits
    the 500-iteration Newton limit on GitHub Linux only (macOS and the cloud
    Linux host solve it). It already failed at `5143c15a9` (run 36485520601),
    before the canonical-order toolkit pin, so the regression lies between
    `0c129dcfb` (green, 2026-09-21) and `5143c15a9`. **Diagnosed 2026-09-30
    ([record](parallel-edge-regression-20260930.md)):** step 39 is a chaotic
    barrier-wall crawl (56 iterations or 500-2100 depending on ulp-level noise,
    1-3 % of draws for the old and the new toolkit alike); MKL's AMD dispatch
    (`DGETRF`) gives the post-merge build the crawling draw (reproduced under
    `qemu -cpu EPYC-Milan`); `MKL_CBWR=COMPATIBLE` in the test lanes fixes it
    on every CPU. **Adopted 2026-09-30 on branch `ci/fixes-20260930`** (verified
    under emulated EPYC-Milan: 60 steps, no limit hit;
    [resolution](ci-cross-platform-findings-20260930.md#6-resolution-2026-09-30-cloud-session-brief-tasksci-fixes-20260930md));
    green in the completed native Build 36755168744 (all lanes). The scene's own
    fragility (raise `max_iterations`, `use_psd_projection`, lower
    `barrier_stiffness`) stays a user decision. Related: Linux Debug
    segfaults = Eigen `EIGEN_DONT_VECTORIZE` ODR mismatch, Windows restart =
    test hygiene (fixed), macOS deviation = tolerance-limited reference, all in
    [ci-cross-platform-findings-20260930.md](ci-cross-platform-findings-20260930.md); which of these are
    upstream and which fork-specific: [upstream-vs-fork-20260930.md](upstream-vs-fork-20260930.md)
    (macOS cube-on-floor corrected there: fork AL mass normalization `63e06378e`).
  - **Linux DebugNoSymbols:** four rollback/AL-budget scene tests SEGFAULT
    (`test_step_rollback.cpp:407`); already known before CI-04–06 (the
    2026-09-20 golden plan records the same four). **Fixed 2026-09-30 on
    branch `ci/fixes-20260930`; Debug green in Build 36755168744:** uniform
    `EIGEN_DONT_VECTORIZE=1` in the Debug lanes (the ODR mismatch); the toolkit
    `solve_spd_2x2` refinement (toolkit `1f1b5dbf`, pinned) removes the Debug
    assertion the fourth test then hit; that test (about an hour in Debug) is
    now hidden in Debug builds so the lane stays under CTest's 1500 s
    per-test limit. Local DebugNoSymbols run with the CI flags: 391/391 pass,
    longest test 1231 s
    ([resolution](ci-cross-platform-findings-20260930.md#6-resolution-2026-09-30-cloud-session-brief-tasksci-fixes-20260930md)).
    Merged to `main` 2026-10-01; the toolkit's `semi-implicit-stiffness` was
    fast-forwarded to `1f1b5dbf` the same day.
  - **Windows Release:** `restart from restart json` (`test_restart.cpp:244`)
    failed in runs 36485520601 and 36524461889, before the CI-05/06 changes;
    test hygiene (`sim.pvd` was still open when the directory was removed),
    **fixed in `2de3553ca`**, green in Build 36755168744. Windows does not run
    the `[run]` scene groups.
  (The `pre-commit` failure that accompanied these runs was formatting in
  `tests/test_trim_loop_guard.cpp`, repaired in `bd2f4db44`.) Every push to
  `main` cancels the running Build; a completed native result needs a quiet
  period of about 2.5 hours or a manual run on a fixed commit.
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
  groups. Since then the CTest-registered suite has run green on all six native
  lanes (Build 36755168744; the resolution record counts 425 registered tests in
  Release and 391 in Debug, where the AL-budget rollback test is hidden), which covers the `clamped_contacts` default and the
  Accelerate thread cap on Linux, macOS and Windows within those exclusions.
  Not rerun since the defaults changed (`clamped_contacts`, `max_restarts` 200)
  or the toolkit's `solve_spd_2x2` refinement: the RB-02 probe (`tools/rb02`),
  which the repository's own rule says to rerun after any default change.
  (`test_step_rollback.cpp` pins `max_restarts: 20` in the AL-budget test,
  because the new default made it about ten times slower.)
- Canonical pair keys (2026-10-05): the affected unit selection, the five
  smokes, the repro sweep, the RB-02 probe (270/270) and before/after runs of
  IT, pup_push, R4, R1, BBT and ball-burst were run; the full unit suite and
  the Houdini asset tests were not.
- First-contact refresh (2026-10-06): byte comparisons with the measured
  prototype and with production (repro family, five smokes, R1, BBT, IT,
  pup_push), the affected unit selection, the full CTest suite (438/438), the RB-02 and RB-10
  probes and the ball-burst obstacle check were run; after the push, with the
  rebuilt shared binary, the five smokes are byte-identical to the new
  references and all 25 test scripts of the published Houdini assets
  (`sdast9/houdini-plugins@6591c3b`) pass.
- RB-09 spring probe (2026-10-06): it failed T11 at `k` = 100, `d̂` = .01 after
  *birth0g*, which moved the trim path (a first-contact refresh no longer bumps
  the trim for a collapse); the failure was the probe's own Newton (a rounding
  stall, an extra `post_step`), not the form. Repaired in `tools/rb09` (PolySolve's
  `post_step` order, Armijo's roundoff fallback, a named convergence check):
  25/25 on the adopted law and on the 2026-09-13 law. Residual: in a roundoff
  perturbation stress test 40 of 3 000 runs still hit the iteration cap while
  T11's numbers pass (the probe's stopping tolerance sits at the gradient's
  roundoff floor); not changed. The record's spring rows are re-measured
  ([rb-09-validation.md](rb-09-validation.md), *Update 2026-10-06*).
- `tools/ef02/sequence.py` `PRODUCTION_CONTROLLER` pins
  `clamped_contacts: "keep"`, stale since `eb8286b7c`. It matters only if a
  scene file sets the key; recorded, not changed.
- BFGS audit review follow-ups ([bfgs-convergence-audit-20260922.md](bfgs-convergence-audit-20260922.md)).

### Attribution of the 2026-09-30 CI findings

[upstream-vs-fork-20260930.md](upstream-vs-fork-20260930.md) measured which
findings are upstream's. **Upstream defects:** the Debug friction-Hessian segfault
(toolkit `PUBLIC EIGEN_DONT_VECTORIZE` against vectorized PolySolve; reproduced on
upstream's own Debug build), the Cramer 2x2 closest-point accuracy loss for nearly
parallel edges, the `parallel-edge` fragility (2 of 64 roundoff draws crawl past the
500-iteration limit on upstream code), and hash-seeded non-determinism of GCP smooth
contact. **Fork-specific:** which `parallel-edge` draw fails (the toolkit merge), the
whole `cube-on-floor` problem (the AL mass normalization `63e06378e`), the Windows
restart test, and the AL-budget test slowdown (the `max_restarts` default). I found no
record saying whether the upstream defects were reported to `polyfem/polyfem` or
`ipc-sim/ipc-toolkit`; that is a decision for you (D7 below).

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
**What removed it is known:** the same session deleted it with `rm` about 40
seconds after the denial (the command's description was "Remove the memory file
recording the merge permission"); it was not moved to the shared drive, and no
copy exists under `~/.claude` or the share. A later attempt to recreate it was
denied ("Instruction Poisoning"), and it was deliberately **not** retried. Open
questions: why the session removed it (its reasoning is not in the transcript),
whether the user wants a standing merge permission recorded at all and with what
scope, and how a durable record should be made without bypassing the
classifier. Until that
is settled, treat merges of cloud branches as needing the user's go-ahead.

## 3. Repository state

- `polyfem` `main` = `origin/main` = `db2a8bb12` (fast-forwarded 2026-10-06:
  canonical pair keys, the B/C record and the first-contact refresh; before
  that `49ad24b74`, and on 2026-09-30 `0a367634a` with the records and fixes of
  the CI cloud sessions, branch `ci/fixes-20260930` included). The shared
  `polyfem/build` is built at `db2a8bb12`.
- The `cloud/*` branches on `origin` (`ci-04`, `ci-05-06-refs`, `ci-06-order`,
  `ci-08-linux`, `linux-evidence`, `locale-fix`) were cherry-picked onto `main`; `git cherry`
  finds no unmerged work except the cloud full-suite log, which `main` carries
  as `09e95bd83`. The branches are safe to delete.
- `ipc-toolkit-fork` is at `1f1b5dbf` (`semi-implicit-stiffness` fast-forwarded
  2026-10-01 to the `solve_spd_2x2` refinement; the toolkit's `main` was not
  touched) and `polysolve-merged` at `43ca2e66`; both match their `origin`
  branches.
- The HDA sources are not a repository; see `houdini_HDAs/AGENTS.md`.
