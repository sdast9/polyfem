# Stall recovery outside semi-implicit mode — 2026-10-01

Date: 2026-10-01. Status: **line-search failure recovery implemented for every mode (user
decision 2026-10-01); small-step stall restarts outside semi-implicit mode measured on an
uncommitted experiment, decision pending (open items D8).**

## Origin

A review of what the fork only applies in semi-implicit mode found that most of it acts on the
per-contact coefficients and the trim, which no other mode has. The classic adaptive barrier
stiffness sat at its built-in floor (`max/100`) in all 226 initializations measured on
`5-cubes-fast`, the 100-step slope test (`high-school-physics-slopetest-mu=0.50`, constant
4.379e14 for the whole run) and the classic smoke `quasistatic-adaptive`, so gradient-balance,
clamped-contact, continuation and friction-lag changes would be inert there (consistent with
RB-10's classic check). Two things generalize:

1. **Recovery from a line search that failed on every strategy.** Only semi-implicit mode with
   stall restarts recovered (its stall controller treats it as a hard stall, `62ee92e6`); every
   other configuration ended the step on the first such failure.
2. **Restarts after repeated small steps.** The `gcp-contact/parallel-edge` crawl
   ([record](parallel-edge-regression-20260930.md)) is such a stall in a non-semi-implicit mode.

The user decided on 2026-10-01: make (1) general now; measure (2) on the emulated-AMD crawl with a
PSD remedy and the `feasible_bound` trigger, and on the classic `contact_3d` group, before
deciding; leave the rest of the semi-implicit machinery where it is.

## 1. Line-search failure recovery (implemented)

`ALSolver::set_line_search_failure_recovery(restarts, recalibrate)`: in a solve without stall
restarts, a "Line search failed" on every strategy restarts the subsolve from the iterate it
reached with a fresh nonlinear solver (no descent-strategy or regularization history), at most
`solver/advanced/line_search_failure_restarts` times per subsolve (**default 1**; 0 restores the
historical behaviour). With the classic adaptive barrier stiffness its own initialization runs
again at that iterate first, as at every solve start (`SolveData::classic_stiffness_recalibration`);
fixed, smooth (GCP), semi-implicit without restarts and contact-free solves restart with a fresh
solver only. A failure after the last restart is rethrown unchanged, so the step fails exactly as
before (named failure, RB-06 rollback). Semi-implicit mode with restarts on keeps its own handling
(its hard-stall path, including the revert to the subsolve start, which only makes sense there
because the retunes accumulate). The record carries `line_search_failure_restarts` only when a
recovery happened, so ordinary outputs are unchanged.

Scope: the forward elastic solve of the varform path (`NonlinearElasticVarForm`) and the legacy
`StateSolveNonlinear` path, in their AL passes and reduced solves. Not covered, as for the stall
restarts: friction lag re-solves (`friction_iterations` ≥ 2 call the nonlinear solver directly),
and the thermo-elastic, fluid, FSI, differentiable and L2-projection solves (their `ALSolver`
keeps the default 0).

Verification (cloud Linux, Release, toolkit `1f1b5dbf`, data `aed03ab`):

| Check | Result |
| --- | --- |
| `[line_search_failure]` (new): transient failure recovered once with the recalibration called at the reached iterate; persistent failure rethrown after a budget of 2; no budget keeps the historical failure; stall restarts own the failure when on | pass (1 case, 15 assertions) |
| `[al_solver]` | 27 cases / 4,097 assertions pass |
| Five `scenes/semi-implicit` smokes, single-threaded | exit 0; all 55 output files byte-identical to `main` before the change |
| Full Release CTest (`MKL_CBWR=COMPATIBLE OMP_NUM_THREADS=1`, `-j2`) | 426/426 pass |

No measured scene had a line-search failure, so the recovery's benefit outside semi-implicit mode
is not measured yet; what is measured is that it changes nothing where no failure occurs.

## 2. Small-step stall restarts outside semi-implicit mode (experiment, not committed)

Experiment: a worktree patch on `ff667021` (this change), never committed, enabled by environment
variables, installs the existing `ALSolver` stall restart for solves without one: the
semi-implicit restart defaults (α < 0.01 for 5 iterations after 5, at most 200 restarts) with the
**soft iteration limit off** (small steps only) and the **`feasible_bound`** basis (EF-04: a small
step counts only when the line search backtracked below the bound CCD set). Remedies: `fresh`
(restart with a fresh solver, nothing changed) and `psd` (switch Newton to PSD projection for the
rest of the step, then restart).

A PSD remedy has to set PolySolve's `Newton/force_psd_projection`: `use_psd_projection` only adds
projected Newton as a fallback strategy after plain Newton fails, which a crawl never does, and
PolySolve's Newton resets the problem's projection flag from its own parameters every iteration
(a first version that set the problem flag directly had no effect and behaved as `fresh`).

### `parallel-edge` crawl, `qemu-x86_64 -cpu EPYC-Milan`, no `MKL_CBWR` (the CI reproduction)

| Build / remedy | Outcome | Stall restarts | Newton iterations (60 steps) | Max relative deviation from the stored metrics (margin 1.2e-5) |
| --- | --- | ---: | ---: | ---: |
| `main` (no restarts) | step 40: `Reached iteration limit (limit=500)` | — | — | — |
| `main`, `MKL_CBWR=COMPATIBLE` (adopted CI setting) | solves | — | 362 | 2.9e-7 (earlier record) |
| `fresh`, `feasible_bound` | solves | 45 | 831 | 2.0e-7 |
| `fresh`, absolute basis | identical to the row above (every small step was a backtrack below the feasible bound) | 45 | 831 | 2.0e-7 |
| **`psd` (forced projection), `feasible_bound`** | **solves** | **1** (step 39, iteration 69) | **436** | **9.4e-7** |

Wall time under emulation: 205 s (`fresh`), 143 s (`psd`).

### Classic `contact_3d` group (native, `MKL_CBWR=COMPATIBLE`, one thread)

| Run | Result |
| --- | --- |
| CTest harness, `fresh` + `feasible_bound` | 50/50 authenticated, 16.2 min (the unmodified group took 16.8–17.9 min on this host) |
| CTest harness, `psd` + `feasible_bound` | 50/50 authenticated, 15.7 min |
| Every scene through `PolyFEM_bin` patched as the harness patches it, warning log level, `psd` + `feasible_bound` | 50/50 exit 0; **0 stall restarts and 0 line-search recoveries** in any scene (the experiment was active in every solve) |

So on the classic group the trigger never fires and the runs follow the same path; on the one
non-semi-implicit stall on record it rescues the step, and the forced-projection remedy does it
with one restart and fewer iterations than the adopted CI path.

### Decision for the user (open items D8)

Whether to make small-step stall restarts part of every mode without them, with the soft
iteration limit off, the `feasible_bound` basis and the forced-PSD remedy (default on, or opt-in).
Not measured: the classic `contact_2d`, `standard` and other groups; native AMD or Apple silicon
(only emulation reproduces the crawl); a crawl in classic barrier contact (none on record). It
would also make the `parallel-edge` scene robust by itself, which bears on D6.

## Evidence

`outputs/stall-recovery/20261001T1740Z/` (not committed): `item1-ctest/` (the full Release CTest
log), `smoke-before.sha` / `smoke-after.sha` (SHA-256 of the 55 smoke output files), and
`experiment/`: `experiment.patch` (the uncommitted worktree patch on `ff667021`), the five qemu
run logs and summaries (`pe-psd-*` are the first PSD version, which acted as `fresh`;
`pe-psdforce-feasible` is the forced-projection remedy), the two `contact_3d` harness logs, and
`sweep.py` with `sweep-psd-summary.tsv` (scene, exit status, wall seconds, stall restarts, remedy
switches, line-search recoveries).
