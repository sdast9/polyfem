# RB-19 — Line-search roundoff fallback, step-1 stall diagnosis, cube-on-floor NaN

Date: 2026-09-11
Status: **done — implemented in PolySolve, validated on the public fixtures, published** (PolySolve `5afe3b5d4` on `sdast9/polysolve:iteration-callback`; PolyFEM pin bump and this record on `sdast9/polyfem:main`). The `contact_2d` observation at the end is recorded as measured.

Three follow-ups from the RB-18 handoff, requested by the user as one bounded item:

1. **Line-search roundoff tolerance in PolySolve** (`ΔE ≤ max(Armijo, ε·(1+|E|))`).
2. **Verify whether the cube-on-floor `1/2Δx^THΔx=nan` prints upstream too.**
3. **Single-threaded rerun of a step-1 no-contact failure.**

Items 2 and 3 were run first because their answers decide what item 1 has to be.

## Contract and authorization

- **User selection (2026-09-11):** implement the three fixes if not already
  implemented, validate the affected suite, confirm the roundoff tolerance does
  not mask genuine non-descent, and document across the md files. In the same
  request the user retired RB-17's per-contact band targeting (recorded in
  [rb-17-validation.md](rb-17-validation.md#retired-per-contact-band-targeting-2026-09-11)).
- **Invariant:** the line search must never accept a step along which the
  gradient norm grows on the strength of an energy difference it cannot resolve;
  every step the previous criteria accepted must remain accepted (so solves that
  never hit a roundoff rejection are unchanged); CCD, the trial-displacement cap,
  the retired floor, the semi-implicit coefficient law (RB-18) and PolySolve's
  configured convergence contract are untouched.
- **Boundary with RB-16/RB-17:** RB-16's gradient-integral arithmetic was
  declared a research control, "not an authorized production line-search
  repair". This item does not install it. RobustArmijo's own (upstream)
  gradient-integral test is left as is; the change adds a *gradient-norm*
  fallback behind it, gated on the energy being at roundoff.
- **Scope of the PolySolve change:** it is in the shared line search and
  therefore applies to every PolyFEM solve that uses `Armijo` or
  `RobustArmijo` (the default), not only contact. That is why the affected
  suite, all five public smokes, HDA E2E and `contact_2d` were run.

## Baseline and reproduction

- PolyFEM `main` at `116f21a8b` (RB-18 done). IPC `af317a65` (local
  `ipc-toolkit-fork`, unchanged). PolySolve `4d372fa8` (local
  `polysolve-merged`, `iteration-callback`), clean at start.
- Build `polyfem/build`, macOS arm64, RelWithDebInfo, TBB. The scratch
  worktrees `premerge`/`upstream`/`upsdeps` from the September sync are
  **gone** (their scratchpad was cleaned; `git worktree list` shows them
  prunable), so the "upstream build" from the sync memory is no longer
  available without a multi-hour rebuild. Item 2 was answered from the code
  and the fork's own logs instead (below).
- Evidence directory: `outputs/rb-19/20260911T173945Z/` in the parent
  workspace. Compact results: [`tools/rb19/results-20260911-step1-threads.json`](../tools/rb19/results-20260911-step1-threads.json).

### Item 3 first: the step-1 no-contact failure is thread-order noise at the energy's roundoff floor

The RB-04 refinement case `quasistatic-semi`, `dt=.0625`, band `[.1,.9]`
(`outputs/rb-04/20260909-refinement-quasistatic/lower-0.1-dt-0.0625/`) was
rerun on the RB-18 binary, three times with the default thread count and three
times with `--max_threads 1` ([runner](../tools/rb19/run_step1_threads.py)):

| Threads | Runs | Completed 16 steps | Failed at step 1 | Iteration logs |
| --- | ---: | ---: | ---: | --- |
| default (TBB) | 3 | 2 | 1 (exit −6, after 2 unchanged restarts — F5) | all three differ |
| 1 | 3 | 3 | 0 | all three **bit-identical** |

What the step is: the top face is pushed down 0.0156 while the cube is not yet
in contact and there is no gravity, so the minimizer is a rigid translation
with zero elastic energy. The first trial-step energy difference (`ls it: 0`)
at `f₀ = 76.56` reads `-76.024953154924{08,15,09}` across the threaded runs and
`…02` in every single-threaded run: the energy carries ~1.3e-13 of
summation-order noise. By the end of the subsolve the evaluated energy sits at
a floor of ~1.7e-14 while ‖∇f‖ is 8.6e-5 against a rescaled tolerance of
2.28e-6, the Newton step has ‖Δx‖ = 1e-10 and predicted decrease
Δx·∇f = −3.4e-15 — a hundred times smaller than the noise. Every trial step is
rejected on noise, the step shrinks to 3.7e-9 (below the ULP of x, so Δf reads
exactly 0), the fork's α<0.01 stall detector fires, and each restart re-runs
the identical problem (20 restarts before RB-18 F5, 2 after).

**Cause in PolySolve.** `use_grad_norm_tol` (default 1e-6) is PolySolve's own
guard for this regime: "when the energy is smaller than use_grad_norm_tol,
line-search uses norm of gradient instead of energy". `LineSearch::line_search`
computes the flag (true here: ‖∇f‖ 8.6e-5 < 1e-6 × 228 = 2.28e-4) and passes it
to `criteria(...)`, but only `Backtracking::criteria` reads it.
`Armijo::criteria` and `RobustArmijo::criteria` — the default — ignore the
argument. RobustArmijo's approximate test (Longva et al. 2023) also cannot
rescue the *full* Newton step: on a locally quadratic energy `∇f(x+Δx) ≈ 0`, so
its error estimate `½α|Δx·(∇f_new − ∇f_old)|` equals its decrease estimate
`½αΔx·(∇f_new + ∇f_old)` exactly and the sum is 0, never ≤ the negative Armijo
bound; it accepts α = ½ at best.

This is upstream behavior (the same three files are byte-identical in
`polyfem/polysolve@da4e7fe`), reproduced through the fork's stall detector.
The flakiness is not the fork's coefficient law: the stall occurs before any
contact is active.

### Item 2: the cube-on-floor `1/2Δx^THΔx=nan` is a disabled criterion, not a NaN curvature

`Solver::minimize` sets `m_current.newtonDecrement = NaN` every iteration and
only evaluates it when `newton_decrement_tol > 0` (`Solver.cpp`, the
`m_stop_rescaled.newtonDecrement > 0` branch). The spec default is `0`, so the
`Finished:` line prints `1/2Δx^THΔx=nan` for **every** solve. Confirmed on the
fork's own RB-18 affected-suite log: 64 of 64 `Finished:` lines print it,
including cleanly converged ones (`Gradient vector norm too small`). The two
`Search direction not a descent direction` lines in the cube-on-floor log
therefore carry no NaN information; this is upstream logging, not something to
chase. The cube-on-floor `err_h1` gap itself (0.35% vs a 1e-5 margin) remains
as recorded in the sync memory; see the `contact_2d` observation below for what
the line-search change does to it.

## The change (PolySolve `5afe3b5d4`)

Files: `src/polysolve/nonlinear/line_search/Armijo.{hpp,cpp}`,
`RobustArmijo.cpp`, `nonlinear-solver-spec.json`, `tests/test_nonlinear_solver.cpp`.

After the Armijo test rejects a step (and, for RobustArmijo, after its
gradient-integral test also rejects it):

```text
if use_grad_norm  or  |E(x+αΔx) − E(x)| ≤ ε·(1 + |E(x)|):
    accept  iff  ‖∇f(x+αΔx)‖ < ‖∇f(x)‖          (finite gradient required)
else reject
```

- `ε` is the new `line_search/Armijo/roundoff_tolerance` (default machine
  epsilon 2.22e-16; `0` disables the floor). `use_grad_norm_tol = 0` disables
  the switch. Both gates are per-solver JSON, so any scene can turn the
  fallback off without a rebuild.
- The user's formula `ΔE ≤ max(Armijo, ε(1+|E|))` is implemented as the
  second gate. On its own it would **not** have rescued the reproduced
  failure: with `|E| ≈ 1e-14` the floor is 2.2e-16, five hundred times below
  the measured 1e-13 noise. The `use_grad_norm` switch is what does the work
  there; the ε gate covers the complementary regime where |E| is large and the
  decrease is below one ULP of it. The acceptance is deliberately *not* "any
  ΔE within roundoff": it additionally requires the gradient norm to fall,
  which is the "does not mask genuine non-descent" property the user asked
  for and the regression checks.
- `Backtracking::criteria` (which already used the gradient when flagged) is
  unchanged. `ResidualBacktracking` is unaffected.
- A trace-level line `ls it: N energy at roundoff; accepted on gradient norm
  a -> b` records every fallback acceptance, so its engagement is countable
  in any trace log.

### Regression (PolySolve `line-search-energy-roundoff`)

Standalone PolySolve build with tests (scratchpad, Release, minimal linear
solvers). A two-variable quadratic whose *measured* energy is `max(E, floor)`
models the observed floor exactly; gradient and Hessian are exact. Generated
over `Armijo` and `RobustArmijo`:

| Section | Setting | Expected | Result |
| --- | --- | --- | --- |
| `use_grad_norm` switch alone | `use_grad_norm_tol=1e-6`, `roundoff_tolerance=0` | converges (`GradNormTolerance`), α=1 every step, ≤2 iterations | pass |
| roundoff floor alone | `use_grad_norm_tol=0`, `roundoff_tolerance=ε` | converges | pass |
| both disabled (pre-fix) | `0`, `0` | Armijo: throws; RobustArmijo: only ever accepts α=½ | pass (reproduces the old behavior) |
| does not mask ascent | gradient descent on `½(100x₀²+x₁²)` floored, always in the roundoff regime | every α ≥ 1/32 (which grows ‖∇f‖) rejected; largest accepted α = 1/64 exactly | pass |

23 assertions. Full PolySolve nonlinear selection `nonlinear*,iteration-callback,
MMA,sample,line-search-energy-roundoff`: **10 cases / 1,200 assertions**,
exit 0; `[json]` 2 cases / 20 assertions. The one failure in a full `[solver]`
run is `mas_block_dim`, a hidden `[.]` linear-solver test hardcoded to
`/home/ian/...`; it fails identically before this change.

## Validation on PolyFEM

Fixed binary hashes in `outputs/rb-19/20260911T173945Z/fixed/tested-binaries.sha256`
(built from the local `polysolve-merged` at the committed change; the final
comment-realignment in `Armijo.hpp` was applied after that build and changes no
code).

| Check | Input/configuration | Expected criterion | Result |
| --- | --- | --- | --- |
| Step-1 failure, threaded | 10 runs, default threads | 16 steps each, 0 restarts | **10/10** complete; fallback accepted 6–13 steps per run, always at α=1 on the final Newton step of a subsolve (‖∇f‖ ~1e-4 → ~1e-10) |
| Step-1 failure, single thread | 3 runs, `--max_threads 1` | identical, complete | **3/3** complete, 9 fallback steps each, iteration logs bit-identical |
| Affected suite | `[contact_cache] [contact_stiffness_mapping] [semi_implicit_coefficients] [al_solver] [physical_diagnostics] [direction_filter] [restart] [form_derivatives] [floor_retirement] [bc_scale] [contact_form] [friction_form]`, seed 1 | no assertion failure | **52 cases / 3,349 assertions**, exit 0 — same counts as RB-18 F7 (`affected-tests.log`) |
| Classic `adaptive` smoke | `quasistatic-adaptive.json` | Linf `0.24282997612162013`; endpoint within noise | Linf identical; max abs endpoint diff vs RB-18 F7 2.1e-16 |
| Semi-implicit smokes | `quasistatic-semi`, `-alhess`, `transient-semi` | four steps, zero error lines, endpoints within noise | all exit 0, 5 saved steps; diffs 1.3e-16 / 1.9e-16 / 2.5e-16; identical refresh-trim histories |
| Friction smoke | `quasistatic-semi-friction` | four steps, zero error lines; Linf `.2424763066234872` (RB-18 F6 value) | exit 0; Linf identical; diff 1.4e-14 (RB-18 measured 1.7e-14 run-to-run noise for this scene) |
| Fallback engagement on the smokes | same five at `--log_level trace` | measured, not assumed | **0** fallback acceptances in 26–39 line searches per scene: the smokes are unchanged by construction |
| HDA E2E | `houdini_HDAs/tests/test_polyfem_hda.py` via `hython` 22.0.429 | pass | `PASS: end-to-end PolyFEM 2.0 HDA test`, exit 0 (`hda-e2e.log`) |
| `contact_2d` observation | `unit_tests "contact_2d"` (28 upstream scenes incl. cube-on-floor) | recorded as measured; not a gate | see below |
| Formatting / diff | changed PolySolve files | clang-format, `git diff --check` clean | clean (the three pre-existing `//std::cout` warnings in `RobustArmijo.cpp` are upstream's) |

Physical accuracy, the full PolyFEM suite, private scenes and Teseo are
**not** in scope. Completing 16 steps is numerical termination, not a physical
acceptance criterion.

### `contact_2d` observation

`unit_tests "contact_2d"` on the fixed build: **27 of 28 scenes pass; the
single failure is `gcp-contact/cube-on-floor/run.json`**, as before RB-18 and
RB-19 (exit 42, `outputs/rb-19/20260911T173945Z/contact_2d/run.log`). This
item cannot have moved it: `gcp-contact/common.json` selects
`"line_search": {"method": "Backtracking"}`, the one class this change does not
touch, and the log confirms both `Finished:` lines are `[Backtracking]`. The
numbers on `main` are `err_h1` 0.09835223 vs stored 0.09881319 (rel. 4.66e-3),
`err_l2` rel. 3.35e-5, margin 1e-5. They are **not** comparable to the 0.35%
in the September sync memory, which was measured on the unmerged
`sync-upstream` branch (a different upstream base); the last solve there had
56 iterations and ‖∇f‖ 3.35e-3, here 68 iterations and 4.16e-3.

What the log does show is that cube-on-floor's final subsolve stops on the
*same roundoff regime* this item addressed, through a different check:
`SparseRegularizedNewton (reg_weight=1e5)` ends with `Search direction not a
descent direction` at `Δf=0`, ‖Δx‖=1.76e-11, ‖∇f‖=4.16e-3, i.e. the solver's
`Δx·∇f ≥ 0` test (`derivative_along_delta_x_tol` is 1e-18 in this scene) fires
on a dot product at roundoff, before any line search runs. That check lives in
`Solver::minimize`, not in the line search, and is unchanged here. It is the
concrete next thing to look at for cube-on-floor; it is not "ours to fix" in
the sense of fork-only code (same code upstream), but the golden gap is still
open and was not investigated further in this item.

## Publication

- PolySolve: `5afe3b5d4` on `sdast9/polysolve:iteration-callback` (pushed).
- PolyFEM: pin bump in `cmake/recipes/polysolve.cmake` to `5afe3b5d4`, this
  record, `tools/rb19/`, plan/README updates, and the RB-17 retirement, on
  `sdast9/polyfem:main`. The configured build uses the local `polysolve-merged`
  override, which is at the pinned commit.
- Parent README updated.

## Next session handoff

- **Behavior change to be aware of:** any `Armijo`/`RobustArmijo` solve that
  previously rejected a step in the near-convergence regime
  (‖∇f‖ < `use_grad_norm_tol` × scale, or |ΔE| ≤ ε(1+|E|)) can now accept
  it when the gradient norm falls. Solves that never hit such a rejection are
  bit-identical (the five public smokes are). Any golden recorded from a run
  that *did* stall in that regime will differ. Turn the fallback off per scene
  with `line_search.use_grad_norm_tol = 0` and
  `line_search.Armijo.roundoff_tolerance = 0` to reproduce old behavior.
- The step-1 flakiness is now understood as TBB summation-order noise at the
  energy floor; it is no longer a coefficient-law suspect. Single-threaded
  runs remain the way to get bit-reproducible logs.
- Cube-on-floor: the NaN question is closed (disabled criterion). The
  residual/golden gap is still open; the upstream-source build that would
  separate "our polyfem sources" from "our dependencies" no longer exists and
  would need to be rebuilt (~hours) if that comparison is wanted.
- Eligible next items: force-continuation κ and parent-keyed κ (RB-18
  handoff; RB-17's successors), RB-05.
