# EF-04 — Stall trigger relative to the feasible bound

Date: 2026-09-24. Item EF-04 of the
[contact-efficiency plan](contact-efficiency-plan-20260923.md). **Status:
done — implemented opt-in, measured; no default change.** The mechanism the
plan names (H-E) is confirmed; removing it does not make any measured scene
cheaper, so the default stays the historical absolute trigger. **Decided
2026-09-25 (user): the default stays `absolute`** after the
[EF-04b retest](ef-04b-feasible-bound-retest.md); `feasible_bound` remains an
opt-in, not exposed in the Houdini asset.

Evidence: `outputs/ef-04/20260924T200652Z/` (not in the repository): `runs/`
(one directory per run, `row.json` with the command, overrides and binary
hash), `byte-identity/` (the five public smokes, base vs this change),
`tables.md` / `metrics.json` (reduced by `tools/ef04/ef04_tables.py`, with
EF-01's production runs as baselines), the three sequence scripts and logs,
and `bin/` (`PolyFEM_bin-base-d48108ef4`, sha256 `01cd8bee…`, the shared
build of the published head; `PolyFEM_bin-ef04`, sha256 `070c7b9b…`, this
change on `d48108ef4`).

## The question

The semi-implicit stall trigger (`solver/contact/semi_implicit/restart`)
restarts the nonlinear solve — refreshing every per-contact coefficient and
retuning the trim — after `patience` (5) consecutive accepted steps with
α < `alpha_threshold` (0.01). α is judged absolutely. On a truncation-bound
scene α is small because CCD or the trial-displacement cap bounded the step,
not because the line search backtracked; EF-01 found 317 of R4's 354
small-α iterations accepted exactly at that bound (H-E). The plan: count a
step only when the line search went below the feasible bound, opt-in, and
measure restarts, iterations, wall and byte-identity.

## What changed in the code

* `StallRestartOptions` (`src/polyfem/solver/ALSolver.hpp`) gains
  `alpha_basis` (`Absolute` — the historical trigger, default — or
  `FeasibleBound`) and `feasible_ratio_threshold` (default 0.999), and a
  `from_json` reader that both call sites now use (the varform and the legacy
  `StateSolveNonlinear` path read the same six keys by hand before). Bad values
  are named errors.
* `ALSolver::minimize_with_stall_restarts`: under `FeasibleBound`, a step with
  α < `alpha_threshold` counts toward the patience only when PolySolve's line
  search reports `accepted_over_feasible` < `feasible_ratio_threshold`; a step
  accepted at the bound resets the count like a large step. The feasible bound
  is what the problem set before the descent search: the finite-energy bound,
  CCD and the trial-displacement cap (`ContactForm::max_step_size` multiplies
  the CCD fraction by `trial_clamp`). A step whose line search reports no
  ratio is judged absolutely. No PolySolve change: the ratio was already in
  `LineSearch::diagnostics()`, current when the iteration callback runs.
* Under `FeasibleBound` the subsolve record carries `stall_alpha_basis`
  (basis, threshold, small-α iterations, how many sat at the bound, how many
  were unclassified). Under the default nothing is added, so default records
  are unchanged.
* Spec: `/solver/contact/semi_implicit/restart/alpha_basis` (`absolute` |
  `feasible_bound`) and `/feasible_ratio_threshold` (float in (0, 1]). The run
  manifest records both through the effective input.
* The soft iteration limit and a line search that fails on every strategy (the
  hard stall) are untouched.
* Tests (`tests/test_al_solver.cpp`, `[stall_trigger][ef04]`): a quartic whose
  step is bounded to 1e-3 of the proposal for its first iterations restarts on
  `alpha` under the absolute basis and converges without a restart under the
  feasible-bound basis (every small step counted at the bound); an objective
  whose understated curvature makes Backtracking halve 13 times still restarts
  on `alpha` under the feasible-bound basis; a lower threshold (1e-4) exempts
  those steps; `from_json` defaults and refusals. `[al_solver]`: 26 cases,
  4,082 assertions; with the groups that exercise the semi-implicit path
  (`[stall_trigger]`, `[contact_stiffness_mapping]`, `[friction_lag]`,
  `[contact_cache]`, `[kappa_continuity]`, `[objective_generation]`,
  `[semi_implicit_coefficients]`, `[rollback]`, `[run_manifest]`,
  `[input_validation]`, `[al_budget]`, `[trim_predictors]`,
  `[fully_prescribed]`, `[iteration_observer]`) 96 cases, 9,123 assertions,
  all pass (`suites.log`). The coefficient path is untouched, so the RB-02
  probe was not rerun.

Not exposed in the Houdini asset (as with RB-07's opt-in budget): it has no
measured benefit and its default is undecided.

## Method

`tools/ef04/ef04_tables.py` (new) reduces the runs with EF-01's
`ef01_reduce.py` and adds the small-α split (accepted α < 0.01; "at bound" =
accepted/feasible ≥ 0.999, EF-01's classification) from the attempt stream.
Runs through `tools/ef01/ef01_run.py` (Newton, the scene's settings, iteration
and physical diagnostics, predictors on, coefficient events discarded), R4 on
all threads, everything else single-threaded, strictly sequential. Accuracy:
relative L2 of `solution` per step against EF-01's pinned tight references
(`ref-SCENE`). Baselines: EF-01's production runs (binary
`PolyFEM_bin-ef01`, sha256 `1df8e4cb…`; the commits between it and the base
touch HDF5 output and input selection reads only — the driver rewrites R4's
file selections anyway) plus a default-setting control of every scene on this
binary.

**Sleep.** The machine entered clamshell sleep on battery at 22:32Z (the
`caffeinate -i` assertion does not hold it then) and cycled through dark wakes
until 23:39Z. Runs finished before 22:32Z (smokes, R1, BBT, `fb-R4`,
`fb-R4-rep`, `ctl-R4`) are clean. The last ≈10 minutes of `fb5-R4` and the
pinned runs until 23:39Z (`fbpin-*`, `ctlpin-R1-k-18`) ran through it, so **their wall times are not comparable**
(the driver's monotonic clock excludes sleep but not the dark-wake
throttling); iteration counts are unaffected.

## Results

### 1. Byte-identity

* The five public smokes (`quasistatic-adaptive`, `-semi`, `-semi-alhess`,
  `-semi-friction`, `transient-semi`), default settings, single-threaded,
  base vs this change: all 55 solution files (VTU/VTM/PVD) byte-identical;
  the manifests differ in run metadata and the two new keys at their defaults.
* `smoke-qs` and `smoke-tr` with `alpha_basis: feasible_bound` vs off: all 10
  VTUs byte-identical — they never take a step with α < 0.01.
* BBT (20 steps) under both bases: identical iteration counts per step and
  identical errors; it never restarts either way.

### 2. The mechanism (H-E) — confirmed on R4, not on R1

In EF-01's production runs, the five iterations that fired each alpha restart:

| run | alpha restarts | fired on 5 consecutive steps at the feasible bound |
|---|---|---|
| R4 `prod-R4` | 8 | 8 |
| R4 `rep-R4-prod` | 9 | 8 |
| R1 `prod-R1` | 6 | 1 |

On R4 the alpha trigger is a feasibility trigger: every restart but one fired
while each step was accepted exactly where CCD or the trial cap put it. On R1
the restarts follow backtracking, as EF-01 found.

### 3. Cost

R4 step 1 (the controller from trim 1; production settings otherwise):

| run | basis | iterations | restarts by trigger | small α: at bound / backtracked | wall s | error vs ref |
|---|---|---|---|---|---|---|
| `prod-R4` (EF-01) | absolute | 800 | alpha 8, soft 3 | 317 / 37 | 1,854 | 11 % |
| `rep-R4-prod` (EF-01) | absolute | 726 | alpha 9, soft 3 | 268 / 31 | 1,589 | 11 % |
| `s5-R4-prod` step 1 (EF-01) | absolute | 732 | — | — | — | — |
| `ctl-R4` | absolute | 828 | alpha 6, soft 5 | 304 / 36 | 1,865 | 9.6 % |
| `fb-R4` | feasible_bound | 879 | soft 8 | 330 / 54 | 1,998 | 12 % |
| `fb-R4-rep` | feasible_bound | 852 | soft 8 | 324 / 37 | 1,877 | 11 % |

R4 steps 1–5: production (EF-01 `s5-R4-prod`) 1,041 iterations
(732/59/77/84/89; alpha 5, soft 5); feasible bound (`fb5-R4`) 1,097
(806/65/49/87/90; soft 7); errors 9.8 % and 14 % against the step-1
reference (R4's Newton-vs-Newton spread is 3.6–8.7 %, EF-01).

R1 (3 steps), BBT (20 steps) and the smokes:

| scene | absolute (runs) | feasible bound | feasible bound, threshold 0.3 |
|---|---|---|---|
| R1 iterations | 377, 388 (EF-01); 357, 362 | 388, 411 | 390 |
| R1 restarts | alpha 4–8, soft 0–1 | alpha 3–5, soft 1 | alpha 1, soft 2 |
| R1 error per step | 7.6e-4, 4.3–4.5e-4, 5.1–8.0e-4 | 7.6e-4, 4.2–4.3e-4, 5.1e-4 | 7.6e-4, 4.8e-4, 7.7e-4 |
| BBT iterations | 186, 186 | 186, 186 | 187 |
| smoke-qs / smoke-tr iterations | 31 / 26 | 31 / 26 | — |

At EF-01's cheapest pinned trims (the regime an EF-02 step-start estimate
would put the controller in):

| run | iterations | restarts |
|---|---|---|
| R1 pin 2⁻¹⁸, absolute (EF-01 / here) | 149 / 166 | alpha 2 / alpha 2 |
| R1 pin 2⁻¹⁸, feasible bound | 153 | none |
| BBT pin 2⁻¹⁸, absolute (EF-01) / feasible bound | 101 / 101 | none / none |
| R4 pin 2⁻¹², step 1, absolute (EF-01 / here) | 109 / 108 | alpha 1 / soft 1 |
| R4 pin 2⁻¹², step 1, feasible bound | 114 | soft 1 |

At the pinned trims the trigger barely fires under either basis (R4: 20–22 of
24–28 small-α steps at the bound, one restart), and the cost is equal.

### 4. Why removing the restarts does not help

Under the feasible-bound basis every R4 attempt runs to the soft iteration
budget (attempt lengths 101 ×8, then 68 or 44), where production's attempts
end on the alpha trigger after 10–98 iterations. The trim's walk is the same
either way: it first leaves 1 at iteration 354–426 under the absolute basis
and 423–437 under the feasible bound, and reaches 2⁻¹⁰ at 649–765 against
792–819. The descent is driven by the in-solve controller (every 30
iterations while the gap is pinned above the band) and by the retunes; the
restarts that fired at the bound neither caused nor delayed it, and the soft
budget takes over their retunes. With the bounded-step restarts gone the step
costs 852–879 iterations against 726–828 for the four absolute runs, at the
top of or just above their spread; the error is inside R4's repeat spread.

On R1 the trigger fires mostly on backtracking, so the basis changes little
(3–5 alpha restarts instead of 4–8, iterations inside the repeat spread). BBT
and the smokes never restart.

## Conclusion

* **H-E holds** as a statement about the trigger: on R4, 16 of 17 production
  alpha restarts fired on steps accepted at the feasible bound.
* **Removing those restarts does not reduce cost** on any measured scene: R4
  +3 to +21 % iterations (inside or just above the absolute runs' spread), R1
  inside its spread, BBT and the smokes unchanged. R4's cost is the trim walk
  (EF-01, H-A), which the trigger basis does not move.
* The feasible-bound basis stays **opt-in, default off**. Recommendation to
  the user: keep the absolute default. The option is a cleaner signal to
  revisit once EF-02/03 start the trim near its optimum (at the pinned optima
  it removes R1's two restarts and costs the same on R1, BBT and R4) and for EF-06's preconditioned
  L-BFGS, whose fixed-interval runs the absolute trigger interrupted on R1.

## Reproduce

```
python3 tools/ef01/ef01_run.py fb-R4 --scene R4 --steps 1 --threads 0 --timeout 2700 --out EVIDENCE --binary BIN \
    --set /solver/contact/semi_implicit/restart/alpha_basis='"feasible_bound"'
python3 tools/ef04/ef04_tables.py EVIDENCE --ef01 EF01_EVIDENCE
```

Unit tests: `unit_tests "[stall_trigger]"`. The scenes R1/R4/BBT are the
user's Houdini exports under `test_cases/` (not in this repository).
