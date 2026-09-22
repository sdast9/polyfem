# Wolfe line-search comparison — BFGS audit stage 3

Stage 3 of the [BFGS convergence audit](../../docs/bfgs-convergence-audit-20260922.md)
added an opt-in strong Wolfe line search to PolySolve
(`solver/nonlinear/line_search/method: "Wolfe"`). `run_matrix.py` checks that it
is inert where it is not selected and measures what it does where it is, on the
five public semi-implicit smokes. The record is
[bfgs-wolfe-line-search-20260922.md](../../docs/bfgs-wolfe-line-search-20260922.md).

```sh
python3 run_matrix.py --build /absolute/polyfem/build --output /absolute/fresh/dir \
    --reference /absolute/outputs/bfgs-stage4/<stamp>/matrix/matrix-results.json --jobs 4
python3 run_matrix.py ... --only quasistatic-semi-lbfgs
```

The exit status is 0 when every Newton control reproduces the reference record's
frames with the diagnostics off, the same frames with them on, and a
diagnostic stream that passes [the RB-04 attempt checker](../rb04/check_solver_attempts.py).

## The rows

| rows | what they answer |
| --- | --- |
| `<scene>-newton-off` / `-on` | the controls: the search is inert where it is not selected, and the attempt stream (schema version 3) keeps its identities |
| `<scene>-newton-wolfe` | Newton under Wolfe at production settings |
| `<scene>-lbfgs-uninterrupted-{robustarmijo,wolfe-0.9,-0.5,-0.1}` | L-BFGS from identical settings in the configuration stage 4 showed converging — the stall controller's soft budget removed, a **diagnostic** setting, not a production one |
| `<pair>-lbfgs-production-wolfe-{0.9,0.1}` | the audit's two scenes at production settings |
| `quasistatic-semi-bfgs-production-*` | dense BFGS at production settings, RobustArmijo and Wolfe |

It reuses [stage 4's runner](../bfgs_stage4/run_matrix.py) for the runs and
their reductions, and adds the Wolfe search's own per-iteration record
(`solver.line_search.wolfe` in `solver-attempts.jsonl`): how each search ended,
its energy and gradient evaluations, its growth sweeps and the accepted slope
ratio. `--jobs` runs single-threaded solves concurrently; wall times are then
contended and comparable only within one matrix.

## Reading `wolfe.outcomes`

| outcome | meaning |
| --- | --- |
| `wolfe` | Armijo decrease and the strong curvature condition |
| `approximate_wolfe` | the energy was within `approximate_wolfe_epsilon · |f|` (or the Armijo roundoff bound) of the start, so the step was judged on its slope: the strong curvature condition held |
| `capped_*` | a decrease point whose slope was still steeper than `c2` allows, at a boundary growth could not cross: `feasibility` (CCD, inversion or the trial cap), `start_capped` (the caps had already shortened the unit step), `growth_limit`, `direction_admits_no_growth` (box-constrained solvers), `sweep_refused` |
| `armijo_*` / `flat_*` | the evaluation budget ran out or the bracket collapsed; the best decrease point was accepted |
| `fallback_*` | no decrease point was found; RobustArmijo from the same start decided |
| `objective_changed*` | the problem's objective changed during the search more often than the restart budget allows |
