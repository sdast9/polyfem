# Contact convergence diagnostics — BFGS audit stage 4

Stage 4 of the [BFGS convergence audit](../../docs/bfgs-convergence-audit-20260922.md)
is a measurement, not a repair. `run_matrix.py` runs the five public
semi-implicit smokes under Newton and L-BFGS with PolySolve's opt-in
per-iteration diagnostics, adds a bounded uninterrupted pass and
one-factor-at-a-time conditioning variants, and reduces each run to the metrics
the plan's item 4 asks for.

```sh
python3 run_matrix.py --build /absolute/polyfem/build --output /absolute/fresh/dir
python3 run_matrix.py --build /absolute/polyfem/build --output /absolute/fresh/dir --only history
```

The exit status is 0 when every Newton control produced identical frames with
the diagnostics off and on, which is the check that makes the rest admissible.

## The two opt-ins

| flag | what it adds |
| --- | --- |
| `solver/nonlinear/advanced/iteration_diagnostics` | PolySolve records, per accepted iteration, the strategy and the source of its direction, the direction's norm against the gradient's and against the previous accepted direction's, the secant pair and the initial inverse-Hessian scale, the line search's feasible bound and the accepted alpha as a fraction of it, the accepted endpoint's gradient and slope, and the strategy transitions since the previous accepted iterate |
| `output/physical_diagnostics` | PolyFEM writes `solver-attempts.jsonl`; since schema version 2 each accepted row carries the above in its `solver` field |

Both are off by default. The diagnostics cost one extra gradient evaluation per
accepted iteration and change no acceptance or stopping decision — the Newton
control pair in the matrix exists to demonstrate that by frame hash, and it is
worth repeating after any change to either side.

## Reading the summary

`matrix-results.json` holds one row per run. Beyond the run's exit code, frames
and wall time:

- `restart_triggers` counts the stall restarts by the condition that actually
  fired. Before this stage the message named the alpha patience whichever
  condition triggered, so a solve the iteration budget interrupted read as a
  line-search collapse.
- `direction_sources` separates an L-BFGS limited-memory direction from its
  first/reset steepest-descent direction and from a curvature restart;
  `strategy_transitions` and `accepted_by_strategy` show whether the configured
  `GradientDescent` fallback was ever reached.
- `direction_norm_over_gradient_norm` against `alpha`, `accepted_over_feasible`
  and `line_search_no_backtracking` separates a collapsed *direction* from a
  line search that shrank the step: a tiny accepted displacement at alpha 1
  with no backtracking and no feasibility cap is the direction's doing.
- `endpoint_slope_over_initial_slope` is the measured Wolfe curvature ratio at
  the accepted alpha, and `strong_wolfe_c2_0p9_satisfied` counts how often the
  textbook condition already held. It is observed, never imposed.
- `inverse_hessian_initial_scale` and the `pair_*` quantiles show what the
  approximation believes the curvature is.

A run that ends in a named failure keeps everything: `endpoints` carries each
step's outcome and, for the failing step, the solver info parsed out of the
exception, including `stall_trigger` and the final iteration's diagnostics.

## What the matrix deliberately does not do

No production default changes. The uninterrupted rows switch the restart policy
off *in the diagnostic run only*, to answer whether the plateau is the budget's
doing; the plan is explicit that the production soft budget must not be changed
merely to make a run finish, and that a solver repair is accepted only on actual
configured convergence.
