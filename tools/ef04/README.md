# EF-04 stall-trigger tools

Reducer for [ef-04-stall-trigger.md](../../docs/ef-04-stall-trigger.md)
(contact-efficiency plan, item EF-04). Runs use [`../ef01/ef01_run.py`](../ef01/README.md)
with `--set /solver/contact/semi_implicit/restart/alpha_basis='"feasible_bound"'`
(and optionally `/solver/contact/semi_implicit/restart/feasible_ratio_threshold=…`).

| script | what it does |
| --- | --- |
| `ef04_tables.py DIR [--ef01 EF01_DIR] [--json OUT]` | per scene: EF-01's production baselines (when `--ef01` is given) and every run under `DIR/runs`, with the trigger basis, exit, wall, iterations per step, restarts by trigger, small-α iterations (α < 0.01) split into "at the feasible bound" (accepted/feasible ≥ 0.999) and "backtracked", end trim, trial-cap/CCD binding, `physical_balance_pass` and the per-step error against EF-01's `ref-SCENE`; writes `DIR/tables.md` and `DIR/metrics.json` |
