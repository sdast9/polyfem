# Quasi-Newton on real contact scenes (qn-contact investigation)

Runners and reducers used by
[qn-contact-investigation-20260922.md](../../docs/qn-contact-investigation-20260922.md).
Nothing here changes a solver default; every deviation from a scene file is
recorded in the run's `row.json`.

| script | what it does |
| --- | --- |
| `qn_run.py LABEL --out DIR --scene R1 [--binary B] [--method M] [--steps N] [--threads T] [--set /ptr=value ...]` | runs one configuration with iteration diagnostics and physical diagnostics on, small visual output, into `DIR/runs/LABEL` |
| `iters.py RUN [EVERY]` (run from the evidence directory) | per-accepted-iteration table: energy, ‖g‖, ‖p‖/‖g‖, α, α over the feasible bound, the bound, trial L∞, backtracks, direction source, objective generation |
| `summarize.py [--ref RUN] RUN...` | per run: exit, wall, completed steps, accepted iterations per step (all sub-solves), Hessian factorizations (Newton: accepted iterations; preconditioned L-BFGS: refreshes summed per sub-solve), restarts, termination reasons, per-step relative L2 error of `solution` against a reference run |
| `compare_vtu.py RUN REF` | the per-step solution comparison alone |
| `energy_time.py RUN_A RUN_B [T...]` | energy and ‖g‖ at given wall-clock times after the start of step 1 (the k-th accepted row paired with the k-th `Line search finished` log line); energies are only comparable while both runs share the barrier trim |
| `kappa.py NEWTON_RUN LBFGS_RUN` | rigorous lower bound on the condition number seen along a solve: max Newton ‖H⁻¹g‖/‖g‖ (≤ 1/λ_min) × max L-BFGS y·y/s·y (≤ λ_max of the path-averaged Hessian) |

Scenes: `R1` = `test_cases/input` (plate + ball head), `R3` =
`test_cases/inflation/input`, `R4` = `test_cases/uniax_mesh_constraintfloor_zero_b/input`,
`smoke-qs` = `scenes/semi-implicit/quasistatic-semi.json`. The `test_cases`
inputs are the user's Houdini exports and are not in this repository.

The preconditioned L-BFGS options (`L-BFGS/preconditioner`,
`L-BFGS/preconditioner_refresh`) exist only in the unpinned PolySolve branch
`qn-contact-experiment`; a binary built from the pinned PolySolve rejects them.
