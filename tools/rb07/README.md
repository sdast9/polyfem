# RB-07 bounded AL stagnation: reproduction and end-to-end checks

Tools behind [docs/rb-07-validation.md](../../docs/rb-07-validation.md).
Public inputs only (the semi-implicit cube-on-slab fixture); nothing here runs
Teseo or a private scene.

```sh
# Baseline reproduction: the incompatible scenes do not stop (wall-clock timeout)
python3 tools/rb07/run_al_stagnation.py --output /absolute/fresh/dir --binary build/PolyFEM_bin --stage reproduce --timeout 300
# Candidate: the budget ends them with a named failure, the compatible one is unchanged
python3 tools/rb07/run_al_stagnation.py --output /absolute/fresh/dir --binary build/PolyFEM_bin --stage validate \
    --budget '{"max_passes": 200, "stagnation_window": 3}' --scenes compatible-multipass incompatible-collision
# Pass table of any debug log and the first pass at which BC-residual-only rules would have stopped it
python3 tools/rb07/analyze_passes.py /absolute/run.log --every 5
```

`run_al_stagnation.py` builds three one-step quasistatic scenes from the
public `quasistatic-semi.json` (single-threaded, RB-04 diagnostics and RB-12
manifest on): `compatible-multipass` (top face pressed 0.3 in one step, AL
initial weight 1e4, subsolves interrupted after four iterations — 105 passes
at the baseline, then a feasible snap), `incompatible-collision` (bottom face
prescribed 0.05 below rest with the slab 0.02 below it — the snap crosses the
obstacle, the AL drags the penalised obstacle nodes along and settles at a
fixed point) and `incompatible-crush` (top face prescribed down by the cube
height — the snap inverts the elements under it). The reproduce stage records
the pass history from the log; the validate stage checks the named failure,
exit status 1, the manifest's `al_stagnation` record (reason, pass history,
blocking gate, window at the ceiling with no progress signal), the verified
RB-06 rollback, that nothing of the step was published, and — for the
compatible scene — that the budget-on run has the same passes and a
byte-identical endpoint as the budget-off run. `results.json` keeps every
command, exit status, input hash, pass history and check; the output directory
is never overwritten.

`analyze_passes.py` is the measurement behind the stagnation rule's design: it
evaluates BC-residual-only windows (relative decrease, running-minimum
patience) on a log and reports where they would have ended the stage, so that
a rule can be judged against a run that is known to succeed. On the compatible
fixture every such window of 3–20 passes triggers before the snap (a false
positive); the production rule adds the snap gates, the collision-free fraction
of the snap and the iterate drift, and on that fixture never fires.

The in-process counterparts are `unit_tests "[al_budget]"`
(`tests/test_al_solver.cpp`: option parsing, the pass cap, an unexhausted
budget on the PF-07 continuation, the stagnation window on a synthetic wall;
`tests/test_step_rollback.cpp`: the public transient fixture driven below the
slab at step 2, rolled back and recorded).
