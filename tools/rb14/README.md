# RB-14 estimator probes

Stage 1 uses standard-library Python and compiles the unchanged RB-13 C++ bridge
with effective IPC's barrier.cpp into an isolated macOS shared library.

```bash
python3 tools/rb14/estimator_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/evidence
```

The output directory must not exist. Outputs are provenance.json (exact compiler
command and source hashes), compile.log and full results.json. The process exits
nonzero on a failed assertion. No production build or scene is launched.

Read the [predeclared protocol](stage1-protocol.md), [contract](../../docs/rb-14-contract.md)
and [validation record](../../docs/rb-14-validation.md). The checked-in
[comparison](results-20260910-stage1.json) retains all 28 parameter sets, candidate
estimates, predicted/realized roots, coverage, controls, cost and check totals.
The larger local result additionally retains assembled network matrices, all
224 positive-k root records and 806 individual assertions. The compact comparison
omits those redundant matrices/root/check lists and adds checks_summary; timings
vary on rerun. Other platforms and assembled production FEM behavior are untested.

Passing a counterexample check means its limitation was reproduced. Approximate
estimates are not required to equal the exact reference; their errors and failed
held-out coverage are retained. No candidate, fallback or empirical interval is
a selected production model. RB-14 stage 2 remains pending.
