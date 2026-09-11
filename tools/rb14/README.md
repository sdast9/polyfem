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
a selected production model. Stages 2 and 3 below extend the comparison.


## Stage 2: actual FEM assembly and mapped contact

From the PolyFEM directory, using an existing compatible Makefiles build:

```bash
python3 tools/rb14/run_fem_probe.py --build build --output /absolute/fresh/fem-evidence --config tools/rb14/stage2-cases.json
python3 tools/rb14/analyze_fem_probe.py --input /absolute/fresh/fem-evidence --output /absolute/fresh/fem-summary.json
```

Read [stage 2's protocol](stage2-protocol.md) before running. The runner compiles
and links only the standalone probe, reusing the existing unit-test build flags
and libraries. Each case runs in a separate process and records input, stdout,
stderr and exit status. Compile/link commands, logs, exit codes and archive hashes
are preserved. The summary includes all 18 cases and classifications; a structural
check passing does not imply every candidate was solved or accurate.

The [published result](results-20260910-stage2.json) omits full snapshot matrices
and endpoint vectors, which remain in local per-case records and are reproducible
from the checked-in case list. It retains nonlinear histories and errors. This
is a single selected-vertex/segment contact proxy, not whole-surface contact.
See the contract/validation for failed empirical coverage, the actual RHS sign,
zero-demand handling and remaining production decisions.

## Stage 3: physical neighborhoods and shared prediction

```bash
python3 tools/rb14/run_fem_probe.py --build build --source tools/rb14/influence_probe.cpp --config tools/rb14/stage3-cases.json --output /absolute/fresh/influence-evidence
python3 tools/rb14/analyze_influence_probe.py --input /absolute/fresh/influence-evidence --output /absolute/fresh/influence-summary.json
```

Read the [predeclared protocol](stage3-protocol.md). The standalone source preserves
stage 2's fixture and adds physical radii, residual-influence diagnostics, a shared
full predictor, sparse batch costs and two protection proposals. The [published
comparison](results-20260910-stage3.json) retains all 21 cases, failed coverage and
one incomplete nonlinear solve. Prior challenges and three new holdouts remain
separate. Structural checks passing do not mean all nonlinear solves converged.
The full local records retain endpoint vectors and matrices omitted here.
No production estimator or protection law is selected.
