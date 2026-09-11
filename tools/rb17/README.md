# RB-17 coupled-controller gate

This directory tests the approved scalar interval controller on exact coupled
quadratic mechanics. It does not implement a production PolyFEM mode. Read the
[predeclared protocol](coupled-protocol.md), [exact cases](coupled-cases.json),
and [validation/decision record](../../docs/rb-17-validation.md).

From `polyfem/`, use new output paths:

```bash
python3 tools/rb17/coupled_probe.py --ipc-source ../ipc-toolkit-fork --output /absolute/fresh/coupled
python3 tools/rb17/audit_coupled.py --evidence /absolute/fresh/coupled --output /absolute/fresh/audit.json
```

The probe requires Python, NumPy and `/usr/bin/c++`. It compiles the existing
RB-13 bridge against the effective IPC barrier source, saves compilation and
provenance, and writes every individual run before its summary. Verify the
effective IPC checkout/pin before reproduction. Source hashes identify the tested
code; no production library or executable is rebuilt or changed by the runner.

The [measured matrix](results-20260911-coupled.json) contains 144 runs from six
mechanical cases, two update factors, three length scales, two parent evaluation
orders and two arithmetic branches. Its 3,372 passing implementation checks do
not mean all solves or gap objectives succeeded. Direct arithmetic retains 24
numerical failures; the separate stable control retains 12. The held-out scalar
coefficient fixed point misses an attainable band. The
[independent audit](audit-20260911-coupled.json) adds 488 passing checks, including
60-digit evaluation and independent force/work reconstruction.

The [provenance](provenance-20260911-coupled.json) identifies the manifest commit,
source hashes, compile command and runtime. Full per-run histories, failed
iterates, coupled-reference forces/coefficients and source copies are retained
at `outputs/rb-17/20260911T143421Z-coupled/` in the parent workspace. These results
are measurements, not test goldens. No failed result is replaced by another
arithmetic branch. FEM, geometric CCD, features, transient cycles, friction and
production-wrapper behavior remain untested in this stage.
