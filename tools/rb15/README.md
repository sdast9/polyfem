# RB-15 standalone feature-assignment comparison

From the PolyFEM repository, with existing `PolyFEM_bin` and `unit_tests` built:

```bash
python3 tools/rb04/run_candidate_probe.py --build build --output /absolute/fresh/baseline
python3 tools/rb15/run_probe.py --build build --output /absolute/fresh/candidate
```

Each output directory must be new. The runner compiles the standalone source
against the configured libraries, captures commands/exits/logs, hashes the source,
protocol and linked archives, and saves `probe-results.json`. No production file,
coefficient option, original baseline, scene input or existing evidence is edited.
The probe uses real IPC/BarrierContactForm geometry, gradients and Hessians, with
experimental assignments confined to this executable. The two-parent scale setter
is a probe helper, not an override installed in the solver.

Read the [protocol](protocol.md), [contract](../../docs/rb-15-contract.md), and
[validation record](../../docs/rb-15-validation.md). The checked-in
[results](results-20260911.json) passed 140 checks; they are dated measurements,
not goldens to regenerate automatically. [Provenance](provenance-20260911.json)
identifies the linked solver/dependencies and retained exploratory outcomes.
Physical accuracy, whole-scene robustness and a production model remain unselected.
