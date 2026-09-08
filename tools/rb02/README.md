# RB-02 coefficient audit

Build the existing `PolyFEM_bin` and `unit_tests` targets, then run from the
PolyFEM repository with a **fresh absolute output directory**:

```bash
python3 tools/rb02/run_probe.py --build build --output /absolute/fresh/evidence
```

This bounded standalone C++ probe reuses the configured unit-test compile/link
recipe. It calls real contact/IPC/friction methods, uses a synthetic known
Hessian with identity collision mapping, and exits nonzero on an assertion
failure. It is not registered in the default Catch suite. No scenes are run by
this script. NaN/overflow are injected only into tiny algebraic fixtures; geometry
remains finite and nondegenerate. Assigned nonfinite coefficients are recorded
and rejected by the probe before energy evaluation.

[Contract](../../docs/rb-02-contract.md), [validation](../../docs/rb-02-validation.md),
and [dated measurements](results-20260908.json) distinguish characterization
assertions from model acceptance. Several assertions deliberately reproduce
undesirable current behavior. Do not treat a passing audit as contact-model
certification or regenerate measurements to conceal a behavior change.
