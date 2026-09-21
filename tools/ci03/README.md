# CI-03 friction-policy A/B on the historical scene fixtures

Tools behind [docs/ci-03-validation.md](../../docs/ci-03-validation.md). Public
data only (the pinned `polyfem-data` fixtures); nothing is modified in place,
every run is an isolated copy, and no Teseo or private scene is involved.

```sh
# The harness hook (tests/verify_run.cpp): any manifest against any data directory,
# through exactly the scene harness (one thread, recorded duration, authentication).
POLYFEM_RUN_MANIFEST=/abs/manifest.txt POLYFEM_RUN_DATA_DIR=/abs/data build/tests/unit_tests run_manifest_env

# A/B matrix on the three CI-03 fixtures: isolated copies of fixture + common chain +
# meshes, one overlay per variant, the harness's six metrics vs the stored reference,
# lag re-solves from sim.json, per-step diagnostics for the -diag variants.
python3 tools/ci03/ab_runner.py --unit-tests build/tests/unit_tests --data data \
    --output /absolute/fresh/dir --label after --variants default,fi1,fi2,fi4,fi8,fiinf,default-diag,fi1-diag
# Metric-by-metric comparison of run sets (bit-identity checks)
python3 tools/ci03/compare.py --output /absolute/fresh/dir before:default after:fi1 after:default
# The fixture changes: pin friction_iterations = 1 in the historical fixtures and
# generate the <name>-friction-defaults.json twins' references through the harness
python3 tools/ci03/make_fixtures.py --data data --unit-tests build/tests/unit_tests --output /absolute/fresh/dir/fixtures
# Compact published results
python3 tools/ci03/summarize.py --evidence /absolute/fresh/dir --out tools/ci03/results-YYYYMMDD.json
```

The P4 ball-bounce fixture needs a build with `POLYFEM_WITH_TRIANGLE=ON` (its
collision mesh is an irregular tessellation); the shared developer build has
it off. `ab_runner.py` writes only the top-level copied fixture: `default`
writes nothing (the copy is byte-identical to the pinned file, checked by
hash), `fiN` writes `solver/contact/friction_iterations`, `-diag` adds
`output/physical_diagnostics` and `output/manifest`.
`results-20260921.json` holds the 39 runs of the record's A/B (before
`91c7ff8ac`, after `11cdf476f`).
