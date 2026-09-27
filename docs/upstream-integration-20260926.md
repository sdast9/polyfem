# Small upstream integration — 2026-09-26

## Scope

Merge `polyfem/polyfem` main at `591b08bd5e115bdefec7e2998e98f5b56bc1367e`
(upstream #544) into fork main at `a0a40ce9250f678dda236c69363e5aa32914d998`.
This adds the `elastic_material_smoothing` optimization objective, which
penalizes relative jumps in neighboring selected elements' Lamé parameters,
with its JSON construction/schema support and 2D/3D derivative test. The
differentiability-data pin advances to
`e97645ba3e78f5b44c9a13f3b056b206d6810849`.

The six upstream files merge without modification or textual conflict. The
schema replaces the old `material_smoothing` placeholder with
`elastic_material_smoothing`. The objective uses Lamé-parameter ratios; this
integration does not extend its domain to zero denominator parameters.
Forward contact algorithms, model/controller defaults and companion pins are
unchanged: IPC Toolkit `75600955`, PolySolve `448f1b8`.

The larger IPC assembly/SIMD/broad-phase integration and PolySolve hybrid-solver,
nano-MPI and complete large-index/CI updates remain separate work. A clean
textual merge of PolySolve is not evidence that its new build defaults have
been validated for this fork.

## Validation

- Shared macOS arm64 RelWithDebInfo `PolyFEM_bin` and `unit_tests` rebuilt.
  Compiler: AppleClang 21; TBB; optimization and Python enabled; CUDA off.
- Affected selection `[opt_gradient],[optimization],[trim_controller]`, seed 1:
  **48 cases / 141 assertions pass**, including the new 2D/3D objective
  derivative checks and the newly synchronized EF-02/03 regressions.
- The four changed C++ files pass clang-format 21.1.8 dry-run with `--Werror`;
  staged whitespace check passes.
- Five public smokes, four steps each, run sequentially with one thread against
  the preserved EF-02/03 binary: all ten baseline/candidate runs exit zero and
  **all 25 exported VTU files are byte-identical**.
- Houdini 22.0.429 `tests/test_polyfem_hda.py` end-to-end check: **pass**
  (export, execution, import, solver-menu round trips and named refusals).
  The first sandboxed launch aborted before the test because Qt could not
  detect NEON; the approved unrestricted rerun completed successfully.

No full unit-suite or cross-platform CI result is claimed for this integration.
The preceding EF-02/03 record remains the evidence for the full suite's two
known golden-scene failures. No Teseo or private scene was run.

## Evidence

Parent-workspace directory: `outputs/upstream-integration/20260926/`.
The earlier source comparison is `outputs/upstream-review/20260926/REVIEW.md`.
Key files are `build.log`, `build-info-tested.json`, `affected-tests.log`,
`affected-test-names.txt`, `format.log`, `smoke-baseline-verification.json`,
`smoke-identity.log`, `smoke-identity/identity.json`, and
`hda-e2e-unrestricted.log` (the initial launch is retained in `hda-e2e.log`).
The smoke baseline is the preserved EF-02/03 final candidate `PolyFEM_bin-v3`;
its SHA-256 and all six feature-source snapshots were checked against the
published EF-02/03 record/current sources before comparison.
