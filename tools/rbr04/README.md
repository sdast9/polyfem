# RBR-04 seam characterization

The improved-max (convergent) collision set combined with the semi-implicit
parent-keyed coefficient law, on the real `BarrierContactForm`
([RB-21 record](../../docs/rb-21-parent-keyed-kappa.md#rbr-04--the-improved-max-operator-is-a-checked-restriction-2026-09-21),
[repair plan](../../docs/rb-review-repair-plan-20260920.md#rbr-04--enforce-the-parent-law-supported-formulation)).

Build the existing `unit_tests` target, then run from the PolyFEM repository
with a **fresh absolute output directory**:

```bash
python3 tools/rbr04/run_probe.py --build build --output /absolute/fresh/evidence --expect-guard
```

The standalone probe reuses the configured unit-test compile/link recipe. It
constructs the form on the `[kappa_continuity]` corner fixture (two edges
meeting at a vertex, a free vertex crossing the corner without a refresh, a
frozen heterogeneous Hessian so the two edge parents carry unequal
coefficients) and crosses both seams of the corner with offsets 1e-2 … 1e-9,
reporting the constructed flags, the signed collision weights and parents,
the assigned coefficients, the energy/gradient on either side and the
limiting behaviour (a finite jump versus variation proportional to the
offset). The measured jump is checked against
`(Σ weight·scale right − Σ weight·scale left) · b(d)`.

With `--expect-guard` (the published state) the five improved-max
semi-implicit configurations must be refused by name at construction and
every control row (equal coefficients under the toolkit's potential, the
non-convergent default, area weighting alone, the stencil identity,
`fixed`/`adaptive` with improved max) must keep its behaviour. Without the
flag the probe is the failing control that was run on the unrepaired source
`b6d0c9a45` (`outputs/rbr-04/20260921T091854Z/baseline/seam-probe/`): the
target configurations construct and show the finite `|κ₁−κ₂|/2·b(d)` jump.

Not registered in the Catch suite; the permanent regressions are
`unit_tests "[rbr04]"` (`tests/test_kappa_continuity.cpp`,
`tests/test_input_validation.cpp`). No scenes run. Assertions characterize
the law; a passing probe is not contact-model certification.
