# Next bounded coherent-coefficient comparison

2026-09-09. This is a predeclared experiment, not a new production law.
The user chose the coherent-model direction; RB-04 observations now distinguish
coefficient-state and friction-lag bookkeeping from equilibrium under each state.

## Candidate baseline and units

Use existing Fixed barrier mode with one strictly positive constant coefficient
as a comparison to the semi-implicit per-feature curvature coefficients.
Let a be the acceleration/form weight and s the form normalization (currently 1):

```
Phi_fixed = (a/s) * k_fixed * sum_i b_i(x)
Phi_semi  = (a/s) * trim * sum_i k_i * b_i(x)
```

For ordinary clamped-log b, b has units L^4. k_fixed has weighted-objective
units J/(a L^4); in physical energy units it is force/L^3. In the tested time
loop physical energy is Phi/a, so holding k_fixed fixed gives the same physical
barrier across dt changes when s=1. Do not rescale k_fixed by dt merely because
the weighted objective changes. Semi-implicit k_i may depend on dt through the
inertial driving Hessian even after dividing out a.

A read-only Luna audit confirmed the Fixed/SemiImplicit construction and the
mode differences below. Its warning about retaining a applies to comparison of
weighted objectives, not a requirement to rescale a fixed physical coefficient
under dt refinement. Source: Form::value, ContactForm::weight, BarrierContactForm
constructor/assignment and SolveData numeric stiffness setup/update_dt.

One global coefficient removes the specific per-feature 70-to-55 coefficient
jump and does not require an interpolated driving-curvature definition. It does
not automatically establish continuity of every collision potential, valid proxy
geometry, physical calibration, mesh independence or good conditioning in
heterogeneous materials. Those remain measured limits, not implied benefits.

## First fixture, before any scene-scale choice

Reuse the geometry and positions of the RB-02 EV/VV transition probe. Hold
ordinary clamped-log, dhat=1, a=s=1, identity mapping, unprojected Hessians,
no area weighting, no improved max, no physical-barrier variant and no friction.
Compare:

1. Current semi-implicit snapshot: EV coefficient 70, VV coefficient 55.
2. Fixed k=70: a declared match to the EV-side coefficient, not physical truth.

Use the original decreasing left/right separations and extend only if needed
to identify a plateau versus a vanishing difference. Report energy and gradient
jumps and their scaling. A finite Hessian jump at a closest-feature boundary
need not violate a C1 potential; do NOT require Hessian continuity blindly.
Retain fixed-region finite differences and a k=100/H=100I scaling control.
Check equivalent length/energy conversions using k'=Qscale/Lscale^4*k for
unweighted physical energy. Preserve CCD independently; no new floor or cap.

This is direct form evaluation, so trial-cap/sequential-clamping policy cannot
confound the result. Use the existing RB-02 standalone compiler/link recipe and
fresh evidence. No production source edit is required for this fixture.

## Follow-up scene comparison boundaries

Declare a reference endpoint and scale-selection rule before executing scenes.
For example, matching trim times a measured per-contact median is an objective
normalization at one state, not calibration against analytical physical truth.
The current endpoint stream records ranges, not the median; do not invent it
from min/max or silently fit the final reaction. A uniformly scaled microfixture
or an explicitly measured reference scale can avoid that ambiguity initially.

Fixed mode lacks the semi-implicit trial-displacement cap/sequential clamp.
Either establish that the cap is inactive in the selected scene comparison or
label completion/timing as a mode-bundle comparison. Friction initially stays
off for the normal-law comparison; its normal-force lag state is a separate axis.
Keep area/physical/max variants identical. Fixed-mode force pullback still uses
the actual collision/FEM map and must retain valid mapping assumptions.

Combine this comparison with fixed-snapshot contact-path quadrature and an
upper-band unloading fixture. Existing data justify state-identity requirements
and keeping .5/.9 as a working comparison baseline, not choosing a universal
sweet spot or certifying a new constant coefficient.

## Predeclared boundary-work follow-up

After the first fixture passed (66 checks), integrate the point-x gradient along
x=.9 to 1.1, with all other coordinates fixed and the original center snapshot.
Split midpoint quadrature at the feature boundary x=1; use 16, 64, 256, 1024
panels per side, avoiding an arbitrary derivative convention at the boundary.
Compare the integral with endpoint energy change. Hypothesis: Fixed k=70 error
converges to zero; Semi error tends to the independently measured energy jump.
This is a local contact-path test, not full trajectory or friction certification.

## Completed fixture and candidate determination

The first fixture passed 66 checks; the extended work fixture passed 68 checks.
Evidence: parent `outputs/rb-04/20260909-fixed-candidate/` and
`outputs/rb-04/20260909-fixed-candidate-work/`, including exact source copies,
compiler/link commands, executable hashes, raw JSON/stderr and exit records.
Compiled against the existing tested build at production commit `e652fae53`.
No production law, configuration or dependency changed in this fixture stage.
Portable results: [candidate-results-20260909.json](../tools/rb04/candidate-results-20260909.json).

At epsilon=1e-9, Semi retains energy jump -44.4977394 and gradient jump norm
247.9419714. Fixed k=70 gives energy difference 0 at floating-point precision
and gradient difference norm 5.81417e-6; the latter shrinks approximately linearly
with epsilon. At epsilon=1e-3 its energy difference was -0.00204539, shrinking
quadratically until roundoff. Both interior finite-difference checks passed:
gradient relative error 5.95e-11, Hessian relative error 3.96e-7. Uniform k=100
matches the H=100I Semi control, and all nine length/energy unit conversions
pass for energy, force and Hessian.

| Midpoint panels per side | Semi: energy change minus gradient integral | Fixed k=70: same mismatch |
| --- | ---: | ---: |
| 16 | -44.49517139 | 0.003268374 |
| 64 | -44.49757896 | 0.000204202 |
| 256 | -44.49772938 | 0.000012762 |
| 1024 | -44.49773878 | 0.000000798 |

This isolates a persistent missing jump contribution in the existing featurewise
energy/gradient contract. It is not resolved by tighter smooth-region quadrature.
The fixed candidate is consistent on this specific path to the tested accuracy;
it is not a proof for all collision configurations. The inherited raw `trim`
field is global barrier stiffness in this standalone helper: for Fixed it is
k_fixed, not an active trim controller.

**Candidate determination:** advance a strictly positive, fixed scalar coefficient
multiplying the ordinary barrier potential as the coherent comparison model.
Hold it fixed over a trajectory and retain exact collision/FEM pullback. Solver
heuristics may assist minimization, but changes to physical coefficients must be
explicit model changes with energy accounting. The present curvature-dependent
feature coefficients and adaptive trim cannot be treated solely as optimization
heuristics: they change energy and forces. A fixed coefficient also avoids
choosing interpolated driving curvature, but does not validate arbitrary proxy
geometry or replace RB-03 mapping requirements.

This selects the next model family for comparison, not a production default or
universal numerical coefficient. k=70 is only the predeclared EV-side fixture
normalization. Keep .5/.9 as the current comparison band; available data do not
establish an optimal lower/upper band.

## Exact next experiment and unresolved decisions

1. Select a public frictionless, uniformly scaled contact fixture and declare a
   coefficient reference rule before running. A directly measured reference
   coefficient is required; endpoint min/max do not determine its median.
2. Compare physical force/displacement and barrier energy under load-increment
   refinement at fixed physical k. Record gap, strain, residual and determinant
   checks. Account for Fixed/Semi solver-policy differences explicitly.
3. Add loading/unloading that actually exercises the upper trim bound. The
   earlier compression-only data cannot identify an upper-band sweet spot or
   establish absence of path-dependent energy from retuning.
4. Keep the solved friction-lag state associated with its endpoint. Compare lag
   refinement separately; do not silently tighten its production acceptance.

No scene-scale k, interpolation redesign, lag budget, convergence gate, CCD or
trial-cap change has been selected or implemented. RB-04 remains in progress.
