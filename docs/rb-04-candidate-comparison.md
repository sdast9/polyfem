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
