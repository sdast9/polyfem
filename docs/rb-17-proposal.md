# RB-17 — Candidate selection and integration protocol

2026-09-11. **Experimental package approved; implementation/validation pending.**
The user requested completion of RB-17. The item's explicit prerequisite is a
selected estimator, assignment and controller contract. RB-14–RB-16 characterize
alternatives but do not select them. This document makes that choice reviewable.
The user subsequently instructed: "Please proceed with this proposal."
That authorizes the five-point experimental package below, including its
bounded unresolved-attempt policy. Production default promotion remains separate.
See [status and provenance](rb-17-validation.md).

## Recommended bounded experimental candidate

1. **Estimator:** fresh full reduced noncontact tangent compliance and shared
   residual prediction, using RB-14's `H z=J^T`, `H y=r`,
   `K=1/(J z)`, `p=d-J y`. Factor once per outer state and charge all normal RHS
   solves to cost. Require finite SPD supported data; use exact-selector P1 maps
   first. This is a local quadratic prediction of nonlinear mechanics, not a
   certified enclosure. No empirical error interval becomes a guarantee.
2. **Assignment:** one immutable coefficient per explicitly declared point/closed
   segment parent in the first 2D fixtures. Exactly one closest-feature potential
   per parent; EV/VV subfeatures inherit it. Predeclare the complete supported
   interaction registry, reject duplicate descriptions, and retain inactive
   parents for recontact. Unsupported seams, unknown parents, EE, interpolation
   and 3D production geometry report unsupported scope instead of receiving an
   invented coefficient. Do not omit any physical collision pair to fit the
   registry: use fixtures whose complete collision geometry is supported and
   retain the complete CCD geometry. A fixture that cannot meet this condition
   stays pending until an explicit extension is defined.
3. **Timing:** freeze coefficients for each full reduced solve. At the same load,
   recompute predictions and correct out-of-band applicable parents, with factor
   2 bounds, one converged observation, no hysteresis and at most 12 corrections.
   Compare factor 4 separately. Restart solver derivative/history state after an
   event. Publish coordinates and their solved coefficient together, without a
   post-publication change to that force state.
4. **Target:** retain the reference distance band
   `L=dhat*sqrt(.5)`, `U=dhat*sqrt(.9)`. For valid point predictions p<=U, use
   `k_lo=K*max(L-p,0)/f(L)` and `k_hi=K*(U-p)/f(U)`, with `f=-b'`.
   Target the geometric center when both bounds are positive, half the upper
   bound when only the lower is zero, and clip each update by the selected factor.
   A zero-only or otherwise unusable interval does not install zero stiffness.
   In ordinary unweighted IPC units k is force/length^3; convert to the actual
   weighted form exactly once. This scalar rule has no coupled-band guarantee.
5. **Protection:** initialize a parent from its fresh positive mapped baseline
   coefficient at the declared initial snapshot, before any trial evaluation.
   Preserve that parent's last positive coefficient on separating/zero-only
   demand. Record that retention need not enforce the band. Invalid estimates,
   nonpositive initialization, unsupported geometry or an exhausted correction
   budget end the experimental attempt with an explicit unresolved result. Do
   not invent a positive floor, restore failed attempts or retry automatically.

All five points are the selected experimental package, including its bounded
stop/report behavior. Current production defaults, CCD, trial-displacement cap,
AL feasibility role and configured numerical convergence remain as specified in
the robustness plan. The proposed mode is frictionless and opt-in. Its unsupported
scope is an explicit applicability boundary, not general scene support.

## Alternatives and measured basis

| Choice | Benefit | Cost or unresolved limitation |
| --- | --- | --- |
| Recommended full tangent plus shared prediction | Avoids the measured missing remote predictor contribution; simplest reference to audit | Global factorization and per-parent solves; nonlinear and coupled errors remain |
| Physical neighborhood compliance plus shared prediction | Candidate for cheaper normal response estimation | Local stiffness error remains; RB-14 coverage missed one prior challenge; global predictor factor still costs |
| Frozen shared parents | Removes the two tested subfeature jumps while preserving normal repulsion for positive k | General seam/discovery/multiplicity rules are not solved |
| Complete smooth coefficient field | Consistent energy derivatives when all terms are included | Adds tangential forces and can become attractive; frame policy unresolved |
| Same-load corrections | RB-16 improves tangent-applicable occupancy and publishes the solved force state | Extra solves; bounds are conditional and correction budget may exhaust |
| Existing callback timing | Preserves baseline cost and behavior for comparison | RB-16 retains band violations and measures a post-publication force-state change |

These are interpretations of checked-in RB-14–RB-16 evidence, not new RB-17
measurements. Full tangent is recommended for this bounded integration first;
cheaper locality remains an ablation rather than an unverified speed claim.

## Predeclared comparison structure after selection

Keep the current Adaptive and SemiImplicit modes distinct. The latter is the
primary baseline for the existing Hessian/gap-feedback method. Include Fixed as
a reference with its actual trial-cap differences labeled. For experimental
ablations compare baseline-estimate versus full-tangent estimate, feature-key
versus shared-parent assignment, and callback versus frozen same-load timing.
Use the resulting 2x2x2 matrix where contracts are compatible. Mark a genuinely
incompatible combination explicitly and provide a matched control; do not label
a multi-change comparison as a single-factor result. Calibrate no held-out case.

Before executing, commit an exact case manifest with physical dimensions,
materials, loads, velocities, meshes, integrator histories, numerical settings,
initial coefficients, parent registry and measured quantities. Start with scalar
spring references, public compression/loading-unloading, two heterogeneous bodies,
a true time-integrated approach, and mechanically coupled contact. Use complete
supported geometry. Keep physical loads fixed through refinement; do not repeat
RB-16's mesh-dependent K0 forcing as continuum refinement. Select at least one
held-out combination before inspecting results. Cross-feature paths and reversed
evaluation order must test the same frozen parent state. Include length-unit
conversions .001/1/1000 with all dimensional quantities converted consistently.

The coupled fixture must measure off-diagonal `J H^-1 J^T` response and compare
to an independently solved coupled reference. Independent scalar formulas are
candidate estimates there, not an exact coupled solver. Any model extension
needed to respond to a coupled failure is a separate documented decision.

Use RB-04 diagnostics for endpoint coefficient identity, physical free residual,
BC error, reactions/displacements, individual and aggregate gaps, det(F), and
parameter-energy events at zero displacement. Measure dynamic kinetic energy
from the actual integrator state. Retain path quadrature resolution/error and
applied-load work limitations. Include partial/failed trajectories, peak memory,
wall time, factorization/RHS costs, Newton steps and correction counts. Separate
numerical termination from observed physical accuracy and band occupancy.

Preserve direct production objective arithmetic. RB-16's gradient-integral
arithmetic remains a separately labeled research control if used; it is not an
authorized production line-search repair. No baseline stall may be erased by
substitution of that control.

## Completion gate

After selection and implementation, rebuild production targets; run new mode and
negative-scope regressions, affected baseline tests, all five public smokes and
HDA E2E. Verify default-mode compatibility, supported discovery/feature behavior,
frozen derivatives, unit conversion, order independence and endpoint accounting.
Keep experimental failure cases in the decision report. Publish the opt-in code,
case manifest, compact evidence and accuracy/robustness/cost comparison. Recommend
defaults only from those results; promotion still needs the user's decision.
RB-09 broad refinement, RB-10 friction and RB-12 release promotion remain separate.
