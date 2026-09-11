# RB-15 — Feature-consistent coefficient assignment

Date: 2026-09-11. **Characterized—decision pending within the bounded fixtures.**
See the [validation record](rb-15-validation.md), [protocol](../tools/rb15/protocol.md)
and [measurements](../tools/rb15/results-20260911.json). No production law changes.

## Conditional energies and dimensions

Use the ordinary unweighted IPC squared-distance barrier with dmin=0 and dhat=1,
form weight=trim=1, unprojected derivatives and positive separation. As derived in
[RB-13](rb-13-contract.md), b has units L^4 and k has physical units force/L^3,
or weighted-objective/L^4 when the underlying objective is weighted. k is neither
a material modulus nor the barrier tangent stiffness. This experiment uses
synthetic positive driving Hessians, not an assembled material estimate. There is
no demand predictor, band controller, uncertainty enclosure or equilibrium solve.
The probe's coefficients establish assignment behavior only.

The current frozen feature-key rule is `E=sum_i k_key(i)(snapshot)*b_i(x)`.
Its coefficients are deterministic at a frozen snapshot but differ across nearest
feature keys. Each open feature region has energy derivatives; the whole energy
can jump. Existing cap, trim, weight, refresh and lag rules remain untouched.

For a declared parent interaction p, define

```text
E_p(x | snapshot, parent state) = k_p * b(d_p(x))
g_p = k_p grad(b_p)
H_p = k_p Hessian(b_p).
```

Here d_p is distance between the point and its entire closed segment in 2D or
triangle in 3D. A change among that primitive's closest subfeatures keeps the same
k_p. For these convex primitives, squared distance and its gradient agree at the
feature boundaries. At positive gap, the composed potential is C1; curvature may
jump. This does not require a continuous Hessian or justify a Hessian projection.

The tested shared coefficient is the current parent coefficient at the initial
snapshot: 70 for the 2D point/edge, 76.976744186 for the 3D point/triangle.
It is **not** an approved way to combine RB-14 mechanical estimates. Two disjoint
parents use independent 70 and 700 values; a global70 control shows the cost of
losing heterogeneous scaling. Global70 represents one frozen snapshot of a
global-coefficient scheme; no global adaptive update schedule is tested or selected.
Each parent supplies exactly one closest-feature
potential in these fixtures; arbitrary production collision multiplicity is not
assumed equivalent to this construction.

A second conditional energy uses a frozen affine relative tangential coordinate
`a(x)=A^T x+a0`, width w=.2*dhat, and two positive constants:

```text
t = a/w; u=tanh(t); delta=(k_right-k_left)/2
k(x) = (k_left+k_right)/2 + delta*u
grad(k) = delta*(1-u*u)*A/w
Hessian(k) = -2*delta*u*(1-u*u)*A*A^T/w^2
E = k*b
g = k*grad(b) + b*grad(k)
H = k*Hessian(b) + grad(b)*grad(k)^T + grad(k)*grad(b)^T
    + b*Hessian(k).
```

The frame A is a frozen relative point/anchor direction, so translations cancel.
It is not a corotational, dynamically transported field; rotational objectivity
under changing frames has not been established. Every derivative above is included
in the probe. Using only `k*grad(b)` while evaluating energy `k(x)*b` is the explicit
negative control. Freezing k at an outer iterate instead defines a different,
lagged constant-coefficient energy and needs a separate update ledger.

## Continuity and work comparison

The actual IPC forms use a 2D EV→VV crossing and a 3D EV→FV crossing. The original
RB-02/04 data and Fixed70 control are preserved. At left/right separation 1e-9:

| Candidate | 2D energy jump | 2D gradient jump norm | 3D energy jump | 3D gradient jump norm |
| --- | ---: | ---: | ---: | ---: |
| Current feature keys | -44.4977394 | 247.941971 | 20.6966230 | 99.8716492 |
| Shared parent | 0 | 5.81417e-6 | 0 | 5.53707e-6 |
| Global70 control | 0 | 5.81417e-6 | 0 | 5.03522e-6 |
| Complete smooth field | -2.22489e-7 | 5.27686e-6 | 1.03483e-7 | 5.29316e-6 |

The decreasing-epsilon sequences distinguish limits from a single small sample.
For shared parents the Hessian jump norms remain about 8263 (2D) and 6815 (3D);
these permitted curvature jumps do not invalidate the observed C1 limits.
Centered fixed-region finite differences have maximum relative gradient error
4.60e-11 and Hessian error 8.29e-11 over all four candidates and both fixtures.
Such interior agreement does not rescue the current boundary discontinuity.

Split midpoint quadrature across the transition, at 1024 panels per side, gives
`Delta E - integral(g dot dx)`:

| Candidate | 2D residual | 3D residual |
| --- | ---: | ---: |
| Current | -44.4977388 | 20.6966222 |
| Shared parent | 7.98e-7 | -8.77e-7 |
| Global70 | 7.98e-7 | -7.98e-7 |
| Complete smooth | 1.17e-6 | -1.05e-6 |
| Smooth omitting grad(k), 2048 total panels | -20.2661418 | 9.42611194 |

This is gradient-path work on a frozen conditional potential, not a full dynamic
energy budget. Physical contact-force work has the opposite sign. No reactions,
FEM residuals, det(F), engineering accuracy or nonlinear termination are measured.

## Neighborhood, multiplicity and material limits

The benefit holds while parent identity and its coefficient are fixed. Switching
from a 70 parent to a 55 parent at the same positive gap produces -44.4977394
energy change. Adding a duplicate 70 description produces +207.6561172. Thus a
region boundary or duplicate parent can recreate the problem. These are algebraic
counterexamples evaluated with a real IPC barrier; they are not a production
neighborhood construction or duplicate-elimination implementation.

For two duplicate descriptions of the **same** distance and equal coefficient,
a partition of unity gives `sum w_p*k*b=k*b`, independent of the weights. The
probe verifies its energy arithmetic for five weights. Unequal coefficients or
different distances need full weight derivatives and a new interaction model;
this equal-description control does not validate arbitrary mesh seams.

Two actual disjoint IPC parents with k=70/700 give energy 2284.21728935. Separating
the second gives 207.65611721; recontact restores the original. Their simultaneous
EV/VV crossing has zero measured energy jump and gradient difference 5.84317e-5
at epsilon=1e-9. Parents are identified by immutable vertex groups only in this
small probe. New subfeatures inherit a known parent's coefficient. For a new
unknown parent no general assignment contract is implemented: a complete parent
registry or an explicit outer discovery/update is still needed.

Finite k contact appearing at support has vanishing b and grad(b), verified with
decreasing distances to dhat. Discovery at an already-active positive gap is a
different event and cannot silently add/remove potential. Separation/recontact,
reversed trial order within actual candidate intervals, and current empty-snapshot
new-contact evaluation are deterministic in the tested cases. Same-x identical
snapshot refresh preserves energy. Doubling the driving Hessian at identical x
instead doubles energy: +207.6561172 (2D), +228.3527402 (3D), with zero displacement.
That is an explicit outer parameter-energy event under the RB-04 convention.
Shared-parent replacement must follow the same ledger, not count it as work.

Merging coefficients across soft and hard parents is a material-model choice.
For prescribed coefficients with ratio R and arithmetic shared mean, soft force
is scaled by (1+R)/2 and hard force by (1+R)/(2R). At R=1000 these are 500.5 and
.5005; global70 scales the hard force by .001. Separate parents preserve their
specified force scales. These are exact coefficient/force ratios, not measured
continuum modulus errors or evidence that mean/min/max is the correct estimator.

## Force direction, mapping, units and cost

For a constant positive parent coefficient and positive interior gap, the point
force follows the repulsive distance normal. For variable k,

```text
F_point dot n = k*f(d) - b*grad_point(k) dot n.
```

Positive k is insufficient to guarantee repulsion. The modest transition fields
have sampled minimum normal force 662.5306 (2D), 779.8169 (3D), but create tangential
forces as large as 111.2443 and 51.7416. A separate positive field varying from
1 to 1000, width .02 and center a=.08, gives minimum sampled normal force
**-20412.12993**. Its gradient term overcomes repulsion. An earlier version centered
at a=0 remained repulsive and is retained as a failed attraction assertion, not
hidden. Smooth energy consistency alone cannot select the desired contact force.

For a constant map y=B*x+rest, the complete smooth derivatives pull back as
`g_x=B^T*g_y`, `H_x=B^T*H_y*B`, including the field derivatives. The real-form exact
permutation test passes, as does current exact-selector coefficient sampling.
This does not select an interpolated mechanical estimator from RB-03.
Under L'=Lscale*L and objective'=Q*objective, convert
`k'=Q/Lscale^4*k`, `w'=Lscale*w`, and `Hdrive'=Q/Lscale^2*Hdrive`.
Nine conversions (.001,1,1000 by .01,1,100) preserve the tested energies and
forces for frozen current/shared/global/smooth assignments; smooth Hessians also
match. Controller conversion is excluded and its known RB-02 issue remains open.

Warm median evaluation times (seven batches, 100 evaluations each) are 19.9/20.7/
19.7/21.1 microseconds in 2D and 28.7/27.2/25.9/29.5 in 3D for current/shared/global/
smooth respectively. Timings include real collision rebuild, energy, gradient and
Hessian, but exclude mechanical estimation, parent construction, initialization,
solver iterations and scene throughput. These tiny differences are not a speedup
claim. Frozen parent lookup is O(1) per known interaction once assigned; registry
construction/storage on real meshes was not measured. The probe's dense rank-one
smooth Hessian operations are not a proposed production implementation.

## Recommendation and decision handoff

Prefer **frozen shared coefficients on stable parent interactions** as the next
RB-16/17 candidate, conditional on an explicit parent/multiplicity definition.
It preserves normal repulsion for positive k, removes both tested subfeature jumps,
and needs no geometry-dependent force correction. A naive patch merge, nearest
region switch or duplicate summation is inadequate. Do not yet claim general
3D transition handling: EE/mollification, adjacent primitives, changing topology,
curved/high-order proxies and moving material frames remain uncharacterized.

Before production implementation, choose the parent construction and seam/multiplicity
policy, how RB-14 estimates attach to it, and how unknown parents and refreshes are
handled outside a frozen solve. The smooth-field alternative additionally needs
an acceptable tangential/normal-force contract and frame invariance. Global adaptive
scaling remains a comparison control; it is not selected as a replacement.
RB-16 may use the bounded parent candidate for investigation while retaining these
limits. RB-17 integrated production comparison needs the explicit model decisions.
