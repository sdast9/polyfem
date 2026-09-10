# RB-13 — Mechanical coefficient reference, bounds and scope

2026-09-09; extended 2026-09-10. **Stages 1–3 characterized; production decisions pending.**
This is a derivation and standalone analytical probe, not an installed coefficient
law. Section 7 establishes the conditional intervals; section 8 characterizes
their mechanical scope and counterexamples. See [validation](rb-13-validation.md) and
[reproduction](../tools/rb13/README.md).

## 1. Coordinates, predictor and compliance

Use one frictionless contact with physical separation d, zero minimum distance,
and a fixed closest feature/normal. Let z be free displacements, measured in
length units, and x = Q z + c(t) the full displacement. Prescribed motion c at
the new time/load belongs in the predictor before condensation. It is not a free
unknown and must not contribute artificial compliance through obstacle entries.
With the [RB-03 map](rb-03-contract.md), y = y_rest + B x and
J = (partial d / partial y) B Q. J is a nonzero, dimensionless row vector.
An exact selector, permutation or fixed linear interpolation admits this chain
rule; the latter does **not** validate production's legacy interpolated-stencil
Hessian sampling. This reference uses the full free-coordinate solve instead.

Let Psi_nc be the noncontact incremental objective in physical-energy units.
Around z0, with new prescribed values already applied, define

```text
Psi_nc(z0 + dz) = constant + g^T dz + (1/2) dz^T H dz
H dz_free = -g
z_free = z0 + dz_free
d_free = d(z0) + J dz_free
```

For a quadratic objective and affine gap these are exact; otherwise they are a
local tangent prediction. In particular d_free may be negative: it is an
extrapolated gap, not a geometry at which to evaluate the barrier or disable CCD.
Assume H is symmetric positive definite on the admissible free space, with finite
entries, stable linear response, fixed loads/predictor and no friction, damping,
other contacts, feature transitions or changing coefficients during evaluation.
These are assumptions, not properties guaranteed by the current provider.

Impose gap d and minimize the quadratic response about z_free. A repulsive
normal force F does work F J dz, so stationarity gives

```text
H (z - z_free) = J^T F
H v = J^T
C_eff = J v                     [length / force], strictly positive
K_eff = 1 / C_eff               [force / length]
z - z_free = v F
d - d_free = C_eff F
min Psi_nc at gap d = constant + (1/2) K_eff (d - d_free)^2
```

This proves K_eff = 1/(J H^{-1} J^T); the notation denotes a solve, not an
instruction to form an inverse. All admissible displacement components relax
in that solve. Using a principal stencil block, prescribing a contact direction,
or allowing constrained DOFs to move generally defines a different mechanical
problem. The current Rayleigh-quotient law is not this condensation. Section 8
compares supported springs, rigid limits, directional curvature and coupled contacts.

If H is singular/indefinite, or J is zero on the free space, this contract cannot
supply a finite positive compliance. The probe rejects nonpositive pivots; it
does not introduce a pseudoinverse, PSD projection or numerical stiffness floor.
All-prescribed contact instead needs a compatibility/reaction assessment.

## 2. Coefficient, repulsive force and spring stiffness

For ordinary unweighted contact energy E_b = k b(d), define f(d) = -b'(d).
Inside support, f is positive. The scalar equilibrium is

```text
K_eff (d - d_free) = k f(d).
F_needed(d) = K_eff max(d - d_free, 0)
k_estimate = F_needed(d_target) / f(d_target),  0 < d_target < dhat.
```

The estimate places the equilibrium at d_target when d_target > d_free under
the stated reference model. The positive-part expression is a **repulsive demand
diagnostic**, not a replacement noncontact force law: below d_free, achieving
the target would require attraction and k=0 cannot place the equilibrium there.
If d_target=d_free, k=0 recovers the free equilibrium. If there is no predicted
compression, a zero estimate does not authorize removing active-contact protection.
An upper-gap requirement must not pull a separating body into contact. Section 7
establishes conditional root/band semantics and inactive/empty cases.

The three quantities have distinct dimensions:

| Quantity | Meaning | Ordinary unweighted units |
| --- | --- | --- |
| k | Coefficient multiplying b | force / length^3 |
| k f(d) | Repulsive normal force | force |
| k b''(d) | Barrier's scalar tangent spring stiffness | force / length |

K_eff is the **noncontact** spring stiffness. It is neither k nor k b''.
The condensed combined tangent is K_eff + k b''. For a non-affine gap,
the full barrier Hessian also contains k b'(d) Hessian(d), in addition to
k b'' J^T J; the scalar spring expression omits that geometric term by assumption.
Retuning k at fixed d changes the objective by Delta k b(d) with zero displacement.
It is a coefficient-state change, not force-displacement work; the
[RB-04 convention](rb-04-work-convention.md) remains applicable.

## 3. Exact conversion from the implemented squared-distance barrier

In IPC's `barrier.cpp`, the argument named `d` is a generic scalar. The
`BarrierPotential` caller supplies squared separation s = d^2 and support
D = dhat^2 for dmin=0. In this section d always means physical separation:

```text
B(s; D) = -(s-D)^2 log(s/D),                    0 < s < D
B_s = (D-s) [2 log(s/D) - D/s + 1]
B_ss = (D/s + 2) D/s - 2 log(s/D) - 3
b(d) = B(d^2; dhat^2)
f(d) = -2 d B_s
b''(d) = 2 B_s + 4 d^2 B_ss.
```

B, B_s, B_ss have units length^4, length^2 and dimensionless; hence f has
length^3 and b'' has length^2. With r=d/dhat an independent expression is

```text
b = -2 dhat^4 (r^2-1)^2 log(r)
f = dhat^3 [8 r (r^2-1) log(r) + 2 (r^2-1)^2/r]
b'' = dhat^2 [-8 (3r^2-1) log(r) - 14r^2 + 12 + 2/r^2].
```

At d=.5, dhat=1, b''=15.1137056388801. Omitting 2 B_s gives
23.7725887222398, a demonstrably different stiffness. This is a negative control
for the proposed derivation, **not a newly found production bug**: the existing
IPC derivatives already apply the distance chain rule.

Outside support B=0; at nonpositive argument its value is infinite. Derivative
sentinels returned there by IPC are not a finite physical contact law. The
reference only evaluates positive interior gaps, with no force saturation.

For nonzero dmin, the actual caller uses s=d^2-dmin^2 and
D=(2 dmin+dhat) dhat; activation occurs at d=dmin+dhat. The same derivatives
use the total separation d. If clearance c=d-dmin is used instead, factors
2d become 2(c+dmin), not 2c. Nonzero-dmin behavior is source-traced here, not
part of the numerical fixture or an approved controller change.

## 4. Weights and normalizations

For a frozen stencil with cached scale k_cache, global trim t, form acceleration
weight a and normalization sigma, collision weight omega and mollifier m(x),
the actual ordinary potential contributes

```text
Phi_b = (a t / sigma) omega k_cache m(x) B(s; D).
```

The reference takes omega=m=1, sigma=1, and absorbs trim into physical k.
For constant omega,m, effective physical k = t omega m k_cache / sigma after
dividing Phi by a. To prescribe a physical effective k in that case requires
k_cache = k sigma/(t omega m), if the denominator is nonzero. When omega carries
area/length units, k_cache has units force/(length^3 [omega]); the unweighted
coefficient units cannot be assigned indiscriminately to weighted stencils.
For variable m, force and Hessian include its derivatives; absorbing it as a
constant coefficient is invalid. No area-weighted or mollified fixture is claimed.

`Form` multiplies derivatives by weight()/sigma; `ContactForm::weight()` returns
a*t. `NormalPotential` multiplies collision.weight, stiffness_scale and mollifier.
`NLProblem::normalize_forms()` currently returns 1. If individual form scales
change, recover the unnormalized terms separately before interpreting a physical
H; there is not necessarily one common correction factor. The probe's nonunit
weight/trim/normalization control verifies the scalar algebra only.

The alternative `use_physical_barrier` branch multiplies B by
dhat / Barrier::units(D), which is dhat/D^2 for ClampedLogBarrier (dhat^-3
when dmin=0). That changes the dimensional convention. Semi-implicit mode rejects
that branch; it and the separate experimental `gap_floor` are excluded here.
The retired constraint floor remains absent.

Under numerical conversion length'=lambda length and force'=mu force:
H'=mu/lambda H, b'=lambda^4 b, f'=lambda^3 f,
k'=mu/lambda^3 k, and (k b'')'=mu/lambda (k b''). Here primes on whole
quantities denote unit conversion, not differentiation. The barrier probe checks
dimensional normalization over dhat=.01, 1, 100; section 8 adds equivalent-unit
mechanical checks, without claiming production controller covariance.

## 5. Integrator scaling and predictor history

With unit form normalizations, the implemented second-order mechanical objective
has the following terms when enabled:

```text
Phi_nc(x) = a [E_elastic(x) + E_load(x)] + (1/2)(x-x_tilde)^T M(x-x_tilde)
Psi_nc = Phi_nc/a
H_phys = Q^T [K_tangent + M/a] Q
g_phys = Q^T [grad(E_elastic+E_load) + M(x-x_tilde)/a].
```

Here K_tangent is the Hessian of E_elastic+E_load; for dead loads it reduces
to the elastic tangent. The provider below omits any nonzero load Hessian.
Psi has physical-energy units but includes an algorithmic inertial potential;
it is not total stored mechanical energy. Derive d_free from its residual and
new prescribed offsets, not just x_tilde. Even a dead load with zero Hessian and
approach velocity with unchanged H can substantially change this predictor.
If inertia is disabled (including quasistatic configurations retaining an
integrator), omit M/a even though a may remain a time-integrator scale.

The currently implemented displacement integrators give:

| Second-order integrator | a | x_tilde |
| --- | --- | --- |
| Implicit Euler | dt^2 | x_n + dt v_n |
| Newmark | beta dt^2 | x_n + dt v_n + dt^2 (1/2-beta) a_n |
| BDF, current history order q | (beta_q dt)^2 | sum alpha_i x_i + beta_q dt sum alpha_i v_i |

BDF uses the currently available history order, including startup; beta_1=1,
beta_2=2/3. Gamma affects Newmark velocity reconstruction and velocity-dependent
terms, although it is absent from this undamped displacement predictor/tangent.
Positive dt and a are required. This table is traced to the actual integrator
source; the probe tests the derived quadratic algebra for Euler, Newmark beta=.25
and BDF2, **not execution of their C++ classes or a full time trajectory**.
First-order Euler uses a=dt; first-order BDF uses a=beta_q dt. Both use
position-only predictor history. They are not the second-order mechanical mass model;
their operator units must be specified separately before reusing this contract.

The current `SolveData` provider already returns weighted elastic plus enabled
inertia Hessians. At unit normalizations, H_provider/a already contains K+M/a;
adding mass again is wrong. PolyFEM deliberately passes zero local masses to
IPC's coefficient routine for this reason. Current assignment divides its
Rayleigh curvature by dhat^2 and then a, with additional caps/trim; it does not
calculate K_eff or predicted demand.

The provider omits body/pressure Hessians, damping, contact, friction and AL.
Dead loads have zero tangent, but displacement-dependent pressure/load terms may
not. ElasticForm can supply a PSD-projected Hessian, which is not necessarily
the physical tangent. Thus a provider-based estimate is approximate when those
terms matter, when the material/geometry is nonlinear, or when the supplied
curvature is projected. This stage neither changes that provider nor chooses a
projection, constitutive model or damping treatment.

## 6. The current controller uses squared-gap statistics

`NormalCollisions::compute_avg_distance` computes the **unweighted arithmetic
mean of squared distances** of collisions with d_i^2 <= dhat^2. It is neither
mean separation nor a force/area-weighted mean. Empty selection returns infinity.
`compute_minimum_distance` returns the minimum squared distance in the current
collision set, with infinity for an empty set. `collapse_severity` uses

```text
severity = min(mean(d_i^2 over active collisions), 100 min(d_i^2))
upward comparison: severity < trim_lower * dhat^2
downward comparison: mean(d_i^2) > trim_upper * dhat^2.
```

For one contact only, a physical band [d_L,d_U] corresponds to
trim_lower=(d_L/dhat)^2, trim_upper=(d_U/dhat)^2. Defaults .5 and .9 mean
[.7071067812,.9486832981] dhat, not [.5,.9] dhat. For multiple contacts the
lower test can also respond to min(d)<sqrt(trim_lower/100) dhat; an RMS/mean
criterion does not enforce the same per-contact physical band. For gaps .01 and
.8 at dhat=1, mean squared gap=.32005 and severity=.01. Cadence, calibration and
trim limits further mean these thresholds do not guarantee a final gap interval.
Stage 2's isolated-contact band must not be advertised as the current controller's
multi-contact guarantee. No JSON setting or production default changed.

## Source authority and pending work

Audited at PolyFEM `f9de2eb07`, IPC `af317a65`, PolySolve `4d372fa8`:

- [SolveData](../src/polyfem/solver/SolveData.cpp): provider and update_dt.
- [InertiaForm](../src/polyfem/solver/forms/InertiaForm.cpp),
  [Form](../src/polyfem/solver/forms/Form.hpp),
  [ContactForm](../src/polyfem/solver/forms/ContactForm.hpp),
  [BarrierContactForm](../src/polyfem/solver/forms/BarrierContactForm.cpp).
- [Euler](../src/polyfem/time_integrator/ImplicitEuler.cpp),
  [Newmark](../src/polyfem/time_integrator/ImplicitNewmark.cpp),
  [BDF](../src/polyfem/time_integrator/BDF.cpp).
- Effective IPC checkout: `src/ipc/barrier/barrier.cpp`,
  `src/ipc/potentials/barrier_potential.cpp`, `normal_potential.cpp`,
  `src/ipc/collisions/normal/normal_collisions.cpp`. Local provenance hashes them.

Stage 2's monotonic force-balance roots and conditional intervals are now
derived below, including enclosing-input uncertainty, empty intervals and inactive demand.
Stage 3's spring series/rigid limits, differing speeds, unit conversion,
nonlinear prediction error, Rayleigh comparison and multi-contact coupling are
characterized in section 8. No production k, global/local replacement, estimator approximation,
uncertainty enclosure or controller timing policy has been selected.

## 7. Stage 2 — conditional bands (2026-09-10)

Retain the stage 1 model and units, and abbreviate K=K_eff, p=d_free, h=dhat.
Let 0<L<U<h, K>0 and k>=0. The reference interval concerns a **single physical
gap**, not the production controller's mean/minimum statistic. All coefficient
weights are frozen and absorbed into k. No estimator, retune timing or fallback
coefficient is selected here.

### Positive decreasing force and the equilibrium root

The ordinary squared-distance barrier has f(d)>0 for 0<d<h because B_s<0.
Its physical curvature is strictly positive throughout that interval. To prove
this without inferring physical convexity merely from B_ss>0, set q=(d/h)^2:

```text
b''/h^2 = 4(3q-1)(-log q) + 2(1-q)(7q+1)/q.
```

For 1/3<=q<1 both terms are nonnegative and the second is positive. For
0<q<1/3, use -log q <= (1/q-q)/2. This inequality follows by differentiating
(t-1/t)/2-log t for t=1/q>=1: its derivative is (t-1)^2/(2t^2)>=0,
and its value at t=1 is zero. Multiplication by the negative 4(3q-1) reverses
the inequality, yielding

```text
b''/h^2 >= 2(3q-1)(1/q-q) + 2(1-q)(7q+1)/q
         = 6(1-q)(q+3) > 0.
```

Thus f'=-b''<0. Define the scalar force residual

```text
R(d;k,K,p) = K(d-p) - k f(d)
R_d = K + k b''(d) > 0.
```

For k>0 and p<h, f(0+)=infinity and f(h-)=0, so R changes from negative
infinity to K(h-p)>0. There is exactly one root, with max(p,0)<d<h.
Implicit differentiation gives

```text
partial d / partial k = f / (K+k b'') > 0
partial d / partial p = K / (K+k b'') > 0
partial d / partial K = -(d-p) / (K+k b'') < 0.
```

At k=0 and p>0 the free equilibrium is d=p; the K dependence is zero.
For p>=h the barrier is inactive at d=p for any k>=0, so increasing k does
not move that equilibrium. For k=0,p<=0 there is **no positive-gap equilibrium**
of this unconstrained quadratic reference. A zero coefficient cannot be used
to infer feasibility or preserved active-contact protection.

For a different positive decreasing force, R_d>0 still gives uniqueness,
but existence additionally needs endpoint sign conditions. For comparison,
positive **increasing** f(d)=d^2 with K=k=1,p=.1 has two interior roots
(1 +/- sqrt(.6))/2. Positivity alone is not enough. This is a mathematical
counterexample to dropping the hypothesis, not an alternative contact law.

### Exact inputs: when the band formulas apply

For p<=U, the unique root belongs to [L,U] exactly when R(L)<=0<=R(U).
Together with k>=0 this is equivalent to

```text
k_L = K max(L-p,0) / f(L)
k_U = K (U-p) / f(U) = K max(U-p,0) / f(U)
k_L <= k <= k_U.
```

If p<L, both bounds are positive and their roots are L and U. If L<=p<U,
k_L=0 and the lower endpoint's root is p, not necessarily L. If p=U,
the formal interval is [0,0], with the free root exactly at U; every positive
k moves it above U. This zero-only case is a reference-model result, not
authorization to remove an existing barrier or override CCD.

For p>U, every repulsive equilibrium is at least p>U. The raw positive-part
formulas return [0,0] but do **not** define a band-valid interval. Classify this
as `inactive_band_demand`: the predictor does not demand repulsion to reach
the band, and no upper-gap requirement or attractive force is imposed. This
label does not mean that the actual collision is outside barrier support; p may
be between U and h. The probe returns no applicable interval for this case.

For exact inputs p<=U the interval is never empty: when p<L the numerator
increases and the positive denominator decreases between L and U, and otherwise
the lower bound is zero. Empty intervals in the box construction below reflect
simultaneous uncertain demands, not this single exact-input case.

### Enclosing uncertainty: the rectangle theorem

Suppose 0<K_-<=K<=K_+ and p_-<=p<=p_+ are true enclosures with **p_+<=U**,
while h, L, U and f are fixed. Enforcing the two residual signs for every
pair in this Cartesian box gives

```text
k_lower = K_+ max(L-p_-,0) / f(L)
k_upper = K_- (U-p_+) / f(U).
```

The lower bound is the largest nonnegative lower requirement in the box,
attained at (K_+,p_-) if positive. The upper is the smallest upper requirement,
attained at (K_-,p_+). Thus a nonempty [k_lower,k_upper] is necessary and
sufficient to cover the **entire Cartesian box** under this exact reference.
The positive bounds are tight: slightly decreasing k_lower fails the lower
corner and increasing k_upper fails the upper corner. If the lower bound is
zero, negative k is excluded; if the upper is zero, any positive k fails its
worst corner. Nonempty intervals with k_lower=0 imply p_->=L>0, so their k=0
members do not suffer the p<=0 domain failure described above.

This is a rigorous exact-arithmetic statement **conditional on the model and
the enclosing inputs**. Here the boxes are manufactured admissible sets. A
finite grid checks the implementation of the theorem but cannot prove that a
real estimator's error bars enclose a mechanical system. Decimal endpoint
calculations and double root checks are not outward-rounded interval arithmetic;
their printed endpoints are numerical estimates of the theorem's exact bounds.
No application-level enclosure or floating-point certificate is supplied.

If p_->U, the whole box is `inactive_band_demand`. If p_-<=U<p_+, classify
`mixed_band_applicability` and return no all-box band interval: the box contains
free gaps that cannot satisfy an upper-band requirement through repulsion.
The raw clipped pair may still be recorded diagnostically, but it is not a
certificate. Separating demand regimes or narrowing the enclosure needs a later
estimator/controller contract; this probe does neither automatically.

For p_+<=U but k_lower>k_upper, report `empty` and preserve that ordering.
Do not sort or clamp the endpoints, choose their midpoint, increase stiffness,
retry the step or label the physical problem infeasible. Each particular member
can have its own valid interval while their common intersection is empty.
A rectangular enclosure can also lose correlations: the manufactured family
K in [1,10], p(K)=.5-f(.5)/K has equilibrium d=.5 at k=1 for **every** K,
yet its enclosing Cartesian rectangle has an empty common interval for [.4,.7].
This demonstrates why an empty conservative box does not prove failure of the
correlated physical family.

The [stage 2 protocol](../tools/rb13/stage2-protocol.md) and standalone
`band_probe.py` test these cases with compiled IPC forces and independent
Decimal reference roots. Results and publication are recorded in
[validation](rb-13-validation.md). Section 8 characterizes mechanical applicability;
RB-14 estimator enclosures and RB-16 controller policy remain separate work.

## 8. Stage 3 — mechanical scope and counterexamples (2026-09-10)

The [stage 3 protocol](../tools/rb13/stage3-protocol.md) declares the fixtures
and tolerances. `scope_probe.py` uses the compiled effective IPC barrier primitive
for force, curvature and energy evaluations. Small spring systems are assembled
analytically; no FEM mesh, production coefficient controller, C++ integrator class
or collision builder is executed. The Rayleigh comparisons below evaluate the
source-traced quadratic expression, not the full assignment/trim pipeline.

### Supported springs and the rigid endpoint limit

Let two endpoints have displacement u=[u_A,u_B], support energy
1/2 A u_A^2 + 1/2 B u_B^2 and gap d=p+u_B-u_A. For repulsive force F,

```text
H = diag(A,B), J=[-1,1]
u_A = -F/A, u_B = F/B
d-p = F(1/A+1/B)
K_eff = AB/(A+B).
```

This is two supported compliances in series, not the sum of the support
stiffnesses. Contact forces [-F,F] and support forces [F,-F] balance. As B tends
to infinity, K_eff tends to A with relative error A/(A+B). Ratios B/A from 1 to
1e6 reproduce that expression. An explicitly prescribed u_B=0 gives K_eff=A
exactly in the reduced space. The finite B/A=1e6 case is close to rigid, not
identical to a prescribed boundary condition.

### Why the current Rayleigh value is different

At nondegenerate frozen geometry the current IPC routine normalizes the vector
formed from stencil coefficients times the contact normal. For this scalar
two-endpoint stencil that direction is w=J^T/||J||. With physical H (the form
acceleration weight removed), its raw curvature is

```text
r = w^T H w = (J H J^T)/||J||^2 = (A+B)/2.
```

PolyFEM passes zero local mass to this routine and subsequently divides the
weighted curvature by dhat^2 and its form weight, then applies the documented
caps/trim. This comparison concerns the raw curvature, not those later policies.

If displacement is restricted to the direction J^T while achieving gap change
delta, u=J^T delta/||J||^2. Its **gap** spring stiffness is therefore

```text
K_direction = (J H J^T)/||J||^4 = r/||J||^2 = (A+B)/4.
```

The r-to-K_direction factor cannot be omitted when comparing a normalized
coordinate direction to a physical gap. Cauchy-Schwarz gives
(J H J^T)(J H^-1 J^T)>=(J J^T)^2 for SPD H, so K_direction>=K_eff.
Allowing all displacements to relax can only lower the minimum energy for a
fixed gap change. Equality holds for this example when A=B; even then r is
twice K_eff because ||J||^2=2.

For A=1,B=100 the measured values are r=50.5, K_direction=25.25 and
K_eff=.9900990099. This mismatch is a difference in displacement restrictions
and normalization, not evidence that the two formulas were intended to be equal.
It does not choose a replacement for the current stiffness estimator.

For a fixed obstacle the current zero-padded two-endpoint stencil with A=2
has r=1, while the truly reduced prescribed-obstacle problem has K_eff=2.
An obstacle placeholder is not a freely relaxing zero-stiffness DOF. The
comparison must state which coordinates are admissible; inversion of that
singular padded matrix is not a substitute for reducing the prescribed DOF.
General interpolated geometry remains subject to RB-03/RB-15's open contract.

### Inertia, history and unit conversion

For the supported endpoints with stiffnesses 4,12 and masses 1,3, implicit
Euler's physical incremental objective is

```text
Psi_nc(u) = sum_i [1/2 A_i u_i^2 + m_i/(2 dt^2)(u_i-dt v_i)^2]
H_i = A_i + m_i/dt^2
u_free,i = (m_i v_i/dt)/H_i
p = .6 + u_free,B - u_free,A.
```

With v_A=s/2, v_B=-s/2, p=.6-s dt/(1+4 dt^2). The initial displacements
are zero; velocity history changes p without changing H at fixed dt. Both mass
terms enter once through H and the predictor. At dt=.2, K_eff=21.75 for every
tested speed; s=1,2,4 gives target-.5 demands 1.575,5.325,12.825 and coefficients
.3637882588,1.2299507797,2.9622758216. All eight compressed cases across the
three timesteps recover .5, including negative extrapolated p. No barrier is
evaluated at that negative predictor. Four cases have zero target demand and
make no active-protection decision.

Holding the speed-2 coefficient for the dt=.2 comparison gives gaps .6997678892
at speed 0 and .3299897579 at speed 4. Thus H alone cannot describe the predicted
compression. This is a one-step incremental reference, not a transient trajectory,
impact restitution result or an implementation of a history-aware controller.

For numerical conversion x'=lambda x, force'=mu force, time'=tau time, use

```text
dt'=tau dt, v'=lambda/tau v, m'=mu tau^2/lambda m
A'=mu/lambda A, H'=mu/lambda H, p'=lambda p, dhat'=lambda dhat
k'=mu/lambda^3 k
Psi'=mu lambda Psi, Phi'=mu lambda tau^2 Phi  (Phi=dt^2 Psi).
```

All 27 length/force/time combinations in the protocol reproduce converted roots,
forces, displacements and physical/incremental objective values. Psi includes
the inertial algorithmic potential; it is not stored mechanical energy. This
does not overturn RB-02's production-controller unit-dependence evidence or test
new unit conversions for absolute safeguards, scene inputs or friction.

### Stable nonlinear response can invalidate a local band

Use g(d)=6(d-.1)+alpha(d-.1)^3 with potential
3(d-.1)^2+alpha/4(d-.1)^4. For alpha>=0 its tangent is strictly positive.
At anchor a, the linear reference is H_a=g'(a), p_a=a-g(a)/H_a, and its
target coefficient uses H_a(.5-p_a)/f(.5). This freezes a local Taylor model.

At alpha=100,a=.1 it predicts .5 using k=.5543440134, but the actual nonlinear
equilibrium is .3713309397. Its force residual at the predicted target is 6.4;
the actual nonlinear root itself has a small residual. Using the exact target
demand g(.5) instead gives k=2.0325947158 and recovers .5. Anchors approaching
.5 reduce error in this convex quartic fixture; this trend is not a theorem for
arbitrary materials or a selected refresh policy.

More decisively, the **upper** coefficient .7491934949 of the local model's
[.45,.55] band gives actual nonlinear gap .3997523752, outside the band.
The global spring remains stable: the failure is extrapolation of a local linear
model, not negative curvature or numerical nonconvergence. Stage 2's theorem
still applies to its declared quadratic reference. Application-level bounds
need justified control of the actual force/compliance over the relevant range;
uncertainty around one tangent alone does not provide that control.

### Coupled contacts require the full compliance matrix

For several affine gaps with rows J_i, minimizing the noncontact quadratic gives

```text
H (u-u_free) = J^T F
W = J H^-1 J^T                (one solve per required column)
d-p = W F.
```

W is SPD when H is SPD and J has independent rows. If the rows are dependent,
W can be singular even though H is SPD; no inverse or independent coefficient
interval follows without further compatibility analysis. For a target d_star,
solve W F_star=d_star-p. Only nonnegative force components can be supplied by
nonnegative coefficients k_i=F_star,i/f_i(d_star,i) in this repulsive model.

The measured fixture has H=diag(2,3,5), J=[[1,-1,0],[0,1,-1]], giving

```text
W = [[5/6, -1/3], [-1/3, 8/15]]
p=[.2,.25], d_star=[.5,.55]
F_star=[.78,1.05], k=[.1801618044,.2913530258].
```

A full three-displacement Newton solve with compiled IPC forces returns both
target gaps, with normalized residual 8.76e-17 and contact action-reaction balance.
Using only W's diagonal yields k=[.0831516020,.1560819781] and actual gaps
[.3859795050,.4724886438], despite converging with residual 3.45e-12. Numerical
convergence does not validate the independent-contact estimate.

For W=[[2,1],[1,2]] and desired gap increment [.1,.5], the required forces
are [-.1,.3]. W is SPD and each desired gap increases, yet one force would need
attraction because of cross-contact response. The fixture records that incompatibility
with the chosen joint target; it neither installs a negative k nor clamps F to
zero. It is not proof that the underlying physical problem lacks an equilibrium.

### Unsupported tangents and remaining decisions

The standalone reference rejects singular/indefinite/nonsymmetric/nonfinite H
and zero J. In particular, a positive curvature in one selected direction does
not make an indefinite full problem a stable energy minimum. For the singular
spring H=[[1,-1],[-1,1]], the relative J=[-1,1] annihilates the rigid mode:
relative compliance can be defined after a suitable explicit quotient/support
and load-compatibility choice. The strict-SPD helper rejects it rather than making
that choice silently. A separately declared fixed endpoint gives K_eff=1.
A rigid-mode-sensitive J=[1,1] instead changes gap at no elastic cost; it cannot
use the finite-positive-compliance contract. These two reasons are distinct.

H=diag(1e-10,1), J=[1,0] retains K_eff=1e-10, tested as a ratio. No arbitrary
positive floor, pseudoinverse or PSD projection is introduced. The diagonal
fixture's conditioning is not evidence of general near-singular solver robustness.

RB-13's three planned analytical stages are now characterized. **Production
choices remain pending:** RB-14 must compare practical compliance/demand estimates
and establish their approximation/enclosure limits; RB-15 must address feature
and mapping consistency; RB-16 must choose controller/update-state handling; RB-17
must compare the integrated candidate before any default promotion. No global
versus local replacement, recovery policy, friction update or new force floor
is implied by the reference calculations.
