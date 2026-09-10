# RB-13 stage 1 — Mechanical coefficient reference

2026-09-09. **Stage 1 characterized within the reference assumptions.**
This is a derivation and standalone analytical probe, not an installed coefficient
law. Conditional intervals (stage 2) and the complete scope matrix (stage 3) are
pending. See [validation](rb-13-validation.md) and [reproduction](../tools/rb13/README.md).

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
problem. The current Rayleigh-quotient law is not this condensation. A quantitative
comparison, supported springs, rigid limits and coupled contacts belong to stage 3.

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
An upper-gap requirement must not pull a separating body into contact. Stage 2
will establish conditional root/band semantics and inactive/empty cases.

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
dimensional normalization over dhat=.01, 1, 100; a full equivalent-unit mechanical
scope test remains stage 3.

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

Next is RB-13 stage 2: monotonic force-balance roots and conditional intervals,
including enclosing-input uncertainty, empty intervals and inactive demand.
Stage 3 must still test spring series/rigid limits, differing speeds, full unit
conversion, nonlinear prediction error, Rayleigh comparison and multi-contact
coupling. No production k, global/local replacement, estimator approximation,
uncertainty enclosure or controller timing policy has been selected.
