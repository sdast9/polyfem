# RB-14 — Practical compliance and force-demand estimator contract

2026-09-10. **Stage 1 characterized; assembled-FEM stage pending.** This is a
standalone comparison of candidates, not a selected production estimator.
See [validation](rb-14-validation.md), [protocol](../tools/rb14/stage1-protocol.md)
and [results](../tools/rb14/results-20260910-stage1.json).

## Reference and source contract

Use RB-13's frozen, stable, frictionless quadratic in free displacement
increments v, with noncontact energy r^T v + v^T H v/2 and gap d=d0+Jv.
Here H is SPD, r is the noncontact residual at the current linearization, and
prescribed motion has already been included in d0 and r. Solve, without forming
an inverse,

```
H z = J^T,  H y = r
C = J z,  K_eff = 1/C,  p = d0 - J y
F_target = K_eff max(d_target-p,0)
k = F_target/f(d_target),  f(d) = -db/dd.
```

For fixed k, the full quadratic equilibrium is v=-y+z k f(d). The probe checks
its free residual and reconstructed gap after solving the scalar force balance.
It never launches a contact-free scene; negative p is only an extrapolation.
A zero candidate demand does not authorize removal of active contact protection.
When that candidate gives k=0 and the reference p<=0, the probe reports an
unprotected invalid free gap and does not evaluate the barrier at that geometry.

Units are physical incremental energy: H and K have force/length, r has force,
p/d0/dhat have length, k has force/length^3 for the ordinary unweighted squared-
distance barrier, and k*f has force. The barrier tangent k*b'' is distinct from K.
For the implicit-Euler network, H=K_elastic+M/dt^2 and r=-M v_history/dt-load
at zero current displacement. No mass term is added to the resulting estimate.
All bands here use physical distance; none are production squared-gap settings.

Inspected at PolyFEM `5b64d316f`, IPC `af317a65`, PolySolve `4d372fa8`:

- [SolveData](../src/polyfem/solver/SolveData.cpp) provides weighted elastic plus
  enabled inertia Hessians. Its gradient provider includes elastic, inertia,
  body and pressure forms. Therefore the provider Hessian is not automatically
  the derivative of every residual contribution (especially deformation-dependent
  loading). These network loads are constant; the mismatch needs an assembled
  fixture and explicit approximation declaration in stage 2.
- [BarrierContactForm](../src/polyfem/solver/forms/BarrierContactForm.cpp) extracts
  blocks with mapped system IDs for distinct exact-selector rows, normalizes
  the IPC direction, removes dhat^2 and form weight, and applies existing
  coefficient policies. This probe evaluates its raw Rayleigh expression, not
  production extraction, caps, trim or assignment.
- [RB-03](rb-03-contract.md) establishes J=J_surface B Q and prescribed offsets
  through Bc. Collision/proxy IDs are not FEM indices. Exact permutation is
  checked algebraically here; actual mapping/extraction remains stage 2. Arbitrary
  interpolated stiffness, nonlinear maps and singular admissible spaces are not
  assigned a model by this work.

## Candidates and locality

Let P select the DOFs in a declared neighborhood, retaining all nonzero J entries.
Set every exterior displacement increment to zero. Use H_N=P^T H P, r_N=P^T r,
J_N=J P and solve the same two equations. This includes the diagonal contributions
of springs crossing the boundary; it does not cut those springs away.
Restricted minimization proves K_full<=K_N for this SPD quadratic. Nested
neighborhoods give K_full<=K_radius1<=K_radius0. It gives no lower stiffness bound
and no one-sided predictor guarantee. Remote residuals can change the sign of
predictor error; an independent two-DOF control demonstrates both signs.

The network has two planar bodies, each with three nodes. Contact acts between
nodes 2 and 3 along n=(.6,.8), with J=[..., -n, +n, ...]. Radius 0 includes these
two nodes (4 DOFs); radius 1 adds their adjacent nodes (8 DOFs). Full solves have
12 DOFs. The holdout topology has four nodes per body and 16 full DOFs. Rotated
SPD spring blocks connect each chain; remote endpoints have elastic supports.
These are quadratic vector-spring networks, not assembled continuum elements.

The diagonal candidate replaces H with diag(H). Positive diagonal entries do not
prove full stability. A separate H=[[2,+/-1],[+/-1,2]], J=[1,1] control gives
K_full=1.5 or .5 versus K_diag=1, demonstrating both error signs.

For the normalized direction w=J^T/||J||,

```
r_R = J H J^T / ||J||^2
K_direction = J H J^T / ||J||^4
p_direction = d0 - ||J||^2 (J r)/(J H J^T).
```

The direction candidate constrains v parallel to J^T. A separate raw-Rayleigh
comparison treats r_R as K while retaining p_direction; it intentionally exposes
the normalization difference. Neither candidate executes the production coefficient
controller, whose trim and caps could alter the outcome. No claim of an actual
production gap follows from these two columns.

## Calibration, holdout and conditional intervals

The [protocol](../tools/rb14/stage1-protocol.md) specifies all parameter ranges
before execution. E*h stiffness and rho*h^3 mass are a declared 3D geometric-size
scaling model. Varying h is not mesh refinement or a unit-conversion experiment.
Separate contrast, anisotropy, support, size, density, dt, speed and remote-load
sweeps give 18 unique calibration fixtures. Ten held-out cases extend individual
ranges, combine extremes, or change topology. No neighborhood or k is fitted.

Calibration uses only radius-1 errors from the 18 calibration cases:

```
K_true / K_radius1 in [.8186187948763494, .9999999997520191]
(p_true-p_radius1)/dhat in [-.3436028927333776, 0].
```

These are empirical extrema with no padding. Coverage on training data is by
construction and cannot certify a bound. Held-out K coverage is 7/10, p coverage
8/10 and joint coverage 6/10. Failures remain in the results; no interval widening,
safety factor or retuning followed holdout inspection.

Apply RB-13's rectangle formula to [.45*dhat,.55*dhat] only as an empirical
experiment. Four calibration and two holdout rectangles are inactive/mixed;
13 calibration and seven holdout intervals are empty; only speed=2 (calibration)
and speed=4 (holdout) have active nonempty intervals. Both endpoint pairs realize
the requested band in those two cases. This limited success does not repair the
missing input enclosures or justify a general coverage claim. Empty means the
chosen empirical rectangle cannot satisfy both requirements, not physical
infeasibility. No coefficient is selected from an empty or inactive/mixed interval.

## Limits demonstrated

Baseline: K_full=16.34210526, p_full=.00206119. Radius 1 estimates
K=16.38888889 and p=.00542373. Its k=168.74129 realizes d=.04874483 versus target
.05; the exact k=180.95199 realizes .05. Radius 0 gives d=.04102651. The raw
Rayleigh control gives .05342850, which is not a measured production-controller gap.

With remote load=10 the same local radius-1 values give d=.03936870. The exact
stiffness is unchanged, but the true predictor moves to -.02370370: curvature
agreement alone cannot determine force demand.

On the combined holdout, radius 1 overestimates K by a factor 27.9682 and misses
p by 376.667*dhat. It predicts zero demand; the full reference has p=-37.5869 and
positive target demand. This extreme manufactured quadratic exposes loss of remote
motion/load response. It is not a prediction of plausible finite-strain geometry.
RB-13's separate nonlinear counterexample remains an additional limitation.

Two independent network blocks at the same dt=.2 and dhat=.1 can represent
heterogeneous contacts. The rho=100 and contrast=100 cases require disjoint
bands [11031.80,19152.69] and [47.95,152.47]. At the first lower endpoint, the
second gap is .09183855; at the second upper endpoint, the first gap is .00499006,
outside [.045,.055]. Thus a single shared raw coefficient cannot satisfy these
particular simultaneous bands even without cross-contact coupling. This says
nothing about a shared trim multiplying different local scales, and does not
select a production global/local policy.

## Obstacles and fallback proposal

The prescribed-obstacle control uses H_full=[[6,2],[2,5]], initial residual
[-.3,0], J_full=[-1,1], and obstacle displacement c. Reduction gives H_free=6,
r_free=2c-.3, d0=.08+c and p=.03+4c/3. Fixed c=0 and moving c=-.03 produce
F=.12 and .36 at target .05. Full contact forces are [-F,F]; free balance and
prescribed external reaction are recorded. Increasing the second endpoint's
support B in H=[[6,2],[2,B]] gives K=(6B-4)/(B+10), approaching 6. These are
algebraic controls, not actual moving-boundary integration.

**Proposal, not selected policy:** invalidate estimates on dt/map changes. If
fresh, supported, finite SPD H and a compatible residual are available, recompute
the full reference; otherwise return unavailable with no new coefficient. The
caller would still need an explicitly selected safe continuation/stop policy;
this probe does not delete contacts, reuse stale k, retry or change dt.

The prototype classifies singular, indefinite, nonfinite, zero-J, missing-residual
and unsupported-map inputs. Tokens model stale dt/map detection; no production
cache implementation or complete invalidation key has been installed. Material,
BC, linearization, load and integrator revisions would also need lifecycle design.
Reusing the baseline coefficient after dt changes .2→2 yields gap .07839541;
fresh recomputation yields .05. Permutation and stale-map recomputation are
checked on the declared algebraic supported map.

## Cost and next stage

The final run's 15-repeat medians are about 98.5 us (full), 18.9 us (radius 0),
43.5 us (radius 1), 8.8 us (diagonal), 15.2 us (direction), 15.1 us (raw Rayleigh).
Peak traced Python allocations are 4672,1216,2240,2816,2688,2688 bytes respectively.
This includes prototype extraction and repeated independent factorizations, excludes
pre-existing input matrices and native allocations, and is noisy local Python cost.
It does not establish production speedup, sparse-factor reuse, cache growth or
cost per FEM contact. Baseline dense input storage alone is 144 matrix scalars.

Stage 2 must measure assembled supported-map FEM snapshots and real extraction,
residual/tangent consistency, neighborhood construction, conditioning and realistic
cost. Preserve a held-out assembled fixture, isolate stiffness/predictor error,
and compare fresh fallback before choosing an estimator. Mesh refinement and
nonlinear realized gaps remain unmeasured here. RB-14 remains in progress.
