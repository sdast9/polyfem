# RB-14 — Practical compliance and force-demand estimator contract

2026-09-10. **Stages 1–2 characterized; further estimator comparison pending.** This is a
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

## Stage 2 — assembled FEM comparison (2026-09-10)

The [stage 2 protocol](../tools/rb14/stage2-protocol.md) and
[case list](../tools/rb14/stage2-cases.json) define 15 calibration and three
held-out configurations. The [results](../tools/rb14/results-20260910-stage2.json)
are assembled with the actual PolyFEM Neo-Hookean, inertia and constant-load
forms, using the public NonlinearElasticVarForm embedding API. No production
coefficient, extraction or solver implementation was changed.

### Geometry, assembly and admissible coordinates

A unit square has P1 triangles, with n=2,4,8 subdivisions per side. This is
fixed-domain refinement: 8/32/128 elements, 18/50/162 total FEM DOFs, and
12/40/144 free DOFs after clamping both components of the top edge. The n=6
held-out mesh reverses the cell diagonals. The actual global basis indices and
coordinates, rather than input vertex ordering, define Q and element adjacency.
Each P1 basis must have one exact global node; unsupported bases are rejected.

Top displacement c is set in the current full state before taking the residual
and Hessian. The initial history is uniform downward velocity, including the
subsequently prescribed top DOFs. This is an explicitly specified one-increment
support constraint, not a smooth support-motion trajectory. Inertia retains the
consistent assembled mass coupling to prescribed coordinates. Its actual gradient
is checked against M(x-x_tilde); the actual integrator predictor is checked against
initial velocity times dt. Physical H and r divide the weighted form values by
the actual dt^2 acceleration scaling once. Elastic/inertia Hessians and
elastic/inertia/body/pressure residuals follow the production provider sets.
Pressure is zero in these cases; no deformation-dependent pressure-tangent
consistency is certified.

A single bottom-center FEM vertex is selected into an IPC proxy with two appended,
prescribed obstacle vertices. Proxy IDs 0 and 1 map to the appended obstacle
system nodes; proxy ID 2 maps to the FEM contact node. The obstacle segment spans
x=-9.5 to 10.5, y=-.08. The actual IPC `to_full_dof` and `map_displacements`
operations verify virtual work and J=J_surface B Q. The exact free selector Q is
explicitly assembled in the probe, not obtained from a running NLProblem/AL phase.
This is a supported selector fixture, not an interpolation-model decision or a
test of the external OBJ/HDF5 builder.

**Only this selected vertex/segment pair participates in contact.** The proxy is
not the square's complete surface, and the study does not certify nonpenetration
of omitted surface nodes. The physical gap is d=.08+u_point,y-u_obstacle,y,
dhat=.1, target=.05. Moving the obstacle by .02 gives initial gap .06. Prescribed
motion remains in the gap and full residual; obstacle reactions are retained in
mapped contact gradients. Contact action/reaction is checked after each solve.

The actual BarrierContactForm semi-implicit extraction receives the assembled
weighted Hessian with FEM-only dimensions. It correctly maps the selected FEM
node and skips appended obstacle blocks. At the centered EV stencil, the measured
assigned coefficient matches H_point,yy/(1.5*dhat^2), after removing form weight.
The factor 1.5 is the squared norm of the [-.5,-.5,1] normal stencil. A separate
fixed-coefficient solve uses this measured coefficient with trim=1. This executes
the extraction control but does not exercise adaptive trim/refresh timing.

### Numerical and cost controls

The reduced H is checked SPD with dense eigenvalues; the largest measured condition
number is 407.388. Dense LLT and independent sparse SimplicialLLT solutions agree
within the predeclared 1e-8 screen, and their linear residuals are checked. The
actual noncontact energy/gradient/Hessian directional differences have maximum
scaled errors 1.03e-9 and 5.60e-11, below the 2e-5 screen. These constant-load
checks do not establish consistency for absent load terms.

Each positive candidate k is frozen. The scalar quadratic root uses actual mapped
contact gradients. The realized endpoint uses the assembled nonlinear energy,
gradient and tangent with the same fixed coefficient. The standalone Newton
loop enforces the declared residual threshold, checks positive element determinants,
and invokes the actual elastic/contact step limits and contact CCD. No Hessian
projection, tolerance relaxation or coefficient retuning is used. Its controls
are local to this experiment, not new production stopping rules.

Across the 126 candidate records, 54 positive-coefficient nonlinear solves
converged and 72 zero-demand candidates were classified without an unprotected
solve. One additional stale-coefficient control converged (55 attempted solves
total). All returned element determinants are positive; the smallest is .470555.
Convergence and positive determinants do not imply target accuracy or whole-surface
collision freedom. The raw trim=1 control sometimes becomes inactive above dhat,
which is retained as a valid outcome rather than an enforced upper-gap condition.

At n=2/4/8, full dense extraction plus two-RHS solve medians are .791/4.916/97.916
microseconds; radius-1 medians are .583/.542/.625 microseconds. Radius 1 retains
10 free DOFs at all three resolutions, so its physical extent shrinks. These
seven-repeat warm timings include submatrix extraction. Graph construction and
first physical assembly are recorded separately. Sparse full factorization costs
3.333/8.667/34.375 microseconds; two RHS solves reusing each factor cost
.375/1.125/5.583 microseconds. The n=8 sparse matrix/factor payload accounts for
21,580/23,128 bytes, excluding allocator overhead, working buffers, other forms,
collision structures and process RSS. Dense input and submatrix storage are
separate. This is measured small-FEM cost, not production throughput or a resource
bound; no multi-contact factor/cache lifecycle is installed.

### Measured estimator limits

| Mesh | Full K | Full p | Radius-1 target-demand outcome | Full-reference k, nonlinear gap |
| --- | ---: | ---: | --- | ---: |
| n=2 | 11.221507 | .04334612 | Positive; realized gap .04510740 | .04959688 |
| n=4 | 7.869371 | .04379470 | Zero demand; unprotected solve omitted | .04955783 |
| n=8 | 5.981886 | .04384994 | Zero demand; unprotected solve omitted | .04954220 |

All graph neighborhoods preserve the restricted-quadratic upper-K inequality.
That does not prevent missed compression: on refined meshes even radius 2 can
report zero demand where the full predictor requires a positive coefficient.
The full K itself changes under refinement because the load/contact acts at a
single vertex. This is a locality experiment, not converged finite-area contact
stiffness or continuum engineering accuracy.

In the held-out n=4/speed=1.5 case, the full-reference coefficient realizes
.04801919, radius 1 realizes .01219297, and radius 2 realizes .03251305, versus
.05. Thus both local predictor loss and nonlinear Taylor error remain measurable.
Even the full frozen reference is not an exact nonlinear target-force model.
The current trim=1 extraction control realizes .07298300 on that case; this is
not a result from the full adaptive controller or a selected global multiplier.

**Load sign is recorded from the actual assembled objective.** The fixture writes
boundary_conditions.rhs=[0,-load]. In this code path positive `load` produces an
upward physical body force and increases the free-gap predictor (load=10 gives
p=.28770534). RhsAssembler's energy-gradient convention and BodyForm's sign agree
with the finite-difference check. These cases are unloading comparisons, not
compressive-load validation. The held-out load=20 case remains unchanged and
identified accordingly; it was not replaced after inspecting its outcome.

### Empirical coverage and stale-state interpretation

The radius-1 ranges calibrated only from the 15 FEM calibration cases are

```
K_true/K_radius1 in [.6327360835343682, .999982295929729]
(p_true-p_radius1)/dhat in [-.3318797684597228, .26814772406686727].
```

They cover K on 2/3 held-out cases and p on 0/3: joint coverage is **0/3**.
Applying the unchanged stage-1 ranges covers only 9/15 calibration cases jointly
and 0/3 held-out cases. Neither set is an input enclosure suitable for a guarantee.
The derived empirical coefficient rectangles yield 13 inactive/mixed cases and
five empty intervals, with no active nonempty interval. No midpoint, clamp,
stiffness increase or coefficient is selected from them.

For dt=.2→.5, reusing the baseline k=17.24624241 gives nonlinear gap .06057172.
Fresh full-reference computation instead reports p=.05493047, above the .05 target,
and zero target demand. It therefore does **not** supply a replacement active
coefficient; the unprotected solve is omitted. This is a useful limit of the
stage-1 fallback proposal: fresh data resolves stale estimation but does not choose
active-contact protection when demand is zero. No production fallback is installed.

### Remaining decisions

The stage-2 comparison is complete within this explicit 2D selected-contact scope.
The evidence argues against promoting fixed graph-radius neighborhoods or their
empirical error ranges as guarantees. It does not select a different estimator.
A fixed physical-radius neighborhood, residual influence beyond the neighborhood,
and reuse of full sparse factors are concrete candidates for a subsequent RB-14
comparison. Coefficient protection for zero demand and nonlinear force-range
control still need a declared contract. General interpolation, deformation-dependent
loads, anisotropic/heterogeneous assembled materials, 3D contact patches and
multi-contact resource scaling remain unmeasured here; stage 1's spring-network
contrast/anisotropy results do not certify those FEM cases. RB-14 remains in
progress until the practical estimator/fallback evidence is sufficient for a
production decision. RB-15 is independently eligible; no RB-16/17 policy change
is implied.
