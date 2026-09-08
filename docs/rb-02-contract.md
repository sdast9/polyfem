# RB-02 — Current coefficient and lifecycle contract

Audited 2026-09-08 against PolyFEM `cfc41224c`, effective IPC `9da3094`, and
PolySolve `4d372fa8`. This describes current implementation, including its
counterexamples; it is not a recommended new model. See [measurements and
validation](rb-02-validation.md) and the [executable probe](../tools/rb02/coefficient_probe.cpp).
The retired constraint floor is absent; `gap_floor=0` in every probe.

## Objective and dimensions

For the tested identity collision/FEM mapping, unweighted collision stencils,
unprojected derivatives, `dmin=0`, and the ordinary clamped-log potential, let
`J` denote the units of the **weighted incremental objective**, `L` length,
`a=weight_` the form's acceleration weight, `s=scale_` its normalization, and
`t=barrier_stiffness_` the global trim. Production `NLProblem::normalize_forms`
returns 1. A transient weighted objective need not have physical energy units.

For each stencil key `i`, evaluated at the frozen surface `X*`:

```text
c_i = stencil.compute_coefficients(X*)
n_i = sum_v c_iv X*_v
w_i = normalize([c_i0 n_i, c_i1 n_i, ...])
r_i = w_i^T H*_local w_i                    [J/L^2]
q_i = r_i / dhat^2                         [J/L^4]
f_i = max(isfinite(q_i) ? q_i : 1e30, kappa_min)
k_i = f_i / a                             [J/(a L^4)]
m = upper median of refresh-batch k_i
C = kappa_spread * m, or infinity for an empty batch / disabled spread
khat_i = min(k_i, C)
E_contact(x | snapshot, trim) = (a t / s) sum_{i in collisions(x)} khat_i b_i(x)
```

`b_i=-(d_i²-dhat²)² log(d_i²/dhat²)` inside support and zero outside in these
fixtures, with units `L^4`. Stencil weighting/area weighting multiplies this
base potential in other configurations and is not characterized here.
Physical-barrier and shape-derivative options are rejected in semi-implicit mode.
For a static physical energy `J=force*length`, `q` has units `force/length³`.
The dimensional analysis of `q` is with respect to the weighted objective;
do not call every transient `q` an unweighted physical spring coefficient.

IPC's effective implementation is in `src/ipc/barrier/adaptive_stiffness.cpp`,
not a file named `semi_implicit_stiffness.cpp`. It returns
`avg_mass / distance² + w^T H w`. PolyFEM deliberately supplies **zero local
mass**, even when stored lumped masses are nonzero. Inertia instead enters
through the weighted system Hessian. Thus the local mass term does not rescue
zero/negative curvature. At degenerate zero distance its arithmetic can still
be undefined; degenerate geometry was not executed in this audit.

For fixed `a,t,s`, a fixed active stencil region, and finite arithmetic, the
implemented gradient and unprojected Hessian are derivatives of that sum with
**coefficients held constant**, with no derivative through `X*`, `H*`, or trim.
This is a piecewise contract. Determinism of the stencil-key cache does **not**
make the objective continuous across a change of nearest feature. The probe
reproduces a nonvanishing edge-vertex/vertex-vertex energy jump with one frozen
snapshot. It does not validate arbitrary stencil transitions or 3D contacts.

## State inventory

| State / owner | Units | Update triggers | Consumers / lifetime |
| --- | --- | --- | --- |
| `kappa_surface_`, BarrierContactForm | L | Explicit refresh; first-contact post-step; optional periodic refresh; solve-start and stall callbacks | Local stencil coefficients/normals; retained during trial notifications |
| `kappa_hessian_`, same form | J/L² | Same refresh calls the installed provider at full x | Extracted local blocks; max absolute entry for conditioning cap |
| Provider closures, installed by SolveData | Hessian J/L²; gradient J/L | Weighted elastic Hessian plus enabled inertia Hessian; gradient of enabled elastic, inertia, body and pressure forms | Weak form references; no contact, friction, damping or AL Hessian term in this provider |
| Contact stencil key / `kappa_cache_`, same form | Integer type + four ordered vertex IDs; stored value J/(a L⁴) | Memoize on first occurrence; clear on refresh | Key tags VV=0, EV=1, EE=2, FV=3; new trial contacts use frozen geometry/H, never trial geometry for kappa |
| Current `collision_set_`, same form | Topology plus per-stencil scalar | `init`, `solution_changed`, `update_quantities`; assignment follows rebuild | Value/gradient/Hessian and post-step distance statistics; plane-vertex assignment skipped as unused by PolyFEM |
| Candidate set, ContactForm | Geometry/topology | `line_search_begin` builds candidates; `line_search_end` clears/disables reuse | Collision rebuilds within that interval; caller must notify current x before refresh |
| Raw `r_i`, then `q_i`, local temporaries | J/L², then J/L⁴ | Cache miss during assignment | Nonfinite replacement then floor; not separately stored in production |
| `kappa_min_`, form configuration | J/L⁴ before division by a | Parsed at construction, clamped nonnegative; default 0 | Lower clamp on q; no automatic unit conversion here |
| `kappa_median_`, `kappa_cap_`, same form | J/(a L⁴) | Uncapped first pass at refresh; upper median includes zeros; spread default 10⁴ | Cap frozen for all subsequent new stencils; zero median gives zero cap; empty refresh gives infinity |
| `weight_`, base Form | Acceleration scaling, often T² transient; 1 static | `SolveData::update_dt` sets contact and friction weights | Divided out when caching, multiplied back in evaluation; must be fixed with snapshot for cancellation |
| `scale_`, base Form | Normalization factor | Explicit normalization API; NLProblem currently returns 1 | Divides final value/gradient/Hessian |
| Global trim `barrier_stiffness_`, ContactForm | Dimensionless multiplier | Initialization; bump/calibration; refresh controller; accepted-step emergency/downward control; stall retune | Multiplies all normal contacts; persists across subsolves |
| `trim_solve_anchor_`, `iters_since_trim_`, `iters_since_refresh_`, `kappa_snapshot_had_contacts_`, same form | Trim, counts, Boolean | Refresh anchors/reset; post-step counts; bump/calibration reset trim cadence | Emergency excursion limited to 256 times anchor; first-contact refresh detection; no complete rollback snapshot |
| `kappa_hessian_max_`, conditioning cap | J/L²; see below | Refresh max-entry scan; constructor option default 10³ | First-contact conditioning when no gradient balance and no collapse |
| Stall counters and saved initial x, ALSolver | Counts, L | Each `minimize_with_stall_restarts`; repeated hard stalls restore initial x | Retune callback, problem init and new minimize; full rollback audit belongs to RB-06 |
| L-BFGS corrections, previous x/g, stopping/search state, PolySolve Solver/strategies | L, J/L, mixed | Reset at `minimize` initialization; also selected strategy fallback resets | No coefficient-generation notification from contact post-step |
| Lagged tangential collisions, FrictionForm | Tangent basis dimensionless, positions L, normal-force scale from lagged potential | `init_lagging` / `update_lagging` rebuild normal collisions, assign frozen per-stencil kappa, copy trim into potential | Stored normal force stays unchanged through later normal-contact retunes until another explicit lagging update |

The provider reads ElasticForm's current derivative behavior, including its PSD
projection state. The contact coefficient routine does not itself enforce a
positive definite driving Hessian. Local extraction uses `to_full_vertex_id` and
skips blocks beyond Hessian dimensions (e.g. obstacles). A unique nonidentity
interpolation contract has **not** been established here; that is RB-03.
Changing a provider or form weight is not a cache invalidation API. Production
must refresh coherently after those changes. Same-form concurrent mutation is
not a supported claim.

## Trial and retune lifecycle

1. Begin a candidate interval, then notify each trial x. Collision rebuilds
   assign cached per-stencil values, computing missing keys from `X*,H*` and
   the frozen cap. No provider calls or trim update occur in this path.
2. Finish the line search before `post_step`. Iteration zero returns without
   controller work. A first contact after an empty snapshot refreshes immediately
   at an accepted iterate even when `refresh_interval=0`.
3. Post-step can change global trim after three iterations below the gap band;
   a slower downward step uses `controller_interval` (default 30). An optional
   positive `refresh_interval` additionally refreshes the full snapshot.
   Thus "frozen objective within a solve" in older comments is too strong:
   the snapshot is normally frozen **between refreshes**, and trim can change
   between line searches at default settings.
4. Refresh freezes geometry/H, clears memoized kappa, computes the batch cap,
   optionally runs the controller, then re-anchors the trim budget. It assigns
   the **current** collision set; it does not rebuild that set itself. Test
   callers notify x first. Synchronization after failed trial/rollback is an
   RB-06 concern, not an established result of this audit.
5. `bump_trim` clamps multiplicatively to `[2^-32,2^32]` by default.
   `calibrate_trim` requires opposing nonzero driving and barrier gradients
   (cosine at least .1), computes `-gB·gE/(a ||gB||²)`, and moves trim upward
   only. A true return can mean no change because the target is lower.
6. `retune_on_stall` first refreshes without controller, then bumps collapsed
   contacts or calibrates/softens according to the gap band. It does not itself
   reset search history or friction. The ALSolver caller subsequently invokes
   problem init and minimize; minimize resets descent-strategy history and
   stopping state. This is separate from in-solve post-step updates.

Source trace: `NonlinearElasticVarForm.cpp` lives under `src/polyfem/varforms/`.
Its on-stall callback only calls `retune_on_stall`. `FullNLProblem::init` calls
form `init`, not `init_lagging`. FrictionForm has an explicit lagging initializer,
so a stall restart does not automatically refresh lagged friction. Its outer
lagging loop updates friction before another solve-start barrier refresh.

PolySolve `Solver::minimize` resets strategies on entry. Within the loop it
retains `old_energy`, calls contact post-step after acceptance, and next evaluates
the changed objective. L-BFGS forms `y=g_new-g_previous` without a coefficient
version check. Consequently a post-step change may enter cross-objective energy
changes and secant differences. This is a source-traced limitation, **not** a
measured convergence failure or authorization to move retunes/change lagging.
No new final convergence criterion is introduced; AL feasibility preparation
and reduced configured minimization retain their PF-01 contracts.

## Unit covariance and model decisions

Under numerical conversion `x'=Lscale*x`, `E'=Qscale*E`, the frozen law without
active absolute safeguards gives `H'=Qscale/Lscale² H`,
`q'=Qscale/Lscale⁴ q`, `g'=Qscale/Lscale g`. This passes 27 combinations of
length, objective and form weight. Any positive kappa minimum must be converted
as `Qscale/Lscale⁴`; the literal nonfinite substitute `1e30` is not covariant.

The first-contact conditioning expression is
`conditioning_cap * max|H| / (a * median_kappa)`. Its ratio has dimensions L²,
so a dimensionless trim requires `conditioning_cap` to carry inverse-length²
or a specified length normalization. Current code treats it as a fixed numeric
option. With default numeric options and an unloaded .8*dhat contact, equivalent
length scales .001, 1, 1000 produce trims .001, 1, 1. This is an additional
measured unit dependence of the controller; the frozen-law conversion pass
must not be generalized to the complete algorithm.

Concrete alternatives requiring user selection and further validation:

| Issue | Alternatives and measured/derived consequences | Required validation before changing production |
| --- | --- | --- |
| Zero median erases a positive contact | Current `[0,0,100]` -> `[0,0,0]`; disabling the existing spread cap preserves `[0,0,100]`, with E=296.651596 and force norm=1431.493641 versus zero. A positive-only median would likewise preserve that positive contact here, but is a new cap law. | Heterogeneous coupled contacts, large-curvature conditioning, same-process/new-contact caps and unit conversion; neither option repairs zero individual kappa |
| Nonpositive normal curvature | Retain zero policy; choose a justified positive reference floor; or change driving curvature (e.g. PSD/reference Hessian). Existing `kappa_min=1` gives E=2.966516 and force norm=14.314936 for the negative fixture versus zero. These scale linearly with the chosen floor. PSD projection alone can still give zero. | Specify reference stiffness and units; derivatives at fixed state; balance/work and mesh/load refinement; no arbitrary minimum is a correctness proof |
| Stencil-switch jump | Retain/document piecewise discontinuity; or define coefficients on a consistent geometric parent interaction with matching feature limits; or choose a differentiable coefficient law and include its derivative terms. No latter alternative is implemented. | EV/VV and 3D feature-boundary continuity, energy derivatives through transitions, cap interactions, CCD and force/work measurements |
| Controller unit dependence | Declare inverse-length² units and convert the conditioning option; or redesign around normalized curvature, for example a specified dhat² factor. These have different parameter meanings; no default chosen. | Equivalent-unit controller runs with loaded/unloaded/collapsed contacts, transient weights, quantitative effect on conditioning and force |
| Retune and history/friction mismatch | Keep existing within-solve controller and lagging contract; explicitly restart/rebuild state after version changes; or freeze retunes between complete minimizations. At fixed x a trim doubling doubles contact E/g while lagged friction is unchanged. | User chooses lifecycle; test solver-history reset, bounded friction behavior, zero-displacement coefficient work, failed-attempt rollback and final convergence reporting |
| Invalid arithmetic | Replace nonfinite q by fixed 1e30 (current); reject invalid inputs/arithmetic; or define a finite dimensioned recovery law. Weight division and cap multiplication happen after the current finite check. | Clear failure/reporting contract, valid near-singular cases, underflow/overflow and no silent acceptance. The probe rejects evaluating nonfinite assigned coefficients. |

For fixed geometry and stencil set, changing effective coefficients gives
`Delta E = sum_i Delta(a*t*khat_i/s) b_i(x)` at **zero displacement**. This is
algorithmic objective change, not mechanical work `force·dx`; forcing positive
coefficients or increasing trim can inject such changes. The measured retune
jumps and feature jump are not a trajectory energy/work balance. RB-04 must
account for coefficient changes separately before physical certification.
