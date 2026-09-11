# Semi-implicit per-contact barrier stiffness

Current on the sdast9 fork's `main`; checked against the source and JSON schema
on September 5, 2026. The feature is opt-in at the solver level; the Houdini 2.0
asset selects it by default.

## Mechanics

The mode uses `ipc::semi_implicit_stiffness` to derive per-contact stiffness from
the local system Hessian (elastic curvature plus inertia for transient problems).
The integration passes zero explicit vertex masses to avoid freezing the
singular mass/distance-squared term, then divides the returned spring stiffness
by `dhat^2` for PolyFEM's squared-distance clamped-log barrier. Per-contact values
are capped relative to the batch median and applied through IPC's
`NormalCollision::stiffness_scale`, including friction's lagged normal force.

With `refresh_interval = 0`, the per-contact snapshot is normally held between
solve starts and stall restarts. Contact appearing after an empty snapshot also
triggers a refresh. The **global trim is not frozen**: gradient balance calibrates
it at refresh points; in-solve control can raise it for collapsing gaps and lower
it at `controller_interval` when the average gap remains above the band. Emergency
control considers the minimum gap as well as the average. Do not describe the
whole objective as fixed throughout every Newton solve.

The default squared-gap band is `[0.5, 0.9] * dhat^2`, actively centering the
contact gap. Small line-search steps or the soft iteration limit trigger bounded
restarts with retuned stiffness. Floor projection removes closing motion at tiny
gaps while allowing sliding and separation. Trial-displacement capping and
sequential step clamping apply only in semi-implicit mode.

## Usage and defaults

Minimal configuration:

```json
{"solver": {"contact": {"barrier_stiffness": "semi_implicit"}}}
```

All semi-implicit options are optional. Defaults from
[`json-specs/input-spec.json`](../../json-specs/input-spec.json):

```json
{
    "solver": {
        "contact": {
            "barrier_stiffness": "semi_implicit",
            "semi_implicit": {
                "refresh_interval": 0,
                "trim_lower": 0.5,
                "trim_upper": 0.9,
                "trim_factor": 2.0,
                "trim_min": 2.3283064365386963e-10,
                "trim_max": 4294967296.0,
                "kappa_min": 0,
                "kappa_spread": 10000.0,
                "gap_floor": 0,
                "trial_displacement_cap": 50.0,
                "force_continuation": true,
                "continuation_max_ratio": 0,
                "coefficient_identity": "parent",
                "conditioning_cap": 1000.0,
                "controller_interval": 30,
                "restart": {
                    "enabled": true,
                    "alpha_threshold": 0.01,
                    "patience": 5,
                    "min_iterations": 5,
                    "soft_iteration_limit": 100,
                    "max_restarts": 20,
                    "stall_trim_factor": 2.0
                }
            }
        }
    }
}
```

`gap_floor = 0` disables experimental force saturation. The constraint-floor
barrier deletion and direction projection have been removed. Old JSON inputs
may still contain `constraint_floor`; this compatibility key is ignored, with
a warning for nonzero values. It cannot reactivate the retired mechanism.
The [PF-02 investigation](../../docs/pf-02-contact-floor.md) records the original
defects; the [removal record](../../docs/pf-02-floor-removal.md) documents the
replacement behavior and targeted validation.
`trial_displacement_cap = 50` bounds trial surface displacement in barrier-support
units. It is enabled by default within semi-implicit mode, not in other modes.
`refresh_interval > 0` deliberately refreshes the snapshot inside the solve.

Coefficient law safeguards (RB-18, 2026-09-11; see
[docs/rb-18-quick-fixes.md](../../docs/rb-18-quick-fixes.md)): the batch
median is taken over positive values only, `kappa_spread` also sets a relative
floor `median / kappa_spread`, a stencil whose local curvature is nonpositive
or overflows keeps its previous coefficient — with no previous value a
negative curvature uses its magnitude |wᵀHw| and a zero one uses the global
scale max|H|/d̂² (user choice 2026-09-11; only an identically zero system
Hessian still yields κ=0) — overflow with no reference is an error, a NaN
curvature or an overflow with no reference is an error rather than a literal
1e30, and `conditioning_cap` is dimensionless (normalized by d̂²) so it binds
at the same trim regardless of length units. At d̂ ≪ 1 the cap now binds at
first contact where it previously never could; raise `conditioning_cap` to
loosen it.

The independent `solver.augmented_lagrangian.initial_weight = "hessian_scaled"`
option initializes the BC penalty from elastic Hessian magnitude, scaled by
`initial_weight_multiplier`. The fork normalizes the lumped BC metric to mean
diagonal one, preserving relative weights without physical mass units.

### Coefficient continuity (RB-20 / RB-21)

Each per-contact coefficient is keyed on the **parent candidate** that built
the collision (`coefficient_identity: "parent"`): the point/edge pair in 2D,
the point/triangle or edge/edge pair in 3D, estimated on the parent's stencil
with the closest point on the whole primitive. A built collision's
`stiffness_scale` is the contribution-weighted mean of its parents, so the
potential is continuous when the closest subfeature switches (EV↔VV, FV↔EV) and
the historical stencil jump is gone (`"stencil"` restores the old keying).

`force_continuation` (default on) keeps, at every between-steps refresh, the
coefficient that acted at the published endpoint for every active contact: the
contact force at the endpoint is unchanged by the refresh, so the published
state is an equilibrium of the state the next step starts from. A contact born
during a step is priced by that step's frozen snapshot and continued from its
publication on; only the global trim (band step, collapse bump, calibration)
still moves the forces between steps, and that is logged as a coefficient
event. `continuation_max_ratio: r > 1` lets a fresh Hessian estimate move a
continued coefficient within a factor `r` per endpoint (0 = pure
continuation). Continuation changes semi-implicit endpoints relative to the
re-estimating behavior (measured on the public smokes: ~1e-4 frictionless,
~1.6e-2 with friction, on a .25 displacement); `force_continuation: false`
restores it.

## Scope and limitations

- Supports the standard clamped-log `BarrierContactForm` for static, quasistatic,
  and transient forward solves.
- Physical barriers, GCP, periodic contact, and shape derivatives are rejected
  for this mode. Optimization's constant-stiffness requirement is separate.
- The documented validation uses `use_convergent_formulation = false`.
- Remeshing local relaxation does not preserve per-contact stiffness; combining
  it with this mode remains outside the documented validation.
- The earlier extreme thin-geometry/floating-point-floor experiments remain
  historical evidence, not a guarantee that every such scene converges.
- Steps with no active contact and a near-rigid displacement can drive the
  objective to its roundoff floor while ‖∇f‖ is still above tolerance; with
  TBB the summation-order noise then decides whether the line search stalls.
  Since RB-19 the `Armijo`/`RobustArmijo` line searches fall back to the
  gradient norm there (`solver.nonlinear.line_search.use_grad_norm_tol`, and
  the new `solver.nonlinear.line_search.Armijo.roundoff_tolerance`, default
  machine epsilon; set both to `0` for the previous behavior). Runs with
  `--max_threads 1` are bit-reproducible; threaded runs are not.

## Smoke scenes and tests

The scenes press a NeoHookean cube into a fixed slab. Their explicit JSON options
may override the current defaults above.

| Scene | Coverage |
| --- | --- |
| `quasistatic-semi.json` | Quasistatic semi-implicit contact |
| `quasistatic-adaptive.json` | Classic adaptive baseline |
| `transient-semi.json` | Transient semi-implicit contact |
| `quasistatic-semi-friction.json` | Friction coupling |
| `quasistatic-semi-alhess.json` | Hessian-scaled AL weight |

From the PolyFEM repository root:

```bash
./build/PolyFEM_bin --json scenes/semi-implicit/quasistatic-semi.json -o output/
(cd build/tests && ./unit_tests 'semi-implicit*')
```

Debug logs expose refreshed stiffness statistics, trim, and average/minimum gap
ratios. Before the September 5 promotion to `main`, both derivative tests passed
(160 assertions) and all five smoke scenes exited successfully without error log
lines. The September 4 full-suite record was 243/244 passing; the GCP
`cube-on-floor` reference mismatch remains unresolved and is not a semi-implicit
smoke scene.

## Companion revisions

CMake pins `sdast9/ipc-toolkit@e3c8d3fe` (per-collision `stiffness_scale`,
`compute_avg_distance`, and the RB-21 parent contributions on built
collisions) and `sdast9/polysolve@5afe3b5d`. The latter carries the PF-06
derivative correction and the RB-19 line-search fallback on top of `713220f`,
the upstream merge of the iteration-callback work. Dependency
feature branches and `main` branches are not interchangeable.
