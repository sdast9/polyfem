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
                "constraint_floor": 0.0001,
                "trial_displacement_cap": 50.0,
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

`gap_floor = 0` disables experimental force saturation. `constraint_floor = 1e-4`
enables floor projection and removes below-floor pairs from the barrier energy
on collision-set rebuilds within this mode. Set it to zero to disable both
effects. The [PF-02 investigation](../../docs/pf-02-contact-floor.md) records
the discontinuity, boundary projection limits, and comparison evidence; this
option is not a validated hard-contact formulation.
`trial_displacement_cap = 50` bounds trial surface displacement in barrier-support
units. It is enabled by default within semi-implicit mode, not in other modes.
`refresh_interval > 0` deliberately refreshes the snapshot inside the solve.

The independent `solver.augmented_lagrangian.initial_weight = "hessian_scaled"`
option initializes the BC penalty from elastic Hessian magnitude, scaled by
`initial_weight_multiplier`. The fork normalizes the lumped BC metric to mean
diagonal one, preserving relative weights without physical mass units.

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

CMake pins `sdast9/ipc-toolkit@9da3094` and `sdast9/polysolve@012658e`. The latter
has the same source tree as `713220f`, the upstream merge of the iteration-callback
work. Dependency feature branches and `main` branches are not interchangeable.
