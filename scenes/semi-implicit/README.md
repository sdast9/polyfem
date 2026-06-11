# Semi-implicit per-contact barrier stiffness

This branch adds an opt-in barrier stiffness mode based on
[Ando 2024, "A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness"]:
each contact gets its own stiffness

```
kappa_i = m_i / d^2 + w^T H_local w
```

computed from the lumped vertex masses and the local block of the (weighted)
elastic Hessian via `ipc::semi_implicit_stiffness`. The stiffness snapshot is
refreshed at every solve start and stall restart (and optionally every
`refresh_interval` Newton iterations; the default 0 keeps the objective
frozen within a solve so Newton converges to a fixed point). On top of the per-contact values, a global **trim** factor is
adjusted by a two-sided controller -- one step per refresh point (subsolve
start or stall restart), never inside a Newton solve -- that keeps the
average active squared gap inside `[trim_lower, trim_upper] * dhat^2`. The default band is a *safety*
band against gap collapse and over-stiff barriers — the per-contact values
alone usually land the gap in a healthy range; raise `trim_lower` to ~0.5 to
actively push the gap towards `dhat` instead. A **stall detector** watches
the line-search step size and the iteration count, and restarts the nonlinear
solve with retuned stiffness (direction chosen from the current gap) when the
solver stops progressing.

Unlike the classic `"adaptive"` heuristic (mass/bbox-scaled initial value,
one-sided emergency doubling, no adaptation at all for static/quasistatic
solves), this mode scales with the actual elasticity of the problem and works
in quasistatic solves.

## Usage

```json
"solver": {
    "contact": {
        "barrier_stiffness": "semi_implicit"
    }
}
```

All knobs (defaults shown; all optional):

```json
"solver": {
    "contact": {
        "barrier_stiffness": "semi_implicit",
        "semi_implicit": {
            "refresh_interval": 0,
            "trim_lower": 1e-4,
            "trim_upper": 0.9,
            "trim_factor": 2.0,
            "trim_min": 1.52587890625e-05,
            "trim_max": 65536.0,
            "kappa_min": 0,
            "restart": {
                "enabled": true,
                "alpha_threshold": 1e-4,
                "patience": 5,
                "min_iterations": 5,
                "soft_iteration_limit": 100,
                "max_restarts": 5,
                "stall_trim_factor": 2.0
            }
        }
    },
    "augmented_lagrangian": {
        "initial_weight": "hessian_scaled",
        "initial_weight_multiplier": 10.0
    }
}
```

`augmented_lagrangian/initial_weight: "hessian_scaled"` is independent of the
contact mode: it sets the initial AL weight to
`initial_weight_multiplier * max|H|` of the elastic Hessian at solve start so
the boundary-condition penalty dominates the problem curvature.

## Limitations (v1)

- Only the standard `BarrierContactForm` (clamped-log barrier). Not supported
  with: `use_physical_barrier`, GCP (`use_gcp_formulation`), periodic
  contact, or shape derivatives (these error out explicitly).
- Validated with `use_convergent_formulation = false`; the Ando derivation
  assumes the plain IPC collision set.
- Remeshing local relaxation uses a fixed stiffness equal to the current trim
  (not per-contact); avoid combining remeshing with this mode for now.

## Smoke scenes (this directory)

A 4x4x4-cell NeoHookean cube (E = 1e7, nu = 0.45) pressed 0.25 of its height
into a fixed slab by a Dirichlet BC on its top face, with `dhat = 1e-3`:

| scene | what it exercises |
| --- | --- |
| `quasistatic-semi.json` | quasistatic + semi_implicit |
| `quasistatic-adaptive.json` | classic adaptive (regression baseline) |
| `transient-semi.json` | true dynamics + semi_implicit |
| `quasistatic-semi-friction.json` | friction coupling (mu = 0.3) |
| `quasistatic-semi-alhess.json` | hessian_scaled AL initial weight |

Run e.g.:

```bash
./PolyFEM_bin --json scenes/semi-implicit/quasistatic-semi.json -o output/
```

Watch the debug log for `Semi-implicit barrier stiffness: trim=... ,
sqrt(avg d2)/dhat=...` — the gap ratio should stay above ~0.01 —
and `Refreshed semi-implicit barrier stiffness over N contacts: min/mean/max`.

## Companion branches

- ipc-toolkit `semi-implicit-stiffness`: per-collision `stiffness_scale`
  applied in `BarrierPotential`/friction + `NormalCollisions::compute_avg_distance`.
- polysolve `iteration-callback`: `Criteria.alpha` + `Solver::set_iteration_callback`
  (cherry-picked from upstream `experimental`).

Build against the local copies with:

```bash
cmake -S polyfem -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCPM_ipc-toolkit_SOURCE=/path/to/ipc-toolkit \
  -DCPM_polysolve_SOURCE=/path/to/polysolve
```
