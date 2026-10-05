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
                "band_statistic": "rms",
                "initial_trim_estimate": false,
                "clamped_contacts": "exclude_statistics",
                "gradient_balance_dofs": "all",
                "restart": {
                    "enabled": true,
                    "alpha_threshold": 0.01,
                    "patience": 5,
                    "min_iterations": 5,
                    "soft_iteration_limit": 100,
                    "max_restarts": 200,
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
An edge/edge pair (and a pair of codimensional points) is keyed on its
**sorted** primitive pair, whichever order the broad phase emitted it in
(2026-10-05, [record](../../docs/canonical-pair-keys-20261005.md)); before,
one such contact could carry two keys, and continuation silently re-estimated
it whenever the emission order flipped. Point/edge and point/triangle keys are
typed and never had two orders; `"stencil"` keys are canonical in the same way.

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

### Trim controller options (EF-02/03, EF-07, clamped contacts; 2026-09-26 – 29)

The production controller is the global gap-band trim with the collision-weighted
mean `sum(w d²)/sum(w)` as its statistic (`band_statistic: "rms"`; the statistic
is weighted by collision weight since `6a4e788bf`, so it no longer depends on how
IPC resolves a distance-type tie). The keys below change what feeds it. All
records are in the [contact-efficiency plan](../../docs/contact-efficiency-plan-20260923.md);
the standing recommendation (user decisions of 2026-09-28/29) is to leave
everything except `clamped_contacts` at its default.

| Key | Default | Meaning |
| --- | --- | --- |
| `clamped_contacts` | `exclude_statistics` | Contacts whose every vertex is Dirichlet-clamped (obstacles included) stay in the energy and CCD but are ignored by the trim controller's statistics. `keep` reproduces runs made before 2026-09-29 that had such a contact; `exclude_collisions` is an unrecommended experiment (a prescribed block passed through a clamped one silently). [Record](../../docs/clamped-contacts-20260928.md). |
| `gradient_balance_dofs` | `all` | `free` takes the gradient balance over free DOFs only. Only the clamped half of contacts against a clamped body moves the balance, by a factor set by that body's mesh (9× on the smokes' two-triangle slab, 2.1× on IT). Opt-in; the default was kept by user decision. [Record](../../docs/gradient-balance-free-dofs-20260929.md). |
| `band_statistic` | `rms` | `force_weighted` is **experimental** (Houdini shows a warning). It failed the held-out `pup_push` scene in 2 of 4 runs and is not adopted. |
| `collapse_guard_basis` | `pair` | Only with `force_weighted`: the gap the band's downward guard is evaluated on. `pair` removes the ball-burst trim loop; `proxy` reproduces the EF-02/03 records and is warned. [EF-07](../../docs/ef-07-trim-loop.md). |
| `initial_trim_estimate` | `false` | Opt-in two-sided, cosine-gated first-stall estimate. Byte-inert wherever its cosine stays below 0.8; saves R4 step 1 (583/510 → 89/133 iterations). Not adopted pending an agreed accuracy standard ([assessment](../../docs/default-controller-assessment-20260929.md)). |
| `restart/alpha_basis` | `absolute` | `feasible_bound` counts a small step toward the stall patience only when the search backtracked below the feasible bound. No benefit measured; opt-in ([EF-04](../../docs/ef-04-stall-trigger.md)). |

Any run that selects an experimental option records it in `run-manifest.json`.

### Restart and reproducibility

A `restart.json` resumes the run it came from: it keeps `dt` and the remaining
steps, absolute paths and the original `sim.pvd` frame times, and its state file
carries the contact controller's memory (trim, per-contact stiffness caches,
continuation keys, realized friction lag) and the augmented-Lagrangian
multipliers. The first resumed step is bit-identical; later steps differ by
one-ulp step-time roundoff. State files are sized to their data
(`c2a57e393`). Older state files resume with a fresh controller and a warning
([restart](../../docs/restart-json-20260927.md),
[AL multipliers](../../docs/restart-al-multipliers-20260928.md),
[state files](../../docs/state-file-chunks-20260927.md)). State files written
before 2026-10-05 store edge/edge and vertex/vertex coefficient keys in the
order the broad phase emitted them; they are converted to sorted keys on read
(where both orders of one pair are stored, the continued or endpoint entry
wins, otherwise the sorted one), so a resumed run keeps its continuation
([record](../../docs/canonical-pair-keys-20261005.md)).

On macOS the default `Eigen::AccelerateLDLT` used vecLib threads that ignored
`--max_threads` and were not bitwise deterministic. Since `ba3ea76b6`
`--max_threads N` also caps Accelerate (`VECLIB_MAXIMUM_THREADS`), and the
manifest records `process.threads.accelerate`; single-threaded runs then repeat
bit-identically ([record](../../docs/it-reproducibility-20260928.md)). Two
different deterministic realizations of a trajectory-sensitive scene can still
differ (IT: up to 23.5 %).

Across CPU vendors, MKL's runtime dispatch can change low-order bits of a dense
factorization; on contact scenes with a chaotic trim or barrier-wall history
(the `parallel-edge` GCP scene) that decides whether a step takes 56 iterations
or runs past the Newton limit. The CI Linux and Windows test steps set `MKL_CBWR=COMPATIBLE`
([record](../../docs/parallel-edge-regression-20260930.md)); set it too when
comparing runs across machines.

### Augmented-Lagrangian budget (RB-07)

The AL stage that prepares a geometrically safe snap to the prescribed
Dirichlet values is unbounded by default: a prescribed motion that can never
be snapped (a face driven through an obstacle, a body crushed to zero height)
keeps the loop running at the weight ceiling until the subsolves degrade. The
opt-in `solver.augmented_lagrangian.budget` ends it with a named failure
(exit status 1; the step is rolled back, RB-06, and nothing of it is
published): `max_passes` is a plain cap on the passes per solve;
`stagnation_window: W` ends the stage after `W` consecutive passes at the
weight ceiling over which none of the progress signals moved — the BC residual
(`progress_tolerance`, relative), the snap's gates (finite energy, validity,
collision-free), the collision-free fraction of the snap (`snap_tolerance`)
and the iterate's drift relative to the largest constrained residual
(`drift_tolerance`). Both default to off; every run's manifest carries, per AL
pass, the BC residual, the gate that blocked the snap and the CCD fraction,
and a budget failure adds the stage's reason and full pass history
(`steps[].al_stagnation`). Measured on the public fixture (2026-09-15): a
compatible 0.3 compression under interrupted passes needs 105 passes and is
unchanged under `{max_passes: 200, stagnation_window: 3}`; a bottom face driven
0.05 into the slab stagnates at pass 10 (passes 8–10 at the ceiling); a top
face driven down by the cube height is not stagnant by these signals (the
crush keeps moving) and ends by the pass cap. A budget's memory is bounded
(RBR-03, 2026-09-21): the stage keeps at most `stagnation_window + 1`
full-space states for its motion measures (one under a pass cap alone) —
the count is in every pass record (`retained_states`) and manifest AL entry
(`al_retained_states`); the scalar pass history itself still grows with
the passes. See
[docs/rb-07-validation.md](../../docs/rb-07-validation.md).

### Run manifest (RB-12)

Every `PolyFEM_bin` run writes `run-manifest.json` into its output directory
(`output/manifest`; `""` disables it, the library default is off) and rewrites
it after every step and at completion, so a run that stops leaves its
identity behind. Schema `polyfem.run-manifest` version 1 (fields have been
added since, none has changed meaning):

| record | content |
| --- | --- |
| `build` | compiled in before every build (`polyfem.build-info`): the effective PolyFEM / IPC Toolkit / PolySolve checkouts — commit, branch, dirty state, a SHA-256 of the uncommitted patch (tracked diff plus untracked file hashes) — next to the pins the recipes declare and whether the effective commit matches them (a local `CPM_<name>_SOURCE` override builds whatever is checked out there); compiler, configuration, generator, threading backend, options |
| `process` | the executable's own SHA-256, size and mtime (binary identity, distinct from source identity), command line, working directory, pid, `uname`, hardware concurrency, requested / effective thread count |
| `input` | the input file and every `common` file with their hashes, the effective input after defaults and command-line overrides (`effective`, hashed as its canonical serialization, once as is and once without `root_path` / `output/directory` so repeats from different directories share a hash), every string of it that resolves to an existing file (meshes, per-element value files, selections, restart states — `referenced_files`, with hashes), the unit system and `characteristic_force_density` (setting and effective value, since the stopping tolerance scales with it — RB-09) |
| `solver` | linear / nonlinear solver summary, the contact settings, and `model`: what the contact form implements (stiffness mode, the coefficient law and its lineage, coefficient identity, continuation, friction lag, controller constants, gap convention, model-selection status, the fallbacks the run can take) |
| `diagnostics` | the opt-in RB-04 streams and the versions of every record schema this binary writes |
| `steps` | one record per solve: outcome, phase reached, wall time, termination (restarts, iterations, reason), every subsolve, stall retunes, lagging state, and the contact form's state (active count, trim, refresh id, batch median / floor / cap, fallback and continuation counts, candidate counts, and `history_sensitivity`: what in the step depended on discrete history, below) |
| `completion` | `completed` / `failed` / `resource_failure`, exit status, message, wall time, peak RSS |
| `producer` | the input's `provenance` block when it carries one (the Houdini asset: producer, version, asset name / version / SHA-256, scene, export time) |

The RB-04 streams of the same run carry the manifest's `run_id`. Absent
measurements are `null` with an `unavailable_reason`; the manifest never
guesses. Documented in [docs/rb-12-validation.md](../../docs/rb-12-validation.md).

#### History sensitivity (`steps[].contact.history_sensitivity`, 2026-10-05)

A semi-implicit step's result can depend on discrete history: a contact that
lost its continued coefficient is re-estimated at a different state, and the
trim moves in discrete steps, one of them on an iteration count
([record](../../docs/canonical-pair-keys-20261005.md), evidence
`semi-implicit-roundoff-work/FINDINGS.md` in the parent workspace). This block
says how much of that happened since the previous step record (the
between-steps refresh after a record belongs to the next one). It is
observational: solutions are byte-identical with and without it, and a failed
attempt's rollback restores it with the form. Other stiffness modes report
`{value: null, unavailable_reason}`.

| field | meaning |
| --- | --- |
| `continuation.force_continuation` | RB-20 continuation on; with it off nothing is carried and nothing can be lost |
| `continuation.losses.orientation` | coefficient keys given a fresh estimate (at an accepted iterate or a mid-solve refresh) while the same pair was carried under its other order — **0 with canonical pair keys**; a regression guard |
| `continuation.losses.reentry` | keys given a fresh estimate that had been carried at a capture of this step or the previous one (the endpoint refresh between steps, an AL pass or reduced-solve start, a lagging iteration) and were dropped by a later capture: not active at the last endpoint, then back |
| `continuation.losses.other` | keys active at such a capture whose coefficient continuation could not carry (not positive and finite) |
| `continuation.split_pairs` | edge/edge and vertex/vertex pairs memoized under both of their orders at an accepted iterate or refresh: one contact with two estimates, carried or not — **0 with canonical pair keys**; a regression guard |
| `continuation.max_abs_log_ratio` | largest \|ln(fresh / carried)\| among the losses (null when none had a carried value) |
| `continuation.gap_shift_dhat` | `{sum, max, contacts}` over the endpoint's contacts whose coefficient includes a lost key: ½ (1 − d/d̂) \|ln(s / s_carried)\|, s the contact's coefficient and s_carried the same with every lost key at its carried value — the first-order shift of its equilibrium gap, in units of d̂ |
| `trim.start`, `trim.end` | the trim at the previous record (the run's first refresh for the first record) and now |
| `trim.moves.<source>` | `{count, log2}` of the trim moves by source: `collapse` (proportional upward bumps), `calibration` (gradient balance), `conditioning_cap` (first-contact cap), `cadence_down` (the in-solve downward step after `controller_interval` iterations), `refresh_down` (a refresh's downward step without a balance signal), `stall_soften` (blind softening at a stall retune), `force_band`, `initial_estimate` (opt-in experiments), `other` (direct library calls); the log2 values add up to log2(end / start) |
| `trim.moves_in_stall_retunes` | how many of those moves stall retunes made (the retunes are `steps[].stall_retunes`) |
| `trim.iters_since_trim`, `trim.controller_interval` | Newton iterations since the trim last moved at the record, against the cadence of the downward step (it fires when the gap is pinned above the band and this count reaches the interval) |
| `trim.gap_shift_per_cadence_move_dhat` | `{sum, max, contacts}` of ½ (1 − d/d̂) ln(trim_factor) over the endpoint's active contacts: how far one more or one fewer cadence step would move the equilibrium gaps; null with `controller_interval: 0` or the force-weighted band |
| `restored_from_state` | the accumulation began at a state restored from a restart file (the refresh that preceded the save is not repeated, and the previous step's captures are not saved, so a re-entry can be missed) |

Gap shifts leave fully clamped contacts out (they cannot move) and are null for
a failed attempt. They are estimates from the log barrier at first order,
within a factor 2–3 of measured branch differences; contacts born at a
CCD-truncated iterate (degenerate estimates, FINDINGS mechanism B) are not
covered.

## Scope and limitations

- Supports the standard clamped-log `BarrierContactForm` for static, quasistatic,
  and transient forward solves.
- Physical barriers, GCP, periodic contact, and shape derivatives are rejected
  for this mode. Optimization's constant-stiffness requirement is separate.
- The documented validation uses `use_convergent_formulation = false`. With
  the convergent formulation, `use_improved_max_operator` is refused by name
  (RBR-04, 2026-09-21: its duplicate-removal corrections are negative
  collision weights, under which the parent-keyed coefficient is no longer a
  sum of parent potentials — a measured `|κ₁−κ₂|/2·b(d)` energy jump where
  two edges with unequal coefficients meet), as is `use_physical_barrier`;
  `State::init` names both at once. Area weighting alone is accepted but
  not part of the documented validation. The improved max operator stays
  available to `adaptive` and fixed stiffness.
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
| `quasistatic-semi-friction.json` | Friction coupling (since RB-10, 2026-09-13, at the new defaults `friction_iterations: 2`, `semi_implicit/friction_lag: realized_force`; the pre-RB-10 endpoint needs both historical settings written explicitly) |
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

CMake pins `sdast9/ipc-toolkit@1f1b5dbf` (branch `semi-implicit-stiffness`:
per-collision `stiffness_scale`, `compute_avg_distance`, the RB-21 parent
contributions on built collisions, the RB-05 broad-phase budget with checked
counting, the merged upstream `869e489e`, Tight-Inclusion 1.1.0 and the
canonical smooth-contact collision order, and the `solve_spd_2x2` iterative refinement for nearly parallel edges) and `sdast9/polysolve@43ca2e66`
(branch `iteration-callback`: the PF-06 derivative correction, the RB-19
line-search fallback, the slope tolerance limited to Hessian-based strategies,
the opt-in Wolfe search, the uphill refusal in Armijo, and the merged upstream
hybrid solvers). Test data is pinned to `sdast9/polyfem-data@aed03ab`
(`fable-fixtures`). Dependency feature branches and `main` branches are not
interchangeable. A run's `run-manifest.json` records the effective checkouts
next to these pins.
