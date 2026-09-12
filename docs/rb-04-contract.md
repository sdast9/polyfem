# RB-04 — Observational endpoint contract, version 1

## Scope and lifecycle

`output.physical_diagnostics` defaults to false. When true, the active nonlinear
elastic VarForm appends JSON Lines to `physical-diagnostics.jsonl`. Each record
has schema `polyfem.physical-diagnostics`, version 1, a run ID, step, outer attempt
1, phase, outcome, termination data and elapsed solve time. Internal AL/reduced
restart counts retain their existing subsolve meaning; outer attempt 1 does not
mean that no internal restart occurred. No new retry policy is implemented.
Repeated invocations append distinguishable run IDs rather than truncate history.

The recording boundary is return/exception of `solve_tensor_nonlinear`, before
callbacks, VTK publication and time-history advancement. `accepted` means this
method returned under the **existing** numerical/finite-lagging policy. It is
not a physical pass, disk-publication guarantee or a new convergence test.
`failed_attempt` means a caught solver exception, which is rethrown unchanged.
Failures before this method (input/setup) and fatal signals cannot be recorded
here. A failure record's coordinates are the retained caller `sol`, with current
attempt form state, **not** an exposed failed internal Newton trial or a
transactionally restored last accepted state. RB-06 owns rollback.

Final AL/reduced termination comes from `ALSolver::info`, including configured
slope/gradient criteria and restart counts. Lagged minimize termination comes
from PolySolve; the independently measured updated-lag residual and convergence
flag are separate. The existing finite lag budget can return a nonconverged lag.
On failure, `subsolve_state_at_failure` is captured before the AL solver dies;
the exception and phase remain explicit. Unknown initialization termination has
an unavailable reason.

## Coordinates, units and signs

Let `s` be the current time integrator's `acceleration_scaling()` (1 for static
without an integrator). Quasistatic time stepping still has an integrator and
scaled energy forms; it must not be assumed that `s=1` in quasistatics.
The active `NLProblem::normalize_forms()` returns 1. Version 1 uses this active
path and does not claim compatibility with independently normalized embedded
`FullNLProblem` forms.

For each supported enabled form, record its objective `phi_i`, actual weight,
`phi_i/s`, and full gradient `g_i/s`. These use internal length, force and energy
units. `phi_i/s` for the inertia form is an **incremental inertial objective**,
not kinetic energy; the friction potential is not integrated dissipated work.
Elastic and barrier energies are separately named. Coefficient ranges refer to
stored per-stencil `stiffness_scale` before the form weight/global trim, with
active zero/nonfinite counts. Their dimensions depend on the chosen barrier
variant; they are not dimensionless trim values. See the RB-02 units contract.

The residual is `r = sum_i g_i/s`, excluding AL penalty/multiplier terms. Free
residual uses the existing `NLProblem::full_to_reduced_grad(r)` chain rule, not
an inference from the solver's stopping status. Dirichlet BC error is
`max(abs(A*x-b))` with the actual selector matrix, in internal length units.
The support force on the system is `A^T*A*r` for that selector, as a full
node-major DOF vector including appended obstacles; force on the support has
the opposite sign. Non-selector constraints are not reported as simple support
reactions. Full residual norm need not vanish on constrained DOFs.

Supported components: elastic, body, inertia, frozen-lag friction, barrier
contact and an empty pressure-boundary/cavity form. Nonempty pressure loads and
other active forms explicitly make the full residual unavailable; no omitted
form is silently counted as zero. Version 1 tests the public P1 Neo-Hookean
fixtures and exact collision selectors, not a general constitutive/material or
interpolated-stiffness certification.

Kinetic energy is `0.5*v^T*M*v`, with `v=compute_velocity(x)` before history
advancement and the assembled physical mass. It is unavailable for
quasistatic/static models rather than reporting artificial loading velocity as
physical kinetic energy. The transient reference uses the fixture's
ImplicitEuler rule and exact P1 consistent-mass integration. Existing nonlinear
VTU velocity exports use `v_prev()` at this boundary and lag a step; this
pre-existing output mismatch is documented, not used as the kinetic oracle.

## Observation without retuning

### Physical-state pairs (2026-09-09)

The final `lagging` object optionally retains `friction_before_update` and
`friction_after_update`, each with the full friction gradient in physical force
units, evaluated at the same returned coordinates. Missing observations carry
an unavailable reason. These label the two sides of the final lag update, not
every earlier lag event. Calling the const friction derivative does not advance
the lag state or integrator. Original numerical and finite-lag policy stays intact.

For supported complete residuals, `full_residual_with_pre_update_friction` and
`free_residual_norm_with_pre_update_friction` replace only the endpoint's friction
gradient by the observed pre-update gradient. Other endpoint forms remain fixed.
This is an explicitly identified force-state reconstruction; it does not assert
that an arbitrary returned solve is converged. The measured default fixtures
have small pre-update-friction residuals and much larger updated-lag residuals.

`barrier_start_energy_with_endpoint_snapshot` evaluates the solver's incoming
coordinates with a private copy of the returned endpoint coefficient snapshot.
In the tested fixed-mesh time loop those coordinates are the previous physical
endpoint. It supplies B(x_(n-1);theta_n) for the decomposition in the
[work convention](rb-04-work-convention.md). It is neither the original start
energy nor an integral of numerical retune events. It remains meaningful as an
energy evaluation across feature regions, but equality to a gradient line
integral requires separate continuity/path-quadrature evidence. Extra observations
remain opt-in; diagnostic on/off comparisons cover the added evaluations.

Contact geometry/stencils are rebuilt in a **copy** of the barrier form, using
a fresh broad phase. Only the copy can memoize new stencil coefficients. It
retains the frozen production snapshot, cap and trim. It never calls refresh,
calibration, post-step, lag update or a driving Hessian/gradient provider.
The production collision cache, coefficient map and broad phase remain intact.
A per-form monotonic refresh ID identifies completed production refreshes.
`iterations_since_refresh` exposes the existing optional periodic-refresh counter;
it advances only when periodic refresh is enabled and is not a total Newton
iteration count.

Other supported form value/gradient calls are const; no lagging or history
advance occurs. Fresh element assembly values sample det(F) at the element's
assembly quadrature points without changing the production assembly cache.
The minimum is a sample statistic, not a global injectivity/CCD certificate.
Gap is the minimum over rebuilt **active** stencils; if none exist the global
minimum is unavailable rather than infinity represented as an apparent pass.
The swept candidate cache normally is inactive at the endpoint, so its count
has an unavailable reason. RSS is in bytes; platform zero means unavailable.

Diagnostic errors are recorded where possible and emission errors are warned;
neither is allowed to turn a successful solve into a failure or swallow the
original solver exception. Numerical on/off equivalence is tested separately
from timing; diagnostics incur extra assembly, broad-phase and serialization
work. Large full-DOF records are opt-in.

## Accounting still pending

### Coefficient-event extension (2026-09-09)

With the same opt-in flag, the active VarForm now also appends
`coefficient-events.jsonl`, schema `polyfem.coefficient-event`, version 1.
Each outer refresh, calibration, stall retune or noninitial post-step operation
records a per-form event ID, run ID, physical step, solver phase, identical full
coordinates, before/after private snapshot energy and gradient in objective
units, raw state and evaluated snapshot state, effective form weight, and whether
the observed operation threw. Nested operations are counted only at their outer
boundary. No-op operations remain recorded; tiny nonzero differences from
independent broad-phase summation are roundoff, not evidence of an actual retune.

The writer divides the objective change by the current acceleration scaling to
report `energy_change_at_fixed_coordinates`. It also pulls the before/after
contact-gradient difference into free coordinates and reports its norm in force
units. This is a contact-force change, not a total residual under a new timestep's
inertia/loading/BC state. Initial coefficient setup has no established prior
snapshot; its before objective and change are unavailable, not presumed zero.

The transient/quasistatic time loop observes its post-publication refresh as
`between_steps_after_endpoint`, including the final scheduled step. These records
must not be attributed to the earlier saved endpoint's coefficient state. The
phase labels distinguish initialization, AL, reduced, lagging and between steps;
they do not yet enumerate every internal AL/reduced subsolve/restart. A stall
retune event is explicitly named and survives the tested failed attempt.

Observation copies and rebuilds the contact state privately and does not call
providers or retune. Observer errors are warned and cannot alter the solve's
exception/success behavior. Disabled observation avoids snapshot allocation.
This stream covers the active VarForm's listed coefficient operations. Manual
initialization setters, direct external bump calls without coordinates, embedding
entry points, changes in form/time weights and coordinate-only feature switches
are outside this stage. The public fixed-dt fixtures have constant time weights.

An optimization iterate is not a physical trajectory point. The sum of event
energy changes cannot be subtracted blindly from physical external work minus
endpoint energy. A complete budget must specify its contact-work path, coefficient
state along that path, and parameter-change term. Existing endpoint version 1
therefore still reports its aggregate `retuning_energy_change` as unavailable.
The new stream supplies evidence, not a completed trajectory balance.

These are **endpoint and scoped coefficient-event instrumentation stages**, not full completion of RB-04.
The following keys have explicit unavailable reasons in version 1:

- `proposed_displacement`: all line-search proposals are not retained.
- `external_work`: no path quadrature of body/traction/prescribed-boundary work.
- `frictional_dissipation`: no trajectory integration of force times relative slip.
- `retuning_energy_change`: endpoint data do not capture every fixed-coordinate
  coefficient change, including changes inside post-step/restart/refresh.
- `physical_balance_pass`: no complete budget or selected physical threshold.

Endpoint energy differences are **not** labeled mechanical work or frictional
dissipation. Before asserting balance, the next stage must record coefficient
events at identical coordinates, define work quadrature (including its endpoint
and lag-state convention), quantify quadrature/time-discretization error, and
account for the RB-02 stencil-transition jumps. No coefficient/interpolation
model, global convergence gate, CCD policy, trial cap or friction budget changes
are part of this implementation. See the validation record for measured results
and the exact remaining acceptance work.


## Optional contact-path observations

`output.physical_diagnostics_contact_path=false` by default. With ordinary physical
diagnostics enabled, accepted endpoints may include `contact_path`: 1025 private
frozen-snapshot samples (`t`, objective energy, objective directional derivative,
contact-key signature), nested trapezoidal estimates and 24-bisection transition
brackets with multi-signature warnings. No production snapshot is advanced.
Postprocessing divides energy/work by the endpoint acceleration scaling. The
scope is the straight start/end displacement segment with frozen endpoint
coefficients; it is neither a Newton-history trace nor a collision certificate.
Errors remain unavailable with a reason. Up to 1024 changed grid intervals can
trigger 24 extra evaluations each; this is an expensive research option.
See [validation and scope](rb-04-validation.md#actual-contact-path-measurements-and-rb-04-disposition-2026-09-09).


## Version 2 — attempt observation, candidate counts and discrete work increments (2026-09-12)

The record schema stays `polyfem.physical-diagnostics`; `version` is now 2.
Every version 1 field keeps its meaning. Version 2 resolves the fields that
version 1 carried as unavailable, except `physical_balance_pass`, which stays
unavailable by decision: no physical acceptance threshold is authorized for
RB-04, and the discrete budget terms are reported separately so that a later
item can select one.

### Iteration observer and the solver-attempts stream

`FullNLProblem` accepts a passive iteration observer. It is installed only
while `physical_diagnostics` is on, for the duration of one
`solve_tensor_nonlinear` call, and cleared on every exit path. It sees the
PolySolve hooks the problem already receives, in full coordinates: the trial
sweep handed to the contact broad phase (`line_search_begin`), the forms'
inversion/CCD step bound (`max_step_size`), every line-search validity trial
(`is_step_valid`), the release of the swept cache (`line_search_end`) and every
accepted iterate (`post_step`). An observer that throws is disabled for that
solve with a warning; it cannot change the solve. Disabled observation costs a
null check per hook.

Two facts of the existing solver stack shape the stream. `ALSolver` performs a
feasibility check before each subsolve (`line_search_begin`, `is_step_valid`,
`line_search_end`, no step bound); it is counted as `feasibility_checks`, not
as a proposal, and its validity trial is not a line-search trial. PolySolve's
`post_step` reports the number of iterations completed *before* the call, so
the start point and the first update both carry 0; the stream identifies the
start of a minimize as an accepted iterate with no pending proposal and numbers
updates from 1, so `accepted_iterations` equals PolySolve's `iterations`.

With the same opt-in flag the active VarForm appends `solver-attempts.jsonl`
(schema `polyfem.solver-attempt`, version 1), flushed per row so that a fatal
solver exception leaves a complete history. Each row carries the run ID, step,
phase (`augmented_lagrangian`, `reduced`, `lagging`), `minimize_index` (count
of PolySolve minimize calls in this attempt), `iteration` and `kind`:

- `start`: the start point of a minimize (`energy_objective_at_x0`; the
  gradient is not evaluated yet and is null).
- `accepted`: `trial` (`norm`, `linf`, `step_bound`, `validity_checks`,
  `validity_rejections` of this iteration's line search) and `accepted`
  (`fraction_of_trial`, `norm`, `linf`). The trial is the sweep handed to the
  broad phase after PolySolve's finite-energy stage; `fraction_of_trial` is the
  accepted fraction of that sweep, so `0 < fraction ≤ step_bound`.
- `rejected`: a trial with a step bound that no accepted iterate followed (the
  line search failed and the solver changed strategy, or the attempt failed).

`energy_objective_at_x0` and `gradient_norm_objective_at_x0` are PolySolve's
solver info at `post_step`: the objective and gradient norm at the iterate the
direction was computed from, not at the accepted point. Norms are Euclidean and
Linf over full node-major DOFs in internal length units. The stream is solver
bookkeeping; it certifies nothing physical.

### Endpoint record additions

- `attempt_summary`: `minimize_calls`, `accepted_iterations`,
  `rejected_proposals`, `feasibility_checks`, `step_bound_limited` (accepted
  iterations whose forms' bound was below 1), `line_search_truncated`
  (accepted fraction below the bound, i.e. energy backtracking beyond it),
  `validity_checks`/`validity_rejections`, `stall_retunes` (calls of the
  stall retune hook in this attempt) and `broad_phase_candidates`
  (`builds`, `last`, `max`). AL weights and restart counts remain in
  `termination` and the coefficient-event stream.
- `proposed_displacement`: the last Newton proposal accepted before the record
  (`trial_norm`, `trial_linf`, `step_bound`, `accepted_fraction_of_trial`,
  `accepted_norm`, `iteration`, `minimize_index`); unavailable when no
  proposal was observed. Every proposal is in the stream.
- `contact.candidate_count`: the retained counts of this solve's trial sweeps
  (`value` = last build, `max`, `builds`), since the swept cache is cleared at
  `line_search_end` and cannot be read at the endpoint. The statistics are
  reset at the start of every `solve_tensor_nonlinear` call; a form that built
  no candidates reports the reason. `builds` counts the feasibility checks too.
- Failed attempts: `last_internal_iterate` (`value` in full coordinates,
  `iteration`, `minimize_index`) is the last accepted Newton iterate observed
  inside the failed attempt. `endpoint` remains the retained caller
  coordinates; neither is a restored accepted state (RB-06).
- `barrier_energy_at_solve_start`: the start coordinates evaluated with the
  production coefficient state at solve start through a private snapshot.

### Right-endpoint discrete work increments

For an accepted endpoint with a complete residual, with `dx` the accepted
displacement of the step and all forces in physical units
(docs/rb-04-work-convention.md):

- `support_work_increment` = Σ_c (AᵀA r)·dx over simple selector constraints:
  the support force on the system dotted with `dx` (`W_D,right`).
- `body_load_work_increment` = −(g_body/s)·dx; identically zero without a
  body/traction form. `external_work_increment` is their sum.
- `frictional_dissipation_increment` = g_f,pre-update·dx, the solved-lag
  friction gradient (before the final lag update) dotted with `dx`
  (`C_f,right`): a resistance-work estimate, not integrated continuum
  dissipation; identically zero without a friction form; unavailable without a
  pre-update observation.
- `retuning_energy_change` = B(x_(n-1); θ_n) − B(x_(n-1); θ_(n-1)) (`P`): the
  endpoint snapshot's start energy minus the previous published endpoint's
  `barrier_energy`. For the first accepted step of a process θ_(n-1) is the
  state at solve start (`barrier_energy_at_solve_start`);
  `previous_endpoint_barrier_energy` records the value used. Under the RB-20
  force continuation every persisting contact keeps its coefficient, so on the
  public fixtures `P` is exactly the global trim change applied to the
  previous endpoint energy.
- `*_cumulative`: sums over the accepted endpoints of this process
  (`cumulative_scope`); unavailable once any increment was unavailable. A
  restarted process starts at zero.

Unsupported active forms make the residual, and therefore the support and
external work, unavailable. Failed attempts carry all increments as
unavailable. `work_convention` restates the convention in the record. These
are the declared bookkeeping terms of the discrete right-endpoint budget, not
path integrals and not a physical balance.

### VTU kinematics alignment

The nonlinear time loop exports each accepted endpoint before advancing the
integrator history, so the `velocity`/`acceleration` VTU fields of version 1
runs were the previous step's values (inherited from upstream PolyFEM; the FSI
embedding exports after advancing and was correct). The exporter now writes
the kinematics of the saved solution: a solution equal to the history head
reads the stored values, any other solution is differenced with the
integrator's own rule at the current history (`saved_solution_kinematics`).
The diagnostic `velocity`/`kinetic_energy` fields were already defined this
way; the VTU fields now agree with them and with the displacement history.
