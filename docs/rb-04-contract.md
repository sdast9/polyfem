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
