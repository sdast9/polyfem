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

This is the **endpoint instrumentation stage**, not full completion of RB-04.
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
