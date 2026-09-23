# Slope tolerance only ends a Newton solve — repair record

Date: 2026-09-23. **Status: repaired and validated within the stated scope**
(the user's decision of 2026-09-23 on recommendation 1 of the
[quasi-Newton investigation](qn-contact-investigation-20260922.md)).

## The defect

PolySolve's `advanced/derivative_along_delta_x_tol` stops a solve when the
slope along the proposed direction, g·Δx, is above a small negative threshold
(status `NotDescentDirection`). For Newton that slope is minus the squared
Newton decrement — twice the energy decrease the step predicts — and the test
is a sound convergence criterion; the user's real scenes converge on it.
PolyFEM's `ALSolver` (PF-01, `4b6e970c3`) accepted that status as convergence
for **every** method, whatever `allow_non_grad_convergence` said. For L-BFGS the
slope is γ‖g‖² with γ ≈ 1e-8 on contact scenes, met almost at once: L-BFGS
"converged" R1 step 1 after one iteration with 100 % solution error and
finished all six steps of R3 with exit 0 and 4–7 % error per step. The same
holds for ADAM, gradient descent, dense BFGS, and for Newton's own
gradient-descent fallback. `x_delta_tol` and `rel_x_delta_tol` have the same
flaw (a direction's length is a Newton step only for Newton), gated by
`allow_non_grad_convergence`.

## The repair

* **PolySolve** (`iteration-callback`, `448f1b8e`, pinned here): `DescentStrategy::direction_solves_with_hessian()`
  (false by default, true for every Newton variant) and a solver hook
  `direction_based_stops_allowed()`. The iteration loop applies the slope
  tolerance, `x_delta_tol` and `rel_x_delta_tol` only while a strategy that
  solves with the Hessian is active; under any other strategy those criteria
  wait (a fallback returns to Newton after `iterations_per_strategy`
  iterations, where they apply again). `BoxConstraintSolver` keeps its
  criteria — a projected step's length is its methods' stationarity measure,
  and PolyFEM's shape/parameter optimization uses them. `solver_info` records
  `active_strategy_solves_with_hessian`. Regression
  `slope-tolerance-stops-only-hessian-directions` (`[solver][bfgs][stopping]`):
  L-BFGS, alone and with its fallback, runs through a slope tolerance and
  `x_delta_tol` that would stop it on an anisotropic quadratic to the gradient
  criterion; Newton still stops on the decrement before its first step.
* **PolyFEM**: `ALSolver`'s `slope_tolerance` branch also requires
  `nl_solver->direction_solves_with_hessian()`, so the contract is explicit on
  this side too.
* **Houdini asset**: see the [HDA README](https://github.com/sdast9/houdini-plugins/blob/main/docs/hdas.md)
  — a warning under the Solver menu for a non-Newton method with contact on, a
  warning when writing or running such a scene (nothing is refused), and the
  *Descent Direction Tolerance* tooltip corrected (it said the setting "only
  catches genuine ascent").

Behaviour change: a non-Newton solve no longer ends on those criteria. On
contact scenes it now runs to the gradient tolerance or to its limits and
fails by name (exit 1) instead of reporting a wrong solution. Newton is
unchanged except when a fallback strategy would have met them — it now
continues until Newton is active again.

## Validation

Evidence: parent workspace `outputs/qn-fix/20260923T134701Z/`.

| check | result |
| --- | --- |
| Five public smokes, pre-change binary vs repaired binary, single thread | 25/25 VTU frames byte-identical |
| PolySolve `[bfgs],[solver],[wolfe],[stress]` | 46 cases / 2,605 assertions pass (stage 5: 45 / 2,591; +1 case / +14 assertions) |
| PolyFEM suites (stage 5's selection: `[al_solver]`, `[direction_filter]`, `[objective_generation]`, `[rollback]`, … `[input_validation]`) | 110 cases / 10,535 assertions pass |
| R1 plain L-BFGS, 1 step | before: exit 0 after 1 iteration, 100 % error; after: **exit 1**, named failure after 20 stall restarts |
| R3 plain L-BFGS, 1 step | before: exit 0, 4–7 % error per step; after: no step accepted in 900 s (30 stall restarts; stopped by the run cap) — no false convergence |
| R1 Newton, 6 steps | exit 0; errors vs the tight reference 4.7e-7…5.0e-4 (before: 4.5e-7…5.0e-4) |
| R3 Newton, 6 steps | exit 0; 20 iterations, errors 1.2e-4…6.4e-4 — the same values as before |
| Houdini asset tests (fresh clone, assets rebuilt there, all 13 files, against the repaired build) | 13/13 pass, including the new warning checks; published as houdini-plugins `7b8d2e0` |

## Limits

* Before-evidence for the PolySolve regression is the scene behaviour above;
  the unit test was not run against the unrepaired library.
* PolySolve still logs a Newton solve that ends on the slope tolerance at
  error level ("Search direction not a descent direction") although PolyFEM
  counts it converged; unchanged, pre-existing.
* The asset warns; it does not stop a user from running a non-Newton method on
  a contact scene (the user's choice of warnings over withdrawal).
