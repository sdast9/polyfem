# Fork correctness remediation: phase-aware session plan

Revised 2026-09-06 after reviewing the initial PF-01 implementation against
`ballburst_thin_membrane` and `teseo_problem`. This supersedes the implementation
scope in the 2026-09-05 memory plan; it does not certify all contact heuristics.

## Invariants for every session

1. **AL prepares a geometrically safe snap to prescribed Dirichlet values.**
   Its intermediate iterates need not minimize the augmented energy. A usable
   interrupted inner solve is continuation, not a failed AL attempt. Assess snap
   feasibility using the existing finite-energy, inversion and collision checks.
2. **The subsequent reduced solve uses the configured minimization criteria.**
   Restart/custom-stop/iteration-budget interruption is not convergence. However,
   PF-01 must not substitute a new raw-gradient-only criterion for configured
   gradient, step, energy or directional-derivative tolerances. Report the actual
   criterion; a tolerance-based stop is not a certificate of exact equilibrium.
   The shared wrapper also handles reduced-stage restarts: the class name does
   not imply that every callback interruption belongs to AL preparation.
3. PolySolve currently labels a small *negative* directional derivative within
   its configured tolerance `NotDescentDirection`. This normal tolerance exit
   must be distinguished from a genuinely non-descending direction or an error.
4. Keep solver phases separate: AL continuation, reduced minimization, stiffness
   retuning, and friction lagging have different termination contracts. Do not
   silently turn bounded lagging into mandatory fixed-point convergence, change
   tolerances, disable a contact feature, or add timestep retries in another fix.
5. Separate **a reproduced implementation defect**, **a modeling concern**, and
   **an optional redesign**. A synthetic counterexample establishes its specific
   failure mode; it does not establish that every production scene is inaccurate.
6. Preserve user inputs/outputs. Validate with isolated copies of approved
   scenes, record any input-schema translation, and keep timestep/load schedule
   unchanged for full runs. A partial run is not a full pass. Do not regenerate
   golden data or relax tolerances to manufacture a pass.
   **User constraint (2026-09-06): do not run `teseo_problem` unless explicitly
   asked.** References to its historical results do not authorize a new run.
7. Read current AGENTS.md, branch state and dependency pins before each session.
   Save incoming uncommitted work before narrowing edits. Rebuild, test, then
   commit/push according to project instructions. Record commits and exact
   validation limits so another session can resume independently.

## PF-01 — Correct phase handling and restart outcomes

Scope: `ALSolver`, its diagnostics and focused tests. Interrupted AL passes remain
usable and do not increment the hard-failure counter merely for not converging.
The final reduced solve distinguishes configured convergence from interruption.
Keep existing contact-floor, gradient normalization, friction-lagging and solver
parameter policies unchanged. Do not require AL stationarity or introduce a new
minimum BC residual when geometric snap checks already succeed.

Required tests:

- Three or more incomplete AL passes can lead to a safe snap; the following
  reduced solve meets its configured criterion. Test the actual `solve_al` loop,
  not just the shared wrapper. Use a clearly labeled synthetic feasibility gate
  for the small unit test and real geometry in scene validation.
- Final restart exhaustion/custom interruption cannot report convergence.
- A final solve that meets enabled non-gradient or negative-slope tolerances
  remains accepted. Hard line-search failures remain errors.
- Both named user scenes, plus affected solver smokes. Preserve diagnostic
  distinction between meeting configured tolerances and physical validation.

Do not expand PF-01 into automatic timestep rollback/retry, friction policy,
contact-floor removal, or stricter global convergence criteria. Those changes
need separate evidence and tests.

## PF-02 — Investigate and isolate the contact-floor model

Start with an **experimental comparison**, not a blanket default change. The
measured energy discontinuity and fixed-DOF projection defect remain concerns,
but feasibility alone and smooth-barrier equilibrium are different contracts.
Document exactly which phase invokes the floor and what problem it is intended
to solve. Compare current behavior, a floor-disabled barrier baseline, and any
proposed correction on fixed obstacles, prescribed motion, and Ballburst.
Teseo is excluded from new testing unless the user explicitly requests it.

A barrier-only candidate must retain CCD and a coherent coefficient definition
through each line search, including newly created contacts. Retuning, if moved
between solves, must reset the necessary solver/lagging state explicitly. Merely
forcing coefficients positive can also change the model; justify the scale.

Acceptance: quantify energy/force behavior near the threshold, collision and
inversion checks, final residuals under the appropriate model, and runtime.
If the baseline stalls, record it; do not count disabling a working feature as
completion. Choose a production default only after presenting the tradeoff and
agreement on the intended model. PF-09 is the separate hard-contact alternative.

## PF-03 — Obstacle-independent BC metric normalization

Retain normalized AL. Define the reference mean from FEM lumped DOFs, excluding
obstacle placeholders, and fill obstacle weights consistently afterward. Preserve
row-sum/HRZ/no-mass fallbacks and relative weights. Hold the metric fixed during an
AL sequence unless multipliers are transformed consistently.

Test FEM masses [1,3] with extra obstacle DOFs: the FEM metric remains [0.5,1.5].
Test upstream/fork equivalence under converted penalties and multipliers; include
inhomogeneous prescribed targets. Do not reinterpret the penalty as a physical
inertial term, require AL energy convergence, or claim mesh-independent behavior.
This is independent of PF-02.

## PF-04 — Material cache ownership

Share an immutable input snapshot within a simulation; a fresh simulation/input
snapshot reloads changed files. Cover ExpressionValue and fiber caches together.
Preserve copy-on-write, parallel initialization and concurrent simulations.
Do not clear a shared global cache while another simulation uses it, or rely only
on file timestamps without a documented lifetime policy.

Test file value 1 -> 9: old snapshot remains stable, new simulation sees 9;
elements still share storage, and mutation detaches. No solver physics changes.
Independent of the solver/contact sessions.

## PF-05 — Nonunit AL form-scale derivatives

Correct the inconsistent extra quadratic-gradient scale to match the existing
objective and Hessian. Test scales 0.5, 1 and 2 with nonzero multipliers and
nonzero prescribed targets. The original probe used homogeneous targets; ensure
constructor slicing does not invalidate the new test setup.

This bug was inherited from upstream, and current NLProblem normalization returns
1. Do not imply it caused the measured production failures. Do not enable new
normalization or change multiplier-update policy as part of this derivative fix.

## PF-06 — Direction-filter derivative contract

Determine the actual intended objective/constraint model before changing the
filter. For a smooth-objective line search the directional derivative is g^T p;
a nonlinear one-sided filter generally does not justify replacing it with
-p^T P(-g). Verify the claimed derivative against finite differences.

Keep this mathematical question separate from PF-01's preservation of existing
configured stopping tolerances. If using an inequality-constrained merit
function, define it and its derivative explicitly. Test coupled/free/fixed DOFs,
descent/ascent, filter cleanup, and unfiltered behavior. Reproduce a relevant
failure before changing the API shared with current PolySolve upstream.
Coordinate with PF-02/PF-09; do not silently remove the filter's intended role.

## PF-07 — AL initialization and continuation control

Keep Hessian scaling a documented initializer, not a proof of curvature dominance.
Measure the need for a characteristic stiffness scale or a generalized-curvature
estimate relative to the BC metric; account for relevant inertial terms if used.
Separate that optional numerical improvement from simple units/documentation fixes.

Any continuation budget must use geometric progress and explicit failure reasons;
nonstationarity of an intermediate AL iterate is not a failure. Do not add a
three-pass interruption limit or assume one penalty works across all constraints.
An optional efficiency improvement is checking snap feasibility at safe solver
boundaries before spending more AL restarts; validate restoration of modes and
caches, and never change coordinate modes inside an active line search.
Compare equivalent unit/density scalings with converted parameters and final
reduced results; iteration counts need not match. Build on PF-03/PF-05.

## PF-08 — Physical and numerical validation

After the selected implementation changes, validate fixed/moving obstacles,
coupled contacts, prescribed motion, friction and deliberately exhausted recovery.
Measure BC accuracy, free-DOF equilibrium or appropriate constrained KKT residual,
reaction balance, gap/inversions and energy/work balance including dissipation.
Distinguish configured numerical tolerance satisfaction from physical accuracy.

Sweep mesh, timestep/load increments, units and material scales with explicit
comparison criteria. Preserve the user's finite friction-lagging policy unless a
separate study changes it. Run focused tests before the broader suite; report
unrelated baseline failures separately. Narrow documentation to measured claims.

## PF-09 — Optional hard-contact formulation

Only proceed after choosing the model with the user, informed by PF-02/PF-08.
Specify gap constraints, objective, multipliers/reactions, activation/release,
friction and convergence. Map contact Jacobians through interpolation and into
free coordinates, including prescribed-motion offsets. Solve coupled constrained
steps with nonlinear feasibility checks and appropriate KKT criteria.

Test fixed-edge projection, multiple contacts, separation/sliding, moving
obstacles and incompatible prescribed motion. Four projection sweeps and abrupt
barrier deletion alone are not an equivalent constrained solver. Do not require
this redesign merely to keep the mathematically valid AL normalization.

## Completion records

PF-01: phase-handling correction implemented; see the [validation record](pf-01-validation.md).
Ballburst completed 200 steps. The first Teseo run saved step 11/20 and then
exhausted 20 restarts in the final reduced solve; the isolated awake rerun
completed all 20 steps under the same numerical settings. No sleep events were
recorded during the rerun, but this does not establish the cause of the first
failure. The narrow regression suite passes. PF-01's phase-handling correction
is validated; reproducible full-scene robustness and physical accuracy are not
established. Investigate the path-dependent final-stage stall separately; do not
accept a callback interruption as convergence.
PF-02: the [first characterization stage](pf-02-contact-floor.md) compares the
current floor with the floor-disabled baseline, reproduces the energy/projection
defects and a refresh inconsistency, and records approved scene evidence. The
production model/default decision and broader physical validation remain open.
PF-03 through PF-09: not implemented. PF-09 retains its model-decision gate;
PF-03/PF-04/PF-05 can be addressed independently.
See the local `outputs/pf-01-correction/` folder for preserved incoming changes,
copied scene inputs and execution logs. Update status from actual code/history
before resuming; a saved plan is not proof a session completed.
