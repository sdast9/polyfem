# Implementation handoff for the completed-RB review — 2026-09-20

**Status: plans; RBR-01, RBR-02 and RBR-05 implemented 2026-09-20,
RBR-03 and RBR-04 on 2026-09-21 (see their sections).** Context
and verdicts are in the
[review](rb-completed-review-20260920.md). The user explicitly requested
detailed plans for other models to code later. This document is not a
completion record or authorization to execute every task in one session.
RBR-05 was not part of the review: it is the zero-free-DOF crash that
RBR-01's reproduction exposed, added here because it was found and
repaired in the same handoff.

## Common session contract

Select one of RBR-01–RBR-04. Read the workspace instructions, the selected
RB record, the [PF invariants](correctness-remediation-plan.md), and the
[robustness plan](robustness-plan.md). Recheck live branches, status,
dependency pins/overrides and binary provenance. The review's starting
PolyFEM implementation was `6a553447b`, IPC `c24d803e`, PolySolve `bce32a39`;
later RB-23/RB-24 work is independent and must be preserved.

Reproduce on the current source before editing. Keep evidence in a new
parent-workspace `outputs/rbr-XX/<timestamp>/`; retain the failing control,
commands, source/binary hashes and every incomplete run. Do not overwrite
the review evidence or reuse its binary as though it were current. Use an
isolated worktree/build if another task uses the shared checkout or build.
No Teseo/private scene, constraint-floor restoration, CCD removal, retry,
new contact law, tolerance relaxation or golden regeneration is in scope.

After a selected repair: rebuild the affected targets, run the specified
tests and affected public smokes; update its validation record with the
actual result and limitations. Publish only the named files. IPC changes
must be committed/pushed to the IPC fork before updating PolyFEM's recipe
pin; verify the final build uses that revision, not a stale override.
Do not use `git add -A` in the shared checkout.

## RBR-01 — Explicit output time state instead of position equality

**Status: implemented 2026-09-20** — see the
[RB-04 record](rb-04-validation.md#rbr-01--explicit-output-time-state-2026-09-20)
(`varform::OutputTimePhase`, owner-maintained; `[output_kinematics]` incl. the
hold-segment VTU scene; solutions/forces identical, only the held-step
kinematics changed). The text below is the plan as handed off.

**Parent item:** RB-04; affects all transient stiffness modes and potentially
contact-free output. **Priority:** P2. **Evidence:** reproduced in the real
linked `saved_solution_kinematics` function.

### Reproduction and invariant

Use `ImplicitEuler`, one DOF, `dt=.25`. Initialize `x=0, v=0, a=0`; advance
once to `x=.1`. The stored history now has `v=.4, a=1.6`. The next accepted
solution is also `.1`, and is exported before advancing history. The
current implementation returns the stored `.4, 1.6`; correct current-step
kinematics are `v=(.1-.1)/.25=0`, `a=(0-.4)/.25=-1.6`.

The existing test covers a changed endpoint and an export after advancement,
but assumes equal positions identify the latter. They cannot. Correctness
requires knowing the output's time/history phase independently of position.

Source entry points:

- `src/polyfem/varforms/ElasticVarForm.{hpp,cpp}`:
  `saved_solution_kinematics`, `elastic_output_fields`.
- `NonlinearElasticVarForm.cpp`: `output_fields`, initial save,
  `solve_transient_step`, `advance_transient_step`, embedding update/save.
- `LinearElasticVarForm.cpp` and `IncompressibleElasticVarForm.cpp`:
  their calls to the shared exporter and actual history-advance ordering.
- `tests/test_time_integrators.cpp` `[output_kinematics]`;
  `tests/test_linear_elastic_time.cpp`; `tools/rb04/run_endpoints.py`.

### Proposed implementation

1. Replace the equality heuristic with an explicit output-kinematics
   contract. A small required enum such as `HistoryHead` versus
   `CurrentStepBeforeAdvance`, or explicit precomputed velocity/acceleration,
   is preferable to an inferred boolean. Choose one representation and
   document it in the shared API.
2. For a current step before advancement, always call the integrator's
   `compute_velocity(solution)` and `compute_acceleration(v)`, even when
   the positions are equal. For an initial/already-advanced history head,
   use `v_prev/a_prev`. Retain defined behavior for uninitialized history;
   do not access empty deques.
3. Trace **every** shared-export call site. The nonlinear standard loop
   saves before advancing; the linear/embedding paths may save after.
   Initial output must retain the supplied initial velocity/acceleration.
   Pass known state from the owner; do not replace one equality guess with
   another. Public `output_fields()` calls after a solve must have a defined
   phase too.
4. If state is stored in a varform rather than passed explicitly, update it
   at initialization, accepted solve and history advancement, including
   embedding/restart paths. Include it in rollback when it is attempt state.
   A failed solve must not make subsequent output use the wrong time phase.
5. Keep force-export scaling (`solved_step_scale_`) and solve/update ordering
   unchanged. Moving time advancement merely to fix output can change
   coefficient refresh, friction, BDF startup and restart behavior.

### Acceptance

- New regression fails on the reviewed helper and passes on the repair
  with the exact held-position values above.
- Initial output with nonzero supplied velocity/acceleration is unchanged.
- Changed endpoint, identical endpoint, return to an earlier position, and
  output after history advancement all have explicit expected values.
- Implicit Euler, Newmark and BDF2/BDF3 startup/mature histories agree with
  independently evaluated integrator rules. Do not merely compare the new
  helper with itself.
- A public prescribed motion with a hold segment verifies **saved VTU
  velocity and acceleration**, not only the helper. Include a before/after
  advancement control and check all relevant timesteps.
- Run `[output_kinematics],[time_integrator],[linear_elastic]`, the relevant
  rollback selection if mutable phase state is added, and RB-04 endpoint
  checks. Show identical solution coordinates/forces between baseline and
  candidate, apart from the corrected kinematics fields.

Deliver code, regression, RB-04 record correction and concise evidence.
This repair is a prerequisite for upstreaming RB-04's output fix (UP-02).

## RBR-02 — Checked arithmetic throughout broad-phase resource accounting

**Status: implemented 2026-09-20** — see the
[RB-05 record](rb-05-validation.md#rbr-02--checked-arithmetic-in-the-broad-phase-resource-accounting-2026-09-20)
(toolkit `a28de2db`: `CheckedCount` behind every budget decision, "at
least" reporting, the named `BroadPhaseUnrepresentable` for grids the key
cannot index and `invalid_argument` for non-finite input before any
conversion, keys in the key type; PolyFEM pin bump, exit 3 classification,
`[resource_containment]` sweep refusals; ordinary counts, candidates and
solutions unchanged). The text below is the plan as handed off.

**Parent item:** RB-05. **Priority:** P2. **Evidence:** real toolkit counting
methods wrap `2^64` items to zero and admit them against a budget of 100.

### Reproduction and invariant

The preserved probe derives a tiny test class from `ipc::HashGrid` to expose
the protected count/check methods. Set a 3D grid's cell size to 1 and
dimensions to `(4194304,2097152,2097152)`. Add one AABB with minimum zero
and maximum `(dimensions-1)`. All individual indices fit `int`. With
`max_cell_items=100`, the current code reports zero and accepts. A
`(1024,1024,1024)` control reports `1073741824` and throws.

**Never run insertion for the huge box.** The invariant is that count or
index arithmetic cannot wrap into an apparently cheap build. When exact
counts are unrepresentable, report that explicitly; never truncate pairs.

Source entry points in the IPC companion:

- `src/ipc/broad_phase/hash_grid.cpp`: `box_cell_range`, `count_cell_items`,
  `check_cell_item_budget`, both emission-count loops, cumulative statistics.
- `src/ipc/broad_phase/brute_force.cpp`: rectangular/triangular pair counts.
- `src/ipc/broad_phase/broad_phase.hpp`: budget, statistics and exception
  representation; `candidates/candidates.cpp`: exception cleanup.
- `tests/src/tests/broad_phase/test_budget.cpp`.
- PolyFEM: `tests/test_contact_cache.cpp` `[resource_containment]`,
  `tools/rb05/run_probe.py`, `tools/rb05/run_scene_limits.py`, recipe pin.

### Proposed implementation

1. Introduce small checked count operations, or budget-capped operations
   with an explicit overflow/exceeded flag. Cover products, sums, TBB
   reduction joins, the vertex/edge/face total and cumulative statistics.
   Checking only the final product is too late.
2. Handle `n*(n-1)/2` without an avoidable intermediate overflow (divide
   the even factor first, then checked multiply). Keep zero/one cases valid.
   Apply the same discipline to rectangular products and diagnostic byte
   estimates such as `items*sizeof(HashItem)`.
3. Early-stop counting after the limit is provably exceeded if practical,
   without racing on shared counters. Preserve deterministic pass/fail
   decisions under different TBB partitions.
4. Saturating to `SIZE_MAX` alone is insufficient if the configured limit
   is also `SIZE_MAX`: retain a separate overflow flag. If the exact count
   is unknown, the exception must say “at least” or “unrepresentable”, not
   claim that a saturated value is exact. Preserve exact counts for ordinary
   representable builds.
5. Audit conversions into grid coordinates before floating-point-to-`int`
   casts. Finite but out-of-range coordinates, invalid cell sizes and
   nonfinite extents must reach a named failure before conversion/allocation.
   This is an adjacent audit requirement, not a second defect reproduced
   by the `2^64` probe; add a separate failing test before changing it.
6. Disabled limits remain disabled as a policy; they do not license
   undefined arithmetic. Specify a named failure for unrepresentable grid
   geometry/counts and keep PolyFEM's resource-error classification coherent.
   Preserve automatic limit values and supported broad-phase choices.
7. Verify `Candidates::build` and contact sweep handling leave no stale
   candidates/use-cache flag on every new failure path. Never proceed with
   a partially counted or partially built collision set.

### Acceptance

- The huge-box count-only regression rejects without large allocation;
  the ordinary control still reports the correct count.
- Cover product overflow, leaf/reducer/category sum overflow, exact limit,
  one over limit, zero/one count, largest representable values, disabled
  budget and `SIZE_MAX` budget. Use synthetic count inputs for extremes.
- Checked triangular counts must accept values whose final result fits
  even when naive multiplication would overflow.
- Thread-count changes cannot change acceptance. An exception leaves
  candidates empty and swept caches disabled; a later small build works.
- Current ordinary hash-grid/brute-force candidate identities and counts
  are unchanged. Tiny limits still cause named failures/exit 3 in the real
  executable and a generous limit preserves public-scene endpoints.
- Run IPC budget tests and PolyFEM `[resource_containment],[contact_cache]`,
  RB-05 quick/budget probes and public limit-scene checks. Rebuild with the
  final published IPC revision before recording acceptance.

Publish IPC first, then PolyFEM pin/record as needed. This is a prerequisite
for UP-05; it is not the separate RB-24 CCD/thread-memory work.

## RBR-03 — Bounded iterate storage for the AL budget

**Status: implemented 2026-09-21** — see the
[RB-07 record](rb-07-validation.md#rbr-03--bounded-carried-state-storage-for-the-budget-2026-09-21)
(`ALSolver::carried_`, a deque of at most `stagnation_window + 1` full-space
states — one under a pass cap alone, none without a budget — evicted before
each pass's state enters and released with the stage; the retained count in
every pass record and manifest AL subsolve entry; the `[al_budget]`
full-history oracle; the three public budgets stop at the same pass with the
same reason, gate history and byte-identical compatible endpoint; the scalar
pass history is still complete). The text below is the plan as handed off.

**Parent item:** RB-07. **Priority:** P2 for large opt-in runs.
**Evidence:** source-confirmed storage growth; no deliberate OOM was run.

The implementation saves every full-space solution when *any* budget is
enabled. With one million DOFs and 1,000 passes, vector data alone is about
8 GB (decimal). A cap-only budget needs only the preceding state to report
`moved`; a window W needs the state W passes back. The compact scalar JSON
pass history is useful and can remain complete.

Source: `src/polyfem/solver/ALSolver.cpp`, `solve_al`'s `carried` vector and
window-drift calculation; `tests/test_al_solver.cpp` `[al_budget]`;
`tests/test_step_rollback.cpp`; `tools/rb07/run_al_stagnation.py`.

### Proposed implementation and acceptance

1. First add a regression/instrumented small control that establishes
   retained vector count grows with passes on the baseline. Prefer an
   explicit retained-history-size check over a flaky RSS threshold.
2. Replace `carried` with a bounded deque/ring of at most `W+1` vectors
   when W>0 and only the previous vector when W=0. Pass zero is a real
   reference. Compute current-vs-previous movement and current-vs-pass-W
   drift before evicting needed entries. Avoid signed/unsigned index errors.
3. Preserve `al_history_` scalar rows, pass numbering, the weight-ceiling
   requirement, rolled-back-pass semantics, all progress thresholds, and
   `check_budget` timing. This is a memory repair, not a stagnation redesign.
4. Check W=0, W=1, W>passes, W=passes, many passes, cap-only, window-only,
   both controls and budget-off. Compare movement/drift values with an
   independent full-history oracle on small sequences, including rollback.
5. `[al_budget],[al_continuation],[rollback]` must retain the same stop pass,
   named reason, gate history and restoration fingerprint. The public
   `compatible-multipass` must retain the same successful continuation and
   byte-identical endpoint; `incompatible-collision` the same stagnation
   exit; `incompatible-crush` the same pass-cap exit for equal settings.
6. Demonstrate vector storage is bounded by the configured window,
   independent of pass count, without allocating a huge production problem.
   Full scalar history still grows O(passes); do not claim total constant
   memory or alter its output contract silently.

Keep defaults off. This change cannot make a window-only budget terminate
every incompatible moving trajectory, and must not claim to do so. Publish
the source/test/record only after bounded validation. UP-07 depends on it.

## RBR-04 — Enforce the parent-law supported formulation

**Status: implemented 2026-09-21 (guard route)** — see the
[RB-21 record](rb-21-parent-keyed-kappa.md#rbr-04--the-improved-max-operator-is-a-checked-restriction-2026-09-21).
Reproduced first on the real form (`tools/rbr04/seam_probe.cpp`: a finite
`|κ₁−κ₂|/2·b(d)` jump, +62.24 / −62.24 at the two seams of the corner
fixture, constant from offset 1e-3 to 1e-9; equal coefficients, the
non-convergent default, area weighting alone and fixed/adaptive improved
max continuous) and through the public JSON path (the fragment ran to
completion with the manifest reporting `improved_max_operator: true` under
`semi_implicit`). Then: the constructor and `State::init` refuse
semi-implicit + `use_improved_max_operator` by name, stating the supported
alternatives; the input spec states the restriction; `[rbr04]` regressions;
the RB-21 record's "never used" claim corrected. No heterogeneous
improved-max energy law was invented. The text below is the plan as handed
off.

**Parent item:** RB-21, also RB-15's continuity contract.
**Evidence level:** public-option dispatch and missing guard confirmed in
source; new end-to-end numerical seam probe not completed by this review.

Read `docs/rb-21-parent-keyed-kappa.md`, especially its seam analysis. The
positive-parent average represents a sum of parent energies when collision
weight is the sum of positive contributions. Improved-max duplicate removal
introduces negative contributions. With heterogeneous parents, the recorded
corner residual jump is `|k1-k2|/2 * b(d)`; assigning one positive weighted
mean cannot make both seams continuous under that construction.

### Reproduce first

1. Start from the existing RB-21 neighboring-edge/parent-contribution
   fixtures and the RB-02/RB-15 transition geometry. Use unequal frozen
   parent coefficients, e.g. 70 and 40, on two edges sharing a vertex.
2. Compare improved-max off/on with the physical barrier off and parent
   identity selected. Through the public JSON path, use:

   ```json
   {
     "contact": {
       "use_convergent_formulation": true,
       "use_area_weighting": false,
       "use_improved_max_operator": true,
       "use_physical_barrier": false
     },
     "solver": {"contact": {"barrier_stiffness": "semi_implicit"}}
   }
   ```

   These are fragments to merge into a valid isolated fixture, not a full
   scene. Also test area weighting true if it reaches the same issue.
3. Confirm actual constructed flags, parents, signed weights and assigned
   coefficients. Approach both seams with successively smaller offsets.
   Report limiting energy/gradient behavior and distinguish a finite jump
   from variation proportional to offset. Equal coefficients and fixed/global
   adaptive modes are required controls. A disabled setting is not evidence
   of success in the requested formulation.

### Bounded recommended resolution

If the combination reaches the documented discontinuous law, reject
semi-implicit plus improved-max with a precise named configuration error
at construction and, where appropriate, early JSON validation. State the
supported alternative explicitly. Keep the non-convergent default, fixed
mode and classic global-adaptive improved-max behavior unchanged. Do not
silently disable an option or change a coefficient averaging rule.

Entry points: `BarrierContactForm` constructor and `coefficient_keys`,
`SolveData::init`, nonlinear varform option forwarding, input-spec docs,
`tests/test_kappa_continuity.cpp`, `tests/test_semi_implicit_coefficients.cpp`,
`tests/test_input_validation.cpp`, companion
`tests/src/tests/collisions/test_parent_contributions.cpp` (locate current
test files/tags before editing).

If supporting heterogeneous improved-max coefficients is desired instead,
stop at a **separate model-design decision**: the energy and all derivatives
need a consistent signed parent construction. Taking absolute weights or
averaging negatives away is not a justified repair. No such new law is
selected by this plan.

Acceptance for the guard route: public JSON and direct constructor both
refuse the unsupported combination before a solve; default parent-law
continuity, explicit stencil controls, fixed/global-adaptive improved-max
controls and the existing RB-02 probe remain intact. Correct the inaccurate
“never used” claim to a checked restriction. If the current code already
prevents the combination, record the actual boundary and close the finding
without an unnecessary behavior change. UP-10 must state this restriction.

## RBR-05 — A fully prescribed body (zero free DOFs) is a valid trivial solve

**Status: implemented 2026-09-20** — see the
[RB-11 record](rb-11-validation.md#rbr-05--a-fully-prescribed-body-is-a-valid-trivial-solve-2026-09-20)
(`NLProblem` empty reduced space, `ALSolver::solve_reduced` guard,
`[fully_prescribed]`, matrix case `g5-fully-prescribed-body`). The text
below records the defect, the decision and its reasons.

**Parent items:** RB-11 (a valid input crashed instead of running) and
RB-04 (the other ordinary hold of RBR-01's output-kinematics scene).
**Priority:** P2. **Evidence:** reproduced at the process level (exit 139)
on `92e0d8c20`, on the RBR-01 binary and on the current source, located
with lldb (`outputs/rbr-05/20260921T002043Z/baseline/repro/lldb.log`).

### Defect

A nonlinear transient scene whose every node is Dirichlet-prescribed (a
2×1×1 P1 Kuhn-tet beam, twelve vertices all on the boundary,
`surface_selection: 1` for the whole boundary and one time-dependent
`dirichlet_boundary`) died with SIGSEGV right after `step_0.vtu`. The
reduced problem has zero unknowns. Upstream's `fix case of no dofs`
(`fb762251e`) made `NLProblem::setup_constraints` return early when the
projection has no columns — before `num_penalty_constraints_`, the penalty
problem of the full-size AL passes and the affine offset `Q1R1iTb_` (the
prescribed values) were set. `reduced_to_full(empty)` then evaluated
`Q1R1iTb_ + Q2_ * reduced` with a size-0 left operand: under `NDEBUG` the
result is a size-0 vector, and the elastic energy of the AL solver's first
snap gate read it (`NeoHookeanElasticity::compute_energy_aux`, address 0).
Two further latent faults sat behind it: `full_to_reduced` would have
solved with a never-factorized `Q2ᵀQ2`, and PolySolve's `minimize` on an
empty vector depends on the configuration (`first_grad_norm_tol` 0 sends
it into a Newton step on a 0×0 system, `norm_type: Linf` into `maxCoeff`
of an empty gradient).

### Decision: valid trivial solve, not a named refusal

The RB-11 input-validation style refuses inputs the solver cannot run.
This input is not one of them: the transient problem with every DOF
prescribed is well posed and its solution is the prescribed motion; the
kinematics (integrator rules), the reactions (the full residual: inertia,
elastic, contact) and every RB-04 record field are defined. Whether a scene
has free DOFs is a property of its discretization, not of the scene: the
same whole-boundary Dirichlet at P2, or on a mesh with an interior node (the
2×2×2 cube), has free DOFs and runs; refusing the coarse P1 version would
be an arbitrary discretization-dependent refusal. Upstream's early return
shows the same intent (support, not refuse). So the empty reduced problem
is solved by definition:

- `NLProblem::setup_constraints` (both the projection and the QR branch)
  sets the constraint count, builds the penalty problem and the affine
  offset for an empty reduced space and skips only the 0×0 factorization;
  `full_to_reduced` returns an empty vector (nothing to solve for) and
  `reduced_to_full(empty)` is the affine offset, i.e. the prescribed values
  at the step's time (refreshed by `update_quantities`).
- `ALSolver::solve_reduced`: the AL stage is unchanged (its snap gate now
  evaluates the prescribed configuration correctly; an infeasible
  prescribed motion — a body driven through an obstacle — takes the same
  full-size AL passes and ends by the same named failures as a partially
  prescribed body, RB-07's characterized behaviour). After the feasibility
  check, a reduced problem of size zero is stated solved without calling
  the nonlinear solver, with the same observable sequence as a solve that
  converges at its start (`solution_changed`, the iteration-0 `post_step`
  for the forms and the RB-04 attempt stream, `finish`), a subsolve record
  `outcome: converged`, `iterations: 0`, termination reason
  "No free degrees of freedom: every DOF is prescribed, the reduced problem
  is empty and the solution is the prescribed values", and an info log line.
  The lagging loop needs no change (an empty gradient converges the lag).

No tolerance, default, law, CCD, retry or dependency changed; PolySolve is
untouched (the guard makes the outcome independent of its configuration).

### Acceptance (met)

`[fully_prescribed]`: the NLProblem transforms/energy/derivatives of an
empty reduced space and the AL stage + reduced stage on a two-DOF fixture
(also with the tolerances/norm that would trip PolySolve); the real
transient loop on the reproduction (default tolerances, the tripping
tolerances, contact on the lone body, friction), every endpoint the
prescribed values to 1e-15, kinematics v .2/.4/0/0 a 0/.8/−1.6/0 on
every node before and after the advance and in the saved VTUs, one
zero-iteration reduced subsolve per step and no AL pass; the body driven
within `dhat` of the public slab (barrier active, motion unchanged); a
static fully prescribed solve. Control: the same tests on the unrepaired
sources segfault (scene cases) or return a size-0 solution (unit case).
Matrix case `g5-fully-prescribed-body` accepted (crash on the baseline
binary). The five public smokes are bit-identical to the baseline binary.

## Suggested implementation order and copy-ready prompts

RBR-01, RBR-02 and RBR-03 are independent and done; RBR-05 is done; RBR-04
is done (reproduced first, then the guard). RBR-01 has the
most direct effect on ordinary quantitative output; RBR-02 hardens resource
containment; RBR-03 prevents the optional budget from becoming a large
memory consumer. None requires work on RB-23 or RB-24.

- “Implement only RBR-01 from `polyfem/docs/rb-review-repair-plan-20260920.md`.
  Reproduce the held-position kinematics defect, make output time state
  explicit, preserve solve/history ordering, and complete its acceptance.”
- “Implement only RBR-02 from the review repair plan. Reproduce the count
  overflow without insertion, repair checked broad-phase accounting,
  preserve all collision candidates and budgets, and validate the pinned
  IPC/PolyFEM integration.”
- “Implement only RBR-03 from the review repair plan. Bound the AL vector
  history while preserving the complete scalar pass record and identical
  numerical decisions; keep the budget off by default.”
- “Investigate RBR-04 from the review repair plan. Confirm the public
  improved-max/semi-implicit combination and its seam behavior. Implement
  the bounded unsupported-mode guard if reproduced; do not invent a new
  heterogeneous improved-max energy law.”
