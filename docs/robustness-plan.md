# Fork robustness: session-ready work plan

Created 2026-09-08 after PF-01–PF-08 and constraint-floor retirement.
Revised 2026-09-09: mechanically informed adaptive-barrier research, RB-13–RB-17,
and corresponding changes to RB-05–RB-12. Existing IDs retain their meanings.
**Current implementation status is recorded below; no new physical certification is claimed.**
Use the `RB-` identifiers below; do not renumber or reuse the historical PF items.

## Start here in every session

A sufficient user prompt is: **“Work on RB-01 using docs/robustness-plan.md.”**
The selected item's scope below is the work authorization, not authorization for
all downstream items. Work on one item at a time. For an investigation item,
finishing the investigation is not permission to choose a new physical model.

1. Read this file, the selected item, its dependencies and existing validation
   record, [the PF invariants](correctness-remediation-plan.md), and
   [floor retirement](pf-02-floor-removal.md). Read applicable workspace and HDA
   `AGENTS.md` instructions. If paths changed, locate the files before proceeding.
2. Inspect current branches, tracked/untracked changes, recent history, running
   builds/simulations, CMake dependency recipes **and effective source overrides**.
   Inspect relevant code; this document describes a dated starting point.
3. Preserve incoming user work and outputs. Do not reset/clean the checkout,
   switch its branch, kill another session's process, or edit a shared dependency
   another task is using. Use an isolated checkout/build when necessary. Work
   serially unless the user explicitly requests agent delegation.
4. If the item already has a record, resume from its last completed stage; do not
   rerun everything or claim a saved result is current. Check its commit and code.
5. State the selected invariant, baseline evidence, exclusions, and next stage.
   Reproduce a claimed defect before changing production code. An unconfirmed
   concern permits investigation, not an assumed repair.
6. Create a fresh parent-workspace evidence directory
   `outputs/rb-XX/<timestamp>/`. Record commands, inputs/hashes, binary/source
   identity, exits, summaries and incomplete/failed runs. Never overwrite earlier
   evidence or tracked baseline measurements to make a comparison pass.
7. Implement only the item's permitted changes. Run its acceptance checks and the
   common validation below. If a model decision is needed, finish the comparative
   evidence and present the concrete alternatives before asking the user.
8. Update this item's status row and `docs/rb-XX-validation.md` using the
   [record template](robustness-record-template.md). Commit/push the selected
   changes under project instructions; report exact remote/branch/commit and
   limitations. Documentation-only sessions need link/diff checks, not rebuilds.

If an expected source/API/test is missing or has changed, locate its replacement
and record the mapping. Do not fabricate an API or blindly apply old line numbers.
If a prerequisite is incomplete, do independent characterization that remains
valid; mark dependent implementation pending rather than silently completing it.

## Verified starting point and authority

At plan creation: PolyFEM `97bd180a4` on `main`, IPC `9da3094` on
`semi-implicit-stiffness`, PolySolve `4d372fa8` on `iteration-callback`, and HDA
`f455d36` on `sdast9/houdini-plugins:main`. The configured build consumes IPC from
its CPM cache and PolySolve from the local `polysolve-merged` override. Future
sessions must verify both effective source revisions rather than assume these.

The removed constraint floor must not be restored. Its old JSON key is ignored
compatibility data; positive values warn and cannot enable projection/deletion.
`trial_displacement_cap=50` remains separate and active in semi-implicit mode.
The optional force-saturation `gap_floor` remains zero by default. A robustness
fix must not quietly replace the retired mechanism with another force cutoff.

The PF records remain historical evidence. This plan governs **new RB work**;
where old PF investigation prose assumes an active floor, the later retirement
record supersedes it. PF-09 hard contact remains deferred, not implicitly approved.
A user instruction in the current session can explicitly change scope; record it.

## Non-negotiable boundaries

- Preserve AL's role as feasibility preparation and the final reduced solve's
  configured convergence contract. Intermediate AL nonstationarity is not failure.
  Callback/budget interruption is not final convergence. Do not impose a new
  gradient-only stopping rule or reinterpret negative-slope tolerance exits.
- Keep numerical termination, geometric feasibility, physical residuals and
  engineering accuracy as separate reported outcomes. Positive saved det(F),
  collision-free endpoints, and successful process exit are not interchangeable.
- Do not change production coefficients, friction lagging, constitutive laws,
  timestep schedules, tolerances, default resource limits or recovery policies
  unless the selected item expressly permits it and any stated decision is made.
- No silent candidate truncation, skipped collision pairs, CCD disablement,
  invalid-state acceptance, golden regeneration, or tolerance relaxation.
- **Do not run Teseo unless explicitly requested in that session.** The public
  fixtures below are available for these items. Private scenes, including
  Ballburst, need an explicit selection; historical permission/results are not
  a reason to run them automatically. Never modify the user's original inputs.
- Any new numerical/model choice without an established contract must be stated
  as a proposal with units, alternatives and measured consequences. An arbitrary
  positive stiffness minimum or larger penalty is not a correctness proof.
- Missing measurements are `not measured`, not zero or passed. A scene that
  stalls must remain in the result matrix. Do not count a successful rerun as
  erasing the initial failure or attribute it to threads/sleep without evidence.

## Agreed adaptive-barrier direction and next session

The user chose to improve the existing adaptive barrier, retaining Hessian-based
scaling and gap-band feedback. Fixed mode is a comparison baseline; replacing the
method with a trajectory-wide fixed coefficient or AL contact is not selected.
The AL work in RB-07 remains boundary-condition feasibility preparation, not a
new contact formulation. No global-versus-local coefficient replacement has been
chosen. This revision authorizes a research plan, not untested production defaults.

The working hypothesis is that contact strength can be estimated from the
materials' effective compliance, inertia and predicted compressive demand. The
barrier coefficient is not itself a material modulus or the barrier's tangent
spring stiffness. Derive its units and force/curvature interpretation before
changing its formula. Gap bounds express a desired numerical contact range,
not an assertion that every intermediate barrier energy is physical stored energy.

**RB-14 stages 1–3 are characterized—decision pending.**
[Stage 3](rb-14-validation.md) compares physical neighborhoods, shared prediction
and zero-demand protection: 5,299 checks, 125/126 nonlinear solves converged, one
retained incomplete outcome. Shared empirical coverage is 3/3 new holdouts but
2/3 prior challenges. No estimator or protection policy is selected. **RB-15 is
[characterized—decision pending](rb-15-validation.md)**: 140 checks compare shared
parents and complete smooth fields on 2D/3D transitions, with neighborhood and
force-direction limits retained. Frozen shared parents are the bounded candidate
recommended for RB-16/17; integration still requires the stated model choices.
All three RB-13 analytical
stages are [characterized in the reference contract](rb-13-contract.md).
The [validation record](rb-13-validation.md) preserves all three probes, including
stage 3's 654 checks, nonlinear prediction and contact-coupling counterexamples.
No production coefficient law is selected; application enclosures, nonlinear
and coupled behavior need the stated downstream work.
RB-14 estimates the reference quantities cheaply, RB-15 resolves feature consistency,
RB-16 compares controller timing/bounds, and RB-17 integrates the selected candidate.
RB-16 [spring stage 1](rb-16-validation.md) now records 126 completed trajectories
and 50,108 checks. Exact fixed-load intervals improve applicable band occupancy
at extra solve cost; misleading/unavailable predictors retain violations. Actual
real-form timing and public FEM cycles remain pending; no controller is selected.
RB-14 and RB-15 are separate investigations and can be selected independently once
their stated prerequisites exist. Finish missing RB-04 measurements as needed;
do not rerun its completed probes without a relevant code change or new question.
RB-04 is not a blocker for RB-05 resource containment or RB-13 analytical work.
Its discrete work convention, actual contact-path errors and coefficient/lag
endpoint identities are now characterized on the bounded fixtures. Use those
limits to decide whether an integrated candidate comparison needs additional
measurements; normal-law theory need not wait for full friction certification. RB-04 is now characterized with endpoint/path accounting limits documented;
see its final disposition before asking for tighter measurements.

Retain the existing RB-05–RB-12 IDs because records already refer to them. Resource
containment, rollback and validation remain necessary; numbering is not execution
order. RB-05–RB-08 implementation is required only when a selected experiment or
recovery feature depends on it. Small read-only probes do not need automatic retry.

Evidence carried forward: RB-02/04 reproduce the frozen EV/VV coefficient jump;
Fixed mode removes that specific jump but does not solve coefficient selection.
RB-03 exact indexing is repaired while interpolation curvature remains open.
RB-04 separates coefficient-event energy from displacement work and pre-/post-lag
friction states. The .5/.9 trim band is a working baseline, not an established
optimum; before-contact failures and successful repeats remain in the evidence.
See the [research log](rb-04-research-log.md) and
[candidate comparison](rb-04-candidate-comparison.md) for measured limits.

## Status and order

Rows began **not started** at plan creation; the table tracks current status.
Dependencies mean the indicated results must be
available before dependent implementation; they do not require unrelated work.
RB-02 and RB-03 can expose decisions needed before later physical certification.

| ID | Deliverable | Prerequisite | Current status |
| --- | --- | --- | --- |
| RB-01 | Contact-cache ownership and invalidation | none | [validated within stated scope](rb-01-validation.md) |
| RB-02 | Coefficient/lifecycle contract and counterexamples | RB-01 for same-process comparisons | [characterized—decision pending](rb-02-validation.md) |
| RB-03 | Collision/FEM coordinate mapping contract | RB-01; consult RB-02 | [characterized—decision pending; exact indexing validated](rb-03-validation.md) |
| RB-04 | Accepted-step physical accounting and diagnostics | RB-02 inventory; RB-03 supported mappings | [characterized—limits documented; endpoint/path diagnostics validated](rb-04-validation.md) |
| RB-05 | Bounded candidate generation and resource failure | RB-01; reuse RB-04 diagnostics where available | not started |
| RB-06 | Failed-attempt state rollback | RB-01; RB-02 state inventory | not started |
| RB-07 | Bounded AL stagnation handling | RB-04 diagnostics; RB-06 restoration | not started |
| RB-08 | Optional timestep/load-increment retry | RB-04, RB-06, RB-07 and explicit policy decision | not started |
| RB-09 | Reference benchmarks and refinement envelope | RB-04; resolve relevant RB-02/03 failures | not started |
| RB-10 | Friction coupling and dissipation validation | RB-04 and reference protocol from RB-09 | not started |
| RB-11 | Geometry/material/input validation envelope | none for audit; RB-09 for accuracy comparisons | not started |
| RB-12 | Repeatability, provenance and release checks | none for provenance; relevant RB checks for release | not started |
| RB-13 | Mechanical coefficient estimate and conditional bounds | RB-02 units; RB-03 maps; RB-04 evidence | [characterized—decision pending; stages 1–3 completed](rb-13-validation.md) |
| RB-14 | Practical compliance and force-demand estimators | RB-13 reference contract; RB-03 supported maps | [characterized—stages 1–3 complete; decision pending](rb-14-validation.md) |
| RB-15 | Feature-consistent local coefficient assignment | RB-02/04 transition evidence; RB-13 units | [characterized—decision pending; bounded 2D/3D comparison complete](rb-15-validation.md) |
| RB-16 | Mechanically informed gap controller and update timing | RB-13 bounds; RB-14 estimator; RB-15 candidate for integrated tests | [in progress—spring stage 1 characterized; real-form/FEM pending](rb-16-validation.md) |
| RB-17 | Integrated adaptive-barrier candidate comparison | RB-14–RB-16; RB-04 required measurements; RB-06 if failed-attempt recovery is exercised | not started |

Recommended research sequence after existing RB-01–RB-04 evidence is
RB-13 → RB-14 / RB-15 → RB-16 → RB-17 → RB-09 → RB-10. RB-05–RB-08
remain the resource/recovery track; select them when needed. RB-11 input auditing
and RB-12 provenance can be selected earlier. A dependency does not authorize completing two items
under one request. Each item may require several sessions with explicit stages.

Status vocabulary: `not started`, `in progress`, `characterized—decision pending`, `characterized—limits documented`,
`implemented—validation pending`, `validated within stated scope`, or
`blocked—<specific missing prerequisite>`. Only the last validation stage closes
an implementation item. Investigation completion must retain its decision-pending
label when a production model remains unresolved.

## Decision register: choices this plan does not make

Selecting an item authorizes its stated investigation and bounded implementation
work. It does **not** preselect any of these remaining decisions:

| Decision | Prepare evidence in | Required before proceeding |
| --- | --- | --- |
| New coefficient positivity/cap/retuning law | RB-02 evidence; RB-13–RB-17 | Compare alternatives with units, derivatives and force/work effects; obtain the user's model choice |
| Local stiffness definition for a nonidentity map | RB-03 / RB-13–RB-15 | Derive the available mappings and compare candidate definitions; obtain a choice if the existing contract is insufficient |
| New acceptance criterion based on physical diagnostics | RB-04 / RB-09 / RB-16–RB-17 | Define quantity, normalization and justified threshold; user selects application acceptance, separately from numerical stopping |
| Enabled production resource or AL budgets | RB-05 / RB-07 | Measure overhead/failure behavior and state proposed limits; user selects defaults; opt-in disabled-by-default mechanisms may be tested first |
| Automatic timestep/load retry policy | RB-08 | User approves the concrete policy or specified opt-in prototype before implementation |
| Friction, material, element or quadrature defaults | RB-10 / RB-11 | Separate model comparison and user agreement; do not bundle with an indexing/validation repair |
| Upstream/dependency upgrade or release promotion | RB-12 | Separate user instruction and applicable publication/validation procedure |

A new physical model is not a “routine implementation detail.” Conversely, a
reproduced ownership/indexing error with a clear existing contract does not need
another permission request to fix within the selected item. Record unresolved
choices precisely and continue independent evidence gathering where possible.

## RB-01 — Contact-cache ownership and invalidation

Completed 2026-09-08: [validation and lifecycle limits](rb-01-validation.md).
The stages below retain the baseline reproduction/acceptance protocol.

**Invariant:** one contact form must never skip its own required rebuild because
another form evaluated the same positions. Cache validity includes the relevant
geometry/topology, candidates and configuration, not merely coordinate equality.

**Read:** `src/polyfem/solver/forms/BarrierContactForm.{cpp,hpp}`
(`update_collision_set`), `ContactForm.{cpp,hpp}` (candidate lifecycle), and
`tools/pf02/contact_floor_probe.cpp`. The baseline function-static position cache
was reproduced and removed in RB-01; see the record. This does not establish
the cause of a historical scene failure.

**Stages / required cases:**
1. Add `[contact_cache]` tests with two real forms sharing numerical positions
   but independent collision sets. Evaluate A, then B, then A, and reverse the
   order. Compare each against an isolated fresh-form reference: collision
   stencils, energy, gradient and Hessian. Use a benign finite gap and constant
   positive system Hessian; avoid relying on a process-global cache in the oracle.
2. Cover equal coordinates with different topology/configuration; candidate-cache
   begin/end transitions; same-form unchanged positions; fresh simulation/init.
   Explicitly determine whether topology is mutable in this API. If not, use two
   distinct meshes/forms rather than inventing an unsupported mutation.
3. Replace shared mutable cache state with instance-owned invalidation or remove
   the optimization if its complete key cannot be made correct cheaply. Explain
   lifecycle and thread ownership. Do not clear a global cache as a workaround.
4. Test independent forms concurrently if their other dependencies permit it;
   do not claim same-form concurrent mutation is supported without such a contract.

**Acceptance:** baseline fails the demonstrated ownership case; corrected results
match the isolated references independent of order, with derivative checks where
needed. Propose relative/absolute tolerances before running and justify by the
well-conditioned fixture scale. Unchanged-position caching must not skip a needed
configuration/candidate rebuild. Run focused suite and public smokes. No stiffness,
CCD, tolerance or input-model change is in scope.

## RB-02 — Per-contact coefficient and lifecycle audit

**Question:** what exact objective is evaluated within a line search and across
retunes, and can an active contact lose its barrier through coefficient policy?
This item starts as characterization, not automatic coefficient redesign.

**Read:** `BarrierContactForm.cpp`: `assign_collision_stiffness`,
`refresh_semi_implicit_stiffness`, `calibrate_trim`, `post_step`, `bump_trim`;
`NonlinearElasticVarForm.cpp` and solve-data providers; IPC's effective
`semi_implicit_stiffness` implementation; `tools/pf02/` and `tools/pf08/`.
Locate provider definitions with `git grep system_hessian_provider`.

**Stages / required cases:**
1. Write a state table listing the owner, units, update triggers and consumers of
   the Hessian/position snapshot, contact stencil key, raw kappa, kappa_min,
   kappa_cap/median, form weight, global trim, restart state and friction data.
   Identify whether coefficients are frozen during each trial; do not infer it
   only from a comment. Record how newly appearing contacts get coefficients.
2. Exercise positive definite, indefinite, zero and singular local curvature;
   initially absent contacts; zero/positive median batches; coefficient caps;
   finite/nonfinite intermediate arithmetic; converted length/objective scales.
   Keep near-singular tests numerically bounded and fail explicitly on invalid
   inputs instead of deliberately crashing/OOMing the host.
3. Revisit the same trial in different evaluation orders under the same frozen
   state. Independently finite-difference energy to check gradient and Hessian.
   At identical positions measure energy/force before and after each permitted
   refresh/trim, keeping configuration fixed. Report which jumps are intentional
   algorithmic coefficient changes and which violate a promised lifecycle.
4. Trace resets of quasi-Newton/search history and lagged friction after retunes.
   Produce minimal reproductions for violations before fixing cache/lifecycle
   implementation errors that preserve the already documented mathematical law.

**Decision boundary:** changing kappa's sign/floor/cap law, forcing all coefficients
positive, moving retunes between solves, or changing the driving Hessian requires
an explicit model decision after comparative evidence. Do not equate zero kappa
with proof a particular production scene is wrong; report occurrence and effect.

**Acceptance:** checked-in state/units contract, deterministic frozen-state tests,
measured coefficient/force behavior and explicit outcomes for every listed case.
If the law itself is deficient, finish as `characterized—decision pending` with
concrete alternatives and validation requirements. Do not mark the contact model
validated merely because the audit is complete.

## RB-03 — Contact geometry to FEM coordinate mapping

**Invariant:** gradients, Hessians and stiffness inputs must refer to compatible
coordinates, including interpolation and prescribed/obstacle DOFs.

**Read:** `NonlinearElasticVarForm.cpp::build_collision_mesh`,
`ContactForm::compute_displaced_surface`, `BarrierContactForm` local Hessian
extraction, `NLProblem` full/reduced transforms, and the effective IPC
`CollisionMesh` displacement/force/Hessian mappings.

**Stages:** document the actual maps (e.g. surface displacement = B times FEM
increment, with prescribed offsets handled separately), ordering, dimensions and
units. Test identity mapping, permutation, nontrivial interpolated surface
vertices, fixed/moving obstacles and the supported higher-order or external
collision mesh path. Verify virtual work and finite-difference derivatives after
mapping. Test rigid translation/rotation where applicable and action–reaction.
Do not assume sampling a FEM Hessian block by surface vertex ID implements the
required mapped local curvature; derive the needed quantity and verify it.

**Permitted fix:** correct an index/chain-rule/invalidation error under an
established map. **Decision boundary:** if a unique local stiffness definition
cannot be derived for interpolation, present alternatives before choosing one;
do not invent a pseudoinverse/projection or quietly disable that mesh mode.

**Acceptance:** analytical tiny-map fixtures plus a real supported nonidentity
collision-map fixture; matching energy derivatives/virtual work and explicit
prescribed-motion behavior. Report unsupported configurations rather than
advertising arbitrary high-order/remeshed support. Keep identity-mode regression
and other contact formulations unchanged.

## RB-04 — Accepted-step physical accounting and diagnostics

**Deliverable:** an opt-in, versioned machine-readable record of accepted states
and failed attempts. It reports accuracy evidence; it does not change acceptance
criteria. Inventory current diagnostics first and extend/reuse them.

**Read:** `tools/pf08/physical_probe.cpp`, `measure_fem.py`, `test_measure_fem.py`,
`NonlinearElasticVarForm.cpp` solve/lagging/output paths, form weights and time
integrator state, and RB-02/03 records. Locate active output owners from code.

**Required fields:** phase and step/attempt identity; actual termination criterion;
BC error; recomputed full/free residual including contact and, when applicable,
inertia/friction; reactions with a stated sign convention; min gap and dhat;
sampled min det(F) with sampling scope; coefficient range/zero/nonfinite counts,
trim/refresh identity; proposed and accepted displacement; candidate/active counts;
timing and available process memory. Missing quantities must carry an unavailable
reason, not zero. Keep physical and acceleration-scaled objective units distinct.

**Accounting stages:** first a frictionless quasistatic public fixture, then a
transient fixture and a friction fixture. Record elastic/barrier/kinetic energies,
external work and frictional dissipation as applicable. Separate energy changes
caused by retuning at fixed coordinates from physical work; do not label numerical
coefficient changes as physical dissipation. Define integration/quadrature and
truncation error before asserting balance. A nonconverged lag still needs its
measured residual identified as such.

**Acceptance:** independent reference reconstruction and finite differences for
small models; compare diagnostic sums with exported results at the same endpoint.
Diagnostic evaluation must not retune coefficients, advance lagging, or alter
subsequent results. Test diagnostic on/off equivalence, failed-attempt labeling,
restarts and unavailable fields. Instrumentation completion and physical pass/fail
are separate. No new global convergence gate is authorized here.

## RB-05 — Candidate generation and resource failure containment

**Revised integration scope:** retain collision-candidate protection and account
separately for RB-14 compliance solves, estimator neighborhoods and retained
contact histories. A cheap estimator exhausting its budget must report that
condition; a fallback coefficient needs its own declared policy. Never confuse
an estimator approximation with permission to omit collision candidates. Keep
first probes small enough that this implementation is not a prerequisite.

**Invariant:** resource protection must never silently omit potentially colliding
pairs or accept an unchecked step. Current displacement capping is not a memory
budget; candidate-count logging happens after construction.

**Read:** `ContactForm::line_search_begin/max_step_size`, effective IPC
`Candidates::build` and selected broad-phase implementation, trial-cap options,
allocator/container behavior and exception propagation.

**Stages:** add pre-build step diagnostics (before large allocations), then build
a bounded thin-surface/swept-motion reproducer. Use small artificial limits to
test the failure path; do not reproduce a host-wide OOM. Inventory which candidate
and intermediate buffers can grow before any limit is checked. Distinguish a
candidate cap from total-byte/process-memory estimates and disclose unbounded
intermediates rather than claiming a hard memory ceiling.

Implement an **opt-in** candidate/resource limit only after defining where it can
be enforced before allocation. Default disabled preserves existing behavior.
On exhaustion, return an explicit resource failure and preserve the accepted
state; no partial candidate set can be treated as complete. Backoff/retry belongs
to RB-08, not this item. Changes to an effective IPC dependency require a tested
companion commit/pin; publish only to its existing fork branch under applicable
instructions. Do not change the production broad-phase default or cap=50 here.

**Acceptance:** disabled-limit equivalence; limit reached during construction;
cleanup and repeat invocation; no success output from a truncated set; useful
pre-failure diagnostics; bounded synthetic allocations. Record measured overhead
and the precise scope of resource protection. A default budget needs a separate
user choice based on target machines and measured workloads.

## RB-06 — Failed-attempt state isolation and rollback

**Revised integration scope:** extend the inventory to RB-14 estimator caches,
reference states, gap/demand uncertainty, RB-15 coefficient identity and RB-16
controller history. Distinguish a warm-started coefficient correction at the same
load/time from failure rollback to the prior accepted simulation step. Inject
failure between a coefficient update and the corresponding equilibrium solve;
never publish the old endpoint as solved under the new coefficient state.

**Invariant:** a failed attempt must not contaminate the last accepted state or
a subsequent attempt. This is in-memory transaction work, not automatic retry.

**Read:** `ALSolver`, active VarForm/legacy solve call chains, time-integrator
state, RB-02 inventories, friction/lagging forms, output/checkpoint writers and
current exception handling. Locate every mutable owner, not only `sol`.

Inventory positions, velocities/accelerations/history, AL multipliers/weights,
contact snapshots/candidates/trim, friction lag state, solver histories, counters,
and file/output commit points. Classify authoritative state versus safely
rebuildable caches. Specify snapshot/restore boundaries without switching
coordinate modes inside a line search.

Inject deterministic failures after AL, after coefficient retune, during reduced
solve and friction lagging, and immediately before accepted-output publication.
Test recovery by solving from the restored state against a fresh control given
identical inputs. Failure artifacts may be retained but must not appear as
accepted frames or overwrite an accepted checkpoint.

**Acceptance:** equality of authoritative state and equivalent subsequent
solution/forces; coherent cache reconstruction; no stale success callback/output;
error details preserved. Test both success and exception paths. Do not claim
crash/power-loss-safe disk checkpoints unless separately implemented and tested.
No smaller timestep, retry count, or new acceptance tolerance in this item.

## RB-07 — Bounded AL stagnation handling

**Revised integration scope:** this remains the existing prescribed-BC AL stage.
Separate its progress and budgets from contact-coefficient corrections (RB-16)
and friction lagging (RB-10). Do not escalate all weights when only one stage
stagnates. AL contact and multiplier-based normal-force replacement are deferred.

**Invariant:** geometrically stagnant continuation must have an explicit exit,
while useful interrupted AL iterates remain allowed under PF-01.

**Read:** `ALSolver::solve_al`, geometric snap gates, PF-01 and PF-07 records,
RB-04 diagnostics and RB-06 restoration contract.

Reproduce compatible prescribed motion requiring more than three interrupted
passes, incompatible motion, and a deterministic no-progress case. Record BC
error, energy finiteness, collision/inversion feasibility and penalty history.
Do not invent a distance-to-feasibility metric from BC error alone.

Propose a stage budget and stagnation metric with units, windows and explicit
reasons for failure. An opt-in budget may be implemented with default disabled;
choosing an enabled production default requires user agreement. Stagnation must
not be counted as convergence. Preserve initial-weight/ceiling semantics and
geometric snap behavior; no AL stationarity or raw-gradient requirement.

**Acceptance:** useful multi-pass continuation succeeds; incompatible/stagnant
cases stop under the configured limit and restore accepted state; zero initial
BC error and ceiling cases retain PF-07 behavior. Report off/on equivalence and
why the selected checks distinguish exhaustion from solver error. Do not add
load/timestep subdivision; that is RB-08.

## RB-08 — Optional timestep/load-increment retry

**Revised integration scope:** a missing/empty estimated k interval is diagnostic,
not automatically permission to subdivide time. Distinguish uncertain prediction,
coupled-contact incompatibility and actual solver failure. In a transient retry,
recompute the integrator-dependent inertia contribution and motion predictor at
the actual substep; do not reuse a dt-dependent k estimate blindly. Retune-at-same-
time and advance-at-smaller-dt remain separately logged operations.

**This is a policy decision, not yet approved production behavior.** Selecting
RB-08 first authorizes a concrete design/comparison. Implementation may begin
only after the user chooses the retry policy, or explicitly authorizes the
specified opt-in prototype. No elapsed waiting period substitutes for a choice.

Require RB-06 restoration and RB-07 bounded failures. Specify eligible failures
(resource limit, reduced nonconvergence, geometric infeasibility), ineligible
failures (invalid model/input, nonfinite constitutive data, unsupported state),
subdivision factor, minimum dt/increment, maximum attempts, target-time semantics
and output/checkpoint behavior. Propose numbers from evidence; do not silently
hardcode a policy. Reevaluate prescribed motion and loads at actual substep times.
Quasistatic path subdivision and transient time integration need separate tests.

**Acceptance after decision:** injected recoverable failure matches an equivalent
manually subdivided reference; final scheduled time/load is reached; no repeated
work/dissipation/output on rollback; irrecoverable and exhausted cases report
failure; retry-disabled behavior is unchanged. Capture all attempted schedules
and changed numerical paths. A successful retry is not the original timestep's
solution or proof of timestep independence. No retries for Teseo without its
explicit authorization.

## RB-09 — Reference benchmarks and refinement envelope

**Revised integration scope:** benchmark the RB-17 candidate only after its
contract is selected; existing modes can be measured earlier. Compare current
per-contact adaptation, the selected candidate and declared Fixed/global-adaptive
controls. Sweep material contrast, density and approach speed independently, plus
loading/unloading that activates both band limits. Include one coupled-contact
case and distinguish unit conversion from a change of physical size. Measure gap
prediction error, coefficient interval coverage, forces, work, Newton/retune counts
and total runtime. No claim of a 'vast majority' envelope without coverage data.

**Deliverable:** a reproducible accuracy matrix tied to quantities of interest,
not a universal “physically accurate” label. Begin by defining benchmark contracts
before executing the expensive matrix.

Use a small analytical spring/contact case, public cube/slab quasistatic and
transient cases, and an analytical continuum contact benchmark whose assumptions
match the implementation. Derive or cite the primary reference; do not select
Hertz or another model without checking small-strain, geometry and material
assumptions. Preserve separate discrete-solver and continuum-model comparisons.

Define displacement, reaction and integrated contact load as initial quantities;
local peak stress/contact pressure require mesh-aware sampling and singularity
handling. Ask the user for application quantities and acceptable error when
needed, while completing generic reference definitions independently. A 1%
screen is a reporting example, not authorized engineering acceptance.

Run at least three meshes where practical, three timestep/load increments, and
three contact-distance scales in **separate sweeps** before selected interactions.
Keep physical geometry/material/loading fixed except for the declared axis.
Also test equivalent unit conversions and separately physical material/density
changes; do not confuse the two. Choose finite-difference/test tolerances from
reference conditioning before observing production results; failed thresholds
stay failed unless a separately justified revised protocol is approved.

**Acceptance:** all input hashes, failed and successful attempts, independent
RB-04 force/work accounting, endpoint comparisons and observed trends/rates.
A completed study with sensitivity or unresolved stalls is `characterized—limits documented`, not
`validated within stated scope`. No golden replacement; investigate the existing
`contact_2d` cube-on-floor mismatch separately before claiming full-suite success.
Bounds on supported accuracy must state tested parameter ranges and uncertainty.

## RB-10 — Friction coupling and dissipation

**Revised integration scope:** use RB-04 pre-/post-lag pairs to identify the force
state actually solved. Cross normal-coefficient updates with slip reversal and
separation/recontact; verify normal-force transfer after retuning. A large residual
under a newly updated friction lag is not evidence that the preceding frozen-lag
solve failed. Assess coupled lag error separately. Frictional work must use a
declared lag convention; coefficient-event energy is not frictional dissipation.

**Invariant:** preserve the user's finite lagging policy while measuring its
accuracy and dissipation. More lagging iterations are a comparison, not an
automatically better or mandatory production policy.

Read `FrictionForm`, `NLProblem` lag updates, active VarForm lag loop, RB-02/04 and
PF-08's frozen-lag experiment. Test zero friction, steady sliding in both
directions, reversal, stick/slip near the smoothing scale, separation/recontact,
a moving obstacle and coupled normals. Recompute residual/reaction balance at
returned endpoints and integrate dissipated work with explicit units/signs.

Hold the default lag budget for the baseline. Compare a predeclared finite set
of budgets and smoothing scales, labeling all changes. Use the normal reactions
actually supplied by the barrier model, including retuning, not a fictitious
hard-contact multiplier. Distinguish frozen constitutive derivative tests from
coupled trajectory validation.

**Acceptance:** nonnegative dissipation under the stated convention, balanced
forces, constitutive derivative/reference checks, quantified budget/smoothing
sensitivity and failures. Resolve implementation inconsistencies after reproducing
them; a change of friction law, default smoothing or lag budget needs explicit
agreement. Do not certify impacts/stick–slip from steady sliding alone.

## RB-11 — Geometry, material and input validation

**Revised integration scope:** record the applicability of RB-13 assumptions:
positive/stable tangent, prescribed/free modes, anisotropy, near-incompressibility,
material softening, multiple coupled contacts and collision/FEM mapping. Distinguish
invalid inputs from valid physics that makes a local estimator unreliable.
Do not reject buckling, free-body quasistatics or heterogeneous materials merely
because H is singular/indefinite; report an unsupported estimator condition and
its selected handling. No silent identity-map or rigid-obstacle assumption.

**Scope:** inventory and strengthen input checks that prevent invalid or wrongly
mapped simulations; characterize the supported physical envelope separately.
Read mesh import/collision-map setup, `MatParams`, material-file snapshots,
HDA mesh/material export, and readPVD derived-field code relevant to the fixture.

Required cases: invalid indices, degenerate/inverted rest elements, duplicate or
nonmanifold collision topology, initial intersections, inconsistent units,
nonfinite/invalid material parameters, zero or misindexed fibers, and conflicting
prescribed motion. Establish which configurations are genuinely invalid versus
valid codimensional/open surfaces; do not reject all open meshes or nonuniform
materials by assumption. Use a supported physically valid control per failure.

Audit fiber/scalar data through import, element ordering, remeshing and export;
compare constitutive derivatives and rigid-motion invariance on a small fixture.
For nearly incompressible, anisotropic, thin or highly distorted elements,
characterize locking/conditioning and resolution needs with RB-09-style references.
Do not introduce stabilization, change quadrature/element/material defaults or
claim validation for all material models in one item.

**Acceptance:** invalid supported inputs fail early with path/element/material
context and no accepted output; valid cases remain accepted; per-element values
and reported units match source inputs. Label unsupported configurations and
unmeasured constitutive/element regimes explicitly. HDA changes require all HDA
tests, rebuilt assets and publication under HDA instructions.

## RB-12 — Repeatability, provenance and release discipline

**Revised integration scope:** manifests must include estimator/controller version,
reference state and coefficient-update sequence, gap convention, uncertainty method,
model-selection status, and any approximation/fallback used. Preserve evaluation-
order comparisons, failed runs and successful repeats. Publish a defaults table
that distinguishes historical behavior, opt-in candidate and explicitly selected
production behavior. A successful microfixture does not authorize promotion.

**Stages:** first run identity, then controlled repeatability, then CI/release
integration. This item can start before other RB work; do not claim a validated
release until the relevant checks actually pass.

A run manifest must identify effective PolyFEM/IPC/PolySolve revisions, dirty
state or source-patch hash, executable hash, build/compiler/linear-solver and
thread settings, HDA definition/hash when applicable, effective expanded input
and referenced-file hashes, platform, actual timestep/attempt history, completion
status and diagnostics schema. Distinguish source identity from binary identity;
never infer linked dependency code solely from the declared recipe. Avoid
publishing private paths/data or environment secrets in public CI artifacts.

Select a public small fixture and the public fine-load case that previously
failed then passed in PF-08. Run five repeats at fixed settings on one platform,
then a declared serial/threaded comparison where supported. Compare convergence,
restarts, reactions, energies, candidate counts and peak memory, not just VTU
byte hashes. Preserve all outcomes. If failures differ, investigate ordering,
shared state and numerical thresholds; do not label concurrency causal without
an isolated counterexample.

Add bounded CI checks for applicable RB regressions and the existing mode-gating
controls. Verify fork workflow configuration/results rather than relying on the
upstream badges in README. Separate unavailable platforms, known baseline
failures and new regressions. Cross-platform/compiler testing may require another
host; record it as pending, never simulated evidence.

**Acceptance:** manifest identifies tested artifacts; repeat matrix and declared
comparison tolerances are saved; appropriate CI is run and linked where available;
README claims match the most recent measured scope. No automatic dependency
upgrade, upstream merge, release tag or feature promotion is authorized here.
Source fixes discovered by the study should become a bounded reproduction/fix
stage, not unrelated sweeping edits.

## RB-13 — Mechanical coefficient estimate and conditional bounds

**Question:** can we derive a mechanically interpretable estimate and a conditional
interval for k from material response, inertia and predicted compressive demand?
This is analytical characterization and a standalone probe, not a production law.

**Read:** RB-02 units/provider contract, RB-03 full/reduced/collision maps, RB-04
work convention and candidate probe; effective integrator and barrier derivatives.
Create `docs/rb-13-contract.md` and `docs/rb-13-validation.md` when executing.

**Stage 1 — reference derivation:** use physical separation d and an isolated,
locally linear, stable frictionless contact. Let J map free displacement to gap,
H be the noncontact tangent in physical-energy units with prescribed motion in
the predictor, and f(d)=-db/dd. State the assumptions needed for

```text
K_eff = 1 / (J H^{-1} J^T)
H = K_tangent + M/dt^2                 [implicit Euler example only]
F_needed(d) = K_eff * max(d - d_free, 0)
k_estimate = F_needed(d_target) / f(d_target)
```

Apply H^{-1} by a solve; do not form an inverse in production. Derive other
integrators from their actual objective scaling. Do not double count mass: the
current provider already includes enabled inertia. Velocity/load history enters
the residual/predictor even when it does not change H. A load-dependent tangent
may require terms absent from the current provider; name every approximation.

Distinguish k (force/length^3 for the ordinary unweighted physical barrier),
barrier force k*f(d), and tangent spring stiffness k*b''(d) (force/length).
For the code's squared-distance potential B(s), s=d^2, explicitly derive
f=-2d*B_s and b''=2*B_s+4d^2*B_ss, with all weights and normalizations.
Do not transfer a physical-distance band directly into squared-distance settings:
audit the actual controller's statistic (minimum/mean squared gap, etc.).

**Stage 2 — conditional band:** for 0<d_L<d_U<dhat and positive decreasing f,
derive the isolated-contact interval k_L=F_needed(d_L)/f(d_L),
k_U=F_needed(d_U)/f(d_U). Establish when the force-balance root is monotone and
lies in the band. Restrict it to compressed contacts: free separation above d_U
must not trigger attractive force or an upper-gap requirement. A zero estimate
means no predicted compression, not permission to delete active-contact protection.

For genuinely enclosing estimates K_-<=K_eff<=K_+ and
p_-<=d_free<=p_+, test the conservative rectangle construction

```text
k_lower = K_+ * max(d_L - p_-, 0) / f(d_L)
k_upper = K_- * max(d_U - p_+, 0) / f(d_U)
```

Call this a rigorous bound only within the declared reference model and proven
input enclosure. Empirical error bars are estimates, not certified bounds. An
empty interval signals incompatible requirements/uncertainty under this model;
it is not proof that the physical problem is infeasible. No silent clamp, midpoint
selection, stiffness escalation or retry is authorized by an empty interval.

**Stage 3 — scope tests:** verify two supported springs in series, rigid/deformable
limit, implicit-Euler masses with differing approach speeds, unit conversion and
nonlinear local-tangent prediction error. Contrast compliance with the current
Rayleigh quotient: they represent different displacement restrictions, not equal
formulas. Derive multi-contact W=J H^{-1} J^T and demonstrate off-diagonal coupling
in a tiny two-contact fixture. Handle rigid modes/indefinite H explicitly; do not
silently select a pseudoinverse, PSD projection or arbitrary positive floor.

**Acceptance:** reproducible derivations, dimensional/chain-rule checks, analytical
force-balance roots and interval coverage including empty/inactive cases; listed
assumptions and counterexamples. Predeclare numerical tolerances from fixture
conditioning. No broad-simulation guarantee, selected production k or new safety
floor. Finish as characterized with remaining modeling choices stated.

## RB-14 — Practical compliance and force-demand estimators

**Prerequisite:** RB-13 reference equations and supported-map contract. Full RB-04
closure is not required for small estimator probes. Read current Hessian extraction,
provider scaling and RB-03 interpolation limits before any new local extraction.

Compare the current local Rayleigh scale with a declared small-neighborhood
compliance estimate and selected full-system solve references. Declare neighborhood
boundary conditions: fixed surrounding DOFs can overestimate stiffness relative
to relaxation. Diagonal/lumped approximations are candidates, not proven bounds.
Use normal gap derivatives with correct full/reduced pullback, not collision IDs
as FEM indices. Evaluate action/reaction and rigid/moving obstacle limits.

Predict the unconstrained gap from the local force residual and motion predictor;
avoid launching a colliding full 'contact-free simulation'. Reuse valid current
linearizations. Measure stiffness error and demand/gap prediction separately:
a good curvature estimate alone does not predict impact force. Vary material
contrast, anisotropy, supports, mesh scale, density, dt and approach speed in
bounded separate sweeps. Use a held-out fixture/parameter range after tuning.

Record compute/memory cost, locality sensitivity and uncertainty calibration.
Classify unavailable/unstable/stale estimates. Define a concrete fallback proposal
and measure it before selecting it; do not default to zero/infinity or reuse stale
k after dt/map changes. Determine whether one global adaptive coefficient suffices
as a control; preserving local adaptation remains the main research direction.

**Acceptance:** checked-in estimator comparisons with exact-reference errors,
predicted-versus-realized gaps, uncertainty coverage and cost. Demonstrate failure
of the assumptions as well as successful cases. No unexplained 'safety factor'
or coverage claim from training fixtures alone. Production estimator selection
requires the comparative evidence; output `docs/rb-14-validation.md` and contract.

## RB-15 — Feature-consistent local coefficient assignment

Completed bounded investigation 2026-09-11: [contract and recommendation](rb-15-contract.md),
[validation and limits](rb-15-validation.md). Production assignment remains unselected.
The protocol below retains the comparison and acceptance contract.

**Question:** can local mechanical scaling survive changes of nearest feature
without the reproduced frozen-snapshot energy/force jump? Reuse RB-02/04's
70/55 EV/VV counterexample; preserve its original data and Fixed control.

Compare a small, explicitly derived set of assignments: the current feature-key
rule, a shared coefficient over a declared contact neighborhood, and (only with
complete derivatives) a smooth geometry-dependent coefficient field. A global
adaptive coefficient is a control, not a selected replacement. For every candidate
state whether it defines an energy derivative at frozen state, a deliberately
lagged approximation, or an explicit outer model update. Merely smoothing k(x)
while omitting b*grad(k) does not prove an energy-consistent force. A shared region
can create jumps at its own boundary; test neighborhood changes and multiplicity.

Freeze all unrelated policies. Check decreasing left/right separations, trial
order independence, newly appearing contacts, separation/recontact, coefficient
refreshes at identical x, mapping and unit conversion. Retain fixed-region finite
differences and split gradient-path quadrature. Extend to at least one supported
3D feature transition before claiming general transition handling. Continuous
Hessians are not required merely because a C1 potential is desired; distinguish
permitted curvature jumps from finite energy/gradient jumps.

**Acceptance:** candidate contracts and measured continuity/work/derivative errors,
normal-force positivity where required, computational cost and material-contrast
limits. No arbitrary k floor/cap, contact deletion, interpolation redesign or
production mode switch bundled into the fixture. Propose the best assignment for
RB-16/17; retain a decision-pending status until its implementation choice is made.

## RB-16 — Mechanically informed gap controller and update timing

**Prerequisites:** RB-13 interval semantics, RB-14 estimator and reference baseline;
RB-15 candidate required for integrated comparisons. Start with simple springs
before public FEM compression/unloading. This item tests controller designs;
no new endpoint gate or default policy is preselected.

Use estimated force-demand intervals to inform, rather than blindly replace, the
existing lower/upper feedback. Compare current trim timing with bounded corrections
at fixed load/time, warm-starting from the latest feasible iterate. Hold coefficient
state fixed for each Newton direction and its line search; identify refresh and
search-history invalidation explicitly. Reassess the final endpoint under its actual
coefficient state. A post-publication refresh can prepare the next solve but must
not relabel the old endpoint. Early interruption requires a declared diagnostic
criterion, not an assumption that every intermediate iterate is equilibrated.

Predeclare small comparisons for update factor, hysteresis/window and interval
uncertainty. Test lower-bound compression, upper-bound unloading, free separation,
recontact, simultaneous heterogeneous contacts, initially missing contact and an
empty/unavailable interval. An average gap band does not bound every contact:
report minimum, distribution and the controller statistic. A desired gap is not
a hard minimum-distance guarantee; CCD remains separately responsible for the
geometric checks it implements. Keep the separate trial cap and retired floor
behavior unchanged.

**Acceptance:** stable mechanical predictions, band occupancy, prediction errors,
oscillation/retune counts, Newton iterations and total cost; energy changes due to
updates separate from displacement work. Include a deliberately misleading
predictor and record proposed handling without silent retry. Compare load-increment
refinement and a cycle exercising both bounds; do not tune using only successful
runs. Distinguish numerical solve termination from any proposed engineering/contact
acceptance criterion. Record concrete policy alternatives and remaining choices.

## RB-17 — Integrated adaptive-barrier candidate and decision

**Prerequisites:** selected estimator, assignment and controller contracts from
RB-14–RB-16; required RB-04 physical-state diagnostics. RB-06 is needed if the
experiment restores failed attempts; RB-05/07/08 only for the features it exercises.
Do not make automatic retry a hidden prerequisite of every contact improvement.

Integrate a candidate behind an explicit experimental mode after the concrete
choice is authorized. Preserve existing defaults and identity of all comparison
modes. Run an ablation matrix that isolates estimator, assignment and controller
changes before their combination; label any unavoidable mode differences such as
Fixed-mode trial-cap behavior. Start frictionless, including heterogeneous bodies
and transient approach; friction coupling remains RB-10's separate axis.

Use RB-09-style references on a bounded subset: spring/contact, public compression
and loading/unloading, plus a coupled-contact fixture. Keep physical inputs fixed
within each declared sweep. Report successful, failed and partial trajectories,
reactions/displacements, gap distributions, det(F), endpoint residual state,
coefficient-event energy, work integration limits and total resource cost.
Require selected unit-conversion and evaluation-order controls. A raw energy jump
at an explicit parameter update is not automatically physical dissipation or a
failure; an undeclared frozen-state derivative inconsistency remains a separate
issue. Apply full affected build/tests/smokes and HDA E2E for production C++ edits.

**Deliverable/acceptance:** one decision report comparing accuracy, robustness and
cost against the current adaptive baseline, with tested applicability ranges,
remaining failures, proposed user-facing parameters and an explicit default
recommendation. RB-09 broad refinement, RB-10 friction and RB-12 promotion remain
separate. No claim of a universal optimal k or coverage of most simulations from
a small matrix. Production default promotion requires a concrete user decision;
retain the tested opt-in implementation and evidence if the decision is deferred.

## Common execution and validation recipes

Run from the workspace root unless stated otherwise. Inspect the configured
build before using these commands; do not replace the user's CMake configuration.

```bash
# Record all three repo states, including user artifacts; do not clean them.
git -C polyfem status --short
git -C polyfem log -5 --oneline
git -C ipc-toolkit-fork status --short
git -C polysolve-merged status --short
rg -n 'SOURCE_DIR|OVERRIDE' polyfem/build/CMakeCache.txt
cat polyfem/cmake/recipes/ipc_toolkit.cmake polyfem/cmake/recipes/polysolve.cmake

# For solver changes, build before testing. Six jobs were used on this Mac;
# adapt concurrency to available memory and other running work, not blindly up.
cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6
```

Run the selected new test first. Then the affected baseline selection is:

```text
[contact_floor_retired],[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives
```

Invoke the **absolute** `polyfem/build/tests/unit_tests` path with that quoted
selection from a fresh evidence working directory; some tests write files in the
current directory. Record exit status and the Catch summary. At retirement this
selection passed 22 cases / 1,189 assertions; new counts may legitimately change.
Expected negative-test error logs are not failures if the assertions pass.

For solver smokes, copy the existing workspace `run-smoke.sh` into the evidence
folder and change **only** its `OUT` variable to a fresh evidence directory.
Run its five public inputs:

```text
quasistatic-adaptive.json
quasistatic-semi-alhess.json
quasistatic-semi-friction.json
quasistatic-semi.json
transient-semi.json
```

Verify each printed process exit and all four intended steps, not merely the
wrapper's final exit. New experiment inputs must be isolated copies; document
changed parameters. The existing script writes into the configured output tree
but reads from `polyfem/scenes/semi-implicit`.

The PF-02 probe can be built/run with:

```bash
python3 polyfem/tools/pf02/run_probe.py --build polyfem/build --output /absolute/fresh/evidence/probe
```

The PF-08 runner normally launches the **full** 108-configuration sweep. Do not
invoke it as a cheap compile check. Reuse its compiler/link recipe for a bounded
new fixture, save exact commands and state which configurations were run. The
historical 85/108 result is not a current green baseline. Preserve original
`tools/pf02/results-20260906.json` and `tools/pf08/results-20260907.json`.

Before publication: relevant formatter/syntax checks and `git diff --check`;
review the complete staged diff. Solver edits require build, selected regression,
affected smokes and HDA end-to-end when they affect its workflow. HDA source or
asset edits require all HDA tests and rebuilding **inside** its publication clone;
read the local `houdini_HDAs/AGENTS.md`, never copy `backup/`, and verify installed
assets against the published build. Dependency edits require the tested companion
commit and matching PolyFEM pin/effective build. Broaden testing when scope or
failures justify it; never conceal an unrelated baseline failure.

## Session completion report

Use the [validation record template](robustness-record-template.md), update the
status row, and report: reproduced/fixed/unresolved findings; exact tests/counts;
full versus partial scenes; parameter deviations; physical quantities measured
and missing; commits/remotes; any pending model decision; the next item that can
start. Link the new record from the status row. Never mark a numerical defect
fixed solely because a scene now finishes, and never mark an RB item complete
solely because its implementation was committed.
