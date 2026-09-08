# Fork robustness: session-ready work plan

Created 2026-09-08 after PF-01–PF-08 and constraint-floor retirement.
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

## Status and order

Rows began **not started** at plan creation; the table tracks current status.
Dependencies mean the indicated results must be
available before dependent implementation; they do not require unrelated work.
RB-02 and RB-03 can expose decisions needed before later physical certification.

| ID | Deliverable | Prerequisite | Current status |
| --- | --- | --- | --- |
| RB-01 | Contact-cache ownership and invalidation | none | [validated within stated scope](rb-01-validation.md) |
| RB-02 | Coefficient/lifecycle contract and counterexamples | RB-01 for same-process comparisons | [characterized—decision pending](rb-02-validation.md) |
| RB-03 | Collision/FEM coordinate mapping contract | RB-01; consult RB-02 | not started |
| RB-04 | Accepted-step physical accounting and diagnostics | RB-02 inventory; RB-03 supported mappings | not started |
| RB-05 | Bounded candidate generation and resource failure | RB-01; reuse RB-04 diagnostics where available | not started |
| RB-06 | Failed-attempt state rollback | RB-01; RB-02 state inventory | not started |
| RB-07 | Bounded AL stagnation handling | RB-04 diagnostics; RB-06 restoration | not started |
| RB-08 | Optional timestep/load-increment retry | RB-04, RB-06, RB-07 and explicit policy decision | not started |
| RB-09 | Reference benchmarks and refinement envelope | RB-04; resolve relevant RB-02/03 failures | not started |
| RB-10 | Friction coupling and dissipation validation | RB-04 and reference protocol from RB-09 | not started |
| RB-11 | Geometry/material/input validation envelope | none for audit; RB-09 for accuracy comparisons | not started |
| RB-12 | Repeatability, provenance and release checks | none for provenance; relevant RB checks for release | not started |

Recommended sequence is RB-01 → RB-02 → RB-03 → RB-04, then resource/recovery
work and physical-validation studies. RB-11 input auditing and RB-12 provenance
can be selected earlier. A dependency does not authorize completing two items
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
| New coefficient positivity/cap/retuning law | RB-02 | Compare alternatives with units, derivatives and force/work effects; obtain the user's model choice |
| Local stiffness definition for a nonidentity map | RB-03 | Derive the available mappings and compare candidate definitions; obtain a choice if the existing contract is insufficient |
| New acceptance criterion based on physical diagnostics | RB-04 / RB-09 | Define quantity, normalization and justified threshold; user selects application acceptance, separately from numerical stopping |
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
