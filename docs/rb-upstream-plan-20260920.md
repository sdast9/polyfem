# RB upstream contribution and dependency plan — 2026-09-20

**Status: plans only; no upstream issue/PR submitted and no dependency changed.**
Read the [review](rb-completed-review-20260920.md) for all RB-01–RB-24 verdicts
and the [repair handoff](rb-review-repair-plan-20260920.md) for RBR-01–RBR-04.
This file describes small future contributions, their dependencies, and the
tests needed before proposing them upstream. It is not an instruction to
merge the whole fork or execute every task at once.

## Scope and terminology

“Fixed” means one prescribed scalar stiffness; “global adaptive” means an
updated scalar shared by contacts; “per-contact variable” means spatially
different coefficients. Temporal changes to one global scalar do not by
themselves need parent-keyed coefficient identity. The general correctness
patches below benefit all modes, often even contact-free problems.

Reviewed upstream baselines, fetched September 20:

- PolyFEM `6c9e7a39063e844d86e2d6938329efab98dd506b`.
- IPC Toolkit `869e489e8db8a964897c797655418abf7bf9103e`.
- PolySolve `da4e7feb27af94b722015c774c501c2ace4e33d2`.

Before each contribution, refresh only the relevant upstream and compare
again: it may already have a repair. Build a minimal branch/worktree from
that upstream revision, verify the failure there, then port the small delta
and its regression. The fork's source commits are provenance, **not blanket
cherry-pick commands**; several also contain unrelated docs, defaults or
fork-only model plumbing. In particular, merging the PolySolve fork's whole
tree could discard unrelated newer upstream solver work.

Prepare a concrete patch and validation report before submission. Keep the
status explicit: proposed, reproduced upstream, patch tested, PR submitted,
merged. User approval to prepare these plans is not a request to send
maintainers messages or submit the entire sequence now.

## UP-01 — Contact cache ownership and exceptional sweep cleanup

**Destination:** PolyFEM. **Modes:** fixed, global adaptive, per-contact;
adhesion/smooth forms as applicable. **Priority:** high. **Origins:** RB-01,
RB-05 and RB-06's companion cache corrections.

Current upstream still has a function-static `cached_displaced_surface` in
`BarrierContactForm`, `NormalAdhesionForm` and `SmoothContactForm`. Position
equality across different form objects does not imply equal collision state.
See [upstream barrier source](https://github.com/polyfem/polyfem/blob/6c9e7a39063e844d86e2d6938329efab98dd506b/src/polyfem/solver/forms/BarrierContactForm.cpp).

Port the collision ownership/invalidation correction without the
semi-implicit coefficient controller. Either rebuild at each required
lifecycle point as the fork does, or use an explicitly object-owned,
fully-invalidated cache; do not reintroduce a position-only fast path.
Treat sweep cleanup as a separable commit: enable cached candidates only
after a successful build; an exception or aborted line search must clear
the cache and its validity flag.

Acceptance: two independent forms at the same positions with different
meshes/filters, reversed evaluation order, same coordinates after a filter
change, concurrent independent forms, and an injected swept-build failure
followed by a successful small solve. Port the relevant `[contact_cache]`
tests and adhesion/smooth coverage from `[rollback]` without requiring the
whole rollback feature. Use a numeric fixed coefficient and classic
adaptive controls so the test genuinely applies to upstream.

## UP-02 — Transient initialization and output at the solved time

**Destination:** PolyFEM. **Modes:** contact independent, hence all contact
modes. **Priority:** high. **Origins:** RB-04/RB-11.

Prepare separate patches:

1. Initialize the time integrator before constructing `InertiaForm` in
   `LinearElasticVarForm::init_linear_solve`. Current upstream constructs
   inertia first and reads an empty history via `x_tilde`.
2. Honor linear `time/quasistatic`: solve the time-indexed static equations,
   omit inertia, refresh the prescribed values/body load at the right time.
3. Normalize exported forces with the **solved step's** scale/predictor,
   including BDF startup. Carry RB-11's final follow-up, not the initial
   partial force fix. Sources: `LinearElasticVarForm.{cpp,hpp}` and the
   shared force-export argument in `ElasticVarForm` (fork commits
   `ef5dffe80`, `eaa624098`).
4. Correct nonlinear output kinematics after **RBR-01** supplies an explicit
   history-phase contract. Do not upstream the position-equality heuristic.
   RBR-01 introduced `varform::OutputTimePhase` on 2026-09-20; carry that
   contract, not the 2026-09-12 helper. The
   [2026-09-21 implementation review](rbr-implementation-review-20260921.md)
   found a missing `CurrentStepBeforeAdvance` transition in the specialized
   differentiable solve; repaired 2026-09-21 with a `differentiable=true`/
   `false` callback/VTU regression (`[output_kinematics][differentiable]`),
   which the port must carry.

Acceptance: real time-dependent linear input, not a preset analytical
problem whose `is_time_dependent` is false; static/quasistatic equivalence
for multiple dt values; nonzero growing prescribed displacement and load;
Euler/Newmark/BDF2/BDF3 startup and mature steps. Check individual exported
elastic/body/inertial forces against assembled `K`, `M`, load and replayed
integrator values as well as the free residual—a common wrong scale can
cancel in the residual. Include RBR-01's hold-segment VTU test. Run
`[linear_elastic],[output_kinematics]` and upstream time-integration tests.

Upstream reference:
[linear initialization](https://github.com/polyfem/polyfem/blob/6c9e7a39063e844d86e2d6938329efab98dd506b/src/polyfem/varforms/LinearElasticVarForm.cpp).

## UP-03 — Input, material binding and Debug-path correctness

**Destination:** PolyFEM. **Modes:** contact independent plus contact input
validation. **Priority:** high. **Origins:** RB-11/RB-12.

Split by contract rather than porting the entire 4,000-line input stage:

- **Per-element data:** preserve both full-mesh/global row files and
  body-local row files by validating length and binding the proper index.
  The non-first body's scalar data must not read the first body's rows.
  Sources: `MatParams`, `ExpressionValue`, `Assembler`, relevant parameter
  classes. Test two bodies with deliberately different scalar/fiber data,
  both valid conventions, wrong lengths, missing paths, and remeshing's
  explicit unsupported-data boundary. Do not weaken immutable snapshots.
- **Geometry and boundary data:** invalid indices/dimensions, duplicate
  cells, degenerate/nonmanifold obstacles and conflicting prescribed data
  should fail early with location/context. Sources: `MshReader`,
  `GeometryReader`, `MeshUtils`, `Mesh`, `GenericProblem`, `RhsAssembler`.
  Keep genuine valid meshes and known unsupported cases separate.
- **Material parameters:** paired Ogden arrays/spec acceptance, valid
  dispersion/fiber/density checks, and per-model validation under multi-model
  dispatch. Do not impose finite Lamé lambda on a shear-only/incompressible
  law where it is mathematically unused. Preserve its valid limit.
- **Debug correctness:** geogram volume mesh creation cannot require an
  absent edge/facet attribute; duplicate-cell checks must follow the actual
  mesh representation; averaged output fields must not copy FE values onto
  obstacle rows. Do not solve Debug failures merely by compiling out checks.

Use source commits `75b4d284d`, `ef5dffe80`, `f5f59db26`, `fffc722b9` as
references. Acceptance uses the existing `[input_validation]` and relevant
assembler/material tests, with upstream-only minimal fixtures, in Debug
and Release. The broad-phase enum repair (`sweep_and_prune` must dispatch
to its named method) is a small independent patch. Announcements of unit
sensitivity belong in UP-09, not in a giant validation bundle.

## UP-04 — Q3 basis correctness and complete collision surfaces

**Destination:** PolyFEM. **Modes:** all; the basis repair also applies
without contact. **Priority:** high, especially RB-23. **Origins:** RB-22/23.

Use three independently reviewable patches:

1. **RB-23 local-to-global frame repair.** Start from fork `329a1af25`'s
   `LagrangeBasis3d.cpp` delta. Preserve generated basis functions. Reverse
   the three misoriented vertical-edge enumerations, orient each face to
   the reference-node table, use the correct cell frame, and keep
   `hex_face_local_nodes` consistent. Current upstream still has the old
   enumeration. Port `test_hex_basis_layout.cpp`'s polynomial, continuity
   and interpolation checks without making them depend on fork contact.
2. **Unsupported-order guards.** Port mixed-order hex refusal before any
   negative placeholder is used as a node index and unavailable-table
   guards from `17678c546` (hex/quad Lagrange beyond existing tables,
   serendipity outside Q2). Explicitly name the current limitation; do not
   imply mixed-order stitching or Q4+ tables were implemented. Preserve
   upstream's supported simplex and uniform per-element-order paths.
3. **RB-22 extraction safety.** Port `BoundaryExtractionReport`, fail when
   any required collision face was skipped, validate displacement-map and
   form dimensions, and offer the DOF-resolution Q2+ proxy. Sources:
   `io/OutData.{hpp,cpp}`, `NonlinearElasticVarForm`, `FullNLProblem`.
   Separate the must-fix complete-or-refuse contract from the upstream
   default-choice discussion between a DOF proxy and sampled extraction.

Acceptance for the basis patch: verify actual stored node locations against
the generated reference nodes, random nodal trace continuity across shared
faces, full Qq reproduction on affine cells, total-degree-q reproduction
on non-affine trilinear cells, and convergence. Add legal local vertex/frame
permutations if upstream's fixtures lack them. Q1/Q2/serendipity/tet results
must remain unchanged. The fork's recorded Q3 rate 3.87 and zero node/trace
disagreements are evidence to reproduce, not substitute for upstream tests.

Acceptance for contact safety: valid closed mapped Q2/Q3 surfaces, explicit
failure on genuinely unsupported/degenerate extraction, no dropped body
in a two-body fixture, fixed and global-adaptive scene controls, and
unchanged Q1/tet proxies. This is geometry/conformity validation; it does
not certify curved-surface contact accuracy. A consistent mass matrix
must be used where row-sum lumping would produce invalid Q2 masses.

Reference: [upstream basis enumeration](https://github.com/polyfem/polyfem/blob/6c9e7a39063e844d86e2d6938329efab98dd506b/src/polyfem/basis/LagrangeBasis3d.cpp).

## UP-05 — Bounded broad-phase API and named resource failure

**Destination:** IPC Toolkit, then PolyFEM. **Modes:** all. **Dependency:**
RBR-02 checked arithmetic. **Origin:** RB-05.

Offer an optional `BroadPhaseBudget` with clearly defined cell-item and
pre-filter emission limits. Port hash-grid/brute-force preallocation checks,
exception type/statistics and candidate cleanup; unsupported methods must
declare that they cannot enforce a requested budget. Do not claim a total
RSS ceiling, change the candidate set, or truncate on exhaustion.

Propose toolkit API defaults separately from PolyFEM's selected automatic
limits (1e8 items / 5e7 emissions). Those values are application policy,
not universal physical or memory constants. Preserve raw exact counts on
normal cases and explicit overflow reporting from RBR-02.

Acceptance: toolkit `test_budget.cpp` including overflow/exception reuse,
candidate equality against unlimited controls, actual PolyFEM tiny-limit
and generous-limit runs, exit-status/manifest contracts, unchanged fixed
and adaptive endpoints. Keep CCD's narrow-phase memory outside this claim
(RB-24 / UP-12).

## UP-06 — Bounded roundoff acceptance in Armijo/RobustArmijo

**Destination:** PolySolve. **Modes:** all, including contact-free solves.
**Origin:** RB-19 plus its corrected September 12 implementation.

Port only `Armijo.{cpp,hpp}`, the `RobustArmijo` integration, schema option
and targeted regressions from fork `5afe3b5d` followed by `ee5b296a6`.
Current upstream Armijo only checks its ordinary energy condition.
Reference: [upstream Armijo](https://github.com/polyfem/polysolve/blob/da4e7feb27af94b722015c774c501c2ace4e33d2/src/polysolve/nonlinear/line_search/Armijo.cpp).

Preserve ordinary Armijo acceptance; if it rejects, permit the fallback
only within a finite explicit energy-change bound and with a finite,
strictly smaller gradient norm. A small gradient by itself must never
authorize an uphill move to a local maximum. Discuss the energy scale and
option/default with upstream; make disabling the new option demonstrable.

Acceptance: existing energy-descent cases, equal-energy/roundoff-floor
progress, nonfinite gradient/energy rejection, the nonconvex uphill control
in `line-search-small-gradient-keeps-energy-bound`, disabled option, and the
public no-contact step-1 stall fixture at 1/default threads. Keep original
stopping contracts, CCD limits and returned failure semantics. Run
PolySolve `[line_search]` and affected PolyFEM solves; do not port unrelated
linear solver or callback history with this patch.

## UP-07 — Attempt transactions and opt-in AL termination budgets

**Destination:** PolyFEM. **Modes:** all supported nonlinear elastic paths.
**Origins:** RB-06/07. **Dependency:** RBR-03 before the budget patch.

First port a clearly scoped solve-attempt transaction: form/problem state,
caller solution, owned collision/candidate state, friction lag, multipliers,
weights and lagged fields are restored when an attempt fails. Keep attempt
failure distinct from a publication failure after an accepted solve. Do not
introduce retries or claim coverage of arbitrary embedding applications
without testing their state owners.

Then propose the opt-in AL pass cap and multi-signal stagnation mechanism.
AL prepares a safe snap to prescribed boundary values; incomplete inner
minimization can be useful progress. Preserve energy/inversion/collision
snap gates and the final reduced solve's own convergence criteria. Carry a
complete scalar pass record and bounded vector history (RBR-03).

Acceptance: injected failures before/after AL, reduced solve, friction lag,
coefficient retune and real resource failure; restored-state solve equals
a fresh control; no failed endpoint is published. Then the compatible
105-pass fixture must not be falsely stopped; incompatible wall/crush
fixtures must report their correct stagnation/cap reasons and rollback.
Test budget-off identity and fixed/global-adaptive variants. An upstream
enabled default is a separate policy decision; retain opt-in behavior in
the initial proposal.

## UP-08 — Residual/lag observability and reproducible run identity

**Destination:** PolyFEM. **Modes:** all supported forms; unavailable fields
must remain unavailable. **Origins:** RB-04/09/10/12.

Offer small modules, not the entire research harness in one patch:

- Endpoint physical-unit residuals, support reactions, gap statistics and
  sampled det(F), with explicit supported-form checks and private contact
  snapshots. Keep the flag observational; no new solver acceptance gate.
- Solved-lag and updated-lag residuals side by side, plus lag termination
  reason. This is useful for fixed and global adaptive friction too;
  coefficient-independent normal-force changes can create lag error.
- Attempt history and outcome reporting without calling interruption
  convergence; failed/incomplete runs remain in the output.
- Effective-source/binary/input provenance and actual-executable CLI tests,
  with library opt-in versus executable defaults documented. Preserve
  upstream preferences for output paths/privacy and avoid absolute-path
  leakage into portable hashes.

Acceptance: diagnostics on/off have identical endpoints; diagnostics do
not mutate contact coefficients, lag or caches; forces divide by the actual
step scaling; unsupported forms do not produce a false pass. Include a
friction case whose updated lag fails while solved lag passes. Document
peak-force normalization, unloaded steps, and unavailable zero-load
normalization. Do not label right-endpoint work as exact dissipation or an
energy-conservation test. Run relevant diagnostics/event/manifest selections
and `tools/rb12/cli_check.py` with the freshly built executable.

## UP-09 — Unit contracts for CCD clearance and nonlinear tolerances

**Destinations:** IPC Toolkit and PolyFEM, separate reports/patches.
**Modes:** fixed/global adaptive/per-contact; the strongest measured
amplification was in the fork's controller. **Origin:** RB-09 T7.

Current [IPC CCD strategy](https://github.com/ipc-sim/ipc-toolkit/blob/869e489e8db8a964897c797655418abf7bf9103e/src/ipc/ccd/tight_inclusion_ccd.cpp)
uses `dmin + min((1-c)*(initial_distance-dmin), 1e-4)`. The `1e-4` is an
**extra-clearance cap**, not a permanent minimum gap and not proof of
collision failure. Current [PolyFEM tolerance scaling](https://github.com/polyfem/polyfem/blob/6c9e7a39063e844d86e2d6938329efab98dd506b/src/polyfem/solver/NLProblem.cpp)
uses `F0*L^(dim/2)` for L2, unlike Euclidean/Linf. `F0`'s name alone does
not specify its scaling under unit changes.

Prepare a minimal upstream reproducer with metre/mm versions of the same
physical problem. Convert every dimensional input consistently, derive
the coefficient conversion for the selected barrier formulation, and
compare raw-number conversion with the declared `units` path. Equalize the
actual force/step stopping criteria; do not assume multiplying F0 by a
volume factor is correct. Instrument the first-contact CCD bound and
subsequent controller changes. Include fixed and upstream global adaptive
controls instead of attributing the fork's measured 1e-3 reaction difference
to all upstream modes without measurement.

First output should be the measured contract and failure, then a proposal:
an explicitly dimensional extra-clearance parameter or documented
normalization for CCD, and precise tolerance units/scaling in PolyFEM.
Changing the default normalization or CCD strategy requires a separate
validated decision. **Do not remove CCD conservativeness or lower its
iteration budget to force unit agreement.** Acceptance must include
nonpenetration/conservative-step checks, units/cost comparisons, ordinary
scene controls and recorded failures. Distinguish the effect of UP-12's
root-finder update from this still-present absolute strategy constant.

## UP-10 — Optional per-contact stiffness identity and lifecycle extension

**Destinations:** IPC Toolkit API, then an optional PolyFEM model.
**Modes:** principally per-contact variable. **Origins:** RB-03/15/18/20/21.
**Dependencies:** RBR-04's supported-formulation boundary; no new model
selected here.

Divide the contribution into three design units:

1. **Coordinate contract:** expose the actual sparse displacement map and
   distinguish collision selection/proxy IDs from simulation basis IDs.
   Upstream's `semi_implicit_stiffness` utility samples Hessian/mass entries
   through `to_full_vertex_id`; document exactly which spaces it supports
   and test permuted selectors. General interpolation needs an explicit
   definition or named refusal, not proxy-ID sampling. The fork's local
   condensation/fallback is one model choice, not a universal chain-rule
   theorem to insert into all clients.
2. **Parent provenance:** carry originating primitive-pair contributions
   through normal-collision construction and TBB merge. Preserve signed
   weight bookkeeping, stable identities, and no-op behavior when every
   scale equals one. Test VV/EV/FV/EE transitions, merged parents,
   ordering/threading and contact birth/death. State RBR-04's restriction
   (2026-09-21): a per-contact scale that is the positive-parent weighted
   mean is a sum of parent potentials only when every contribution is
   positive; with the improved-max duplicate-removal corrections (negative
   weights) and unequal parent coefficients the energy jumps by
   `|κ₁−κ₂|/2·b(d)` where two edges meet (measured on the fork), so the
   fork refuses `use_improved_max_operator` under its semi-implicit mode by
   name. Positive-only averaging is not generally a signed-energy identity;
   supporting signed sets needs a consistent signed parent construction in
   the energy and all derivatives — a separate model decision.
3. **Coefficient lifecycle:** supply regression contracts for a frozen
   line-search objective, explicit retunes, finite/positive arithmetic,
   continued realized coefficients at refresh and defined friction-lag
   semantics. Make instrumentation distinguish coefficient changes at
   fixed geometry from geometric force changes. Keep new-contact behavior,
   timestep/material changes and topology/remeshing invalidation explicit.

Do not package positive-median caps, absolute-curvature fallbacks, global
trim control and force continuation as proven optimal physics. Upstream's
raw semi-implicit estimate also contains an inertial term; the fork's
mass-term choice cannot be imposed on it as a mechanical bug fix. The
optional model needs independent constitutive/unit/mapping/derivative,
transition, friction and accuracy comparisons. Retain fixed and global
adaptive references and the deliberately discontinuous stencil control.

The most portable pieces are mapping clarity, parent provenance and tests.
A full model adoption is a larger upstream design discussion. Fixed/global
adaptive users do not need spatial parent identity merely because their
single global coefficient is configurable or time-dependent.

## UP-11 — Public accuracy and negative-result benchmarks

**Destinations:** PolyFEM documentation/tests and IPC examples as appropriate.
**Modes:** comparisons across all. **Origins:** RB-09–RB-17, RB-23.

Extract small distributable fixtures/reference scripts with hashes and
predeclared quantities: analytical block and spring, same-mesh hard-contact
reference with non-tensile reactions, separate mesh/dhat/increment/unit
sweeps, friction lag refinement and Q3 interpolation/conformity. Clearly
separate model error, FE discretization error, algebraic residual and
termination. The public coarse cube's roughly 11% discretization error and
0.35–0.6% barrier-gap error are fixture measurements, not solver-wide error
bounds. Gap/compression is a useful conditioned comparison, not a universal
stress-accuracy theorem.

Retain the estimator/zero-demand/coupled-controller counterexamples from
RB-13–17. Rejected candidates must not become new default code. Test-oracle
inputs must be mandatory; missing Newton counts, deformation fields or
failed solves cannot count as passes. Avoid moving gigabytes of local
evidence or private scenes upstream. Publish compact inputs, reference
calculations, a runner and a versioned outcome table.

## UP-12 — RB-24 dependency adoption at the next sync

**Destination:** this fork's dependency integration; optional benchmark
contribution to Tight-Inclusion/IPC upstream. **Modes:** all CCD users.
**Status:** selected future remedy, not implemented; preserve the user's
September 20 decision to take it with the next dependency sync.

Upstream IPC already pins Tight-Inclusion 1.1.0 (`bb36e293`, August 7),
while `ipc-toolkit-fork/cmake/recipes/tight_inclusion.cmake` still pins
1.0.6. The current upstream
[recipe](https://github.com/ipc-sim/ipc-toolkit/blob/869e489e8db8a964897c797655418abf7bf9103e/cmake/recipes/tight_inclusion.cmake)
is primary evidence that a duplicate upstream code fix is unnecessary.
RB-24 measured the 1.1.0 bucket-DFS candidate in isolation; its reduced
queues address the diagnosed mechanism. It can return different conservative
TOIs under a cap, so this is a numerical dependency update, not a guaranteed
bit-preserving allocation optimization.

When a dependency-sync session is selected:

1. Inspect upstream IPC/Tight-Inclusion changes since the recorded refs
   and choose the actual target version explicitly. Preserve fork parent
   contributions, stiffness scaling, mapping API and resource budget.
   Reuse the existing RB-24 evidence before doing new instrumentation.
2. Record the effective loaded source, commit/tag and root-finding method,
   not just the requested pin. The older cached source's CMake project
   version string is stale, so a library version string alone is not
   sufficient provenance. Extend `RunManifest::libraries` and generated
   build identity to expose the resolved Tight-Inclusion version/ref and
   method where feasible.
3. Build baseline and candidate separately; preserve all CCD tolerances,
   iteration caps, rescaling, thread limits and scene inputs. A BFS control
   under the newer library can help attribute the effect to the finder.
4. Rerun five public smokes at one/default threads, the RB-05 resource
   suite (including RBR-02 when present), RB-02 parent-law probe, relevant
   IPC CCD tests, and RB-12 repeat matrix. Include fixed/global-adaptive
   controls as well as semi-implicit/friction. Record conservative-step
   behavior, cap hits, accepted endpoints, physical residuals and failures;
   identical single-thread endpoints were measured previously but are not
   a universal acceptance assumption for every query.
5. Measure peak RSS and live queue allocations/cost on the RB-24 probe at
   1/18 threads (or actual available counts). Keep platform/allocator and
   scene dimensions in the table. The prior 2,387→578 MB result is a
   reference measurement, not a promised 4x reduction on all scenes.
   Five repeat successes do not prove deterministic parallel execution.
6. Publish the validated dependency integration and pin with updated
   RB-24/RB-05/RB-12 records. Do not lower CCD iteration caps, alter default
   thread counts or fork a packed BFS as an incidental optimization.

For upstream contribution, extract the captured flat-face/parallel-edge
hard queries into small public CCD benchmarks after checking their
provenance. Include iteration-cap settings, input scale and memory/TOI
metrics. That can improve regression coverage of a remedy upstream already
has. Keep this separate from UP-09's absolute-clearance contract.

## Suggested order and exit criteria

Start with UP-01, UP-02's independent linear fixes, and UP-04's Q3 basis
repair: concrete upstream-shared defects with small regression contracts.
UP-03 can be split among subsequent independent contributions. UP-05 waits
for RBR-02; nonlinear output in UP-02 waits for RBR-01; the AL-budget part
of UP-07 waits for RBR-03. UP-06 is a standalone solver proposal. UP-08/11
add observational evidence. UP-09/10 require explicit API/model decisions.
UP-12 is taken when the dependency sync is selected, not as a new patch now.

A future task can be started with: “Prepare only UP-XX from
`polyfem/docs/rb-upstream-plan-20260920.md` against refreshed upstream.
Reproduce the defect there, port the minimal change and tests, validate,
and report a concrete patch with its remaining limits.” Preparing a patch
does not mean it has been submitted or accepted by maintainers.
