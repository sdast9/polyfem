# RB-21 — Parent-keyed κ (coefficient identity carried through the toolkit builder)

**Review follow-up, 2026-09-12 (validated):** The assignment arithmetic was corrected after two finite parent coefficients overflowed their weighted sum even though their mean was representable. The parent-identity decision is retained; the bounded arithmetic repair and fresh validation are tracked in the [2026-09-12 follow-up](rb-review-followup-20260912.md).

**RBR-04, 2026-09-21 (implemented):** the improved-max seam of the analysis below was reachable through the public options and is now a checked restriction — measured on the real form (a finite `|κ₁−κ₂|/2·b(d)` jump, 62.24 on the fixture) and refused by name at input validation and at construction; see the [RBR-04 section](#rbr-04--the-improved-max-operator-is-a-checked-restriction-2026-09-21).

Date: 2026-09-11
Status: **done — toolkit `e3c8d3fe` and PolyFEM parent-keyed assignment implemented, regression-tested, validated with RB-20, published**. See the [progress log](#progress-log).

Companion of [RB-20 force-continuation κ](rb-20-force-continuation.md), which
holds the shared authorization and the sequencing decision (D8: RB-20 first).

## Contract and authorization

- **User selection (2026-09-11):** implement parent-keyed κ in the ipc-toolkit
  fork's `NormalCollisionsBuilder` — carry the candidate identity through to
  the built collision and accumulate the per-parent contribution instead of
  only the integer/area `weight` — then key PolyFEM's coefficient assignment on
  the parent; convert the RB-15 probe into a regression. Toolkit change +
  companion pin + PolyFEM change.
- **Invariant (RB-15):** for a declared parent interaction p (a candidate
  pair: point/edge in 2D, point/triangle or edge/edge in 3D), the potential
  `E_p = k_p · b(d_p(x))` with `d_p` the distance to the *whole* closed
  primitive keeps the same `k_p` when the closest subfeature changes. At
  positive gap the composed potential is then C¹ across EV↔VV / FV↔EV / EE
  subfeature switches; curvature may jump (no Hessian projection implied).
- **Boundaries preserved:** `stiffness_scale = 1` must be a no-op — the
  classic `adaptive` smoke and every non-semi-implicit path must stay
  bit-identical; toolkit `weight`/`weight_gradient` semantics, duplicate
  removal, OGC, area weighting and shape derivatives unchanged; RB-20's
  continuation semantics unchanged except for the key.

## Design

### Toolkit (`ipc-toolkit-fork`, branch `semi-implicit-stiffness`)

Today `add_edge_vertex_collision(mesh, candidate, dtype, weight, ...)` routes an
EV candidate to a VV collision when the closest point is an endpoint
(`P_E0`/`P_E1`), and the VV merge (`add_vertex_vertex_collision(vv, vv_to_id,
vv_collisions)`) *sums the weights* of every candidate that lands on the same
VV pair. The candidate identity is lost at that point; PolyFEM's stencil key
`(0, vi, vj)` therefore differs from the EV key `(1, ei, vi)` the same
interaction had one step earlier, and κ is re-estimated on a different stencil
(RB-04's 70→55 jump; RB-15's 3D 70→76.98).

Change:

1. `NormalCollision` gains `std::vector<ParentContribution> parents;` with
   `struct ParentContribution { int type; index_t id0, id1; double weight; }`
   (type ∈ {VV, EV, EE, FV} of the *candidate*, ids the candidate's primitive
   indices, `weight` the contribution that candidate added to this collision's
   `weight`). Sum of `parents[].weight` == `weight` is an invariant checked in
   debug builds.
2. Every builder path that creates or merges a collision appends the originating
   candidate's contribution: the four `add_*_collision(mesh, vertices, candidate,
   is_active)` entry points, the duplicate-removal helpers (`add_edge_vertex_
   negative_vertex_vertex_collision` etc., whose negative weights are also
   contributions), and `merge()` across TBB thread-local builders (concatenate
   parent lists on weight accumulation).
3. `stiffness_scale` stays a single double set by the caller. The toolkit does
   not compute it; PolyFEM sets `stiffness_scale = Σ_p w_p κ_p / Σ_p w_p` over
   the **positive** contributions so that `weight · stiffness_scale · b(d) =
   Σ_p w_p κ_p b(d)` — each parent's contribution is continuous across its own
   subfeature switches, hence the sum is. With all `κ_p = 1` this is exactly
   `weight`, so the no-op property holds by construction.

   **Seam analysis (2026-09-11).** In the default (non-convergent) formulation
   the builder applies no duplicate-removal corrections, so a vertex crossing a
   corner shared by two edges is simply two merged parent contributions
   (weight 2, scale `(κ₁+κ₂)/2`): energy `κ₁ b(d₁) + κ₂ b(d₂)` on either side,
   **exactly C⁰** for heterogeneous parents. Under `use_improved_max_operator`
   (the convergent formulation) the corrections subtract one term in the
   corner region, and no constant per-collision scale can then be C⁰ on both
   sides unless `κ₁ = κ₂`; the residual jump is `|κ₁−κ₂|/2 · b(d)` between
   *neighboring* parents. Recorded as the inherent limit of that formulation.
   *Correction (RBR-04, 2026-09-21):* this record originally said the
   combination was "never used by the semi-implicit mode"; it was reachable
   through the public options (`use_convergent_formulation: true`,
   `use_improved_max_operator: true`, `use_physical_barrier: false`) and ran
   without notice. It is now a checked restriction — see the RBR-04 section.
4. Toolkit test (`tests/`): a 2D vertex sliding past an edge endpoint into the
   neighbouring edge's region: the built collision changes EV→VV→EV while the
   parent lists show the two EV candidates with their weights; sum invariant;
   `merge()` preserves parents. Needs `ipc-sim/ipc-toolkit-tests-data` cloned
   into `tests/data` (see the fork-layout memory).
5. Companion pin bump in `polyfem/cmake/recipes/ipc_toolkit.cmake`.

### PolyFEM

1. `stencil_key` is replaced for coefficient purposes by a *parent key*
   `(candidate type, id0, id1)`; `kappa_cache_`/`prev_kappa_cache_`/RB-20's
   continued set are keyed by parent. The memo value κ_p is the Hessian
   Rayleigh quotient on the **parent's** stencil (EV: vertex + both edge
   vertices; FV: vertex + three face vertices; EE: four; VV: two) with the
   direction given by the parent primitive's distance gradient at the snapshot
   positions (`point_edge_distance_gradient` etc. with AUTO dtype — the closest
   point on the whole primitive), evaluated via a parent-type collision object
   passed to `ipc::semi_implicit_stiffness`.
2. `assign_collision_stiffness`: for each built collision,
   `stiffness_scale = Σ_p w_p κ_p / Σ_p w_p` over `collision.parents` (each κ_p
   resolved through the RB-18 law and RB-20 continuation exactly as a stencil
   value is today). Collisions with an empty parent list (should not occur;
   plane-vertex is skipped as today) fall back to the stencil path with a
   warning, so an unexpected builder path cannot silently zero a barrier.
3. RB-20's continuation now survives EV↔VV: the parent key persists, so the
   continued κ_p carries across the switch and the built collision's scale is
   the weighted mean of continued parents.
4. Regression: RB-15's frozen 70/55 EV/VV fixture (from `tools/rb04/
   candidate_probe.cpp` / `tools/rb15`) converted into a Catch case on the real
   form: energy and gradient continuous (to FD tolerance) across the subfeature
   switch at a positive gap; the 3D point/triangle FV↔EV case likewise; and a
   no-op check that with all parent coefficients equal the potential equals the
   unkeyed one exactly.

### Validation (final; evidence `outputs/rb-20/20260911T182152Z/`, shared with RB-20)

| Check | Expected | Result |
| --- | --- | --- |
| Toolkit unit tests (standalone fork build, tests data downloaded) | new `[parents]` case passes | **27 assertions**, `test_parent_contributions.cpp` |
| PolyFEM regression `[kappa_continuity][parent]` | builder parents recorded; seams exact under parent identity, historical jump under stencil identity; homogeneous equality; no-op outside semi-implicit; continuation survives the switch only under parent identity | pass (part of the 66 assertions, seeds 1–3) |
| PolyFEM affected suite | pass | 55 cases / 3,415 assertions, exit 0 |
| Classic `adaptive` smoke | **bit-identical** (stiffness_scale ≡ 1) | 2.2e-16 vs RB-19, Linf identical |
| Parent identity alone on the semi-implicit smokes | measured | frictionless/transient ≤7e-15, friction 4.0e-7 vs RB-19 (`smokes-parent-off`) |
| Quasistatic refinement matrix | cost-neutral alone; enables continuation | parent-off 431 Newton iterations (baseline 434); parent + continuation 433, 0 restarts, drift 1e-16 (stencil + continuation: 5,274 / 45 restarts / 1 failure) |
| Ball-on-plate (8 steps) | complete | see RB-20 (656 iterations on, 628 off) |
| HDA E2E | pass | pass |
| RB-04 candidate probe (`tools/rb04/candidate_probe.cpp`) | — | **not converted**: the probe drives the toolkit's stencil potential directly with hand-set coefficients; the equivalent statement (single- and two-parent seam continuity on the real form, with the historical jump reproduced under stencil identity) is the `[kappa_continuity][parent]` regression instead |
| RB-02 coefficient probe (`tools/rb02/coefficient_probe.cpp`) | the audit's EV↔VV fixture: coefficient continuous and objective C¹ under parent identity; the historical 70/55 jump under `coefficient_identity: "stencil"` | **270/270** (2026-09-13, library `756070f44`, `outputs/rb-02/20260913T054358Z`) — the probe's stencil-keyed expectation (70/55 at check 93) had been stale since the default flip and was updated to this law, not the law to it; measured jump −2.045e-3·(ε/1e-3)², exactly ½ ε·\|∂E/∂x\|, gradient jump 5.8·(ε/1e-3); see the [RB-02 record](rb-02-validation.md#regression-update-for-the-parent-keyed-law-2026-09-13) |

## Publication

- Toolkit `e3c8d3fe` on `sdast9/ipc-toolkit:semi-implicit-stiffness` (pushed).
- PolyFEM: `21f9fd592` (parent-keyed assignment, WIP) and the final RB-20/21
  commit `beb6ef641` (pin bump to `e3c8d3fe`, default flip, records) on
  `sdast9/polyfem:main`. Parent README and plan rows
  updated.

## Progress log

- **2026-09-11 18:35Z** — Plan written (see RB-20 for the shared context and
  the reason RB-21 follows it). Not started.

- **2026-09-11 20:40Z** — **Toolkit:** `ParentContribution {type, id0, id1,
  weight}` and `NormalCollision::parents`; every builder path (four candidate
  entry points, the EV/EE/FV subfeature routers, the four duplicate-removal
  helpers, the three merge helpers used by `merge()`) records its candidate;
  direct `emplace_back` paths set the single parent. Compiles inside the
  PolyFEM build; standalone toolkit test build with
  `tests/collisions/test_parent_contributions.cpp` (corner fixture: two edge
  parents on one VV collision, single parents in the interior region, weight
  sums, `stiffness_scale == 1`, area weighting) in progress.
  **PolyFEM:** `assign_collision_stiffness` refactored into
  `coefficient_keys()` (positive parents, tag `10 + type`, else the stencil key),
  `estimate_stiffness(stencil, key)` (fresh RB-18 law on any `CollisionStencil`
  — for a parent the rebuilt candidate with AUTO distance type) and
  `memoized_stiffness()`; a collision's scale is the contribution-weighted mean.
  Option `semi_implicit.coefficient_identity: "parent" | "stencil"` (default
  parent). Batch statistics are now taken over the memo (continued seeds +
  fresh keys) and the re-resolve pass is a second `assign` call.
  **Result on the quasistatic matrix** (`outputs/rb-20/20260911T182152Z/matrix-parent-on`, `-parent-off`):
  parent identity + continuation **9/9 complete, 433 total Newton iterations
  (baseline 434), 0 restarts, drift 1e-16 on every run**; parent identity
  alone 431 iterations with the baseline's drift — the identity change is
  cost-neutral and continuation now costs nothing. New PolyFEM regression
  `tests/test_kappa_continuity.cpp` `[kappa_continuity]`: 3 cases / 66
  assertions across seeds 1–3 (continuation: kept vs re-estimated control,
  bit-identical endpoint force and trim scaling, born-mid-solve pricing,
  mid-solve refresh keeps only endpoint keys, floor/cap bypass, max-ratio pull,
  invalid ratio rejected; parents: recorded by the builder, single-/two-parent
  seams exact under parent identity with the historical jump reproduced under
  stencil identity, homogeneous equality, no-op outside semi-implicit,
  continuation survives the switch only under parent identity). Next: toolkit
  tests, affected suite, smokes on/off, transient/friction/ratio matrices,
  ball-on-plate, HDA, then commit toolkit + pin + PolyFEM and flip the RB-20
  default.

- **2026-09-11 22:10Z** — Toolkit tests built standalone (tests data
  downloaded): `[parents]` 27 assertions pass. Validation table filled from
  the shared RB-20 evidence. Toolkit committed as `e3c8d3fe` and pushed;
  PolyFEM pin bumped in `cmake/recipes/ipc_toolkit.cmake`. The RB-04
  candidate probe was not converted (see the table); the seam regression on
  the real form replaces it. Done.

- **2026-09-13** — The RB-02 probe, last run at RB-18's final state
  (243/243), still asserted the stencil-keyed 70/55 coefficient jump at the
  EV↔VV switch and failed at check 93 against this item's default (found by
  the RB-10 session's run). Its feature-transition block now asserts the
  parent-keyed law — 70/70, one memo key across the switch, energy jump
  ≤ ε·max|∂E/∂x| (measured exactly half), gradient jump ≤ 3ε·max‖H‖ — with
  the stencil identity kept as the control reproducing the historical jump:
  **270/270** at `756070f44`, evidence `outputs/rb-02/20260913T054358Z`;
  published as `948014dbc`. No production change;
  the toolkit and this item's regression are untouched. Details and the
  measured table: [RB-02 record](rb-02-validation.md#regression-update-for-the-parent-keyed-law-2026-09-13).

## RBR-04 — the improved-max operator is a checked restriction (2026-09-21)

**Source:** the [completed-RB review](rb-completed-review-20260920.md) finding
4 and the [repair plan](rb-review-repair-plan-20260920.md#rbr-04--enforce-the-parent-law-supported-formulation).
**Evidence:** parent workspace `outputs/rbr-04/20260921T091854Z/` (`README.md`
maps it); starting sources PolyFEM `b6d0c9a45` (clean), IPC `482b9eab`,
PolySolve `bce32a39` = the pins.

### Reproduction (before any edit)

`tools/rbr04/seam_probe.cpp` drives the real `BarrierContactForm` on the
corner fixture of `[kappa_continuity]` (edges e0 = (v0,v1), e1 = (v1,v2)
meeting at v1; free vertex v3 at height .2; `dhat` 1; frozen snapshot with
v3 interior to e0, heterogeneous frozen Hessian → parent coefficients
κ(e0,v3) = 158.038, κ(e1,v3) = 200) and crosses both seams of the corner
without a refresh, with offsets 1e-2 … 1e-9 (`baseline/seam-probe/`,
525 checks). Constructed flags, signed weights, parents and assigned
coefficients were read back from the form:

| configuration | collision set left / right of seam A (x = 0) | jump, ε → 0 | classification |
| --- | --- | --- | --- |
| improved max, semi-implicit, parent identity, heterogeneous | EV(e0) w 1 s 158.038 / VV(v3,v1) w **1 = +1 +1 −1** (parents EV e0 +1, EV e1 +1, VV −1) s 179.019 = (158.038+200)/2 | **+62.2405 = (200−158.038)/2 · b(.2)**, seam B −62.2405 | **finite jump** (constant from 1e-3 to 1e-9; gradient jump norm → 347) |
| the same with area weighting (free vertex with its own edge) | VV w .5 + EV w .325 / VV w .825 | +11.43 / −29.02 | finite jump |
| improved max, semi-implicit, parent identity, **equal** coefficients | same sets, every scale 100 | 2.9e-3 · (ε/1e-2) → 0 | variation ∝ offset (the homogeneous improved-max potential) |
| improved max, `coefficient_identity: "stencil"` | EV / VV w 1 s κ_VV | 124.48 / −186.60 | the documented historical stencil jump (identical without improved max) |
| non-convergent (production default), parent identity | VV w 1 + EV w 1 / VV w 2 s 179.019 | **0** exactly | continuous — the sum identity |
| area weighting alone (no improved max), parent identity | positive contributions only | 0 | continuous |
| improved max under `fixed` / `adaptive` (scale 1) | same signed sets | 2.9e-3 · (ε/1e-2) → 0 | continuous (the toolkit's potential) |

The measured jump equals `(Σ weight·scale right − Σ weight·scale left) · b(d)`
in every row (the probe checks it), i.e. exactly the analysis above: the
positive-parent mean multiplies a *net* weight that contains the negative
correction. The toolkit itself warns on the plan's fragment that
"enabling the improved max approximator while not using area weighting may
lead to incorrect results".

Through the public JSON path (`baseline/json-path/`, the quasistatic public
smoke with the plan's fragment merged, `PolyFEM_bin` `5c539466…`): the
fragment **ran to completion** (4 accepted steps) with the manifest's
`solver.model` reporting `stiffness_mode: semi_implicit` and
`convergent_formulation.improved_max_operator: true` — the combination was
reachable and silent; the same with area weighting; the convergent defaults
(physical barrier on) were already refused by the constructor's existing
rule; area weighting alone and the classic `adaptive` improved-max run ran.

### Decision and change

The guard route of the repair plan. A signed parent construction (a
coefficient for the negative correction, consistent in the energy and all
derivatives) would make the combination C⁰ but is a separate model
decision; taking absolute weights or averaging the negatives away is not a
repair. No law, default, tolerance, CCD, retry or dependency changed:

- `BarrierContactForm` constructor: semi-implicit + `use_improved_max_operator`
  is refused by name (`unsupported_improved_max_message()`, shared with
  `State::init`), next to the existing physical-barrier and
  shape-derivative refusals; the message states the alternatives
  (`use_convergent_formulation: false` — the validated configuration; the
  convergent formulation with `use_improved_max_operator: false` and
  `use_physical_barrier: false`, area weighting alone keeping every
  contribution positive; or the improved max operator with
  `barrier_stiffness: "adaptive"` or a fixed value).
- `State::init` (RB-11 style, before any mesh is read): with
  `barrier_stiffness: "semi_implicit"` and `use_convergent_formulation: true`,
  every unsupported convergent option is named in one error
  (`contact.use_improved_max_operator and contact.use_physical_barrier are
  unsupported in this mode. …`); the physical barrier was refused later by
  the constructor before.
- Input spec: `use_convergent_formulation`, `use_improved_max_operator` and
  `barrier_stiffness` state the restriction (the improved-max doc no longer
  claims "currently not implemented").
- Unchanged: the non-convergent default (`[kappa_continuity][parent]` seams
  exact), the `"stencil"` identity control, `fixed`/`adaptive` with the
  improved max operator (the probe's continuous rows; `[rbr04]` asserts the
  signed parents `+1 +1 −1`, scale one and continuity at both seams), area
  weighting alone under semi-implicit (accepted; **not** newly validated —
  the scenes README's scope statement stands), the RB-02 probe.
- The Houdini asset emits `use_convergent_formulation` only (the convergent
  defaults follow), so with its default semi-implicit mode an area-weighted
  export was already refused by the physical-barrier rule; it now gets the
  combined message. No asset change.

### Regressions

- `[kappa_continuity][parent][rbr04]` (`tests/test_kappa_continuity.cpp`):
  the direct constructor refuses the combination (with and without area
  weighting) and names the alternatives; the default, area weighting alone
  and the other modes construct; fixed/adaptive improved-max sets carry the
  signed parents with scale one and are continuous at both seams; and the
  seam the refusal prevents, on the toolkit's improved-max set with the
  positive-parent mean applied by hand to the plan's example coefficients
  70/40: 70·b → 55·b at seam A (−15·b = −|70−40|/2·b), 55·b → 40·b at
  seam B, equal coefficients continuous, the non-convergent set 110·b on
  both sides.
- `[input_validation][settings][rbr04]` (`tests/test_input_validation.cpp`):
  `State::init` refuses the fragment (with and without area weighting) and
  the convergent defaults with both options named, refuses the physical
  barrier alone, accepts area weighting alone, the non-convergent default,
  and the fragment under `adaptive` / a fixed value.
- `tools/rbr04/seam_probe.cpp --expect-guard`: the five unsupported
  configurations refused by name, every control row identical to the
  baseline.

### Validation (`outputs/rbr-04/20260921T091854Z/candidate/`)

| Check | Expected | Result |
| --- | --- | --- |
| `unit_tests "[rbr04]"` | the constructor and `State::init` refusals, the accepted neighbours, the fixed/adaptive improved-max controls, the 70/40 seam on the toolkit's set | **2 cases / 79 assertions pass** (`polyfem-tests/rbr04.log`) |
| Affected selection `[kappa_continuity],[semi_implicit_coefficients],[input_validation],[contact_stiffness_mapping],[friction_lag],[contact_cache],[coefficient_events],[physical_diagnostics],[form],[run_manifest],[direction_filter],[contact_floor_retired],[resource_containment],[rollback],[al_budget],[fully_prescribed],[output_kinematics]` | pass | **117 cases / 10,673 assertions pass** (`polyfem-tests/affected-selection.log`) |
| `tools/rbr04/seam_probe.cpp --expect-guard` on the repaired build | the five improved-max semi-implicit configurations refused by name; every control row unchanged | **336 checks, exit 0**; refusals name the operator and the alternatives; control rows identical to the baseline run in classification and collision sets (parents order-insensitive), worst relative sample difference 3.8e-16 (`seam-probe/control-comparison.txt`) |
| Public JSON path on the candidate `PolyFEM_bin` | the fragment refused before any solve; the accepted neighbours unchanged | `improved-max-semi`, `improved-max-area-semi` and `convergent-default-semi` **refused at `State::init`, exit 1** (named failure, before the manifest is written — like the other RB-11 settings refusals; the defaults case names both options); `area-only-semi` and `improved-max-adaptive` complete — `improved-max-adaptive` identical to the baseline binary to roundoff (displacement 2.8e-16); `area-only-semi` identical to the baseline binary **single-threaded** (`compare-area-only-semi-threads1.json`), while threaded runs of this non-default area-weighted scene scatter by 2e-4 in displacement run to run on either binary (`area-only-semi-repeat{1,2}`; single-threaded repeats bit-identical) |
| Five public smokes, `--max_threads 1`, vs the baseline binary `5c539466…` (RBR-03's runs of the same binary) | byte-identical | **every frame byte-identical**, every VTU field identical (`ab-smokes/identity.txt`, `compare-*.json`) |
| RB-02 probe (`tools/rb02/run_probe.py`) | 270/270 | **270/270** (`rb02-probe/`) |
| clang-format on the changed C++ files | clean | clean (the probe source formatted after its evidence copies were taken: two joined lines, whitespace only) |

Limits: the restriction is checked, not the formulation validated — area
weighting alone under semi-implicit stiffness is accepted because every
contribution stays positive, and it is still outside the documented
validation (its threaded scatter above is one reason). The reproduction is
the 2D corner fixture and the public 3D smoke's dispatch; no 3D seam
(FV/EV/EE duplicate removal) was measured, since the guard refuses the whole
operator regardless of dimension. The companion toolkit test file
(`tests/src/tests/collisions/test_parent_contributions.cpp`) was located and
left unchanged: the toolkit's signed bookkeeping is unchanged and is pinned
from the PolyFEM side by `[rbr04]`. The Houdini asset was not changed (it
never emits `use_improved_max_operator`; its area-weighted export under the
default semi-implicit mode was already refused and now gets the combined
message). No golden file regenerated; nothing run on Teseo or a private
scene.

**Published:** the implementation commit on `sdast9/polyfem:main` (hash recorded by the follow-up note, as for RBR-03).
