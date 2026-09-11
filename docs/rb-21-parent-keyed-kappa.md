# RB-21 — Parent-keyed κ (coefficient identity carried through the toolkit builder)

Date: 2026-09-11
Status: **done — toolkit `e3c8d3fe` and PolyFEM parent-keyed assignment implemented, regression-tested, validated with RB-20, published**. See the [progress log](#progress-log). See the [progress log](#progress-log).

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
   (the convergent formulation, never used by the semi-implicit mode) the
   corrections subtract one term in the corner region, and no constant
   per-collision scale can then be C⁰ on both sides unless `κ₁ = κ₂`; the
   residual jump is `|κ₁−κ₂|/2 · b(d)` between *neighboring* parents. Recorded
   as the inherent limit of that formulation; not exercised by production.
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
