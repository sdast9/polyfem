# RB-21 — Parent-keyed κ (coefficient identity carried through the toolkit builder)

Date: 2026-09-11
Status: **planned — starts after RB-20 Stage 1 is done**. See the [progress log](#progress-log).

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
   not compute it; PolyFEM sets `stiffness_scale = Σ_p w_p κ_p / Σ_p w_p` so
   that `weight · stiffness_scale · b(d) = Σ_p w_p κ_p b(d)` — each parent's
   contribution is continuous across its own subfeature switches, hence the sum
   is. With all `κ_p = 1` this is exactly `weight`, so the no-op property holds
   by construction.
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

### Validation plan

| Check | Expected | Status |
| --- | --- | --- |
| Toolkit unit tests (fork) incl. new parent test | pass | pending |
| PolyFEM affected suite + new `[parent_keyed]` case | pass | pending |
| Classic `adaptive` smoke | **bit-identical** Linf and endpoint (stiffness_scale ≡ 1) | pending |
| RB-04 candidate probe EV/VV transition | energy jump 0 (was −44.4977394), gradient continuous | pending |
| Five smokes (semi-implicit) | complete; endpoints reported vs RB-20 baseline | pending |
| RB-04 refinement matrix | drift metric unchanged or better than RB-20; no new failures | pending |
| Ball-on-plate (8 steps) | complete; metrics as RB-20 | pending |
| HDA E2E | pass | pending |

## Publication

Toolkit commit on `sdast9/ipc-toolkit:semi-implicit-stiffness`, pin bump +
PolyFEM implementation commit on `sdast9/polyfem:main`, documentation commit
with hashes. Parent README and plan row updated.

## Progress log

- **2026-09-11 18:35Z** — Plan written (see RB-20 for the shared context and
  the reason RB-21 follows it). Not started.
