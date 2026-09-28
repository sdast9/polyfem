# CI-04 — Mixed P1/P2 tetrahedral collision surface

Date: 2026-09-28
Status: **done — validated within stated scope** (cloud session, Linux GCC
13.3 Release). The default boundary extraction now tessellates the boundary
faces of P2 tetrahedra whose edge nodes are stitched to a P1 neighbor; the
collision surface of `multi-material/stretch-cubes.json` is complete, closed
and exact, the fixture's contact solve runs, and the `standard` group passes.
Native CI lanes have not run this branch yet. Item of the
[CI portability plan](ci-portability-plan.md#ci-04--mixed-p1p2-tetrahedral-contact-boundary).

## Contract and authorization

- Scheduled task: "CI-04 … repairing collision-surface extraction for mixed
  P1/P2 tetrahedral meshes", branch `cloud/ci-04`. In scope: keep the whole
  geometric boundary and the collision-to-FE map for the constrained P1/P2
  interface nodes by extending extraction. Not authorized: routing the case
  through a sampled proxy or any other change to models or defaults (that is
  the user's decision), disabling contact, lowering the order, suppressing
  the completeness error. RB-22's named errors stay.
- Read: CLAUDE.md, the plan section, [RB-22](rb-22-validation.md) (the
  completeness contract, `BoundaryExtractionReport`, the DOF-resolution proxy
  for high-order hexahedra), the decision boundaries in the
  [robustness plan](robustness-plan.md), `OutData.cpp` extraction, the
  conforming mixed-order branch of `LagrangeBasis3d.cpp`.
- No default, model or tolerance changed. No routing changed: the fixture
  still goes through "the default boundary extraction", and so does
  every other simplicial mesh.

## Host

Cloud container: Intel Xeon @ 2.80 GHz, `nproc` 4, GCC 13.3.0 (Ubuntu
24.04), CMake 3.28.3, Ninja, Release, TBB. `PolyFEM_bin --build_info`:
configuration Release, `POLYFEM_PORTABLE_BUILD=ON`, `POLYFEM_WITH_MISO=OFF`,
`POLYFEM_WITH_CLIPPER=ON`, `POLYFEM_WITH_TRIANGLE=ON`; sources polyfem
`ce5f750c` (the first branch commit; the later commits change tests and
docs only), ipc_toolkit `cf99893b`, polysolve `6a8c2cc9`; data
`sdast9/polyfem-data@e6ed5cf`.

Build note: the configure step could not fetch Boost 1.83 from
`archives.boost.io` (proxy 403). SourceForge served the identical archive
(SHA-256 `6478edfe…3b8e`, the pinned hash), so the build used
`-DBOOST_URL=file://…` on the cmake command line with the same checksum.
This changes nothing in the repository, and no tests were excluded.
Clipper downloaded normally.

## Reproduction (before the change, `origin/main` `c2a57e39`)

Isolated copy of `stretch-cubes.json` + `cubes_v41.msh`, single thread,
`--log_level debug`:

```
n_bases 18371
Contact is enabled but the default boundary extraction produced an incomplete
collision surface: skipped 437 of 7610 boundary faces on 437 elements
[12393, 13736, 13755, …] (order 2 with 10 nodes: 437 faces); first reason:
simplex face with 5 owned nodes (boundary export supported up to P4).
```

All 437 skipped faces had the same reason, 5 owned nodes on a P2 face. The
new focused test reproduced the problem on a 27-tet mesh with the
`x < 0.5` half at P2 (`quad_test/tet.msh`): *skipped 12 of 40 boundary faces
on 10 elements … simplex face with 5 owned nodes*.

## Root cause

On a conforming mesh `tet_local_to_global` gives an edge the lowest order of
the elements around it. A P2 tetrahedron's node on an edge that also belongs
to a P1 element therefore gets no global DOF (`-le-10`), and the basis builder
stitches it by evaluating the P1 neighbor's bases at the node position. Its
`global()` is then `{(v_a, ½), (v_b, ½)}`, the two endpoint vertices of the
edge. The simplex branch of `extract_boundary_mesh` dropped every node with
`global().size() != 1` and then only knew how to tessellate 3, 6, 10 or 15
owned nodes. A P2 boundary face with one stitched edge (5 owned nodes) or
two (4 owned) was skipped. Three stitched edges (3 owned) happened to form
the plain triangle already. Before RB-22 these faces were dropped silently;
since RB-22 the completeness check refuses the scene. This is correct
behavior, and it is why `standard` failed.

Traced weights on the fixture (test *The CI-04 fixture's mixed P1/P2
collision surface is complete and exact*): 437 faces with stitched edge
nodes, all of them 5 owned nodes (one stitched edge each). Every stitched
node is exactly `½ v_a + ½ v_b` of its edge's endpoints (max deviation
< 1e-12, no other entries). Evaluating the P2 element's FE field for a
random nodal vector at t = ¼, ½, ¾ along each stitched edge agrees with the
linear interpolant of the endpoint values within 1e-12. The displacement
along such an edge is linear, like on the P1 face across it.

## Change

`src/polyfem/io/OutData.cpp`, `tessellate_constrained_p2_face` and its call
in the simplex branch. The branch runs only for conforming meshes and only
for faces with at least one node where `global().size() != 1`. Such a face
is tessellated over its owned nodes, and every stitched edge is kept as the
straight segment between its endpoints:

| stitched edges | owned | triangles (polygon order, same orientation as the P2 pattern) |
| --- | --- | --- |
| 1 (a–b) | 5 | (m_bc, c, m_ca), (a, b, m_bc), (a, m_bc, m_ca) |
| 2 | 4 | (p, m_pq, r), (m_pq, q, r) |
| 3 | 3 | (v0, v1, v2) (same triangle as before) |

- **Conformity:** the face across a stitched edge is either a P1 face (its
  edge a–b) or a P2 face with the same stitched edge (orders are per edge).
  Both sides therefore have edge a–b with no midpoint, and no T-junction
  forms.
- **Map:** every surface vertex is still an FE node. On simplicial meshes the
  identity map is unchanged, and on non-simplicial conforming meshes the
  existing selector rows are unchanged. No proxy vertices are added.
- **Guards:** the face is refused through `skip_face`, and so through RB-22's
  named completeness error, if a vertex node is stitched, if a stitched node
  is not a positive affine combination of exactly its edge's two endpoints,
  or if the face is not a 6-node P2 face (for example mixed P2/P3). Non-conforming
  (`NCMesh3D`) meshes keep their previous code path unchanged.

A sampled proxy is not needed. The complete geometric boundary and an exact
map are available in the existing extraction.

## Acceptance

Evidence lived in `outputs/ci-04/<UTC>-prefix/` and `…-fixed/` (not
committed).

| Check | Result |
| --- | --- |
| Reproduction | 437 / 7,610 faces skipped, all "5 owned nodes"; synthetic 12 / 40 |
| Surface coverage | fixture: 0 skipped; collision-surface area = straight FE boundary area (rel. 1e-12); synthetic the same |
| Nondegenerate faces | all doubled areas > 0 (fixture 14,046 triangles, 7,025 vertices) |
| Expected topology | closed, edge- and vertex-manifold; fixture 1 component (the three regions share one glued mesh) with χ = 2; synthetic one sphere, χ = 2 |
| Intersection-free at rest | `ipc::has_intersections` false (fixture and synthetic) |
| Mapping dimensions | map rows = collision vertices, columns = FE nodes (18,371 on the fixture); no empty rows |
| Partition of unity | every row is a single exact entry 1 (selector rows = vertices, 0 interpolated) |
| Displacement agreement | `map · X_FE` reproduces the rest vertices (< 1e-14); FE field along stitched edges = straight edge (< 1e-12) |
| Contact solve | `PolyFEM_bin` on the isolated fixture copy, single thread: 20/20 steps, every Newton solve stopping on "gradient vector norm too small" in 4–5 iterations, rc 0, 647 s |
| `standard` group | `ctest -R ^standard$`: **Passed** (912 s), including `stretch-cubes.json` against its stored references |
| Related tests | `[rb22],[ci04],[hex_collision_surface],[collision_surface],[rb23]`: 18 cases, 13,383 assertions pass; `[build_collision_proxy],[coefficient_events],[contact_cache],[contact_stiffness_mapping],[upsample_mesh]`: 18 cases, 3,413 assertions pass |
| Unchanged paths | single-threaded before/after, SHA-256 of every step `.vtu`/`.vtm`: pure P1 tet (`5-cubes-fast`), pure P2 tet (same at order 2), Q1 hex (`5-cubes-hex-fast`), Q2 hex (DOF proxy), and the five `scenes/semi-implicit` smoke scenes: **9/9 byte-identical** |
| Formatting | clang-format 21.1.8 `--dry-run --Werror` clean on both files |

New tests in `tests/test_hex_collision_surface.cpp` (tag `[ci04]`):

- *Mixed P1/P2 tetrahedra get a complete conforming collision surface*
  (the 27-tet mesh, half P2): failed before the change with the named
  error, passes after.
- *Pure P1 and P2 tetrahedral collision surfaces are unchanged by the
  mixed-order path*: no stitched faces, closed sphere, 40 / 160 triangles.
- *The CI-04 fixture's mixed P1/P2 collision surface is complete and exact*
  (extraction only; the fixture's solve is in `standard`).

## What is left

- Native CI: the Linux/macOS Release lanes still have to confirm that
  `standard` passes on this branch once it is merged to `main`.
- Out of scope, still refused with a named reason: stitched nodes on faces
  of order ≥ 3 (mixed P2/P3 and similar), where a stitched node depends on
  a lower-order element's edge nodes rather than only on the vertices. The
  refusal names the face as "not an affine combination of its edge's
  endpoints" or as a non-P2 face. The item did not require handling these,
  and no fixture needs it. Handling them would require the stitched node
  as a weighted proxy vertex, plus splitting of the lower-order neighbor
  face.
