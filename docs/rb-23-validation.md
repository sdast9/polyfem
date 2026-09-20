# RB-23 — Q3+ hexahedral basis node bookkeeping

Date: 2026-09-20
Status: **validated within stated scope** (Q3 Lagrange hexahedra: every
basis stores the image of its reference node, the global space is conforming,
both RB-22 collision proxies are geometrically valid and the RB-22 Q3 scenes
complete; Q1/Q2/serendipity/tetrahedral paths bit-identical; mixed
per-element hexahedral orders are a named early failure). **Mixed-order
hexahedral stitching is not implemented** — the plan's "implement or refuse"
choice is resolved as *refuse* here; implementing it is a separate decision
(see the decision section).
Selected stage: 1 (reproduce, locate the swap) → 2 (repair, Q1/Q2
bit-identical) → 3 (unhide the acceptance tests, patch/continuity/convergence
tests, RB-22 `q3-*` scenes) in one session; the mixed-order TODO handled as a
named error.

## Contract and authorization

- User-selected item: "Let's address RB 23" (2026-09-20); RB-23 was
  `not started`. The plan's decision boundary — "a basis-builder change is
  outside any contact item; it needs its own authorization" — is satisfied
  by the selection of RB-23 itself, whose scope *is* the basis builder.
- Invariant (plan): every basis function's stored node position is the
  image of its reference node under the element's geometric map, and
  elements sharing a global node map it to the same point. Success
  criterion: the hidden RB-22 tests `[q3_hex_defect]` pass (0 mismatches of
  256 positions on the 4-hex column, 0 disagreements on the 48 shared
  nodes, both proxies closed / positive-area / intersection-free), Q1 and Q2
  hexahedra, tetrahedra and the HDA paths are bit-identical to the previous
  binary, and the RB-22 `q3-dof` / `q3-max_order` scenes complete or stop
  with an error unrelated to node positions.
- Dependencies: RB-22 (characterization, hidden acceptance tests, the
  degenerate-face refusal that stood in for this repair; record
  `rb-22-validation.md`), RB-03 (map contract: the Q3 DOF proxy rows are
  exact selectors, the lattice rows are the known 1−ε near-selectors),
  RB-11 (named early failures for unsupported input). Effective IPC
  `c24d803e` (`semi-implicit-stiffness`), PolySolve `bce32a39`
  (`iteration-callback`), both clean local overrides; unchanged.
- Exclusions / not authorized: no change to the autogen tables
  (`q_bases.py`, `auto_q_bases_3d_*`; the basis *functions* are the source
  of truth, they pass upstream's Kronecker/partition tests against
  `q_nodes_3d`), no Q4 hexahedra (no tables: `MAX_Q_BASES = 3`,
  `q_nodes_3d` asserts), no change to tetrahedral, prism, pyramid or 2D
  builders, no change to any default, no contact change, no implementation
  of mixed-order hexahedral stitching, no physical accuracy claim for Q3
  hexahedra beyond the interpolation checks below.

## Baseline and reproduction

- PolyFEM `main` at `a6d70bd49` (RB-07 record), clean tree, no other
  session's work present; the shared checkout and live build directory
  were used (no build or run was in progress).
- IPC `c24d803e` and PolySolve `bce32a39` from the local checkouts
  (`CPM_ipc-toolkit_SOURCE`, `CPM_polysolve_SOURCE` in
  `build/CMakeCache.txt`), both clean, unchanged by this item.
- Build: RelWithDebInfo, AppleClang 21.0.0, macOS Darwin 25.5.0 arm64, TBB,
  `POLYFEM_WITH_PYTHON=ON`, `POLYFEM_WITH_MISO=OFF`; runs single-threaded
  (`--max_threads 1`, `set_max_threads(1)`), `Eigen::SimplicialLDLT` where
  the scenes say so.
- Binaries: baseline `PolyFEM_bin` `922e08d4…` / `unit_tests` `23517e93…`
  rebuilt from the clean `a6d70bd49` tree (`--build_info`: `dirty: false`;
  the September-15 build in place hashed `62939e21…` because of embedded
  timestamps — see `baseline-bin/sha256.txt`); candidate `PolyFEM_bin`
  `a339afda…` / `unit_tests` `3d7bbcc1…` (`stage3-candidate-bin/`).
- Evidence: `outputs/rb-23/20260920T135948Z/` in the parent workspace —
  `source-identity.txt`, `stage1-baseline/` (hidden tests, mixed-order
  scenes), `stage2-candidate/` (tag runs), `stage3-regressions-on-baseline/`
  (the new tests against the unrepaired basis), `stage3-ab-smokes/`,
  `stage3-matrix-baseline/` and `stage3-matrix-candidate/` (RB-22 matrix,
  `--compare`), `stage3-full-suite/`, `stage3-hda-tests/`.
- Reproduced (stage 1, baseline `unit_tests "[q3_hex_defect]"`): 2 cases /
  4 assertions failed — **124 of 256** node positions are not the images of
  their reference nodes and **12** of the 48 shared nodes are placed
  differently by their two elements; both proxies refused with the RB-22
  degenerate-face error (32 zero-area faces of 324). Same numbers as the
  RB-22 record. Negative control: Q2 (`[hex_nodes]`) passes on the baseline.
- Reproduced (stage 1, mixed orders, `stage1-baseline/mixed-order/`): the
  column with per-element orders `1,2,1,1` dies with an unexplained
  `bad alloc std::bad_alloc` (exit 1) after "Node ordering disabled"; with
  `2,3,2,2` it runs a corrupt solve until "Gradient is nan; stopping"
  (exit 1). Neither names the cause.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| **Where the swap is: the local→global enumeration, not the stored positions and not the autogen tables.** `MeshNodes::node_ids_from_edge/face/cell` produce node ids in the frame of the `Navigation3D::Index` they are given (edge: from `index.vertex` towards `switch_vertex`; face: bilinear `(i, j)` with `i` along `index.vertex → switch_vertex` and `j` along `index.vertex → switch_vertex(switch_edge)`; cell: the same two directions plus the vertical one). `hex_local_to_global` handed them (a) the vertical edges e5, e6, e7 oriented `v1→v5, v2→v6, v3→v7` while the autogen layout (`q_bases.py`, the `1 − (i+1)/order` rule of edges 5–7) lists them **downward** `v5→v1, v6→v2, v7→v3`; (b) for every face the index returned by `find_quad_face`, i.e. the mesh's own arbitrary corner/edge frame (`get_index_from_element(c, lf, 0)`), while the layout enumerates each face's interior nodes in a fixed frame (x = 0 in the reversed frame `v7, v7→v4, v7→v3`; x = 1 `v1, →v2, →v5`; y = 0 `v0, →v1, →v4`; y = 1 `v3, →v2, →v7`; z = 0 `v0, →v1, →v3`; z = 1 `v4, →v5, →v7`); (c) for the cell the x = 0 face's frame while the layout is x outer, y middle, z inner (the z = 0 frame). Q2 is insensitive (one node per edge/face/cell; `face_node(·, 1, ·)` and `cell_node(·, 1, ·)` are frame-free centres). | code reading of `LagrangeBasis3d.cpp`, `MeshNodes.cpp`, `Mesh3D.cpp::edge_node/face_node/cell_node`, `q_bases.py`, `auto_q_bases_3d_nodes.cpp`; the baseline warnings (element 0 nodes 18–23: pairwise swapped within e5–e7; nodes 32–33: face 0 permuted) | located |
| Repair: `hex_local_to_global` enumerates edges in `HEX_EDGE_DIRECTION` (e5–e7 reversed), rotates every face index into `HEX_FACE_FRAME` with the new `oriented_quad_face_index` (asserting the third direction) and walks the cell from the z = 0 frame; `hex_face_local_nodes` uses the same edge table so its per-face edge-node order (used by the interface data and every `local_nodes_for_primitive` consumer) agrees with the layout. `MeshNodes` is untouched: its existing-node paths are position-matched (edge: first-node test; face: nearest of the stored set) so they follow the requested frame. | `git diff src/polyfem/basis/LagrangeBasis3d.cpp` | fixed |
| Acceptance: `unit_tests "[q3_hex_defect]"` — 0 of 256 mismatches, 0 of 48 disagreements, DOF and lattice proxies 164 vertices / 324 faces, closed, edge- and vertex-manifold, Euler 2, positive area, intersection-free | `stage2-candidate/q3_hex_defect.log`: 44 assertions / 2 cases pass | fixed |
| Q1/Q2 identity: the enumeration change cannot alter a single-node edge/face/cell (same primitive id, frame-free position); measured bit-identical on the five public smokes and the RB-03 Q1 hex scene (proxies and solutions, max\|diff\| = 0.0) and on the RB-22 Q1/Q2/serendipity matrix rows (below) | `stage3-ab-smokes/identity.txt`, `stage3-matrix-candidate/summary.json` | verified |
| The new regressions catch the defect: against the unrepaired basis (stash of the builder edit only, same tests) Q3 loses polynomial reproduction (342 assertions), the traces jump on both meshes (2), the interpolation rate collapses to 0.53 with errors 0.30 → 0.21 (not even consistent), the node checks fail (2 + 2), and the mixed-order case crashes with EXC_BAD_ACCESS (exit 138); Q1/Q2 pass on the baseline | `stage3-regressions-on-baseline/case-*.log` | verified |
| Mixed per-element hexahedral orders: a hexahedron of higher order than an edge/face neighbour gets placeholder ids (`-le-10`, `-lf-1`) that only the simplicial interface stitching resolves; the hex branch is `// TODO assert(false)` and, before it is even reached, the cube branch initialised every basis from `node_position(<0)` (out-of-bounds) — the baseline `bad_alloc` / NaN solve. Now `build_bases` refuses right after `compute_nodes` with a named error listing the hexahedra, their order and the lower neighbour orders (first 8, then a count). Uniform orders from a per-element file are still accepted (checked). Prisms never emit placeholders; pyramids keep their own path (unchanged). | `refuse_mixed_order_hexahedra`; `[rb23][input_validation]` test (three order patterns); `stage1-baseline/mixed-order/` | fixed (named error) — **implementation is the pending decision** |
| RB-22's degenerate-face error no longer names the repaired defect as the "known cause"; it now points at a degenerate input mesh (or a basis with wrong stored positions) and both records | `NonlinearElasticVarForm.cpp` | updated |
| Interpolation evidence for Q3 hexahedra (new `[rb23][hex_basis]` tests, `tests/test_hex_basis_layout.cpp`): the full tensor space Q_q is reproduced to 1e-10 on the axis-aligned column and total-degree-q polynomials on a 2×2×2 grid of non-affine trilinear hexahedra (every vertex moved by `x + 0.15 y z, …`); random nodal data are continuous across all interior faces of both meshes (jump < 1e-10, 12 interior points per face, Newton-inverted geometric map); sup-norm nodal interpolation of `sin(1.3x+.4) cos(.9y−.2) e^{.5z}` on the skewed 2³ and 4³ grids converges at **1.90 / 2.90 / 3.87** for Q1 / Q2 / Q3 (errors 3.2e-2 / 4.4e-4 / 1.2e-5 at h = 1/4); the test requires rate > q + ½ and err < 5e-2 / 8^(q−1) | `stage2-candidate/tag-rb23.log`: 12,068 assertions / 6 cases | measured |

Why the change restores the contract: basis `j` of a hexahedron is the
autogen Lagrange polynomial of reference node `q_nodes_3d(q).row(j)`; the
id assigned to it must be the global node whose stored position is the
geometric image of that row. `MeshNodes` creates or looks up nodes in the
frame of the index it is given, so the builder must hand it, per edge, face
and cell, the frame in which the autogen enumeration walks — which is now
tabulated (`HEX_EDGE_DIRECTION`, `HEX_FACE_FRAME`, `HEX_CELL_FRAME_FACE`)
and asserted. No numerical or model change: no setting, default, tolerance,
quadrature or law was touched; the geometric map (trilinear Q1 geometry) is
unchanged; edge, face and cell nodes are linear, bilinear and trilinear
blends of the same vertices as before, only assigned to the right basis.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail / not run |
| --- | --- | --- | --- | --- |
| Baseline reproduction | `unit_tests "[q3_hex_defect]"`, baseline `23517e93…` | fails as recorded by RB-22 | 124 mismatches / 12 disagreements, both proxies refused (32 zero-area faces) | reproduced (exit 42) |
| Baseline mixed orders | `stage1-baseline/mixed-order/mixed-{1211,2322}.json` | unnamed failure | `bad_alloc` / "Gradient is nan" | reproduced (exit 1 / 1) |
| Acceptance | `unit_tests "[q3_hex_defect]"`, candidate | pass | 44 assertions / 2 cases | pass |
| New regressions | `unit_tests "[rb23]"` (polynomial reproduction, continuity, convergence, mixed-order error, the two unhidden RB-22 cases) | pass | 12,068 assertions / 6 cases | pass |
| New regressions on the unrepaired basis | same tests, builder edit stashed | fail on Q3 and the mixed case | 342 + 2 + 2 + 2 + 2 failures, exit 138 on the mixed case | fail as intended |
| RB-22 suite with the Q3 sections rewritten | `unit_tests "[rb22]"` | pass, Q3 surfaces 164 / 324, all selector rows on the DOF proxy | 802 assertions / 10 cases | pass |
| Neighbouring tags | `[hex_nodes]`, `[contact_stiffness_mapping]`, `[quadrature]`, `[bases]`, `[input_validation]`, `[build_collision_proxy]` | pass | 18 / 289 / 24,905 / 347,907 / 183 / 1,097 assertions | pass |
| A/B smokes | `tools/rb22/run_ab_smokes.sh` baseline vs candidate: five public semi-implicit smokes + RB-03 Q1 hex, single-threaded | bit-identical proxies and solutions | all `IDENTICAL`, max\|diff\| = 0.0, 5/5 steps each | pass |
| RB-22 scene matrix | `tools/rb22/run_matrix.py` on both binaries, `--compare` | Q1/Q2/serendipity/two-body rows proxy- and solution-identical; `q3-dof` / `q3-max_order` complete; `q3-default` completes (routed to the DOF proxy) | 18 / 18 identical rows (max\|diff\| = 0.0); the 3 Q3 rows complete with valid surfaces, `q3-default` ≡ `q3-dof` bit-for-bit | pass |
| Full unit suite | `unit_tests` from the evidence directory, candidate `3d7bbcc1…`, 1 h 02 min wall (sharing the machine with the matrix) | only the pre-existing failures | **361 cases: 358 passed, 3 failed (4 of 5,170,910 assertions)** — `contact_2d`: `gcp-contact/cube-on-floor` and `large-ratios/large-mass-ratio.json`; `contact_3d`: `large-ratios/large-mass-ratio.json`; `standard`: `multi-material/stretch-cubes.json` (RB-22's named error on a P5 *simplex* face) — the same three cases as the RB-11/RB-12 records; the baseline binary fails `contact_2d` on the same two scenes (`stage3-baseline-contact2d/`: 26 / 28) | pass: no failure attributable to RB-23 |
| HDA tests | `houdini_HDAs/tests/test_*.py` (13 files) under Houdini 22.0.429 hython, the live `build/PolyFEM_bin` = candidate `a339afda…` | all pass | 13 / 13 exit 0 (`stage3-hda-tests/`); a first attempt earlier in the session found no SideFX license installed (`sesictrl print-license` empty) and was repeated once the user restored it | pass |
| Formatting / diff / links | the two records, the plan row, README | links resolve, `git diff --check` clean | checked before commit | pass |

Scene matrix (candidate vs baseline, `stage3-matrix-candidate/summary.json`,
single-threaded, `--timeout 900`; the machine also ran the full suite):

| Scene(s) | Baseline | Candidate | Identity |
| --- | --- | --- | --- |
| `q1-default`, `q1-dof`, `q1-max_order` | exit 0 | exit 0 | proxy and solution identical, max\|diff\| = 0.0 |
| `q2-default(-adaptive)`, `q2-dof(-adaptive)`, `q2-max_order(-adaptive)` | exit 0 | exit 0 | identical, 0.0 |
| `q2s-default`, `q2s-dof`, `q2s-max_order` (serendipity) | exit 0 | exit 0 | identical, 0.0 |
| `hextet2-q1-default`, `hextet2-q2-*` (two-body hex/tet, 6 rows) | exit 0 | exit 0 | identical, 0.0 |
| `q3-default` | exit 1: DOF proxy, 109 degenerate faces of 1728 | **exit 0**, 5 saved steps (0–4), DOF proxy 866 vertices / 866 selector rows / 1728 faces / 0 zero-area, closed, 141 Newton iterations, contacts 207–264, min distance 6.9e-5 (d̂ 1e-3), 206 s | new completion; solution **bit-identical to `q3-dof`** (the RB-22 default contract) |
| `q3-dof` | exit 1 (same error) | **exit 0**, same surface and solution as `q3-default`, 198 s | new completion |
| `q3-max_order` | exit 1 (lattice, 109 degenerate) | **exit 0**, 866 vertices (1 selector row, 865 near-selector rows within 1e-12 of 1 as RB-22 measured), 1728 faces, 0 zero-area, 142 Newton iterations, 201 s | new completion; endpoint within 1.0e-11 of the DOF proxy |

No physical acceptance is claimed for the Q3 rows (RB-22's own words for
the matrix apply: completion, conformity and identity measurements). The
Q3 cube takes ~4× the Newton iterations of the Q2 cube (141 vs 33 over the
same four steps) on 12,300 DOF; not investigated here.

- Numerical termination criterion versus independently measured residual:
  not applicable — this item changes no solve; the interpolation checks are
  exact (1e-10) and the convergence rates are measured, not gated by a
  solver tolerance.
- BC/reaction/gap/inversion/energy-work metrics: not measured; no physical
  claim is made for Q3 hexahedral contact beyond "the RB-22 scenes complete
  with a valid surface". A Q3 accuracy envelope (RB-09/RB-11 style) is a
  separate stage.
- Missing quantities: Q4+ hexahedra cannot be checked (no autogen tables);
  hex–hex order mismatches cannot be measured (now refused by name).
- Retained failed/partial runs: the baseline reproductions above; nothing
  else failed or was rerun.
- Test tolerances: node/continuity/reproduction 1e-10 absolute on O(1)
  quantities (the column polynomial is evaluated in scaled coordinates);
  rate bound q + ½ against measured 1.90 / 2.90 / 3.87; error bound
  5e-2 / 8^(q−1) against measured 3.2e-2 / 4.4e-4 / 1.2e-5.
- Not performed: Windows/Linux lanes (CI runs on push), private scenes,
  Teseo, any HDA rebuild (no asset change).

## Publication and reproducibility

- Rebuilt targets: `libpolyfem`, `PolyFEM_bin`, `unit_tests` (incremental,
  live build directory) from the working tree described by the diff; the
  tested candidate hashes are above.
- Committed files: `src/polyfem/basis/LagrangeBasis3d.cpp`,
  `src/polyfem/varforms/NonlinearElasticVarForm.cpp`,
  `tests/test_hex_basis_layout.cpp` (new), `tests/test_hex_collision_surface.cpp`,
  `tests/CMakeLists.txt`, `docs/rb-23-validation.md`,
  `docs/robustness-plan.md`, `docs/rb-22-validation.md` (handoff note),
  `tools/rb22/README.md` (the Q3 rows now complete), `README.md`.
  Remote/branch/commit: PENDING.
- Companion pins: unchanged (IPC `c24d803e`, PolySolve `bce32a39`). No HDA
  change; the HDA tests were run for evidence only.
- Remaining working-tree changes: none after the commit (the golden-test
  artefacts of the full suite were written to the evidence directory, not
  the repository).
- Small fixtures: the tests generate their skewed grids in the system
  temporary directory at run time; nothing new under `data/`.

## Decision required: mixed-order hexahedral stitching

The plan left "implement or refuse with a named error" open. This session
*refused*: the hexahedral interface stitching is a feature (constraining the
higher-order element's shared edge/face nodes to the neighbour's coarser
trace, as `LagrangeBasis3d` does for tetrahedra), not a bookkeeping repair,
and the previous behaviour was undefined. Implementing it would need its own
plan row (Q2/Q1 and Q3/Q2 hex pairs, hybrid hex/tet interfaces, the RB-22
proxies on constrained nodes, tests). Until then, a per-element hexahedral
order file must be uniform.

## Next session handoff

- Completed: stages 1–3 of RB-23; the mixed-order TODO as a named error.
  Pending inside RB-23: nothing.
- Status: validated within stated scope — the hidden tests pass, Q1/Q2/HDA
  paths are bit-identical, the RB-22 Q3 scenes complete with valid surfaces.
- Decision for the user: implement mixed-order hexahedral stitching (new
  row) or leave it refused.
- Next: RB-24 (per-thread contact memory) is the remaining open row; the
  RB-07 enabled-budget default is still the user's pending decision.
- Plan row and README updated in the same commit.
