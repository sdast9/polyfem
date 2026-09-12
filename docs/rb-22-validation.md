# RB-22 — High-order hexahedral collision surface

Date: 2026-09-12
Status: **validated within stated scope** for Q2 and serendipity-Q2 hexahedra (named error for every skipped face, DOF-resolution proxy as the selected default); **blocked for Q3+ hexahedra** by an upstream basis defect characterized here (nonconforming Q3 hex node positions), which now stops with a named error instead of a degenerate surface
Selected stage: stages 1–3 of the [plan section](robustness-plan.md#rb-22--high-order-hexahedral-collision-surface) — reproduction and crash mechanism, named error, candidate-default comparison with measurements, the user's default decision, tests through the production builder, bit-identity of the unchanged paths

## Contract and authorization

- User-selected item: RB-22 ("let's address RB-22"). Explicit decision in
  session, 2026-09-12: **(b)** — a contact-enabled mesh whose boundary has
  Q2+/serendipity hexahedral faces gets the DOF-resolution proxy by default
  ("I want accuracy, so if b provides an accurate solution, let's proceed
  with that"). What "accurate" means here is stated under
  [Findings](#findings-and-changes): exact displacement-map rows and the same
  oriented surface as the lattice alternative on Lagrange Q2, with the
  chordal representation of the curved surface that every available proxy
  shares.
- Invariant: a contact-enabled scene must either build a conforming,
  watertight collision surface for every boundary face of every element type
  it accepts, or stop with a named error naming the elements and orders. A
  silently partial or empty surface and an out-of-bounds crash are both
  violations. Success criterion: no crash on a Q2+ hex contact scene; a
  conforming surface with completed public scenes or an explicit error;
  unchanged Q1/tetrahedral/HDA paths (bit-identical); record and status row.
- Dependencies read: RB-03 [map contract](rb-03-contract.md) and
  [record](rb-03-validation.md) (exact selectors vs interpolated rows are
  both supported by the stiffness code; `1ea71d93c`), the
  [PF invariants](correctness-remediation-plan.md), floor retirement,
  workspace and HDA instructions. No unresolved limit of RB-03 applies: the
  default proxy selected here produces only exact selector rows.
- Exclusions / not authorized: no change to coefficients, friction, CCD, the
  trial cap, tolerances, defaults other than the routing decided above, the
  basis builder (the Q3 hex defect is characterized and handed off, not
  repaired), the legacy `polyfem::legacy::State` extraction (not the
  production path for elasticity), or the `max_edge_length` / `linear_map`
  proxy builders (they already throw named errors for unsupported elements).
  No Teseo, no private scene.

## Baseline and reproduction

- PolyFEM `main` at `34aff1fb4` (docs) on implementation `1ea71d93c`; clean
  working tree at start; the RB-03 evidence's binary
  `b9e3aca4d9587469…` was the installed `build/PolyFEM_bin` (no source newer
  than it). During the session another session updated `README.md` and the
  Houdini memory (HDA `d49121e`); both were preserved.
- Effective companions: IPC Toolkit `ipc-toolkit-fork` clean at `e3c8d3fe`
  (`CPM_ipc-toolkit_SOURCE` override, matches the recipe pin), PolySolve
  `polysolve-merged` clean at `5afe3b5d` (`CPM_polysolve_SOURCE` override,
  matches the pin). Neither companion changed.
- Build: macOS arm64 Apple Clang, RelWithDebInfo, Unix Makefiles,
  `POLYFEM_WITH_MISO=OFF`, `POLYFEM_WITH_OPTIMIZATION=ON`; linear solver
  `Eigen::SimplicialLDLT` in every scene; scene runs single-threaded
  (`--max_threads 1`) unless stated; unit tests `set_max_threads(1)`.
- Binaries: pre-change `build/PolyFEM_bin` `b9e3aca4…`; an independent
  **baseline binary built from `34aff1fb4` in a scratch worktree** with the
  same options, `128bfc2944344f73…`, used for every A/B comparison; stage-1
  binary `546c71693d52f6a9…`; final binary and `unit_tests` hashes in
  `inputs/sha256-binaries-final.txt`. HDA identity unchanged (no asset
  change; the HDA test was rerun against the rebuilt binary).
- Evidence (parent workspace): `outputs/rb-22/20260912T140021Z/` — `inputs/`
  (copied RB-03 probe scenes, SHA-256 of inputs, mesh, slab and binaries,
  companion HEADs), `baseline/` (pre-change reproduction), `stage1/` (named
  error, opt-ins), `matrix-baseline-34aff1fb4/`, `matrix-rb22/` (stage-2
  binary), `matrix-rb22-final/` (final binary, `--compare` against the
  baseline), `matrix-run1-misplaced-outputs/` (first matrix run, whose
  VTU/proxy files the runner had written under the scene directory before
  its path bug was fixed; kept as evidence), `hextet-probe*/`,
  `hextet2-probe*/`, `hextet2-consistent-probe/`, `exploratory-scenes/`
  (the archived lumped-mass two-body variants), `ab-smokes*/`,
  `affected-suite*/`, `hda-e2e*.log`, `endpoint-differences.json`.
  Commands: `tools/rb22/run_matrix.py` writes `command.txt` per scene;
  `tools/rb22/run_ab_smokes.sh`.
- Reproduced symptom (pre-change binary, RB-03 Q2 hex probe inputs):
  `q2-default` (semi-implicit) and `q2-adaptive` both **exit 138**
  (`EXC_BAD_ACCESS`) after saving `step_0`; 96 of 96 boundary faces skipped
  at trace level (0 visible lines at `debug`); `q2-nocontact` exit 0;
  `q2-max-order` exit 0 (140 contacts, 5.8 s wall). Baseline matrix
  (single-threaded, scratch baseline binary): `q2-default` exit −10,
  `q2-default-adaptive` −11, `q2s-default` (serendipity) −10 — all with a
  0-face proxy; `q3-default` **wrote 4 corrupted steps** (max |u| 0.1875)
  before exiting −11 after 27.9 s; `q2s-max_order` aborted in a
  `GradientDescent` fallback (0 faces: the sampled extraction skipped the
  serendipity elements); `q3-max_order` "initial solution has intersections"
  (degenerate surface); **`hextet2-q2-default` exit 0 with 31 completed steps
  while the hex body was absent from the collision surface** (48 faces =
  the P2 tet cube only, one closed component; max |u| 0.456 against a
  0.05 initial gap — the cube fell through the hex; the Q1 control rests at
  0.156). Negative controls: every Q1 and tetrahedral scene completes.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Crash mechanism: the Q1-only cube branch of `extract_boundary_mesh` skips every non-Q1 hex face, emits no displacement-map entry, but still pads `node_positions` to `n_bases + n_faces` rows; the toolkit takes the empty map as identity, so `full_num_vertices = n_bases + n_faces + n_obstacle`; `to_full_dof` returns an oversized contact Hessian; Eigen's dynamic `CwiseBinaryOp::rows()` reports the rhs size, so `FullNLProblem::hessian`'s `+=` silently grows the system Hessian; `BCLagrangianForm::project_hessian` indexes `old_to_new_` past its end (its size assert is compiled out) → `startVec(outer = 2143883344)` | stored lldb backtrace (RB-03 evidence), code reading, unit tests `[rb22]`; the Q3 baseline run shows the same UB degenerate into a corrupt solve instead of a crash | reproduced, contained |
| Silent partial surface: with any other element present the map is non-empty, sizes match, and the scene *runs* with the hex faces missing | baseline `hextet2-q2-default`: exit 0, 48/96 faces, cube through the hex | reproduced, fixed (named error / default proxy) |
| Stage 1 repair: `BoundaryExtractionReport` (skipped faces with element id, primitive id, order, node count, reason; whole-mesh unsupported reason; `describe()`), filled by `extract_boundary_mesh`, `extract_boundary_mesh_sampled` and the new `extract_boundary_mesh_nodal` at every skip site except the intentional non-conforming follower skip; per-face messages raised trace → debug plus one aggregated **warning**; the contact builder (`NonlinearElasticVarForm::build_collision_mesh`, shared by the optimization forms) refuses an incomplete report, an empty surface with boundary faces, and an identity map whose row count differs from the FE node count, each with a named error (`log_and_throw_error`; exit 134 like every PolyFEM named error — `main` does not catch simulation exceptions) | `stage1/q2-default`: warning + error naming 56 elements / order 2 / 27 nodes / 96 faces; unit test "An extraction that skips boundary faces is refused with a named error" (P5 tetrahedra: "skipped 40 of 40 boundary faces … simplex face with 21 owned nodes") | fixed |
| Containment: `FullNLProblem::gradient/hessian` refuse a form whose derivative size differs from the problem (previously silent truncation / silent growth) | code; exercised indirectly (no form now produces a mismatch) | fixed |
| `element_ref_nodes` hardened: resets the layout, guards orders the autogen tables do not carry, and maps 20-node Q2 hexahedra to the serendipity layout (`q_nodes_3d(-2)`), so serendipity elements are no longer silently skipped by the DOF and lattice proxies | baseline `q2s-max_order` aborted; new `q2s-dof` / `q2s-max_order` complete | fixed |
| Candidate (b), DOF-resolution proxy: the upstream hybrid (prism/pyramid) default, factored into `extract_boundary_mesh_nodal` and exposed as `tessellation_type: "dof"`; every proxy vertex is a node (weight-1 row), edges subdivided by the DOFs on them, faces by owned nodes, lattice or convex-sweep triangulation | Q2 column: 74 nodes / 144 faces / 74 exact selectors; serendipity: 56 / 108 / 56; cube: 386 / 768 / 386 selectors, 0 interpolated; closed, edge- and vertex-manifold, Euler 2, one component, no intersections, positive areas; build 2 ms | measured |
| Candidate (a), `max_order` lattice (unchanged code, report added): same counts on Lagrange Q2; serendipity 386 / 768 with **96 interpolated** face-center rows; Q3: 1 exact selector, 163 single entries within 1e-12 of 1 (the Q3 Lagrange basis at thirds is 1−ε, so the RB-03 exact-selector classification does not apply and every contact would take the condensation path) | unit tests (`WARN` line), `matrix-rb22` | measured |
| (a) and (b) are the **same oriented surface** on uniform-order Lagrange hexahedra: identical vertex sets and identical oriented triangle sets (Q1 98/192, Q2 386/768, two-body 52/96); endpoints differ by 1.5e-16 (Q2 semi-implicit), 2.1e-16 (Q2 adaptive), 4.1e-13 (two-body semi-implicit); Newton 33 vs 33 / 44 vs 44; contact counts 136 vs 135 are ordering ties | proxy-set comparison and `endpoint-differences.json` | measured |
| Where they differ: serendipity (b: 290 nodes / 576 faces all selectors, 52 contacts, 34 Newton; a: 386 / 768 with 96 interpolated rows, 117 contacts, 32 Newton; endpoints differ 1.7e-2) and, in principle, Q3+ selector exactness; the two-body scene under **adaptive** stiffness differs 6.2e-3 between the two vertex orderings (139 vs 150 Newton) while semi-implicit differs 4.1e-13 — an ordering sensitivity of the adaptive law on that impact, not a surface difference | `matrix-rb22`, `endpoint-differences.json` | measured, recorded |
| Order mismatches between hexahedra cannot be measured: per-element mixed hex orders hit `assert(false)` (a `// TODO`) in the interface-element stitching of `LagrangeBasis3d::build_bases` (release builds fall through with uninitialized bases). Both proxies are conforming by construction for the supported uniform-order case; the hybrid prism/pyramid conformity is upstream's own default path | code reading | not measurable — documented |
| **Q3 hexahedra are broken upstream, independently of contact**: the basis stores edge/face node positions that are not the geometric images of their reference nodes (4-hex column: 124 of 256 positions mismatch — the two nodes of an edge swapped, the four face-interior nodes permuted), and the 12 face-interior nodes of the 3 shared faces are placed at different physical points by their two elements, i.e. the global Q3 hex basis is discontinuous across faces. Q2 passes both checks (108 positions, 27 shared nodes). Upstream has only linear/quadratic hex basis tests. Consequence for RB-22: both proxies are combinatorially right at Q3 (866 nodes / 1728 faces, closed, Euler 2) but carry 109 zero-area faces; the builder now refuses them with a named error naming the cause instead of "initial solution has intersections" | hidden tests `unit_tests "[q3_hex_defect]"` (fail today by design), `q3-dof` / `q3-max_order` rows | characterized — **blocked**, handed off as RB-23 |
| Selected default (b) routed in `build_collision_mesh` (`has_high_order_hex_boundary` → DOF proxy; spline basis keeps the lattice; Q1 hexahedra keep the centroid split; simplicial meshes untouched); JSON spec option `dof` documented | tests "Q2+ hex contact scenes get the DOF-resolution proxy by default" | fixed (decision implemented) |
| Two-body Q2 impact failures were the upstream example's `lump_mass_matrix: true`: row-sum lumping of quadratic elements gives zero/negative corner masses; Q1 completes, every Q2 variant blew up at the impact step with either proxy, and a consistent mass matrix completes (31 steps, 134 Newton) | `hextet-probe*/`, `hextet2-probe/`, `hextet2-consistent-probe/`; the committed two-body scenes use the consistent mass | not an RB-22 defect — recorded for RB-11 |

Why the change restores the contract: the FE surface handed to the toolkit is
now either complete (every boundary face tessellated, every proxy vertex
mapped) or refused with the element ids, orders, node counts and reason; the
two silent corruption channels (empty map with padded rows; a form of the
wrong size) are closed by explicit checks; and high-order hexahedral
boundaries get a proxy whose rows are exact node selectors, which is the
mapping regime RB-03 validated. Accuracy statement for the user's condition:
the DOF proxy introduces no interpolation error in the displacement map and
coincides with the lattice alternative on Lagrange Q2; like every proxy
available it represents the curved Q2 surface by chords between its nodes
(`tessellation_type: "max_order"` with `sampling_order > 2` refines those
chords at the cost of interpolated rows). No physical accuracy of the
contact response is certified by this record.

Changed settings: none besides the new default routing for meshes with
Q2+/serendipity hexahedral boundary faces (previously a crash or silent
partial surface) and the new `tessellation_type: "dof"` option. Existing
behavior of every other configuration is bit-identical (below). Cache/state:
the report is a per-call value; no static state. Compatibility: scenes that
relied on the silent partial surface now stop with an error.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail / not run |
| --- | --- | --- | --- | --- |
| Baseline reproduction | RB-03 Q2 hex probe inputs, pre-change `b9e3aca4…`; scratch baseline `128bfc29…` on the matrix | crash / silent surface | exit 138 in both stiffness modes; matrix exits −10/−11, corrupt Q3 steps, two-body pass-through with exit 0 (see above) | reproduced |
| Named error | `q2-default`, `q2-default-adaptive`, `q2s-default`, `q3-default`, `hextet2-q2-default` on the stage-1 binary; P5 tetrahedra in the unit test | error naming elements/orders, no crash | all five stop with the error (elements, order, node count, faces, first reason), exit 134; unit test pins the P5 case on the final default | pass |
| New regressions | `unit_tests "[rb22]"` (8 cases): named error, Q2/serendipity default = DOF proxy, DOF and lattice proxies of Q2/Q3/serendipity (topology, geometry, selector rows), extraction report, Q2 node consistency; `[contact_stiffness_mapping][hex][rb22]` RB-03 oracle on Q2 hex contacts under both tessellations (78 selector rows, every contact condensed and equal to the oracle to 1e-9, zero fallbacks) | all pass | **702 assertions / 8 cases**; mapping suite 289 / 5 | 0 |
| Hidden acceptance for the upstream defect | `unit_tests "[q3_hex_defect]"` | pass once the Q3 hex basis is repaired | fails today: 124 position mismatches, 12 shared-node disagreements; the proxy test throws the degenerate-face error | expected fail (documented) |
| Affected suite | RB-03 tag list + `[rb22]`, `[build_collision_proxy]`, `[upsample_mesh]`, seed 1 | no failure | **51 cases / 4,008 assertions** on the final binary (`affected-suite-final/`; the stage-2 binary had 50 / 3,950 before the default-routing test case was added) | 0 |
| Scene matrix, final binary | `tools/rb22/scenes` (21 scenes), single-threaded, `--compare` baseline | former failures are errors or complete; Q2 scenes complete under both stiffness modes; the default equals the `dof` opt-in | `matrix-rb22-final/summary.json`: `q2-default`, `q2-default-adaptive`, `q2s-default`, `hextet2-q2-default` complete (386/768, 290/576, 52/96 all-selector surfaces) and are **bit-identical (Δu = 0.0)** to the stage-2 `dof` runs; `q1-default`, `q1-max_order`, `q2-max_order(-adaptive)`, `hextet2-q1-default`, `hextet2-q2-max_order(-adaptive)` are proxy- and solution-identical to the baseline; `q3-*` stop with the degenerate-face error (866 nodes / 1728 faces / 109 zero-area). Timings cited from the stage-2 `matrix-rb22/` run on an idle machine (same code paths) | pass |
| Q2 hex cube completion | `q2-dof` / `q2-max_order` (semi-implicit), `-adaptive` | 5 saved steps, no error | both: 5 steps, 33 Newton, 136/135 contacts, min distance 1.0004e-4 (dhat 1e-3), 6.0 s solver time; adaptive: 44 Newton, min distance 1.96e-8 (the adaptive law's own behavior, identical on the baseline lattice run) | 0 |
| Serendipity Q2 completion | `q2s-dof` / `q2s-max_order` | complete | 5 steps; 34 / 32 Newton; 52 / 117 contacts | 0 |
| Two-body Q2 hex + P2 tet | `hextet2-q2-{dof,max_order}[-adaptive]`, consistent mass | complete, both bodies in the surface | 31 steps; 52 nodes / 96 faces, two closed components (Euler 4); 134 / 131 Newton; min distance 7.8e-4 | 0 |
| Bit-identity of unchanged paths | `run_ab_smokes.sh`: five public smokes + RB-03 Q1 hex scene, baseline vs final binary, single-threaded; matrix `--compare` | identical proxies and solutions | all six: proxy SHA-256 identical, solution identical, max |Δu| = 0.0; matrix: `q1-default`, `q1-max_order`, `q2-max_order(-adaptive)`, `hextet2-q1-default`, `hextet2-q2-max_order(-adaptive)` proxy and solution identical | pass |
| HDA end-to-end | `hython houdini_HDAs/tests/test_polyfem_hda.py` on the rebuilt binary | PASS | 6/6 PASS, exit 0 (`hda-e2e.log`, `hda-e2e-final.log`); the one "not a descent direction" solver line is present in the RB-03 log too | 0 |
| Physical comparison | — | — | not measured: no physical benchmark exists for these scenes; completion and conformity only | not run |
| Formatting / diff / links | this record, plan row, README, tool README | links resolve, no stray outputs in the tree | checked before commit | pass |

- Numerical termination vs residual: the scene checks are completion and
  conformity checks; the solver's own criteria terminated every completed
  run (gradient norm), no independent residual was measured.
- BC/reaction/gap/inversion/energy metrics: minimum distances above; no
  reaction, inversion or energy accounting was measured.
- Missing: Q3+ hex surfaces (blocked by the basis defect); hex–hex order
  mismatches (unsupported upstream); physical accuracy of high-order
  contact; multi-threaded timing (all timings single-threaded; the two
  baseline `q3-default`/`hextet2` timeouts are bounded kills, not
  measurements).
- Retained failed/partial runs: the first matrix run (`matrix-run1-…`),
  the lumped-mass two-body variants (`hextet-probe*`, `hextet2-probe`),
  the killed baseline `q3-default` (run 1: killed after 180 s; run 2:
  exit −11 after 27.9 s with 4 corrupt steps).
- Test tolerances: exact selector = single entry `== 1.` (RB-03 contract);
  oracle agreement 1e-9 relative (RB-03's); degenerate face =
  `|ab × ac| ≤ 1e-12 · max edge²`; node-position checks 1e-9 absolute on
  meshes of size 20–100.
- Not performed: whole unit suite, private scenes, Teseo, Windows/Linux.

## Publication and reproducibility

- Rebuilt: `PolyFEM_bin`, `unit_tests` (hashes in `inputs/`).
- Committed files: `src/polyfem/io/OutData.{hpp,cpp}`,
  `src/polyfem/varforms/NonlinearElasticVarForm.cpp`,
  `src/polyfem/solver/FullNLProblem.cpp`, `json-specs/input-spec.json`,
  `tests/test_hex_collision_surface.cpp`, `tests/test_contact_stiffness_mapping.cpp`,
  `tests/CMakeLists.txt`, `tools/rb22/` (runner, A/B script, 21 scenes,
  README), this record, the plan row, README rows.
- Remote/branch/commit: implementation, tests, tools and this record in
  `8f76fef69` on `sdast9/polyfem:main` (tested source identity: the
  working tree of that commit; binary hashes in `inputs/`).
- Companion pins: unchanged (`e3c8d3fe`, `5afe3b5d`). HDA: unchanged.
- Remaining working-tree changes: none owned by this item after the commit;
  the other session's README/memory edits were preserved.
- Public fixtures only; the evidence directory is local (VTU outputs, logs)
  and its summaries are small JSON files.

## Next session handoff

- Completed: stages 1–3 for Q2 and serendipity-Q2 hexahedra, including the
  user's default decision. Pending: nothing inside RB-22's scope.
- Status: validated within stated scope; Q3+ blocked by the upstream basis
  defect. Acceptance satisfied for what the basis supports: no crash on any
  Q2+ hex contact scene (named errors or working surfaces), conforming
  surfaces with completed public scenes, unchanged Q1/tet smokes bit-for-bit.
- External prerequisite: **RB-23 (new row)** — repair the Q3+ hexahedral
  node bookkeeping in `LagrangeBasis3d` (`hex_local_to_global` edge/face node
  order vs `q_nodes_3d`), then run `unit_tests "[q3_hex_defect]"`; and the
  mixed-order hex stitching `TODO`. Separate observation for RB-11:
  `lump_mass_matrix` with quadratic elements should be refused or lumped
  consistently (HRZ) — a row-sum lump is indefinite.
- Next command: `python3 tools/rb22/run_matrix.py --binary build/PolyFEM_bin --output <fresh>`;
  next eligible item per the plan: RB-05, then RB-09/RB-10; RB-23 when
  authorized.
