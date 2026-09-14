# RB-11 — Geometry, material and input validation

**Follow-up review (2026-09-14):** the published envelope matrix is reconciled
and supported by the saved evidence. Stage 1 remains validated within scope.
Two bounded implementation/reporting follow-ups remain open: BDF force-output
normalization, and acceptance of missing Newton/determinant evidence. See the
[follow-up review and completion instructions](rb-11-followup-review-20260914.md).

**Latest continuation guidance (2026-09-13 evening):** read the
[Stage 1 / Stage 2 review](rb-11-stage-review-20260913.md) before resuming.
It supersedes the envelope handoff's run-completion claims and identifies
the reproduced quasistatic force-output defect and bounded remaining work.
*(2026-09-14: its completion sequence was carried out; see the
[envelope stage section](#envelope-stage--characterized-limits-documented-2026-09-14).)*

> **Independent review, 2026-09-13:** [read the continuation guidance](rb-11-review-guidance.md)
> before resuming. It reproduces a valid 2D material rejected by the new rule,
> an unhandled zero constant fibre, a false CCD notice, and fixture/acceptance
> gaps. The historical results below are retained, but the handoff's claim
> that only two edits and final re-validation remain is superseded. The
> implementation needs these corrections before publication.

Date: 2026-09-13
Status: validated within stated scope (input-validation stage). The
independent review of 2026-09-13 ([rb-11-review-guidance.md](rb-11-review-guidance.md))
found four groups of defects in the first implementation (a 2D domain rule,
an unclosed zero-fibre case, a wrong CCD notice, a fixture and an oracle that
did not enforce their contracts); all are corrected and the frozen candidate
(identity below) passes the matrix (93/93 in verify mode), the new
regression, the affected selection with the assembler/cache/derivative tags,
the smokes, the HDA tests and the full suite (320/323, the three remaining
failures pre-existing and attributed). The physical envelope
characterisation (locking/conditioning of nearly incompressible, anisotropic,
thin and distorted elements against RB-09-style references) is **not
started** and stays a separate stage
Selected stage: inventory of the input checks, reproduction of the invalid
cases the plan requires on a public probe matrix, bounded named-error repairs
with regression tests, and the two RB-09 unit observations as startup notices

## Contract and authorization

- User-selected item: "Let's address RB-11" (2026-09-13). No explicit scope or
  policy decision was given beyond the plan section; no HDA change was made.
- Invariant: an invalid supported input stops early with a named error that
  carries path/element/material context and leaves no accepted simulation
  output; a valid input (open or codimensional obstacles, nonmanifold but
  connected FE meshes, nonuniform materials, auxetic ν, massless quasistatics)
  keeps running; per-element values and reported units match the source input.
  Numerical termination, geometric feasibility and physical accuracy stay
  separate outcomes; a completed scene is not an equilibrium certificate.
- Dependencies read: [robustness plan](robustness-plan.md) (RB-11 section,
  boundaries, decision register), the [PF invariants](correctness-remediation-plan.md),
  RB-09's [unit observations](rb-09-validation.md), RB-05's `sweep_and_prune`
  note and RB-22's `lump_mass_matrix` note. Nothing in RB-13's estimator
  assumptions is rejected by these checks (see the applicability table).
- Exclusions / not authorised: no material, element, quadrature or friction
  default changed; no stabilisation; no CCD change (the absolute clearance is
  reported, not altered); no acceptance criterion; no HDA source change (the
  HDA test gap below is recorded for a later publication); the physical
  envelope stage is pending.
- Judgment calls taken without a user decision, open to reversal:
  1. two Dirichlet boundaries prescribing **different values for the same
     component of a shared node** are refused (the previous behaviour was
     last-write-wins in element order, not a rule a user can rely on);
     identical values and different components stay accepted;
  2. an **indefinite row-sum lumped mass** (quadratic corners) is a
     **warning**, not an error: upstream's `contact/examples/common.json`
     lumps every contact example, quadratic bodies included, and
     `higher-order/golf-ball-P2.json` matches its golden that way; the RB-22
     Q2 hexahedral impact that failed under lumping stays the documented
     consequence;
  3. a **body without a material** is an error (the placeholder-parameter or
     out-of-range-read alternative is never what the user meant), which
     changed one upstream unit test (`assembler material dispatch`) to expect
     the error;
  4. per-element value lists/files are bound **by length** (both the fork's
     documented global contract and upstream's body-local one stay valid
     inputs), rather than choosing one of the two contracts;
  5. remeshing with per-element value/fibre files is **refused** (a named
     diagnosis at startup and at the first local relaxation) rather than
     transferred: the local patches re-bind by patch-local ids and no
     transfer fixture exists; the lifecycle stays unsupported.

## Baseline and reproduction

- PolyFEM `a5aa87ea0` on `main`, clean tree; IPC Toolkit `bb795446`
  (`semi-implicit-stiffness`, local override), PolySolve `ee5b296a6`
  (`iteration-callback`, local override), pins matching.
- Build `RelWithDebInfo` (`-O2 -g -DNDEBUG`), Apple clang 21.0.0, arm64,
  `Eigen::SimplicialLDLT`; the matrix runs single-threaded, the smokes with the
  default thread count. `POLYFEM_WITH_MISO=OFF`.
- Baseline binary `PolyFEM_bin` sha256 `2f2af0b55ac95a68…5fa` (built
  2026-09-13 09:55 from `a5aa87ea0`), preserved as
  `outputs/rb-11/20260913T140845Z/baseline/PolyFEM_bin`.
- Evidence `outputs/rb-11/20260913T140845Z/`: `baseline-identity.txt`,
  `matrix-baseline-v3/` (baseline binary, final case set),
  `matrix-candidate-final/` (the committed code; `matrix-candidate-2/` and
  `-3/` are the same 73/73 on intermediate builds), `ab-table.md` (baseline
  v3 vs candidate 2), earlier runs `matrix-baseline/`, `matrix-baseline-v2/`,
  `matrix-candidate-1/` (kept; two fixture defects of the first run — the
  two-body value files written in generator instead of Gmsh element order,
  and contact controls whose slab the cube never reached — were corrected
  before v2; candidate-1's 69/72 were fixture/expectation corrections, see
  below), `unit-input-validation-final/`, `affected-suite-final/`,
  `smokes-final/`, `smokes-baseline/`, `smokes-st/`, `hda-final/`,
  `full-suite/` (earlier copies of these on intermediate builds are kept
  without the `-final` suffix; `full-suite-aborted-prefinal/` is a run
  stopped after 20 minutes, without failures, when the last four edits
  — obstacle "all" exemption, shear-only laws, fibre parse message, nodal
  file guard — made a rerun on the final binary necessary).
- Probe matrix: `tools/rb11/` (`fixtures.py`, `cases.py`, `run_matrix.py`,
  [README](../tools/rb11/README.md)); 73 cases at the first candidate, 93
  after the review additions (52 expected named failures, 38 accepted, 3
  accepted with a notice: declared millimetres, zero density in a transient
  run, lumped quadratic mass), each with its contract phrase, intended step
  count and checks; every case is a
  synthetic public fixture (2×2×2 tet cube, open slab). Command per case:
  `PolyFEM_bin --json scene.json -o out --log_level debug --max_threads 1`
  from the case directory (`command.txt`, `run.log`).
- Reproduced on the baseline: **40/73** cases behaved as the plan requires.
  The 33 others: 5 segfaults (missing mesh file, Gmsh element on a
  nonexistent node tag, MEDIT index out of range, a tet listing a vertex
  twice, a body without a material), 2 hangs (`dt = 0` — 2,062 VTU files in
  120 s; `dhat = 0` — 20 stall restarts), 16 silent completions with output
  (duplicate element, obstacle face on vertex 99 of 4, duplicate and zero-area
  obstacle faces, `E = 0`, `ρ < 0`, per-element files with more rows than
  elements, materials for an id the mesh does not have, HGO `k2 = 0`,
  `κ = 0.6`, two Dirichlet entries for one id,
  conflicting values on shared nodes, an obstacle id prescribed in both lists,
  `tend < t0` running with `dt = −0.25`), 6 late solver failures with the step-0
  output already written and no material context (`E < 0`, `ν = 0.5`,
  `ν = 0.7`, `ν = −1.5` after 500 Newton iterations, `E = 0/0`, `E = 1/0`),
  2 accepted with the wrong content (a global-length per-element `E` file
  read body-locally on the second body — its elements got body 1's rows;
  `broad_phase: sweep_and_prune` running the hash grid), 3 accepted without the
  envelope notice (declared millimetres; zero density in a transient run;
  row-sum lumping of P2 elements — counted among the 16 silent completions
  in the runs recorded before the lumping check became a warning).
- Negative controls: the 24 valid cases completed on both binaries; the
  candidate is bit-identical to the baseline on the friction smoke
  single-threaded (`smokes-st/`, solution sha256 `24d44024…`), and within
  threading noise with the default thread count (≤ 3e-16 frictionless;
  friction 3.5e-15 on the final build, 1.7e-4 on an intermediate build — the
  known run-to-run band of that scene).

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Missing/unreadable FE mesh file: `Mesh::create` logs and returns null, `read_fem_mesh` dereferenced it | `g1-missing-mesh-file`, `g1-medit-index-out-of-range` (geogram reports "tet # 5 references an invalid vertex: 999" then null) — exit −11 | fixed: named error naming the path and geometry entry (`GeometryReader.cpp`) |
| Gmsh element on a nonexistent node tag (or tag 0): assert-only, compiled out; index −1 or garbage reached the builder | `g1-msh-unknown-node-tag` segfault in the basis; `g1-msh-node-tag-zero` misreported as "Invalid number of vertices 2" | fixed: `MshReader` refuses with the element and node tag (`node_index_of`) |
| Cell matrices with out-of-range/negative/repeated indices or too few corners: assert-only | `g1-repeated-vertex-in-tet` segfault | fixed: `Mesh::create(V, cells)` validates every row (`Mesh.cpp`) |
| Duplicate elements assemble the material twice, silently | `g1-duplicate-element` completed with 3 VTU | fixed: refused in `Mesh::create` (matrix, any ordering) and `validate_rest_elements` (loaded meshes, `MeshUtils`) |
| Obstacle OBJ face on vertex 99 of 4 reached the collision mesh as a garbage vertex; duplicate and zero-area obstacle faces/edges accepted | `g1-obstacle-bad-face-index`, `-duplicate-face`, `g1-degenerate-obstacle-face` completed | fixed: `validate_surface_mesh` after `read_surface_mesh` (index range, duplicates, zero area/length, codim points) |
| Valid nonmanifold/open/codimensional inputs must stay accepted | `g1-pinched-fe-mesh`, `g1-obstacle-tjunction`, `-edges-only`, `-points-only`, `g1-obstacle-extract-volume` (the reader's documented alias for `surface` on obstacles) | not rejected (controls pass on both binaries) |
| `time.dt = 0`, `tend < t0`, `time_steps ≤ 0`: assert-only | `g6-dt-zero` hang; `g6-tend-before-t0` ran with `dt = −0.25` | fixed: named errors in `State::init_time` |
| `contact.dhat = 0` (spec minimum is inclusive): the semi-implicit trial cap divides by it, "inf supports", 20 restarts | `g6-dhat-zero` timeout | fixed: `dhat` must be positive and finite when contact is enabled (`State::init`) |
| `broad_phase: sweep_and_prune` / `SAP` in the spec but not in the enum map → first entry (`hash_grid`) silently (RB-05 observation) | `g6-broad-phase-sap` log "broad phase HashGrid" | fixed: enum entries added (the toolkit implements SAP); `ContactForm::is_known_broad_phase_name` + startup check; regression asserts every spec option is mapped |
| A body without a material: warning, then placeholder parameters or an out-of-range parameter read | `g3-missing-material-for-body` segfault; `g3-material-id-unmatched` ran with the constructor's defaults | fixed: named error listing missing and given ids; an id matching no body is a warning (`Assembler::set_materials`) |
| Nonfinite / out-of-range parameters reach the solve and fail late or not at all | `g3-E-*`, `g3-nu-*`, `g3-rho-negative-transient`, `g3-hgo-k2-zero`, `g3-hgo-dispersion-kappa-*` | fixed: `validate_material_parameters` after `set_materials` — a law-keyed contract evaluated at every element barycentre at `t0`: everything finite; Lamé laws `E > 0`, `μ > 0`, `λ + 2μ/d > 0`, `−1 < ν < ½` (3D) / `< 1` (2D); shear-only laws `μ > 0`; HGO `k1 ≥ 0`, `k2 > 0`, `0 ≤ κ ≤ 1/d`, nonzero fibre of the mesh dimension; `ρ ≥ 0` (`ρ = 0` in a transient run is a warning); unlisted laws finiteness only; MultiModel elements against their own model; values paired within one composite child. Sampled startup checks: expressions are evaluated at the barycentres at `t0` only |
| A constant or expression fibre direction of zero length was accepted (only per-element files were checked) | review case `zero-constant-fiber`; `g4-fiber-constant-zero`, `g4-fiber-expression-zero`, `g4-fiber-wrong-dimension` | fixed: numeric zero/nonfinite vectors and dimension mismatches refused at load (`FiberDirection`), zero evaluated directions refused by the sweep; normalisation semantics unchanged (`HGODispersion` normalises; `HGOFiber`/`ActiveFiber` use the length as given, `g4-fiber-constant-nonunit`, `g4-fiber-expression-valid`) |
| Shear-only laws (`IsochoricNeoHookean`, `IncompressibleLinearElasticity*`) accept `ν = ½`: λ is infinite and the back-computed `E`/`ν` are not finite, but only μ acts (the pressure block uses `1/λ = 0`) | code inspection; `[input_validation]` "shear-only laws accept the incompressible limit" | exempted from the `E`/`ν`/`λ` rules; `μ > 0` still enforced |
| Per-element scalar files in a material array bind **body-locally** since upstream `c6796916f` (#333, 2026-08-25), fibre files globally; the HDA writes global-length files → every non-first body reads the wrong rows | `g3-E-file-two-bodies-global`: body 2's top vertices read `1e6` (body 1's rows) instead of `3e6`; `g4-fiber-file-two-bodies-global` correct | fixed: one contract for scalars and fibres — rows == elements of the mesh → global id; rows == elements of the body → body-local; anything else refused at load (`ExpressionValue::bind_per_element`, `FiberDirection`, `MATERIAL_ELEMENT_COUNTS`); upstream's body-local test still passes |
| Per-element file longer than the mesh accepted silently; a short one failed only at the first assembly | `g3-E-file-long`, `g4-fiber-file-long` completed; `g3-E-file-short`, `g4-fiber-file-short` failed in the solve phase | fixed by the length contract (load time) |
| A list of expressions as a material value evaluated to 0 silently | code inspection (`ExpressionValue::operator()` never evaluates `mat_expr_` without `time_reference`) | fixed: refused at bind and at evaluation |
| A value-file path with a typo was reported as an invalid expression | `g3-E-file-missing` | message: "is not an existing value file (resolved to …) and is not a valid expression" |
| `lump_mass_matrix` with P2 elements: row-sum lumping gives 81 of the 375 mass-carrying DOFs a nonpositive nodal mass (RB-22 observation) | `g3-lump-mass-p2-transient` completed silently | **warning** (judgment call 2): `check_lumped_mass` on the consistent matrix before lumping in the four varforms names the count and the smallest row sum; P1 lumping and the consistent P2 mass are silent. Two iterations behind this: a first version was an error on every zero row and refused 8 upstream scenes whose obstacle vertices / codimensional points carry no mass by construction (caught by the first full-suite run, `full-suite-aborted-lumping-false-positive/`); the second was an error on mass-carrying rows only and refused upstream's `golf-ball-P2.json`, which lumps a quadratic body through `common.json` and matches its golden (caught by the completed full-suite run) — hence the warning |
| Two `dirichlet_boundary` entries for one id: only the first acts; an id after an "all" entry never acts | `g5-duplicate-dirichlet-id` | fixed: named error at parse (`GenericTensorProblem::set_parameters`) |
| Conflicting prescribed motion on shared nodes: last write in element order wins | `g5-conflicting-shared-nodes` (+x face vs top edge) | fixed: `sample_bc` refuses two different values for one DOF from different tags (rel. 1e-9 + 1e-12·bbox), naming node, position, component, tags and values; same value from two tags and different components stay accepted (`g5-consistent-shared-nodes`, `g5-component-split-shared-nodes`); the `lsq` bc method is not covered |
| Obstacle id in both `obstacle_displacements` and `dirichlet_boundary` with different values: the former won silently | `g5-obstacle-displacement-conflict` | fixed: named error when both name the id explicitly; an "all" Dirichlet entry stays the FE default and does not conflict |
| RB-09 unit observation 1: the nonlinear tolerance `grad_norm·F0·L^1.5` keeps the SI default `F0 = 1e4` under declared non-SI units | `g2-units-mm-raw` | notice: warning when the base units are not m/kg/s and `characteristic_force_density` is the default (`NonlinearElasticVarForm::build_basis`); no tolerance change |
| RB-09 unit observation 2: IPC's Tight-Inclusion CCD caps one step's extra clearance at an absolute 1e-4 length units (`d_min + min((1−c)(d_0 − d_min), 1e-4)`, `tight_inclusion_ccd.cpp`, upstream) — the first version of this notice wrongly called it a minimum gap and claimed contact could not reach a smaller `d̂` (review counterexample: `d̂ = 1e-5 m` settles inside the band) | `g2-units-mm-raw` (ratio 1e-4 `d̂`), `ctl-contact` (0.1 `d̂`), review case `ccd-small-band` | notice: info line naming the strategy and the cap; warning below `1e-4/d̂ < 1e-2` reporting only RB-09's measured unit sensitivity; CCD unchanged (plan: preserve CCD) |
| Fibre VTK with a nan token was reported as "ended early" | `g4-fiber-nonfinite` | message names the vector; nonfinite vectors refused |
| Initial intersections: named error exists; only `intersection.obj` locates them | `g1-initial-intersection-*` | unchanged (named failure before any output); the intersecting primitives are not listed — limit |
| Inverted and zero-volume rest elements: named error exists ("element i is flipped") | `g1-inverted-tet`, `g1-degenerate-tet` | unchanged; a zero-volume element reads "flipped" — wording limit |
| Unit dimension mismatches (geometry/material/time) | `g2-*-unit-dimension`, `g2-geometry-unit-unknown` | unchanged: named failures already (an unknown unit string parses to an incompatible unit and fails the conversion) |

Why the changes restore the contract: each check turns an undefined or
silently wrong state into a refusal at the point where the input is first
interpreted (mesh reader, material binding, BC sampling, `State::init`) and
before any output is written; none changes a coefficient, a tolerance, a
default or a numerical path — the smokes are bit-identical single-threaded.
The per-element binding contract is decided by length, so both the fork's
documented global contract (Houdini export, `SPEC_per_element_materials.md`)
and upstream's body-local one remain valid inputs and are told apart without
ambiguity (a multi-body mesh has strictly more elements than any body). No
new model, estimator or acceptance criterion is introduced.

Applicability of the RB-13 assumptions (recorded, not enforced): a singular
or indefinite tangent (buckling, free-body quasistatics, material softening)
is a valid input — no RB-11 check looks at `H`; the semi-implicit
coefficient's nonpositive-curvature fallback (RB-18 F2: previous κ →
|wᵀHw| → max|H|/d̂²) is the selected handling of the unreliable-estimator
case. Prescribed and free modes, anisotropy (fibre models), heterogeneous and
per-element materials, near-incompressibility up to `ν < ½`, multiple coupled
contacts and the RB-03 collision/FEM mapping are valid inputs; `ν = ½`
exactly is refused because the compressible laws divide by `1 − 2ν`. The
RB-13/RB-14 estimators are closed evidence; nothing here re-opens them.

## Review corrections (2026-09-13)

An independent review ([rb-11-review-guidance.md](rb-11-review-guidance.md),
evidence `outputs/rb-11/20260913T191847Z-review/`) ran five small
configurations on the preserved baseline and the first candidate and
reproduced four groups of defects in that candidate. Each was verified
against the code before acting:

| Review finding | Verified | Correction |
| --- | --- | --- |
| The `kappa` rule hard-coded `[0, 1/3]`; `HGODispersion` uses `E4 = kappa I1 + (1 - d kappa) I4 - 1` with domain `[0, 1/d]`, so a valid 2D `kappa = 0.4` was refused | `HGODispersion.hpp:15,40`; review case `hgo2d-04` | the validation is now a law-keyed contract (`Assembler.hpp` doc): rules per constitutive law and mesh dimension (`kappa ∈ [0, 1/d]`), values paired within one child of a composite (`by_model`), everything unlisted checked for finiteness only; regressions: 2D 0.4/0.5 accepted, 0.6 refused, plain and composite; 3D endpoint 1/3 and −0.1 |
| A constant or expression fibre direction `[0,0,0]` was accepted (only per-element files were checked); the fibre term silently vanished | review case `zero-constant-fiber`; `GenericFiber::I4Bar_generic` normalises a zero vector to zero | `FiberDirection::add_multimaterial` refuses a numeric zero/nonfinite vector and a dimension mismatch (both were asserts); the sweep refuses a zero evaluated direction at the barycentres (expressions); normalisation semantics kept as they are and documented: `HGODispersion` normalises, `HGOFiber`/`ActiveFiber` use the vector's length as given (a non-unit vector is a pre-stretch there, not an error) |
| The CCD notice claimed contact "can never reach the band" below `dhat = 1e-4`; the toolkit's `1e-4` is a cap on one step's *extra* clearance `d_min + min((1-c)(d_0 - d_min), 1e-4)`, not a floor; the review's `dhat = 1e-5 m` run settled inside the band | `tight_inclusion_ccd.cpp:45-48`; review case `ccd-small-band` | the notice states the actual rule and names the strategy; the warning below `1e-4/dhat < 1e-2` only reports RB-09's measured unit sensitivity ("measured limit, not a prescription"); the `F0` note is qualified to the norm rules of `NLProblem::grad_norm_rescaling` |
| The "T-junction" obstacle had a maximum edge degree of 2 (a bent open surface) | `fixtures.edge_degrees` on the old faces | the fixture and the Catch section now share edge `(0,1)` between the slab and two fins; both assert the incidence (degree 3) independently of the run |
| The runner's oracle accepted an "accepted" record without steps, a "notice" record without the notice and a named failure with an unrelated error | review `oracle` probe | every named failure carries its contract phrase (`expect_error`, checked in the log), every accepted run its intended step count and its `check.json`; `--verify` exits nonzero on a mismatch or an empty selection; `--self-test` feeds hollow records and fails if any is accepted (kept in the tool) |
| The per-element transfer check (constant per body) established only the global/body-local repair, not element order | review remark | per-element checks match every output element to its fixture element by centroid and compare its own row: unique values on one body, unequal bodies (8 + 40 tets), two geometry meshes (96 rows), body-local files with unique values, unique fibre directions; `g3-E-file-permuted-oracle` declares that the oracle must *fail* on a permuted file (`expected_status: fail`) and does (max relative error 0.40) |
| Length-based binding does not cover remeshing, where local patches re-bind by patch-local ids | review remark; the record already noted the length rule fails there | remeshing (legacy state) with per-element value/fibre files is now refused with a named diagnosis at startup (`legacy::State::init`) and at the first local relaxation (`LocalRelaxationData`), via `materials_use_per_element_files`; the lifecycle stays unsupported, not silently misbound |
| The lumping message claimed row-sum lumping is positive *only* for linear elements and did not separate negative/zero/nonfinite sums from structural zeros | review remark | the warning counts negative, zero and nonfinite row sums, reports the all-zero obstacle/codimensional rows separately, says some higher-order bases have nonpositive corner sums, and places the regime outside the measured envelope |
| Remote `main` had moved (`0465e3a28`, `a327e2932`: CI portability documentation and repairs) | `git fetch` | integrated by fast-forward under the staged work (disjoint files); the frozen candidate is built on `a327e2932` |

Frozen candidate (`review-fixes/final/identity.txt`): PolyFEM `a327e2932` plus
the staged patch (sha256 `693e5704…`), `PolyFEM_bin` `27e0eb3d…`,
`unit_tests` `e9b105fe…`, fixture/runner hashes recorded.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail / not run |
| --- | --- | --- | --- | --- |
| Baseline reproduction | `tools/rb11` matrix, baseline binary | the plan's required cases fail as described | 40/73 as required; 5 crashes, 2 hangs, 16 silent, 6 late, 4 wrong/unnoticed (`ab-table.md`) | reproduced |
| Candidate matrix (first candidate) | 73 cases | invalid → named failure without output; valid → completed with matching fields/notices | 73/73 (`matrix-candidate-final/summary.md`) — superseded by the frozen candidate below | pass |
| **Frozen candidate matrix** | 93 cases (the 73 plus the review additions), `--verify` (`review-fixes/final/matrix/`) | every case meets its contract: named failures carry their phrase and write nothing; accepted runs save 3 steps and pass their per-element / incidence / notice checks; the permuted-file oracle control fails as declared | **93/93**, verify exit 0; oracle `--self-test` pass | pass |
| New regression | `unit_tests "[input_validation]"` (`review-fixes/final/`) | all pass | 17 cases / 160 assertions | pass (exit 0) |
| Affected selection + material/cache/derivative tags | `[contact_floor_retired],[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives,[input_validation],[assembler],[material_cache],[form_derivatives]` (`review-fixes/final/affected-suite.log`) | all pass | 100 cases / 4,771,646 assertions | pass (exit 0) |
| Full `unit_tests` (frozen candidate) | `unit_tests` `e9b105fe…`, `review-fixes/final/full-suite/run.log`, 18:45–19:32 local | the three pre-existing failures and the known one only | **323 cases: 320 passed, 3 failed (4 of 5,155,530 assertions)** — `contact_2d`: `gcp-contact/cube-on-floor` (known) and `large-ratios/large-mass-ratio.json` (pre-existing, RB-10 default); `contact_3d`: `large-ratios/large-mass-ratio.json` (same cause); `standard`: `multi-material/stretch-cubes.json` (pre-existing RB-22 named error). `golf-ball-P2` and `assembler material dispatch` pass | pass: no failure attributable to RB-11 |
| Full `unit_tests` (first candidate) | `unit_tests` `04fed1f8…` (the error version of the lumping check), `full-suite/run.log`, 14:19–15:06 local | only the known cube-on-floor golden failure | **321 cases: 317 passed, 4 failed (6 of 5,155,482 assertions)** — `contact_2d`: `gcp-contact/cube-on-floor` (known) and `large-ratios/large-mass-ratio.json` (pre-existing, RB-10 default, see below); `contact_3d`: `large-ratios/large-mass-ratio.json` (pre-existing: baseline binary L2 error 0.17272674248190 vs the suite's 0.17272674248175, stored 0.17766395877815 — the same RB-10 cause) and `higher-order/golf-ball-P2.json` (my lumping error, since downgraded to a warning); `standard`: `multi-material/stretch-cubes.json` (pre-existing: the RB-22 "incomplete collision surface" named error, identical on the baseline binary, `golden-check/baseline-stretch-cubes/`); `[assembler]` `assembler material dispatch` (upstream test with a body lacking a material; updated to expect the new error) | 3 pre-existing + 1 known, 2 attributable to RB-11 and both corrected in the working tree; **rerun on the corrected build pending** |
| Affected smokes | the five public scenes, frozen candidate (`review-fixes/final/smokes/`) | 4 intended steps, exit 0, 0 error lines | 5/5 exit 0, 5 VTU each (step 0–4), 0 error lines; endpoints vs the baseline binary ≤ 1e-15 (an earlier multithreaded run of the friction scene differed by 1.7e-4, its run-to-run band) | pass |
| Numerical identity | friction smoke, `--max_threads 1`, baseline vs frozen candidate | bit-identical | identical (`smokes-st/baseline`, `review-fixes/final/smokes-st`, solution sha256 `24d44024…`) | pass |
| Lumped P2 reference | `contact/examples/3D/higher-order/golf-ball-P2.json` (lumps a quadratic body through `common.json`) | runs with the warning; its golden is judged by the full suite | completes; warning "24 of the 5361 DOFs that carry mass … 24 negative, 0 zero, 0 nonfinite" (`review-fixes/final/golf-ball-P2/`) | pass (golden: full suite) |
| HDA end-to-end | `tests/test_polyfem_hda.py`, `tests/test_polyfem_materials.py`, `tests/test_remesh_hda.py` with the frozen candidate (`review-fixes/final/hda/`) | all PASS | all exit 0; materials 63 PASS lines incl. "per-element fibers reached the solver on the right elements 710/710"; the E2E and remesh parameter round trips PASS (the remesh test is a parameter round trip, not an execution of remeshing with per-element data) | pass |
| Formatting / diff / links | `git diff --check`; record links | clean | clean | pass |

- Numerical termination versus residual: unchanged by this item; not
  re-measured (see RB-09/RB-04).
- Metrics: per-element `E` and `fiber_direction` at the top/bottom vertices
  of the two-body fixtures (exact per-point output) match the source files;
  no physical quantity is claimed beyond that.
- Missing quantities: the physical envelope of nearly incompressible,
  anisotropic, thin and distorted elements (locking, conditioning, resolution
  needs) is **not measured**; constitutive derivative and rigid-motion
  invariance checks on a small fixture are **not run** in this stage;
  remeshing with per-element files is refused rather than transferred
  (judgment call 5; before: misindexed silently) and no remeshing transfer
  fixture exists; the `lsq` boundary method has no conflict
  detection; the intersecting primitives of an initial intersection are not
  listed.
- Retained runs: `matrix-baseline/` (72 cases, two fixture defects on my
  side) and `matrix-baseline-v2/`, `matrix-candidate-1/` (69/72 before the
  fixture/expectation corrections: the "consistent" BC fixture was itself
  inconsistent at the bottom edge — the check caught it; `rho = 0` transient
  reclassified to a notice; SAP reclassified to accepted) are kept.
- Test tolerances: the shared-node conflict tolerance is 1e-9 relative plus
  1e-12 of the bounding-box diagonal; the zero-area face test is
  `|ab × ac| ≤ 1e-12 · max edge²` (the RB-22 rule); no golden changed.
- Not performed: private scenes, Teseo, other platforms/compilers, the
  HDA-side regression for a per-element scalar on a non-first body (the
  existing HDA test authors kappa on subdomain 1 only, so it could not detect
  the misindexing; adding it is an HDA publication).
- Unrelated baseline failure found by the full suite, not concealed: the
  `contact_2d` golden of `contact/examples/2D/large-ratios/large-mass-ratio.json`
  (friction μ = 0.1) fails by 0.5 % on the **baseline** binary as well
  (`golden-check/`: L2 error 0.20349977573650 with both binaries against the
  stored 0.20446614217846); with `solver/contact/friction_iterations: 1` the
  baseline binary reproduces the stored value to 5e-12. It is the RB-10
  default decision (`friction_iterations: 2`, 2026-09-13) reaching the
  upstream friction goldens, not RB-11; no golden was touched. The 3D
  `large-mass-ratio.json` (μ = 0.5) fails the same way on both binaries
  (2.8 %). `multi-material/stretch-cubes.json` stops with RB-22's named
  error on both binaries (an upstream mixed-order scene whose hexahedral
  faces the default extraction skips — an RB-22/RB-23 follow-up). The
  full-suite row above lists every failure with its attribution.

## Publication and reproducibility

- Rebuilt targets: `PolyFEM_bin` `27e0eb3d…` and `unit_tests` `e9b105fe…`
  from `a327e2932` plus the RB-11 patch (sha256 `693e5704…`), the frozen
  candidate that every validation row above reports; the tree committed as
  `75b4d284d` is that patch.
- Committed files and remote/branch/commit: `75b4d284d` on
  `sdast9/polyfem:main` (42 files: 35 source files, `tests/test_input_validation.cpp`,
  `tests/CMakeLists.txt`, `tests/test_assembler.cpp`, `tools/rb11/`, this
  record, the review guidance, the plan, the RB-10 note), on top of the
  integrated remote `a327e2932`. This documentation commit follows.
- Companion pins unchanged (IPC `bb795446`, PolySolve `ee5b296a6`); no HDA
  change, so no HDA publication.
- Remaining working-tree changes: none after the commit.
- Evidence is local (`outputs/rb-11/20260913T140845Z/`, ~1 GB with the
  dt = 0 output); the fixtures are generated by the checked-in tools.

## Procedure and progress log

1. **Contract** — read the plan section, its boundaries and decision
   register, the PF invariants, and the RB-05 (`sweep_and_prune`), RB-22
   (`lump_mass_matrix`) and RB-09 (unit dependence) observations handed to
   RB-11. Repo state, effective overrides, build flags and binary hashes
   recorded in `baseline-identity.txt`; the baseline binary copied.
2. **Inventory** — read the input path end to end: `GeometryReader`,
   `Mesh::create`, `MshReader`, `read_surface_mesh`, `Assembler::set_materials`,
   `MatParams`/`ExpressionValue`, `GenericTensorProblem::set_parameters`,
   `RhsAssembler::sample_bc`, `State::init`, the input spec. Structural
   finding: the RelWithDebInfo build defines `NDEBUG`, so every `assert()` on
   the input path is dead code; `Mesh::create` returns null on a missing file
   and the caller dereferences it; upstream `#333` (2026-08-25) made scalar
   per-element files body-local while fibre files stayed global.
3. **Probe matrix** — `tools/rb11/` generates 73 public synthetic cases (a
   2×2×2 tet cube, an open slab) with an expectation each and a valid control
   beside every invalid input; the runner classifies crash / hang / silent /
   named failure and checks exported per-element fields and log notices.
4. **Reproduction on the baseline binary** — 40/73 as required (two fixture
   defects of my own were corrected first: two-body files written in
   generator instead of Gmsh element order, and contact controls whose slab
   the cube never reached; kept as `matrix-baseline/`, `-v2/`).
5. **Repairs** — refusals at the point of interpretation only (readers,
   `Mesh::create`, obstacle validation, material binding and range
   validation, BC parsing/sampling, `State::init`), the length-based
   binding contract, the SAP enum entries, startup notices; no coefficient,
   tolerance, default or numerical path changed.
6. **Validation** — matrix 73/73 on every candidate build; `[input_validation]`
   15 cases / 121 assertions; affected selection 39 cases / 1,694 assertions;
   five smokes exit 0 with the friction smoke bit-identical single-threaded;
   HDA end-to-end, materials and remesh tests; one full-suite run (321 cases,
   317 passed) whose four failures are attributed above.
7. **Corrections the full suite forced** — the lumped-mass check: first
   flagged the massless obstacle DOFs (fixed), then refused upstream's lumped
   quadratic golf ball (downgraded to a warning); the upstream dispatch test
   updated for the body-without-material error. These two edits are the
   pending rebuild/re-run.

Dated log (local time, 2026-09-13):

- **10:08** — evidence directory created, baseline identity and binary
  preserved.
- **12:50–13:10** — first matrix run (72 cases) against the baseline
  binary: 38/72; two fixture defects found and corrected; baseline v2 40/72.
- **13:20–13:35** — repairs implemented, `PolyFEM_bin` rebuilt; candidate-1
  69/72 (three expectation/fixture corrections: the "consistent" BC fixture
  conflicted at the bottom edge, `rho = 0` transient reclassified to a
  notice, SAP reclassified to accepted; one misspelt-name case added);
  candidate-2 73/73; baseline rerun on the final case set 40/73.
- **13:40–13:55** — `[input_validation]` written and passing; affected
  selection 38/1,685; smokes; single-threaded identity; HDA materials and
  E2E tests pass.
- **14:05–14:20** — review corrections (obstacle "all" entries exempt from
  the conflict check, shear-only laws exempt from the compressible-parameter
  rules, fibre parse message, nodal-file guard in the duplicate-id check);
  rebuilt; a full-suite run on the earlier build stopped
  (`full-suite-aborted-prefinal/`); the next full-suite run refused the
  massless obstacle DOFs — lumping rule corrected
  (`full-suite-aborted-lumping-false-positive/`); matrix 73/73,
  `[input_validation]` 15/121, affected 39/1,694, smokes, identity and HDA
  tests re-run on that build (`*-final/`).
- **14:19–15:06** — full suite on build `04fed1f8…`: 321 cases, 317
  passed, 4 failed; attribution by rerunning the failing scenes on the
  baseline binary (`golden-check/`): `large-mass-ratio` 2D/3D and
  `stretch-cubes` pre-existing, `golf-ball-P2` and the dispatch test RB-11's
  — lumping check downgraded to a warning, test updated (not yet rebuilt).
- **15:15** — this record brought up to date at the user's request; the
  work paused before the rebuild.
- **15:18–15:26** — independent review ran its five probes
  (`outputs/rb-11/20260913T191847Z-review/`) and wrote
  [rb-11-review-guidance.md](rb-11-review-guidance.md).
- **18:00–18:35** — review corrections implemented (law-keyed validation
  with `[0, 1/d]`, fibre load/sweep checks, CCD notice, T-junction fixture,
  oracle contract with `--verify`/`--self-test`, per-element transfer
  checks, remeshing diagnosis, lumping message); remote `main` (`0465e3a28`,
  `a327e2932`) integrated by fast-forward under the staged work; both
  targets rebuilt once and the candidate frozen (`review-fixes/final/identity.txt`).
- **18:30–18:45** — frozen candidate: matrix 93/93 in verify mode, oracle
  self-test pass, `[input_validation]` 17/160, affected selection with
  `[assembler],[material_cache],[form_derivatives]` 100 cases / 4,771,646
  assertions, five smokes, single-threaded identity, lumped `golf-ball-P2`
  with its warning, HDA materials/E2E/remesh tests; full suite started on
  that candidate.
- **18:45–19:32** — full suite on the frozen candidate: 323 cases, 320
  passed, 3 failed — all pre-existing and attributed (RB-10 friction default
  ×2, RB-22 named error) plus the known cube-on-floor golden; committed and
  pushed as `75b4d284d`.

## Next session handoff

- Completed: input-check inventory, probe matrix, bounded repairs, regression
  tests, unit notices, record, the review corrections, the full suite on
  the frozen candidate, publication. Pending stages of RB-11: (1) the physical
  envelope characterisation (nearly incompressible / anisotropic / thin /
  distorted elements against RB-09-style references; constitutive derivative
  and rigid-motion invariance on a small fixture) — no production change is
  implied; (2) an HDA test for a per-element scalar on a non-first subdomain
  (publication procedure applies); (3) optional: list the intersecting
  primitives on an initial intersection; conflict detection for the `lsq`
  boundary method.
- Status: validated within stated scope for the input-validation stage
  (published, see below). The item stays open for the envelope stage, which
  is characterisation, not implementation.
- Model decision required: none. Observations for RB-12: the RB-09 unit
  dependence is now announced at startup but the tolerance scaling itself is
  a documentation/defaults matter; the run manifest should record the unit
  system and `characteristic_force_density`.
- Next command: `python3 tools/rb11/run_matrix.py --binary build/PolyFEM_bin --output <fresh>`
  after any change to mesh import, material binding or BC sampling. Next
  items eligible: RB-23, RB-12, RB-06/RB-08, the RB-11 envelope stage.
- Updated: the plan's RB-11 row and section, README.

## Envelope stage — characterized, limits documented (2026-09-14)

**Status: characterized within stated scope.** No element, quadrature,
material, tolerance or contact default was changed. The stage produced two
bounded production repairs on the time-dependent `LinearElasticity` path
and one input correction (Ogden term lists), all with regressions, plus
the sampled envelope below. Contract: [rb-11-envelope-contract.md](rb-11-envelope-contract.md)
(declared before the runs; amendment 1 declared before the refinements).
The independent [Stage 1 / Stage 2 review](rb-11-stage-review-20260913.md)
of 2026-09-13 is retained as evidence of what needed correction: its
findings (the quasistatic force-output defect, the incomplete run
inventory, the vacuous verifier, the qualified reference claims) are
addressed here one by one.

### Evidence and candidates

`outputs/rb-11/20260913T235827Z-envelope/`:

- `identity.txt` + `PolyFEM_bin` — **candidate A** (`452e6244…`, built from
  `ce7c88b4b` plus the transient crash fix and the quasistatic-mass fix,
  before the output repair and the Ogden change); preserved untouched.
- `identity-candidate-B.txt` + `PolyFEM_bin-candidate-B` (`a134f6c8…`) +
  `working-tree-candidate-B.diff` — **candidate B**, the final candidate:
  the sources published as `ef5dffe80` on `sdast9/polyfem:main`
  (2026-09-14; the review note itself is `c599ca99e`).
- `matrix-attempt1-grad1e-10/` — protocol 1 (aborted, see tolerances);
  `floor-probe/` — the roundoff-floor probe; `matrix/` — candidate A's
  protocol-2 runs (51 recorded exit-0 runs reused, 19 linear outputs whose
  process exit was never recorded — re-executed on B — and 39 cases that
  never ran); `matrix-candidate-B/` — the reconciled **109/109** matrix plus
  the 4 refinements (`summary.md`, `summary.json`, `verify.json`,
  `candidate.json`; every entry names its candidate and binary sha);
  `independence-check/` — candidate B reruns of three candidate-A nonlinear
  cases, bit-identical solutions and iteration counts (`comparison.json`),
  which with source inspection (the changes touch `LinearElasticVarForm`,
  Ogden and the list-parameter binder only) justifies reusing A's
  NeoHookean/MaterialSum results; `qs-output-probe-final/` — the review's
  force-output reproduction on B; `tests-candidate-B/`,
  `input-matrix-candidate-B/`, `smokes-candidate-B/`, `hda-candidate-B/`
  — the validation runs listed at the end.

### Defects found by the stage and repaired (candidate B)

1. **Every time-dependent `LinearElasticity` run segfaulted** (transient or
   quasistatic with a `time` block): `LinearElasticVarForm::init_linear_solve`
   built the `InertiaForm` before `time_integrator->init`, and upstream
   #508's constructor reads `x_tilde()` → `x_prev()` on an empty deque. The
   golden `standard` scenes with `time` are preset analytical problems
   (`is_time_dependent()` false), which is why the suite never saw it.
2. **`time/quasistatic` was ignored by the linear formulation** — the mass
   term and `x_tilde` were always solved. `is_quasistatic()` now solves
   `K u = f(t)` per step.
3. **Quasistatic force output** (reproduced by the review on the working
   patch): exported elastic/body forces were divided by `dt²` although the
   quasistatic forms carried no acceleration scaling, and a live
   `InertiaForm` reported a force absent from the solved equations
   (`dt = .5`: elastic/body ×4, inertia .93). Repair: the forms carry the
   integrator's acceleration scaling in every time-dependent solve (the
   nonlinear `SolveData::update_dt` convention, divided out by the export),
   no inertia form in quasistatics (zero inertia force, like `SolveData`),
   and per step the body force is updated at the step's time and the
   inertia form's `x_tilde` refreshed before the history advances, so the
   export describes the step just solved (the step-0 export carries the
   load at `t0`). `qs-output-probe-final/`: `dt = 1` vs `.5` elastic/body
   ratio 1.0, quasistatic inertia 0, dynamic run inertia .155.
4. **`UnconstrainedOgden` accepted one term only**: the spec offered
   `alphas` as a scalar (a list was "invalid input json") while `mus`/`Ds`
   are lists; the energy loops over `alphas` and reads `mus[N]`, so a
   longer `mus` was silently truncated and a shorter one read out of range
   (dead asserts). Spec entry `/alphas` list added; named errors for
   `alphas`/`mus` count mismatch, empty `Ds`, `IncompressibleOgden` `c`/`m`
   mismatch (only when the law is configured — under `MultiModels` every
   law sees every body's json) and, in `GenericMatParams`, a body giving a
   different term count than the elements before it — reported as an
   implementation limitation (one term list per law), not a material
   restriction.

Regressions: `tests/test_linear_elastic_time.cpp` (`[linear_elastic]`,
also tagged `[rb11_envelope]`): a transient run at `dt = .05` (3 steps)
completes and its exported forces satisfy `−K u + f − M(u − x̃)/dt² = 0` on
the free DOFs to 1e-9 with `−K u` checked against the assembled stiffness;
a quasistatic schedule with a nonzero growing prescribed end value and
gravity reproduces the static solve at `dt = 1, .5, .25` (1, 2, 4 steps) to
1e-10 with identical elastic/body forces and an exactly zero inertia force;
a load `9.81·t` is exported at the saved step (two steps of `.5` = twice
one step of `.5`). `tests/test_input_validation.cpp` "Ogden term lists must
pair up" (scalar/list acceptance, three named refusals, the two-body
mismatch, the spec accepting a list).

### E1 — constitutive derivatives and rigid motion (`[rb11_envelope]`)

`tests/test_material_envelope.cpp`, 5 cases / 423 assertions, on a 6-tet
unit cube (P1) through the production `ElasticForm`, for 17 sampled
configurations (every law of `AssemblerUtils::elastic_materials()` except
AMIPS and MultiModels, plus NeoHookean at ν = .4999 and
MaterialSum(NeoHookean, HGODispersion)). These are sampled 3D constitutive
checks, not all-law/all-regime certification.

| Check | Result |
| --- | --- |
| FD gradient / Hessian at 6 random states of amplitude .02 | all consistent at 1e-5 (FixedCorotational's Hessian at 1e-4, upstream's tolerance for its polar-decomposition tangent) |
| translation invariance (energy, gradient, Hessian) | ≤ 1e-9 for every law |
| objectivity `E(R(X+u)+t) = E(u)`, forces rotate, tangent `R H Rᵀ` | ≤ 1e-9 / 1e-8 / 1e-7 for every hyperelastic law |
| linear laws (LinearElasticity, HookeLinearElasticity) | not objective: rotation energy `∝ θ⁴` (ratio 16 ± 2 % between θ = .01 and .02) — the documented small-strain limit |
| stress-free reference `E(0) = 0`, `∇E(0) = 0` | every law except ActiveFiber (active stress by design; its affine energy is negative) |
| fibre frame indifference `E(F; a) = E(QFQᵀ; Qa)` | HGOFiber, HGODispersion, ActiveFiber, MaterialSum ≤ 1e-9 |

### E2 — the sampled envelope (109 cases + 4 refinements, all exit 0)

All runs are public synthetic fixtures (Kuhn tetrahedralised beams and
cubes), single-threaded, quasistatic, `E = 1e6`, `ρ = 1000`; the nonlinear
runs stop at `grad_norm_tol 1e-8` (protocol 2, below). `r` is the tip ratio
to the P2 `h = .125` reference of the same ν and load; κ(K) is the reduced
stiffness condition number of the `LinearElasticity` twin (the
stress-free tangent on that mesh, not the deformed or fibre-reinforced
tangent).

**Volumetric locking (cantilever 4×1×1, tip deflection ≈ 1e-3 L):**

| ν | P1 h=.5 | P1 h=.25 | P2 h=.5 | P2 h=.25 | κ(K) P1 h=.5 | κ(K) P2 h=.5 |
| --- | --- | --- | --- | --- | --- | --- |
| .3 | .526 | .795 | .987 | .997 | 1.6e4 | 1.1e5 |
| .45 | .337 | .613 | .958 | .990 | 2.8e4 | 2.8e5 |
| .49 | .162 | .327 | .928 | .983 | 6.1e4 | 1.2e6 |
| .499 | .0885 | .117 | .909 | .977 | 3.0e5 | 1.1e7 |
| .4999 | .0796 | .0828 | .907 | .976 | 2.7e6 | 1.1e8 |

P1 tetrahedra lose half the deflection already at ν = .3 on `h = .5`
(ordinary coarse bending error) and 92 % of it at ν = .4999 (volumetric
locking on top); P2 stays within 2.4 % of the reference on `h = .25` at
every ν. C-L1 (`r ≥ .95`, P2 `h = .25`) passes at every ν; C-L2 (monotone
in ν) passes for the three sweeps — both are empirical targets on this
fixture, not theorems (the review's counterexample on pointwise bounds
stands). κ(K) grows ~`1/(1 − 2ν)`: ×170 (P1) and ×1050 (P2) from ν = .3 to
.4999.

**Thin sections (square section H, two cells across, ν = .3):**

| H (aspect of the P1 elements) | P1 nx=8 | P1 nx=16 | P2 nx=8 | P2 nx=16 | κ(K) P2 nx=8 | reference vs Timoshenko |
| --- | --- | --- | --- | --- | --- | --- |
| 1 (1) | .526 | .587 | .987 | .993 | 1.1e5 | 2.1 % |
| .25 (4) | .163 | .353 | .979 | .992 | 2.3e7 | .66 % |
| .1 (10) | .034 | .114 | .975 | .990 | 8.9e8 | .38 % |

P1 shear-locks on thin sections (3 % of the deflection at aspect 10); P2
keeps 97–99 %. The references approach beam theory as `L/H` grows (C-A).

**Distorted meshes (h = .5 beam, ν = .3):** interior jitter of .15 h / .30 h
moves the P1 ratio from .526 to .526 / .516 and the P2 ratio from .987 to
.987 / .987 (κ(K) 1.6e4 → 1.7e4, 1.1e5 → 1.5e5); stretched cells 2:1 / 4:1
(4×2×2 and 2×2×2 cells — a combined aspect *and* resolution effect) give
P1 .382 / .203 and P2 .968 / .912.

**Homogeneous near-incompressibility (compressed cube, exact state in the
space):** Cauchy stress and lateral stretch match the exact Neo-Hookean
state to ≤ 1.03e-7 / ≤ 3e-12 at every ν up to .49999 (C-H); the stress is
uniform to 1e-13. κ(K) of the `n = 2` twin grows from 50.8 (ν = .3) to
5.7e5 (ν = .49999). Newton iterations to the protocol-2 stop: 4/5 (ν = .3,
n = 2/4), 4/6, 5/14, 10/16, 8/14, 12/**24** — **C-N (≤ 20) fails at ν =
.49999, n = 4 (24, converged)**: a measured cost-target miss retained as a
performance observation, not evidence of physical error and not a reason
for a new stopping or line-search policy.

**Anisotropy:** the ten affine MaterialSum(NeoHookean, HGODispersion)
cases (fibre x / 45° / z, κ 0, 1/6, 1/3) match the law's Cauchy stress to
≤ 3.23e-10, uniform to 1e-13; a fibre of length 3 gives the unit fibre's
stress (HGODispersion normalises). Fibre-reinforced bending (k1 = E along
the axis): P1 .651 / .865 (h = .5 / .25), P2 .993 / .999 — no anisotropic
conditioning measurement exists in this stage.

**Reference accuracy (amendment 1, C-REF ≤ 1 %):** one P3 refinement on the
reference mesh moved the tip by **.106 %** (ν = .3), **.996 %** (ν = .4999),
**.108 %** (thin H = .1) and **.046 %** (fibre-reinforced). The ν = .4999
reference therefore carries ≈ 1 % of remaining locking itself — the
criterion is met at its edge, and the P2 `h = .25` space is 3.4 % below
the P3 value there; the C-L1 verdict stands with that uncertainty stated.
No further refinement was run (declared bound; the P3 runs cost 13–19 min
at 182k DOF).

**Tolerances.** Protocol 1 (`grad_norm_tol 1e-10`) never terminated on the
56k-DOF ν = .4999 reference: the mass-weighted L2 gradient floors at
5–6.5e-6 (relative 2e-7 to the initial gradient, `‖Δx‖` ~ 1e-16) from the
third iteration on (`floor-probe/`), below that stop. Protocol 2 uses
`grad_norm_tol 1e-8`, `rel_grad_norm_tol 1e-12` — still 1e3 below the
production default, which is untouched. The review's paired comparison of
the 41 outputs common to both protocols found a worst tip difference of
2.76e-8 relative; the timed-out reference is not in that pair set.

### Runner and verification (the review's §2–3)

`tools/rb11/envelope.py` now separates input rendering, execution
(`run.json` version 2 with command, binary sha, input hashes, exit/signal/
timeout, wall time — written before any parsing), per-case parsing that
never raises (nonlinear step lists, the linear `{'solver_info':
'Success'}` dict that crashed the first run inside `pool.map`, missing and
malformed output), analysis, and verification against an explicit manifest
(unknown names refused, empty manifest refused; every check reports
pass/fail/not evaluated with value, threshold and cases; missing partners
are "not evaluated", never passed). Endpoints are taken from the PVD's last
timestep and must sit at `t = 1`; nonlinear runs must report `converged`;
fields must be finite with matching sizes, `det F > 0`, and the tip sample
exact. Reuse of an earlier candidate's record requires exit 0, byte-identical
regenerated inputs and a stated binary identity; stale records are moved
aside, never overwritten. `--self-test` exercises these failure modes
(string/dict/list/missing/malformed `solver_info`, empty manifest, hollow
exit-0 record, unrecorded exit, wrong endpoint, unconverged run, missing
iteration count).

### Validation of candidate B (2026-09-14)

- `unit_tests "[input_validation],[rb11_envelope],[linear_elastic],[time_integrator]"`: 31 cases / 920 assertions.
- `unit_tests "[assembler],[material_cache],[form_derivatives]"`: 61 cases / 4,770,073 assertions.
- `unit_tests standard` (golden scenes incl. `unconstrained_ogden`, `incompressible_ogden`, `multimodel`, `pyramid`): 67/68 assertions — the one failure is the pre-existing RB-22 named error on `multi-material/stretch-cubes.json` (documented in the Stage 1 record).
- `tools/rb11/run_matrix.py --verify`: 93/93; `--self-test` pass.
- five public smokes: exit 0, 0 error lines, last-step solutions within 8e-16 of the Stage 1 frozen-candidate fingerprints; the friction smoke single-threaded bit-identical (`smokes-candidate-B/summary.json`).
- HDA: `test_polyfem_materials.py` (with the new non-first-subdomain per-element kappa check: 707/707 body-1002 elements carry the authored value, worst 1.6e-8, nothing leaks onto body 1), `test_polyfem_hda.py`, `test_remesh_hda.py` — see `hda-candidate-B/`.
- Full suite: not rerun for this stage; the three pre-existing failures (RB-10 goldens ×2, RB-22 stretch-cubes) are unchanged in nature (the `standard` set reproduces the RB-22 one).

### Supported sampled envelope and limits

Supported by this stage's evidence: hyperelastic laws are objective and
derivative-consistent on the sampled fixture; homogeneous and affine
states are reproduced to roundoff at any ν up to .49999 and any sampled
fibre/dispersion; P2 tetrahedra resolve bending within 2.4 % of a P2
`h = .125` reference (itself within 1 % of P3) at ν ≤ .4999 on `h = .25`,
within 3 % on thin sections down to aspect 10, and within 1.3 % under
.30 h jitter. Limits: P1 tetrahedra lock volumetrically (ν ≥ .45) and in
shear (aspect ≥ 4) — a resolution/order statement for users of the
Houdini node, not a defect; the linear laws are not objective; near
incompressibility raises κ(K) as `1/(1 − 2ν)` and the Newton count to the
protocol-2 stop up to 24; no anisotropic conditioning, no large-strain
bending, no contact and no dynamic envelope was measured; the constitutive
checks are sampled, not exhaustive.

### Procedure log (local time)

- 2026-09-13 19:40–20:30 — contract, fixtures, E1 tests, transient crash
  and quasistatic-mass fixes, protocol 1 aborted at the roundoff floor,
  protocol 2 started, runner crashed inside `pool.map` on a linear twin's
  `solver_info`; handoff written.
- 2026-09-13 evening — independent review (`rb-11-stage-review-20260913.md`,
  `c599ca99e`): quasistatic force-output defect reproduced, inventory
  corrected (51 / 19 / 39), verifier gaps and reference claims qualified.
- 2026-09-14 02:40–03:15 — runner rewritten and self-tested; force-output
  repair with per-step form updates; `[linear_elastic]` regressions;
  Ogden message reworded; candidate B frozen beside A.
- 03:15–03:30 — independence check (bit-identical), the 58 missing/
  re-executed cases on B (42 s), reconciled 109/109 matrix, one contract
  failure (C-N).
- 03:35–04:00 — amendment 1 declared; four P3 refinements (72 s – 19 min);
  focused/affected/golden selections, input matrix, smokes, HDA tests on B.
