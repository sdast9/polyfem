# RB-05 — Candidate generation and resource failure containment

Date: 2026-09-12
Status: **validated within stated scope** (stage 1: containment of the swept
candidate cache, pre-build step diagnostics, broad-phase resource limits
enforced before allocation; **the limits are on by default since the
follow-up below (user decision 2026-09-12)**, together with meaningful exit
statuses and the Houdini controls)
Selected stage: inventory → bounded reproduction → containment + opt-in limit
→ validation. Retry/backoff is RB-08; rollback of the exposed failed iterate
is RB-06; the production broad-phase default and the 50·d̂ trial cap are
unchanged.

## Contract and authorization

- User-selected item: "start investigating RB-05 if it is still open"
  (2026-09-12); RB-05 was `not started`. No scope or policy decision was
  given beyond the plan's section, so: candidate protection, pre-build
  diagnostics, an **opt-in, default-disabled** limit, no production default,
  no broad-phase default change, no cap change, no retry.
- Invariant (plan): resource protection must never silently omit potentially
  colliding pairs or accept an unchecked step. Success criterion of this
  stage: (1) no partial or stale candidate set is ever treated as complete,
  including after an exception escapes a line search; (2) with a limit
  configured, the failure is raised **before** the bounded allocation, is
  named, is not retried by any solver handler, leaves the accepted state
  untouched and leaves a diagnostic; (3) with the limits at their default (0)
  every public scene is bit-identical to the previous binary.
- Dependencies: RB-01 (cache ownership; the swept cache lifecycle extended
  here), RB-04 (attempt stream and candidate statistics reused as the
  pre-build diagnostics; observer order changed, checker extended). Effective
  IPC source is the local `ipc-toolkit-fork` override (`e3c8d3fe` →
  companion commit below); PolySolve `5afe3b5d` unchanged.
- Exclusions: a **default** budget (needs the user's machine/workload
  choice); a top-level `main` handler turning the abort into a clean exit
  code (a process-wide policy change that would move every failure's exit
  from −6 to 1 and is pinned by RB-04's failure check — proposed below, not
  done); budgets for the LBVH/spatial-hash/sweep-and-prune broad phases
  (explicitly refused at configuration, see limits); CCD time budgets; the
  friction/adhesion forms' own broad-phase uses (the friction form reuses the
  contact form's collision set; adhesion is a separate form and is not
  budgeted); any change to `NormalAdhesionForm`.

## Baseline and reproduction

- PolyFEM `d842f4f06` on `main`, clean tree at start (the RB-04 remainder had
  just been published by another session; its running validation was left
  alone until it finished). Toolkit `e3c8d3fe` (`semi-implicit-stiffness`),
  PolySolve `5afe3b5d` (`iteration-callback`), both local overrides per
  `CMakeCache.txt` (`IPCToolkit_SOURCE_DIR`, `PolySolve_SOURCE_DIR`).
- Build: `polyfem/build`, RelWithDebInfo, Apple clang 21.0.0, Unix Makefiles,
  `-j 6`; macOS 26.5.2, 18 cores, 128 GB. Baseline binaries
  `PolyFEM_bin b29eeb6d…`, `unit_tests 51312cc0…` (the RB-04 remainder's
  tested binaries), copied to the evidence directory before any rebuild.
- Evidence: `outputs/rb-05/20260912T182650Z/` in the parent workspace —
  `baseline.txt`, `session-log.md` (inventory and findings as they were
  made), `r1-baseline-repro/`, `r2-probe-baseline*/`, `r2-probe-budget/`,
  `tests-candidate/`, `tests-final/`, `ab-smokes-*`, `scene-limits*/`,
  `rb04-endpoints-final/`, `toolkit-tests/`, `hda-e2e*.log`, `build-logs/`.
  A user-owned `PolyFEM_bin` run (`test_cases/uniax_mesh_constraintfloor_zero_b`)
  was in progress for the first part of the session; `PolyFEM_bin` was not
  rebuilt until it had finished.

### Inventory (code reading; see `session-log.md` for the full notes)

The sweep handed to the contact broad phase is PolySolve's full trial step
`x + α₀·Δx` (`LineSearch::line_search` → `FullNLProblem::line_search_begin`
→ `ContactForm::line_search_begin` → `ipc::Candidates::build`), capped at
50·d̂ **only in semi-implicit mode**; adaptive/fixed modes sweep the whole
Newton step. The sequential clamp of `FullNLProblem::max_step_size` shortens
only the interval CCD prices afterwards, on the cache built over the longer
sweep. In the default `HashGrid` the cell is the **median** box extent, so a
uniform sweep re-scales its cells while a localized far sweep (a few nodes)
rasterizes each of their boxes into ~(L/cell)³ 16-byte `HashItem`s in
`insert_box`, then `detect_candidates` emits one candidate per pair of items
sharing a cell **before** the collision filter, the AABB test and the
`unique` pass. Nothing checks either count; the candidate count is logged
after the build (RB-04's statistics after that). Other broad phases: LBVH and
sweep-and-prune emit true overlaps only (no pre-count possible without the
traversal); `SpatialHash` has the same per-primitive voxel-list growth;
`BruteForce` compares |B₀|·|B₁| pairs. The CCD narrow phase allocates nothing
per candidate (its resource is time: `max_iterations` 10⁶ per candidate).

Exception propagation: PolySolve's line search has no guard, so
`line_search_end()` is **not** called when an exception unwinds through it;
`Backtracking` absorbs a `std::runtime_error` from `solution_changed` (smaller
step), `ALSolver::solve_al` absorbs one from a subsolve (scaled weight, up to
three times) and `minimize_with_stall_restarts` turns "Line search failed"
into a restart; all three re-enter through `nl_problem.init(sol)` →
`ContactForm::init` → `update_collision_set`, which used the swept cache
whenever `use_cached_candidates_` was still set. `main` has no handler: an
escaping exception is `std::terminate` (SIGABRT, exit −6) without unwinding;
the file log sink flushes every 3 s only. The RB-04 Proposal observation was
emitted **after** the forms' builds, so a failing build lost its own trial.

### R1 — stale swept cache after an escaped line search (reproduced)

`tests/test_resource_containment.cpp`, `[resource_containment]`, on the
baseline production sources (`r1-baseline-repro/baseline-sources-run.log`):
a 2D edge with a free vertex, d̂ = .1; `line_search_begin(zero, away)` builds
an (empty) sweep, no `line_search_end` (the exception path), then `init` /
`update_quantities` at coordinates in contact: **0 collisions instead of 1,
energy 0 instead of 7.797905781299e-05, gradient difference 5.30e-3** — 1 case
failed, 7/24 assertions failed. The third section (a new `line_search_begin`
replaces the cache) passed. In the release binary this window is escaped only
by a non-`runtime_error` (an allocation failure) or a budget — i.e. exactly
the RB-05 failure — so the reproduction is the mechanism, not an observed
production trajectory.

### R2 — bounded growth probe on the baseline toolkit

`tools/rb05/broad_phase_probe.cpp` + `run_probe.py` (compiled with the
`unit_tests` flags/link line, i.e. the effective toolkit; each configuration
in its own process; configurations whose estimated items exceed `--max-items`
are never built). Two 20×20 triangulated sheets (800 vertices, 2,242 edges,
1,444 faces), gap .05, d̂ = 10⁻³, one trial sweep along (1,1,1)/√3.

| sweep | items (16 B each) | pre-filter emissions EE / FV | candidates | max RSS | build |
| --- | --- | --- | --- | --- | --- |
| none | 11,974 | 43,093 / 12,168 | 6,196 | 10 MB | 1 ms |
| uniform, all 400 upper vertices, L = 5 … 1000 | 39,431 (cell re-scales) | 11.5 M / 5.3 M | 912,192 | 586–603 MB | 0.12 s |
| one vertex, L = .5 | 13,480 | 45,646 / 13,410 | 6,630 | 10 MB | 1 ms |
| one vertex, L = 5 | 974,248 | 541,252 / 339,970 | 11,047 | 34 MB | 12 ms |
| one vertex, L = 20 | 60,758,080 (972 MB) | 30.4 M / 20.3 M | 11,047 | 1,451 MB | 0.66 s |
| two spread vertices, L = 20 | 142,221,602 (2.28 GB) | 208 M / 99 M | 14,367 | 8,798 MB | 3.5 s |
| one vertex, L = 50 / 200 / 1000 (estimate only) | 9.41e8 (15 GB) / 6.0e10 (959 GB) / 7.5e12 | — | — | not built | — |

The independent pre-allocation estimate (cell size, grid, per-box coverage
from the boxes alone) **equals the instrumented item count in all 13 built
hash-grid configurations**. Brute force, BVH and sweep-and-prune return the
same 11,047 candidates for the L = 5 single-vertex case in 9–10 MB (spatial
hash 64 MB). Allocation failure is not a usable signal on this platform:
`setrlimit(RLIMIT_DATA)` fails and `ulimit -d`/`-v` return `EINVAL` on macOS
26.5, so an oversized build never throws `std::bad_alloc` — it grows until
the kernel kills the process. Containment therefore has to be a
pre-allocation check.

## Findings and changes

| Finding | Evidence | Outcome |
| --- | --- | --- |
| A swept candidate cache left active by an escaped line search is consumed by the retry paths' `init`/`update_quantities` (RB-05/RB-01 invariant) | R1 | **fixed**: `ContactForm::init` and `update_quantities` discard the swept cache first; `line_search_begin` discards any previous interval and is exception-safe (clears, logs the sweep, flushes, rethrows); toolkit `Candidates::build` clears every list on any exception |
| Hash-grid items and pre-filter emissions grow unbounded before any count exists; a localized far sweep costs ~(L/cell)³ items per box; uniform sweeps grow emissions ~N² | R2 | **characterized; opt-in limit implemented**: `ipc::BroadPhaseBudget{max_cell_items, max_candidate_emissions}` on `ipc::BroadPhase`, checked from the boxes (before insertion) and from the sorted items (before enumeration); `BruteForce` bounds its box pairs; other methods refuse a budget explicitly |
| No pre-build step diagnostics; the failing trial's norms were lost | inventory; E2E `sweep` | **fixed**: pre-build debug line (trial L∞, in supports, clamp, swept L∞, median vertex sweep, surface sizes, method, budget); Proposal observed before the forms build; `aborted` row with `broad_phase_built`; static-build failures (`init`, `update_quantities`, `solution_changed`) logged and flushed |
| Allocation failure cannot be provoked or caught on the target platform | R2 `rlimit` | **documented limit**: only pre-allocation checks contain the failure |
| `SpatialHash` voxel lists, `HashGrid::hash()` int product / `HashItem(int,int)` narrowing above ~2³¹ cells (keys wrap: extra false positives filtered by the AABB test, no lost pairs; UB in principle) | reading; R2 grid counts up to 1.25e15 cells | **disclosed, not changed** (upstream behavior; a budgeted grid never reaches it) |
| Spec offers `sweep_and_prune`/`SAP` but `ContactForm.hpp`'s enum map has no entry, so the string maps silently to `hash_grid` | reading | **not RB-05**: recorded for RB-11 |

### What changed

Toolkit (`ipc-toolkit-fork`, `semi-implicit-stiffness`):
- `broad_phase.hpp/.cpp`: `BroadPhaseBudget` (0 = unlimited; `enabled()`),
  `BroadPhaseBudgetExceeded` — derives from `std::exception`, **not**
  `std::runtime_error`, so the solver handlers that retry a runtime_error
  (smaller step, scaled AL weight, stall restart) cannot absorb a resource
  failure — with `method`, `quantity`, `requested`, `limit`, `details`;
  `BroadPhaseBuildStatistics` (boxes, cell items, emissions, cell size, grid);
  `supports_budget()` (HashGrid, BruteForce) and `check_budget_supported()`
  (`std::invalid_argument` otherwise), called by every public `build` and by
  `Candidates::build` (some methods override the public builds).
- `hash_grid.hpp/.cpp`: `box_cell_range()` shared by `insert_box` and the
  new `count_cell_items()` (a `parallel_reduce` of integer arithmetic, no
  allocation) so the count is exactly what insertion emits;
  `check_cell_item_budget()` between `resize` and `insert_boxes`; in both
  `detect_candidates` overloads a run-length count over the sorted items
  (per key: n₀·n₁ cross pairs, or n(n−1)/2) before the enumeration, checked
  per detect call; statistics filled (items always — free from the vector
  sizes; emissions only while a budget is enabled).
- `brute_force.hpp/.cpp`: emissions = |B₀|·|B₁| (n(n−1)/2 within one set)
  checked before the loop.
- `candidates.hpp/.cpp`: `build()` = budget-support check + `clear()` +
  `try { build_unchecked } catch (...) { clear(); throw; }`; statistics
  accumulated over the main and codimensional passes (`set_build_statistics`).
- `tests/src/tests/broad_phase/test_budget.cpp` (`[budget]`).

PolyFEM (`polyfem`, `main`):
- `ContactForm`: `discard_swept_candidates()`; `init`/`update_quantities`
  discard first; `rebuild_collision_set(x, operation)` wraps every
  `update_collision_set` with a logged, flushed rethrow; `line_search_begin`
  discards the previous interval, logs the pre-build sweep summary at debug,
  wraps the build (error log with the sweep summary + flush + rethrow), logs
  the toolkit's measured items/emissions/grid, and extends
  `CandidateStatistics` (`intermediates_measured`, `last/max_cell_items`,
  `last/max_candidate_emissions`); `set_broad_phase_budget()` (refuses an
  unsupported method with a named error, resetting the budget);
  `broad_phase_budget_from_args()`.
- `BarrierContactForm::diagnostic_state()["candidate_count"]` gains
  `broad_phase_intermediates`.
- `FullNLProblem::line_search_begin`: the Proposal observation precedes the
  forms' builds.
- `NonlinearElasticVarForm`: `aborted_proposals`, `proposal_builds`; the
  failure path writes the pending proposal as an `aborted` row with
  `trial.broad_phase_built`; the attempt summary carries `aborted_proposals`
  and `broad_phase_candidates.intermediates`; the budget is set after
  `init_forms` (also in the legacy `StateSolveNonlinear` path).
- `json-specs/input-spec.json`: `/solver/contact/CCD/resource_limits/{max_cell_items,
  max_candidate_emissions}` (ints, default 0 = disabled) with the precise
  scope in their docs.
- `tools/rb04/check_solver_attempts.py`: accepts an `aborted` row only in a
  failed attempt, at most one, last; `builds == accepted + rejected +
  feasibility + aborted_with_built` (exact); a failed attempt aborted before
  any iterate reports the explicit unavailable iterate; `minimize_index 0`
  for an aborted feasibility check before the first minimize.
- `tests/test_resource_containment.cpp` `[resource_containment]`;
  `tools/rb05/` (probe, driver, `run_scene_limits.py`).

Semantics of the two bounds (precise scope of the protection): `max_cell_items`
bounds the total `(box, cell)` items of one hash-grid build (16 B each, plus
8 B of merge indices per item during detection); `max_candidate_emissions`
bounds, per detection pass, the item pairs sharing a cell (hash grid) or the
box pairs compared (brute force) **before** the collision filter, the AABB
test and deduplication — every candidate buffer of that pass is at most that
many entries. Not bounded: the final collision set, CCD time, the memory of
the other forms, the codimensional passes' own item counts beyond the same
bounds (each pass is checked separately), and the LBVH / spatial-hash /
sweep-and-prune / STQ methods (a budget with those is refused at startup).
Defaults are 0: the unbudgeted path executes no counting at all.

## Validation

Final binaries: `PolyFEM_bin` `c0e694f4…`, `unit_tests` `7728ebb3…`
(sources = the committed ones after clang-format; earlier "preformat" runs
with the same code are kept in `tests-candidate/`, `ab-smokes-disabled-limits/`,
`scene-limits/`).

| Check | Input/configuration | Expected criterion | Measured result | Status |
| --- | --- | --- | --- | --- |
| Baseline reproduction | `[resource_containment]` on baseline sources | stale cache consumed | 1 case / 7 of 24 assertions failed (0 vs 1 collisions) | reproduced |
| New regression | `[resource_containment]` | passes on the fix | 2 cases / 73 assertions | pass |
| Affected selection incl. the new tag | RB-04 selection + `[resource_containment]`, seed 1 | no failure | 78 cases / 5,126 assertions (RB-04: 76 / 5,053) | pass |
| Disabled-limit equivalence | five public smokes + RB-03 Q1 hex, single-threaded, baseline `b29eeb6d` vs final | proxies and solutions bit-identical | 6/6 scenes exit 0/0, proxies identical, solutions identical, max|diff| 0.0 (`ab-smokes-final-formatted/`) | pass |
| Enabled-but-not-reached equivalence | `quasistatic-semi`, `quasistatic-adaptive`; limits 10⁹/10⁹ vs none, diagnostics on, 1 thread | final solution SHA identical; attempt streams pass the checker | identical SHAs on both scenes (`b33a92a4…`, `98a813e6…`); 39 / 45 pre-build lines; checker pass on all four runs | pass |
| Limit reached during a static build | `max_cell_items: 1` (init needs 2,118 items) | named error, no accepted step, non-zero exit | exit −6, 0 frames, `Contact collision set rebuild failed in init (… would need 2118 cell_items (limit 1) …)` logged and flushed; no attempt stream (no minimize started) | pass |
| Limit reached inside a trial sweep | `max_cell_items: 2414` (first sweep needs 2,415; static build 2,118) | pre-build sweep line logged, `aborted` row (`broad_phase_built: false`), failed-attempt record, checker passes, no accepted step | exit −6, only the initial frame; failure in the ALSolver feasibility check before the first minimize: pre-build line `trial Linf=0.0625 (62.5 supports; clamp 0.8 -> swept Linf 0.05), median vertex sweep 0`, `aborted` row (`minimize_index` 0, `broad_phase_built` false), `failed_attempt` record with the exception text, checker pass; both scenes | pass |
| Unsupported method refused | limits + `broad_phase: bvh` | named configuration error before any solve | exit −6 at startup: `solver.contact.CCD.resource_limits cannot be enforced by broad phase "LBVH" …`; no frames, no attempt | pass |
| Bounded synthetic allocations under the limit | probe `budget` set (final library) | limits fire before allocation; empty candidate set after failure; generous limits reproduce the unbudgeted counts; toolkit emission count = independent count | items 6.08e7/1.42e8 refused at 9 MB RSS in 0.000 s (unlimited: 1,451 / 9,081 MB); emissions 30.4 M / 208 M / 11.5 M refused, brute force 2,512,161; `candidates_after_failure` 0 in all 6; generous limits: 11,047 / 14,367 / 912,192 candidates as unlimited, toolkit emission count = independent count (4/4); estimate == measured 11/11 built (`r2-probe-final*/`) | pass |
| Toolkit regression | standalone toolkit tests with the pinned tests-data (`c7eba549`) | `[budget]` and `[broad_phase]~[.]` pass | `[budget]` 3 cases / 80 assertions; `[broad_phase]~[.]` 15 cases / 1,464,063 assertions (`toolkit-tests-final/`, standalone build of the committed sources) | pass |
| RB-04 endpoint regression (observer order, aborted rows) | `tools/rb04/run_endpoints.py` + both checkers, final binary | all pairs and checks pass | `passed: true`; 8 runs (three on/off pairs exit 0, the failure pair −6/−6 as designed); coefficient-events check exit 0; attempt-stream check exit 0 (`rb04-endpoints-final/`) | pass |
| HDA end-to-end, Houdini 22.0.429 | `hython houdini_HDAs/tests/test_polyfem_hda.py` on the final binary | pass | 6/6 PASS, exit 0 (`hda-e2e-final.log`) | pass |
| Formatting / whitespace / links | clang-format on changed hunks (both repos), `git diff --check`, record links | clean | clean | pass |

Measured overhead (single runs, not benchmarks): E2E `time_line_search`
0.1495 s (unlimited) vs 0.1478 s (generous) on `quasistatic-semi`, 0.3094 vs
0.3087 s on `quasistatic-adaptive`; process totals 1.766 vs 1.750 s and
6.221 vs 6.200 s — the counting is not resolvable on these scenes. Probe
builds with generous limits vs none: 0.666 vs 0.663 s (one vertex, L = 20),
2.75 vs 3.51 s (two vertices, L = 20; the unlimited run pages 9 GB),
0.115 vs 0.122 s (uniform, L = 5). The item count is one integer pass over
the boxes; the emission count is one linear pass over the sorted items.

- Numerical termination versus residuals: unchanged by construction —
  the A/B smokes are bit-identical; the E2E generous runs are bit-identical
  to the unlimited runs.
- Physical quantities: none measured here; no claim of physical accuracy.
- Missing quantities: no limit-reached measurement on a private or large
  production scene (public fixtures only); no default budget; no LBVH /
  spatial-hash / SAP protection; no allocation-failure test (impossible on
  this platform, see R2).
- Retained failed/partial runs: the E2E `tiny`, `sweep` and `unsupported`
  runs are deliberate failures and are kept; the first toolkit
  `[broad_phase]` run selected the hidden benchmark cases by mistake and was
  stopped by me (my own process), then rerun with `~[.]`.
- Tolerances: none changed; no golden regenerated. The checker was
  **extended** (new row kind, exact build identity kept), not relaxed.
- Not performed: the whole unit suite; private scenes; Teseo; other
  platforms; the toolkit's hidden benchmarks.

## Publication and reproducibility

- Rebuilt targets: `ipc_toolkit`, `unit_tests`, `PolyFEM_bin` (final hashes
  above, `tests-final/binaries.txt`); standalone toolkit test build in the
  session scratchpad (`toolkit-tests/binary.txt`).
- Toolkit companion commit: bb795446 on
  `sdast9/ipc-toolkit:semi-implicit-stiffness`; PolyFEM pin bumped in
  `cmake/recipes/ipc_toolkit.cmake`; the build's local override points at the
  same checkout.
- PolyFEM commit: `db38cbff2` on `sdast9/polyfem:main` (this documentation note follows it).
- HDA assets unchanged (no source or asset edit; E2E run only).
- Remaining working-tree changes: none intended.
- Local/private evidence not distributed: the evidence directory (logs, VTUs,
  probe binaries); the checked-in `tools/rb05/` sources and this record are
  the reproducible part.

## Next session handoff

- Completed: inventory, R1/R2 reproductions, containment, pre-build
  diagnostics, opt-in limits, validation, publication.
- Pending decisions (user): a **default** budget — the measurements say a
  hash-grid item bound of 10⁸ (1.6 GB of items + 0.8 GB of indices) would
  have refused every configuration above 1.5 GB RSS in R2 while every public
  scene needs ≤ 2,415 items per sweep; an emission bound of 10⁷ would catch
  the uniform-sweep regime (11.5 M) at ~10 MB. Neither is set. A clean exit
  code on a resource failure (a top-level handler in `main`) is a separate
  policy change (RB-12): today the named error is logged and flushed, the
  failed-attempt record and the `aborted` row are written, and the process
  aborts with −6 as every uncaught failure does.
- Next eligible: RB-10 (friction), RB-09 (references), RB-23 (Q3+ hex basis),
  RB-11 (the `sweep_and_prune` enum-map gap above), RB-06 (rollback of the
  exposed failed iterate; the `aborted` row and the discarded interval are
  the state it starts from), RB-08 (retry, now that a resource failure is a
  distinct, non-retried exception type).

## Follow-up — defaults on, exit statuses, Houdini controls (2026-09-12)

**User decisions (2026-09-12):** enable the limits by default with the
measured values, give a resource failure a real exit status, and expose the
controls on the HDA with explanations a novice can follow.

### What changed

- `solver/contact/CCD/resource_limits/{max_cell_items, max_candidate_emissions}`
  now default to **-1 = automatic**: `ContactForm::apply_resource_limits`
  applies the production defaults (`default_max_cell_items` 10⁸ ≈ 2.4 GB of
  items + merge indices, `default_max_candidate_emissions` 5·10⁷ ≈ 1.75 GB at
  the measured ~35 B/emission) when the broad phase can enforce them
  (`hash_grid`, `brute_force`) and drops them with an info-level notice
  otherwise — the HDA's default broad phase is **BVH**, so an error there
  would have broken every HDA scene. `0` disables a bound; an explicit
  positive bound with a method that cannot enforce it stays a startup error.
  A one-line notice states the effective limits on every run.
- `main` catches every named failure (`src/polyfem/utils/ExitStatus.hpp`):
  `ipc::BroadPhaseBudgetExceeded` and `std::bad_alloc` → **exit 3** with a
  plain-language explanation (safety stop, not a crash; steps on disk are
  kept; reduce the step / load increment, use BVH, or raise the limits); any
  other `std::exception` → **exit 1** with `PolyFEM stopped: <what>`. Sinks
  are flushed before returning. An abort signal (−6 / 134) now means a real
  crash or assertion; RB-22's record documented 134 for named errors, which
  is historical from this commit on. No existing tool asserted on −6 (the
  RB-04 runner requires equal non-zero codes for its failure pair).
- HDA (`sdast9/houdini-plugins` `0f7b8fd`): Contact ▸ CCD Parameters
  gains *Resource Limits* (Automatic / Off / Custom) with *Max Grid Items*
  and *Max Candidate Pairs* for Custom, exported as −1 / 0 / N and restored on
  import; novice tooltips explain the mechanism, the numbers, the exit status
  and the fixes; the Broad Phase tooltip says which methods can blow up.
  `test_polyfem_hda.py` checks the automatic default, the Custom/Off round
  trips and the tooltips. All three assets rebuilt; local, published and
  installed hashes match.
- `tools/rb05/run_scene_limits.py`: `unlimited` is now explicit 0/0, a
  `default` run (no key) and a `bvh-automatic` run were added, and the
  expected exit statuses (0/0/0/3/3/1/0) are asserted.

Published as `377a83fda` on `sdast9/polyfem:main` (this note follows it);
HDA `0f7b8fd` on `sdast9/houdini-plugins:main`.

### Validation (binaries `PolyFEM_bin` `edb2789f…`, `unit_tests` `bb85f83e…`)

| Check | Criterion | Result | Status |
| --- | --- | --- | --- |
| `[resource_containment]` (option semantics −1/0/N, automatic resolution per method, explicit refusal) | pass | 2 cases / 86 assertions | pass |
| Affected selection | no failure | 78 cases / 5,139 assertions | pass |
| A/B smokes, baseline `b29eeb6d` (no limits) vs follow-up (automatic limits on) | bit-identical | 6/6 proxies and solutions identical, max|diff| 0.0 (`followup/ab-smokes/`) | pass |
| E2E both scenes: `default` == `unlimited` == `generous` | identical SHAs; exits 0/0/0/3/3/1/0 | `quasistatic-semi` and `quasistatic-adaptive`: identical SHAs, all exits as expected; `tiny`/`sweep` end with `PolyFEM stopped: … Exit status 3`, `unsupported` with exit status 1 (`followup/scene-limits/`) | pass |
| `bvh-automatic` | runs, notice logged, exit 0 | exit 0, five steps, notice "automatic limits are not enforceable by broad phase LBVH … running without them" | pass |
| RB-04 endpoint runner + both checkers | pass | `passed: true`, events check exit 0, attempts check exit 0 (`followup/rb04-endpoints/`) | pass |
| HDA: all 13 test scripts on this binary (`followup/hda-tests/`) | pass | 13/13 exit 0; `test_polyfem_hda.py` 7/7 PASS incl. the new resource-limit round trip | pass |
| clang-format / `git diff --check` | clean | clean | pass |

The BVH run's solution differs from the hash-grid runs by 1.9·10⁻¹⁶
(`followup/scene-limits-quick/`): the collision set's summation order, as
expected — not a limit effect.
