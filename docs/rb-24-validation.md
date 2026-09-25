# RB-24 — Per-thread memory of the contact path

Date: 2026-09-20
Status: **characterized — limits documented; closed by the user's decision
of 2026-09-20: no code change** (see [Decision](#decision-root-finding-method));
**remedy adopted 2026-09-25** (the user pulled option 1 forward: Tight-Inclusion
1.1.0's bucket DFS, see [Adoption](#adoption-tight-inclusion-110-2026-09-25)).
The allocation is named and measured; the remedy is upstream's replacement
of the algorithm (Tight-Inclusion 1.1.0), measured here in isolation, then
adopted ahead of the dependency sync.
Selected stage: 1 (locate the allocation without a rebuild) → candidate
measurement for stage 2 (isolated worktree, nothing published) → record.
The plan's hypothesis — a fixed per-thread block in PolyFEM's contact form or
the toolkit's potential/Hessian assembly — is **not** what the measurement
found: the memory is Tight-Inclusion's interval root finder on the few CCD
queries that run to the iteration cap, one transient block per query in
flight.

## Contract and authorization

- User-selected item: "let's begin looking at RB-24" (2026-09-20); RB-24 was
  `not started` (opened 2026-09-14 from RB-12's stage-2 probes).
- Invariant (plan): the memory a contact scene needs is proportional to the
  collision-surface size and the candidate set, with a per-thread overhead
  that is small against the problem, not a fixed hundred-megabyte block per
  thread. Success criterion of stage 1: the allocation is named and its
  scaling located at 1 and 18 threads. Stage 2 asks for a bound or a lazy
  sizing *without changing any result*; the only effective remedy found is a
  change of the CCD root-finding algorithm, which does change results in
  general, so it is measured here and left to the user.
- Decision boundary (plan, respected): no default thread count, resource
  limit or contact setting was changed; nothing was published; the shared
  tree was not rebuilt (the RB-23 session was running its full suite from it
  when this session started — stage 1 ran on the existing binary with an
  external allocation tracer, the candidate in an isolated worktree).
- Dependencies: RB-12 manifests (`completion.peak_rss_mb` is the measurement),
  RB-05 (the broad-phase budget is unrelated: same growth with hash grid,
  BVH and brute force), RB-12's stage-2 repeat matrix and
  `tools/rb12/repeat.py` (reused for the candidate).
- Exclusions: Teseo and private scenes (not run); Linux/Windows (one
  platform, the macOS allocator's retention behaviour is part of the
  numbers); the Houdini asset (no control until a decision).

## Baseline and reproduction

- Shared `polyfem/build/PolyFEM_bin` of 2026-09-20 10:27 (HEAD `a6d70bd49`
  plus the RB-23 session's then-uncommitted hex-basis edits, which the
  tetrahedral probe does not exercise) for stage 1; for the A/B measurement an
  isolated worktree at `a6d70bd49` with the companion checkouts as worktrees
  (IPC `c24d803e`, PolySolve `bce32a39`), the saved configure command
  (`configure-command.txt`; RelWithDebInfo, Apple clang 21.0.0, TBB, Python
  on, miso off); macOS 26.5.2 arm64 (M5 Max, 18 hardware threads).
- Fixture: RB-12's `rss-probe-2` scene — the public `quasistatic-semi` smoke
  (384-tet cube, 387 DOFs, `dhat 1e-3`, 4 quasistatic steps driving the top
  face down 0.25) over the 4-vertex / 2-triangle slab obstacle at
  `z = −0.02`; variants without contact and with the adaptive law and the
  BVH broad phase as in RB-12. Copied to `outputs/rb-24/20260920T150231Z/scene/`.
- Reproduced (`baseline/`, peak RSS from the manifest; `/usr/bin/time -l`
  agrees within 0.5 MB): contact off 37.6 / 43.1 MB at 1 / 18 threads;
  contact on 222.0 / **2427.3 MB** (RB-12: 222 / 2392 MB).

Evidence: `outputs/rb-24/20260920T150231Z/` — `summary.md`, `tool/`
(tracer, replay, A/B comparison), `trace/` and `trace-ti110/` (allocation
logs and their analyses), `lldb/` (query counts and the captured hard
queries), `matrix-baseline/` and `matrix-ti110/` (probe + the five smokes),
`repeat-ti110/` (the RB-12 matrix on the candidate), `bin/` (both
executables, SHA-256 in the manifests), the worktrees and `build/`.

## Method

Stage 1 needed no rebuild. `tool/mallocspy.c` is a `DYLD_INSERT_LIBRARIES`
interposer on `malloc` / `calloc` / `valloc` / `posix_memalign` / `realloc` /
`free` that logs every block of at least 1 MiB (256 KiB for the candidate)
with thread, timestamp and a raw backtrace, records per-thread totals of all
sizes, and writes the loaded-image table so `tool/analyze_trace.py` can
replay the live set over time and symbolize the peak's composition with
`atos`. TBB is linked statically and the binary is ad-hoc signed, so every
allocation goes through the system allocator and interposing is complete
(`/usr/bin/time` is SIP-protected and scrubs `DYLD_*` from its children, so
the binary is launched directly). `lldb/` holds batch scripts that break on
the root finder's entry and on its iteration-cap exit (by line in 1.0.6; by
module-relative address in 1.1.0, where the cap's line maps onto the per-
iteration test — a positive control at `max_iterations = 1000` fires 25
times) and read the query arguments.

## Findings

### F1 — the allocation is Tight-Inclusion's BFS queue on iteration-capped queries

Every large block in both runs comes from one site
(`trace/semi-t{1,18}.analysis.json`):

```
ticcd::split_and_push(...)                         interval_root_finder.cpp:522
  ← ticcd::interval_root_finder_BFS<true|false>            (Tight-Inclusion 1.0.6)
  ← ticcd::CCD<true|false>                                          ccd.cpp:213
  ← ticcd::vertexFaceCCD / ticcd::edgeEdgeCCD
  ← ipc::TightInclusionCCD::point_triangle_ccd / edge_edge_ccd
  ← ipc::Candidates::compute_collision_free_stepsize   (tbb::parallel_for)
```

It is the level-ordered BFS priority queue
(`std::priority_queue<std::pair<Interval3,int>, std::vector<…>>`, 104 bytes
per entry: three exact dyadic intervals of two 16-byte `NumCCD` each plus the
level). The queue holds the subdivision frontier; every refinement pops one
box and pushes at most two, so its size is bounded by the refinement count,
which PolyFEM caps at `solver/contact/CCD/max_iterations` = 1e6 (the
toolkit's unlimited `no_zero_toi` re-run never fires on this scene: all
~19,600 `ticcd::CCD` calls of the single-threaded run carry
`max_itr = 1000000`, `ms = 1e-4` — the toolkit's minimum-separation cap —
and `tolerance = 2.5e-7` = 1e-6 × the characteristic length). The vector
doubles 1.7 → 3.4 → 6.8 → 13.6 → 27 → 54.5 → 109 MB (exactly 2²⁰ × 104 B),
and at the last doubling the 54.5 MB predecessor is still live: **one query
at the cap costs 156 MB, transiently**. Nothing is retained — every block is
freed when the query returns (0 bytes live at exit in both traces).

| run (baseline) | ≥ 1 MiB blocks | live at the peak | threads with a live queue at the peak | peak RSS |
| --- | ---: | ---: | ---: | ---: |
| 1 thread | 107 | 156 MB (one query: 52 + 104 MB) | 1 | 222 MB |
| 18 threads | 258 | 1768 MB (5 × 156, 8 × 104, 3 × 52 MB) | 16 | 2427 MB |

The rest of the RSS (≈ 66 MB at 1 thread, ≈ 660 MB at 18) is the 37–43 MB
base plus the allocator's retention of the freed doubling predecessors;
sub-MiB allocations total ~10 MB per worker thread (7 GB of churn on the main
thread, 158 M calls, all small). The toolkit's own per-thread assembly
storage (`LocalThreadMatStorage`, `reserve(min(1e7, ndof))`) is 300 triplets
here — the plan's hypothesis does not appear in the trace at all.

### F2 — the hard queries are flat-on-flat contact against a coarse obstacle

`lldb/ccdspy-stats.json`, `lldb/cap-queries.json`: single-threaded,
**12 of ~19,600 queries hit the cap** (6 edge-edge, 6 vertex-face), each
36–75 ms. All are the cube's flat bottom in the trial step that drives it
≈ 0.031 down onto the slab 0.02 below (a uniform displacement
≈ (8e-3, 8e-3, −3.1e-2), impact at t ≈ 0.64): bottom edges parallel to the
slab's edges, and the bottom vertices (0,0,0) and (1,1,0) — later steps'
vertices near them — which project exactly onto the slab's diagonal `y = x`.
In these configurations the whole (u, v) overlap satisfies the inclusion
test at the impact time, the level-ordered BFS cannot prune, the frontier
grows geometrically until the cap, and the query returns the conservative
level toi with `output_tolerance > tolerance`. This is the geometry of every
object resting on a floor plane — the Houdini scenes' normal case — not a
pathology of the probe.

### F3 — threading multiplies the capped queries

18 threads: **36 cap hits** (18 ee, 18 vf) spread over all 18 workers
(`lldb/capcount-t18.json`). `Candidates::compute_collision_free_stepsize`
hands each query the earliest toi known when it *starts*
(`candidates.cpp:339`, `tmax = earliest_toi.load()`); serially the first
capped query lowers `tmax` to ≈ 0.64 and `t_upper_bound` prunes the later
hard pairs, in parallel the hard pairs all start at `tmax = 1`. So RB-12's
"~125 MB per thread" is the number of capped queries in flight times
104–156 MB, with wall time falling only from 1.83 s to 1.08 s because those
queries are the run's critical path. A capped query's conservative toi
depends on the `tmax` it started with, i.e. on scheduling — a candidate
source for the roundoff-level threaded path differences RB-12 characterized
(see F5).

### F4 — upstream has replaced the algorithm

Tight-Inclusion **v1.1.0** (2026-08-06, PR #9 "Bucket DFS") adds
`interval_root_finder_bucket_DFS` — a LIFO stack per exact `t.lower`, kept in
a `std::map`, always traversing the stack with the smallest `t.lower`; boxes
are tested in chronological order, the first box within tolerance returns,
memory is proportional to depth — and makes it the default
`CCDRootFindingMethod`; the BFS code is unchanged and still selectable.
Upstream ipc-toolkit pinned 1.1.0 on 2026-08-07 (`bb36e293`, "Update Tight
Inclusion to 1.1.0 (bucket DFS root finding)"). Our toolkit fork
(`ipc-toolkit-fork/cmake/recipes/tight_inclusion.cmake`) pins **1.0.6**. The
1.0.6 → 1.1.0 source diff is additive (the new finder, the enum value, the
dispatch case, the default argument); upstream reports identical TOI without
an iteration cap on two Scalable-CCD datasets and a better lower bound under
a cap.

### F5 — the candidate, measured (isolated worktree, not published)

Candidate = the worktree baseline with only the pin bumped to 1.1.0 (the
toolkit passes no method, so the bucket DFS is selected); incremental
rebuild; executables in `bin/` (baseline `0ba4b7ee…`, candidate `8322da06…`,
manifest `sources.ipc_toolkit.patch_sha256` `5f95dfa4…` = the one-line pin).

| | baseline (1.0.6 BFS) | candidate (1.1.0 bucket DFS) |
| --- | ---: | ---: |
| probe peak RSS, 1 thread | 221.9 MB | **94.2 MB** |
| probe peak RSS, 18 threads | 2386.5 MB | **577.9 MB** |
| probe wall, 1 / 18 threads | 1.83 / 1.08 s | 0.92 / 0.69 s |
| cap hits, 1 / 18 threads | 12 / 36 | **1 / 11** |
| largest block; live at the peak (18 threads) | 109 MB; 1768 MB on 16 threads | 48 MB; 364 MB on 7 threads |

The remaining capped queries hold their pending boxes in per-bucket
`std::vector<Interval3>` (96 B each): one bucket of 262k boxes (24 + 12 MB
during its doubling) single-threaded. The bound per capped query is now
proportional to the pending boxes rather than the whole frontier.

Five public smokes, one repeat each (`matrix-*/smokes`, `ab-baseline-vs-ti110.json`):

| scene | threads | peak RSS A → B (MB) | wall A → B (s) | solver path | ‖du‖∞ (last step) |
| --- | --- | ---: | ---: | --- | ---: |
| quasistatic-adaptive | 1 | 222 → 149 | 9.75 → 6.12 | identical | **0** |
| quasistatic-adaptive | default (18) | 2268 → 753 | 6.32 → 5.03 | differs at step 1 | 1.5e-16 |
| quasistatic-semi | 1 | 222 → 92 | 5.26 → 4.36 | identical | **0** |
| quasistatic-semi | default | 2360 → 543 | 4.43 → 4.15 | identical | 4.4e-16 |
| quasistatic-semi-alhess | 1 | 222 → 92 | 5.30 → 4.37 | identical | **0** |
| quasistatic-semi-alhess | default | 2419 → 591 | 4.49 → 4.24 | identical | 1.1e-16 |
| quasistatic-semi-friction | 1 | 222 → 92 | 5.55 → 4.46 | identical | **0** |
| quasistatic-semi-friction | default | 2501 → 559 | 4.71 → 4.32 | identical | 9.4e-16 |
| transient-semi | 1 | 220 → 150 | 5.32 → 4.38 | identical | **0** |
| transient-semi | default | 2620 → 558 | 4.49 → 4.11 | identical | 3.0e-16 |

Single-threaded, every smoke is **bit-identical** to the baseline (same
solver path, ‖du‖∞ = 0): on these scenes the capped queries' conservative
toi never decided an accepted step. Threaded, the differences are at the
roundoff floor RB-12 documented for threaded runs.

RB-12's repeat matrix on the candidate (`repeat-ti110/summary.md`, 5
repeats × {quasistatic-semi, quasistatic-semi-friction,
quasistatic-semi@dt=0.0625} × {default, 1 thread}, `--verify` clean): every
cell has **one solver path and ‖du‖ = 0 across its five repeats — the
threaded cells included**, and the threaded-versus-serial endpoints agree to
2.1e-16 / 6.7e-16 / 5.1e-16. RB-12 measured, on the same fixtures with the
BFS, threaded friction repeats spread over three paths with endpoints 1.7e-4
apart. The mechanism of F3 explains why the spread would shrink with fewer
scheduling-dependent capped queries; five repeats are a small sample (RB-12's
1.7e-4 branch appeared once in ten), so this is an observation, not a claim
that threaded runs are now reproducible. Peak RSS in that matrix: 511–922 MB
threaded (was 2066–2546 MB), 92–143 MB serial (was 212–226 MB).

## Decision: root-finding method

The plan's stage 2 asks for a bound or lazy sizing without changing any
result. The queue is already lazily sized and already bounded — by
`max_iterations`, at 156 B per iteration per query in flight — and the only
change that removes the cost is the algorithm. The options were put to the
user with what was measured; **the user chose no code change (2026-09-20):
the upstream solution (option 1) is the right remedy, to be taken with the
next dependency sync rather than as a fork-side pin bump now.** The options,
for the record:

1. **Adopt Tight-Inclusion 1.1.0 with its default bucket DFS** — the
   upstream fix, upstream's default since 2026-08-06 and ipc-toolkit's since
   2026-08-07; 4× less peak memory, 3× fewer capped queries, 8–37 % faster,
   bit-identical single-threaded on all five smokes and clean on the RB-12
   matrix. A CCD result change in general (a different toi sequence where a
   query would have been capped): the five smokes give no measurable
   evidence of one. Work: bump the toolkit fork's pin (one line), bump the
   PolyFEM pin to the new toolkit commit, note the Tight-Inclusion version in
   the manifest's library table, re-run the smokes, RB-05's resource tests
   and RB-02's probe, record. Recommended.
2. Bump to 1.1.0 but pass `CCDRootFindingMethod::BREADTH_FIRST_SEARCH` —
   bit-identical by construction, no memory gain; only useful as a
   stepping stone.
3. Fork Tight-Inclusion and pack the BFS entries (each interval is a dyadic
   child of [0, 1], so `(numerator, power)` per axis suffices: 104 → 32 B;
   exact integer arithmetic, so bit-identical): ≈ 3× less memory, no speed
   gain, a third fork to maintain against an algorithm upstream has
   superseded.
4. Document only: the bound `max_iterations × 156 B` per concurrent capped
   query, `Max Threads` as the workaround (the plan's meanwhile).

Not eligible under the boundary: lowering `max_iterations` (a contact
setting, changes the conservative toi), limiting concurrency of the narrow
phase (changes timing, not results, but a scheduler change for a memory
symptom the algorithm change removes).

## Validation

Stage 1 acceptance: the allocation is named (F1) with its scaling at 1 and
18 threads (F1, F3), on the plan's fixture, from the existing binary; the
baseline numbers reproduce RB-12's. Stage 2 acceptance (before/after on the
public smoke, every listed check bit-identical) is met by the candidate for
the single-threaded checks; by the user's decision the candidate is not
published, so the RB-05 resource tests and the RB-02 probe were not run on
it. Stage 3 (the new per-thread cost in the manifest evidence and RB-05's
record) reduces to the cross-reference added to RB-05's record: the cost is
the narrow phase's capped queries, outside the broad-phase budget.

Limits: one platform and allocator; the probe's hard queries are the
plan's fixture, not a survey of the HDA scenes (the mechanism — flat faces
and parallel edges against large obstacle triangles — is expected there);
`lldb` counts are per run and the candidate's 11 threaded cap hits are one
run's scheduling; 5 repeats in the matrix.

## Publication and reproducibility

No source change published. Record `docs/rb-24-validation.md`, the plan's
status row and section, the RB-05 cross-reference and the README rows:
`58a08a820` on `sdast9/polyfem:main` (2026-09-20). The worktrees under the evidence
directory are left at their clean base (`a6d70bd49` / `c24d803e` /
`bce32a39`); the candidate's one-line pin edit is kept as
`candidate-pin.patch` and both executables are in `bin/`. Reproduce stage 1 from `outputs/rb-24/20260920T150231Z/summary.md`
(tracer build line in `tool/mallocspy.c`; `lldb/*.lldb`); the A/B from
`configure-command.txt`, `build-baseline.sh`, `build-candidate.sh`
(pin edit in `ipc-toolkit-fork/cmake/recipes/tight_inclusion.cmake` of the
worktree) and `run-matrix.sh`, compared with `tool/ab_compare.py`.

## Next session handoff

**Done 2026-09-25** — see [Adoption](#adoption-tight-inclusion-110-2026-09-25).
Nothing pending under RB-24. The handoff as written on 2026-09-20: at the next dependency sync (the upstream
toolkit already pins Tight-Inclusion 1.1.0, so a toolkit sync brings the
bucket DFS with it): re-run the five smokes (single-threaded golden:
bit-identical expected, F5), `tools/rb05/run_scene_limits.py`, `tools/rb02`
and the RB-12 repeat matrix, add `tight_inclusion` to the manifest's
`libraries` table, and record the new per-thread cost here and in RB-05's
record. RB-12's threaded-friction limit may be revisited in the light of
F3/F5 then. Until the sync, a lower *Max Threads* on large floor-contact
scenes is the workaround (peak RSS ≈ base + capped queries in flight ×
156 MB).

## Adoption: Tight-Inclusion 1.1.0 (2026-09-25)

**Decision (user, 2026-09-25):** take option 1 now rather than at the next
dependency sync.

**Change.** IPC toolkit fork `75600955` (`semi-implicit-stiffness`): the
Tight-Inclusion pin 1.0.6 → 1.1.0 — the same one-line edit as F5's candidate
(`candidate-pin.patch`) — plus upstream `bb36e293`'s Python-binding change
(`BUCKET_DEPTH_FIRST_SEARCH` exposed and the default of
`ipctk.tight_inclusion.edge_edge_ccd` / `point_triangle_ccd`, so the bindings
match the C++ default; PolyFEM does not build them). The toolkit passes no
root-finding method, so every edge-edge and point-triangle CCD query uses the
bucket DFS. PolyFEM: the toolkit pin → `75600955`, and the run manifest's
`libraries` table gains `tight_inclusion` — the version CPM added, because the
library's own `project()` version reads 1.0.4 in both the 1.0.6 and the 1.1.0
tags (its `TIGHT_INCLUSION_VER` macro cannot tell them apart). `[run_manifest]`
checks that the entry is present and ≥ 1.1. No contact setting, tolerance,
`max_iterations`, thread default or resource limit changed.

**Evidence:** `outputs/rb-24-adopt/20260925T150323Z/` (not in the repository):
`bin/` (`PolyFEM_bin-baseline` `070c7b9b…` = the published `b92e2dd0a`
sources; `PolyFEM_bin-ti110` `e75ce40d…` = the pin change, on which the
matrices ran; `PolyFEM_bin-final` `550806ac…` = the published change, which
adds only the manifest entry — the probe's outputs are byte-identical to
`-ti110`'s), `seq-validate.sh` and its log, `probe-{A,B}/`, `smokes-{A,B}/`
with `ab-summary.md`, `repeat-B/`, `rb05-scene-limits/`, `rb02/`,
`unit-suite-final.log`, `hda-tests.log`. Shared build, all runs sequential, on
AC power.

| check | result |
| --- | --- |
| RB-24 probe (RB-12 `rss-probe-2`), peak RSS 1 / 18 threads | 222 → **92 MB** / 2,456 → **609 MB** (≈ 131 → ≈ 30 MB per added thread); wall 1.81 → 0.91 s / 1.01 → 0.69 s — F5 reproduced |
| five public smokes, 1 thread, baseline vs 1.1.0 | same solver path, ‖du‖∞ = **0** on all five; peak RSS 220–223 → 93–151 MB; wall −13 to −34 % |
| five public smokes, default threads | same path on four, `quasistatic-adaptive` differs from step 1 (‖du‖∞ 2.4e-16); ‖du‖∞ ≤ 7.3e-16 on all; peak RSS 2,246–2,562 → 511–823 MB |
| RB-12 repeat matrix (`quasistatic-semi`, `-friction`, `@dt=0.0625` × {default, 1} × 5, `--verify`) | 0 violations; five cells one path each (‖du‖ ≤ 1.6e-15); `quasistatic-semi@dt=0.0625` threaded took **two paths** from step 11 with endpoints 3.2–6.0e-16 apart; peak RSS 507–872 MB threaded, 93–144 MB serial |
| RB-05 `run_scene_limits.py` | every case exits as expected (sweep 3, unsupported 1, `bvh-automatic` 0; generous / default = unlimited) |
| RB-02 coefficient probe (`tools/rb02`) | **270/270** |
| full unit suite (`unit_tests-final`, own cwd) | 388 cases / **386 passed**, 5,177,857 assertions / 2 failed: the two known golden scenes, `multi-material/stretch-cubes` (pre-existing, RB-22) and `gcp-contact/cube-on-floor` (CI-06: ≈ 0.3 % H1 against the stored reference) |
| HDA tests (`houdini_HDAs/tests`, 13 files, Houdini 22.0.429 `hython`, shared build) | all **PASS** |

`gcp-contact/cube-on-floor` is not reproducible run to run under either
binary even though the golden harness runs it single-threaded (`cof/`, through
CI-03's `run_manifest_env`): H1 error 0.098335–0.098493 over five baseline runs
and 0.098284–0.098490 over four with 1.1.0 — the same spread and the same
failure, so the change is not visible there.

F5 found one path in every repeat cell; this matrix has one cell with two, so
the bucket DFS does **not** make threaded runs reproducible — it removes most
of the scheduling-dependent capped queries, and the branches that remain end at
the roundoff floor (RB-12 once measured 1.7e-4 between threaded friction
branches under the BFS). RB-12's threaded limit stands as written.

**Workaround retired.** A lower *Max Threads* is no longer needed for memory on
floor-contact scenes; the remaining per-capped-query cost is proportional to
the pending boxes of one bucket (F5), tens of MB, not 156 MB.

