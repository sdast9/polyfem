# R4 "production speedup" since EF-02/03 — attribution, 2026-09-28

Status: **attributed. Measurement only; no default, solver or scene changed.**
Question from [ef-07-trim-loop.md](ef-07-trim-loop.md), *Open*.

## Question

EF-02/03 ([ef-02-03-trim-controller.md](ef-02-03-trim-controller.md), binary
`6a59cb387`, 2026-09-25/26) measured the production trim controller
(`band_statistic: rms`, `initial_trim_estimate: false`) on R4 at 657 / 703
Newton iterations for one step and 1050 / 1110 for five. EF-07 (binary
`ef07-b` = EF-07 on `6a4e788bf`) recorded "production" R4 at 102 / 116 and
401 / 348. Which change between `6a59cb387` and `6a4e788bf` made production
~6× cheaper on R4?

## Answer

**None. EF-07's R4 "production" runs did not run the production controller.**

* The R4 scene file (`test_cases/uniax_mesh_constraintfloor_zero_b/input/params.json`)
  was re-exported from Houdini at **2026-09-26T20:18:23Z**, after EF-02/03's
  production R4 runs (started 2026-09-26T01:46). The re-export sets
  `solver/contact/semi_implicit/band_statistic: "force_weighted"` and
  `initial_trim_estimate: true`, and `continuation_max_ratio` 0 (the old
  export had 1.99). The mesh, selection and volume files are byte-identical;
  the other differences are PolySolve Newton keys written at their defaults.
* `tools/ef02/sequence.py --mode production` does not override the
  controller, so all four EF-07 production R4 runs (one- and five-step) ran
  with `band_statistic force_weighted`, `initial_trim_estimate true` and (the
  default of `ef07-b`) `collapse_guard_basis proxy` — each run's
  `run-manifest.json` records this under `input/effective`. That is exactly
  the EF-02/03 v3 candidate. Their trim-predictor streams leave trim 1 at the
  first observation through an `initial_estimate` event straight to 2^-12,
  like every v3 run; only the opt-in estimate emits that event.
* Only R4 is affected. EF-07's production R1, BBT, IT, BB and smoke runs, and
  the ball-burst `rms` runs, recorded `rms` / estimate off.
* The true production controller on the current code, on the exact EF-02/03
  R4 input, still takes **591 / 600** iterations (`6a4e788bf`), against
  **871** on `6a59cb387` rerun here (EF-02/03: 657 / 703). The ~110 vs ~650
  discriminant the bisection was meant to use does not exist between the two
  commits, so no bisection over them was needed.
* The code does make production **moderately** cheaper on R4 (≈ 590–670 vs
  657–871): it leaves trim 1 earlier (212–291 accepted iterations vs 314–459).
  `c133948cf` — after the EF-02/03 code, the elastic-smoothing merge and the
  PolySolve and IPC upstream adoptions, before the band weighting — behaves
  like `6a59cb387` (744 / 719 iterations, leaves trim 1 after 379 / 352).
  The earlier exit therefore comes with `6a4e788bf`, the **collision-weighted
  band statistic**. Its other change, PolySolve `6a8c2cc9` (Armijo refuses an
  uphill direction), never fired on these runs.

## Method

Evidence: parent workspace `r4-bisect-work/` (local; historical `outputs/`
is being archived, see AGENTS.md). Binaries are existing immutable copies,
hashes in `r4-bisect-work/bin/hashes.txt`:

| name | PolyFEM | IPC | PolySolve | source / SHA-256 |
|---|---|---|---|---|
| A | `6a59cb387` | `75600955` | `448f1b8e` | EF-02/03 `bin/PolyFEM_bin-production-6a59cb387`, `550806ac…` |
| C | `c133948cf` | `cf99893b` | `6099b9cd` | band-statistic `PolyFEM_bin-baseline-c133948cf`, `fbb861d5…` |
| B | `6a4e788bf` | `cf99893b` | `6a8c2cc9` | EF-07 `bin/PolyFEM_bin-ref-6a4e788bf`, `a6ff0656…` |

Tight-Inclusion is 1.1.0 at both IPC pins (`75600955` and `cf99893b`), so
it is not a candidate. The companion pins are those of `cmake/recipes/` at
each commit.

Inputs are the saved `input.json` of EF-02/03 `production-R4-s1-r1` ("old",
export 2026-09-22) and of EF-07 `production-R4-s1-r1` ("new", export
2026-09-26), rerun byte for byte except the output directory and the stated
pointer edits (`r4-bisect-work/run_template.py`; EF-01 row format, so
`tools/ef01/ef01_reduce.py` applies). One step, all threads, `debug` log,
`coefficient-events.jsonl` discarded, as in the EF-02/03 driver. Runs were
sequential; the host was also running the `outputs/` transfer, so wall
times are not compared. `r4-bisect-work/leave_trim1.py` reports when the trim
first leaves 1 and the band statistics there.

## Results (R4, one step)

Accepted Newton iterations over all sub-solves; "leaves 1" = accepted
iterations before the trim first changes; "walk" = accepted iterations after.

| binary | input / controller | iterations | leaves 1 after | via | walk | end trim |
|---|---|---:|---:|---|---:|---:|
| A (EF-02/03 record) | old, rms, cont. 1.99 | 657 / 703 | 314 / 371 | band step | 332 / 319 | 1.1e-3 / 4.9e-4 |
| A (rerun) | old, rms, cont. 1.99 | 871 | 459 | band step | 399 | 2.4e-4 |
| C | old, rms, cont. 1.99 | 744 / 719 | 379 / 352 | band step | 354 / 354 | 1.1e-3 / 5.1e-4 |
| B | old, rms, cont. 1.99 | 591 / 600 | 212 / 248 | band step | 369 / 341 | 1.2e-4 / 2.8e-4 |
| B | new, rms, estimate off (cont. 0) | 668 | 291 | band step | 366 | 2.9e-4 |
| B | new, rms + estimate | 122 | 0 | `initial_estimate` → 2^-12 | 120 | 1.2e-4 |
| B | new, force-weighted, no estimate | 178 | 19 | `force_band` → 0.81 | 156 | 4.1e-4 |
| `ef07-b` (EF-07 "production") | new as exported: fw + estimate + proxy | 102 / 116 | 0 | `initial_estimate` → 2^-12 | 100 / 114 | 2.4e-4 / 2.2e-4 |

All runs exit 0; every `physical_balance_pass` is false, as in EF-02/03 and
EF-07. Continuation (1.99 vs 0) is not a material factor: B on the new input
with the production controller (668) is within the old input's spread.

Each opt-in alone removes most of the cost (estimate alone 122, force band
alone 178; together 102 / 116 in EF-07, 99 / 107 for v3 in EF-02/03). The
initial estimate removes the plateau at trim 1 and most of the walk (one
seed to 2^-12 before the first iteration). The force-weighted band leaves
trim 1 at accepted iteration 19, when its force-weighted mean gap reaches
0.56 d̂, and then steps about every 10 iterations by proportional factors
(0.81, 0.63, 0.54, 0.47, …) instead of halving at most once per 30.

## Mechanism of the moderate production change

The production controller steps the trim down (÷2, at most once per
`controller_interval` = 30 iterations) only on an iteration whose band
statistic exceeds `sqrt(trim_upper) dhat` = 0.9487 d̂. On R4 the statistic
starts at ≈ 0.77–0.79 d̂ and climbs slowly while contact spreads (17 → ~2,000
active collisions at trim 1); the downward step fires on the first iteration
where it crosses the edge, after which the walk to 2^-12…2^-13 costs ~320–400
iterations on every binary.

`6a4e788bf` changed the statistic from the count-based mean over collisions
(`compute_avg_distance`) to the collision-weighted `sum(w d²)/sum(w)`
([band-statistic-weighting-20260927.md](band-statistic-weighting-20260927.md)).
In the trim-predictor stream, which since `6a4e788bf` records both (`gap.rms`
count-based, `gap.band_rms` weighted), on the same B trajectories:

* the weighted value starts 0.021 d̂ **below** the count mean (0.766 vs 0.786
  at the first refresh) and ends the trim-1 plateau 0.009–0.014 d̂ **above**
  it;
* the trim leaves 1 when the weighted value crosses 0.9487 (0.9492–0.9501 at
  the decision), while the count mean is 0.936–0.940 there and never reaches
  the edge during the trim-1 plateau (maximum 0.935–0.939); on these
  trajectories the count mean first exceeds the edge only at accepted
  iteration 331 / 385 / 430, after the weighted trim walk had begun.

So the weighted statistic lets the downward step fire earlier (212–291 vs
314–459 accepted iterations at trim 1), because late in the plateau the
heavier-weighted collisions sit at larger gaps than the count mean suggests.
It does not change the walk, which remains the dominant cost.

Across binaries on the same input (accepted iterations at trim 1 / after):
`6a59cb387` 314, 371, 459 / 332, 319, 399; `c133948cf` 379, 352 / 354, 354;
`6a4e788bf` 212, 248 (291 on the new export) / 369, 341 (366). Only the time
at trim 1 moves, and it moves at `6a4e788bf`. Run-to-run spread is large
(R4 is multi-threaded; one-step totals 657–871 on one binary), so the size of
the saving is ≈ 100–200 iterations, not a precise figure.

PolySolve's uphill-direction refusal (`6a8c2cc9`) is the other change in
`6a4e788bf`. It logs no refusal in any run here. The "not a descent
direction … reverting to SparseProjectedNewton" lines are Newton's existing
descent screen, present on all three binaries (2–5 per run).

## Consequences for existing records

* EF-07's R4 production column (one step 102 / 116, five steps 401 / 348),
  its statement that production "is no longer the slow reference on R4", the
  cost gate's "pair ≤ production everywhere except R4", and the R4 accuracy
  comparison "against production's repeat spread" all refer to v3-mode runs:
  the R4 "production" rows are two further v3 repeats (a third and fourth
  beside EF-07's own v3 column), and the "production repeat spread" on R4 is
  v3's spread. The ball-burst and other-scene production statements are
  unaffected. [ef-07-trim-loop.md](ef-07-trim-loop.md) carries a correction
  note; its measured numbers are left as recorded.
* The case for the force-weighted mode on R4 is therefore unchanged from
  EF-02/03: the production controller is still slow on R4 (≈ 600 iterations
  for step 1 on the current code).
* The scene file now selects opt-in experimental controller options. Any
  measurement that means "production" on R4 must set
  `band_statistic "rms"` and `initial_trim_estimate false` explicitly (or use
  the old export); `tools/ef02/sequence.py` production mode does not. Not
  changed here (tooling decision).

## Not done

Five-step runs were not repeated: the one-step result already excludes a
code cause for the ~6× change. The commits between `6a59cb387` and
`c133948cf` (EF-02/03 code, elastic-smoothing merge, PolySolve and IPC
upstream adoptions, restart work) were not bisected individually. Taken
together they leave production on R4 in the old range (the C row), so no
single one among them can account for a large change. No binaries were
built: all three were existing immutable copies with recorded hashes and
provenance.
