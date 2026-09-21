# CI-03 — Pin the intended friction policy in historical fixtures

Date: 2026-09-21
Status: **done — validated within stated scope** (2026-09-21). The three
scene fixtures that started failing their stored references between the
last green macOS Release scene run (`91c7ff8ac`) and the RB-10 defaults
commit (`756070f44`) are explained entirely by the lag budget
(`solver/contact/friction_iterations` 1 → 2); each now states the policy its
reference was generated under, each has a current-defaults twin with a
harness-generated reference, and the historical and the current-default
coverage both pass. Item of the
[CI portability plan](ci-portability-plan.md#ci-03--pin-the-intended-friction-policy-in-historical-fixtures).

## Contract and authorization

- **User selection (2026-09-21):** "Let's address CI-03". The plan's required
  work is followed as written: (1) reproduce the fixtures on isolated copies at
  the before/after revisions; (2) compare omitted / explicit 1 / explicit 2 at
  the current revision with the resolved input and the friction diagnostics;
  (3) encode the recovered historical settings in the fixtures and add
  separate tests for the approved current defaults; (4) preserve the
  production defaults, generate new current-policy references only with
  independent numerical checks and a reviewed change, never bulk-regenerate.
- **Decisions taken by the user this session:** the fixture changes are
  published on a fork of the data set, `sdast9/polyfem-data`, branch
  `fable-fixtures` (asked 2026-09-21; the alternative was an in-repo overlay
  mechanism in the harness). Nothing else was decided: the RB-10 defaults
  stand (`friction_iterations: 2`, `friction_lag: realized_force`) and the
  audit's statement that it does not reopen that decision is honoured.
- **Not authorized / not done:** no solver change, no default change, no
  reference of an existing fixture altered (the three historical fixtures
  keep their stored values digit for digit), no change to CI-04/05/06's
  fixtures, no Teseo or private scene.

## Baseline and reproduction

- PolyFEM `main` at `11cdf476f` (clean), IPC Toolkit `482b9eab`
  (`semi-implicit-stiffness`), PolySolve `bce32a39` (`iteration-callback`),
  data pin `polyfem/polyfem-data@e0efb6b` at session start.
- **CI evidence read first:** [Build run 35584793121](https://github.com/sdast9/polyfem/actions/runs/35584793121)
  (`303b54cc0`, 2026-09-21): Linux Release and macOS Release fail exactly the
  four CI-03–CI-06 groups; the `LastTest.log` artefacts give the three
  fixtures' computed metrics on both platforms
  (`outputs/ci-03/20260921T142215Z/ci-35584793121/`), identical to the
  2026-09-13 audit's values — macOS digit for digit, Linux to ≤ 6e-11
  relative. The mismatch is deterministic and platform-independent.
- **Harness hook (`tests/verify_run.cpp`):** `run_data` now delegates to
  `run_manifest(manifest_path, data_dir)`, and the hidden case
  `run_manifest_env` (`[.][run_env]`) runs any manifest against any data
  directory named by `POLYFEM_RUN_MANIFEST` / `POLYFEM_RUN_DATA_DIR`, so an
  isolated fixture copy goes through exactly the scene harness (recorded test
  duration, the harness's linear-solver selection, one thread, the same
  authentication). The harness also logs `Computed tests: {...}` for every
  fixture now (before, a passing fixture left no trace of its values in a CI
  log). Nothing else in the harness changed.
- **Isolated builds** (`outputs/ci-03/20260921T142215Z/`, RelWithDebInfo,
  AppleClang 21, TBB, `POLYFEM_WITH_TRIANGLE=ON` — the P4 ball's irregular
  collision tessellation needs it, which is why CI runs it in
  `triangle_data`; the shared `polyfem/build` has Triangle off and was not
  reconfigured): `build-before` = `91c7ff8ac` + the harness patch, companions
  as worktrees at that revision's pins (IPC `bb795446`, PolySolve
  `ee5b296a`), `unit_tests` sha256 `9de06486…`; `build-after` = `11cdf476f` +
  the harness patch + the manifest lines, companions at the current pins,
  `unit_tests` `b4a9a845…`. Configure commands: `configure-command-*.txt`.
- **Runner:** `tools/ci03/ab_runner.py` copies a fixture, its complete
  `common` chain and every referenced mesh to a private data root (the
  `default` copy is byte-identical to the pinned file — checked by hash), writes
  the variant's overlay into the copied top-level file only, runs
  `run_manifest_env` in the run's own directory (`sim.json`, `sim.pvd`,
  `run.log`, `run.json` with hashes stay there) and reads the six metrics, the
  harness verdict, the lag re-solves from the fixture's own `sim.json` and,
  for `-diag` variants (`output/physical_diagnostics` + `output/manifest`),
  the per-step lagging record and the manifest's resolved input.
  `tools/ci03/compare.py` compares run sets; `tools/ci03/make_fixtures.py`
  writes the fixture changes and generates the twins' references through the
  harness; `tools/ci03/summarize.py` writes the compact
  [results-20260921.json](../tools/ci03/results-20260921.json).

## Findings

### The A/B (plan steps 1 and 2)

Six harness metrics per run; `max rel` is the largest relative difference
over the six against the fixture's stored reference (the harness's own
normalisation, margin 1e-5).

| Fixture | before `default` (= 1) | before `fi1` | after `fi1` | after `default` (= 2) | after `fi2` | before `fi2` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `2D/large-ratios/large-mass-ratio` (μ .1, 120 steps) | 5.247e-12 ✅ | ≡ | ≡ | 6.225e-03 ❌ | ≡ | ≡ |
| `3D/large-ratios/large-mass-ratio` (μ .5, 120 steps) | 7.734e-10 ✅ | ≡ | ≡ | 3.167e-01 ❌ | ≡ | ≡ |
| `3D/higher-order/ball-bounce/P4-dt=0.01` (μ .2, BDF2, 20 steps) | 3.386e-13 ✅ | ≡ | ≡ | 4.408e-01 ❌ | ≡ | ≡ |

`≡` = all six metrics bit-identical to the column on its left within its
group (`compare.md`): the before-revision default, the before-revision
explicit 1 and the current-revision explicit 1 are one number set; the
current-revision default, explicit 2 and the before-revision explicit 2 are
another, and that second set is the CI value set on both Release lanes.
Hence (a) the whole mismatch is the lag budget; (b) the RB-10 code that
entered with `7cb267f18` is inert on these classic adaptive-IPC scenes (its
realized-force lag is gated on semi-implicit stiffness), and no other commit
between `91c7ff8ac` and `11cdf476f` touches these fixtures' numbers; (c) the
new default is exercised on 117 of the 120 steps of both mass-ratio scenes
(one lag re-solve each) and on 4 of the 20 ball steps — the 3 / 16 other
steps met the lag tolerance after the first solve (`sim.json` `rc` rows).
The `-diag` twins of `default` and `fi1` reproduce their metrics exactly
(the diagnostics and manifest are observational); their manifests' resolved
input shows `solver/contact/friction_iterations` 2 / 1, `barrier_stiffness`
`adaptive`, the fixtures' μ, and the complete `common` chain
(`contact/examples/common.json`, plus `3D/higher-order/common.json`, `P1.json`
and `P4.json` for the ball) pins no friction key anywhere.

### What the budget does on these scenes (the checks behind the new references)

Budget sweep at the current revision (`fi4`, `fi8`, `fiinf` = iterate to the
tolerance), with the RB-04 per-step record on: `resid` is the largest
updated-lag free residual over the run (objective units; tolerance
`friction_convergence_tol · L` = 2.5e-4 / 2.5e-4 / 1e-4), `D_cum` the
cumulative `frictional_dissipation_increment`, `balance` the steps whose
`physical_balance_pass` holds, and the distances are relative to the
converged-lag run (budget 8 = ∞ everywhere: the sequence has converged).

| Fixture | budget | lag re-solves | resid max | states converged | balance pass | `err_h1_semi` | max rel distance to converged (6 metrics) | `D_cum` (rel. to converged) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 2D mass ratio | 1 | 0 | 10.2 | 3/120 | 4/120 | 0.146352 | 6.7e-3 | 1118.7 (+15.2 %) |
| | **2** | 117 | 0.446 | 28/120 | 43/120 | 0.145441 | 4.2e-4 | 960.7 (−1.1 %) |
| | 4 | 249 | 0.0447 | 100/120 | 118/120 | 0.145375 | 9.7e-5 | 972.7 (+0.13 %) |
| | ∞ (6) | 271 | 2.5e-4 | 120/120 | 120/120 | 0.145380 | 0 | 971.5 |
| 3D mass ratio | 1 | 0 | 40.5 | 4/120 | 3/120 | 0.051938 | 2.2e-2 | 100.2 (+7.4 %) |
| | **2** | 117 | 6.57 | 75/120 | 102/120 | 0.035492 | **3.3e-1** | 73.3 (**−21 %**) |
| | 4 | 224 | 0.106 | 102/120 | 107/120 | 0.053144 | 7.4e-4 | 93.0 (−0.3 %) |
| | ∞ (8) | 261 | 2.4e-4 | 120/120 | 119/120 | 0.053104 | 0 | 93.3 |
| P4 ball | 1 | 0 | 1.5e-3 | 16/20 | 15/20 | 1.313e-3 | 8.3e-1 | 7.17e-3 (+35 %) |
| | **2** | 4 | 1.7e-4 | 19/20 | 16/20 | 7.343e-4 | 2.3e-2 | 1.45e-2 (+172 %) |
| | 4 = ∞ | 7 | 9.6e-5 | 20/20 | 16/20 | 7.177e-4 | 0 | 5.33e-3 |

Reading, per fixture:

- **2D mass ratio:** the correction is monotone in everything measured; the
  current default sits 4e-4 from the converged lag against 7e-3 for the
  historical single solve, with the dissipation within 1.1 % (the single
  solve overestimates it by 15 %).
- **3D mass ratio (μ .5, a 10,000 kg/m³ block on a 1,000 kg/m³ cube on a
  steel slab):** the lag loop's own criterion improves as designed (residual
  40 → 6.6, balance 3 → 102 steps), but the **second iterate overshoots the
  endpoint**: the budget-2 trajectory ends 33 % from the converged lag in
  `err_h1_semi` and dissipates 21 % less, whereas the historical budget 1 is
  2.2 % / +7.4 % away and budget 4 is 7e-4 / −0.3 %. The lagged fixed point
  is reached, but not monotonically on this scene. This is a property of
  the finite lag under the RB-10 default — recorded here as an observation
  for RB-10 (whose fixtures converged geometrically), not repaired and not a
  reason to change the reference policy of this item.
- **P4 ball (BDF2):** the correction removes most of the 83 % single-solve
  error; budget 2 is 2.3 % from the converged lag and budgets 4 and ∞
  coincide. Step 15 (the rebound) has a *negative* dissipation increment at
  every budget (−8.3e-4 / −2.8e-4 / −2.4e-4 against increments of 1e-3–1e-2
  on the neighbouring steps): RB-04's increment is `g_f · Δx` with the step
  increment, while under BDF2 the friction potential opposes the BDF2 velocity
  `(3x_{n+1} − 4x_n + x_{n−1})/(2Δt)` — the convention RB-10 documented on a
  decelerating step ("a held body after a +s step sees v = −s/2"); the
  toolkit identity `g · v ≥ 0` is on the potential's velocity, not on `Δx`.
  Observation for RB-04/RB-10 (the convention under multistep integrators),
  no change.

So the generated current-policy references pass these checks: they are
deterministic and platform-stable (my macOS values = the macOS lane's digit
for digit and the Linux lane's to ≤ 6e-11, six orders inside the 1e-5
margin), they are explained entirely by the budget (bit-identical to the
before-revision code at budget 2), and their place on the budget sequence is
measured above — in 2D and for the ball the current default is closer to the
converged lag than the historical single solve, in 3D it is not. They are
regression references for the current policy, like every other golden of the
set; none is a physical certification (RB-09's limits apply).

### The fixture changes (plan step 3) — `sdast9/polyfem-data@e6ed5cf`, branch `fable-fixtures`

- The three historical fixtures gain `"solver": {"contact":
  {"friction_iterations": 1}}` (a text insertion that keeps each file's
  formatting; the stored `tests` values are untouched). Nothing else in the
  set is pinned: every other scene of `contact_2d`, `contact_3d` and
  `triangle` authenticates at budget 2 within its margin (CI and the group
  runs below), so its reference does not discriminate the policy and its
  intent is not asserted here.
- Each gets a twin `<name>-friction-defaults.json`: `common` → the fixture,
  `patch: [{"op": "remove", "path": "/solver/contact/friction_iterations"}]`,
  and a `tests` block generated by the harness itself (`*` manifest line on an
  isolated copy, `make_fixtures.py`, `outputs/…/fixtures/generate/`). The twin
  therefore runs at the solver's *current default*, whatever it is — a future
  default change fails it and forces a reviewed regeneration (the situation
  this item repairs, made deliberate) — and the `remove` throws if the parent's
  pin ever disappears. `time_steps` is inherited (`"all"` / 20).
- `cmake/recipes/polyfem_data.cmake`: `GIT_REPOSITORY` →
  `https://github.com/sdast9/polyfem-data`, `GIT_TAG` → `e6ed5cf2d6514ef2595d28a23400521e2b3c717c`
  (`--build_info` / the run manifest report it as `test_data_declared_pin`);
  the twins are listed after their parents in `tests/contact_2d.txt`,
  `tests/contact_3d.txt` and `tests/triangle.txt`.
- Data-set convention (as for the two companions): the fork's `main` mirrors
  upstream `polyfem/polyfem-data` (`e0efb6b` = upstream `main` on
  2026-09-21); the fork's fixture changes live on `fable-fixtures`, pinned by
  SHA. Publish a data change there before advancing the pin; a local edit to
  `polyfem/data/` never reaches a clean CI clone.

## Validation

| Check | Input / configuration | Expected | Measured | Result |
| --- | --- | --- | --- | --- |
| Harness hook | `unit_tests --list-tests "[run_env]"`; the six-fixture manifest on the branch (below) | the case exists, hidden; missing variables fail by `REQUIRE` | listed as `run_manifest_env [.][run_env]`; runs | pass |
| CI cross-check | local `default` metrics vs run 35584793121 `LastTest.log` (macOS, Linux Release) | within the 1e-5 margin | macOS identical; Linux ≤ 6e-11 | pass |
| Before/after A/B | 3 fixtures × {`default`, `fi1`, `fi2`} on `build-before`; × {`default`, `fi1`, `fi2`, `fi4`, `fi8`, `fiinf`, 4 `-diag`} on `build-after` (39 runs, all exit 0 or the harness's 42 on a violated reference, no solver failure) | explicit 1 recovers the references; the two number sets as stated | as tabulated; `fi1` 5.2e-12 / 7.7e-10 / 3.4e-13 | pass |
| Six fixtures on the branch | `run_manifest_env`, data = the `fable-fixtures` checkout, authenticate mode (`validate/six-fixtures/run.log`) | 6/6 authenticated | 6/6 ✅, 10 assertions | pass |
| Data classification | `unit_tests "all PolyFEM data JSON files are classified"` with the new manifests | every JSON classified exactly once, twins included | 1,489 assertions, pass | pass |
| `contact_3d` | `build-after`, live data checkout on the branch | every fixture authenticates | **50/50 authenticated** (49 + the twin), `All tests passed` — green for the first time since `756070f44` | pass |
| `contact_2d` | same | only `gcp-contact/cube-on-floor/run.json` (CI-06) fails | 28/29 authenticated (27 + the twin); `cube-on-floor` violates by 0.0037 in `err_h1_semi` (0.0030 on the macOS lane and 0.0049 on the Linux lane of run 35584793121 the same day — CI-06's cross-platform repeat is visibly needed) | pass (CI-06 unchanged) |
| `triangle_data` | same | only `contact/examples/3D/higher-order/microstructure.json` (CI-05) fails | 6/7 authenticated (P1–P4, `P4-dt=0.01` and its twin); `microstructure` refused by the RB-05 budget (65,153,532 emissions > 50,000,000) | pass (CI-05 unchanged) |
| Dissipation sign | the 8 `-diag` runs | `frictional_dissipation_increment ≥ 0` per accepted step | ≥ 0 at every step of both mass-ratio scenes at every budget; the ball's step 15 negative at every budget (BDF2 convention, above) | pass / observation |
| Formatting / diff | `clang-format` on `verify_run.cpp`, `git diff --check`, JSON parse of the fixtures, `py_compile` of the tools | no issue | clang-format 21.1.8 (the audit's venv) and 23.1.1 report `verify_run.cpp` clean; `git diff --check` clean; the six fixtures parse; the four tools compile | pass |
| Native lanes | the Build run of this publication | Linux Release and macOS Release: `contact_3d` green, `contact_2d` and `triangle_data` failing only on CI-06 / CI-05, the `[data]` case green with the fork's data pin | read after publication (see the plan's implementation update) | pending at publication |

- Not performed: Windows (its Release lane does not run the scene groups —
  CI-07 item 3), the `slow` group, any private scene; no threaded run (the
  harness is single-threaded by construction; RB-12's threaded friction
  limit of 1.7e-4 is not exercised).
- Retained failures: none of this item's runs failed unexpectedly; the two
  expected group failures are CI-05 and CI-06, unchanged.
- Tolerances: the harness's 1e-5 margin, untouched; no test tolerance
  changed.

## Publication and reproducibility

- Data: `sdast9/polyfem-data` created 2026-09-21 as a fork; commit `e6ed5cf`
  on `fable-fixtures` (3 fixtures edited, 3 added).
- PolyFEM: `tests/verify_run.cpp` (hook + metric logging), the three manifest
  lines, `cmake/recipes/polyfem_data.cmake`, `tools/ci03/`, this record, the
  plan/README/RB-10 notes — commit `bf6ea5c58` on `sdast9/polyfem:main` (this hash note is a documentation follow-up).
- Evidence: `outputs/ci-03/20260921T142215Z/` (worktrees, both builds, 39 A/B
  run directories with `sim.json`/`run.log`/`run.json`, the CI artefacts, the
  validation logs; not distributed). Reproduction:
  [tools/ci03/README.md](../tools/ci03/README.md).
- Shared `polyfem/build`: only `unit_tests` was rebuilt (the harness hook);
  its configuration is unchanged (Triangle off).

## Next session handoff

- CI-03 is done. The plan's work order continues with CI-04 (mixed P1/P2
  tetrahedral collision boundary, `standard`), CI-05 (microstructure vs the
  RB-05 budget, `triangle_data`) and CI-06 (cube-on-floor reference,
  `contact_2d`); CI-06's data fix now has its writable home.
- Observations handed to other items (no action taken): the 3D mass-ratio
  overshoot of the budget-2 lag iterate (RB-10: the fixed point is not
  approached monotonically on every scene; a per-scene `-1` budget or a
  tolerance-based default would be the alternatives to measure) and the BDF2
  sign of RB-04's `frictional_dissipation_increment` (RB-04/RB-10).
- Harness: `run_manifest_env` is available to every later scene item.

## Progress log

Append-only. Newest entry last.

- **2026-09-21 14:10Z** — Session start on `11cdf476f`. Read the plan's CI-03
  section, the evidence record, RB-10's record and the harness; read the
  2026-09-21 Build run's `LastTest.log` artefacts (the three fixtures'
  values identical to the audit's on both Release lanes).
- **14:22Z** — Harness hook written and built in the shared build; first
  matrix on the shared build: `fi1` recovers the 2D/3D references, the P4 ball
  needs Triangle. Before/after worktrees created (`91c7ff8ac`, `11cdf476f`,
  companions at their pins), both built with Triangle on (2.5 / 4.5 min).
- **14:33Z** — Full matrices: 9 runs before, 30 after; the two number sets;
  the budget sweep and the diagnostics (3D overshoot, ball step-15 sign).
- **14:35Z** — User decision: fork the data set. `sdast9/polyfem-data`
  created; branch `fable-fixtures`; `make_fixtures.py` pinned the three
  fixtures and generated the twins (`e6ed5cf`); six fixtures authenticate;
  manifests, pin, `[data]` classification; the three scene groups run.
- **14:55Z** — Groups read: `contact_3d` 50/50, `contact_2d` 28/29
  (cube-on-floor, CI-06), `triangle_data` 6/7 (microstructure, CI-05).
  Record, plan, RB-10 note, READMEs written; formatting checked with the
  pinned clang-format; publication.
