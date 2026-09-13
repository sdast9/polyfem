# Closed RB review follow-up — 2026-09-12

Status: **validated within the bounded review scope**. The user accepted the review's recommendation to repair its four reproduced defects and correct the RB-03 algebra wording. This corrects those bounded parts of RB-18, RB-19 and RB-21; it does not select a new coefficient or interpolation model.

## Evidence and isolation

The pre-fix review is in parent-workspace `outputs/rb-review/20260912T154256Z/`: actual Armijo/RobustArmijo accept an uphill step to a strict local maximum; the AL no-change guard rejects progressing soft restarts; global curvature fallback overflow deletes a barrier; two finite parent coefficients overflow their weighted mean. All four have controls. Existing focused regressions passed (35 cases, 2,202 assertions), demonstrating the coverage gaps.

The work resumed after RB-22 and the RB-04 remainder were published. The isolated PolyFEM checkout starts at `d842f4f06`, PolySolve at `5afe3b5d4`, IPC at `e3c8d3fe`. Fresh evidence: parent-workspace `outputs/rb-review-fixes/20260912T201345Z/`. Its manifest and incoming patch preserve the ongoing RB-05 containment tests/probes. That work and the shared build are excluded from these edits. Earlier smokes under `20260912T160000Z` belong to the earlier source/binary and are historical evidence only.

## Bounded changes and acceptance

1. RB-19: require a finite energy-change bound even in the small-gradient regime, using the existing characteristic energy scale to accommodate cancellation. Reject the nonconvex uphill counterexample; preserve equal-energy progress and the original public step-1 regression without changing tolerances.
2. RB-18 F5: compare the next restart's iterate with the just-completed attempt's start after rollback. Continuing soft progress is allowed; identical problems still terminate and restart budgets retain their meaning.
3. RB-18 F7: preserve global-fallback overflow for the existing batch cap/error path; never convert it to zero. Invalid/underflowing positive fallback arithmetic reports a named error.
4. RB-21: compute a representable parent weighted mean without overflowing intermediate sums; validate the final assigned scale after batch resolution. A further test exposed IPC's `weight * scale` intermediate: if that product is not representable, report a named error before assignment, even when multiplication by the small barrier value could make the exact energy finite. Reordering all IPC potential/derivative arithmetic is outside this bounded repair.
5. RB-03: correct the claimed equivalence of condensation and the gap-normalized force-direction fallback; retain the chosen law.

Rebuild in isolation; run focused regressions, affected PolyFEM tests, five public solver smokes, original step-1 default/single-thread repeats, and the HDA pipeline using that binary. Preserve every failed/incomplete run. Review the final diff, publish companion PolySolve commit and PolyFEM pin/fixes, and verify that RB-05 edits remain intact. CCD, retired floor state, trial cap, numerical stopping contracts, and research/model decisions remain as recorded.

## Progress

- Four failures reproduced before editing; isolation and incoming RB-05 state recorded.
- Production fixes, regressions and current validation are complete; source publication is to the existing fork branches below.

- Initial affected-suite run found that the corrected mean still overflowed IPC's weighted-coefficient intermediate in the 1e308 fixture. Added a named PolyFEM boundary error; the initial failed test evidence is retained. IPC is unchanged, preserving the separate RB-05 work.

## Final validation and publication

Compact, committed evidence: [results JSON](rb-review-followup-20260912-results.json). Raw inputs, logs, binary hashes, commands, source snapshots and earlier failures remain in the isolated parent-workspace evidence directory above. The `published-controls` binaries replace the four changed production translation units with `git show HEAD:<path>` snapshots of the starting commits and link the same unchanged support libraries as the isolated full build. Archive hashes are stable. This re-reproduces all four original failures on the updated RB-04/RB-22 foundation; it does not rely on the old shared binary.

| Check | Result |
| --- | --- |
| Isolated full CMake build, arm64 RelWithDebInfo/TBB | `PolyFEM_bin` and `unit_tests` build successfully; IPC and PolySolve use isolated source overrides |
| Affected PolyFEM suite | 63 cases, 4,229 assertions, exit 0; includes restart/convergence, contact cache/mapping, coefficient/continuation, derivatives, physical diagnostics/event accounting and the retired floor |
| PolySolve related suite | 4 cases, 534 assertions, exit 0; includes the line-search subset (2 cases, 49 assertions) |
| Public smokes | 5/5 complete, five saved states each, exit 0 and zero error lines; maximum endpoint component difference from the published controls `7.2945122164824738e-16` |
| Classic adaptive single-thread control | Exactly identical solution bytes (maximum endpoint difference 0) |
| Original step-1 stall fixture | 10 default-thread + 3 single-thread runs, 16 timesteps/17 saved states each, every exit 0 and zero restarts; default-thread fallback accepts 5–9, single-thread 10 each |
| HDA end-to-end | Exit 0; strict input, solver fallback and control import/export checks pass using the isolated binary |

The nonconvex fixture now takes `alpha=.125` in both Armijo variants instead of the uphill full step. The progressing quartic with an unchanged coefficient callback converges after 14 restarts to `x=7.822642576269849e-5`, final gradient `4.786967314877977e-13`, with zero unchanged restarts counted. Global-fallback overflow without a reference now throws; with a finite batch it uses the cap. The parent-mean fixture now stops with `overflowing weighted collision coefficient` when IPC's required intermediate is unrepresentable. The smaller controls remain finite.

The initial affected run's two assertions exposed that last IPC intermediate; they are retained and were resolved by the final boundary guard, not a tolerance relaxation. The initial HDA attempt stopped at Qt's sandboxed processor check (`neon`) before loading the test. The final HDA run outside the sandbox passes. An initial line-search test assertion also mistook the API's NaN failure return for an exception; the test was corrected to recognize the documented failure result and its first log was retained.

Publication: PolySolve [`ee5b296a6`](https://github.com/sdast9/polysolve/commit/ee5b296a690ce225fb34f6f375d27f9ea2a16027) on `sdast9/polysolve:iteration-callback`; PolyFEM's companion pin, implementation, regressions and this record are published together on `sdast9/polyfem:main`. IPC remains `e3c8d3fe6`. RB-05's incoming files are excluded from the commit. The shared build is not rebuilt; the validated executable is `outputs/rb-review-fixes/20260912T201345Z/build/PolyFEM_bin` in the parent workspace.

## Limits retained

These are targeted implementation repairs, not a proof of engineering accuracy or all floating-point paths. The characteristic scale bounds tolerated energy noise; a decreasing gradient alone still does not prove descent on a nonconvex objective. The extreme `weight * scale` case is a named unsupported-arithmetic error, even when reordering later products could represent its exact energy; general IPC multiplication rescaling is not implemented here. Failed-attempt rollback remains RB-06's contract. RB-05 continues separately, RB-09/RB-10 accuracy work remains open, RB-17 remains retired, and the selected coefficient/interpolation/controller laws are retained. No Teseo or other private scenes were run.
