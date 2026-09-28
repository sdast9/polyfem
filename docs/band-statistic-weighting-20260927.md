# Trim band statistic weighted by collision weight — 2026-09-27

## Defect

After the IPC upstream integration (`6570e0410`, IPC `cf99893b`) the public
friction smoke (`quasistatic-semi-friction`) left the step-4 trim at 8 instead
of 16, with a final displacement difference of 1.7e-4
([IPC record](ipc-upstream-20260927.md)). That record attributed the change to
arithmetic differences before a stopping decision. The cause is more specific.

* The decision is taken at Newton iteration 1 of step 4, not at termination:
  the global trim band bumps the trim when its statistic, the rms gap of the
  active collisions, is below `sqrt(trim_lower) dhat = 0.7071 dhat`. The old
  binary measured 0.70537 (bump to 16), the new 0.70796 (no bump). The minimum
  gap agreed to 1e-16; the active collision count differed (47 vs 49).
* The slab of the smoke is two triangles sharing the diagonal x = y, and the
  cube deforms symmetrically about x = y, so five bottom vertices sit exactly
  over that edge (x − y = 0 or ±1e-15). Such a vertex is one merged
  edge-vertex collision (both triangles classify it onto the shared edge), or
  a face-vertex plus an edge-vertex collision when roundoff puts it inside one
  triangle. IPC adds the weights of merged candidates, so energy, gradient and
  Hessian are the same either way; the count-based mean
  (`NormalCollisions::compute_avg_distance`, a fork addition) is not.
* Upstream rewrote `point_triangle_distance_type` (per-edge LDLT solve →
  closed form). On identical saved positions the old and new classifiers
  resolve these ties differently (e.g. 30 vs 27 face/edge-vertex entries at
  step 2), which moved the statistic across the band edge. Eight threaded
  repeats per binary: old 8/8 trim 16, new 8/8 trim 8 — a systematic branch,
  not a coin flip.
* The same multiplicity also explains active-count variations of ±2 without
  an endpoint effect in threaded runs (RB-12 attributed them to contacts at
  the `dhat` band edge) and the RB-12 threaded trim-8 friction branch.

## Change (user decision 2026-09-27)

`BarrierContactForm::band_statistic` computes the controller's statistic as
`sum(w d^2) / sum(w)` over the active collisions (`d <= dhat`), `w` the
collision weight, summed in collision order (deterministic, where the previous
reduction was a parallel one). Merging candidates adds their weights, so the
value no longer depends on how a distance-type tie is resolved. Collisions
with a nonpositive or nonfinite weight carry no barrier energy and are left
out; if every active collision has one, the unweighted mean is used. The
refresh controller, the stall retune and the in-solve controller use it; the
collapse test (with the minimum gap), bump factor, band edges and every
other controller rule are unchanged. The opt-in force-weighted band (EF-02/03)
is untouched.

Reporting: `gap_statistics` keeps its per-collision `count/mean/rms/min/max`
and adds `band_rms`, `band_rms_over_dhat`, `band_total_weight`,
`band_weighted`; the EF-01 trim-predictor record adds `gap.band_rms`. The
debug log line now reads `band rms/dhat`. The run manifest's controller entry
names the statistic. The `band_statistic: "rms"` option token is unchanged.

The PolySolve pin moves to `6a8c2cc9` in the same commit (Armijo refuses an
uphill direction instead of asserting; PolySolve
`docs/armijo-uphill-direction-20260927.md`). Newton-family directions are
screened before the line search, so PolyFEM scenes do not reach that path.

## Verification

Native macOS arm64, AppleClang 21, shared RelWithDebInfo build (PolyFEM
`de95980c9` + this change, IPC `cf99893b`, PolySolve `6a8c2cc9`, both clean
and matching their pins):

| Check | Result |
| --- | --- |
| New `[trim_band]` test (IPC's builder on the slab-diagonal tie) | 5 vs 6 collisions for a 1e-12 offset; count mean moves 6 %; weighted statistic and total weight equal |
| Contact-cache / diagnostics / trim-predictor tests | 11 cases / 861 assertions pass |
| Affected selection (IPC-integration tags + `[trim_band]`, `[physical_diagnostics]`, `[trim_predictors]`, `[al_budget]`) | 99 cases / 10,987 assertions pass |
| RB-02 coefficient probe | 270 checks pass |
| Five public smokes, single-threaded, vs saved `c133948cf` | all 25 VTUs byte-identical; no refused line-search direction |

The smokes are byte-identical because the statistic enters the solution only
through discrete trim decisions and none changed on them. At the friction
smoke's step-4 decision the weighted statistic is 0.7142 `dhat`, clear of the
0.7071 edge the count-based values straddled (0.70537 / 0.70796), so both IPC
resolutions of the tie take the same branch (trim 8). Other scenes can take
different trim decisions where a count-based mean and the weighted mean fall
on different sides of a band edge; this is a controller-law change.

Evidence: parent workspace `outputs/band-statistic/20260927/`
(`PolyFEM_bin-baseline-c133948cf`, `PolyFEM_bin-candidate`,
`PolyFEM_bin-pinned` and their build-info JSONs, `tests-*.log`, `rb02/`,
`smoke-identity*/`, `polysolve/`).
