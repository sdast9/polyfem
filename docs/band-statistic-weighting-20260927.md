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
| Full unit suite on `61a7a4507` (this change + restart commit `c2a57e393`, PolySolve `43ca2e66`), 92 min | 398 cases: 395 pass; the two known `verify_run` scene failures (`gcp-contact/cube-on-floor`, `multi-material/stretch-cubes`) and `shape-transient-friction` (below) |

`shape-transient-friction` (`[opt_gradient]`, fixed barrier stiffness 1e5, so
the band statistic is not involved) is a regression of the IPC upstream
adoption, found here because that integration's full suite hit its time cap:
it passes on `a0a40ce92` and on an isolated build of `3c40ae557` (IPC
`75600955`) and fails on one of `6570e0410` (IPC `cf99893b`) with the values
of the shared build. The adjoint derivative is unchanged (404239.98534 before,
404239.98533 after, 3e-14 relative); the central finite difference with step
1e-6 moved from 404242.13 to 404222.51, so the relative error is 4.3e-5
against upstream PolyFEM's tolerance 1.1e-5 (5.3e-6 before). Upstream PolyFEM
still pins IPC `b40e9c07`, 27 commits before the merged `869e489e`, so its
tolerance was tuned without these IPC changes.

**Characterized (2026-09-28): finite-difference noise, not a derivative
defect.** A step study on the isolated `6570e0410` build (same scene, seed and
directions; `bisect/fd-step-study.patch`, uncommitted) shows:

* truncation error falling as about 2.5e5 h² (2.5e-4 at h = 1e-5, 2.3e-5 at
  3e-6), and below h ≈ 1e-6 a noise floor from the forward solves scattering
  the relative error by up to ±4e-5;
* the test's step is `float(1e-6)` = 9.99999997e-7. At exactly 1e-6 the
  difference agrees to 3.1e-6 (passes), at the float step it is off by
  4.3e-5, and at 3e-7 it agrees to 4.4e-8;
* with the forward solve's gradient tolerance tightened from 2.5e-10 to
  2.5e-11 or 2.5e-12, the test's step agrees to 3.4e-6 (passes);
* the other two random directions have 40–150× larger derivatives and agree
  to 1e-9 – 1e-6 at every step near 1e-6;
* with IPC's `IPC_TOOLKIT_WITH_MESHFEM_SPARSE=OFF` (triplet assembly) the
  merged IPC reproduces the pre-merge derivative and finite difference
  bit-for-bit (404239.98533951573 / 404242.13287967740) and the test passes;
  the triplet curve has the same noise scatter below 1e-6.

So MeshFEMSparse's summation order changes roundoff in the forward solves, and
the test, whose trial 0 sits at the crossover of truncation and noise with a
1.1e-5 tolerance, draws an unlucky value. The adjoint derivative agrees with
well-conditioned differences to 4e-8. The test is left failing pending the
user's choice (tighten its forward tolerance, or keep it as a known failure);
no tolerance has been changed.

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
