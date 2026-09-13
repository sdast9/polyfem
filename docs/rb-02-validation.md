# RB-02 — Per-contact coefficient and lifecycle audit

**Review follow-up, 2026-09-12:** The six closure claims below apply to their recorded counterexamples. The later closed-item review found additional overflow paths in the RB-18 global fallback and the RB-21 parent mean, plus a restart-progress bug. These bounded repairs are tracked in the [2026-09-12 follow-up](rb-review-followup-20260912.md); the closure is not an exhaustive certification of floating-point arithmetic.

**Regression update, 2026-09-13:** the probe's feature-transition expectation still encoded the stencil-keyed 70/55 coefficient jump that RB-21 removed, so it failed at check 93 against the current default; the block now asserts the parent-keyed law (coefficient and objective continuous across the switch) with the stencil identity as the control, 270/270 at `756070f44`. See [the update](#regression-update-for-the-parent-keyed-law-2026-09-13). No production change.

Date: 2026-09-08 (audit); closed 2026-09-11
Status: **closed — characterized, limits documented; production law retained** (see [Closure](#closure-2026-09-11); the audit finished as characterized—decision pending and its record below is unchanged)
Selected stage: RB-02 stages 1–4, bounded characterization and model alternatives.
No production coefficient, lifecycle, solver, dependency or HDA change.

## Contract and authorization

The user selected RB-02 from [the robustness plan](robustness-plan.md).
Read its scope, [RB-01 validation](rb-01-validation.md), the historical
[PF invariants](correctness-remediation-plan.md), [floor retirement](pf-02-floor-removal.md),
and workspace instructions. RB-01 implementation `3abfcfe25` is present in the
starting commit and its same-process regression passes in this session.

The checked-in [state/units/lifecycle contract](rb-02-contract.md) answers what
objective is evaluated in a trial and across retunes. It inventories state
owners, providers, stencil keys, caps, trim, restart/search and friction state.
The [probe](../tools/rb02/coefficient_probe.cpp) measures every requested category
with synthetic real-form fixtures; its pass means the recorded characterization
is reproduced, **not** that the law is physically acceptable.

No positive floor, new cap law, driving Hessian choice, retune relocation,
friction policy, convergence threshold, resource budget or retry policy was
selected. CCD and the separate trial-displacement cap remain. No private scene,
Ballburst or Teseo was run. RB-03 interpolation and RB-06 rollback are outside
this item; their unresolved scope remains explicit.

## Baseline and reproduction

PolyFEM started on clean tracked `main` at `cfc41224c`; existing untracked
simulation/VTK artifacts were present; their final location check is recorded
below. IPC recipe and effective CPM checkout
are `9da3094a46bcc054cc19024a5c748557c5bb6b9e` at
`~/.cache/CPM/ipc-toolkit/0c20`, clean detached checkout. The companion IPC tree
is clean on `semi-implicit-stiffness`. PolySolve recipe and effective local
override are `4d372fa8a73f42bc224e31d464f1a308e1159ba8` at
`../polysolve-merged`, clean `iteration-callback` branch. Source overrides and
recipes were inspected; nothing was repinned. No concurrent solver/build was
found before the work began.

Configured macOS arm64 build: Apple Clang, `RelWithDebInfo`, TBB and Accelerate;
six build jobs. Public smokes use their unchanged `Eigen::SimplicialLDLT`
configuration. The probe itself is an algebraic contact fixture, not a nonlinear
scene solve. It uses unprojected contact Hessians, no area weighting, a HASH_GRID
broad phase, finite nondegenerate 2D geometry and identity collision/FEM mapping.

Local parent-workspace evidence: `outputs/rb-02/20260908-coefficient-audit/`.
`baseline-manifest.json` saves repo states, compiler/platform, source heads,
binary and scene SHA-256 hashes; `CMakeCache.txt` preserves build configuration.
`commands.md`, per-run compiler/link commands and provenance, build/test logs,
exits, copied smoke script, PVD files, smoke summaries and tested hashes preserve
the execution. The final run is `units-probe/`; its small result is published as
[results-20260908.json](../tools/rb02/results-20260908.json). Historical PF-02 and
PF-08 results were not overwritten or regenerated. No HDA modification was
needed; its installed binary identity was not remeasured in this audit.

All findings below were measured against **unchanged production sources**.
There is no before/after production repair or claim that a historical scene
failure was explained.

## Findings and measured outcomes

| Finding / required case | Measurement | Outcome |
| --- | --- | --- |
| Positive definite local curvature | `H=100 I`, gap .2, support 1 gives kappa=100, E=296.651596, force norm=1431.493641 | Reproduced baseline |
| Indefinite curvature | Negative tangential / positive normal diagonal gives kappa=100; negative definite H gives zero | Sign of normal Rayleigh quotient matters, not simply presence of negative eigenvalues |
| Zero / singular curvature | Zero H and singular H with zero normal curvature both yield zero energy, gradient and Hessian at active contact | Reproduced loss of barrier through coefficient law; model decision pending |
| Bounded near-singular curvature | `H=1e-12 I` gives kappa=1e-12 and force norm=1.43149e-11 | Finite; no inversion/division of H required |
| CCD when kappa is zero | Crossing rejected and maximum step lies strictly between 0 and 1 for three zero-coefficient fixtures | CCD remains independent; no inference of equilibrium or physical repulsion |
| Zero median batch | Uncapped `[0,0,100]` at default spread 10⁴ becomes `[0,0,0]`; E and force norm zero | Reproduced suppression of a positive-curvature contact; decision pending |
| Positive median / cap | `[1,10,1000]`, spread=2, becomes `[1,10,20]`; E=91.961995 | Existing cap law reproduced; experimental spread is explicitly changed |
| Existing-option comparisons | Spread=0 preserves positive contact in zero-median batch; kappa_min=1 restores E=2.966516 to negative-curvature single contact | Comparative effects, not endorsed defaults |
| Initially absent contact | Snapshot gap 1.2, trial gap .3; coefficient assigned from frozen 100 I with no extra provider call, empty-batch cap stays infinite | Deterministic new-key assignment reproduced |
| Frozen evaluation orders | A/B/A and B/A, for initially present and absent contacts; full gradient/Hessian comparisons and provider call count | Pass within tested stencil regions |
| Independent energy differences | Central energy gradient and energy-only mixed second differences on all 6 DOFs, present/absent fixtures | Relative errors below 5.8e-11 for gradient and 1.34e-7 for Hessian |
| Geometry-only refresh | Fixed H with endpoint stiffness 10, point stiffness 100: center snapshot gives 70; point shifted .5 and refreshed gives 65.384615 | Energy/force at the same evaluation position changes under refresh; intended coefficient update |
| Nearest-feature transition during a frozen trial | EV coefficient 70 versus VV 55; separations 2e-3, 2e-5, 2e-7 across endpoint; energy differences −44.4993465, −44.4977396, −44.4977394 | Reproduced nonvanishing objective discontinuity despite deterministic cache; model decision pending |
| Frozen converted scales | L=.001,1,1000; Q=.01,1,100; a=.25,1,4 (27 combinations) | Energy, force and Hessian obey converted units with fixed trim |
| Controller converted scales | Unloaded gap .8*dhat, default options, H converted by 1/L²: trims .001,1,1 at L=.001,1,1000 | Reproduced first-contact conditioning dependence on length units |
| NaN / infinity / finite arithmetic overflow | NaN H entry, infinite H entry, and finite signed rank-one H with entries up to 1e308 each assign kappa=1e30 | Literal recovery silently replaces invalid curvature; validity is not established |
| Nonfinite after weight division | Weights 0 and 1e-310 yield nonfinite assigned scale after the finite check | Probe explicitly rejects energy evaluation; production validation gap remains |
| Cap multiplication overflow | Finite spread 1e308 and kappa=100 produce infinite cap | Finite coefficient retained; safeguard becomes ineffective |
| Lagged friction | Tangential potential 11.6880966 before trim doubling; same immediately after; 23.3761933 after explicit lagging update | Lagged normal-force persistence reproduced; no new friction policy selected |
| Search/history after retunes | Source trace shows new minimize resets strategies; post-step updates do not send a coefficient-change reset; L-BFGS uses previous gradient | Characterized source contract; no measured L-BFGS convergence failure claimed |

The finite-overflow fixture uses a six-vector `v=[0,-1,0,-1,0,1]` and
`H=1e308 v v^T`. All H entries are finite, but its normal Rayleigh quotient
overflows. Invalid coefficients are serialized as explicit strings, not JSON
nulls, in the final result. No degenerate contact geometry or unbounded resource
allocation was used to trigger these cases.

At unchanged x, the permitted event probes give these energy jumps (full before
and after force/coefficient data are in the JSON):

| Event | Configuration / effect |
| --- | --- |
| Explicit bump | Trim 1 -> 2, E doubles; Delta E=296.651596 |
| Gradient-balance calibration | Synthetic opposing driving gradient requests trim 3; Delta E=593.303192; a separate target .5 leaves trim 1 |
| Refresh with controller | At gap .2, trim 1 -> sqrt(.5/.04)=3.535534; Delta E=752.170180 |
| First-contact post-step | Empty snapshot -> contact at .8; controlled H increase 100 I -> 200 I doubles E |
| First-contact conditioning | Empty snapshot, initial trim 1e5 -> 1000 at contact .8 and unchanged H |
| Emergency post-step | Three accepted-step callbacks at gap .2 give the same 3.535534 trim bump |
| Downward post-step | Gap .98, test controller_interval=1 gives trim 1 -> .5; Delta E=−.003168108 |
| Periodic post-step | Test refresh_interval=1 and H doubled give E multiplier 7.071068 |
| Stall retune | Gap .2, factor=2 gives trim 3.535534 after refresh |
| No-op control | Identical x/H, controller disabled: refresh leaves energy unchanged |

H changes in selected probes are deliberate changes to a synthetic provider's
backing matrix; they isolate the refresh effect, not a measured material law.
Geometry-only refresh uses an unchanged provider. Direct post-step calls test
the real controller at fixed x, not an actual trajectory. These energy jumps
are **algorithmic objective changes at zero displacement**, not integrated
mechanical work. Their signs do not alone determine whether a scene is accurate.

## Validation

Tolerances were specified in the probe before the corresponding runs: frozen
comparisons use absolute-plus-relative 1e-10, converted quantities 1e-9;
central energy-gradient differences use h=1e-6 and normalized norm tolerance
1e-6; energy-only mixed Hessian differences use h=1e-4 and tolerance 1e-5.
The .3 finite-difference gap is far from singularity and nearest-feature switches.
The larger Hessian step controls double-difference cancellation. No production
setting or test tolerance was relaxed to obtain a pass.

| Check | Result | Exit |
| --- | --- | --- |
| Build `PolyFEM_bin unit_tests -j 6` | Both targets complete against unchanged production sources | 0 |
| Final standalone RB-02 probe | 257 checks, all requested case categories plus feature transition and controller scaling | 0 |
| `[contact_cache]`, seed 1 | 703 assertions / 4 cases | 0 |
| Plan's affected contact/AL/BC/filter/derivative selection, seed 1 | 1,189 assertions / 22 cases | 0 |
| quasistatic-adaptive | Four steps through t=1; zero error lines; 2.831333 s reported | 0 |
| quasistatic-semi-alhess | Four steps through t=1; zero error lines; .879073 s | 0 |
| quasistatic-semi-friction | Four steps through t=1; zero error lines; .975635 s | 0 |
| quasistatic-semi | Four steps through t=1; zero error lines; .892256 s | 0 |
| transient-semi | Four steps through t=1; zero error lines; .965412 s | 0 |
| C++ formatting, Python syntax, local links, diff whitespace | Changed files checked | pass |

Each smoke process exit was checked separately. Every PVD contained time entries
`[0,.25,.5,.75,1]`; the copied script changes only OUT. Expected negative-test
error logs in the affected suite did not fail assertions. Times are solver log
totals, not controlled performance comparisons. HDA tests were not rerun because
only standalone audit tools and documentation changed. No full solver suite,
full PF-08 sweep, other platform, private scene or physical certification run.

Retained development runs: the initial 212-check probe passed; a 232-check
extension passed; the first feature-boundary extension exited 1 at check 88
because it compared the entire diagnostic state, including a cache that had
legitimately gained a new stencil key. The assertion was narrowed to evaluated
scales/energy/derivatives; no numerical tolerance changed. Its failed log and
exit remain in `feature-probe/`. The corrected expanded probe passed 251 checks;
the final controller-unit extension passed 257. The earliest attempted overflow
fixture instead measured cancellation to zero; the final signed rank-one fixture
explicitly asserts the nonfinite fallback. Those earlier runs are retained,
not presented as equivalent final coverage or as production failures.

Contact energy, force, Hessian, finite-gap coefficient behavior, CCD rejection,
lagged tangential potential and zero-displacement objective changes were measured.
Continuum BC accuracy, reaction balance at a solved equilibrium, det(F), trajectory
energy/work balance, friction dissipation, mesh convergence, global residuals
and engineering accuracy were **not measured**. These microfixtures have no FEM
elements or nonlinear solve; successful public smokes establish numerical
completion only. The feature discontinuity and zero barrier are counterexamples
to general model claims, not proof that a particular production scene is wrong.

## Publication and reproducibility

Publish only `tools/rb02/` (source, bounded runner, README, dated JSON), this
record, the coefficient contract and the RB-02 plan row to `sdast9/polyfem:main`.
The task completion reports the resulting commit. The linked result records
unchanged production library revision `cfc41224c` through local provenance and
source/binary hashes; probe source is separately hashed. No dependency or HDA
publication is required. No audit command intentionally moved or deleted incoming
untracked artifacts. Final verification found the 432 original untracked paths
absent from the repository; all 432 names exist in the pre-existing local
`outputs/pf-08/` evidence directory. Baseline artifact content hashes were not
collected, so content identity is not established. The user subsequently confirmed
that they cleaned up the repository artifacts during this session. The audit did
not restore or overwrite either location. Production binary hashes
are unchanged and the tracked checkout is clean.
The parent workspace README is updated locally to reflect this characterization.

Reproduction uses the [tool README](../tools/rb02/README.md) and current build's
compiler/link recipe, saving results to a fresh absolute evidence directory.
The runner calls the real library and includes its own checks; it is deliberately
bounded and is not an invocation of the PF-08 108-configuration sweep.

## Next session handoff

RB-02's inventory, deterministic tests, comparative effects, retune measurements
and source tracing are completed within the stated fixture scope. Its status
remains **characterized—decision pending** because the sign/cap law, feature
continuity, conditioning units and retune/history/friction model need choices.
Concrete alternatives, units and validation requirements are in the final table
of [the contract](rb-02-contract.md). No production model was selected or fixed.

RB-03 can start its mapping audit using this inventory and the RB-01 prerequisite;
it must consult the coefficient counterexamples. RB-04 may use the inventory
but its dependent supported-mapping work remains pending RB-03. A model-choice
follow-up should select a bounded alternative and its validation protocol before
changing coefficients or moving retunes. Do not mark the contact model validated
because this investigation is published.

## Closure (2026-09-11)

**Status: closed — characterized, limits documented; production law retained.**

On 2026-09-11 the user reviewed RB-01–RB-17 and the interior-point
literature in the parent workspace and decided to keep IPC's adaptive barrier
as the production contact model: no new coefficient law, estimator or
controller is to be selected. That is the model decision this item was held
open for. Every defect the audit reproduced has since been repaired in
production, bounded to the existing law, by RB-18 (`e3fa362e0` … `4a0df80a1`),
RB-20 and RB-21 (`0324ce096`, `21f9fd592`, `beb6ef641`; toolkit `e3c8d3fe`).
Disposition of the six decisions in the [contract's final table](rb-02-contract.md#unit-covariance-and-model-decisions):

| Contract issue | Disposition |
| --- | --- |
| Zero median erases a positive contact | Repaired — [RB-18 F1](rb-18-quick-fixes.md#the-six-fixes): positive-only batch median with a relative κ floor; `[0,0,100]` no longer becomes `[0,0,0]` |
| Nonpositive normal curvature | Repaired — [RB-18 F2/F7](rb-18-quick-fixes.md#the-six-fixes) (user choice B+E): previous κ → \|wᵀHw\| → max\|H\|/d̂²; κ=0 only for an identically zero system Hessian |
| Stencil-switch (EV/VV, FV/EV) jump | Repaired — [RB-21](rb-21-parent-keyed-kappa.md): coefficient keyed on the candidate primitive pair and weight-averaged over parent contributions; seams exactly C⁰ in the default formulation (`[kappa_continuity][parent]` regression) |
| Controller unit dependence | Repaired — [RB-18 F3](rb-18-quick-fixes.md#the-six-fixes): `conditioning_cap` normalized by d̂², so the first-contact trim is equal across converted length scales |
| Retune and history/friction mismatch | Repaired within the retained lifecycle — [RB-18 F6](rb-18-quick-fixes.md#the-six-fixes) (lagged friction follows the trim), [RB-18 F5](rb-18-quick-fixes.md#the-six-fixes) (no repeated unchanged stall restarts) and [RB-20](rb-20-force-continuation.md) (persisting contacts keep their realized coefficient across refreshes; post-publication drift 17–53 % → ~1e-16). Remaining documented limits: a retune inside a solve still does not reset quasi-Newton history (stage 4; no convergence failure measured; the one allowed restart of F5 rebuilds the solver), and the friction lag follows the trim but not per-contact κ assignment for contacts born mid-step |
| Invalid arithmetic | Repaired — [RB-18 F4](rb-18-quick-fixes.md#the-six-fixes): NaN curvature, overflow with no reference and zero/subnormal weights are errors naming the stencil; finite overflow uses the cap; the original checks run after weight division and cap multiplication; the 2026-09-12 follow-up also checks global-fallback arithmetic and the final parent mean/weighted collision coefficient |

The RB-02 probe (`tools/rb02/`) now carries the repaired expectations and is
the regression for this law: 243/243 checks at RB-18's final state
(`final-probe-f7/`), with the pre-fix 257-check run retained as the defect
baseline. (Updated 2026-09-13: that run predates the RB-21 default flip and
its feature-transition check encoded the stencil-keyed jump; 270/270 on the
current law, see [the regression update](#regression-update-for-the-parent-keyed-law-2026-09-13).) Retained, not closed here: RB-03 interpolation curvature (its own
item), physical certification of the retained law (RB-09/RB-10 references;
the RB-20 friction endpoint move of 1.6e-2 on the public smoke is where a
reference comparison belongs), and the private-scene/Ballburst/Teseo runs this
audit never made. Closing this item records a decision, not a physical
validation of the contact model.

## Regression update for the parent-keyed law (2026-09-13)

**Status: closed (unchanged); the probe is again the regression for the
repaired law — 270/270 checks at `756070f44`.** No production code, default,
public input or unrelated probe check changed.

The closure above cites the probe's 243/243 pass at RB-18's final state
(`576b1d3d0`, `outputs/rb-18/20260911T164356Z/final-probe-f7/`). That run
predates the RB-20/RB-21 default flip (`beb6ef641`): the probe's
feature-transition check still encoded the *stencil-keyed* coefficient jump —
`close(l["scales"][0], 70) && close(r["scales"][0], 55)`, "feature coefficient
change" — that [RB-21](rb-21-parent-keyed-kappa.md) deliberately removed, so
the regression claim was stale from that commit on. The RB-10 session's
2026-09-13 run reproduced it: exit 1 at check 93 of 243
(`outputs/rb-10/20260912T232617Z/rb02-probe-new-defaults/`, library
`1809a705f` plus RB-10's then-uncommitted default flip); the 92 checks before
it pass.

### What the parent-keyed law produces at the switch

Same fixture as the audit (edge `(v0, v1)` on the x-axis, point `v2` at gap
.2 above `x = 0`, frozen driving Hessian 10 on the edge vertices and 100 on
the point, one snapshot at the center, `line_search_begin` frozen trials with
the point at `x = 1 ∓ ε`). Under the default `coefficient_identity: "parent"`
the edge–vertex collision left of the endpoint and the vertex–vertex collision
right of it are built from the same candidate `(e0, v2)`, so both carry that
parent's memoized snapshot value and the memo gains no key at the switch; the
distance to the whole edge is C¹ at a positive gap with a zero tangential
derivative at the endpoint, so the frozen objective is C¹ across it. Measured
(`outputs/rb-02/20260913T054358Z/probe-results.json`, `feature_transition`):

| ε | parent κ (EV / VV) | ΔE = E(1+ε) − E(1−ε) | bound ε·max\|∂E/∂x\| (+ 32 ε_mach \|E\|, visible at 1e-7) | ‖∇E(1+ε) − ∇E(1−ε)‖ | bound 3ε·max‖H‖ | stencil κ (EV / VV) | stencil ΔE | stencil E ratio |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1e-3 | 70 / 70 | −2.0454e-3 | 4.0907e-3 | 5.8128 | 32.332 | 70 / 55 | −44.4993 | .7857065 |
| 1e-5 | 70 / 70 | −2.0454e-7 | 4.0908e-7 | 5.8142e-2 | .32333 | 70 / 55 | −44.4977 | .7857143 |
| 1e-7 | 70 / 70 | −2.0464e-11 | 4.2384e-11 | 5.8142e-4 | 3.2333e-3 | 70 / 55 | −44.4977 | .7857143 |

The parent-law jump is exactly half its bound at every ε, as the fixture
predicts: with the toolkit's squared-distance barrier `B(d) = b(d², d̂²)`,
`B′(.2) = −11.69` and `ΔE = ½ κ B′(d)/d · ε² = −2045 ε²`; the tangential
force at `1+ε` is `κ B′(d) ε/d = −4091 ε` (measured −4.0907 at ε=1e-3) and
zero at `1−ε`. The stencil-law ratio converges to 55/70 = .7857142857: the
historical jump is the coefficient ratio, the distance itself being
continuous. The stencil values are the audit's original measurements
(−44.4993465, −44.4977396, −44.4977394 in the findings table).

### Probe changes (`tools/rb02/coefficient_probe.cpp`, feature-transition block only)

Replaced the stencil-keyed expectation with the current law and made the
recorded-only energy jump an assertion; per ε the block now checks:

| Check | Assertion |
| --- | --- |
| `parent feature trial order repeatability` | unchanged A/B/A comparison of evaluated scales, energy, gradient and Hessian norms |
| `feature coefficient continuity` | `l.scales[0] == 70 && r.scales[0] == 70` (was 70 / 55) |
| `feature switch adds no coefficient key` | the memo holds one key on both sides |
| `feature energy continuity` | `\|ΔE\| ≤ ε·max(\|∂E/∂x\|_l, \|∂E/∂x\|_r) + 32 ε_mach \|E_l\|` — the monotone-force bound above |
| `feature gradient continuity` | `‖∇E_r − ∇E_l‖ ≤ 3ε·max(‖H_l‖_F, ‖H_r‖_F) + 32 ε_mach ‖∇E_l‖` — path length 2ε times the Hessian along it, with margin for its variation |
| `stencil feature trial order repeatability` | the same A/B/A comparison on a second form with `coefficient_identity: "stencil"` |
| `stencil identity feature coefficient change` | the retired expectation verbatim, `70 / 55`, as the control for the repaired defect |
| `stencil identity energy jump is the coefficient ratio` | `E_r/E_l == 55/70` to 1e-4 |

Fifteen checks became forty-two; every other check is untouched (243 − 15 +
42 = 270). Tolerances of unrelated checks were not changed. The JSON
`feature_transition` entries now hold `parent` and `stencil` sub-records
(samples, jumps, bounds, tangential forces). The friction section's explicit
`friction_lag: "follow_stiffness"` is RB-10's `756070f44` change (the F6
behaviour is opt-in since that commit) and is untouched here. The
measurement pass that preceded the assertions (all checks removed at the
switch, both identities recorded) passed 246/246 including every later
section; that scratch run is not retained as evidence.

### Validation

| Check | Result | Exit |
| --- | --- | --- |
| `python3 tools/rb02/run_probe.py --build build --output outputs/rb-02/20260913T054358Z` | **270 checks**, `passed: true` | 0 |
| clang-format (`.clang-format`) on the probe | identical | — |
| Unit tests, smokes, HDA | not rerun: no production source changed (as for the 2026-09-08 audit) | — |

Tested revision: PolyFEM `756070f44` — RB-10's defaults commit; HEAD at run
time was its documentation hash note `e332b8b2c` (`provenance.json:
polyfem_head`), the tracked tree was clean and no tracked source was newer
than the shared `polyfem/build/libpolyfem.a` (2026-09-13 01:26 EDT), which the runner
links; local source overrides IPC toolkit `bb795446` (= the pin,
`ipc-toolkit-fork` clean) and PolySolve `ee5b296a` (`polysolve-merged`
clean). Probe source SHA-256 `f41b3c4c1da869a7efe114c250195532d7de06ca6fad7750174080747063cb08`, executable `3a9e08b01e7ddd2f6b89315e76fa2a67094e939b1690077af37f5c0c68a809d7`. The
same probe source had passed 270/270 against the same library while RB-10's
flip was still uncommitted (`1809a705f` + working tree); the coefficient law
is untouched by that flip and the probe pins `friction_lag` explicitly in its
only friction fixture. Publication hash: see the progress note below.

- **2026-09-13** — Probe updated and published on `sdast9/polyfem:main`
  (`tools/rb02/coefficient_probe.cpp`, this record,
  `docs/rb-21-parent-keyed-kappa.md`); the commit hash is recorded in a
  documentation follow-up.
