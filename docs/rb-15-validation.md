# RB-15 — Feature-consistent local coefficient assignment

Date: 2026-09-11 (started September 10).
Status: **closed 2026-09-11 — characterized; recommendation implemented by RB-20/RB-21** (see [Closure](#closure-2026-09-11)); the bounded investigation itself finished characterized—decision pending.
Selected stage: compare frozen feature keys, shared parents and complete smooth
fields; reproduce 2D discontinuity, extend to 3D and characterize lifecycle/limits.

## Contract and authorization

The user selected RB-15 using `docs/robustness-plan.md`. Its invariant is a
consistent conditional potential and force as nearest subfeatures change, with
parameter updates distinguished from displacement work. The comparison and
recommendation are in the [contract](rb-15-contract.md). Implementation choice
remains open as required by the plan; no production mode or estimator was changed.

Read the RB plan, PF invariants and floor-retirement record, RB-02 coefficient
contract, RB-03 map contract, RB-04 real-form transition/work probe, RB-13 reference
units and RB-14 current disposition at PolyFEM `e3a23f1a3`. RB-14's nonlinear
estimator coverage and zero-demand protection remain unresolved. RB-03 interpolation
stiffness remains unselected. This work does not depend on resolving them to compare
assignment continuity. No controller, friction/lagging, material, mapping law,
tolerance, CCD, trial cap, force floor/cap or stopping policy changes are included.

## Baseline and reproducibility

All three repositories were initially clean, on their existing branches. No
incoming build/simulation process was found or interrupted.

| Component | Effective source / revision |
| --- | --- |
| PolyFEM | `main`, `e3a23f1a3e45fc82da9b98589438c52080fc8c3f` |
| IPC | workspace `ipc-toolkit-fork`, `semi-implicit-stiffness`, `af317a65d69d0ac7c5efa4bf103bf75e280c323b` |
| PolySolve | workspace `polysolve-merged`, `iteration-callback`, `4d372fa8a73f42bc224e31d464f1a308e1159ba8` |

Both recipe pins match these companion revisions. CMake's effective source records
and compile/link arguments use the local companions, not an assumed CPM copy.
Build: macOS arm64, Apple clang 21.0.0, RelWithDebInfo, TBB, existing linear-solver
configuration. No nonlinear solver is invoked by the standalone feature probe.
The focused Catch regressions exercise their own configured solvers.

Fresh local evidence:
`outputs/rb-15/20260910-feature-study/` under the parent workspace. It contains
initial/final identity, compile/link/run logs and hashes, the unchanged baseline
run, retained exploratory runs, rebuild and focused regressions. The compact
[provenance](../tools/rb15/provenance-20260911.json) includes binary/source identities
and hashes of the linked archives. The [source and commands](../tools/rb15/README.md)
and [predeclared protocol](../tools/rb15/protocol.md) are published with the
[140-check result](../tools/rb15/results-20260911.json).

From `polyfem/`, baseline and final probe commands were:

```bash
python3 tools/rb04/run_candidate_probe.py --build build --output ../outputs/rb-15/20260910-feature-study/baseline
python3 tools/rb15/run_probe.py --build build --output ../outputs/rb-15/20260910-feature-study/validated
```

Use new output names to reproduce; runners refuse to overwrite existing evidence.
The original `tools/rb04/candidate-results-20260909.json` hash remains
`93a598fddd6345e1bd096af23c809c7de748ab341783d66d739bfe83d195392c`.
The initial wrong-directory `tools/rb04/... --help` invocation exited 2 before
any probe ran; the corrected baseline above ran from `polyfem/`.

## Findings and disposition

| Finding | Evidence | Outcome |
| --- | --- | --- |
| Frozen 70/55 EV/VV jump | Unchanged RB-04 probe: -44.497739403; Fixed70 control passes | reproduced |
| 3D frozen EV/FV jump | Coefficients 70/76.976744186; jump +20.696622978 | reproduced |
| Shared parent removes tested finite jumps | Decreasing separations, full derivatives and split quadrature | characterized in experimental probe; production unresolved |
| Complete smooth field removes tested jumps but changes force | Product derivatives pass; tangential force and positive-k attraction measured | characterized; model unresolved |
| Region switch/duplicate parent can reintroduce jumps | -44.497739403 and +207.656117214 at identical gap | reproduced counterexamples |
| Frozen heterogeneous parents survive tested lifecycle | Two real parents, simultaneous transitions and separation/recontact | validated only for this fixture |
| Coefficient refresh is not displacement work | Same-x doubled Hessian doubles energy | characterized outer-update accounting |

Shared-parent assignment is the recommended bounded RB-16/17 candidate. It is
not a general parent-construction algorithm: adjacent primitives, overlap,
unknown-parent discovery and multiplicity policy must be chosen and tested before
production implementation. See the contract for alternatives and material limits.

## Validation

| Check | Expected criterion / input | Result | Exit |
| --- | --- | --- | --- |
| Unchanged RB-04 baseline | Preserve 70/55 discontinuity and Fixed control | 68 checks passed | 0 |
| Final standalone feature probe | Protocol continuity, derivative, work, state, units and mapping checks | 140 checks passed | 0 |
| Candidate limits at epsilon=1e-9 | Shared/global/smooth energy jump <1e-6; gradient norm difference <1e-4 | all six candidate/transition combinations pass | 0 |
| Fixed-region finite differences | Relative gradient <1e-6; Hessian <1e-5 | max 4.60e-11 / 8.29e-11 | 0 |
| Split path quadrature | Complete candidate residual <1e-5 at 1024 panels/side | max absolute 1.18e-6; current jumps retained | 0 |
| Rebuild | `cmake --build polyfem/build --target PolyFEM_bin unit_tests -j 6` from parent | both targets built | 0 |
| Affected regression selection | Cache, mapping, floor retirement, direction filter, BC scale/metric, AL, normal/friction derivatives | 1,980 assertions / 29 cases passed | 0 |
| Formatting, Python syntax, local links, JSON and diff | New/changed artifacts only | passed before publication | 0 |
| Five scene smokes / HDA E2E / full suite | No production or HDA change | not run in this investigation | not run |

The exact Catch command and working directory are in the published provenance;
it was run by absolute binary path from a fresh `regression/` evidence directory.
Expected negative-test error messages accompany a passing Catch summary.

The small probe uses positive finite gaps, static unweighted potential, synthetic
Hessians, no friction and no nonlinear termination criterion. It measures contact
energy/gradient/Hessian, path work, local normal/tangential force, coefficient
updates and local evaluation time. BC/reaction balance, full residual, det(F),
continuum refinement, physical contact accuracy and dynamic dissipation are
**not measured**. Passing tests are not whole-fork or whole-scene certification.
Teseo and all other private scenes were not run. No tolerances/goldens changed.

## Retained partial and failed outcomes

- `candidate-01`: compilation failed because the probe's `Form` name conflicted
  with the imported base class. Renamed the probe class; no solver change.
- `candidate-02`: failed its final attraction assertion after preceding checks
  passed. The positive field centered on the feature boundary stayed repulsive,
  with minimum sampled force 5511.16463. This result is preserved.
- `candidate-03`: shifted the attraction diagnostic's field center to a=.08 as
  documented in the protocol; this gives -20412.12993. Added real two-parent and
  support-onset checks; 122 checks passed. No assertion threshold was relaxed.
- `candidate-final`: removed probe `const_cast`, made includes explicit, added
  the provenance-capturing runner; 122 checks passed.
- `validated`: added actual line-search candidate intervals and unit checks for
  current/shared/global coefficients; final 140 checks passed.

Old source copies, logs and results remain in the local evidence directories.
Only the final compact measurements and summarized exploratory outcomes are
checked in. Their meanings remain distinct; a later success does not erase a
failed exploratory expectation.

## Publication and next session

Changed files are the new `tools/rb15/` probe, protocol, runner, measurements and
provenance; this contract/record; and the RB plan status/handoff. The parent README
is updated locally outside Git. No production source, dependency pin, HDA or
baseline measurement changed. The tested probe source hash and linked archive
hashes are in provenance; the publication commit is reported in the task's final
message and local `publication.json` rather than self-referenced here.

Publish to `https://github.com/sdast9/polyfem.git`, branch `main`, under the standing
workspace instructions. The final status remains **characterized—decision pending**.
The bounded comparison is complete; general production transition handling is
unresolved. Before integration select parent construction, seam/multiplicity
semantics, the RB-14 estimator and outer discovery/refresh policy. RB-16 can use
this bounded candidate for further controller investigation; RB-17 still needs the
explicit model decisions. No downstream work is silently included in this session.

## Closure (2026-09-11)

**Status: closed — characterized; recommendation implemented by RB-20/RB-21.**

On 2026-09-11 the user reviewed RB-01–RB-17 and the interior-point literature and decided to keep IPC's adaptive barrier as the production contact model: no new coefficient law, estimator, assignment or controller is to be selected. This item's recommended bounded candidate — frozen shared parents —
is what production now does; the alternatives are not selected. Disposition:

| Open decision | Disposition |
| --- | --- |
| Parent construction and unknown-parent discovery | Implemented by [RB-21](rb-21-parent-keyed-kappa.md): the toolkit builder records the candidate primitive pair(s) that built each collision (`NormalCollision::parents`, toolkit `e3c8d3fe`), so parents are known by construction — no discovery algorithm is needed |
| Seam / multiplicity semantics | Implemented by RB-21: the coefficient is keyed on the parent candidate and the built collision's scale is the weight-mean of its parent contributions; seams are exactly C⁰ in the default formulation (`[kappa_continuity][parent]` regression), with the improved-max formulation's half-jump documented and not used |
| Frozen coefficients across the lifecycle | Implemented by [RB-20](rb-20-force-continuation.md): a contact keeps the coefficient that acted at the published endpoint; refresh drift 17–53 % → ~1e-16 |
| Complete smooth field (changes forces; tangential attraction) | Not selected |
| Estimator for the parent (RB-14) | Not selected; the production law estimates new parents (see RB-14 closure) |

The 2D/3D probes (`tools/rb15/`), measurements and contract are unchanged.
Closing records a decision and its implementation, not a physical validation.
