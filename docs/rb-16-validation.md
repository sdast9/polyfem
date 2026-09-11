# RB-16 — Gap controller and update timing

Date: 2026-09-11.
Status: **characterized—decision pending; spring and bounded real-form/FEM stages complete**.
The stage-1 handoff below is historical; the stage-2 disposition supersedes it.

## Contract and authorization

The user selected “continue with RB-16.” This first stage follows the item's
instruction to start with simple springs. Read the RB plan, PF invariants,
floor-retirement record, RB-13 interval contract, RB-14 estimator evidence and
RB-15 parent-assignment comparison. The [contract](rb-16-contract.md) defines
units, state timing, comparison branches and remaining limits. No production
coefficient, controller, numerical stopping, friction, material, mapping, CCD,
trial cap or retired-floor behavior changed. There is no new endpoint gate.

## Baseline and reproduction

All three incoming repositories were clean; no running build/simulation was found
or interrupted. Existing outputs were preserved. Effective CMake source overrides
and recipe pins agree:

| Component | Branch | Revision |
| --- | --- | --- |
| PolyFEM | main | e3a01a732 |
| IPC, local ipc-toolkit-fork | semi-implicit-stiffness | af317a65d69d0ac7c5efa4bf103bf75e280c323b |
| PolySolve, local polysolve-merged | iteration-callback | 4d372fa8a73f42bc224e31d464f1a308e1159ba8 |

Source inspection checked BarrierContactForm's collapse statistic, post_step
cadence, trim rails and refresh/calibration. The spring timing control executes a
reduced source-derived rule, not the real form. Production build is RelWithDebInfo;
the standalone bridge uses macOS arm64, /usr/bin/c++, C++17, O2 and system Python.
Only the actual IPC barrier primitive is linked; production threading and solver
settings are not exercised. A bridge rebuild occurs for every fresh probe run.

Local evidence: `outputs/rb-16/20260911-060028-stage1/` in the parent workspace.
Baseline repository/process state, production binary/cache/source hashes, exact
commands, compiler outputs, all full trajectories and failed outcomes are retained.
The [published provenance](../tools/rb16/provenance-20260911-stage1.json) records
source/config/primitive hashes and exploratory dispositions. From `polyfem/`:

```bash
python3 tools/rb16/controller_probe.py --ipc-source ../ipc-toolkit-fork --output ../outputs/rb-16/20260911-060028-stage1/validated
```

Use a new output name for reproduction. The protocol was written before the first
run; dated amendments describe the two probe corrections without rewriting prior
evidence. No original RB-13–RB-15 result or production binary was altered.

## Measured results

The [final matrix](../tools/rb16/results-20260911-stage1.json) has **126/126 completed
spring trajectories and 50,108 passing checks**. Counts include per-step state and
root assertions, not 50,108 independent cases. No final trajectory exhausted the
12-correction budget. All endpoints satisfy the unchanged normalized residual
criterion 1e-10; independent bisection agreement is checked within 1e-8 gap units.

The following predeclared eight-subdivision cycle slice uses factor 2, one outer
observation and zero hysteresis for the exact-predictor candidate. Occupancy counts
only true p<=U contact endpoints, including initially absent-but-needed contact.

| Fixture | Timing control | In-band endpoints | Newton iterations | Retunes |
| --- | --- | ---: | ---: | ---: |
| Weak single spring | in-Newton trim | 96.43% | 130 | 8 |
| Weak single spring | post-publication | 67.86% | 119 | 11 |
| Weak single spring | exact fixed-load intervals | 100% | 160 | 13 |
| Strong single spring | in-Newton trim | 25.00% | 141 | 4 |
| Strong single spring | post-publication | 75.00% | 127 | 9 |
| Strong single spring | exact fixed-load intervals | 100% | 171 | 13 |
| Heterogeneous springs | global in-Newton trim | 94.20% | 152 | 6 |
| Heterogeneous springs | global post-publication | 81.16% | 138 | 6 |
| Heterogeneous springs | local exact fixed-load intervals | 100% | 191 | 19 |

Exact outer intervals with zero hysteresis have 100% applicable occupancy at all
4/8/16 subdivisions, both factors and both observation windows. This is an
exact-reference result, not proof for estimated nonlinear FEM intervals. The
heterogeneous comparison changes locality as well as timing. Expanded triggers
can intentionally retain original-band violations; window two adds observations
that may take zero Newton steps. Costs and retune reversals for every alternative
are in the matrix; wall times are one local measurement, not performance claims.

Narrow enclosing intervals give 96.43%, 96.43%, 98.55% occupancy for the same slice;
mixed applicability near unloading retains the prior coefficient. Wide or
unavailable intervals produce no updates and occupancy of 17.86%, 0%, 7.25%.
The misleading predictor gives 89.29%, 64.29%, 94.20%, with maximum predictor-gap
error .6. These violations remain visible despite numerical convergence. The
handling is retention/reporting, not silent retry or a claimed safe fallback.

## Counterexamples and accounting

| Finding | Evidence | Disposition |
| --- | --- | --- |
| Average band does not bound individual gaps | d=[.2,.99], mean square=.51005, neither gap in band | reproduced |
| Post-publication state differs from solved state | k=1 to 2 at d=.7515214: residual 1.27e-16 to .2632253 | reproduced |
| Coefficient change is not displacement work | same event adds .10821353 barrier energy at fixed coordinates | reproduced |
| Empty uncertainty interval | K=[.1,10], p=[.2,.8]: lower 3.0053 > upper .24349 | reproduced, prior k retained |
| Incorrect inactive prediction misses demand | actual p=.4, predicted p=1, k=.0001 gives d=.400598<L | reproduced, unresolved model handling |
| Exact/narrow box roots in band | four enclosing corners per box, independent compiled-force roots | verified within quadratic assumptions |
| Conservative force-path accounting | 4096-panel Simpson path .4 to .8, residual 2.22e-16 | verified on one fixed-k path |

Every inner solve and full trajectory separates barrier displacement-energy change
from coefficient events and checks the telescoping identity. Full trajectories
retain all coefficient states, positive iterates, search lengths and endpoint
residuals. This is not full applied-load work, dynamic dissipation, coupled FEM
reaction balance, or an engineering acceptance certificate.

## Retained incomplete and exploratory outcomes

- `probe-01`: 24,057 checks passed but only 42/126 cycles completed. All 84
  iteration-limit trajectories remain stored. Near-stationary total-energy
  subtraction led to rounding stalls; one inspected residual was 5.23e-10.
- `probe-02`: stable factored/log1p energy differences complete 126/126 at the
  original tolerance and Armijo condition. Three 60-digit Decimal comparisons
  independently check this arithmetic. Its reduced timing counter incorrectly
  reset each load, so those timing results are superseded, not used above.
- `probe-03`: correct persistent trim counter, 49,982 passing checks, 126/126.
- `validated`: adds whole-trajectory energy accounting, 50,108 passing checks,
  126/126. No case, tolerance, coefficient alternative or failure was removed.

These are standalone probe corrections. They do not reproduce or repair a
production PolyFEM line-search defect. No causality about historical scenes is
inferred from them.

## Validation and publication

| Check | Result |
| --- | --- |
| Compiled effective IPC primitive and final matrix | exit 0; 50,108 checks; 126/126 trajectories |
| Frozen direction/search and coefficient event identity | passed |
| Positive endpoints, unchanged residual criterion, independent roots | passed |
| Decimal energy-difference and independent force integral | passed |
| Python syntax, new JSON, local documentation links, diff whitespace | checked before publication |
| Production binary/source/dependency baseline hashes | unchanged |
| Full PolyFEM rebuild, regression suite, public FEM/smoke/HDA runs | not run; no production or HDA changes in this standalone stage |
| Private scenes, including Teseo | not run |

Publish `tools/rb16/`, this contract/record and roadmap update to
`https://github.com/sdast9/polyfem.git`, branch `main`, under standing workspace
instructions. The task completion message and local publication record identify
the commit. Parent README changes are local outside Git. No companion or HDA
publication is needed.

## Next stage and remaining choices

Spring characterization is complete. **RB-16 itself is not complete.** Next run
actual real-form timing/refresh and public P1 FEM compression/unloading, retaining
failed solves, CCD and determinant checks, and endpoint/work observations. Start
from `tools/rb14/influence_probe.cpp`'s assembled reference and
`tools/rb15/feature_probe.cpp`'s bounded parent design. Predeclare comparisons before extending either; do not rerun completed
RB-14/15 matrices without a new question. Unknown-parent discovery, nonlinear
estimator reliability and coupled-contact behavior are not measured here.

The alternatives are now concrete: cheaper cadence feedback with possible endpoint
violations; bounded same-load interval corrections with extra solve cost; and
explicit unresolved handling for unavailable/misleading intervals. No candidate,
uncertainty enclosure, protection default, stopping gate or early-interruption
criterion has been selected. RB-17 remains dependent on the remaining RB-16 work
and the documented model decisions.

## Stage 2 — real-form and assembled FEM comparison (2026-09-11)

### Scope and reproducibility

The user requested “continue.” Resumed clean at `4af92be7b` on main; no interrupted
solver/build process or incoming changes were found. IPC/PolySolve effective
local overrides and revisions remain the stage-1 values. Read the handoff,
source callbacks, current dependency cache, RB-14 assembled fixture and RB-15
feature probe. The [stage-2 protocol](../tools/rb16/stage2-protocol.md) was written
before the main matrix and retains declared supplements. Production code,
dependencies, inputs, CCD, trial cap, floor retirement and tolerances are unchanged.

Fresh evidence: `outputs/rb-16/20260911-083920-stage2/` under the parent workspace.
The [runner commands](../tools/rb16/README.md#stage-2-assembled-fem-and-real-contact-callbacks),
[compact measurements](../tools/rb16/results-20260911-stage2.json) and
[provenance](../tools/rb16/provenance-20260911-stage2.json) are published. Full
Newton histories, coefficient events, displacements, every input/source snapshot,
compile/link output and process exits remain in local evidence. No result was
replaced by a successful rerun.

The probe uses real P1 Neo-Hookean assembly, implicit-Euler-scaled noncontact
forms, exact-selector IPC geometry, actual semi-implicit refresh/post_step,
real CCD and elastic validity checks. The alternative uses a positive coefficient
on a single known parent and recomputes full reduced tangent compliance/demand
outside each frozen solve. Both forms retain the 50-dhat trial cap. Finite element
meshes are n=2/4. The inertial predictor is fixed while the load parameter cycles;
this is not physical time integration or a dynamic refinement claim.

The first matrix applies a bottom-node force against a fixed top. It tests normal
contact-gap closure but can stretch the block. Eight separate prescribed-top
cycles exercise actual block compression/unloading and verify prescribed-coordinate
step validity before solving. Only the selected bottom node and long edge form
the collision model: neither comparison is whole-surface floor contact.
The known single parent avoids seam/multiplicity choices; coupled many-contact
mechanics and general RB-15 parent construction are not certified.

The actual callbacks run in an explicit driver schedule: load-start refresh,
post_step after accepted Newton steps, then post-publication refresh. The driver
recomputes derivatives after any event and reports the endpoint under its actual
solved coefficient. It is not the full production PolySolve/AL/restart lifecycle.

### Completed and incomplete outcomes

| Matrix | Cases | Completed cycles | Incomplete cycles | Checks |
| --- | ---: | ---: | ---: | ---: |
| Original direct-energy arithmetic | 76 | 35 | 41 | 43,586 |
| Paired gradient-integrated energy differences | 76 | 76 | 0 | 48,721 |
| Prescribed-top compression, integral arithmetic | 8 | 8 | 0 | 6,781 |

All processes/checks passed, totaling **99,088 checks** across these matrices.
The 41 original iteration-limit cycles are still incomplete, not “passed solves.”
No residual threshold, iteration budget or Armijo constant was relaxed. Converged
endpoints satisfy the RB-14 probe criterion `||g_free||/(1+||g_nc||+||g_c||)<=1e-8`.
This experimental criterion does not replace production's configured termination
contract. No same-load correction reached its 12-update budget in these cases.

The first original callback case stalls at residual 1.472813529e-8. A separate
read-only diagnostic demonstrates a feasible full Newton step reaching
4.834485606e-16, while direct energy subtraction reports +4.996003611e-16.
Three-/five-point gradient integration gives -9.941810505e-18 and
-9.941810498e-18; the Armijo RHS is -1.988362066e-21. This reproduces an arithmetic
sign discrepancy in this driver, not a new controller law or a production fix.

The separately declared paired matrix uses `d5+abs(d5-d3)<=Armijo_rhs` for the same
objective and frozen coefficient. It has no raw-to-integral fallback or silent
retry. Quadrature disagreement is an empirical diagnostic, not a certified error
bound. Added gradient evaluations and measured wall cost are retained. The two
modes remain separate results even though the paired cycles all complete.

### Controller comparison and physical observations

A predeclared slice, n=2/four subdivisions/multiplier=.01, using integral arithmetic:

| Alternative | Active endpoints in band | Newton steps | Retunes | Minimum gap |
| --- | ---: | ---: | ---: | ---: |
| Actual refresh/post_step control | 7/13 | 84 | 25 | .073312 |
| Tangent intervals, factor 2/window 1/no hysteresis | 13/13 | 129 | 20 | .071203 |
| Narrow uncertainty | 12/13 | 93 | 7 | .071351 |
| Wide uncertainty | 2/13 | 86 | 0 | .001436 |
| Misleading predictor | 11/13 | 101 | 7 | .059184 |
| Unavailable interval | 2/13 | 86 | 0 | .001436 |

Here dhat=.1 and the original band is [.0707107,.0948683]. Empty/unavailable
intervals retain prior positive k and explicitly report the unresolved state.
That preserves a parameter, not an engineering gap guarantee. The biased
predictor's maximum start-prediction error is .61414 dhat in this slice.

All 32 no-hysteresis tangent-interval configurations in the paired force matrix
have 100% occupancy among endpoints whose **assembled tangent predictor** has
p<=U. This is not a true nonlinear-demand oracle. On eight matched window-1 cases,
factor 4 uses 1,074 Newton steps/83 retunes versus factor 2's 1,199/140. That is a
bounded comparison, not a selected factor or a general performance optimum.
Expanded hysteresis can intentionally retain original-band violations; all
window/hysteresis and refinement results are retained in the JSON matrix.

The prescribed-top supplement completes all eight cycles, each with separation
and two contact-activation episodes. For n=2, four subdivisions, current timing
has 8/11 active endpoints in band and 8/9 tangent-applicable endpoints; the outer
candidate has 9/11 and 9/9 respectively. Active but separating endpoints explain
why these denominators differ. All four outer top-motion cases have 100% tangent-
applicable occupancy; they do not pull separating endpoints back into the band.
Minimum recorded det(F) across top cycles is .737530; prescribed-coordinate error
is zero and contact action/reaction checks pass. The refinement comparison changes
load increments, not the fixed dt or inertial history. Point-force amplitudes use
each mesh's K0, so that mesh comparison is not a same-load continuum convergence
study. Top-motion amplitudes are identical across meshes.

Start-of-solve tangent predictions are compared with realized endpoints under
the final coefficient; the full predictor/actual gap and errors are saved for
each solve. The supplied assembled Hessian has the same projection/model limits
as RB-14. General estimator coverage, friction, complete global reaction/work
balance and continuum contact accuracy remain unestablished.

### Endpoint refresh and work separation

The real callback experiment reproduces a post-publication force-state change:
in the top n=2/eight-subdivision case, the new-state residual reaches **.0570714**
at unchanged coordinates after the old state satisfied 1e-8. The old endpoint
is never relabelled as converged under the new state. Its before/after forces,
coefficients, refresh identity and energy change are recorded explicitly.

Per-direction/search trim and refresh identity checks pass. Every coefficient
change records zero displacement work and its fixed-coordinate contact-energy
difference. Conservative contact-energy changes along accepted displacements
remain separate; whole-cycle telescoping errors are at most 6.94e-17 in the
original matrix, 4.17e-17 in the paired matrix and 6.94e-18 in top motion.
These accounting identities do not establish full external-work balance or
physical dissipation. Force-quadrature/endpoint-energy and derivative controls
supply independent checks on finite fixed-state paths.

### Verification, source identity and publication

- Rebuilt `PolyFEM_bin` and `unit_tests`, exit 0. Production source/binary hashes
  remain unchanged; the standalone executable was compiled separately.
- Targeted cache/mapping/floor-retirement/direction-filter/BC/AL/contact/friction
  derivative suite: **29 cases / 1,980 assertions passed**. Negative-test error
  messages accompany the passing summary; no golden was changed.
- Four independent derivative-only controls: **352 checks**, maximum normalized
  gradient/Hessian error 4.92e-11/5.95e-11. Integrated and direct finite-path energy
  differences agree within 1.31e-16.
- Final-source validation: **1,713 checks** across those four controls and a paired
  callback case. The raw case remains incomplete; the integral case completes.
  Both reproduce the earlier endpoint gaps and residuals exactly. This validates
  the final source after adding the top-motion and derivative-only branches.
- Compilation, source snapshots, linked archive hashes, cases, runs and replay
  equivalence are in provenance. The original matrices used earlier saved source
  versions; additions and their final-source tests are explicit. No original
  matrix or failure has been overwritten.
- Formatter, Python syntax, JSON consistency, documentation links, source hashes
  and `git diff --check` are checked before publication.
- Full suite, scene smokes, HDA E2E and private/Teseo scenes were not run. This
  standalone research probe does not alter their production paths.

Publish the new source, protocols, case matrices, analyzer, compact results and
provenance with these records to `sdast9/polyfem:main`. The final task message and
local publication record identify the commit; the parent README is local outside
Git. No companion/HDA publication is needed.

### Final RB-16 disposition

**Characterized—decision pending**, within the stated spring/single-parent FEM
scope. The required bounded timing/bounds, uncertainty, load-cycle/refinement,
endpoint-state and cost comparisons are complete. No production controller,
coefficient, protection default, line-search arithmetic or acceptance gate changed.

The evidence supports carrying bounded same-load corrections with fresh assembled
predictions into an experimental RB-17 comparison. Keep factors 2/4 and protection
alternatives explicit; these data do not select production defaults. Current
callback timing can leave band violations and prepares a different force state
after publication. Empty/misleading estimates need explicit unresolved handling
or a separately selected estimator/protection response, not a claimed gap guarantee.

Before production integration choose the RB-14 estimator/uncertainty and zero-demand
policy, RB-15 parent/seam/discovery semantics, and RB-16 coefficient-update timing,
bounds and endpoint interpretation. Do not promote the gradient-integral arithmetic
control as a production solver repair without its own investigation. RB-17 is the
next named investigation; its model choices and remaining coupled/feature/friction
validation remain separate from this completed RB-16 characterization.
