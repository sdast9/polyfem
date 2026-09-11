# RB-16 — Gap controller and update timing

Date: 2026-09-11.
Status: **in progress — spring stage 1 characterized; real-form/FEM stage pending**.

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
