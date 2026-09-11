# RB-17 — Integrated adaptive-barrier comparison

Date: 2026-09-11.
Status: **retired — per-contact band targeting withdrawn after the coupled gate; evidence retained; not integrated**.

The initial preflight below is historical. The approved-candidate investigation
is recorded in the dated section that follows it, and the retirement decision in
the final section.

## Authorization and prerequisite audit

The user requested “Let's move to complete RB-17.” Read the robustness plan,
PF invariants and floor retirement, RB-14/15/16 contracts and final validation
dispositions, and RB-04 accounting record. RB-17 expressly requires authorization
of a concrete candidate before integration. The prior records leave that choice
open. The [candidate proposal](rb-17-proposal.md) gives a recommended package,
alternatives, measured tradeoffs and a comparison/acceptance protocol.

This is preparatory documentation, not completion of RB-17's integrated matrix.
No production estimator, assignment, controller, protection or stopping policy is
selected by this document. No new defect reproduction or solver run is claimed.

## Verified incoming state

PolyFEM main was clean at `12439a1f5`; IPC semi-implicit-stiffness was clean at
`af317a65d69d0ac7c5efa4bf103bf75e280c323b`; PolySolve iteration-callback was clean
at `4d372fa8a73f42bc224e31d464f1a308e1159ba8`. CMake's effective IPC and PolySolve
source paths point to the local companion checkouts and agree with recipe pins.
No existing PolyFEM/RB-15–17 simulation process was found or interrupted.
All incoming outputs remain untouched.

Read current BarrierContactForm interfaces and refresh/post_step call sites;
the existing feature cache and callback timing do not already implement the
proposed parent/outer-solve contract. Historical results in the proposal come
from the checked-in prerequisite records; they were not rerun in this audit.

Local evidence is `outputs/rb-17/20260911T142110Z-preflight/` in the parent
workspace. It records Git state, effective cache/dependency identity and hashes.
No build configuration, production binary, HDA asset or input was changed.

## Validation and publication

Documentation link checks and `git diff --check` apply to this stage. Builds,
solver tests, scenes, physical metrics and timings are **not run/not measured**:
there is no implementation to validate yet. No private scene or Teseo was run.
Publish the proposal, this record and plan status to `sdast9/polyfem:main` under
standing project instructions. The task response records the publication commit.

## Remaining work

Select the concrete candidate package or amend its estimator, parent, timing or
protection choices. Then write the exact case manifest, implement the opt-in mode,
execute ablations/reference comparisons and required build/test/smoke/HDA checks,
and publish the decision report. RB-17 remains incomplete until those acceptance
checks and evidence are delivered. Production default promotion is a later
explicit decision, even if the experimental comparison succeeds.

## Approved candidate: coupled gate (2026-09-11)

### Authorization, execution and provenance

The user instructed "Please proceed with this proposal." This authorizes the
five-point package in [the proposal](rb-17-proposal.md), including reporting
unresolved attempts. The assistant stated that the coupled-contact check would
run first. This stage executes that check; it does not withdraw the user's
authorization to integrate the selected candidate with its known limitations.
A different coupling law would be a new model choice.

Started clean at PolyFEM `29a18c3f1` on main. Companion checkouts were clean and
unchanged at the full revisions above; current CMake effective source paths
agree. No incoming solver/build process was interrupted. Fresh local evidence:
`outputs/rb-17/20260911T143421Z-coupled/`. `baseline.json` records repository,
binary and cache identities. The exact 144-run manifest and protocol were
committed as `8f954f27d` **before execution**, with one held-out stronger-coupling
case declared in that manifest. No case or criterion was changed after results.

The runner compiles RB-13's existing bridge against the actual IPC barrier
primitive. Python/NumPy implements the isolated Newton driver. Full output,
compile log, provenance and source snapshots remain local. Small published
[results](../tools/rb17/results-20260911-coupled.json),
[audit](../tools/rb17/audit-20260911-coupled.json) and
[provenance](../tools/rb17/provenance-20260911-coupled.json) accompany the
[reproduction commands](../tools/rb17/README.md).

### What was tested

The six systems use `E_nc=u^T H u/2`, affine gaps `d=p+J u` and compiled ordinary
IPC barriers. Full noncontact H is SPD. Shared prediction and compliance are
recomputed each outer solve; `W=J H^-1 J^T` and `K_i=1/W_ii` are exact here.
The contact parents are fixed abstract identities, not a collision surface.
Initialization uses the raw normalized mapped Rayleigh coefficient, with no
trim/cap/calibration. This is not execution of the full SemiImplicit baseline.

The approved scalar rule freezes each coefficient during Newton, uses the
geometric-center/half-upper interval target, corrects only applicable out-of-band
contacts, and limits changes by factors 2/4. Unchanged coefficients outside the
band report `unresolved_coefficient_fixed_point`; the 1e-13 relative equality
screen classifies lack of progress and never accepts a band violation. Up to 12
corrections are allowed. No retry, restoration or new coefficient floor exists.

Every case has three length conversions (.001,1,1000), two parent orders, two
factors and two separately declared arithmetic branches. The direct branch uses
ordinary energy subtraction. The algebraically stable quadratic control uses
RB-16's factored barrier difference. Neither is a production solver modification
or fallback. Both use residual tolerance 1e-10, 100 Newton steps, 60 backtracks
and Armijo constant 1e-4. The residual's force reference converts with units.

### Outcomes retained

| Arithmetic | Runs | Band satisfied | Converged but controller unresolved | Numerical failure |
| --- | ---: | ---: | ---: | ---: |
| Direct objective subtraction | 72 | 38 | 10 | 24 |
| Separate stable arithmetic control | 72 | 40 | 20 | 12 |

All 36 numerical failures are retained as line-search-limit results. The stable
control is also imperfect and must not be described as fixing arithmetic failures
generally. The six mechanical cases are independent inputs; their 144 unit/order/
factor/arithmetic combinations are not 144 independent physical benchmarks.
No run reached the outer correction budget: it either satisfied the band,
stopped at a coefficient fixed point, or encountered a numerical failure.

### Reproduced feasible-band counterexample

The held-out case has

```text
W = [[1, .8], [.8, 1]], p = [.3, .1], dhat = 1
L = sqrt(.5) = .7071067812, U = sqrt(.9) = .9486832981.
```

The full tangent and free predictor are exact; each diagonal stiffness is one.
Both contacts satisfy the selected applicability test p_i<=U. Under factors 2
and 4, all 12 stable-control unit/order variants reach the same scalar-controller
fixed point with one gap outside the band. All 12 direct-arithmetic variants of
this case instead retain numerical failure before the controller correction.

At unit scale, forward order, factor 2:

| Quantity | Selected scalar rule | Coupled feasibility reference |
| --- | ---: | ---: |
| First coefficient | 1.6009423172 | .6050038865 |
| Second coefficient | 2.7777777778 | .5316530794 |
| First gap | .9572881197 | .8943506737 |
| Second gap | .8909575536 | .7750225616 |
| Independently evaluated normalized residual | 2.95e-13 | 2.50e-13 |
| Both gaps in band | no | yes |

The independent reference enumerates the feasible polygon
`L<=p+W F<=U, F>=0` and averages its vertices to obtain a strictly positive
interior force. It converts that force to coefficients and verifies equilibrium
using coordinate minimization with scalar bisection. This centroid is a diagnostic
witness, **not a selected production force target or optimization policy**.

A separate 60-digit evaluation confirms the scalar interval target remains
`1.6009423172377934`, matching the stalled controller coefficient, while the first
gap exceeds U by `.0086048216092*dhat`. The coupled equation error is 7.33e-13.
The second contact contributes `.5891712398*dhat` to the first gap through W12.
Fresh scalar predictions cannot remove that coupling because they continue to
use the same free p and diagonal K. Increasing the update factor alone does not
resolve this fixed point. This reproduces a limitation of the approved model,
not a production implementation defect or proof of physical inaccuracy.

### Infeasible-band control is a different outcome

For `W=[[2,1],[1,2]]`, `p=[.7,.1]`, nonnegative forces and d2>=L imply
`d1>=.7+.5*(L-.1)=1.0035533906>U`. No repulsive coefficient selection can put both
gaps in this band. This is **infeasibility of the chosen simultaneous gap target**,
not absence of a physical equilibrium. The independent halfplane enumeration
and analytic inequality agree across all conversions/orders.

At unit scale the candidate equilibrates at approximately `[1.06974531,.83949062]`
and retains one violation. Its first contact is outside barrier support, opened
by mechanical coupling. Any future applicability rule must distinguish this
case from a contact requiring protection; forcing it downward would require a
different force model. The current stage does not change applicability.

### Verification and limits

- Main probe: **3,372 checks passed / 144 recorded runs**, exit 0. Checks include
  frozen direction/search state, SPD, exact shared prediction, full reconstructed
  equilibrium, coefficient updates, finite differences and independent roots.
- Independent artifact audit: **488 checks passed**, exit 0. Explicit analytical
  barrier force, bisection equilibria and a 60-digit held-out check confirm the
  counterexample without the Newton Hessian/line search. Maximum normalized
  coupling defect is 3.50e-12.
- All 144 frozen-coefficient coordinate-minimization oracles converged. This
  does not relabel any failed Newton/controller run as successful.
- 100 applicable unit/order comparisons passed; maximum normalized endpoint
  difference is 1.11e-16. The remaining 44 comparisons are explicitly unavailable
  because a direct comparison endpoint or its reference did not converge.
- Coefficient events have zero displacement work. Independent conservative
  contact-energy telescoping error is at most 8.72e-17; event arithmetic error
  is at most 1.30e-18. Full applied-load work is not measured in this static gate.
- Peak Python process RSS was 38,420,480 bytes on macOS. Per-run estimation/wall
  times are recorded but are single tiny-fixture timings, not production cost.
- Python syntax, JSON consistency, documentation links, source identity and
  `git diff --check` are checked before publication. Production source, binary,
  cache, companions and HDA assets remain unchanged. No full build, C++ regression,
  scene smoke or HDA E2E was run: production integration has not happened.
- FEM, real feature transitions/parent discovery, geometric CCD, det(F), boundary
  reactions, transient approach, friction and full production-wrapper behavior
  are **not measured**. Affine positive-gap steps are exact only for these springs.
  No private or Teseo scene was run.

### Current decision and remaining authorized work

**RB-17 is not complete.** The approved scalar candidate can still be integrated
with this known limitation and explicit unresolved outcomes. Its non-guarantee
for coupled bands was already in the approved proposal. The alternative is to
investigate a coupling-aware force selection before production wiring; that is
recommended because the feasible counterexample persists with exact estimates.
The assistant asked the user which direction to take while preparing this record.

For a coupling-aware investigation, keep `d=p+W F`, F>=0 and the same band.
Compare a feasible joint target against independent scalar selection, and report
empty feasible sets without attraction or artificial protection floors. The
current centroid witness establishes existence only; a target-selection objective,
zero-force handling and nonlinear outer policy still need a concrete contract.
Do not silently install that reference or replace the approved rule.

If continuing unchanged, retain the held-out and incompatible cases as unresolved
in the eventual ablation report; do not claim factor 4 or extra corrections fixes
them. The parent/feature implementation, full production opt-in wrapper, remaining
FEM/dynamic/heterogeneous comparisons and required regression/smoke/HDA validation
remain authorized but unperformed. Production promotion is not authorized.

## Retired: per-contact band targeting (2026-09-11)

**User decision (2026-09-11):** retire per-contact band targeting from RB-17.
After reviewing RB-01–RB-17 the user chose to keep IPC and the existing global
gap band, and asked for the quick-block repairs ([RB-18](rb-18-quick-fixes.md))
and the line-search/step-1 work ([RB-19](rb-19-line-search-roundoff.md)) instead.
This section records what is withdrawn, what is kept, and why.

**What is withdrawn.** The approved five-point candidate's controller — one
coefficient per contact, each corrected independently so that *its own* gap
lands in the interval `[L, U]` — is no longer a production target. The
coupled gate above is the reason: with the exact free predictor and exact
diagonal compliance, independent scalar corrections reach a fixed point with a
gap outside an attainable band (the held-out `W12 = .8` case; the second contact
contributes `.589 d̂` to the first gap through the coupling, which no per-contact
rule using the same free `p` and diagonal `K` can remove). A coupling-aware joint
force selection would be a different model choice; the user did not select one.
No production integration of the per-contact band is authorized or pending.

**What is kept.** The global band (`trim_lower`/`trim_upper` acting on the
single trim, the batch's average and minimum gap) remains the production
controller, unchanged. Per-contact Hessian scaling of κ remains (RB-18 repaired
its law). The RB-17 proposal, this record, the predeclared protocol, the 144-run
coupled matrix, its independent audit and the reproduction tooling under
`tools/rb17/` are retained as evidence; they are measurements, not goldens.

**What is not claimed.** Retiring the candidate does not certify the global band,
does not resolve the RB-04 post-publication force drift, and does not replace the
still-open items that the RB-18 handoff named as next: force-continuation κ
(set each existing contact's κ for the next step from its realized force at the
published endpoint; Hessian value only for new contacts) and parent-keyed κ in
the toolkit builder (removing the EV/VV jump). Those are the recommended
successors to RB-17's goal and need their own items and authorization.

**Repository effect.** No solver, dependency, HDA or scene source changes. The
coupled-gate record, the plan status row and the `tools/rb17/` files that the
RB-17 session had left uncommitted are committed together with this section;
the RB-18 commits deliberately excluded them. The RB-17 row in
[robustness-plan.md](robustness-plan.md) now reads *retired*.
