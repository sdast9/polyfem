# RB-04 candidate investigation log

## Authorization and durable objective — 2026-09-09

The user selected the coherent barrier-model direction and requested continued
RB-04 testing to determine a candidate, with durable hypotheses/results/handoffs.
The user observes a possible sweet spot in Trim Band Lower/Upper. This authorizes
comparative testing and observational accounting, not an untested production law.
Preserve CCD, retired constraint floor, separate trial cap, and finite friction
policy. Public fixtures only; no Teseo or private scenes.

Starting clean PolyFEM main: 2c45411a2; effective IPC local af317a65;
PolySolve local 4d372fa8. No pre-existing solver/build process found.
Prior endpoint results and unresolved scope: [validation](rb-04-validation.md).

## Hypotheses and decision criteria

H1: A trim band avoids both near-singular contact and excessive barrier stiffness,
reducing solver difficulty. Test separately from changes in mechanical response.
H2: A useful band has a region of stable reactions/displacements under band and
load-step refinement, rather than only one fastest parameter pair.
H3: Fixed-coordinate coefficient changes explain part of apparent energy-budget
defects. Record every event before interpreting trajectory work differences.
H4: Friction residual sensitivity can be amplified by changes in normal-force
scale; this is not established causality and must be separated from finite lag.

Band parameters compare squared gaps with dhat squared. Default lower=.5 and
upper=.9 correspond to sqrt(.5) and sqrt(.9) in gap/dhat. The lower trigger also
uses min-gap slack; calibration can supersede the upper fallback. These are not
hard constraints or a guaranteed endpoint gap interval.

## Predeclared first comparison

Use unchanged public quasistatic, transient and friction cube/slab fixtures,
dt=.25, dhat=.001, material/mesh/tolerances unchanged. Five pairs:
(.1,.9), (.5,.9) default, (.8,.9), (.5,.6), (.5,.99).
This varies one threshold at a time; it is a pilot, not a search for an optimum.
Record all outcomes, endpoint forces/energies/gaps/trim/refresh IDs, independent
elastic reaction reconstruction, BC/det(F), and observed retune log counts.
Timeout 120 seconds per process is an experimental observation budget, not a
production stopping rule. Retain failed/partial runs. No accuracy threshold is
selected. Timing is descriptive until repeated controlled measurements exist.

## Required continuation (do not substitute pilot completion for goal)

1. Analyze pilot and choose informative comparisons, retaining negative results.
2. Instrument fixed-coordinate coefficient events with read-only before/after
   snapshots, explicit event/step identity and objective-to-physical unit scaling.
   Avoid double-counting nested refresh/calibration/bump events. Validate on/off
   equivalence and independent small-model energy differences.
3. Record initial state and integrate prescribed/body work and friction slip work;
   distinguish coefficient events, feature jumps and discretization error.
4. Refine load/time integration with at least three increments on selected bands;
   retain failures. Evaluate response sensitivity, not merely process completion.
5. Make a concrete candidate determination, including interpolation scope and
   lifecycle, using the complete evidence. A favorable band alone cannot repair
   zero-median, feature-continuity or interpolation counterexamples from RB-02/03.

Results, exact evidence paths and next actions will be appended below. Production
changes require rebuild, affected tests/smokes and publication per AGENTS.md.

## Pilot results — 2026-09-09

Evidence: parent `outputs/rb-04/20260909-trim-band-pilot/`; 15 serial runs,
all complete four steps, no timeout. Exact commands/input/binary hashes and all
logs/endpoints/VTUs retained. Published compact data:
[trim-band-results-20260909.json](../tools/rb04/trim-band-results-20260909.json).
Runner and independent summarizer are in `tools/rb04/`. Production unchanged.

Final frictionless quasistatic measurements:

| lower, upper | min gap/dhat | trim | support reaction z | free residual |
| --- | ---: | ---: | ---: | ---: |
| .1, .9 | .0581763 | .729692 | -3629169.13 | 1.12e-7 |
| .5, .9 | .576232 | 8 | -3640034.65 | 1.06e-7 |
| .8, .9 | .863655 | 64 | -3644527.55 | 3.74e-7 |
| .5, .6 | .576232 | 8 | -3640034.65 | 1.09e-7 |
| .5, .99 | .576232 | 8 | -3640034.65 | 1.04e-7 |

Reactions are independently assembled elastic top reactions, equal to support
reactions in these quasistatic fixtures. The transient table uses elastic-only
top reactions and must not be advertised as total transient reactions.
Reaction range across lower values is approximately .42% of default magnitude.
Largest full-DOF endpoint displacement difference from default is .000616274
over four steps for lower=.1, and .000367171 for lower=.8. Upper-only changes
remain below 2e-14 across all three fixture types. This pilot did not sufficiently
excite the upper-band distinction; it cannot establish that upper is irrelevant.

Friction final updated-lag free residuals for lower=.1,.5,.8 are
59488.55, 59485.57, 59620.34. All retain the unconverged lag state.
Band tuning does not resolve the measured friction discrepancy here; causality
or acceptability is not established. Transient frictionless residuals remain
approximately 1e-7 to 4e-7. Independent energy/det(F)/BC reconstructions use the
existing RB-04 tolerances, checked by the summarizer.

Partial trapezoidal top work minus elastic energy change is about 8060.53,
7894.61, 7889.60 for the three quasistatic lower values. Final barrier energies
are 684.91, 405.40, 127.68 respectively. DO NOT interpret their difference as a
balance defect: initial barrier energy, coefficient events, feature jumps and
quadrature error are missing. This is a concrete reason to complete event
accounting before declaring an energetically preferable band.

H1 is plausible but not established: all runs succeed, so these fixtures do not
locate a robustness failure boundary. H2 remains pending refinement; response
changes are measured, no engineering tolerance is selected. H3 remains pending
event instrumentation. H4 is not supported as an explanation of the large lag
residual by this parameter pilot alone. No optimal band or new default selected.

## Exact next implementation entry points

`BarrierContactForm` mutates semi-implicit coefficients in refresh, calibration,
bump and post-step (including first-contact conditioning). `retune_on_stall`
nests refresh plus bump/calibration; refresh also nests bump/calibration. Observe
outer events once to avoid double-counting, or record primitive mutations with
explicit parent IDs. A direct bump has no x argument: establish its evaluation
coordinate from the calling operation, not an assumed previous endpoint.
`diagnostic_snapshot(x)` already copies the coefficient cache and builds a fresh
broad phase; it is the read-only before/after reference. An initialization with
no prior snapshot needs an explicit unavailable or initial-model convention.
`ContactForm::set_barrier_stiffness` initializes trim through SolveData; include
this in the event scope inventory rather than promising all mutations blindly.

Attach event collection before solve-start updates and tag step/subsolve/restart
context in `NonlinearElasticVarForm::solve_tensor_nonlinear`. Preserve partial
events on exceptions. Test retune/no-op/new-feature and nested updates, on/off
trajectory equivalence and weights. Do not silently include changes in time
integrator weights as physical work. Keep disabled diagnostics allocation-free
where practical. Only after this validated instrumentation, run selected lower
bands (.1,.5,.8) at three load increments with complete work conventions.

## Event instrumentation in progress — 2026-09-09

Current evidence folder: `outputs/rb-04/20260909-coefficient-events/`.
The prior goal turn was progress: committed/published the measured pilot.
This stage adds opt-in outer-operation observers for refresh/calibration/stall/
post-step, with private same-coordinate snapshots and no nested double counting.
The transient loop also refreshes after saving each accepted endpoint; it is
now observed as `between_steps_after_endpoint`. Time-weight changes remain
separate, and external embedding/manual setters are not advertised as covered.

Accounting correction to H3: coefficient-event deltas occur at Newton iterates,
which are not a physical trajectory. Their sum must NOT simply be subtracted
from endpoint energy minus physical work. A valid budget needs an explicit path
convention for contact work or a common coefficient state at physical endpoints,
as well as the parameter-change term. Event coordinates/phase are retained so
future reconstruction can distinguish these paths. This qualification is central
to candidate assessment, not just a missing diagnostic field.

Planned tests: isolated real-form no-op/doubled-H/retune/new-feature comparisons,
weights .25/1/4, identical observer-on/off energy/gradient/Hessian and provider
call counts, observer failure isolation, provider-failure event retention; public
on/off fixtures and deliberate restart failure, stream arithmetic/phase checks.
The first build compiled solver sources but failed the new test's missing Catch
matcher header. Added the required header; original build log retained. This was
a test compilation error, not a production finding.

## Event-stage results and next handoff — 2026-09-09

Final source includes force as well as energy snapshots. Both targets build;
affected suite 2178 assertions/31 cases passes (171 new event assertions).
Three off/on public pairs complete, max endpoint difference 1.565e-14; the
deliberate failure remains -6 in both modes with its stall event retained.
Five smokes and HDA end-to-end pass. Final artifacts use `force-endpoints/`,
`affected-force-tests.log`, `force-smokes/`, `force-hda.log` within the event
evidence folder; energy-only development runs remain preserved. Compact results
are `tools/rb04/coefficient-events-results-20260909.json` and detailed scope is
in the appended [validation stage](rb-04-validation.md).

New finding: the post-publication refresh at final quasistatic coordinates
changes contact energy by +70.8181 and the free contact-force vector by norm
145746.710, despite the saved state's approximately 1e-7 free residual under
its earlier coefficients. The force change is independently located at exactly
the saved coordinates and before-refresh contact values match the endpoint.
This is model-state drift after publication, not proof of displacement error.
Transient/friction event changes cannot be called total post-update residuals.

Candidate hypothesis H5: retain a fixed, explicit coefficient state through a
complete minimization and associate published state/reactions with that model.
Outer continuation is permitted only with a defined target and an equilibrium
assessment under the claimed final coefficients. The current post-publication
refresh evidence supports this requirement, not a particular coefficient law.

NEXT: implement a bounded trajectory/refinement runner for lower=.1,.5,.8,
upper=.9, with dt=.25,.125,.0625, first quasistatic then transient/friction.
Use fresh directories and preserve timeout/failure outcomes; do not rerun the
full pilot. Establish initial snapshot and energy/state conventions explicitly.
For the fixed-obstacle public fixture, prescribed top work can be integrated
from independently reconstructed elastic reactions in quasistatics; transient
needs full reactions. Examine `FrictionForm` before using gradient dot slip:
updated-lag endpoint forces and the forces actually used during the solve are
different discrete choices. Record both if available; do not equate friction
potential to dissipated work. Derive the physical contact-work convention before
combining optimization-event energy sums with a physical-time budget. Run three
load increments and compare observed quadrature convergence, force response,
band dependence, and failure behavior. No numerical/engineering tolerance or
new default is implicitly selected. Upper-band effect still needs an informative
loading/unloading fixture; the first pilot did not excite it.

## Refinement stage started — 2026-09-09

Clean baseline `9e7c3b59f`. User authorized tightly scoped Luna assistance with
minimal monitoring. One read-only Luna agent is auditing the friction/discrete
work convention; it makes no source edits or simulations. Main task owns the
runner and serial scene execution. No production source changes at this stage.

`tools/rb04/run_refinement.py` records the predeclared lower=.1,.5,.8,
upper=.9 and dt=.25,.125,.0625 matrix in a fresh manifest before execution.
First evidence: parent `outputs/rb-04/20260909-refinement-quasistatic/`.
Each solver gets a 120-second experimental observation budget; failures and
partial outputs are retained and do not abort the remaining matrix. Work and
response analyses use saved accepted states, not restart trial coordinates.

Current discrete conventions: prescribed work uses right endpoint and trapezoid
reaction dot full nodal displacement increment. Component resistance costs use
positive physical gradient dot increment. Initial undeformed, at-rest and unloaded
state is checked through initial VTU and the first coefficient snapshot (zero
energy/force), not assumed for arbitrary fixtures. Friction endpoint gradients
are the updated-lag state and may differ from the solved law. No event-energy
sum is subtracted from physical work. Elastic stress-work minus elastic energy
is explicitly a quadrature remainder. Combined residual virtual work is an
algebraic diagnostic, not physical certification. Missing early active gaps are
unavailable rather than zero. Runner analyzes each completed/failed process and
persists results immediately; corrected postprocessing can reuse saved runs.

## Refinement results — 2026-09-09

Published compact data: `tools/rb04/refinement-results-20260909.json`.
Evidence folders in parent outputs/rb-04: `20260909-refinement-quasistatic`,
`20260909-refinement-transient`, `20260909-refinement-friction`, and
`20260909-refinement-quasistatic-repeat`. Primary 27-run matrix: quasistatic
7/9, transient 9/9, friction 7/9 complete; 188 accepted endpoints total.
One exact repeat of each failed quasistatic configuration also fails (0/2).
No process hit the 120-second experimental timeout.

All six failures are at step 1, dt=.0625, lower=.1 or .8, after 20 restarts.
Every observed coefficient event has zero active contacts and trim=1. This
does not establish band-trigger causality. The default-band step succeeds with
the same lack of active contacts. Preserve the unexplained pre-contact failure
as a robustness limitation; do not call .5 an optimum based on this contrast.
No retries, tolerances, stop rules or production defaults changed.

At lower=.5, upper=.9, quasistatic final reaction magnitudes for dt=.25,.125,
.0625 are 3640034.645, 3640327.728, 3640473.181 (range ~.012% of coarse).
Final gaps/dhat are .57623,.59842,.60952: a stable reaction does not imply
gap independence. Trapezoidal support work minus elastic energy decreases
7894.607 -> 2083.172 -> 392.000; this is incomplete contact accounting, not a
closed energy balance. Transient reaction trends are similarly small at the
default band, but lower=.1/.8 show different fine-step coefficient histories.

Default-band friction final reaction magnitudes are 3923675.572,
3975714.330,3999292.641 (~1.93% coarse-to-fine change). Updated-lag residuals
are 59485.575,27779.768,13356.634. Updated-lag right resistance-work estimates
are 22930.380,19050.134,17557.050. Prescribed right work minus all component
costs is -9289.886,-4156.292,-1997.122, consistent with a nonzero free-residual
work contribution, not extra physical dissipation. Thus H4 requires direct
pre/post lag force measurement; band tuning alone did not remove the discrepancy.

Independent tetrahedral mass integration verifies inertia-work/kinetic/IE
dissipation identity at 84 transient endpoints (maximum absolute inertia-work
error 7.82e-14). Its accumulated IE dissipation increases with refinement in
these runs; do not claim monotonic convergence of physical dissipation.

Development evidence: the first running quasistatic runner version could not
divide an unavailable early active gap by dhat. The default fine run's solver
exit was 0 and all 16 endpoints exist. Fixed only postprocessing and reanalyzed
saved data; original parser error remains in raw results and compact data.
Both initially executed runner versions were recovered and matched to their
startup SHA-256, then saved as runner-at-start.py. Later runs save this source
automatically. No numerical threshold changed and no failed solver run was erased.

The [work convention](rb-04-work-convention.md) records the reviewed Luna audit,
the quasistatic-integrator correction, equations and the next minimal observations.
NEXT: capture pre/post lag friction gradients and evaluate the endpoint coefficient
snapshot at the prior physical coordinates. These are observations, not changes
to model or lag policy. Validate against unobserved controls, then reuse the
three-increment default-band comparison for work reconstruction. The mid-band
remains a useful comparison baseline, not a selected universal sweet spot.

## Physical-state pair instrumentation in progress — 2026-09-09

Refinement/tools/work-convention stage committed as `798cd4bd6` and pushed to
origin/main before further source edits. New evidence directory:
parent `outputs/rb-04/20260909-physical-state-pairs/`.

Observational source change in `NonlinearElasticVarForm.cpp`: retain friction
gradients immediately before and after the final lag update; reconstruct the
endpoint residual with pre-update friction and all other endpoint forms fixed.
Also evaluate barrier energy at solve-start coordinates using a private copy of
the endpoint coefficient snapshot. These implement the two minimal measurements
specified in the work-convention document, without changing forces used in any
minimization, the coefficient controller, or lag/termination policy.

The build completed. Running affected suite, standard on/off/failure endpoint
checks, smokes and HDA E2E against the updated binary. An initial runner command
misspelled the home-directory path; directory creation failed before any solver
launched. Corrected to an absolute path generated from cwd; no output was moved.
`tools/rb04/analyze_state_pairs.py` independently checks the common-snapshot
start energy against solve-start event energy times the changed global trim
when refresh ID is unchanged, and checks post-update friction against the
endpoint's separately recorded friction gradient. It reports the declared
physical-coordinate coefficient decomposition and paired friction work estimates.
Next run the remaining default-band .125/.0625 increments after on/off checks
pass; reuse the three default .25 on-runs rather than rerunning them.

## Physical-state pair results — 2026-09-09

All required build/affected tests (2178 assertions/31 cases), default on/off
measurements (max displacement difference 2.004e-14), failure pair, five smokes
and HDA E2E pass. The default .125/.0625 extension completed five of six first
attempts. Fine friction failed before contact with 20 restarts; one identical
repeat completed. Retain both. This extends the evidence that observed startup
failures cannot be called a causal trim-band sweet spot. New compact data:
`tools/rb04/physical-state-pairs-results-20260909.json` (10 trajectories including
failure, 9 complete, 84 accepted endpoints). Raw coarse runs are `endpoints/`,
fine runs are the three scene folders, and repeat is `friction-fine-repeat/`.

Common start energy agrees with the independent event/trim-ratio route in 75
applicable frames (max error 1.137e-13). Paired updated friction matches the
endpoint's separately recorded friction component. Quasistatic parameter-energy
terms remain finite as dt decreases: 544.163 -> 589.017 -> 625.316, under the
explicit physical-start convention. This cannot be silently omitted from an
energy account; a contact gradient line integral still needs feature/path checks.

H6 (supported in measured fixtures): the large friction diagnostic residual is
from using the post-update lag state, not nonstationarity of the prior frozen-lag
endpoint. At dt=.25,.125,.0625, pre-update residuals are
1.236e-8,1.250e-8,2.260e-6 versus post-update
59485.575,27779.768,13348.598. Before-update right friction work is
13640.494,14893.843,15558.075; after-update work is
22930.380,19050.134,17557.237. The coarse difference 9289.886 accounts for the
earlier virtual-work discrepancy. The finite-lag approximation still shows
increment dependence; no physical acceptance or larger lag budget is selected.

NEXT bounded candidate comparison: distinguish a coherent positive frozen scalar
barrier coefficient from the existing per-feature curvature rule in RB-02's
feature-switch fixture, with explicit coefficient units and geometry/mapping
controls. A follow-up read-only Luna audit (same agent) is preparing the exact
Fixed-vs-SemiImplicit setup and confounding mode switches. Do not claim that a
uniform scalar is physically calibrated or generally mesh-independent; it is a
coherent-objective comparison baseline. Then compare fixed-snapshot path work
against energy changes and design an unloading/upper-band fixture. Keep H5
(published-state coefficient identity) and H6 (lag-state identity) in the
candidate's contract; neither alone fixes geometric feature discontinuity.

The Luna follow-up audit has completed. Its reviewed equations, dt-weight
qualification, mode-confounding limits and exact next probe protocol are saved in
[candidate comparison](rb-04-candidate-comparison.md). The key first comparison
is Fixed k=70 versus the existing 70/55 EV/VV snapshot fixture; require vanishing
energy/gradient jump for the candidate where supported, not an unjustified
continuous Hessian. No scene-scale coefficient has been selected or inferred
from endpoint min/max ranges. Both Luna tasks were read-only, with no progress
polling; their bounded outputs were reviewed against source/units.


## Completed fixed-coefficient boundary and work probe (2026-09-09)

The predeclared comparison and follow-up are now complete; see
[candidate comparison](rb-04-candidate-comparison.md) for exact numerical results,
limits, evidence paths and next experiment. First probe: 66 checks. Extended
probe: 68 checks. All passed against the tested `e652fae53` production build.
H7 supported on this fixture: uniform positive fixed k removes the reproduced
feature-coefficient energy jump. Gradient-path quadrature error converges to zero
for Fixed k=70 (~7.98e-7 at 1024 panels/side), while Semi converges to a missing
jump of -44.4977388. The fixed candidate also passes interior derivatives,
uniform-curvature normalization and length/energy conversion controls.

Advance the fixed-positive-scalar ordinary barrier family as the next coherent
candidate. This is a recommendation, not a production default or calibrated k.
Remaining: declared public-scene reference coefficient, load/unload comparison
including active upper trim control, and separate friction-lag refinement.
Do not repeat these boundary probes unless relevant code changes or a new
counterexample warrants it. Do not interpret pre-contact run failures as a
causal trim-band result. All original failures and successful repeats remain.
