# EF-02/03 — Guarded trim estimate and force-weighted global band

Date: 2026-09-26. Status: implemented and measured as an opt-in experiment; strict adoption gates unmet; defaults unchanged.

## Question

Can an opt-in early estimate and a force-weighted proportional global band avoid
the expensive descent from trim 1 without worsening the multi-step solution?
EF-01 found that a stationary gradient-balance estimate is an identity, whereas
an off-equilibrium estimate can identify R4's useful plateau. Its force-weighted
gap responds to the trim on scenes where the production rms gap barely moves.

## Design

The existing Hessian-based per-contact law, continuation, CCD, trial cap,
collapse statistic, emergency bump and in-solve emergency climbing budget stay.
`band_statistic: rms` and `initial_trim_estimate: false` remain the defaults.
No Houdini control is added. The stall trigger stays absolute and the AL budget
and automatic retry remain off.

The opt-in `force_weighted` band weights each active primitive collision's
normalized gap by its local barrier-gradient norm, including its coefficient
and collision weight. Common positive form weight and trim cancel. A rescaled
accumulator avoids overflowing the sum; empty/invalid forces give no update.
The target interval is [.35,.50] in distance/dhat, with a .025 expanded dead
zone. Every 10 accepted iterations, outside the dead zone, the multiplier is
the scalar IPC barrier-force ratio at the observed gap and nearest band edge,
clamped to [1/4,4]. This is a proportional heuristic, not an equilibrium or
per-contact accuracy oracle. Refresh and stall observations may update sooner.
The factor-four choice uses RB-16's measured candidate as a starting point,
not as a general optimality claim. Occupancy uses the unexpanded target band.

With `initial_trim_estimate: true`, the first eligible non-endpoint refresh or
accepted iterate can use the non-contact/barrier gradient balance; a rejected
signal stays pending for later observations, including the first stall. The
cosine must be at least .8 and the change is bounded by a factor of 4096 in
either direction and the existing trim rails. A published endpoint never
supplies a seed. Each new step resets eligibility. Lowering is blocked by the
existing collapse statistic, including its minimum-gap safeguard. The normal
band's upward branch also respects the existing in-solve climbing budget.
A downward band proposal is also vetoed when its scalar barrier-force response
predicts that the existing collapse-severity proxy would cross the retained
lower threshold. With q(g) the positive scalar barrier-force magnitude, a
proposed factor f is safe in that scalar proxy only if
f >= q(sqrt(severity)/dhat) / q(sqrt(trim_lower)). This guards against immediately undoing an emergency bump;
it is a heuristic and does not certify the next nonlinear iterate. The original
collapse feedback itself is unchanged. A snapshot saves/restores eligibility,
band cadence and the last decision.

The new keys under `/solver/contact/semi_implicit/` are `band_statistic`,
`initial_trim_estimate`, `force_band_lower`, `force_band_upper`,
`force_band_hysteresis`, `force_band_max_factor`, `force_band_interval`,
`initial_trim_max_factor`, and `initial_trim_cosine`. Invalid values are named
errors. The manifest identifies the active experiment and all parameters;
the trim-predictor stream records accepted/rejected estimates, gap, source,
before/after trim and the collapse guard.

## Method and evidence

Evidence: `outputs/ef-02-03/20260925T184403Z/` in the parent workspace. Isolated
worktrees start at PolyFEM `6a59cb387`, IPC `75600955`, PolySolve `448f1b8e`.
The saved configure command retains Python, tests and the production solver
backends. Every measured binary is copied into `bin/` with hashes; source
snapshots and build logs are retained. Runs are strictly sequential and do not
overlap builds. R4 and BB use all threads; the other scenes use one. R4 runs
have a 2700-second cap. The matrix includes two independent one-step and two
five-step R4 runs per mode, R1 (3), BBT (20), IT (200), BB (1), and the five
public smokes (4 steps each). R3 is a separate 40-step diagnostic, outside
acceptance. Power and sleep records qualify wall-time comparisons.

The immutable final candidate is `bin/PolyFEM_bin-v3`, SHA-256
`4d856c4dc6deb9577d46743b9847c5df2f82dbb3f74cc4bdba8907197c84daf3`.
Production is `bin/PolyFEM_bin-production-6a59cb387`, SHA-256
`550806acc05792091889b53a5cd7d7b4ec62d5484265923d9aa5ae87940b622e`.
`bin/v3-hashes.json` includes the unit binary and the source/spec/test snapshot;
`source-verification.json` verifies that the final sources match that snapshot.

`tools/ef02/run.py` calls the EF-01 driver with the explicit workspace;
`sequence.py` records commands before execution; `reduce.py` adds target-band
occupancy, expanded-band occupancy, trim moves and direction reversals;
`smoke_identity.py` compares every exported VTU byte on all five option-off
public smokes. Historical EF-01/EF-04b evidence remains separate from current
production comparisons. Failed runs and exploratory candidates are retained.
`compare.py` pairs every current candidate and production repeat, checks the
number of solution fields compared against the requested step count, and
reports balance changes per step. Cost acceptance conservatively compares
the largest candidate count with the smallest production count. Both the
production and candidate repeat spreads are reported. A missing reference
or a one-run baseline cannot establish a Newton repeat envelope.

Accuracy uses relative L2 differences of the exported `solution`, with
EF-01's tight pinned reference where available and fresh production pairs.
These are solution differences, not a proof of physical accuracy. RB-09's
reaction error bound is gap-dependent and benchmark-specific: the reported
normalized active-contact gaps do not supply a reaction sensitivity for the
arbitrary self-contact scenes. The force-residual `physical_balance_pass`
flag is assessed separately, without converting an existing false flag into
a new failure or treating a count of true flags as step-by-step equality.

## Results

### Cost on the current binaries

All 26 acceptance runs exited zero, with all 512 requested steps accepted.
Counts include every accepted Newton iteration across all attempts. A scene
cost is the total over its requested steps; individual later steps can cost
more. The same final candidate was used throughout; v1/v2 are excluded.

| Scene, steps | Production iterations | Candidate iterations | Production seconds | Candidate seconds |
|---|---:|---:|---:|---:|
| BB, 1 | 448 | 115 | 884.07 | 274.65 |
| BBT, 20 | 187 | 122 | 32.29 | 24.9 |
| IT, 200 | 5700 | 3400 | 485.25 | 294.13 |
| R1, 3 | 374 | 177 | 52.15 | 25.33 |
| R4, 1 | 657 / 703 | 99 / 107 | 1445.4 / 1536.1 | 257.02 / 269.98 |
| R4, 5 | 1050 / 1110 | 485 / 392 | 2305.26 / 2481.86 | 1166.81 / 933.63 |
| smoke-adaptive, 4 | 37 | 37 | 6.29 | 6.2 |
| smoke-alhess, 4 | 31 | 31 | 4.46 | 4.51 |
| smoke-friction, 4 | 62 | 62 | 4.73 | 4.51 |
| smoke-qs, 4 | 31 | 31 | 4.65 | 4.5 |
| smoke-tr, 4 | 26 | 26 | 4.63 | 4.57 |

R4 step 1 is 5.4–6.0 times faster by wall time in this matrix and costs
84–86% fewer iterations. Across five steps the reduction is 54–65% in
iterations. Production first leaves trim 1 after 314/371 accepted observations
in the one-step runs and 409/381 in the five-step runs; every candidate does
so at the first accepted observation and has already reached 2^-12. The first
seed has cosine .9502 and its factor bound selects the top of EF-01's plateau.

The five-step counts are production `[793,55,68,68,66]` /
`[782,67,57,96,108]`, candidate `[120,86,55,100,124]` /
`[115,68,43,75,91]`. The saving is dominated by step 1. Later steps total
365/277 for the candidate versus 257/328 for production: later-step cost
does not uniformly improve. The early seed is useful, but it is not a
general optimum over a full trajectory.

### Occupancy, oscillation and attempts

Occupancy is the fraction of valid accepted-iteration observations in
[.35,.50]; it excludes refreshes and rejected/invalid samples. Expanded-band
counts, every trim move by source, and per-step records are in
`ef02-metrics.json`. Reversals reset at step boundaries. `n/max` counts
solver attempts and their longest accepted-iteration count; the retained
soft budget can produce a 101-iteration attempt. None exceeds the AL cap.

| Scene, steps | Production occupancy | Candidate occupancy | Reversals P / C | Attempts n/max P / C |
|---|---|---|---|---|
| BB, 1 | 21/443 | 9/113 | 0 / 2 | 5/101 / 2/101 |
| BBT, 20 | 0/161 | 55/96 | 1 / 1 | 20/19 / 20/14 |
| IT, 200 | 443/3559 | 384/2094 | 22 / 60 | 213/101 / 201/101 |
| R1, 3 | 0/360 | 1/166 | 0 / 0 | 9/83 / 5/84 |
| R4, 1 | 11/646; 23/690 | 60/97; 64/104 | [3, 1] / [0, 0] | [11/101, 13/101] / [2/98, 3/92] |
| R4, 5 | 10/1031; 69/1089 | 331/473; 229/379 | [3, 2] / [2, 2] | [19/101, 21/101] / [12/101, 13/101] |
| smoke-adaptive, 4 | n/a | n/a | 0 / 0 | 4/19 / 4/19 |
| smoke-alhess, 4 | 1/27 | 1/27 | 0 / 0 | 4/11 / 4/11 |
| smoke-friction, 4 | 2/54 | 2/54 | 0 / 0 | 8/12 / 8/12 |
| smoke-qs, 4 | 1/27 | 1/27 | 0 / 0 | 4/11 / 4/11 |
| smoke-tr, 4 | 1/22 | 1/22 | 0 / 0 | 4/9 / 4/9 |

R4 single-step moves are one downward seed (both runs), plus one
iteration-band decrease in repeat 2. Five-step candidate repeat 1 has
4 down/1 up seed moves, 8 down/1 up iteration moves and 2 endpoint decreases;
repeat 2 has 3 down/2 up seed moves, 6 iteration decreases and 2 endpoint
decreases. These are source counts across the whole run, including any
unchanged emergency controller under its original event. The predictor
decision object on another event is context, not a second decision.

R4 occupancy improves strongly, but the target is not universal: R1 remains
at 1/166 in band, BB at 9/113, and IT has 60 direction reversals versus 22
in production. The retained collapse guard can veto softening before the
force-weighted target is reached. This work does not change the collapse
law to make the occupancy statistic look better.

### Accuracy and balance: adoption gates are not met

Every cross comparison contains the full requested number of solution
fields. R4 five-step differences against each fresh production repeat are:

| Step | Candidate vs production range, % | Production repeat, % | Candidate repeat, % |
|---|---:|---:|---:|
| 1 | 5.42–11.49 | 6.00 | 9.72 |
| 2 | 4.05–8.71 | 3.06 | 7.98 |
| 3 | 4.86–6.32 | 2.08 | 6.83 |
| 4 | 4.24–5.10 | 1.56 | 5.80 |
| 5 | 3.36–3.95 | 1.40 | 4.50 |

The candidate exceeds the measured production repeat spread, especially
in later steps. The independent one-step matrix gives 4.91–7.07% cross-mode
differences versus a 5.67% production repeat spread (candidate 5.69%). The
historical 3.6–8.7% spread does not justify treating all later-step differences
as a pass. Against EF-01's tight reference, the five-step runs' **step 1 only**
is 12.88/8.49% for the candidate versus 10.23/11.23% for production. That
reference contains only one step; it is not five-step accuracy evidence.

R1 stays within 7.92e-4 of its tight reference over all three steps
(production at most 7.75e-4). BBT's first step is essentially unchanged at
1.285% reference error versus 1.287%; later errors generally decrease,
ending at 6.73e-4 versus 2.61e-3. These small or improved reference errors
do not establish strict equality to a Newton repeat envelope.

BB differs from fresh production by 1.387%, above EF-04b's historical
0.4% production repeat difference. The CCD dependency changed since that
baseline, and no fresh BB repeat is available, so this gate is unverified,
not passed. IT agrees before contact and diverges afterward, with a maximum
92.21% relative difference over 200 steps. EF-04b already found production
IT trajectories non-reproducible after first contact; without a fresh repeat
envelope this does not isolate a controller accuracy defect, but also
cannot establish acceptance. Every enabled smoke solution is identical
to its production counterpart.

`physical_balance_pass` is unchanged on R1 (all false), R4 (all false), BB
(false), and the five smokes (friction all false, the other four all true).
BBT changes from 18 true/2 false to all 20 true. IT changes from 95 true/
105 false to 110 true/90 false: **33 false-to-true and 18 true-to-false**
changes, 51 steps in total. Thus the literal unchanged-flag requirement
fails even though the total number of true flags increases.

Active-contact normalized gaps remain positive and below the barrier support
in the endpoint observations. R1's endpoint mean gap/dhat falls from
.938–.987 to .711–.733. R4 candidate five-step endpoint force-weighted gaps
are .418–.546, while its unweighted mean gaps are .735–.948. RB-09 relates
reaction error to the physical mean gap times a benchmark-specific
sensitivity, not to force-weighted occupancy. The unchanged public smoke
fields preserve the previously characterized smoke gap error; these
self-contact measurements do not certify a new universal reaction-error
bound. This part of the broad accuracy gate remains limited.

### Regression checks and exclusions

| Acceptance check | Result |
|---|---|
| No new solver failure in the requested matrix | Pass: all 26 runs /512 steps complete; diagnostics present and valid |
| Total iterations <= production on every scene | Pass, including the most expensive candidate vs cheapest production repeat |
| Accuracy within Newton repeats and RB-09 gap error | Not passed: R4 repeat envelope exceeded; BB/IT and general gap-to-reaction accuracy not established |
| Physical balance flags unchanged | Fail: BBT 2 changes, IT 51 changes including 18 true-to-false |
| Band occupancy and oscillations reported | Done above; full expanded occupancy and move sources retained |
| Five option-off smokes byte-identical | Pass: all 25 exported VTU files, SHA-256 equality, both exits zero |
| RB-02 coefficient probe | Pass: 270/270, `bin/rb02-v3/` |
| Focused unit tests | Pass: 23 cases, 2692 assertions, including statistic, two-sided step, seed/collapse guard and rollback |
| Full unit suite | 392/394 cases, 5,177,902/5,177,904 assertions; only the known gcp-contact/cube-on-floor and multi-material/stretch-cubes golden failures |
| Public run-smoke.sh | Five scenes exit zero with the final saved binary |
| HDA integration | Pass: all 13 scripts, Houdini 22.0.429; `hda-recheck-ledger.jsonl` |

The first HDA attempt used Houdini 21 instead of the installed Houdini 22
test environment and lacked gmsh and the isolated smoke fixture. Those
errors are retained. The rerun uses Houdini 22.0.429, its existing gmsh,
copied test modules (so imported helpers keep the isolated root), linked
assets, and smoke output produced by the final binary. No HDA source or
default was changed.

R3, outside acceptance, accepts steps 1–38 and reaches the 1800-second cap
on step 39 (467 accepted iterations there). It does not demonstrate a fix
for the known step-39 problem; the timeout prevents asserting the same
eventual named solver failure. No automatic retry was enabled.

`power-audit.json` checks all 26 measured run windows: AC power at every
start, no overlapping solver windows, and no sleep entry within a run.
The earlier clamshell-sleep entries occur before final-candidate measurements.
Builds precede the measured runs; regression tests follow the timing matrix.
Timing is supporting evidence; the acceptance comparison uses iterations.

### Exploratory candidates retained

Exploratory v1 recovered R4 step 1 in 128 iterations (328 s) by seeding from
1 to 2^-12 at cosine .95. Its cadence reset each step and it skipped endpoint
band updates, leaving BBT's short steps largely uncontrolled (171 iterations).
V2 carried cadence across steps and applied the band at endpoint refreshes:
R1 187 iterations, BBT 120, IT 3264, BB 95, all solver exits zero. However,
it immediately softened the trim after emergency strengthening on public
smokes: qs/alhess/tr each 59 iterations versus production 31/31/26; friction
108 versus 62. This candidate failed the per-scene cost requirement and its
large R4 matrix was not started. Friction's four false balance flags were
already present in the direct production comparison. The new scalar-response
guard addresses this observed controller conflict. Both exploratory versions
and their binaries, inputs, logs and results are retained.

## Conclusion and default recommendation

The early off-equilibrium seed removes R4's expensive walk from trim 1, and
the new band lowers cost across the measured non-smoke scenes. The final
collapse guard also removes v2's smoke cost regression without changing
production behavior. This establishes an efficiency result, not full
acceptance: the strict accuracy and unchanged-balance gates are unmet.

**Recommendation: retain the production `rms` controller and disable the
initial estimate by default.** Keep this mode explicitly experimental and
opt-in. Further adoption work needs a declared accuracy/physical contract
for the trajectory-sensitive scenes, repeat evidence for held-out dynamics,
and a controller design that respects that contract; do not relax the gates
because it is faster. No default or Houdini exposure decision is made here.

Reproduction tools: `tools/ef02/README.md`. Evidence includes
`ef02-metrics.json`, `ef02-comparisons.json`, `ef04-tables.md`,
`ef01-tables.md`, `final-summary.json`, the command ledgers, all input/binary
hashes and raw logs under the evidence root. Source/spec/test snapshots match
the measured final binary. The workspace's shared source and build were not
used as a publication staging area.
