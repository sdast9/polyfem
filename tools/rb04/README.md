# RB-04 endpoint diagnostics

Enable `output.physical_diagnostics: true` to append version 2 JSON records to
`physical-diagnostics.jsonl` in the configured output directory (version 1
fields keep their meaning; see the contract's version 2 section). The default is
false. Records are observational: a returned solve is `accepted` even if finite
friction lagging did not converge. They impose no new acceptance criterion.

The implementation and missing accounting stages are specified in
[the contract](../../docs/rb-04-contract.md) and
[the validation record](../../docs/rb-04-validation.md).

From the PolyFEM repository, after building `PolyFEM_bin` and `unit_tests`:

```sh
python3 tools/rb04/run_endpoints.py --output /absolute/fresh/evidence
```

Requires NumPy. This runs three public four-step fixtures with diagnostics off
and on, then a paired deliberate reduced-solve exhaustion. It checks endpoint
agreement, independent P1 Neo-Hookean energy/det(F)/quasistatic reaction
reconstruction, exact P1 mass integration using ImplicitEuler displacement
velocities, record completeness within the fixture, and failure labeling.
Every run retains its command, input hashes, exit, log and output in a new folder.
The expected failure currently terminates the CLI with SIGABRT from its uncaught
solver exception; a flushed failure record must exist and step 1 must not be
published. The runner does not change that CLI behavior.

Since 2026-09-12 the VTU `velocity`/`acceleration` fields of the nonlinear path
are the kinematics of the saved endpoint (before that they were the previous
step's history head). The runner checks them against `(u_n-u_{n-1})/dt` for
the public ImplicitEuler fixtures and still reconstructs the kinetic-energy
reference from the displacement history, not from the VTU field.

Focused snapshot test: `[physical_diagnostics]`. It checks private coefficient
memoization, unchanged production energy/gradient/Hessian, finite differences,
subsequent production continuation and empty-contact unavailable fields.

No Teseo/private scene or full PF-08 sweep is run. No integrated physical work
balance, friction dissipation or accuracy threshold is certified by these checks.

The same opt-in now writes outer coefficient operations to
`coefficient-events.jsonl`. After `run_endpoints.py`, validate arithmetic,
contiguous identity, failure-event retention, phase coverage and same-coordinate
agreement with saved endpoint contact energies/forces:

```sh
python3 tools/rb04/check_coefficient_events.py /absolute/fresh/evidence --output /absolute/new-event-summary.json
```

Focused real-form regression: `[coefficient_events]`. Event sums describe
algorithmic changes at solver iterates, not physical trajectory work. See the
contract for initial-state, external-setter, time-weight and feature-switch limits.

# Trim-band candidate pilot

The ongoing [research log](../../docs/rb-04-research-log.md) preserves hypotheses,
authorization, measurements and next steps. Run the five-band/three-fixture pilot
with a NumPy-enabled Python interpreter and an already verified solver build:

```sh
python3 tools/rb04/run_trim_bands.py --output /absolute/fresh/evidence
python3 tools/rb04/summarize_trim_bands.py /absolute/fresh/evidence --output /absolute/new-summary.json
```

Each process has a 120-second experimental timeout; partial results remain.
Summary reconstruction checks reuse the endpoint stage tolerances. Its partial
work quantities are explicitly not a complete energy balance. This tests the
existing controller, not an implemented replacement contact model.

# Load/time refinement and work estimates

```sh
python3 tools/rb04/run_refinement.py --scene quasistatic-semi --output /absolute/fresh/study
python3 tools/rb04/summarize_refinement.py /absolute/fresh/study --output /absolute/new-summary.json
```

Run the corresponding transient/friction scene names separately. Optional
`--lower .1 .8 --dt .0625` selects an explicit subset for a retained repeat.
The default is nine runs per fixture, each with a 120-second observation budget.
The summarizer reprocesses saved outputs without rerunning or replacing solver
failures; `work_reference.py` independently integrates transient P1 mass work.
See [work conventions](../../docs/rb-04-work-convention.md) before interpreting
right/trapezoidal component costs, friction state, or incomplete energy budgets.

# Physical-state pair analysis

```sh
python3 tools/rb04/analyze_state_pairs.py /absolute/run-directory --output /absolute/new-summary.json
```

Supply directories containing `params.json` and `output/` from the new paired
diagnostics build; multiple runs can be supplied. The analysis records both sides
of the final friction lag update and the declared common-coefficient contact
energy decomposition. It does not identify that decomposition with a gradient
path integral across RB-02's unresolved feature discontinuities.


### Fixed-coefficient candidate fixture

`python3 tools/rb04/run_candidate_probe.py --build build --output /new/evidence`
compiles/runs `candidate_probe.cpp` against the existing unit-test link libraries.
The output directory must be new. No scenes or production settings are changed.
It reproduces the RB-02 EV/VV transition, compares Fixed k=70, checks derivatives
and units, and integrates the gradient across the boundary. See
[candidate comparison](../../docs/rb-04-candidate-comparison.md) for normalization,
results, interpretation and remaining scene work. Committed compact data are
`candidate-results-20260909.json`; raw outputs retain both successful probe runs.


### Discrete trajectory budget

`trajectory_budget.py --evidence /physical-state-pairs --output /new/evidence`
reads the committed paired-result index and saved solver artifacts. It separately
reports solved-lag friction, elastic/contact right-work remainders, parameter
energy and implicit-Euler terms. An independent P1 stress-path quadrature checks
elastic energy changes. `python3 tools/rb04/test_trajectory_budget.py` runs three
analytical/negative controls. Use Homebrew Python with NumPy. The output directory
must be new. No solver runs. Failed zero-endpoint runs have unavailable totals.
See [work convention](../../docs/rb-04-work-convention.md) for fixture restrictions
and the unresolved contact path contribution. Compact results are in
`trajectory-budget-results-20260909.json`.


### Actual frozen contact paths

Add `--contact-path` to `run_endpoints.py` to exercise the separately opt-in
path observer in each diagnostic-on run. It performs up to 1025+24*1024 private
snapshot evaluations per accepted endpoint; use bounded fixtures. Analyze with
`analyze_contact_path.py /run1 /run2 --output /summary.json`. It independently
reconstructs trapezoids and adds Simpson estimates; full raw signatures stay in
solver outputs. `run_candidate_probe.py --source tools/rb04/contact_path_probe.cpp
--build build --output /new/evidence` checks the known jump and Fixed control.
That source includes `candidate_probe.cpp`; retain both when copying the probe.
Compact results: `contact-path-results-20260909.json`. See the validation record
for the first-contact .00131 quadrature discrepancy and non-exhaustive event limits.


### Solver-attempt stream (record version 2)

The same opt-in also writes `solver-attempts.jsonl`: one row per PolySolve
minimize start, accepted Newton update (trial sweep norms, forms' step bound,
line-search validity trials, accepted fraction) and rejected proposal. The
endpoint record carries `attempt_summary`, `proposed_displacement`, the retained
`contact.candidate_count` statistics, the right-endpoint work increments
(`support_work_increment`, `external_work_increment`,
`frictional_dissipation_increment`, `retuning_energy_change`, cumulative sums)
and, for a failed attempt, `last_internal_iterate`. `run_endpoints.py` checks
all of these; the stream alone can be checked with

```sh
python3 tools/rb04/check_solver_attempts.py /absolute/fresh/evidence --output /absolute/new-attempt-summary.json
```

Compact results of the 2026-09-12 validation: `results-20260912.json`.
`physical_balance_pass` remains unavailable by decision (no threshold selected).
