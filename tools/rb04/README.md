# RB-04 endpoint diagnostics

Enable `output.physical_diagnostics: true` to append version 1 JSON records to
`physical-diagnostics.jsonl` in the configured output directory. The default is
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

The existing VTU `velocity` field on the nonlinear path contains `v_prev()`
before history advancement. It is **not** used as a current-endpoint kinetic
energy oracle. The reference uses `(u_n-u_{n-1})/dt` for the public ImplicitEuler
fixture. Higher-order time integrators require their own history reference.

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
