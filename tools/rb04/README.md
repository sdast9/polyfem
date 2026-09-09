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
