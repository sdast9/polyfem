# RB-06 failed-attempt rollback: end-to-end runner

Tool behind [docs/rb-06-validation.md](../../docs/rb-06-validation.md).
Public inputs only; nothing here runs Teseo or a private scene.

```sh
python3 tools/rb06/run_injection.py --output /absolute/fresh/dir [--binary build/PolyFEM_bin]
```

Runs the public `transient-semi` and `quasistatic-semi-friction` fixtures
single-threaded with the RB-04 diagnostics and the RB-12 run manifest on:
once as controls, once per failure injection (`solver/advanced/failure_injection`,
a test hook that throws a deterministic failure at a chosen point of step 2 —
after the AL stage, at an accepted Newton iterate of the reduced solve as a
named failure and as a `runtime_error`, after the friction-lag update, after a
stall retune under a restart budget, before and after the step's publication),
and the RB-04 real-failure fixture (`quasistatic-semi` with a restart budget
that exhausts itself: "Final reduced solve did not converge", not injected).

For every failed attempt it checks: exit status 1; the manifest's completion
`failed` and its last step `failed_attempt` with `rollback.performed` and
`rollback.verified` true; no frame, PVD entry or energy row of the failed
step; the RB-04 failure record announcing the rollback with the restored
coordinates equal to the previous accepted endpoint; the coefficient-event
stream ending the step with the `rollback` event; and every frame written
before the failure byte-identical to the control's. For the publication
failures it checks that the solve was accepted, that the frames/PVD/energy
rows stop where the failure struck, and that the published frames equal the
control's. `results.json` keeps every command, exit status, input hash and
check. The runner never overwrites its output directory.

The in-process counterpart — the recovery test that re-solves a rolled-back
step from the restored state and compares it with a fresh control — is
`unit_tests "[rollback]"` (`tests/test_step_rollback.cpp`).
