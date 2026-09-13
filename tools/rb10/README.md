# RB-10 friction coupling and dissipation: probe, fixtures and analysis

Tools behind [docs/rb-10-validation.md](../../docs/rb-10-validation.md).
Public inputs only (`scenes/semi-implicit/cube.mesh`, `slab.obj`, the
`quasistatic-semi-friction.json` smoke); nothing here runs Teseo or a private
scene, and the public scene files are copied, never edited.

```sh
# Stage 1: unit-level probe of the lagged friction path (compiled with the
# unit_tests flags/link line, i.e. the effective toolkit / PolySolve)
python3 tools/rb10/run_probe.py --build build --output /absolute/fresh/dir/stage1-probe

# Stage 2: fixture matrix at the default friction policy (14 runs, ~7 s each, 1 thread)
python3 tools/rb10/run_scenes.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/stage2-fixtures --stage 2
# Stage 3: predeclared budget {2,4,8} and epsv {1e-2,1e-1,1} sweeps on slide_plus / reversal (24 runs)
python3 tools/rb10/run_scenes.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/stage3-sensitivity --stage 3
# classic-adaptive observation (2 runs), and the friction_lag A/B incl. the public smoke (20 runs)
python3 tools/rb10/run_scenes.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/stage3-classic --stage classic
python3 tools/rb10/run_scenes.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/stage4-friction-lag-ab --stage ab

# Per-endpoint analysis of any run directory (writes endpoints.json next to the runs)
python3 tools/rb10/analyze_endpoints.py --runs /absolute/fresh/dir/stage2-fixtures
# Summary tables + the compact published JSON
python3 tools/rb10/summarize.py --evidence /absolute/fresh/dir --out tools/rb10/results-YYYYMMDD.json
```

`friction_probe.cpp` puts one 2D point above a floor edge (RB-02's probe
mesh) under a semi-implicit `BarrierContactForm` with a synthetic driving
Hessian and a `FrictionForm` lagged at the pressed configuration, and
measures: the constitutive response over a slip sweep (`μ N f1`, `g·v`,
point/edge balance, finite differences with the trim scale active); the
normal-force transfer through trim bumps, calibration, mid-solve refresh
(with and without force continuation), stall retune and the between-steps
refresh; the classic-adaptive rule (observation only); and the slip
convention with no integrator (static: total displacement), ImplicitEuler and
BDF2. The invariant parts are also the `[friction_lag]` Catch2 regression.

`run_scenes.py` writes isolated scenes on the public unit cube pressed onto
the slab (top face prescribed: press .05 by t = .2, then the fixture's
tangential program; dt .05, 20 steps; NeoHookean E 1e7, ν .45; d̂ 1e-3;
μ .3; semi-implicit barrier; RB-04 physical diagnostics on) — zero friction,
steady sliding ±x, reversal, separation/recontact, moving obstacle and a
half-height-wall corner — in quasistatic and transient (ImplicitEuler) form,
plus the sensitivity sweeps and the `semi_implicit/friction_lag` A/B.
`analyze_endpoints.py` reads the record: support force totals, barrier and
friction force totals per body and per obstacle interface, the sliding ratio
`|F_t|/(μ N)` with the solved-lag and the endpoint-rebuilt friction, the lag
error between them, the contacting nodes' slip against the prescribed
increment, the RB-04 work increments and both residuals of the final lag
update. Nothing is recomputed from the solver's internals; a missing quantity
is reported as unavailable.
