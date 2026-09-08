# PF-08 physical and numerical validation — 2026-09-07

**Validation stage implemented; physical certification remains open.** No solver,
contact-floor, friction-lagging, recovery, or tolerance defaults were changed.
This stage adds reproducible measurements, including negative results, after
PF-01 and PF-03–PF-07. PF-02's production-model decision is still open. Teseo
was not run. Scene completion is not treated as physical convergence.

## Manufactured fixed/moving and coupled-contact experiment

[physical_probe.cpp](../tools/pf08/physical_probe.cpp) links the actual PolyFEM
contact forms, BCLagrangianForm, NLProblem and ALSolver. A free point interacts
with one horizontal edge or two perpendicular edges sharing a prescribed corner.
A translation-invariant coupled spring connects the point and corner. This is a
small discrete model, **not a continuum FEM mesh**. Moving-obstacle cases move
the prescribed corner/edges while changing the point's relative target.

The system-Hessian provider is the constant positive matrix `100 S/L² I`.
Coefficients are initialized once; global trim is explicitly pinned to one to
make the energy/work experiment well-defined. Floor remains `1e-4`, but measured
gaps are at least `0.2 dhat`: this is **inactive-floor** evidence. The filter uses
the production affine full/reduced coordinate mapping. Real collision checks
remain active; inversion is inapplicable without volume elements. This does not
validate the production trim controller, an active floor, or hard-contact KKT.

There are 108 configurations: length L in {0.001,1,1000}, objective scale S in
{0.01,1,100}, one/two contacts, fixed/moving obstacles, and 2/4/8 load increments.
Geometry, targets, dhat and CCD tolerance scale by L; spring/system stiffness
and AL penalties by S/L²; gradient tolerance by S/L. The experiment explicitly
uses Euclidean norms, with tolerance `1e-9 S/L`, zero relative/first-gradient and
step tolerances, and a maximum of 100 Newton iterations. It does not change the
production solver's configured convergence contract.

Loads are manufactured using a five-point finite difference of the **energy**
at a known target, independently of Newton's analytic gradient. Acceptance checks
on returned solutions are BC error/L < 1e-10, point error/L < 1e-7, free residual
and reaction-plus-load imbalance divided by S/L < 1e-6, and positive gap. Reactions
are reconstructed from full-coordinate derivatives on prescribed coordinates;
the free residual is evaluated again at the returned endpoint.

- **85/108 complete configurations passed; 23 stopped with a hard line-search
  failure.** They are retained as failures, not removed from the sweep.
- Across 487 returned solutions, maximum BC error is zero; normalized free
  residual and reaction imbalance are below 9.24e-10; point error below 2.81e-11.
- Among completed endpoints, normalized-energy spread across converted units,
  objective scales and load increments is at most 2.04e-9 within each geometry/
  motion group. This conditional agreement is not an all-configurations pass.
- The manufactured load's finite-difference discrepancy at the nominal target
  reaches 2.09e-8 in force units S/L, exceeding the strict solve tolerance in some
  cases. The nominal target is therefore only an approximate stationary point.
  Failures in this near-machine-precision experiment do not alone establish a
  production defect. No tolerance was relaxed to make the sweep green.

Early local fixture-development runs used an implicit L2 norm and an incorrect
filter coordinate adapter. They are excluded from these measurements; the
published source and `probe-validated` provenance identify the corrected run.

## Conservative work and lagged friction

On a prescribed relative-motion segment, Simpson integration of the actual
spring/contact gradient is compared with endpoint energy change. The 4/8/16/32
interval refinement reaches normalized discrepancy below 1.33e-9 at 32 intervals.
This is a frozen-coefficient conservative-work check, not work accounting across
production retunes, floor switches or timestep integration.

The real FrictionForm is lagged at a fixed normal-contact configuration with
mu=0.3, smoothing scale `0.001 L` and a **finite lag budget of two**. Five positive,
negative and zero slips cover the smoothed and sliding regimes. The checks require
nonnegative dissipation `g_friction dot slip`, balanced point/obstacle forces,
finite-difference agreement with its dissipation potential, and nonzero sliding
friction with magnitude `mu * normal_force` on the horizontal edge. Maximum
normalized sliding-magnitude error is 6.59e-14; the centered finite-difference
error is 8.16e-5 against 1e-3 (its largest error is in the regularization region).

This checks the frozen lag's constitutive force and dissipated work for prescribed
slip. It does not solve a friction-coupled moving-obstacle trajectory or establish
that two lagging iterations reach a fixed point. The production finite-lagging
policy is preserved.

## Public FEM fixtures and independent reconstruction

All five existing semi-implicit/adaptive smokes retain their original inputs and
four-step schedules and complete. Additional resolution experiments retain the
same end time and prescribed motion law and explicitly vary dt or mesh refinement.
All input copies, output paths, commands and input hashes are isolated under
`outputs/pf-08/` in the parent workspace.

[measure_fem.py](../tools/pf08/measure_fem.py) reconstructs P1 tetrahedral F and
Neo-Hookean energy/elastic forces from saved **rest coordinates and displacement**.
It merges duplicated output vertices by exact coordinates, separates rigid
triangles from volume cells, integrates `P grad(N)` per tetrahedron and sums
prescribed-top reactions. The volume must sum to one and reconstructed F must
match saved F columns to 1e-10; maximum observed difference is 2.67e-15. This is
independent of solver stopping logs and saved stress values. Two reconstruction
checks pass: rigid rotation produces zero energy/reaction, and the assembled top
reaction agrees with a finite difference of energy under prescribed stretch.
These tests use a unit cube with duplicated per-element output vertices.

Across 83 saved frames, prescribed-top displacement error is zero, all reconstructed
volume determinants are positive (minimum 0.83936157), and maximum quasistatic
elastic interior-force norm divided by E is 1.07e-13. Interior here excludes both
the prescribed top and contact bottom. This is **not the full free-DOF residual**:
contact-node forces, inertia and friction must be included for that claim.
Transient elastic residuals are deliberately not labeled equilibrium residuals.
Saved positive determinants do not certify every line-search trial or independent
whole-scene intersection freedom.

| Fixture | Steps saved / intended | Final elastic energy | Final top z reaction |
| --- | ---: | ---: | ---: |
| Quasistatic semi, dt=.25 | 4/4 | 380017.684 | -3640034.645 |
| Quasistatic adaptive, dt=.25 | 4/4 | 377530.493 | -3625688.917 |
| Quasistatic semi + friction, dt=.25 | 4/4 | 390255.498 | -3923675.572 |
| Quasistatic semi, dt=.125 | 8/8 | 380067.253 | -3640327.728 |
| Quasistatic semi, dt=.0625, first run | 0/16 | unavailable | unavailable |
| Quasistatic semi, dt=.0625, repeat | 16/16 | 380091.824 | -3640473.181 |
| Quasistatic semi, one uniform refinement | 4/4 | 357164.816 | -3403740.675 |
| Transient semi, dt=.25 | 4/4 | 380017.685 | -3640048.030 |
| Transient semi, dt=.125 | 8/8 | 380067.260 | -3640344.623 |
| Transient semi, dt=.0625 | 16/16 | 380091.851 | -3640491.244 |

The Hessian-initializer smoke also completes and matches the base quasistatic
energy/reaction at the reported precision. Values use the fixtures' input units.

The mesh changes from 384 to 3072 tetrahedra. Energy changes by about 6.01% and
reaction magnitude by 6.49%, so even a 1% engineering comparison screen is not met.
Two meshes cannot establish an asymptotic convergence rate. Successful load
refinements change final energy by 0.0130% and then 0.00647%; this endpoint
agreement must be read alongside the failed first fine-load run.

Top work is the trapezoidal integral of independently reconstructed reaction
against prescribed displacement. For the quasistatic semi dt=.25/.125/.0625
successful runs, work minus elastic-energy change is 7894.607 / 2083.172 /
392.000. For the friction smoke it is 18137.680. These are **incomplete balance
measurements**, not energy errors: barrier energy, coefficient/trim changes,
frictional dissipation and (for transient cases) kinetic energy are not all
included. Their reduction under load refinement does not close that accounting.

## Active-floor negative control

The existing PF-02 real-form probe was rerun against this baseline in four fresh
processes (floor on/off, pre-existing/new contact). Its three defects remain:
barrier energy disappears below the threshold, restoring prescribed edge motion
breaks the full-coordinate projection's gap guarantee, and stiffness refresh
restores energy at the same below-floor position. At gap 5e-5, refresh changes
energy from zero to 1980.697501 with the floor on. The floor-disabled scalar
force-balance comparator still brackets equilibrium; the current floor does not.
This is current reproduced negative evidence, not a production-model fix.

## Exhausted recovery and robustness limits

The first dt=.0625 quasistatic run exhausted 20 reduced-solve recovery restarts
before saving step 1. It reported interruption, gradient 1.41119e-4 against
2.27951e-6, and threw `Final reduced solve did not converge` (process SIGABRT).
A second isolated run with the same numerical inputs completed all 16 steps.
The cause is unresolved; concurrency/timing is not established as causal.
Both results are preserved. No partial run is counted as a full pass.

Existing focused regressions deliberately exhaust 0/2 soft restarts, cover hard
line-search failures, and verify that an unsuccessful reduced solve cannot
report success or replace the caller's solution. They pass on this baseline.
The new scene failure supplies additional real-binary recovery evidence without
changing budgets or accepting an interrupted solve.

## Validation and remaining work

Started from clean tracked sources at PolyFEM `4f1a04052`. Build and effective
pins: IPC `9da3094`, PolySolve `4d372fa8`; companion sources are unchanged.
`PolyFEM_bin` and `unit_tests` rebuilt; focused suite: **22 cases / 1,161 assertions
passed**. The default full suite completed with **266/267 cases and
5,152,778/5,152,779 assertions passing** (exit 42). Its only failed case is
`contact_2d`, solely for the known `gcp-contact/cube-on-floor/run.json` golden
comparison: for example err_lp relative difference 0.000546995 exceeds its
1e-5 margin. No golden values or tolerances were changed. The Houdini PolyFEM
2.0 real-binary end-to-end test and JSON round-trip both passed. C++ formatting,
Python syntax, the two reconstruction checks and `git diff --check` passed.
The full suite ran from the isolated parent-workspace `outputs/pf-08/` directory
using the absolute path to `build/tests/unit_tests`, with no selection filter.

PF-08 is **not a physical-accuracy certification**. Remaining acceptance work:
full scene free/contact residual and reaction balance with the actual retained
coefficients; complete energy/work/dissipation accounting; mesh convergence;
robust load refinement; independent material/inertia contrasts beyond converted
objective scales; friction-coupled moving/coupled obstacles; and active-floor
behavior after the PF-02/PF-09 model decision. The known active-floor defects are
not fixed by this validation stage. No private scene or Teseo run is claimed.

## Reproduction

From the PolyFEM checkout, using the configured Unix Makefiles build:

```sh
cmake --build build --target PolyFEM_bin unit_tests -j 6
build/tests/unit_tests '[direction_filter],[bc_scale],[bc_metric],[al_solver],semi-implicit barrier contact form derivatives,semi-implicit friction form derivatives'
python3 tools/pf08/run_probe.py --build build --output /absolute/evidence/probe-validated
python3 tools/pf08/run_scenes.py --binary build/PolyFEM_bin --output /absolute/evidence/scenes
python3 tools/pf02/summarize_scenes.py /absolute/evidence/scenes
python3 tools/pf08/test_measure_fem.py
python3 tools/pf08/measure_fem.py /absolute/evidence/scenes
python3 tools/pf02/run_probe.py --build build --output /absolute/evidence/active-floor
python3 tools/pf08/summarize.py /absolute/evidence
```

The probe exits nonzero if any configuration fails. Scene execution records each
exit, including failures, continues the matrix, and exits nonzero if any run
failed; read `scene-results.json` for complete/partial results. Use a fresh output path.
The checked-in [measurements](../tools/pf08/results-20260907.json) preserve the
baseline's negative results; they are not golden values to regenerate or enforce.
