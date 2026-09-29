# R3 inflation — why it stops at step 39 (diagnosis, 2026-09-26)

Status: **diagnosed as a material/structural problem, not a trim-controller
problem.** Measurement only; no source, default or asset change. Recorded in
the repository on 2026-09-28 (the diagnosis itself was done on 2026-09-26).
Full report, scripts, runs and plots:
`outputs/r3-inflation/20260926-diagnosis/report.md` in the parent workspace,
which is being archived to the Pitt share (AGENTS.md, *Archived test
outputs*).

**Update 2026-09-29 — the stall is a structural pressure maximum followed by a
snap-through, not the local loss of ellipticity.** The controlled Ogden
comparison and the NeoHookean hold probes are in
[Controlled Ogden comparison and why the NeoHookean stops](#controlled-ogden-comparison-and-why-the-neohookean-stops-2026-09-29).
The Ogden run loses strong ellipticity earlier and far more widely than the
NeoHookean and still completes, so the ellipticity finding below stands as a
measurement but is not the cause of the stall.

Context: EF-01 found that R3 (`test_cases/inflation`, a pressurized cavity
inflated against an obstacle) fails at step 39 at every trim tried, including
on the published binary, and said it needed its own item
([ef-01-trim-survey.md](ef-01-trim-survey.md)); EF-02/03 hit the same wall
([ef-02-03-trim-controller.md](ef-02-03-trim-controller.md)). This record is
that item's diagnosis.

## Scene

`test_cases/inflation/input/params.json`: compressible NeoHookean, E = 1e6,
ν = 0.45, ρ = 1000; pressure boundary 20010101 = −100000·t; ImplicitEuler,
t_end 15 over 200 steps (dt 0.075). Geometry scale 0.001 (metres, SI).

## Findings

* **Time step is not the cause.** Production at dt 0.075 accepts step 38
  (t 2.85, 285 kPa) and stalls in step 39; at dt 0.0375 it accepts step 76
  (the same t 2.85) and stalls in step 77. Both runs stopped at the
  observation cap (timeouts, not named failures). The accepted paths agree to
  0.0035 % in displacement at t 2.85 (0.108 % maximum over all matched
  times).
* **The inflation is approaching a structural limit.** The closed cavity
  (1,836 triangles, verified closed and consistently oriented) has grown to
  8.47× its initial volume at t 2.85; the pressure–volume slope stays positive
  but falls from 23.6 to 6.2 kPa/mL over the last five steps (5.2 kPa/mL over
  the last half-step). A pressure maximum was not reached, so a limit point is
  indicated but not proven.
* **The material loses strong ellipticity locally just before the stall.**
  PolyFEM's NeoHookean (`NeoHookeanElasticity.cpp`) is the log-J form
  W = μ/2 (F:F − 3 − 2 ln J) + λ/2 (ln J)². Its rank-one curvature is
  μ + [μ + λ(1 − ln J)] (a·F⁻ᵀb)²; when μ + λ(1 − ln J) < 0 (large volume
  growth) the minimum over directions can go negative, violating the
  Legendre–Hadamard condition. On R3 it is positive at t 2.775 (≈ 0.30 MPa),
  negative in one tetrahedron at t 2.8125 (≈ −1.07 MPa) and in four at t 2.85
  in both time steps (worst ≈ −3.11 MPa; J = 3.37, principal stretches 7.45,
  1.48, 0.31), confirmed by converged central finite differences of the exact
  energy. The four elements (Gmsh tags 18790, 22952, 28285, 29652) sit on the
  pressure boundary and occupy 0.00017 % of the solid volume.
* **Contact and the solver's step limits make the stalled step expensive but
  are not the root cause.** Newton repeatedly meets non-descent directions and
  switches to projected Newton; the accepted displacement per iteration sits at
  the 50·d̂ trial cap in most iterations; the active contact count grows from
  7 to 40–127 during the unfinished solve.

## What it means for the efficiency work

R3's step-39 failure is not an EF item: no trim or controller change is
expected to carry the scene through a constitutive loss of ellipticity near a
structural inflation limit, and none should be tuned to do so. R3 stays
outside the EF acceptance matrices (as in EF-02/03 and EF-07).

## The completed Ogden run (located 2026-09-28)

The run the user completed is in `test_cases/inflation/output/`: steps 39–200
(dt 0.075, t 2.925–15.0 s) written on 2026-09-21 between 12:10 and 13:27:55;
its steps 0–38 and manifest were overwritten by the NeoHookean run of 13:56
that this diagnosis used. Its material was identified from the stored
deformation gradient and Cauchy stress: **IncompressibleOgden, c = [1000],
m = [13], k = 1e7** (median relative stress error 0.09–0.11 against 0.79–0.99
for the NeoHookean; the NeoHookean run's step 38 is the control, 0.15 against
1.32), the values saved in every Houdini scene of the case. The scene
`test_cases/inflation/kristin_sim.hipnc` still holds the run's full settings:
exported with the current asset, they match the NeoHookean input in every
setting except the material (t_end 15, 200 steps, dt 0.075; three keys the
current asset writes at their defaults are new). Copies of steps 39/100/200,
the identification script and that export (`params-ogden-from-scene.json`)
are in `r3-ogden-evidence/` in the parent workspace.

This is not a like-for-like material swap. PolyFEM's IncompressibleOgden
(W = Σ c/m² (Σ λ̃ᵐ − 3) + k/2 (ln J)²) has initial shear modulus c/2 =
500 Pa, about 700× softer than the NeoHookean's 345 kPa, with strong
stiffening from m = 13 and a larger volumetric modulus. Its completion shows
that this material carries the inflation to 1.5 MPa; it does not by itself
say the NeoHookean failure is a solver defect, nor which difference matters.

## Not established / next (2026-09-26; superseded below)

At the time of the diagnosis a global pressure limit or bifurcation, mesh
independence, the causal split between constitutive, structural and contact
effects, and a constrained tangent eigenanalysis were not done, and the Ogden
run was not yet reproduced. The next section does the controlled comparison
and brackets the pressure limit; what remains open is listed at its end. No
material or default change follows from this diagnosis; do not pick Ogden
coefficients just to obtain convergence.

## Controlled Ogden comparison and why the NeoHookean stops (2026-09-29)

Measurement only; no source, default, material or asset change. Evidence,
scripts and runs: `r3-ogden-work/` in the parent workspace (local, not in the
repository; nothing was written to `outputs/`). Figures:
`r3-ogden-work/r3-ogden-comparison.png` and `r3-ogden-work/r3-neohookean-stall.png`.

### Answer

**The NeoHookean stops because the structure reaches a pressure maximum at
285 < p\* ≤ 288.75 kPa, with the cavity at about 7.3–8 mL, long before the
obstacle can carry the load.** Under prescribed pressure there is no
equilibrium near the accepted state beyond p\*. The next step is a snap-through
toward a far larger cavity, and the solver walks it at the 50·d̂ = 50 µm
per-iteration trial cap until the stall-retune budget (50 retunes) ends the
attempt with a named failure, or an observation cap ends it first.

* **Pressure bracket (hold probes).** The diagnosis NeoHookean with the
  pressure ramp held after t = T (`-50000*(t+T-abs(t-T))`, i.e. min(t, T),
  identical to the ramp before T; steps 1–38 reproduce the diagnosis to
  5e-15 relative):
  * held at **285 kPa**: all eight held steps (39–46) are accepted
    immediately with **0 Newton iterations**. The t = 2.85 state is an
    equilibrium at 285 kPa.
  * held at **288.75 kPa**: step 39 fails by name (`Final reduced solve did
    not converge`, 709 iterations, 50 retunes, 1,994 s). Its last iterate has
    moved the cavity from **7.26 mL to 100.5 mL** (max nodal move 33 mm) and
    was still descending.
  * Quadratic fits to the last 5–9 accepted p–V points put the maximum at
    284.3–286.1 kPa (cavity 7.2–8.0 mL). Cubic fits have no interior maximum,
    so the fitted value is indicative only; the probes are the bracket.
* **Snap-through signature.** It is the same in every unfinished NeoHookean
  step: the diagnosis runs (dt 0.075 step 39, dt 0.0375 step 77), EF-01's
  named failure (binary 9f8f25881) and the 288.75 kPa hold:
  * Newton directions of 0.08–0.56 m median and up to 16–412 m (L2 over DOFs),
    so the quadratic model's minimiser is far away;
  * the accepted step is cap-limited, with an accepted fraction of 1.4–2.2 %
    median;
  * the objective at every restart is lower than at the previous one
    (monotone descent across 7–51 minimize calls);
  * the displacement accumulated within the one step is 0.34–1.2 m L2,
    against 0.03–0.06 m for the accepted steps 37–38;
  * the Hessian is indefinite: most iterations use regularized or projected
    Newton.

  EF-01's failed iterate had reached **89.4 mL** (104× initial; max J 73.7,
  1,275 non-elliptic elements). Its pressure work (≈ 292.5 kPa × 82 mL ≈
  24 J) exceeds its elastic-energy increase (1.37 → 20.6 J), so the potential
  was still falling. The Ogden's accepted steps look different: short
  directions and convergence within 0.07 m (step 39).
* **Where the snap would have to go.** In the Ogden run, obstacle contact
  takes over only after the cavity reaches roughly 100 mL (at most ~50 active
  stencils up to step 78, then 164 at step 90 and 980 at step 200). If the
  NeoHookean's supported state is similar, it lies an order of magnitude
  beyond the pressure maximum in volume. This is inferred from the Ogden's
  geometry and not established for the NeoHookean.
* **Local ellipticity is not the blocker.** The Ogden's own energy loses
  rank-one ellipticity at step 10 (75 kPa, max J 3.03) and stays negative in
  every later step: 128 elements at 285 kPa, 3,164 at step 200, min
  −39.6 MPa. It still completes. The NeoHookean's four non-elliptic elements
  at t = 2.85 accompany the limit point; they do not by themselves stop the
  solve.

The **no-restart probe** (the diagnosis NeoHookean with only
`solver/contact/semi_implicit/restart/enabled = false`, so step 39 runs as one
uninterrupted Newton solve at the unchanged trial cap; 8 threads, 3 h cap)
tests whether the snap is traversable at all. At the time of this record it
was still running (1,270 iterations into step 39, 1.8 m accumulated, objective
still falling); its outcome is appended below when it ends.

### The Ogden rerun (reproduction)

Input: `r3-ogden-evidence/params-ogden-from-scene.json` (exported from
`kristin_sim.hipnc`). The overrides follow the diagnosis protocol: iteration,
physical and trim diagnostics, solution-only VTU every 50 steps, per-step
displacement from the physical-diagnostics `endpoint`, copied meshes,
file-selection rewrite, coefficient events to `/dev/null`, no retries. The full
horizon is kept (t_end 15, 200 steps).

Binary: the shared `polyfem/build/PolyFEM_bin` built from `1cb1efc25` (clean;
embedded manifest), sha256 `194fb239a6d10d4366607d93e8c7d3c84c93ca12551135099b505bd8d68bf9af`,
copied to `r3-ogden-work/bin/`.

| run | threads | result |
|---|---|---|
| `ogden-full` | 1 (`VECLIB_MAXIMUM_THREADS=1`, bit-deterministic) | 106 of 200 steps in the 12 h observation cap; 13,520 iterations |
| `ogden-mt8` | 8 (Accelerate unrestricted) | **all 200 steps** in 3.07 h; 17,494 iterations, median 52/step, max 690, 131 stall retunes |

The one-thread run was the protocol run; at about 4 s per iteration it needs
roughly a day. The 8-thread run, closer to how the user ran the scene, is the
one compared at steps 100/200. Over their 106 shared steps the two agree to
1.1 % in cavity volume and 0.8 % in max J. Against the user's surviving steps:

| step (t) | cavity / initial (user / rerun) | max J | min rank-one curvature | non-elliptic elements | elastic energy |
|---|---|---|---|---|---|
| 39 (2.925) | 33.49 / 33.53 | 13.8 / 13.8 | −13.6 / −13.3 MPa | 143 / 145 | 3.570 / 3.571 J |
| 100 (7.5) | 163.7 / 163.2 | 213 / 216 | −38.2 / −38.1 MPa | 3,074 / 3,063 | 64.05 / 63.79 J |
| 200 (15) | 184.35 / 184.36 | 328 / 327 | −39.6 / −39.6 MPa | 3,155 / 3,164 | 80.71 / 80.72 J |

The rerun therefore reproduces the user's run in every aggregate. Pointwise
displacements differ (42 % relative L2 at step 39), mostly as a ~16° swing of
the weakly constrained inflated body about its clamped base; the two reruns
also separate transiently (11 % at steps 7–8) and reconverge. This is expected
for this scene and is not treated as a reproduction failure.

### Pressure-matched comparison (dt 0.075, same geometry, loading, contact)

| p (kPa) | NeoHookean cavity / max J / min curvature | Ogden cavity / max J / min curvature |
|---|---|---|
| 7.5 | 1.07× / 1.04 / 0.34 MPa | 2.14× / 1.22 / 45 Pa |
| 75 | 1.67× / 1.48 / 0.34 MPa | 13.1× / 3.03 / −0.23 MPa (1 element) |
| 225 | 4.02× / 2.36 / 0.34 MPa | 26.7× / 8.55 / −8.5 MPa (35) |
| 285 | 8.47× / 3.44 / −3.11 MPa (4) | 32.7× / 13.1 / −12.8 MPa (128) |

* The paths diverge from the first step. The Ogden's initial shear is
  c/2 = 500 Pa against the NeoHookean's 345 kPa, so at 7.5 kPa it has already
  doubled its cavity.
* **The two p–V paths are shaped differently.** The NeoHookean slope falls
  from ~134 to 6 kPa/mL (limit point). The Ogden slope rises
  from 4–8 to 13.5 kPa/mL by ~160 kPa (step 21) as the m = 13 stiffening engages, then
  falls toward a near-limit of **1.1 kPa/mL at ~600 kPa** (step 81; cavity
  ~100 mL, max J 139). Obstacle contact then grows and the slope rises to
  ~245 kPa/mL at 1.5 MPa. The slope stays positive at every Ogden step.
  Whether the Ogden without the obstacle would reach its own pressure maximum
  is not established.
* **Volume growth.** J > e first appears at step 9 in the Ogden and at step
  36 in the NeoHookean. For J > e the shared volumetric term k/2 (ln J)² is
  no longer convex in J, and its hydrostatic stress k ln J / J peaks at
  k/e = 3.7 MPa for the Ogden. At high stretch the m = 13 isochoric energy
  grows as λ̃¹³, so dilating (which lowers λ̃ = λ J^(−1/3)) is cheaper. The
  "incompressible" Ogden's solid volume reaches **2.38×** its initial volume
  at step 200, with 25 % of the solid at J > e and elements at J ≈ 327.
* **Cross-evaluation.** The Ogden energy would be strongly elliptic at every
  NeoHookean state, including t = 2.85. The NeoHookean energy would fail at
  the Ogden's states (687 elements at 285 kPa). Neither law is elliptic
  along its own path at 285 kPa.
* **Contact and effort.** The NeoHookean has 0–7 active stencils and 3–29
  iterations per step up to t = 2.85. The Ogden has at most ~50 stencils up
  to ~580 kPa and 980 at 1.5 MPa, with median 52 and max 690 iterations per
  step: it is expensive throughout but never runs away.

**Sensitivity probe (not a recommendation).** A NeoHookean with the Ogden's
linearised moduli (μ = 500 Pa, κ = 1e7 Pa; E = 1499.975 Pa,
ν = 0.4999750) tracks the Ogden in step 1 (cavity 2.20× vs 2.14×), then
shows the same snap-through signature in step 2 (917 iterations, directions
up to 1,059 m) and hits the 1 h cap. The low initial shear alone therefore
does not explain completion. What the Ogden adds is the stiffening that moves
its near-limit from ~0.3 MPa to ~0.6 MPa, by which point the cavity has
reached the obstacle.

### Method checks

* Energies are the implemented ones (`NeoHookeanElasticity.cpp`,
  `OgdenElasticity.tpp`). The reconstructed P1 elastic energy matches the
  solver's to ≤ 3e-13 J, and min det F to ≤ 1e-13, at every Ogden step.
* The rank-one curvature is the minimum over unit a, b of D²W(F)[a⊗b, a⊗b].
  It is evaluated from the principal-stretch form of the isotropic tangent
  (Ogden 1984, §6.1.4), which agrees with central differences of the exact
  energies at 300 random F per law to ≤ 3.5e-5 relative. The minimum over
  directions uses a coarse octant grid for all elements, then a fine grid and
  Nelder–Mead for the lowest 200–400. This reproduces the diagnosis's
  closed-form NeoHookean minimum (−3.1108 MPa, same four elements) to 2e-12.
  At every step the minimising direction is re-checked by central
  differences of the exact energy (≤ 1.5e-5 relative, best of h).
* The endpoint vector is mapped to Gmsh order by a permutation. It matches
  the diagnosis VTU displacements exactly at five steps.

### Not established

* Whether the NeoHookean snap-through lands on an obstacle-supported state,
  and where (see the no-restart probe above).
* Whether the Ogden has its own pressure maximum without the obstacle.
* Mesh independence of either path. The non-elliptic elements are few and
  small for the NeoHookean, and thousands for the Ogden.
* A constrained eigenanalysis of the assembled tangent at the limit point.
* The physical adequacy of either completed state. The Ogden completion
  involves large volume growth of an "incompressible" solid.

No material, coefficient or default recommendation follows. The per-iteration
trial cap and the retune budget determine how the unfinished step ends, not
whether an equilibrium near the accepted state exists.
