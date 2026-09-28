# R3 inflation — why it stops at step 39 (diagnosis, 2026-09-26)

Status: **diagnosed as a material/structural problem, not a trim-controller
problem.** Measurement only; no source, default or asset change. Recorded in
the repository on 2026-09-28 (the diagnosis itself was done on 2026-09-26).
Full report, scripts, runs and plots:
`outputs/r3-inflation/20260926-diagnosis/report.md` in the parent workspace,
which is being archived to the Pitt share (AGENTS.md, *Archived test
outputs*).

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
1.32), the values saved in every Houdini scene of the case. Copies of steps
39/100/200, the identification script and a reconstructed input (material
replaced, other settings assumed unchanged — they were not saved) are in
`r3-ogden-evidence/` in the parent workspace.

This is not a like-for-like material swap. PolyFEM's IncompressibleOgden
(W = Σ c/m² (Σ λ̃ᵐ − 3) + k/2 (ln J)²) has initial shear modulus c/2 =
500 Pa, about 700× softer than the NeoHookean's 345 kPa, with strong
stiffening from m = 13 and a larger volumetric modulus. Its completion shows
that this material carries the inflation to 1.5 MPa; it does not by itself
say the NeoHookean failure is a solver defect, nor which difference matters.

## Not established / next

A global pressure limit or bifurcation, mesh independence, the causal split
between constitutive, structural and contact effects, and a constrained
tangent eigenanalysis were not done. The Ogden run is located (above) but not yet
reproduced from the reconstructed input. The next discriminator is a
controlled comparison with that input (same geometry, loading, time and
contact settings), comparing both the
deviatoric and the volumetric large-strain response against this NeoHookean
law, the pressure–volume path and the local stability margin. No material or
default change follows from this diagnosis; do not pick Ogden coefficients
just to obtain convergence.
