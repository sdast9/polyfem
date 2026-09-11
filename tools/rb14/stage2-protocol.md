# RB-14 stage 2 — assembled FEM protocol

2026-09-10. Declared after API setup, before estimator comparisons. Retain failed
setup builds/runs. No production estimator selection, floor, CCD, stopping or
controller change. Use actual PolyFEM Neo-Hookean P1 triangle assembly on a unit
square, consistent assembled inertia via ImplicitEuler, and constant body load.
The embedding API initializes forms without running a contact-free simulation.

Clamp the top edge through an explicit coordinate selector Q. Capture the current
residual at prescribed top displacement c (default zero); other current DOFs
start at zero. A single selected bottom-center vertex contacts a long, prescribed
horizontal segment with initial gap .08 and dhat .1. Build an actual IPC selector
map from the three proxy vertices into FEM plus two obstacle nodes. Use the
actual BarrierContactForm for frozen contact energy/derivatives/CCD and current
semi-implicit extraction. This selected-contact fixture does not represent whole-
surface collision geometry or continuum contact-area convergence.

Calibration: default n=2 subdivisions/side, E=10, nu=.3, rho=1, dt=.2,
initial downward speed=.3, body load=0, top c=0, obstacle motion=0.
Separate changes: n=4,8 (fixed domain mesh refinement); E=1,100; rho=.1,10;
dt=.05,.5; speed=.1,.8; body load=1,10; top displacement=-.02;
obstacle displacement=.02. Held out: n=6 with opposite cell diagonals, E=3,
rho=.3, dt=.35, speed=.6, nu=.45 combined; plus n=4/speed=1.5 and
n=4/E=1/rho=.1/load=20. Keep these cases after calibration, without retuning.

Compare full reduced SPD reference; diagonal; graph-radius 0,1,2 neighborhoods
from actual basis element adjacency, holding external increments fixed; gap-
direction control; actual current assigned coefficient with trim=1 as a distinct
control. Reference H is enabled elastic+inertia Hessian divided by integrator
acceleration weight. Residual includes enabled body load too. Verify Hessian
versus directional finite differences of that residual and inertia scaling;
deformation-dependent pressure is absent and remains an explicit limitation.

Compute K and p by two RHS solves, never an inverse. Estimate target d=.05 with
k=K*max(.05-p,0)/f(.05), holding k fixed for each solve. Measure linear-reference
predicted roots and nonlinear assembled realized gaps separately. Zero demand
is classified, not active-contact deletion: do not run an unprotected candidate.
Use dense SPD checks for the small reference and independent sparse LLT solves.
Record conditioning, matrix/factor nonzeros and storage accounting, graph-build,
submatrix-extraction, factorization/solve cost and assembly cost. Timings use seven
repeats; no production throughput or total-process memory claim.

Calibrate radius-1 empirical multiplicative K-error and additive p/dhat-error
ranges on calibration cases only; report held-out joint and marginal coverage.
Do not pad ranges or choose safety factors. Report empty/inactive empirical
bands without clamping. Reuse of stage-1 empirical ranges is a separate control.
Measure fresh full-reference recomputation as a fallback proposal and compare
old-dt k on a new-dt assembled state; no production cache or recovery installation.

Tolerances before comparison: relative/scaled solve residual 1e-8; dense/sparse
reference agreement 1e-8; noncontact directional gradient/Hessian FD 2e-5 with
central step 1e-6; force pullback/virtual work 1e-8. Standalone nonlinear Newton
limit 80, backtracking limit 40, Armijo 1e-4, normalized free residual 1e-8.
Reject indefinite directions, use actual contact and elastic admissibility/step
limits and verify positive element determinants at endpoints. No tolerance
relaxation or silent projection. Nonconverged/invalid outcomes stay in the matrix.
Target mismatch is a measured model error, not forced to pass a linear tolerance.
One run per process avoids inherited function-static scene state. Rebuild the
isolated probe, run all declared cases, check relevant existing mapping/assembly
regressions, preserve source/binary identity, and publish results and limits.
