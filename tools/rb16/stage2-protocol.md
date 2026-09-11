# RB-16 stage 2 — real contact form and assembled FEM

Declared before execution, 2026-09-11. Bounded experimental comparison, not a
production controller selection or a full production nonlinear-solver execution.
Reuse RB-14's P1 Neo-Hookean square, E=10, nu=.3, rho=1, implicit-Euler incremental
noncontact objective at dt=.2, initial velocity zero and fixed top nodes. The
inertial predictor stays fixed throughout the load cycle: this is load-parameter
refinement of one incremental objective, not a dynamic trajectory/time refinement.
Physical energy is the assembled weighted objective divided by dt^2.

A real exact-selector collision map couples the bottom-center node to a fixed
long edge at initial gap .12; dhat=.1. One known parent remains EV whenever active,
so Fixed-mode positive coefficient is the bounded single-parent RB-15 candidate.
Geometry starts without contact, then compression, unloading, free separation
and recontact occur under point-force knots K0*[0,-.25,0,.10,-.15,0]. K0 is the
initial full reduced noncontact compliance inverse. Positive force is upward;
energy includes -force*u_y. This local loading is not whole-surface compression
against a floor: only the selected node/edge collision is in the fixture.

Predeclared matrix: n=2/4 mesh, 4/8 subdivisions per segment; initial multiplier
.01/1 relative to the same initial raw Rayleigh coefficient. Compare actual
semi-implicit refresh plus post_step callbacks, and positive fixed-parent
coefficients corrected at converged fixed-load endpoints. For the callback
control, refresh once at each load start and after publication; capture both
states and never relabel a published endpoint. This is an explicit driver schedule,
not a claim to reproduce every production wrapper/restart/AL path.

The outer candidate obtains full reduced tangent K=1/(J H^-1 J^T) and
p=d-J H^-1 r from the current feasible iterate, including the applied point force.
Compare factors 2/4, observation windows 1/2 and hysteresis 0/.001 physical length.
At most 12 corrections per load, each followed by a warm-started solve at identical
load and predictor. Recompute tangent predictions after every converged endpoint.
The band is [.1 sqrt(.5), .1 sqrt(.9)]. Interval target uses the stage-1 convention.

Primary outer alternatives use exact local tangent inputs; these are not exact
nonlinear predictions. Additional n=2, 4-subdivision, multiplier=.01 challenges
use K*[.9,1.1], p+[-.001,.001]; K*[.1,10], p+[-.03,.03]; central p bias +.06;
and unavailable intervals. Keep prior positive k for invalid, indefinite, empty,
mixed, inactive, unavailable or zero-only intervals. No silent fallback or retry.
No empirical bound is claimed; retained engineering violations are results.

Newton/LLT and Armijo follow RB-14: 80 iterations, normalized residual <=1e-8,
40 halvings, Armijo 1e-4. Retain total-energy subtraction as in RB-14; stage-1
arithmetic improvements are not transplanted into production/FEM. Every direction
and search freezes trim/refresh identity. Invalidate derivative/search state after
each callback/update. No quasi-Newton history, early interruption, AL or rollback
is used. Both modes use real IPC CCD, elastic max-step/validity and the separate
50-dhat trial-displacement cap. No constraint floor or force saturation.

Record iterations, searches, solves, timings, per-endpoint actual coefficient,
active count, gap/RMS/minimum (identical for one parent), tangent-prediction error,
residual, prescribed-coordinate error, minimum element det(F), contact action/
reaction balance, controller disposition and band occupancy separately. Record
coefficient-event energy at fixed coordinates separately from conservative
contact-energy changes along displacement; check whole-cycle telescoping.
Report post-publication residual and force changes under the new state.

The full predeclared matrix is attempted, including numerical failures. Failed
cycles stop at their last iterate without advancing load or retrying. A process
exit/check pass does not mean the cycle converged or all gaps lie in band.
Stage-1 already covers independent heterogeneous contacts; this stage does not
certify coupled multi-contact mechanics, friction, EE/mollification, general
feature discovery, constitutive refinement or continuum contact accuracy.

## Paired energy-arithmetic control, declared after the diagnostic

The original raw-energy matrix remains the primary retained reproduction. Its
first callback case stalls at residual 1.472813529e-8. A read-only full Newton-step
diagnostic at that endpoint remains CCD/element-valid, reaches 4.834485606e-16,
and gives raw energy difference +4.996003611e-16, while 3/5-point Gauss integration
of the unchanged assembled objective gradient gives -9.941810505e-18 and
-9.941810498e-18. Armijo's right-hand side is -1.988362066e-21. The arithmetic
sign discrepancy is reproduced; this is not a controller success/failure verdict.

Run the same full 76-case matrix as an explicitly labelled `arithmetic=integral`
comparison. Every search uses 3/5-point Gauss energy-difference estimates for the
same fixed-k objective and accepts only when d5+abs(d5-d3)<=Armijo_rhs. Preserve
80 iterations, 40 halvings, 1e-8 residual, CCD and elastic validity. There is no
raw-to-integral fallback within a run or silent retry of a failed cycle. Record
quadrature gradient evaluations and discrepancies; the quadrature disagreement
is an error diagnostic, not a rigorous error bound. Costs include that extra work.
The driver retains and reports failures from both arithmetic modes independently.
This experimental arithmetic change is not installed in PolySolve/PolyFEM.

## Prescribed block-compression supplement

The primary point-force matrix closes the normal gap but can stretch the block
against its fixed top; do not call it a bulk-compression benchmark. Add eight
explicit prescribed-top cycles: n=2/4, 4/8 subdivisions, current/outer factor-2
window-1/no-hysteresis, multiplier=.01, integral arithmetic. The same knots now
specify top y displacement in length units, with zero applied point force and
unchanged incremental inertia. Before each prescribed-coordinate update check
positive det(F), elastic step validity and CCD. If infeasible, retain the last
state and report the boundary-step failure without retry or AL substitution.
Normal corrections warm-start only after that fixed prescribed update is valid.
These cases separately exercise actual block compression/unloading. They do not
change the original matrix or erase any of its failures.

Four additional derivative-only controls (n=2/4, both forms) check assembled total
energy/gradient/Hessian with finite differences and compare 3/5-point gradient
quadrature against direct endpoint energies on a finite, fixed-k feasible path.
