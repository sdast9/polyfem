# RB-14 stage 1 — estimator comparison protocol

2026-09-10, declared before running comparisons. Standalone supported quadratic
networks establish estimator behavior, not production FEM accuracy or selection.
Use fresh output and the unchanged RB-13 Cholesky reference and compiled IPC
barrier primitive. Preserve all runs, including failed or partial runs.

Two planar bodies each have three nodes joined by rotated SPD spring blocks.
Their remote endpoints have elastic supports. Each node has lumped mass.
H=K+M/dt^2 in physical incremental-energy units; r at u=0 includes the velocity
predictor and optional remote compression loads. The contact normal is (0.6,0.8),
with opposite endpoint derivatives. dhat=0.1*h, current gap=0.08*h, target=0.05*h.
Spring stiffness scales E*h and nodal mass rho*h^3 (a declared 3D size model,
not mesh refinement). Parameters default to E=10, contrast=1, anisotropy=1,
support=2, h=1, rho=1, dt=.2, approach speed=.4, remote load=0.

Compare: full SPD solves for J and r; radius-0 and radius-1 principal-block
solves with all exterior displacement increments fixed at zero; full diagonal
approximation; normalized Rayleigh raw scale and its gap-direction conversion.
For the Rayleigh-based candidate use the displacement-restricted predictor.
Retain raw r_R separately; treating it as K is an explicitly labeled comparison,
not execution of the production assignment, cap or trim.

Separate sweeps (one parameter changes from default): contrast .1,1,10,100;
anisotropy .1,1,10; support .1,2,100; h .25,1,4; rho .01,1,100;
dt .02,.2,2; speed 0,.4,2; remote load 0,1,10. Deduplicate the shared baseline.
Use all these calibration fixtures; do not optimize a neighborhood or fit k.
Calibrate radius-1 empirical K_true/K_local min/max and normalized predictor-error
min/max. No padding or safety factor. These are empirical ranges, never certified.

Then evaluate an untouched holdout set: each individual parameter outside the
calibration range (contrast 1000, anisotropy 100, support .01, h .1, rho .001,
dt 4, speed 4, remote load 100), one combined contrast=1000/anisotropy=100/
support=.01/rho=.001/remote load=10 case, and a four-node-per-body topology.
Report separate K and p coverage and joint coverage, interval existence and
realized endpoint gaps where an active empirical interval exists. Do not widen
ranges or retune after inspecting holdout results.

Controls: nested fixed-boundary K is an upper bound for the exact SPD quadratic
(by restricted minimization); no such claim for a diagonal approximation or
local predictor. Demonstrate both signs of diagonal stiffness error. Check
normal force action/reaction, exact permutation pullback, and a prescribed rigid
obstacle with fixed and moving targets. Reject singular/indefinite/nonfinite,
unsupported maps, unavailable residuals and stale dt/map tokens explicitly.
Proposal only: invalidate stale state, recompute from fresh full SPD data if
available; otherwise return unavailable without a coefficient. Measure full
recomputation on stale-dt and stale-map controls; do not install production policy.
Compare a shared/global k by intersecting exact bands of heterogeneous independent
fixtures; if empty, measure each scene at both conflicting endpoints. No clamp.

Before execution tolerances: solve residual / (1+||rhs||_inf) <=1e-8; identities
<=1e-8*(1+|reference|); target gap and equilibrium residual normalized by dhat
and force scale <=1e-8. Scalar bisection 120 steps, compiled IPC force, coefficient
held fixed. Approximation errors and missed empirical coverage are findings,
not failed accuracy assertions. Record 15-repeat median/min wall time per estimator
and tracemalloc peak Python allocation for one call; no production speedup or
factor-reuse claim. Exact small solve is a double-precision reference, not an
interval-arithmetic certificate. No scenes, HDA, production rebuild or default
change; assembled-FEM and realistic cost follow-up remains stage 2.
