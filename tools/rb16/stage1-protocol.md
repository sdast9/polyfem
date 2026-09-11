# RB-16 stage 1: spring controller protocol

Declared before execution, 2026-09-11. Experimental controls only; no production
policy selection. Start with the exact RB-13 affine-gap quadratic reference and
compiled effective IPC barrier. h=1 length unit; K in force/length; k in
force/length^3; energy in force*length. L=sqrt(.5), U=sqrt(.9).

Compare (a) source-derived in-Newton global trim timing, (b) frozen solve with
one post-publication feedback update for the next load, (c) bounded local interval
corrections at the same load, from the latest positive iterate. The timing control
isolates post_step's cooldown=3, downward cadence=30, factor=2, upward excursion
256, trim rails 2^-32..2^32 and severity=min(mean(d^2),100 min(d^2)). It does not
execute BarrierContactForm, calibration, cap selection, stall recovery or discovery.
The post-publication control is a timing experiment, not a full production refresh.

Every Newton direction and Armijo search freezes k. Recompute gradient/Hessian
and discard any search history after a coefficient event; no quasi-Newton history
is used. Scalar positivity step limiting implements the exact affine non-crossing
condition, not IPC CCD. The production CCD and trial cap are untouched.

Predeclared cycles: p knots [1.2,-.5,.94,1.2,-.3,1.2], with 4,8,16 subdivisions
per segment. Single K=1 starts k=.02 (weak) or 100 (strong); heterogeneous K=[1,100]
has p2=.5*p1-.25 and k=[.02,2]. All start at positive d=1.2 with no active contact.
All configurations run; no outcome-based parameter selection. Outer factors 2/4,
window 1/2 consecutive equilibrated observations, gap hysteresis 0/.01 (expanded
trigger, original target band), maximum 12 corrections per fixed load. Windows
count independently solved fixed-load endpoints; repeated observations can cost
zero Newton iterations and are explicitly an outer observation window, not time.

Prediction variants: exact; enclosing K*[.9,1.1], p+[-.01,.01]; wide
K*[.1,10], p+[-.3,.3]; deliberately misleading central p+0.6 with zero width;
unavailable. Exact/narrow/wide centers otherwise use true K,p. RB-13 bounds apply
only when p_upper<=U. Empty/mixed/inactive/unavailable or zero-only intervals keep
positive previous protection and report unresolved/applicability; no invented
positive floor, attraction, retry, or acceptance gate. When violated and valid,
move k toward the geometric center of positive interval bounds, or half upper
bound for a zero-lower interval, bounded by the selected multiplicative factor.
This interior target is a comparison choice, not an approved production formula.

Newton max 100, normalized residual <=1e-10, Armijo 1e-4, max 60 halvings;
normalization is 1+max absolute spring force+max absolute barrier force.
Independent bisection endpoint error <=1e-8. Failures remain in results and do
not advance the cycle. A no-progress/12-correction endpoint remains numerically
converged but is labelled separately from band occupancy. No early-interruption
policy is exercised; only converged endpoints drive the experimental outer loop.

Measure all gaps and coefficients, minimum/median/max, RMS active gap and severity,
per-contact original-band occupancy for mechanically applicable contacts, prediction
errors, residuals, Newton/search evaluations, retunes, reversal counts and wall cost.
Record endpoint coefficient identity and post-publication residual separately.
Track displacement barrier-energy change and fixed-coordinate Delta k*b separately;
test their telescoping identity. This is conservative spring-path accounting,
not dynamic dissipation, full FEM work/reactions or continuum accuracy.

Controls: independent root checks; exact/narrow enclosing-box coverage when valid;
mean-band counterexample; misleading predictor violation; empty/unavailable
retention; separation/recontact; fixed-coordinate refresh changes old residual;
force integral versus energy difference by Simpson quadrature on one fixed-k path.
These are expected-phenomenon checks, not goldens for iteration counts or timings.
Stage 2 must execute real-form timing and public assembled-FEM compression/unloading,
with selected experimental RB-14/RB-15 candidates and unchanged solver criteria.

## Retained first-run arithmetic finding

The first run used subtraction of total energies for Armijo and completed only
42/126 cycles. Near-stationary rounding stalls remain in probe-01. The second
run evaluates the identical energy difference using log1p and factored polynomial
differences, preserving the original residual and Armijo criteria, iteration
limits, cases and parameters. This is a probe arithmetic change, not a production
repair or tolerance relaxation. Validate it against ordinary energy differences
away from cancellation and Decimal values for close points.

## Timing-control source review

The third run corrects the reduced in-Newton control to preserve iterations-since-
trim across load increments, as the real form does unless a trim update resets
it. Probe-02 incorrectly reset that counter at every load. Retain probe-02 as an
exploratory result, not the current timing comparison. The reduced control still
reanchors upward excursion once per new load; full production refresh/calibration
remains outside this stage. Budget disposition is emitted only when a further
correction is actually requested after the 12th correction.
