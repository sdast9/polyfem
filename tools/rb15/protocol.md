# RB-15 bounded feature study (predeclared 2026-09-10)

Reuse the unmodified RB-04 real-form probe and its 70/55 EV/VV data and Fixed70
control first. New tools change no production source or option.

Compare current frozen feature keys, frozen shared parent coefficient, global70
control, and a smooth tanh field with full product derivatives. Shared means one
coefficient for a declared point/edge parent (point/triangle in 3D), including its
subfeatures, fixed through every trial. Choose its initial parent coefficient from
the actual current form at the initial snapshot; this tests assignment consistency,
not estimator accuracy. For disconnected parents use separate values. New members
inherit the frozen parent value; unknown parents require an outer update.

The 2D edge is (-1,0),(1,0), point (0,.2); initial H has endpoint blocks 10I
and point block 100I. The 3D triangle is (-1,0,0),(1,0,0),(0,2,0), point
(0,.5,.2), with triangle blocks 10I and point block 100I; cross y=0 (FV/EV).
No weights, dmin, friction, force saturation, controller, CCD or trial-cap changes.

For a frozen affine relative tangential coordinate a(x), dimensionless t=a/w,
k=mid+delta*tanh(t), w=.2*dhat, with endpoint constants from the two feature
limits (2D 70/55). Geometry uses actual IPC distance/gradient/Hessian. Include
b grad(k), both cross Hessian terms and b Hessian(k). Also report the deliberately
incomplete k grad(b) as a negative control. The field frame is frozen at refresh;
this is a conditional energy, not an objective corotational production model.

Check eps=1e-3,1e-5,1e-7,1e-9: last normalized energy/gradient jumps <1e-6
for continuous candidates, current 2D |jump|>40. Check fixed-region centered
energy-gradient differences (relative <1e-6), gradient-Hessian differences
(<1e-5), split midpoint quadrature n=16,64,256,1024 (work residual <1e-5
for complete candidates). Continuous Hessians are not required.

Check reversed trial order, empty initialization/new contact, separation/recontact,
identical-x refresh and changed outer coefficient (account Delta k*b separately).
Check exact permutation pullback and length/energy conversions L=.001,1,1000,
Q=.01,1,100. Test membership changes at positive gap: discontinuous region switch,
additive duplicate parent, and partition-of-unity blend of duplicate descriptions.
Report the failure examples rather than asserting general topology invariance.
Contrast ratios 1,10,100,1000 test shared/global force distortion; k positivity
alone does not establish contact-force direction for variable k. Report sampled
normal force, tangential force and timing (warm repeats, no performance guarantee).

No nonlinear solves, continuum accuracy, arbitrary mesh refinement, 3D EE/mollifier,
interpolated mechanical estimator, mesh changing topology, or production adoption
is claimed. Deliver candidate comparison and explicit decision-pending handoff.

## Retained exploratory outcome and added diagnostic

Candidate-01 had a C++ class-name ambiguity (compile failed). Candidate-02 passed
all preceding checks, but its boundary-centered increasing field stayed repulsive:
minimum sampled normal force 5511.16463. The attraction assertion failed, so that
run is retained as failed; it does not establish attraction for that field.
For the next diagnostic move the field center to a=.08 at positive corner gap,
with width .02 and endpoint k=1,1000. The explicit condition is
`b grad(k) dot n > k f(d)`, which gives attraction despite k>0. This is a changed
counterexample geometry relative to the coefficient field, not a relaxed check.
Also add two real disconnected IPC parents with frozen k=70/700, simultaneous
EV/VV transitions, one-parent separation/recontact, and support-onset convergence.
