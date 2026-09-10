# RB-13 stage 3 — predeclared mechanical scope protocol

2026-09-10, before running the new probe. This is standalone characterization;
no production default, law, controller, mapping or invalid-H recovery is selected.
Preserve stage 1/2 code/results and use fresh evidence. Compile an isolated shared
wrapper around effective IPC's actual barrier.cpp; all new equilibrium forces
use those primitives. The current Rayleigh expression is source-traced algebra,
not execution of the full coefficient/provider/trim pipeline.

Fixtures, with dimensionless coordinates representing one consistent internal
length/force/time system unless explicitly converted:

1. Two separately supported contact endpoints: H=diag(A,B), J=[-1,1],
   p=.2, target=.5, dhat=1; (A,B)=(1,1),(2,8),(1,100),(10,100),
   (1,1000),(1,1e6). Verify AB/(A+B), displacement, force and support balance.
   With A=2 and B/A=1,100,1e4,1e6, verify the analytic rigid-limit error
   1/(1+B/A). Compare an explicitly prescribed endpoint, K_eff=A.
2. For these springs compare r=(J H J^T)/||J||^2, the gap stiffness
   r/||J||^2 for displacement constrained parallel to J^T, and relaxed
   compliance K_eff. Equality is not presumed. Also show the zero-padded fixed
   obstacle stencil and explain its different admissible coordinates.
3. Implicit Euler: supported A=4,B=12, masses 1,3, initial displacement zero,
   initial gap .6, dt=.1,.2,.4 and relative approach speed 0,1,2,4 split equally
   between endpoints. Derive H and free gap from the full incremental objective.
   Show unchanged H at fixed dt but different demand; verify compressed target
   roots at .5 and full free residuals. Zero demand is not barrier deletion.
4. Convert the dt=.2, speed=2 case with all 27 combinations of length scale
   .001,1,1000; force scale .01,1,100; time scale .1,1,10. Convert mass, velocity,
   dt, spring stiffness, gap, dhat and k consistently. Check root, force,
   displacement and physical/incremental objective scaling. This does not test
   the production controller's unit covariance.
5. Stable nonlinear spring: g(d)=6(d-.1)+alpha(d-.1)^3, alpha=0,10,100;
   tangent anchors .1,.3,.45,.5, target=.5. Quantify local prediction versus
   actual nonlinear equilibrium and exact target demand. At alpha=100, anchor=.1,
   also test the upper coefficient of the tangent-model band [.45,.55]. A band
   failure is retained as a counterexample to global use of a local tangent.
6. Two contacts: H=diag(2,3,5), J=[[1,-1,0],[0,1,-1]], p=[.2,.25],
   target=[.5,.55]. Form W by solves, retain its off-diagonal terms, manufacture
   positive forces from W F=target-p, and solve the full 3-DOF nonlinear reference.
   Compare coefficients from only diag(W). Separately test W=[[2,1],[1,2]],
   target increment [.1,.5], which requires a negative contact force.
7. Reject unsupported singular, indefinite, nonsymmetric, nonfinite H and zero J.
   Show singular H=[[1,-1],[-1,1]] with both relative and rigid-motion-sensitive
   J, then an explicitly prescribed support as a different well-posed fixture.
   Preserve small positive stiffness for H=diag(1e-10,1): no floor/projection.

Tolerances before execution: direct scaled identities 1e-10; normalized scalar
root residual 2e-10 and gap error 2e-11*dhat. Tiny positive stiffness is checked
as a ratio, not against an absolute tolerance that would accept zero. Scalar
residual normalization is abs(g(d)-k*f(d))/(abs(g(dhat))+abs(k*f(d))); the unit
conversion control additionally uses K*dhat+abs(k*f). Nonlinear
prediction errors are measured, not forced to pass a linear-model tolerance.
Coupled Newton: at most 80 iterations, up to 60 halvings, positive gaps and
Armijo constant 1e-4; normalized residual <=1e-10, target gap tolerance 2e-9.
These are standalone fixture controls, never changes to production termination
or CCD. Known diagonal conditioning reaches 1e10 only in the tiny-positive-H
case; moderate coupled fixtures do not justify general conditioning claims.

Record failures and incomplete runs without relaxing tolerances. Publish the
small result, reproducible probe, derivations, limitations and remaining model
choices. Do not run scenes, private inputs/Teseo, HDA or broad FEM suites for
this isolated analytical extension. Completing the RB-13 investigation is not
production coefficient promotion.
