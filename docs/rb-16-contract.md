# RB-16 — Controller timing comparison contract

2026-09-11. Stage 1 is characterized; **RB-16 remains in progress**.
No production controller, estimator, coefficient law or engineering gate is selected.
See the [validation record](rb-16-validation.md) and
[predeclared spring protocol](../tools/rb16/stage1-protocol.md).

## Mechanical reference and coefficient state

Use the [RB-13](rb-13-contract.md) exact, independent spring model
`E_nc = sum K_i (d_i-p_i)^2/2`, with affine positive gaps, ordinary unweighted
IPC barrier `sum k_i b(d_i)`, h=1, zero dmin and no friction. K is in force/length,
k in force/length^3 and energy in force*length. This is a load-parameter cycle,
not a time integrator. The nonlinear barrier primitive is compiled from the
verified effective IPC checkout. The experimental Newton solver is Python.

For enclosing K and p intervals with p_upper<=U, use
`k_lower=K_upper max(L-p_lower,0)/f(L)` and
`k_upper=K_lower (U-p_upper)/f(U)`, with f=-b'.
These are conditional exact-model intervals, not certified FEM estimator bounds.
Exact predictors are an oracle control; RB-14's empirical errors remain unresolved.
Every scalar parent is known beforehand and its identity remains fixed. That is
compatible with the bounded shared-parent idea from [RB-15](rb-15-contract.md),
but does not exercise its feature geometry or resolve unknown-parent discovery.

The band is L=sqrt(.5), U=sqrt(.9), matching the single-contact interpretation of
production's squared-gap thresholds. Report each gap, minimum/median/maximum,
active RMS and `min(mean(d^2),100 min(d^2))`. Band occupancy counts contacts with
true p<=U; separating/inapplicable demand is reported separately. This criterion
is an observer in the experiment, not a new production acceptance condition.

## Compared timing alternatives

1. **In-Newton timing control:** source-derived global trim, up cooldown 3,
   down cadence 30, factor 2, excursion 256 and existing trim rails. The trim
   counter persists across loads; the excursion is reanchored per load. Coefficient
   state changes only after a complete accepted direction/search. This reduced
   control excludes production refresh/calibration, caps, stall handling and
   collision discovery; its measured costs cannot be assigned to PolyFEM.
2. **Post-publication timing control:** freeze k for the solve, report its endpoint,
   then take one source-derived gap feedback step for the next load. Save the
   changed coefficient and residual at the old coordinates separately. This is
   not execution of production's complete refresh path.
3. **Fixed-load interval correction:** fully solve, observe per-parent gaps,
   and make at most 12 multiplicatively bounded updates, warm-starting at that
   same load. For a valid interval, target its geometric center, or half its
   upper bound when the lower is zero. Compare factors 2/4, windows 1/2 and
   trigger expansion 0/.01. A window counts converged outer observations, not
   physical time or consecutive Newton steps. No production parameter is chosen.

All Newton directions, gradients, Hessians and Armijo searches use one frozen k.
A coefficient event invalidates the previous derivatives/search state; this
probe has no quasi-Newton history to retain. It reassesses convergence using the
actual new coefficient. There is no early interruption, retry or rollback policy.
The positive affine-gap step limit is an exact scalar non-crossing condition;
this stage does **not** execute IPC CCD or the production trial cap. Both remain
unchanged in the actual solver, and the retired constraint floor remains absent.

## Unavailable predictions and limits

Empty, mixed, inactive, unavailable and zero-only intervals retain the previous
positive coefficient and state the classification. This is a comparison branch,
not evidence that retention gives good physical protection or correct band gaps.
A deliberately misleading predictor can report inactive demand while the actual
root remains below L. A future response could request a better enclosing estimate,
report unresolved engineering acceptance, or use an explicitly selected diagnostic
fallback. Automatic retries and unapproved positive floors are not used here.

For the two heterogeneous contacts, the global timing controls and local interval
candidate differ in assignment as well as timing. Their comparison cannot isolate
pure timing causality. Single-contact controls isolate that distinction more
cleanly. No mechanically coupled contact, mesh, material nonlinearity, changing
normal, friction, unknown contact parent or constitutive dynamics is certified.

## Endpoint and work accounting

At a fixed x, the coefficient contribution is `sum Delta k_i b(d_i)` with zero
displacement work. Between coefficient events, the barrier-energy difference is
recorded as the negative conservative barrier-force work. Per-solve and whole-cycle
telescoping checks keep these two terms separate. One fixed-k path is independently
integrated using Simpson quadrature. This does not measure full external-load work,
frictional dissipation, continuum energy balance or dynamic contact acceptance.

A post-publication change cannot relabel an old endpoint as equilibrated under
its next coefficient state: the standalone control changes residual from
1.27e-16 to .2632 while adding .10821 energy at unchanged coordinates.

## Remaining RB-16 stage

Execute the actual real-form timing/refresh lifecycle and public assembled-FEM
compression/unloading, starting from the RB-14 n=2/4 P1 Neo-Hookean reference and
RB-15 bounded parent candidate. Predeclare experimental estimator/protection and
assignment alternatives, fixed-load correction budgets, load refinement and
physical observations before running. Keep CCD, determinant validity, existing
solver convergence and failed solves visible. The spring arithmetic improvement
is not permission to change production line search. No model decision is needed
to continue bounded comparisons, but integration/default selection remains gated
by the plan's decision register.
