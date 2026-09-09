# RB-04 discrete work convention and remaining measurements

2026-09-09; reference implementation `9e7c3b59f`. This defines the refinement
postprocessor, not a new contact model or production acceptance threshold.

For physical step n, x_n is the returned full displacement before time-history
advancement, s_n the acceleration scaling, g_i,n the form's objective gradient
divided by s_n, and R_n the recorded external support force on the system.
The public fixtures have a fixed slab, prescribed moving top, no distributed
loading, constant mesh/dhat, and an initially undeformed, configured-at-rest
state. Initial elastic stress/energy and the first contact snapshot's zero
energy/gradient are checked. This does not generalize initial-state inference
to arbitrary loaded, contacting or restart inputs.

Define dx_n=x_n-x_(n-1). The computed estimates are:

```
W_D,right = sum_n R_n dot dx_n
W_D,trap  = sum_n .5*(R_n+R_(n-1)) dot dx_n
C_i,right = sum_n g_i,n dot dx_n
C_i,trap  = sum_n .5*(g_i,n+g_i,n-1) dot dx_n
```

R has support-on-system sign. C has resistance-gradient sign, opposite the
form's force on the system. Fixed-obstacle increments vanish. Trapezoids use
endpoint forces and must be tested under load/time refinement. Right-endpoint
elastic stress work minus elastic energy change is a quadrature remainder,
not a material dissipation. W_D,right minus sum C_i,right equals minus free
residual virtual work under the sampled endpoint force state (simple selector
BCs); this is an algebraic equilibrium diagnostic, not physical certification.

For transient implicit Euler, dx_n=dt*v_n. With fixed physical mass M:

```
C_inertia,n = (M*(v_n-v_(n-1))) dot v_n
            = K_n-K_(n-1) + .5*(v_n-v_(n-1))^T M*(v_n-v_(n-1))
```

The latter term is the implicit-Euler numerical dissipation. The independent
P1 tetrahedral mass integration uses integral(N_i*N_j)=V*(1+delta_ij)/20 and
rho=1000 for these fixtures. It verifies saved kinetic energy and accumulated
inertia gradient work across all 84 transient endpoints, with maximum absolute
work discrepancy 7.82e-14. It does not establish smooth-path convergence at
contact onset. Observed total numerical dissipation increases in this matrix
as dt decreases, so no monotonic dissipation convergence is claimed.

## Friction state distinction

The endpoint recorder currently samples friction after the final lag update.
The just-completed minimization used the preceding frozen lag state. Therefore
the measured C_friction,right is an **updated-lag resistance-work estimate**,
not necessarily the work of the force used to produce x_n. Its positive sign
in these monotone fixtures does not certify trajectory dissipation or stick/slip.
The friction potential itself is not integrated dissipation.

One read-only Luna audit independently confirmed the support-work sign,
implicit-Euler identity, and missing pre-update friction forces. A statement in
that audit conflated quasistatics with absence of a time integrator; source and
RB-04 scaling evidence correct it: these quasistatic fixtures retain the
integrator, so friction uses (x_n-x_(n-1))/dt, not absolute displacement x_n.
Relevant implementation: `FrictionForm.cpp` compute_surface_velocities,
first_derivative_unweighted and update_lagging; `ImplicitEuler.cpp`;
`NonlinearElasticVarForm.cpp` final lag loop and diagnostic writer.

Next observation: capture friction gradient immediately before and immediately
after each lag update at the same x. Report both discrete work estimates and
the residual reconstructed with pre-update friction while holding other forms
fixed. Do not silently increase the user's lag budget or change acceptance.

## Normal contact and coefficient-state path

Optimization-event energy changes occur at Newton iterates, not physical-time
trajectory points. Their sum is not an interchangeable physical parameter-work
term. A possible explicit discrete convention for contact storage B is:

```
Delta B_n = [B(x_n;theta_n)-B(x_(n-1);theta_n)]
          + [B(x_(n-1);theta_n)-B(x_(n-1);theta_(n-1))]
```

This changes coefficients at the previous physical coordinates, then moves at
fixed endpoint coefficients. The second term can be called a parameter-state
energy change **under this declared convention**. The first equals the contact
gradient line integral only when the potential is continuous along the chosen
path; RB-02's feature jumps prevent assuming that. Current outputs lack
B(x_(n-1);theta_n). A private endpoint-coefficient snapshot evaluated at the
step's starting coordinates is the minimal extra observation. Subsequent path
quadrature/feature-boundary checks must distinguish quadrature error from jumps.

Until those observations exist, no sum of external work, endpoint barrier energy,
friction objective and numerical event deltas is called a complete balance.
