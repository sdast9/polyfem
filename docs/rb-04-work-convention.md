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

Historical next observation (implemented in the paired-state extension below): capture friction gradient immediately before and immediately
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
path; RB-02's feature jumps prevent assuming that. The initial outputs lacked
B(x_(n-1);theta_n); the paired-state extension below now supplies it. A private endpoint-coefficient snapshot evaluated at the
step's starting coordinates is the minimal extra observation. Subsequent path
quadrature/feature-boundary checks must distinguish quadrature error from jumps.

Until those observations exist, no sum of external work, endpoint barrier energy,
friction objective and numerical event deltas is called a complete balance.

## Paired-state observation extension (2026-09-09)

The source extension records `friction_before_update` and
`friction_after_update` gradients in the final `lagging` object, plus the full/free
residual reconstructed with the former friction gradient and other endpoint
forms held fixed. It records `barrier_start_energy_with_endpoint_snapshot`,
which supplies the common-coefficient start energy in the decomposition above.
No gradient used by a minimization, lag budget or coefficient law changes.

In the coarse default friction fixture, the pre-update-friction residual is
1.236e-8 versus 59485.575 after updating. Accumulated right friction work is
13640.494 before and 22930.380 after. Their difference, 9289.886, matches the
earlier updated-lag residual-work discrepancy (up to numerical error). This
locates that discrepancy in the selected force-state convention; it does not
certify friction's physical accuracy. The finite-lag approximation still has
the measured load-increment dependence and must be assessed as such.

For coarse frictionless quasistatics, the physical-start convention gives
accumulated parameter energy +544.163 and fixed-snapshot motion energy -138.763,
which sum to final barrier storage 405.401. The negative motion contribution
is not automatically a defect: coefficient stiffening evaluated at prior
coordinates can create stored barrier energy that is released along the next
segment. Equality of this bookkeeping identity does not prove that the contact
gradient integral equals the motion-energy difference across feature switches.

At dt=.25,.125,.0625 the default-band friction pre-update right work estimates
are 13640.494,14893.843,15558.075, while post-update estimates are
22930.380,19050.134,17557.237. The corresponding final pre-update-friction
residuals are 1.236e-8,1.250e-8,2.260e-6; updated residuals are
59485.575,27779.768,13348.598. This supports interpreting the discrepancy as
a finite-lag force-state difference, with a measured load-increment dependence.
It neither mandates a larger lag budget nor establishes physical convergence.
The fine friction run initially failed before contact, then an identical repeat
completed. Both outcomes remain in the compact paired-state data.

For frictionless quasistatics, physical-start parameter energy is
544.163,589.017,625.316 across those increments, while final barrier storage is
405.401,387.038,377.952. This term does not disappear in the measured refinement;
it must remain explicit in a numerical barrier-energy account. The refresh after
the final published endpoint is outside this final endpoint's coefficient state;
it is not silently added to its physical-time budget.


## Discrete trajectory budget and elastic path reference (2026-09-09)

This completed stage uses the preserved physical-state-pair outputs from the
`e652fae53` production build; no new solver run or production source change.
Tool: [trajectory_budget.py](../tools/rb04/trajectory_budget.py).
Data: [trajectory-budget-results-20260909.json](../tools/rb04/trajectory-budget-results-20260909.json).
Fresh local evidence: parent `outputs/rb-04/20260909-trajectory-budget/` (first
analysis) and `outputs/rb-04/20260909-trajectory-budget-final/` (final analysis,
including explicit unavailable totals for the zero-endpoint failed run). Neither
analysis overwrites earlier solver outputs. Exact analyzer copies, input hashes,
reference hash and results are retained. Command from PolyFEM:

```bash
/opt/homebrew/bin/python3 tools/rb04/trajectory_budget.py --evidence ../outputs/rb-04/20260909-physical-state-pairs --output ../outputs/rb-04/20260909-trajectory-budget-final
/opt/homebrew/bin/python3 tools/rb04/test_trajectory_budget.py
```

Let Ce and Cb be right-endpoint elastic and normal-contact gradient work;
Cf uses the friction gradient immediately BEFORE the final lag update. Let
P=B(x_previous;theta_endpoint)-B_previous and M_B=B_endpoint-B_common_start.
Define R_elastic=Ce-Delta E and R_contact=Cb-M_B. Summed over physical steps:

```text
W_support,right = Delta E_elastic + Delta B + Delta K + D_IE + Cf
                + R_elastic + R_contact - P + epsilon_equilibrium
```

Here epsilon_equilibrium=W_support,right-(Ce+Cb+Ci+Cf).
This equality closes a discrete algebraic budget, not a physical energy proof.
R_contact combines right-endpoint quadrature error and any frozen-snapshot
feature-jump contribution; this stage does not separate them or call either
physical dissipation. R_elastic is independently identified as endpoint work
approximation error through integration of the constitutive stress along each
straight displacement segment. D_IE is the independently reconstructed
implicit-Euler mass identity; it is absent in quasistatics. Cf is a numerical
resistance-work estimate under the solved lag, not a continuum dissipation proof.

The support forces are those recorded by the existing writer. For these fixtures,
the before/after lag difference contributes zero work on reaction-carrying DOFs
(checked at 1e-9 absolute tolerance); do not reuse this support-state shortcut
for arbitrary moving/frictional boundaries. The initial undeformed/noncontact
state, fixed mesh, Neo-Hookean material, rho=1000, no distributed work and existing
integrator interpretation are fixture restrictions, not inferred general defaults.

All 9 complete trajectories / 84 accepted endpoints pass the retained 1e-9
relative-plus-absolute reference screen. The tenth trajectory is the preserved
fine-friction failure before step 1; its totals are unavailable, not zero.
Independent P1 stress-path integration with 2/4/8-point Gauss rules gives maximum
absolute work-minus-energy errors 0.8932054 / 4.248e-7 / 1.783e-10 across the
84 segments. Quadrature-sampled minimum det(F) is .836284, not a proof of
positivity everywhere on every segment. Endpoint stress work also agrees with
the saved full nodal elastic gradient dotted with displacement increment.

The maximum per-step algebraic closure error is 2.911e-11; maximum absolute
solved-state equilibrium-work defect is 7.305e-7. These have different meanings:
closure is an arithmetic consistency check; the latter reflects approximate
endpoint equilibrium along the actual increment. All measured solved-lag friction
step costs are nonnegative (minimum zero) on the monotone fixtures only.
Independent uniaxial-stretch energy/work, rigid-translation zero-work and inverted-
endpoint rejection controls pass (3 tests). Existing independent mass/kinetic
checks pass at every transient endpoint in this dataset.

| Quasistatic frictionless dt | Elastic right-work remainder | Contact right-work remainder | Parameter-state energy P |
| --- | ---: | ---: | ---: |
| .25 | 111325.1021 | 10459.3500 | 544.1632 |
| .125 | 56175.5549 | 2989.7173 | 589.0173 |
| .0625 | 28354.8107 | 725.7493 | 625.3159 |

The elastic endpoint remainder approximately halves as the increment halves;
independent quadrature identifies it as work-approximation error on these
segments. The contact remainder decreases but is not yet attributed to a unique
cause. Parameter-state energy persists; keep it explicit. These quantities must
not be relabeled physical loss to make a balance appear complete.

**Next exact RB-04 stage:** obtain/reconstruct a private endpoint-coefficient
snapshot for quadrature along one saved frictionless physical displacement
segment, including its collision map and coefficient identity. Compare gradient
integrals at increasing orders/subdivisions with common-snapshot energy changes;
record feature identities and isolate discontinuities. Start with one segment
having nonzero contact remainder, then expand only as needed. Reuse the completed
RB-02 boundary probe as evidence, not as a substitute for the actual scene path.
No coefficient/controller redesign is authorized here; that belongs to RB-13–RB-17.
Friction's broader reversal/stick-slip accuracy remains RB-10. RB-04 remains in
progress until its required work-accounting limits are resolved or explicitly
characterized; this algebraic closure does not mark physical balance passed.
