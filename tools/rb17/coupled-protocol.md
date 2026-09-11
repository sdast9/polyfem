# RB-17 first gate: selected scalar controller on coupled mechanics

2026-09-11, declared before execution. The user approved the RB-17 proposal.
This first gate checks its coupled-contact limitation before production wiring.
The exact [manifest](coupled-cases.json) is committed before any run.

Use E_nc(u)=u^T H u/2 with affine d=p+J u, ordinary compiled IPC barriers,
two fixed scalar parents and dhat=1. H is SPD, the full shared predictor is p
at every state, and W=J H^-1 J^T is exact. The independent scalar estimate is
K_i=1/W_ii. Thus estimator inaccuracy, unknown parents, nonlinearity of the
noncontact model and geometry discovery cannot explain a failure here.
Start d=(.8,.8); reconstruct the minimum-energy full displacement via H and J.
Initialize positive coefficients from the raw mapped Rayleigh expression listed
in the manifest. It is the existing raw scaling, not the complete production
trim/cap/calibration lifecycle. Each declared scalar parent has one affine gap;
this stage does not test real EV/VV feature transitions or CCD geometry.

Execute the selected half-upper/geometric-center interval rule, bound updates
by factors 2/4, freeze coefficients in every Newton solve, and publish the solved
state. Recompute full tangent/prediction per outer solve, including factorization
cost. A coefficient fixed point outside the eligible band is unresolved, never
controller success. A solve can converge while the controller remains unresolved.
An unchanged proposal ends as no-progress; do not repeat identical solves merely
to exhaust the correction budget. No force/target model is altered in response.

Five declared cases plus one held-out stronger-coupling case compare different
signs and asymmetry of coupling. Each has two factors, three length conversions,
two evaluation orders and two explicitly separate arithmetic modes: 144 runs.
Direct subtraction represents the ordinary objective arithmetic. The stable
quadratic/barrier-difference control reuses RB-16's algebraically equivalent
formula solely to distinguish arithmetic stalls from model failures. No fallback,
tolerance relaxation or production arithmetic edit is allowed. Preserve every
failure and all Newton/outer histories. These are static endpoints, not cycles.

Units: energy'=Q energy, length'=s length, H'=Q/s^2 H, p'=s p,
k'=Q/s^4 k, force'=Q/s force. Residual normalization includes Q/s as the unit
reference so the numerical criterion is unchanged under conversion. The affine
positive-gap line bound replaces no production CCD: it is exact non-crossing
for these abstract scalar constraints. There is no FEM determinant or friction.

Independently solve frozen-k equilibria by coordinate minimization with scalar
bisection (no Newton Hessian or line search). Check reconstructed full residual,
W coupling identity, finite differences, coefficient-event energy, zero event
displacement work, unit covariance and evaluation order. Use a numerical screen
of 1e-8 in dimensionless gap for oracle/unit/order comparisons; failures remain
recorded, not used to alter the actual solve tolerance 1e-10.

Independently characterize band feasibility by enumerating intersections of
the linear inequalities L<=p+W F<=U and F>=0. If a strictly positive-force interior
point exists, convert it to positive coefficients and verify its equilibrium
with the independent solver. This is a diagnostic oracle, not installation of a
coupled coefficient rule. For the incompatible positive-coupling case,
d1 >= p1+(W12/W22)*(L-p2) > U proves that no nonnegative forces satisfy both
bands. Report this separately from numerical failure or physical infeasibility.

Decision gate: a feasible-band case in which the selected controller stabilizes
outside the band despite an exact tangent and converged equilibrium disproves
sufficiency of that scalar update for the coupled fixture. Preserve the result
and prepare concrete alternatives before changing the selected model. A failure
does not forbid useful independent implementation work, but full production
integration of a replacement coupling law requires a new explicit decision.
