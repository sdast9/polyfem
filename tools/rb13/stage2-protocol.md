# RB-13 stage 2 — predeclared analytical test protocol

2026-09-10, before the first stage 2 probe run. No production change is planned.
Retain stage 1 and its result unchanged. Use a new local evidence directory.

Reference: one affine physical gap, SPD-condensed scalar K>0, free predictor p,
fixed ordinary clamped-log barrier, dmin=0, all weights/normalizations absorbed
in physical k>=0, dhat=1. No friction, feature transitions or time trajectory.
Prove force positivity/decrease, unique equilibrium and monotonicity before
interpreting an interval as a band guarantee.

Test exact bands [.2,.55], [.4,.7], [.65,.9], K=.5,6,50 and predictors below,
at, inside and above each band, including p<=0 and p>=dhat. Test both coefficient
endpoints and quarter-grid interior values. Check roots outside either positive
bound using coefficients 1% beyond it. These are test samples, not an algorithm
for selecting a production coefficient or a fallback for empty intervals.

At band [.4,.7], cover enclosing Cartesian boxes:

| K range | p range | Intended category |
| --- | --- | --- |
| [4,6] | [.1,.2] | Positive interval |
| [4,6] | [.45,.55] | Zero lower bound |
| [4,6] | [.45,.7] | Zero-only interval |
| [1,100] | [-.2,.65] | Empty interval |
| [4,6] | [.8,.9] | All free gaps above band; inactive demand |
| [4,6] | [.5,.9] | Mixed applicability; no all-box band certificate |
| [4,6] | [-.2,.9] | Mixed applicability with positive raw lower bound |

For each applicable nonempty box use an 11x11 K,p grid and five coefficient
fractions. Check the worst-case corners with coefficients just outside the
bounds. The box is a manufactured admissible set, not an empirical error bar.
The proof establishes enclosure coverage in exact arithmetic; a finite grid is
a numerical check, not proof of an application's enclosure or a floating-point
interval-arithmetic certificate.

Counterexamples: raw [0,0] for free gaps above the band; an actual K outside its
assumed box; an empty rectangle containing a correlated manufactured family
that does admit one coefficient; positive increasing f with two equilibria.
Reject invalid K, bands, reversed boxes and nonfinite inputs. Explicitly report
k=0,p<=0 as lacking a positive-gap equilibrium. No attractive force, automatic
clamp, midpoint fallback, stiffness escalation, retry or active-contact deletion.

Compute interval endpoints with 60-digit Decimal log arithmetic. Compile a
new standalone root bridge against the effective IPC `barrier.cpp`; solve
K(d-p)=k f(d) by at most 120 bisections and preserve raw requests/results.
Use a 120-step independent Decimal reference on selected roots. Predeclared
criteria: gap/band error <=2e-11*dhat; force residual normalized by
K*dhat+abs(k*f) <=2e-10; dimensional scalar/derivative comparisons <=1e-11
after normalization. Fixtures avoid endpoint singularities and extreme scales;
no tolerance relaxation or regenerated stage 1 measurements is permitted.

Report all failed runs and protocol changes. Publish small fixtures, summarized
results, contract/validation updates and the roadmap; do not run solver scenes
or claim stage 3, physical accuracy, estimator calibration or production bounds.
