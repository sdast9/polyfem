# PF-02: contact-floor characterization — 2026-09-06

This is the first experimental stage of PF-02. The production solver and its
defaults are unchanged. The experiment uses solver baseline `4b6e970c3`, IPC
`9da3094`, and PolySolve `012658e`; the effective build dependencies match those
pins. Teseo is excluded from new tests by the user's September 6 instruction.

## Which behavior is being compared

`solver.contact.semi_implicit.constraint_floor` defaults to `1e-4`; its gap
threshold is that value times `dhat`. It has two distinct effects:

1. `BarrierContactForm::update_collision_set` sets a below-threshold pair's
   stiffness scale to zero after assigning coefficients. This removes its
   barrier energy, gradient and Hessian. It happens on collision-set rebuilds,
   including line-search trials, not just in AL preparation.
2. `project_floor_pairs` filters closing search directions through four
   sequential full-coordinate projection sweeps. The caller maps the result
   back to reduced coordinates, discarding changes to prescribed DOFs.

The intended purpose is to avoid extremely large barrier derivatives when
contacts reach tiny gaps while retaining collision checks. This is different
from minimizing a smooth positive-coefficient barrier and is not, by itself, a
complete hard-contact formulation with contact multipliers and KKT conditions.

In the forward VarForm path, the filter is installed on the shared ALSolver for
AL preparation and the initial reduced solve. Its callbacks are cleared after
each inner solve. Subsequent friction-lagging minimizations call PolySolve
directly; they still use the contact form's floor-dependent energy, but do not
install this ALSolver direction filter. The legacy and differentiable call
sites also install the filter for their ALSolver calls. Thus neither the energy
switch nor its projection can be described as an AL-only operation.

The comparator sets only `constraint_floor=0`. It keeps CCD, the trial-step cap,
the coefficient provider, refresh/controller/restart policies, tolerances and
friction settings. It also keeps the existing `gap_floor=0` default (the separate
optional barrier-force saturation). This is a floor-disabled semi-implicit
baseline, not a proof that every coefficient is positive or every whole-solve
objective stays fixed. Indefinite system curvature, coefficient caps, trimming
and refreshes remain separate concerns.

## Real-form micro-experiment

The standalone probe links the actual built PolyFEM libraries. It uses a point
above an edge in 2D, `dhat=1`, a constant system Hessian `100 I`, unit form/global
weights, and frozen coefficients. One snapshot already contains the contact
(gap 0.2); the other starts outside support (gap 1.2), so the pair first appears
after the snapshot. Each configuration runs in a fresh process. The form has a
function-static position cache shared across instances; separate processes
avoid contaminating this comparison. That cache ownership is not fixed here.

With the pre-existing contact, both configurations assign coefficient 100 above
the threshold:

| Gap | Current energy | Floor-disabled energy | Current point vertical gradient | Floor-disabled point vertical gradient |
| --- | ---: | ---: | ---: | ---: |
| 0.000100001 | 1842.066038 | 1842.066038 | -1,999,980.697 | -1,999,980.697 |
| 0.000099999 | 0 | 1842.070038 | 0 | -2,000,020.697 |
| 0.000050000 | 0 | 1980.697501 | 0 | -4,000,000.376 |

The threshold secant is approximately **+9.21033e11** with the current floor,
versus **-2.00000e6** without it. Closing across the threshold therefore reduces
the current barrier energy abruptly. Reversing the sample order back above the
threshold restores the original coefficient and energy. The newly created
contact gives the same results to floating-point precision; the frozen-snapshot
coefficient remains 100 without the floor, including on revisiting a trial.
This verifies this positive-curvature stencil, not all possible contact types.

For an initial point velocity -1 and fixed horizontal edge, full-coordinate
projection produces vertical velocity -1/3 on all three vertices. Its relative
gap velocity is zero only while the edge is allowed that projected motion.
Restoring fixed edge velocities to zero leaves gap velocity **-1/3**. In a
direction-level prescribed-motion example with edge velocity +0.25, restoring
the prescribed motion leaves gap velocity **-5/12**. This example illustrates
an affine constraint on directions; it does not imply that reduced Newton
iterations advance the prescribed BCs during a line search.

The floor-disabled form performs no such projection; its barrier force remains
active. Both configurations reject a segment that crosses the edge, and CCD
limits its step to about 0.399994; the limited segment also passes the direct
collision check. This establishes the checked segment's
collision behavior, not a global collision certificate.

As an independent force-balance reference, scalar bisection on the real
floor-disabled point force balances a downward load of 4e6 at gap
**5.00000047e-5**. The relative force residual is below **1.4e-12** for both
snapshot cases; the opposite edge force also balances that load. This is not a
production nonlinear-solver run. The current floor does not bracket that scalar
force balance over [1e-6, 0.01]; below the threshold it reports zero contact force
and residual 4e6. No hard-contact equilibrium claim follows from this comparison.
Element inversion is inapplicable to this point-edge micro-experiment.

## Additional reproduced refresh inconsistency

At a fixed gap of 5e-5, the current collision rebuild yields zero barrier
energy/coefficient. Calling `update_barrier_stiffness` at the **same position**
restores coefficient 100 and energy 1980.697501. The test resets the global trim
to one and uses a position-independent Hessian, so this is not a change in
material curvature or global trim. The refresh assigns coefficients without
reapplying the floor mask; a subsequent nearby collision rebuild removes them
again. The floor-disabled comparison has no such jump.

Refreshes occur at subsolve setup/restarts and can also occur in `post_step`
for new contacts or configured refresh intervals. This result strengthens the
need for a consistent model and lifecycle policy. It does not establish how
often this path affects a production scene. Moving refreshes to another phase
or changing the floor mask is not implemented in this investigation.

## Scene comparison and validation limits

Ballburst uses the preserved PF-01 input copy: its obsolete
`pressure_discr_order` was already translated to `discr_orderq`. The prescribed-cube/fixed-slab fixture uses the
repository's `quasistatic-semi.json`. Numerical changes between each pair are
limited to `constraint_floor`; output paths are isolated. Supporting files are
copied and SHA-256 recorded. Timestep/load schedules and visualization settings
are unchanged. No Teseo run is part of this experiment.

| Scene / floor | Completion / exit | Wall time | Minimum logged gap / dhat | Minimum saved tetrahedral det(F) |
| --- | --- | ---: | ---: | ---: |
| Ballburst / current | 200/200, exit 0 | 264.06 s | 0.01128479356 | 0.93547696425 |
| Ballburst / disabled | 200/200, exit 0 | 261.46 s | 0.01128479382 | 0.93547696426 |
| Prescribed cube, fixed slab / current | 4/4, exit 0 | 4.28 s | 0.10010344272 | 0.85711727214 |
| Prescribed cube, fixed slab / disabled | 4/4, exit 0 | 4.30 s | 0.10010344272 | 0.85711727214 |

The checked-in [scene measurements](../tools/pf02/scene-results-20260906.json)
retain the raw termination diagnostics and both volume and nonvolume counts.
All four runs have zero floor-projection log entries, zero restart entries and
zero final-reduced-solve failure entries. Logged accepted-iterate gaps are above
the current floor threshold of `1e-4*dhat`. These runs therefore supply regression
evidence, not validation of the active floor's constraint treatment. Rejected
line-search trial gaps are not exhaustively instrumented by these log counts.
CCD remains enabled and logged gaps are positive; there is no independent
whole-scene intersection audit here.

The deformation check covers 201 saved Ballburst frames and five cube/slab
frames **per configuration**, including initial frames. No tetrahedral sample
has a nonpositive or nonfinite determinant. The slab VTUs also have four
zero-filled F entries on the rigid obstacle's four vertices in each frame.
Their coordinates match `slab.obj`, and their connectivity belongs only to its
two triangles. They are reported separately, not mistaken for inverted FEM
elements: volume samples are selected by tetrahedral connectivity before
examining determinant signs. This checks saved output samples, not every trial
state or a general curved-element inversion certificate.

Both Ballburst runs finish each step through the configured negative-slope
tolerance. At the final solve, the reported gradients are 0.223305 and 0.229881;
the reported slopes are -4.62469e-18 and -4.62718e-18, inside the configured
-1.40572e-15 threshold. These are solver iteration diagnostics, not independently
recomputed final physical residuals. The cube/slab final reported gradients
are 1.2595e-7 and 1.26406e-7 against 2.27951e-6 tolerance.

The largest componentwise differences in final saved displacements are
1.52e-12 for Ballburst and 2.43e-16 for the cube/slab. Thus these two pairs do not
show a material final-displacement difference. One run per configuration and
incidental local workload are not a controlled runtime benchmark. Neither the
timings nor the small differences establish a causal benefit of disabling the
floor, especially without observed floor projections.

The build completed, all four micro-probe configurations ran, and the focused
AL plus contact/friction derivative selection passed **10 cases / 205
assertions**. Both modes of the affected prescribed-motion smoke passed as
listed above. No new full-suite claim is made. Scene-scale reaction balance and
independently recomputed endpoint equilibrium remain unmeasured; the only
independent force-balance measurement here is the scalar micro-experiment.

## Reproduction

From the PolyFEM checkout, with a configured Unix Makefiles build:

```sh
cmake --build build --target PolyFEM_bin unit_tests -j 6
python3 tools/pf02/run_probe.py --build build --output /absolute/evidence/probe
```

The script compiles only the characterization source, using the existing test
target's compiler flags and libraries. It records exact build commands and
JSON measurements and source/executable hashes. The checked-in
[measurements](../tools/pf02/results-20260906.json) record this baseline. It is
not a golden regression that asserts the defective
behavior must remain unchanged.

For scene comparisons, copy each input directory into separate `current/input`
and `floor_disabled/input` folders. Set absolute isolated `output.directory`
paths and set `solver.contact.semi_implicit.constraint_floor` to `1e-4` and `0`,
respectively. Run the same binary with `--json input/params.json --log_level
debug`, recording exit status and wall time. Do not substitute truncated
schedules. The local runner, copied inputs, hashes, logs and outputs are in
the parent workspace's `outputs/pf-02-contact-floor/`; private scene assets are
not included in the repository.

`tools/pf02/summarize_scenes.py` reads that runner's `scene-results.json`, logs,
and saved VTU deformation gradients. It computes determinants from all saved
`F_1`/`F_2`/`F_3` samples, separately selects samples attached to tetrahedra,
reports logged gap minima and restart/filter counts,
and retains the last raw solver termination message. It requires NumPy and
supports the uncompressed inline-binary VTU format used in these runs.

## Decision boundary

The micro-experiment reproduces implementation defects; it does not show that
every production result is inaccurate. Disabling the floor is an experimental
baseline, not an accepted production fix. A production choice requires deciding
between a barrier model and a separately specified hard-contact model, followed
by validation of the chosen model, including coupled contacts and reactions.
PF-09 is the latter formulation's separate scope. PF-06 covers the filtered
directional derivative. PF-02 remains open beyond this characterization stage;
PF-03 through PF-05 can be pursued independently without choosing a new contact
model. No tolerances or defaults were changed to make the experiments pass.
