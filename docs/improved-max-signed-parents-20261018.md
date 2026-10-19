# Improved max operator under semi-implicit stiffness: the signed parent law — 2026-10-18

**Status:** implemented and verified on the local branches `improved-max-signed`, in PolyFEM
(base `9bd8bb223` = `origin/main`) and in the IPC Toolkit fork (base `1f1b5dbf` = the pin).
**Not pushed. The pin is not moved.** No default changed: the convergent formulation stays off by
default, and nothing changes unless a scene asks for `use_improved_max_operator` under
`semi_implicit` stiffness. Isolated build, evidence and scene runs: parent workspace
`improved-max-work/` and `wp3c-1a-03-work/`.

## Why

**The flagged pattern.** The MPM contact study flagged WP3C-1A-03: under semi-implicit stiffness,
the tangential jump on a frictionless 2D pressure patch rose on the finest mesh. The investigation
(parent workspace `wp3c-1a-03-work/FINDINGS.md`) traced it to exactly coincident interface nodes:
- The test meshes are structured with a 3:2 node ratio.
- At a coincident pair, standard IPC adds a vertex–vertex term on top of the point–edge terms
  (merged weight 2).
- That term's gradient has a tangential part Δy/d, so it pushes the pair apart tangentially.
- Per-contact stiffness amplifies it in a strong first impulse.

**Why the existing fix was unavailable.** IPC's improved max operator (Li et al. 2023) removes
exactly this duplicate term with a negative vertex–vertex correction. Adaptive K2, which has it on,
does not show the step-1 artifact. Semi-implicit stiffness refused the operator (RBR-04, the
[RB-21 record](rb-21-parent-keyed-kappa.md#rbr-04--the-improved-max-operator-is-a-checked-restriction-2026-09-21)):
- The toolkit merges the positive and negative contributions of one stencil into a single collision
  with a net weight.
- The parent law scaled that net weight by the mean over the positive parents.
- With unequal coefficients that is not the sum of the parents' potentials. The energy jumps by
  |κ₁−κ₂|/2·b(d) at a corner: 62.24 on the RBR-04 fixture.
- A collision whose contributions cancel is dropped, together with its signed remainder.

## The law

Every parent contribution p, including the improved max operator's negative corrections, carries
its own coefficient κ_p from the RB-21 law: the Rayleigh quotient of the frozen Hessian on the
parent's own stencil, at the closest point on the whole primitive. The barrier is

  E = Σ_p w_p κ_p b(d_p(x))

**Continuity.** Each term is C¹ in its own distance at positive gap, and a parent keeps its key
across closest-feature switches. The sum is therefore continuous at every seam for **any**
coefficients, and the corner's effective coefficient Σ w κ is the same on both sides.

**At a coincident pair** (the WP3C-1A-03 geometry), the edge parent whose closest point is the shared
vertex has the same stencil direction as the vertex–vertex correction. So both get the same
coefficient and cancel exactly, and the point–edge term that remains is normal.

**After sliding.** When a vertex has slid across a corner since the snapshot, a signed remainder
(κ_edge − κ_vv)·b(d_vv) persists until the next refresh. It is continuous, and its size is the
coefficient difference, not the full κ.

## Implementation

**Toolkit (`ipc-toolkit-fork`, branch `improved-max-signed`):**
`NormalCollisions::set_separate_signed_contributions(bool)`, off by default.
- When it is on, the builder keeps positive and negative contributions to the same stencil in
  separate collisions: one merge map per sign, both in the thread-local builders and in `merge()`.
- Each collision's parents then share one sign, so none cancels to zero and none is dropped.
- With unit coefficients the separated collisions sum to the merged set. With the flag off every
  call routes to the original map: identical behaviour.

**PolyFEM (branch `improved-max-signed`):**
- `BarrierContactForm` turns the flag on for semi-implicit stiffness with the improved max operator.
  The stencil identity has no parent coefficients and is still refused by name.
- `coefficient_keys` keys negative parents too.
- `assign_collision_stiffness` sets the scale to the |w|-weighted mean of a collision's parents. As
  a safety net, a collision that mixes signs is refused by name: it would mean a set built without
  separation.
- `FrictionForm` copies the flag into its lagged normal set.
- `State::init` refuses `use_improved_max_operator` under `semi_implicit` only with
  `coefficient_identity: "stencil"`. The physical barrier stays refused.
- The input spec documents the new behaviour.

**Side effects, by design:**
- **Trim controller.** The band statistic weights positive collisions only, so the negative
  corrections stay out of the trim controller's gap band.
- **Hessian projection.** A negative collision's Hessian is projected to PSD on its own. That gives
  a stiffer Newton matrix than the merged set would. Energy and gradient are exact; the measured
  cost is below.

## Verification

All on the isolated build (`PolyFEM_bin` sha256 `9ec7aebe…`; after formatting `0174c171…`, behaviour identical), single-threaded.

| Check | Expected | Result |
| --- | --- | --- |
| `unit_tests "[rbr04]"` (rewritten to this contract) | stencil identity refused by name, parent identity constructs a separated set, the real form continuous at both seams with heterogeneous coefficients, homogeneous coefficients reproduce κ × the improved-max potential, a mixed-sign set refused; fixed/adaptive merged and unchanged; the merged positive-parent mean still shows the historical 70/40 seam, and the signed sum on the separated set does not | **149 assertions / 2 cases pass** |
| Affected selection (RBR-04 list minus `[al_budget]`) | pass | see *Affected tests* below |
| `tools/rbr04/run_probe.py --expect-signed` (new mode) | parent improved-max configurations continuous (heterogeneous included), stencil + improved max refused by name, every control unchanged | **492 checks pass**. Heterogeneous jump at both seams **0** (was 62.24), area-weighted heterogeneous ±2e-13. The stencil controls reproduce the historical 124.481 / −186.603 exactly (`evidence/seam-probe-signed/`) |
| Five `scenes/semi-implicit` smokes vs the `db2a8bb12` binary (no library change since) | byte-identical | **all 55 VTU/VTM/PVD/stats files byte-identical**. The 5 run manifests differ only in provenance, paths and timings (`evidence/smoke-*`) |
| Saved Stage 1a inputs without improved max (K3, K4, level 2) | unchanged | frame-by-frame identical to the saved histories, largest difference 1.1e-19 m/s |

### The WP3C-1A-03 pressure patch

Central-region maximum tangential jump (m/s), same inputs as the study except the contact flags.
K6 = semi-implicit + improved max. K5 = semi-implicit + area weighting + improved max. "Shifted" =
body 1 moved 2 µm so no node pair coincides, the control that isolates the artifact.

| Configuration | Level 0 | Level 1 | Level 2 | Level-2 step 1 |
| --- | ---: | ---: | ---: | ---: |
| K3 semi, non-convergent (study) | 1.20e-3 | 9.34e-4 | **1.88e-3** | 1.88e-3 |
| **K6 semi + improved max (signed)** | 1.27e-3 | 7.93e-4 | **2.58e-4** | **1.08e-5** |
| K3 shifted | 1.27e-3 | 6.74e-4 | 1.99e-4 | 1.05e-5 |
| K4 semi + area (study) | **2.84e-3** | **2.29e-3** | 1.22e-3 | 1.22e-3 |
| **K5 semi + area + improved max (signed)** | 1.43e-3 | 7.96e-4 | 3.51e-4 | 3.06e-5 |
| K4 shifted | 4.93e-4 | 3.29e-4 | 1.96e-4 | 2.82e-5 |
| K2b adaptive + area + improved max (control) | — | — | 3.84e-4 | 1.50e-5 |

**K6 removes the artifact.**
- Its maxima decrease monotonically under refinement, and at level 2 it is within 30 % of the
  shifted control.
- Through the first eight steps it tracks the shifted run frame by frame.
- With the signed law, the interior coincident pairs carry µN of tangential force, as on the shifted
  mesh, against 2.6–5.3 mN before.

**K5 removes most of K4's excess** (2–3.5×). At levels 0 and 1 it stays above its shifted control.

**The remaining step-1 tangential force sits on the two end nodes** (0.9–9 mN), where the bodies'
square corners line up within dhat.
- That vertex–vertex contact is real geometry: the closest feature is the other corner, and improved
  max correctly keeps one term for it.
- The shifted control lacks it only because its corners no longer line up.
- Its influence on the central region shrinks with refinement.

**Newton cost:** total nonlinear iterations at level 2 are K6 995 vs K3 1001, K5 978 vs K4 990. No
penalty is visible from the separate PSD projection.

**Not addressed:** with no coincident nodes, every mode still has about 2e-4 m/s peak and 3e-5 m/s
late at level 2. That fails the study's 1e-5 limit and is the separate discretization question
(the study's cluster C09).

## Affected tests

- `unit_tests` with the RBR-04 record's affected selection minus `[al_budget]` (`[kappa_continuity]`,
  `[semi_implicit_coefficients]`, `[input_validation]`, `[contact_stiffness_mapping]`,
  `[friction_lag]`, `[contact_cache]`, `[coefficient_events]`, `[physical_diagnostics]`, `[form]`,
  `[run_manifest]`, `[direction_filter]`, `[contact_floor_retired]`, `[resource_containment]`,
  `[rollback]`, `[fully_prescribed]`, `[output_kinematics]`): **124 cases / 10,046 assertions pass**.
- After clang-format (pinned hooks of both repositories) the build was rerun. `[kappa_continuity]` and
  `[input_validation]` pass (35 cases / 742 assertions).
- The formatted binary (`0174c171…`, line numbers changed) reproduces the K6 level-2 run frame by
  frame (961 of 961 identical).
- The full CTest suite was not run.

## Limits

- **2D only.** In 3D the improved max operator also emits negative edge–vertex corrections and
  positive vertex–vertex ones. Separation is implemented for all three merge maps, but no 3D seam
  was measured.
- **Friction.** Friction with this combination is untested. All scenes here are frictionless. The
  friction set receives the same separated, signed collisions.
- **Toolkit tests.** The toolkit's own test suite was not built. The toolkit behaviour is pinned
  from the PolyFEM side (`[rbr04]`).
- **Not production-validated.** This is not a production validation of the convergent formulation
  under semi-implicit stiffness: area weighting with semi-implicit stiffness remains outside the
  documented validation, as in RBR-04.

## Publication (pending the user's decision)

1. Push the toolkit branch to `sdast9/ipc-toolkit:semi-implicit-stiffness`.
2. Move the pin in `cmake/recipes/ipc_toolkit.cmake`.
3. Rebuild against the pin and rerun the checks above.
4. Push PolyFEM to `main`.

The Houdini asset emits `use_convergent_formulation` only, with the physical barrier on by default,
so it still gets the physical-barrier refusal. No asset change is implied.
