# Completed RB review — 2026-09-20

**Status: review and implementation plans only. No production repair is implemented by this review.**

The user requested a review of the completed RB work, asking whether the
solutions were correct and which findings belong upstream under fixed or
variable barrier stiffness. The scope was subsequently expanded to include
RB-23/RB-24: this review now covers **RB-01–RB-24**. The user requested plans
detailed enough for other models to implement later. The two handoffs are:

- [Bounded repair plan](rb-review-repair-plan-20260920.md): RBR-01–RBR-04.
- [Upstream contribution plan](rb-upstream-plan-20260920.md): UP-01–UP-12.

These identifiers are review tasks; they do not renumber the RB plan. A future
implementation session should select one task and recheck its reproduction.
Publishing these documents does not mark any proposed repair implemented.

## Assessment

**Most solutions address the right failure mechanism, within their recorded
scope. A blanket statement that all implementations are correct is not
supported.** Two additional defects were reproduced during this review, and
two further implementation gaps were identified in source:

1. **RBR-01 / RB-04: output kinematics still fail when consecutive positions
   are exactly equal.** `saved_solution_kinematics` guesses that history has
   advanced from `x_prev() == solution`. In the nonlinear save-before-advance
   path, a new held position can also equal the old position. A real-library
   Implicit Euler probe moves from 0 to 0.1 with `dt=.25`, advances history,
   then holds at 0.1. Export returns velocity **0.4** and acceleration **1.6**;
   the integrator's current-step rules give **0** and **−1.6**. The solution
   itself is unaffected; quantitative output is wrong. Source:
   [`ElasticVarForm.cpp`](../src/polyfem/varforms/ElasticVarForm.cpp),
   `saved_solution_kinematics`, reviewed lines 453–466. Priority P2.
   *Repaired 2026-09-20 (RBR-01, explicit `OutputTimePhase`); see the
   [RB-04 record](rb-04-validation.md#rbr-01--explicit-output-time-state-2026-09-20).*
2. **RBR-02 / RB-05: an overflowing item estimate can bypass the resource
   budget.** The actual toolkit count/check methods accept an AABB covering
   `4194304 × 2097152 × 2097152 = 2^64` cells with `max_cell_items=100`,
   because the `size_t` product wraps to zero. A `1024^3` control correctly
   throws. The probe performs no insertion or large allocation. Source:
   companion `src/ipc/broad_phase/hash_grid.cpp`, `count_cell_items` lines
   75–80 and the category sum in `check_cell_item_budget` lines 92–93.
   Priority P2; this is an extreme-input containment defect, not evidence
   that ordinary scenes commonly reach this count.
   *Repaired 2026-09-20 (RBR-02, checked `CheckedCount` arithmetic, named
   refusal of unrepresentable grids and non-finite input before any
   conversion; toolkit `a28de2db`); see the
   [RB-05 record](rb-05-validation.md#rbr-02--checked-arithmetic-in-the-broad-phase-resource-accounting-2026-09-20).*
3. **RBR-03 / RB-07: enabling a budget retains every full-space iterate.**
   `ALSolver::solve_al` uses an ever-growing `vector<VectorXd> carried`,
   including with a pass cap alone and `stagnation_window=0`. Only the
   previous iterate and the configured window are needed. Storage is
   `8 × ndof × (passes+1)` bytes of vector data, in addition to the compact
   scalar pass history. This is a direct source finding, not a measured
   out-of-memory run. Source: [`ALSolver.cpp`](../src/polyfem/solver/ALSolver.cpp),
   reviewed lines 414–416 and 503–508. Priority P2 for large opt-in runs.
   It is separate from RB-24's CCD/thread memory investigation.
4. **RBR-04 / RB-21: the documented unsupported improved-max combination is
   reachable.** The RB-21 record acknowledges heterogeneous-parent seam
   discontinuity with improved-max corrections, but says this formulation
   is never used by semi-implicit mode. The public options can enable it:
   `use_convergent_formulation=true`, `use_improved_max_operator=true`,
   `use_physical_barrier=false`, with `barrier_stiffness="semi_implicit"`.
   The constructor rejects physical-barrier and shape-derivative modes but
   does not reject improved-max. Positive-only coefficient averaging is
   still used when duplicate-removal contributions are negative. The
   dispatch/guard gap is source-confirmed; this review did **not** complete
   a new end-to-end seam reproduction. The handoff requires that check
   before adding a narrow supported-mode guard or proposing a model change.
   Source: [`BarrierContactForm.cpp`](../src/polyfem/solver/forms/BarrierContactForm.cpp),
   constructor, `coefficient_keys`, `assign_collision_stiffness`;
   [`NonlinearElasticVarForm.cpp`](../src/polyfem/varforms/NonlinearElasticVarForm.cpp),
   arguments forwarded to `SolveData::init`. Priority P2, conditional on
   selecting this nondefault combination.

The four numerical defects from the **September 12 review were already
repaired**; they are not new findings here. The reviewed implementation
includes the bounded-energy line-search correction, progressing soft
restarts, curvature-overflow handling, and representable parent averaging
with its final weighted-product guard. See the
[existing follow-up](rb-review-followup-20260912.md).

## What “completed” means for each item

“Appropriate” below means the mechanism and evidence support the stated
bounded contract; it is not an exhaustive proof or general physical
certification. Research closure and explicit policy decisions are assessed
as such, rather than as missing implementations.

| Item | Verdict on the final solution | Important qualification |
| --- | --- | --- |
| [RB-01](rb-01-validation.md) | Appropriate: eliminate shared position-only contact-cache ownership and rebuild with the correct lifecycle. | Applies to fixed and adaptive contact too. RB-06 extends the repair to adhesion/smooth contact. |
| [RB-02](rb-02-validation.md) | Appropriate investigation and regression contract; original six counterexamples were addressed by later items. | Closure is not an independent physical validation of the retained adaptive law. Keep both the parent-law regression and the old stencil-law control. |
| [RB-03](rb-03-validation.md) | Exact selector/permutation/obstacle indexing repair is correct. Interpolated-stencil condensation is a coherent selected approximation. | Local parent-block condensation is not the full assembled compliance; the gap-normalized directional fallback is a different approximation, not an algebraically equivalent inverse. |
| [RB-04](rb-04-validation.md) | Observational accounting, private snapshots and unavailable-value handling are appropriate. Kinematics repair was incomplete: RBR-01 (repaired 2026-09-20). | Right-endpoint work estimates are not exact trajectory integrals. `physical_balance_pass` is a force-residual check, not an energy or engineering-accuracy certificate. |
| [RB-05](rb-05-validation.md) | Preallocation limits, named failures, cache clearing and exit status 3 are the right containment design. Arithmetic needs RBR-02. | Limits bound selected intermediates, not total process RSS. Unsupported broad phases must never silently truncate or pretend to enforce a limit. |
| [RB-06](rb-06-validation.md) | Appropriate transaction around the nonlinear elastic attempt, with deep state restoration and control-solve checks. | The documented solve-attempt boundary is narrower than arbitrary application rollback. An output failure after acceptance is a publication failure; automatic retry is not implemented. |
| [RB-07](rb-07-validation.md) | Pass cap and combined progress signals are appropriate opt-in controls. Correct RBR-03's storage growth. | Still off by default. A stagnation-only budget cannot guarantee termination when a bad configuration continues moving; the pass cap supplies that bound. A production default remains a separate decision. |
| RB-08 | Appropriate explicit decision to leave automatic subdivision/retry off. | No retry implementation is claimed. Rollback remains valuable independently of retry. |
| [RB-09](rb-09-validation.md) | Appropriate benchmark decomposition and honest characterization. | Unit-conversion test T7 failed as declared. Gap/increment sensitivity and finite-lag residuals remain limits; warning about a finding is not repairing it. |
| [RB-10](rb-10-validation.md) | Realized-normal-force lag is a coherent frozen-friction policy; the default change was an explicit decision. | Two lag iterations do not establish a fixed point. Recorded friction runs still fail the strict 1e-6 physical residual flag by approximately 0.2–0.3% in the free residual. Do not describe all accepted steps as friction-converged. |
| [RB-11](rb-11-validation.md) | Named early validation, per-body data binding, transient linear repairs and paired Ogden term validation address concrete defects. | Sampled material/mesh envelope is not universal validity. Preserve valid incompressible/shear-only limits and distinguish FE locking from a contact bug. |
| [RB-12](rb-12-validation.md) | Provenance, actual-executable checks and explicit CI classification are appropriate. | Historical passing lanes do not prove current all-platform parity. Threaded friction sensitivity and known CI groups were documented/transferred, not eliminated. |
| [RB-13](rb-13-validation.md) | Correctly closed as analysis with conditional bounds; no replacement coefficient law installed. | An exact result for the stated frozen positive-definite system is not a guaranteed bound for a local nonlinear estimator. |
| [RB-14](rb-14-validation.md) | Retaining the production law was justified by counterexamples and incomplete estimator coverage. | Empirical holdout coverage and local/physical neighborhoods did not establish global compliance or force-demand bounds. |
| [RB-15](rb-15-validation.md) | Frozen parent identity is a well-supported answer to the measured nearest-feature coefficient discontinuity. | This supports RB-20/21's selected formulation; it does not prove every seam, weighting formulation or topology change is smooth. |
| [RB-16](rb-16-validation.md) | Appropriate controller/timing characterization; retained limits and targeted later fixes are reasonable. | Global gap-band feedback is still a controller, not an enforcement guarantee for every contact. |
| [RB-17](rb-17-validation.md) | Retiring a candidate that failed the coupled gate was the correct outcome. | Do not revive the discarded per-contact band law as though it were a completed production improvement. |
| [RB-18](rb-18-quick-fixes.md) | Positive-reference statistics, dimensional normalization, restart handling and explicit arithmetic failure address their counterexamples. | Absolute/global-curvature fallback and batch floor/cap remain selected heuristics. F6's trim-following friction is a retained comparison option, superseded as default by RB-10. |
| [RB-19](rb-19-line-search-roundoff.md) | The final bounded-energy plus decreasing-gradient fallback is appropriate for the observed roundoff stall. | Port the corrected implementation, not its earlier small-gradient-only variant. It is shared solver behavior and requires nonconvex rejection controls. |
| [RB-20](rb-20-force-continuation.md) | Continuing realized parent coefficients removes the measured fixed-coordinate refresh drift. | It does not conserve physical energy or make all forces invariant to geometry, contact weights, global trim, new contacts or changes in the problem. |
| [RB-21](rb-21-parent-keyed-kappa.md) | Parent provenance and stable weighted means are appropriate in the selected default formulation. Address the exposed-mode gap RBR-04. | The positive-parent sum identity does not extend automatically to improved-max negative corrections. |
| [RB-22](rb-22-validation.md) | Complete-or-refuse collision extraction, dimension guards and the selected Q2 DOF proxy address the actual crash/partial-surface defect. | A piecewise planar Q2 collision proxy is not the exact curved surface. RB-23 separately repairs the Q3 basis. |
| [RB-23](rb-23-validation.md) | Correct repair at the local-to-global enumeration: match edge, face and cell frames to the existing reference basis. Named refusal of unsupported orders is appropriate. | Conformity, polynomial reproduction and interpolation convergence support this much more strongly than scene completion alone. Mixed-order stitching and Q4+ basis generation remain unimplemented by decision; no new physical-accuracy claim. |
| [RB-24](rb-24-validation.md) | Correct diagnosis of transient Tight-Inclusion BFS queues rather than a contact-form leak. Adopting upstream's bucket DFS at the next dependency sync is the appropriate selected remedy. | Closed investigation, **not a deployed memory repair**. The fork still pins 1.0.6. The 1.1.0 candidate can change capped-query TOIs; five identical single-thread smoke endpoints do not prove universal identity or bounded total memory. |

### Additional assessment of RB-23 and RB-24

RB-23's repair preserves the basis functions and geometric map and corrects
which global nodes their local indices reference. The vertical-edge direction
fix and explicit face/cell frames agree with the generator's enumeration.
The new tests exercise random nodal trace continuity, the full tensor
polynomial space on affine cells, and total-degree polynomials on non-affine
trilinear cells. This distinction is mathematically appropriate. The stored
validation log confirms **12,080 assertions / 7 cases** and unchanged
Q1/Q2/tetrahedral smoke endpoints. Current upstream still has the old
directions/arbitrary face frames. This is a high-priority upstream basis
correctness fix even when contact is disabled (UP-04).

RB-24's allocation traces and captured capped queries support the diagnosis.
Its saved A/B results independently distinguish single-thread identical
solutions from small threaded differences. The upstream IPC recipe already
selects Tight-Inclusion **1.1.0**, while the fork recipe and effective cached
headers select the older BFS route. Thus the primary code remedy already
exists upstream; the needed fork action is a validated dependency sync
(UP-12), respecting the user's explicit decision to wait for that sync.
The recorded probe RSS reduction, approximately **2,387 to 578 MB at 18
threads**, is a measurement on one configuration, not a general memory
bound. Do not infer a universal O(depth) bound for all bucket queues from
that experiment, or equate RB-05's broad-phase budget with narrow-phase CCD
memory containment. Upstream could benefit from the compact adversarial
queries and reproducible benchmark evidence, rather than a duplicate fix.

## Which findings belong upstream?

**Yes—several are general correctness defects, independent of barrier
stiffness. Others are useful variable-stiffness research/API contributions
and should be proposed separately.** Define the modes explicitly:

- **Fixed:** one prescribed scalar barrier coefficient.
- **Global adaptive:** one scalar coefficient updated by the existing adaptive method.
- **Per-contact variable:** spatially varying coefficients plus the fork's
  coefficient lifecycle/trim controller. Upstream IPC having a
  `semi_implicit_stiffness` utility does not mean upstream PolyFEM already
  implements this entire fork model.

| Finding/change | Fixed | Global adaptive | Per-contact variable | Upstream destination and recommendation |
| --- | --- | --- | --- | --- |
| RB-01 cache ownership; RB-05 stale swept state; RB-06 equivalent form repairs | Yes | Yes | Yes | PolyFEM: high-priority small correctness patches (UP-01). |
| RB-04 endpoint velocity/acceleration; RB-11 linear solve/force output | Yes / contact independent | Yes | Yes | PolyFEM: high priority after RBR-01; separate independent repairs (UP-02). |
| RB-11 geometry/material/file validation, Ogden lists; RB-12 Debug fixes | Yes / often no contact required | Yes | Yes | PolyFEM: small coherent patches with public failing inputs (UP-03). |
| RB-22 complete collision surfaces; RB-23 Q3 basis enumeration and unsupported-order guards | Yes / RB-23 also contact independent | Yes | Yes | PolyFEM: high priority; separate basis correctness, extraction safety and proxy-default changes (UP-04). |
| RB-05 bounded broad phase and failure cleanup | Yes | Yes | Yes | IPC Toolkit API plus PolyFEM integration, after RBR-02 (UP-05). Automatic limit values are application policy. |
| RB-19 roundoff fallback | Yes / contact independent | Yes | Yes | PolySolve: propose with nonconvex and roundoff controls (UP-06). |
| RB-06 rollback; RB-07 opt-in AL budget | Yes | Yes | Yes | PolyFEM: generally useful, larger integration; RBR-03 first (UP-07). AL here handles prescribed BC feasibility, not an alternative contact law. |
| RB-04/09 physical diagnostics; RB-10 lag mismatch; RB-12 provenance | Yes | Yes | Yes | PolyFEM: opt-in observability, tests and benchmark evidence (UP-08). |
| RB-09 absolute CCD extra-clearance cap and tolerance scaling | Yes, numerical path | Yes, numerical path/controller response | Yes, observed controller amplification | IPC Toolkit and PolyFEM: issue/reproducer and explicit units contract before changing defaults (UP-09). |
| RB-03 stiffness-map contract, RB-18 coefficient guards, RB-20 continuation, RB-21 parent identity | Not required for one fixed scalar | Not required merely because one global scalar changes | Yes | IPC Toolkit and PolyFEM: separate optional extension/design discussion (UP-10), not a universal bug-fix bundle. |
| RB-09 accuracy decomposition; RB-11 locking/material envelope; RB-13–17 negative findings | Useful comparison evidence | Useful comparison evidence | Useful comparison evidence | Public benchmark/documentation contribution (UP-11); not new production models. |
| RB-24 Tight-Inclusion memory finding | Yes | Yes | Yes | Main remedy already upstream; validate adoption in this fork at the next dependency sync (UP-12). Offer measurements/queries upstream. |

In particular, do **not** upstream the complete fork by merging its branches
wholesale. Current upstream PolySolve has unrelated newer work absent from
the fork. Reconstruct each small patch on current upstream and carry its
regression. An issue, draft patch and submitted PR are different states;
none was submitted to upstream by this review.

## Evidence, versions and review limits

Review began on clean PolyFEM `a6d70bd49` (implementation `6a553447b`), IPC
`c24d803e6b175d71a610f3ec85bd33c042e2be0d`, PolySolve
`bce32a39a2c8f0a64cb8ffa85b89f0ee773df0ec`. `PolyFEM_bin --build_info`
confirmed those effective dependencies via the local CPM overrides and
implementation revision `6a553447b`. The HEAD difference was the RB-07
validation document only. Later independent RB-23/RB-24 publications
advanced the shared PolyFEM checkout to `aebe83159`; after the user expanded
scope, their records, source diffs, relevant regression code and saved result
files were reviewed at that revision. None of the four target source
mechanisms above was repaired by that intervening work.

The following upstream refs were freshly fetched/read on September 20:

- [PolyFEM `6c9e7a390`](https://github.com/polyfem/polyfem/commit/6c9e7a39063e844d86e2d6938329efab98dd506b).
- [IPC Toolkit `869e489e`](https://github.com/ipc-sim/ipc-toolkit/commit/869e489e8db8a964897c797655418abf7bf9103e).
- [PolySolve `da4e7feb`](https://github.com/polyfem/polysolve/commit/da4e7feb27af94b722015c774c501c2ace4e33d2).

Direct upstream source checks still show function-static contact caches,
the nonlinear save-before-advance/old-kinematics combination, linear inertia
construction before integrator initialization, IPC's absolute `1e-4`
extra-clearance cap, PolyFEM's `F0 * L^(dim/2)` L2 rescaling, and absence of
the fork's Armijo roundoff fallback. The upstream plan links the exact
source entry points; no current upstream binary was built in this review.

Live checks before the plans-only continuation:

- `[contact_cache],[resource_containment],[al_budget],[rollback]`: **21
  cases / 2,765 assertions passed**. Log in parent-workspace
  `outputs/rb-review-20260920/lifecycle/selected-tests.log`.
- `[contact_stiffness_mapping],[semi_implicit_coefficients],[friction_lag],[kappa_continuity]`:
  **19 cases / 561 assertions passed**, existing unit executable, seed 1.
- Actual-library kinematics counterexample:
  `outputs/rb-review/20260920T135404Z/kinematics_probe.json`, source,
  build commands and provenance beside it.
- Actual-toolkit budget counterexample:
  `outputs/rb-review-20260920/lifecycle/count-result.txt`, source and build
  commands beside it. The source deliberately calls only the count/check
  methods, avoiding the enormous insertion the guard is meant to prevent.

Passing the existing selections alongside these counterexamples demonstrates
coverage gaps. This review did not rerun every historical scene matrix,
full suites, HDA tests or all platform lanes; their earlier counts remain
dated evidence in the individual records. No Teseo/private scene ran.
No default, solver source, toolkit source, test expectation or golden was
changed. The proposed follow-ups remain unimplemented.
