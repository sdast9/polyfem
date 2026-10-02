# Semi-implicit contact across PolyFEM workflows

Assessment date: 2026-10-02. Source baseline: PolyFEM `48f15d26dc1a17ac263cc9a1d511c7577835722f`, IPC Toolkit `1f1b5dbf46f4f69a558effac74c71c1a5a5d4ca4`, PolySolve `43ca2e661069ba3971e16ec4c26969ea1f1d41cc`.

**Conclusion.** The fork has a reusable contact implementation that can support additional workflows. Ordinary elastic forward solves already share much of it. Simulation remeshing needs a substantive integration of contact state and solver lifecycle. Built-in adjoint optimization is explicitly blocked for semi-implicit contact and needs a mathematical differentiation contract as well as code. Thermoelasticity and two-mesh fluid–solid interaction already reuse the contact forms, but that establishes a code path, not validation of the combined method.

This is a source and design assessment, informed by the existing validation records. No new remeshing, inverse-gradient or physical-accuracy experiments were run, and no solver defaults or implementation were changed. The local CMake configuration was inspected, the solver and unit-test targets were rebuilt, and three existing focused tests passed as described below. Earlier validation is cited only within its recorded scope.

**What the completed work makes reusable**

The central integration point is `SolveData::init_forms`. Semi-implicit mode constructs the standard `BarrierContactForm`, supplies an elastic-plus-inertial Hessian, supplies a non-contact energy gradient, and uses global barrier stiffness as a trim multiplier over local coefficients. Contact therefore remains compatible with the existing nonlinear problem, friction, collision detection and time-integration machinery. Here “semi-implicit” describes the contact-coefficient treatment; it does not require replacing the implicit time integrator with an explicit dynamics scheme. [Source: contact assembly][assembly]

The earlier work supplies useful building blocks: consistent collision-to-FEM mappings and interpolated stiffness (RB-03), contact coefficient identity and continuation (RB-20/21), friction coupling (RB-10), high-order collision surfaces (RB-22/23), failed-attempt isolation on the ordinary elasticity path (RB-06), and objective-generation tracking for solver history. These reduce the amount of new infrastructure needed. Their validation does not automatically extend to a different solve loop or changing mesh. See the [robustness plan](robustness-plan.md), [RB-03 contract](rb-03-contract.md), [RB-10 record](rb-10-validation.md), [RB-23 record](rb-23-validation.md), and [objective-generation implementation][generation].

The coefficient is not simply a material constant. It depends on a frozen mechanical Hessian and contact geometry, with bounds, fallback rules, parent identities, endpoint continuation, and a separate trim controller. Refreshes occur at defined solver events. Those dependencies are the main reason remeshing and inverse differentiation need more than forwarding a JSON option. [Source: coefficient lifecycle and model description][coefficient]

**Compatibility assessment**

“Integrated” below means a source path exists; it does not certify every material, mesh, parameter combination or physical regime.

| Workflow | Current assessment | Work needed |
|---|---|---|
| Static, quasistatic and implicit transient elasticity; self-contact and obstacle contact | Integrated core use | Application-specific convergence, gap/force accuracy and mesh/time refinement remain necessary. |
| Linear elastic materials with contact, nonlinear hyperelasticity, multiple elastic materials, pressure loading | Shared elastic contact path | Contact makes even a linear-material problem nonlinear. Check the selected constitutive law, pressure/contact balance and admissible element regime. |
| Frictional elastic contact | Integrated, with a dedicated validation record | Preserve the finite lagging policy and assess dissipation, lag residual and sensitivity to smoothing. |
| Higher-order elements and collision proxies on a fixed mesh | Substantial implementation already exists | Respect supported extraction and basis limits. Q3 hex support is recorded; mixed-order hex stitching and Q4+ remain refused. This is not automatic adaptive remeshing. |
| Remesh the initial geometry before an independent forward solve | Feasible using the ordinary forward path | Rebuild material and boundary assignments and check the new collision surface. There is no evolving simulation history to transfer. |
| Remesh during an elastic simulation | Not ready as a supported combination | ITR build support, legacy-path reconciliation, local contact energies and transfer/reinitialization of contact and time history. |
| Remesh between shape-optimization rounds | The optimization remeshing framework exists | Semi-implicit differentiation remains blocked; remeshing itself does not resolve that blocker. |
| Built-in adjoint material, shape, friction or initial-condition optimization with semi-implicit contact | Explicitly blocked in the normal differentiable elastic path | Define the differentiated model, implement missing coefficient/history sensitivities or a restricted frozen-coefficient mode, and pass gradient tests. |
| External parameter sweeps or derivative-free inverse fitting using forward solves | Feasible without enabling adjoints | A fitting wrapper, repeatability checks, failed-solve handling and an accuracy-controlled objective. Cost grows with parameter count. |
| Thermoelasticity | Contact assembly is reachable | Validate coupled residuals, thermal loading in coefficient calibration, and the separate solver loop. No contact-dependent heat-transfer law is implied. |
| Two-mesh Navier–Stokes FSI with an elastic solid | Solid contact assembly is reachable | Validate solid/fluid coupling and contact together, including lifecycle, interface work and fluid-domain validity. No general fluid–fluid IPC contact is implied. |
| Periodic contact and contact across periodic images | Explicitly rejected | Extend coefficient identities, mappings and assembly over images. Ordinary periodic boundary constraints are a different feature. |
| Homogenization | Depends on route and contact configuration | Legacy forward homogenization can reach shared assembly for ordinary contact; periodic contact is blocked, and differentiable homogenization encounters the derivative restriction. No general semi-implicit homogenization claim is justified. |
| Mixed displacement–pressure incompressible elasticity | Modern factory declines contact for this formulation | A mixed-system integration and an appropriate mechanical stiffness/compliance estimate; a whole saddle-point matrix is not a drop-in stiffness provider. |
| Pure Stokes/Navier–Stokes, operator-split fluids, scalar PDEs | Modern factory declines contact | Mechanical contact belongs on an appropriate moving solid/interface model, not on a scalar or pure-fluid unknown by changing a flag. |
| GCP, physical-barrier option, improved-max operator | Explicitly rejected in semi-implicit mode | Separate formulation work; removing the guards would not supply the missing mathematics. |

The dispatch evidence is in [the formulation factory][factory], [CLI routing][cli], and [contact construction and exclusions][assembly]. High-order status is in [RB-23](rb-23-validation.md) and [CI-04](ci-04-validation.md). The matrix classifies the audited paths rather than claiming an exhaustive audit of every legacy formulation.

**Remeshing: three different tasks**

Preparing a new mesh before a new simulation is the easiest case. Existing forward machinery reconstructs bases, mass, the collision surface and coefficients for that mesh. Mesh-quality improvement may be useful, but it does not by itself fix a constitutive instability or establish physical contact accuracy.

Remeshing between optimization rounds is already implemented separately in `OptState`: MMG remeshing can be triggered periodically or by minimum scaled Jacobian, and each round recreates the optimization problem. The code also recognizes that vertex numbers change: explicit shape-node selections must be replaced by supported geometric selectors, and a shape variable shared across the remeshed and another state is rejected. Those are useful foundations for a future contact-aware shape optimization workflow. This is not the `space.remesh.enabled` simulation path. [Source: optimization remeshing][opt-remesh]

Remeshing during simulation is the substantial integration task:

1. The modern formulation factory returns false when `space.remesh.enabled` is set; the CLI then selects `legacy::State`. The inspected local build has `POLYFEM_WITH_ITR=OFF`, `POLYFEM_WITH_MMG=ON` and `POLYFEM_WITH_OPTIMIZATION=ON`. In the legacy transient loop, requesting remeshing without ITR produces a warning and does not execute remeshing. MMG availability does not imply ITR availability. [Sources: factory][factory], [legacy loop][legacy-loop], [build options][build]
2. Legacy main solves do pass the semi-implicit configuration to shared contact assembly. However, local remeshing relaxation passes the numeric `contact_form->barrier_stiffness()` into the stiffness-mode argument. That selects fixed stiffness; in semi-implicit mode this numeric value represents global trim, not the complete local coefficient field. The constrained projection also constructs a scalar fixed barrier. These are concrete model mismatches requiring design and validation, even if an ITR-enabled build runs. A projection barrier may legitimately differ as a transfer constraint, but its scale and contract must be deliberate. [Sources: local relaxation][local-relax], [projection][projection]
3. The remesher already transfers displacement and time-integrator quantities, rebuilds bases/mass and reconstructs the nonlinear solve. It does not establish a transfer rule for parent-keyed coefficients, continuation, trim history or the fork's full restart state. Its node/DOF assertions and P1 projection bases also argue for a P1 triangle/tetrahedron pilot, not an initial claim of arbitrary high-order support. Per-element material files are explicitly rejected because local patches cannot transfer their original indexing. [Sources: state rebuild][remesh-state], [material guard][material-guard]

A viable design should treat an accepted mesh change as a transaction. Transfer physical state and all required time history; reconstruct boundary/body/material assignments, collision geometry and mappings; rebuild mass and mechanical Hessian providers; invalidate topology-dependent caches; initialize coefficients under a declared rule; rebuild friction lagging; and require a valid converged solve on the new mesh. Failure must retain the previous accepted state. Contact safety must cover the proposed mesh and transfer operation as well as subsequent line searches.

Transfer must preserve the material reference configuration as well as the deformed configuration; replacing the rest mesh with the current deformed surface would change the stress state. A solve after transfer must also use the intended physical time and integrator history rather than accidentally advancing the same time step twice.

Simply copying old coefficient arrays to new vertex IDs is invalid. Even unchanged surface positions can have different area weights, interpolation, mass and mechanical response after remeshing. Re-estimation is the most direct starting policy, but changes in force and energy must be measured. Transferring an effective traction/force field to initialize new coefficients is a possible later design, not a proven equivalent operation. The policy for retaining or resetting global trim must also be explicit.

My recommended progression is a single remesh at a completed quasistatic load increment, followed by dynamic history transfer, friction and repeated changes. In-timestep local refinement is a later step because its acceptance criterion compares energies across mesh operations; changing the coefficient law during that comparison can change the meaning of improvement. The original [In-Timestep Remeshing research](https://zferg.us/research/in-timestep-remeshing/) integrates mesh changes with contact-safe state transfer and a solve on the updated discretization. Its guarantees cannot be assumed for a substituted contact law.

**Inverse problems: the current blocker and the missing derivatives**

`BarrierContactForm` explicitly throws when semi-implicit mode is combined with `enable_shape_derivatives`. The differentiable elastic initializer passes that argument as true whenever it operates in differentiable mode. Consequently the blocker also affects the usual built-in material, friction and initial-condition optimization paths with active semi-implicit contact; it is not limited to an objective whose variable happens to be shape. [Sources: constructor guard][guard], [differentiable initialization][diff-init]

The existing semi-implicit form derivative test deliberately freezes the coefficient snapshot and checks displacement derivatives of that frozen objective. IPC's `stiffness_scale` is likewise documented as constant with respect to current and rest positions. This is appropriate for the inner Newton solve and does not establish derivatives of a full simulation whose coefficients change. [Sources: derivative test][frozen-test], [IPC multiplier contract][ipc-scale], [IPC shape derivative][ipc-shape]

For a fixed contact configuration, write the residual schematically as

\[
R(u,p;s)=R_{\mathrm{mech}}(u,p)
 + \tau\sum_c \kappa_c\,\nabla_u b_c(u,p),
\]

where `p` denotes design parameters, `s` contains coefficient/controller history, `tau` is trim, and `b_c` includes the appropriate geometric and weighting terms. An exact parameter derivative of a coefficient-dependent model contains both the usual geometric/mechanical terms and

\[
\sum_c\left(\tau\,\frac{d\kappa_c}{dp}
 +\kappa_c\,\frac{d\tau}{dp}\right)\nabla_u b_c.
\]

In a trajectory the state updates must also be represented, schematically `s_(n+1) = G_n(s_n, u_n, p)`, along with the mechanics and friction history. This is a chain-rule requirement, not a derivation of the current controller's adjoint. Refreshes inside a solve can require a finer event sequence than one update per time step.

Material calibration is affected even with fixed geometry: changing an elastic parameter changes the Hessian used to choose contact coefficients. Shape optimization additionally changes distances, normals, area weights, mass, collision-to-FEM mappings and coefficient identities. Friction and initial-condition optimization change the trajectory and the states at which coefficients are refreshed. The contact gap and numerical regularization therefore need accuracy controls so that a fit does not compensate for numerical contact effects by shifting inferred material parameters.

The differentiable-simulation authors explicitly chose fixed barrier stiffness because adaptive updates would require differentiating those updates. That supports the distinction above; their results do not certify this fork's semi-implicit controller. [Huang et al., section 4.4](https://arxiv.org/html/2205.13643v4#S4.SS4)

There are four practical development choices:

| Approach | Meaning and limitations |
|---|---|
| Forward-only fitting | Reuse the existing forward solver for a small parameter search or derivative-free optimizer. No adjoint implementation is needed. Repeatability and numerical error still matter, and a large parameter space can be expensive. Finite differences of entire runs are possible but must be checked for controller-branch changes and step-size sensitivity. |
| Semi-implicit warm start for a differentiable target | Use a semi-implicit forward solution as an initial guess, then converge a specified fixed-stiffness contact problem and differentiate that final problem. This could reuse forward-solver robustness without differentiating the trim controller. Its speed benefit is unmeasured, and its gradients describe the final fixed-stiffness model, not the adaptive semi-implicit result. |
| Explicitly frozen coefficient model | Hold a defined coefficient field/schedule, trim and associated contact settings fixed across objective and derivative evaluations. This can support exact derivatives of that restricted model after the required implementation. It does not provide the full adaptive model's gradient. Recomputing coefficients every design iteration while omitting their derivatives instead defines an approximate surrogate method. |
| Differentiated adaptive model | Differentiate the chosen coefficient law and update/history policy, or redesign the updates into a differentiable formulation. Hessian sensitivities, bounds/fallbacks, contact births, feature switches, friction and restart decisions make this the largest task. Smooth replacements change the model and require separate decisions. |

A frozen-coefficient prototype should start with a fixed mesh, frictionless contact and a controlled active set. It needs an explicit rule for newly active contact parents; merely saving the initially active pairs is insufficient. The current cache copies collision sets, so individual stiffness scales can already travel with them, but there is no complete dedicated semi-implicit trim/controller sensitivity history. Extend the cache deliberately and preserve the unprojected physical Jacobian for adjoints rather than differentiating a PSD solver modification. [Sources: differentiation cache][diff-cache], [contact force derivative][contact-diff]

After passing that restricted gradient check, progress to material/load variables, shape, friction and dynamics. Material-specific derivatives are a separate requirement: the existing material-force derivative uses the assembler's Lame-parameter derivatives, so enabling contact differentiation would not automatically expose every HGO, Ogden or active-material parameter for fitting. [Source: material derivative][material-diff]

Remeshing between outer optimization rounds can then reuse `OptState`. Remeshing inside a differentiable trajectory is substantially harder: sensitivities must pass through state transfer and changing DOF spaces, with a defined treatment of discrete mesh decisions. It should not be part of the first inverse implementation.

**Other coupled solves and formulation limits**

Thermoelasticity inherits the nonlinear elastic form, initializes the usual displacement forms and embeds them in a temperature/displacement problem. Two-mesh FSI embeds a `NonlinearElasticTransientVarForm` for the solid and forwards solid contact updates. These are promising existing connections. Their outer nonlinear routines are separate from the ordinary elasticity routine, however; ordinary-path restart, rollback, retry and diagnostic evidence cannot simply be carried over. [Sources: thermoelastic assembly/solve][thermo], [FSI assembly/solve][fsi]

The current coefficient provider uses elastic and inertial curvature, and its calibration gradient has a fixed list of non-contact forms. Coupled thermal or fluid/interface contributions are not automatically part of that estimate. A solid-block estimate may be a deliberate choice, but must be assessed against the coupled residual. For a conservative block system, eliminating auxiliary variables can change effective mechanical stiffness through a Schur complement; a general fluid residual can be nonsymmetric and needs its own reasoning. Do not feed the whole mixed Jacobian into a local elastic-stiffness formula without deriving its meaning.

Pressure loading and ordinary elastic anisotropy are nearer to the current validated route than these mixed problems. Damping and adhesion forms can also coexist at assembly level, but combined-law validation is still needed. Similarly, the material name `IncompressibleOgden` in the elastic factory should not be confused with the separate mixed displacement–pressure formulation that declines contact. [Sources: supported materials and modes][factory], [assembly][assembly]

Periodic image contact, GCP, the physical-barrier option and improved-max remain named exclusions. In particular, improved-max has a recorded signed-weight/coefficient consistency problem; treating it as a switch to enable would discard a deliberate guard. [Source: constructor restrictions][guard]; [RBR-04 context](rb-review-repair-plan-20260920.md).

**Recommended bounded implementation and acceptance stages**

These are proposed future tasks, not completed validation or authorization to change defaults.

| Stage | Bounded deliverable | Acceptance evidence |
|---|---|---|
| A. Establish extension baselines | One small compression/indentation fixture and a matched fixed-mesh reference; explicit supported-mode diagnostics | Record revisions, inputs, effective settings, solved residual, force/reaction, gap, det(F), energy/work and repeatability. Preserve failed cases. |
| B. Remesh once after an accepted load increment | P1 frictionless elastic case on the modern path, or a deliberately audited legacy integration | Deliberate renumbering; no stale IDs; valid surface/CCD; correctly transferred materials/BCs; measured force/energy change; agreement under mesh refinement; failed transfer rolls back. |
| C. Restricted inverse prototype | One scalar parameter and an explicitly frozen coefficient policy | Directional finite differences over a step-size ladder; forward error demonstrably below derivative signal; manufactured-parameter recovery and held-out loading. Re-evaluate the final design with the intended production forward model. |
| D. Dynamic/friction remeshing | Complete integrator history and friction transfer across repeated mesh changes | Momentum and energy/work accounting, frictional dissipation, endpoint residuals, time/mesh refinement, restart continuity and rollback. |
| E. Coupled forward qualification | Separate thermoelastic and two-mesh FSI fixtures | Limiting uncoupled cases, solid/interface force balance, coupled residuals, energy/work transfer where applicable, and controller/restart lifecycle coverage. |
| F. Full adaptive adjoints or in-timestep remeshing | A separately specified coefficient/update/transfer contract | Gradients or mesh-operation acceptance across refreshes, bounds, contact birth/death and friction events; reproducible improvement on held-out problems. |

The existing remeshing and inverse infrastructures make both directions plausible. For immediate inverse exploration, start with a small forward-only fitting experiment. For a solver extension, prioritize the single-remesh stage before full ITR. Develop frozen-coefficient adjoints as a separate controlled branch of work, keeping the distinction between that model and the production adaptive contact law explicit. Existing R3 and trajectory-sensitive accuracy questions remain governed by the [open-items record](open-items-20260929.md); remeshing or optimization should not be presented as their automatic solution.

**Evidence and review limits**

The source trace covers contact assembly, guards and lifecycle, formulation/CLI dispatch, legacy remeshing local relaxation and projection, optimization-round remeshing, derivative interfaces/cache, and the two coupled forward paths described above. Existing test code was inspected, including ordinary optimization tests. Publication checks rebuilt `PolyFEM_bin` and `unit_tests`, then passed **3 test cases / 201 assertions**: `semi-implicit barrier contact form derivatives`, `semi-implicit friction form derivatives`, and `varform factory supports migrated formulations`. The derivative checks concern the existing frozen-coefficient forms, not gradients of the adaptive forward simulation. The pinned pre-commit hook skipped this Markdown-only change, and source-reference/link/whitespace checks passed. No ITR build, remeshing run, full inverse-gradient test, full suite, HDA smoke or all-combinations certification is claimed.

During the assessment the shared checkout advanced to `ddb4579a3105f95c324c0267e7dad18e8af060eb` for pressure-condition time functions and long value strings. Every implementation file in the pinned source references below was checked byte-for-byte against its audited revision and remained unchanged; the newer commit does not alter these contact integration findings.

Local audit evidence is under `contact-integration-analysis-work/evidence/` at the parent workspace root, with repository state, source hashes/excerpts and focused search outputs. Archived simulation evidence was not needed or rerun. Permalinks below pin implementation claims to the audited revisions, even after this report is published.

[assembly]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/solver/SolveData.cpp#L427
[factory]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/varforms/VarFormFactory.cpp#L131
[cli]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/main.cpp#L300
[guard]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/solver/forms/BarrierContactForm.cpp#L256
[coefficient]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/solver/forms/BarrierContactForm.cpp#L580
[generation]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/solver/NLProblem.cpp#L724
[opt-remesh]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/optimization/OptState.cpp#L115
[legacy-loop]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/legacy/state/StateSolveNonlinear.cpp#L78
[build]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/CMakeLists.txt#L71
[local-relax]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/mesh/remesh/wild_remesh/LocalRelaxationData.cpp#L291
[projection]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/mesh/remesh/L2Projection.cpp#L104
[remesh-state]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/legacy/state/StateRemesh.cpp#L159
[material-guard]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/legacy/state/StateInit.cpp#L241
[diff-init]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/varforms/diff/DifferentiableNonlinearElasticVarForm.cpp#L189
[frozen-test]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/tests/test_form_derivatives.cpp#L304
[ipc-scale]: https://github.com/sdast9/ipc-toolkit/blob/1f1b5dbf46f4f69a558effac74c71c1a5a5d4ca4/src/ipc/collisions/normal/normal_collision.hpp#L123
[ipc-shape]: https://github.com/sdast9/ipc-toolkit/blob/1f1b5dbf46f4f69a558effac74c71c1a5a5d4ca4/src/ipc/potentials/normal_potential.cpp#L325
[diff-cache]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/optimization/DiffCache.cpp#L90
[contact-diff]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/optimization/force_derivatives/BarrierContactForceDerivative.cpp#L10
[material-diff]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/optimization/force_derivatives/ElasticForceDerivative.cpp#L118
[thermo]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/varforms/ThermoElasticVarForm.cpp#L690
[fsi]: https://github.com/sdast9/polyfem/blob/48f15d26dc1a17ac263cc9a1d511c7577835722f/src/polyfem/varforms/NavierStokesFSIVarForm.cpp#L455
