# Review of the completed RBR implementations — 2026-09-21

**Verdict: one reproduced P2 regression in RBR-01; no additional actionable
defect found in RBR-02–RBR-05 within the scope below.** The explicit-phase
design is correct, but its differentiable solve path is missing a phase
transition. The implementation should not be treated as complete for every
transient owner until that path and its regression test are repaired.

This session reviewed code and produced a follow-up plan. No production
source, test, dependency pin, solver setting or physical model was changed.
The original [repair handoff](rb-review-repair-plan-20260920.md) and the
individual validation records remain the history of implementation.

## Reviewed source and verification boundary

- PolyFEM `main`: `303b54cc07867b2e62d7c139857382ad3e013cb4`, clean at
  the start; all changes after the original review commit `92e0d8c20`.
- IPC Toolkit `semi-implicit-stiffness`:
  `482b9eab2f81bbc5ee59586f80ffb041bcd488dd`, clean and equal to the recipe
  pin; reviewed the runtime delta from `c24d803e` and the tests-only
  portability follow-up.
- PolySolve `iteration-callback`:
  `bce32a39a2c8f0a64cb8ffa85b89f0ee773df0ec`, clean and equal to the pin;
  no RBR changes.
- Rebuilt `PolyFEM_bin` and `unit_tests` in the existing macOS arm64
  RelWithDebInfo build. The new build-info record identifies the clean
  PolyFEM revision above and both effective companion revisions. Optimization
  and Python support are enabled; threading is TBB.

Evidence is in the parent workspace's
`outputs/rbr-review/20260921T124409Z/`: `source-provenance.json`,
`build_info.json`, `build.log`, test/probe logs and command records, the
standalone `review_probe.cpp`, and the two differentiable-mode VTU runs.
The probe was compiled and linked using the rebuilt unit-test target's
recipe; it did not replace or modify library code.

This is a source review plus focused local execution. The prior full-suite,
public-smoke and CI results in the implementation records were inspected as
historical evidence, not rerun or counted as fresh passes. No Teseo or
private scene ran. Existing golden-scene failures and the previously stated
model/physical-validation limits remain outside this verdict.

## Finding: RBR-01 leaves differentiable output at the previous time step

**Priority P2; reproduced; repair pending.**

Source locations at the reviewed revision:

- `src/polyfem/varforms/diff/DifferentiableNonlinearElasticVarForm.cpp`,
  `solve_tensor_nonlinear`, lines 272–297: when `differentiable_mode_` is
  true, this function takes its own path and bypasses the base method that
  sets `CurrentStepBeforeAdvance`.
- The same file, transient `solve_problem`, lines 602–618: the step callback
  and `save_timestep` run before `update_quantities`. RBR-01 added the
  reset to `HistoryHead` after advancement, but no corresponding transition
  before the differentiable solve.
- `src/polyfem/varforms/ElasticVarForm.cpp`, `saved_solution_kinematics`:
  `HistoryHead` returns `v_prev()` and `a_prev()` regardless of the passed
  solution's value, as intended by the new contract.
- `src/polyfem/optimization/AdjointNLProblem.cpp`, `solve_pde`, lines 449
  and 469: both production optimization dispatches call the differentiable
  varform's `solve(..., true)`. This is a reachable application path.

Consequently, the phase stays `HistoryHead` throughout each differentiable
step. The callback, subsolve export and saved timestep can describe the new
displacement with the previous step's kinematics. The callback and timestep
VTU failures were measured; the subsolve consequence follows from the same
`save_subsolve`/shared-export call chain and was not separately exercised.

### Reproduction and distinction from the original defect

The probe uses the same public cube and move-then-hold input as
`tests/test_output_kinematics.cpp`: Neo-Hookean, contact-free quasistatic
Implicit Euler, `dt=.25`, the +z face moves by .1 in x at step 1 and then
holds, with supplied initial x velocity .2. It creates the differentiable
varform via `State::init(args, true, true)` and runs its public
`solve(solution, nullptr, callback, differentiable)` in both modes.

Expected kinematics are evaluated from the saved displacements using
`v_n=(u_n-u_(n-1))/.25`, `a_n=(v_n-v_(n-1))/.25`, including the supplied
initial velocity. No integrator compute method is used as the oracle.

On the moving face:

| Step | Expected vx | Exported vx, differentiable=true | Expected ax | Exported ax, differentiable=true |
| --- | ---: | ---: | ---: | ---: |
| 0 | .2 | .2 | 0 | 0 |
| 1, first move | .4 | .2 | .8 | 0 |
| 2, first hold | 0 | .4 | -1.6 | .8 |
| 3, continued hold | 0 | 0 | 0 | -1.6 |
| 4 | 0 | 0 | 0 | 0 |

`differentiable=false` passes every callback and VTU comparison on the same
varform class. The two runs' saved point-data arrays are byte-identical
except for velocity/acceleration at the affected steps
(`cross-mode-vtu-comparison.json`). With `true`, the probe fails seven assertions: five callback
kinematics comparisons at steps 1–3 and two VTU velocity comparisons at
steps 1–2. Both runs complete their physical solves.

The changed endpoint at step 1 is a **new regression**: before RBR-01, that
endpoint differed from `x_prev()`, so the old helper selected the current
step's differencing rule. The held-step error was the original RBR-01
defect and remains unfixed on this path. The measured finding concerns
exported kinematics; it does not establish an error in the solved
displacement, the internal integrator history, or the adjoint derivatives.

Evidence: `differentiable-output.json`, `vtu-independent-kinematics.json`
(both VTU kinematic fields checked independently), `review-probe-rerun.log`,
`review-probe-run.json` (exit 42, Catch2 failure), and
`differentiable-{false,true}/step_*.vtu`. The first probe build had a probe
constructor type error; the corrected probe linked successfully. The first
execution's wrapper failed while hashing after the run; that log is retained,
and the repeat explicitly persisted the process exit before hashing.

## Item-by-item assessment

| Item and implementation | Correctness assessment | Remaining scope limits |
| --- | --- | --- |
| RBR-01, `766410565`: explicit output time phase | Correct replacement for position equality in the ordinary nonlinear loop; initial state, linear/incompressible/thermo transitions, embedding advancement and ordinary rollback are accounted for. **Incomplete for the differentiable=true branch**, above. | Existing regression exercises the ordinary transient class, so it cannot detect the separate differentiable dispatch. Arbitrary externally supplied solutions in HistoryHead still follow the documented owner contract. |
| RBR-02, IPC `a28de2db`/`482b9eab`, PolyFEM `2bce3eb4a`/`0bf93bdd0` | Correct checked products/sums and unordered-pair counts; overflow remains sticky through joins and exceeds even a SIZE_MAX budget. Grid representability is checked before conversion, hashing is performed in the key type, and the exception remains a containment stop. No new defect found. | Local execution is LP64 macOS. The retained Windows/CI evidence is historical; this review does not certify a new cross-platform run. Disabled memory limits still permit expensive representable builds by design. |
| RBR-03, `82be18314`: bounded carried-state deque | Correct eviction/reference order: read the preceding state for motion, remove entries older than k-W, read that reference for drift, then append. Cap-only retains one state, a window retains at most W+1, and no-budget retains none. Stage exit releases vectors; scalar history and decisions remain unchanged. No new defect found. | Scalar history still grows with pass count intentionally. This is a storage correction, not a new stagnation or retry policy. |
| RBR-04, `fb8377e36`: reject improved max with semi-implicit stiffness | Correct conservative response to the measured signed-weight/positive-parent incompatibility. Both direct construction and input validation reject the combination, while fixed/global-adaptive improved max and the existing supported semi-implicit configuration remain available. No new defect found. | The guard does not validate a signed parent law or establish physical accuracy for area weighting alone; those limits in the implementation record stand. |
| RBR-05, `0028d637a`: fully prescribed solve | Correct affine offset/penalty initialization when the reduced space is empty, with only the empty factorization bypassed. The trivial solve keeps the feasibility check and observable iteration-0 callback/finish sequence. The new independent full-rank general-matrix constraint probe also passes. No new defect found. | The documented legacy/scalar solver paths and homogenization are not certified by this review; they have separate solve dispatches. |

## Fresh execution results

| Check | Result |
| --- | --- |
| Rebuild both PolyFEM executable and unit tests at the reviewed clean sources | Exit 0; effective dependencies match pins |
| Affected PolyFEM selection, seed 1 | Catch2 reports all 94 cases / 10,829 assertions passed; post-run wrapper metadata failure described below |
| IPC Toolkit's actual `test_checked_count.cpp`, linked against the rebuilt effective toolkit, `[checked_count]`, seed 1 | 11 cases / 192 assertions pass |
| Additional general-matrix fully prescribed constraint probe | Pass: zero reduced DOFs, correct affine target, successful trivial solve with zero tolerances and Linf norm |
| Differentiable/ordinary output comparison probe | 2 probe cases total, 577 assertions: 570 pass, 7 fail in differentiable=true output only; the matrix-constraint case passes |

The affected selection is recorded verbatim in `affected-tests-run.json`
and the first line of `affected-tests.log`. It includes the RBR regressions
and neighbouring integration, AL, rollback, contact, input and diagnostic
tests. Expected refusal/failed-lagging fixture messages are not themselves
test failures; the Catch2 summary governs the result.

The broad-selection wrapper encountered the same unavailable
`hashlib.file_digest` call after Catch2 completed, before persisting its
process return code. The recovered record retains the test summary and
binary hash and explicitly leaves the raw test exit unavailable; wrapper
exit was 1. This is not reported as an exit-0 process run. The independent
IPC and repeated review probes have captured process exits.

## Implementation handoff: complete RBR-01's differentiable transition

**One bounded follow-up; no contact-model decision needed.** Keep this under
RBR-01 rather than renumbering the already completed repairs.

1. Read this finding, the RBR-01 section of `docs/rb-04-validation.md`, the
   original repair handoff and the PF/robustness invariants. Recheck current
   sources and effective dependency pins. Reproduce on the current build
   with optimization enabled; retain source, binary, input, commands and
   exit records in a new evidence directory.
2. Add a permanent scene regression that selects the actual differentiable
   transient class, using `State::init(args, true, true)` and its public
   differentiable `solve` overload. Explicitly exercise `true` and `false`;
   calling the ordinary staged `NonlinearElasticTransientVarForm` accessors
   does not cover this override. Reuse/refactor the public move-and-hold
   fixture in `tests/test_output_kinematics.cpp`, without changing its
   physical parameters or production solver tolerances. The standalone
   review probe gives the exact failing construction and oracle.
3. In `DifferentiableNonlinearElasticVarForm::solve_tensor_nonlinear`, set
   `output_time_phase_` to `CurrentStepBeforeAdvance` on the specialized
   differentiable branch before any current-step subsolve output. The base
   branch already owns its transition. Retain `HistoryHead` after integrator
   initialization and after the transient loop's `update_quantities`.
   Audit the callback, timestep save and subsolve saves together. Do not
   reintroduce position equality or move integrator advancement earlier.
4. Keep the differentiable branch's intentional lagging behavior, normal
   branch rollback, derivative-cache timing, force scaling, constraints,
   coefficient updates and convergence logic unchanged. This follow-up
   does not authorize adding a new optimization rollback or retry policy.
5. Assert the initial supplied kinematics, first changed endpoint, held
   endpoint and subsequent hold using the independent Euler rules above.
   Check callback fields and both VTU fields against **saved displacements**,
   not exported previous velocity. Verify a return/changed endpoint too,
   and verify the same final state after the history advance. For a useful
   final-state assertion, finish on a step with nonzero velocity or
   acceleration, rather than only a long zero-motion tail. Add a subsolve
   sequence check if that export includes kinematics. Keep helper-level
   Newmark/BDF2/BDF3 tests and ordinary rollback checks passing.
6. Rebuild `PolyFEM_bin` and `unit_tests`. Run the expanded
   `[output_kinematics]` selection, `[time_integrator],[linear_elastic],
   [fully_prescribed],[rollback]`, and the relevant existing differentiable
   forward/adjoint tests from `tests/test_diff.cpp` (`[opt_gradient]`, including
   a transient fixture such as `material-transient`) and `tests/test_opt.cpp`
   (`[optimization]`) after inspecting their current tags/fixtures. Use a small public input;
   do not invoke unrelated private scenes. Compare the same fixture's
   displacements and forces before/after the patch to establish that only
   output kinematics changed. Retain the failing pre-patch regression.
7. Update the RBR-01 status and its record, this finding's disposition, and
   the UP-02 prerequisite. Publish only named files to the PolyFEM fork
   after checks pass. No IPC/PolySolve pin change is expected.

Acceptance: the presently failing comparisons pass at 1e-12 (or a justified
interpolation-roundoff bound for VTU output), initial/final history-head
exports remain correct, and ordinary/differentiable solved displacements
and unaffected fields remain unchanged. Do not mark the finding resolved
from helper-only tests or a differentiable=false run.

## Upstream consequences

The original split still applies: RBR-01, RBR-02, RBR-03 and RBR-05 address
shared correctness/resource behavior and are relevant independently of
fixed, global-adaptive or per-contact stiffness. Each must still be
reproduced on the then-current upstream before a minimal port. RBR-04 is
specific to the fork's semi-implicit parent-coefficient formulation; it
does not justify disabling upstream's improved max for fixed or globally
scaled barriers. UP-02's nonlinear output contribution must include the
differentiable transition and its test before it is ready to propose.
No upstream issue or pull request was submitted in this review.
