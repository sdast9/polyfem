# RB-11 Stage 1 / Stage 2 review and completion handoff

**2026-09-14 update:** this completion sequence was carried out. The
[follow-up review](rb-11-followup-review-20260914.md) verifies the completed
matrix and identifies two remaining bounded repairs. Continue from that
review; retain the original findings below as historical evidence.

Reviewed 2026-09-13, after the 20:30 envelope handoff. This is a review of
published Stage 1 and the incoming, unpublished Stage 2 work on `ce7c88b4b`.
It does not implement or approve the Stage 2 solver patch. Read this before
following the older envelope handoff in [the validation record](rb-11-validation.md).

## Verdict

**Stage 1 looks correct within its documented input-validation scope.** Its
previous review findings were corrected in `75b4d284d`. The saved 93-case
verification matrix and affected/full-suite logs agree with the record. Fresh
checks of the 2D material domain, zero/non-unit fibre inputs, nonmanifold
topology, global scalar binding on unequal bodies, and the permuted-file
negative control pass (8/8); the acceptance-oracle self-test also passes.
No new Stage 1 defect was reproduced in this review. This is not a claim that
all material laws, future expression values, remeshing, or physical accuracy
are validated. The full suite still has the documented three failing Catch
cases/four assertions, attributed to pre-existing behavior.

**Stage 2 has a useful experimental design, but needs a focused correction
before continuing.** Keep the constitutive checks, synthetic fixtures, exact
affine/homogeneous controls, and saved results. Correct the newly reproduced
quasistatic force-output defect, recover the actual incomplete matrix, and
establish the accuracy limits of the references. Do not restart the whole
investigation or tune the solver/element/material defaults to make every
characterization target green. A documented limit is an acceptable result of
this stage.

## Evidence and exact incoming state

Local review evidence, relative to the parent workspace:
`outputs/rb-11/20260914T003829Z-stage-review/`. It contains `identity.json`,
the incoming tracked diff, scripts, fresh probe inputs/logs/results, and a
read-only reconstruction of the existing matrix.

- PolyFEM `ce7c88b4b5f745443f629dee3b07ed588dffa865`; IPC `bb795446`;
  PolySolve `ee5b296a6`. The effective build uses both local companion sources.
- Incoming solver SHA-256 `e5cf18fdb25e34a945d9a7a2468a7877ecea6c91b9458c6e63dbf52cf34e245b`;
  test executable `f247c62a2836c5063ea0ddfb0f43ed20380fb20f5d8bc3c0e4c1cdffa45ec2ac`.
- The envelope's frozen solver is a different artifact:
  `452e6244132285a39b8e565028195ad7a26767ad1a619f8b1e944c3ca4e0e1ad`.
  Preserve it and its existing `identity.txt`; do not overwrite it with the
  later Ogden/output repair build. Save each later candidate under a new name
  and record which cases used which candidate.
- This review's numerical reruns use the incoming built executables; it does
  not claim to have rebuilt them or to have rerun the full suite.
- Fresh `[input_validation],[rb11_envelope]`: **23 cases / 593 assertions
  passed**, exit 0 (`focused-tests.log`). This includes the 5 envelope cases
  / 423 assertions and the current Ogden input test. The roughly 4.5-minute
  run was active; a sampled stack was in repeated fixture/schema setup.
  Do not mistake quiet fixture initialization for a stalled nonlinear solve
  or rerun this selection repeatedly before the final candidate is ready.

## 1. Fix the newly reproduced quasistatic force-output error

The linear solve now correctly omits mass in quasistatics, but its output
path still assumes a dynamic, acceleration-scaled objective.

**Reproduction:** take the existing `locking-nu0.3-P1-h0.5-linear` fixture,
enable `output/paraview/options/forces`, and compare constant-load schedules
`dt=1, time_steps=1` and `dt=.5, time_steps=2`, both quasistatic and ending
at time 1. Both return 0. The solution arrays are exactly identical, but:

| Output maximum absolute value | dt = 1 | dt = .5 |
| --- | ---: | ---: |
| `elastic_forces` | 43.1016962262 | 172.406784905 |
| `body_forces` | 1.3 | 5.2 |
| `inertia_forces` | .231465611677 | .925862446706 |

The elastic/body forces increase by exactly four, and inertia is nonzero
despite being absent from the solved quasistatic equations. Script and
results: `probe_quasistatic_output.py`, `quasistatic-output-probe.json`,
`qs-dt1/`, `qs-dt0.5/` in the review evidence.

**Cause:** `LinearElasticVarForm::init_linear_solve` leaves elastic/body form
weights at 1 for quasistatics, but retains the time integrator and an enabled
`InertiaForm`. `LinearElasticVarForm::output_fields` passes all three to
`ElasticVarForm::elastic_output_fields`, which divides every force by
`time_integrator->acceleration_scaling()` (dt squared for implicit Euler).
The current matrix uses dt=1 and does not request forces, so it cannot catch
either problem.

**Required repair:** make the force conversion reflect the objective actually
solved: unit scaling and zero/absent inertial contribution for quasistatics,
the solved step's acceleration scaling for dynamics. Preserve scheduled
velocity/acceleration output deliberately; passing a null integrator merely
to fix force scaling must not accidentally drop those fields. Check the
per-step form-update lifecycle too: the new time-dependent path must not
export a constructor-time body force or stale cached `x_tilde` on later
steps. Keep this a bounded repair to the now-working linear time path.

Required regressions, beyond an exit-0 test:

1. A true transient run with nontrivial motion, at dt other than 1, completes
   and satisfies the assembled discrete equation (for implicit Euler,
   `(dt^2 K + M) u = dt^2 f + M x_tilde`, with BC elimination accounted for).
2. A quasistatic scheduled solve matches the same static load/BC problem,
   including nonzero prescribed values and more than one step.
3. The two schedules above give matching physical elastic/body forces and
   zero/absent quasistatic inertia. Time-dependent load output follows the
   current step. Retain the transient initialization-order crash regression.

These defects are in the Stage 2 working patch/path, not evidence against
Stage 1's input-validation changes. The Ogden term-list repair is a useful
separate input correction; retain its tests and check heterogeneous/composite
dispatch before publishing. Report unequal term counts across bodies as an
implementation limitation, not physically invalid material behavior.

## 2. Correct the run inventory and recover without destroying provenance

The handoff's statements that all 109 runs completed and only ten linear
records are missing are incorrect. The actual `matrix/` contains:

| State | Count | Action |
| --- | ---: | --- |
| `run.json` present, exit 0, output present | 51 | Reuse after verifying identity/input hashes |
| Output and linear `solver_info` present, but no `run.json` | 19 | Parse existing output; label lost process exit/wall time honestly, or rerun these cheap cases in a new candidate directory for clean provenance |
| No solver output | 39 | These still need execution |
| Declared cases | 109 | Reconcile every one explicitly |

`inventory.json` lists every name. All 19 unrecorded outputs are linear
twins: 10 distorted, 6 homogeneous, 3 locking. The 39 absent outputs include
21 locking and all 18 non-reference thin cases. Nine fine reference outputs
already exist; do not rerun them solely because summary generation failed.

The exception occurred **inside `pool.map(work, ordered)`**, not after the
entire matrix. Iterating the dictionary `{'solver_info': 'Success'}` produces
a string key containing `info`, then `s['info']` raises. The resulting
exception cancels pending work. Guard all three parser expressions at
`envelope.py:517-519`, not only `newton_iterations`.

Separate subprocess execution, parsing, analysis, and verification. Persist
the command, exit/signal/timeout, duration, input hashes and binary identity
before optional JSON parsing. Catch a parsing exception per case, record it,
and continue collecting other results. Handle nonlinear lists, linear
dictionaries, missing/malformed output, and unknown outcomes explicitly.

**Do not use the present same-directory resume blindly.** `main()` rewrites
every `scene.json`, `mesh.msh`, and `case.json` before `run_case()` reuses any
existing `run.json`; the cache checks neither changed inputs nor binary
identity. That can relabel old results as a new protocol. Add a read-only
analysis/resume mode with validated hashes, or preserve the old directory
and create a new candidate directory only for missing/revalidated cases.
Keep mixed-candidate lineage visible in the combined report. Fixing the
output-only defect need not invalidate displacement-only references if
source inspection and a representative comparison establish independence.

The review's `inspect_saved.py` runs no solvers and changes no existing
evidence. It extracts all 70 available outputs, retaining the distinction
between recorded exits and unrecorded exits. Its summary is useful partial
evidence, not a 109-case verification pass.

## 3. Strengthen verification before interpreting the matrix

The new envelope verifier has reintroduced gaps that Stage 1 already fixed:

- `verify([])` returns an empty failure list; unknown stages/case selections
  can therefore claim a vacuous pass. Require a nonempty, explicit manifest
  and reject unknown names. For partial selections, report missing required
  comparisons as not evaluated, not passed.
- `read_last_vtu` accepts whichever step file exists, including step 0.
  Verify the intended endpoint/time via PVD and solver metadata, and require
  the expected nonlinear termination outcome. Output existence is not enough.
- C-N treats a missing iteration count as zero; C-T and C-L2 silently skip
  missing comparison partners. Require those fields for the applicable check.
- Require finite fields and valid sampled `det(F)` on all nonlinear cases,
  matching sample coordinates/array sizes, and a sufficiently close tip sample.
  The current 70 outputs sample their intended beam tip exactly, but the
  runner must enforce it. Report geometric and numerical checks separately.

Give each check a status (pass/fail/not evaluated), measured value, threshold,
and relevant case names. Characterization criteria can fail without making
the data collection invalid. Preserve every original contract mismatch.

## 4. Qualify the references and the mathematical claims

The overall separation is sensible: a homogeneous solution representable
by P1 checks the constitutive/solver path, while bending resolves mesh error.
The 17 material/parameter configurations exercise real assembled energy,
gradient and Hessian on a small P1 fixture. Treat them as sampled 3D
constitutive checks, not all-law/all-regime certification.

The following contract claims need qualification:

- One finest P2 mesh is a **fine comparison**, not demonstrated convergence.
  The `.25 -> .125` comparison can estimate one increment where available;
  it does not bound the `.125` reference's remaining error near
  incompressibility. Finish the existing coarse cases first. Then select
  only the worst near-incompressible and any suspect thin/anisotropic
  reference for one additional h- or p-refinement (same law and load), with
  a declared quantity-of-interest reference tolerance materially below the
  5% adequacy target (for example 1%, explicitly an engineering criterion).
  If that bounded check cannot establish it, report reference uncertainty
  and leave the affected accuracy claim unresolved. Do not start an
  unbounded series of ever-larger solves.
- C-L1's `r >= .95` for every nu is an accuracy target for that resolution,
  not a consequence of P2 interpolation or a correctness requirement for
  the solver. Standard conforming approximations can lose accuracy near
  incompressibility; polynomial order alone supplies no uniform 5% bound.
  See the dependence on the Lame parameter in
  [Mustapha et al.](https://arxiv.org/abs/2407.06831). That source motivates
  checking the reference; its proposed numerical modification is not adopted.
- C-B's pointwise tip bound does not follow from energy minimization.
  Galerkin energy/compliance properties do not order arbitrary point
  displacements. A simple counterexample is `K=I`, `f=(1,0)`, with the
  coarse space spanned by `(1,1)`: `u_h=(.5,.5)` has lower compliance than
  `u=(1,0)`, but its second displacement is larger. The 5% tip rule may
  remain an empirical target, not a theorem.
- C-L2 compares different Poisson ratios, hence different boundary-value
  problems and references. Monotone pointwise tip ratios are a hypothesis,
  not a general invariant; a nonmonotone result alone is not a code defect.
  Separate ordinary coarse bending error from additional volumetric locking.
- `kappa(K)` from the isotropic linear twin describes the reduced
  stress-free NeoHookean tangent on that mesh. It is not the deformed
  tangent at 10% compression, a fibre-reinforced tangent, or a general bound
  on later Newton conditioning. The current anisotropic cases have no
  anisotropic conditioning measurement. The stretched meshes also change
  resolution/DOF count: report a combined aspect/resolution effect.
- C-N's 20 iterations is a declared cost target. The saved nu=.49999,
  n=4 homogeneous run uses **24** and converges; retain that failure as a
  performance observation. It does not require a new line search or stopping
  policy, and is not evidence of physical error by itself.

Recovered examples (not a completed envelope): P1 h=.5, nu=.3 has a tip
ratio **.52610**, P2 h=.25 at the same nu **.99707**. The 10 affine cases
have maximum relative stress error **3.23e-10**. The 12 nonlinear homogeneous
cases have maximum relative error **1.03e-7** against the runner's full
reference stress tensor, below its 1e-6 criterion; the n=4 extreme case
still misses C-N. All 19 available reduced stiffness matrices have positive
minimum eigenvalues. The homogeneous n=2 linear condition number grows
from **50.8** at nu=.3 to **5.70e5** at nu=.49999.

The stopping-tolerance adjustment has supporting evidence, not merely an
assertion: comparing 41 available beam/linear outputs common to protocols 1
and 2 gives a worst tip difference **2.76e-8 relative**. The timed-out extreme
reference is absent from this paired comparison, so keep its original
timeout and avoid claiming the comparison certifies that case. The separate
roundoff-floor probe is useful. Do not further relax production tolerances.

## Bounded completion sequence

1. Preserve the incoming sources/evidence and read the inventory. Correct
   the runner's parsing, durable result records, manifest and verification.
   Add small parser/oracle tests that exercise actual failure modes.
2. Correct the quasistatic output defect and add the linear dynamic/static/
   scheduled/output regressions above. Keep Ogden term validation and test
   it separately. Rebuild and freeze this final candidate once.
3. Reuse the 51 trustworthy recorded results, recover/revalidate the 19
   linear outputs, and run the 39 cases with no output. Preserve candidate
   identities. Run only a bounded set of reference refinements justified
   by the finished table; otherwise label reference accuracy unresolved.
4. Tabulate physical/numerical/geometry/accuracy/cost results separately.
   Record C-L1/C-L2/C-B/C-N misses without changing the physics or their
   original thresholds. State the supported sampled envelope and remaining
   limits. Stage 2 can finish as characterized with limits documented.
5. On the final candidate run `[input_validation]`, `[rb11_envelope]`, the
   affected assembler/form-derivative/material-cache selection, applicable
   time-integration/linear regressions, the 93-case input matrix in verify
   mode, five public smokes, and affected HDA checks. Attribute existing
   full-suite failures; neither regenerate goldens nor expand this into
   unrelated CI/contact repairs. Read `houdini_HDAs/AGENTS.md` before the
   pending tests-only HDA publication and follow its applicable test scope.
6. Update the RB-11 validation record, plan and parent README with the real
   counts, revised interpretation and limits. Publish only validated Stage 2
   code/tests to the PolyFEM fork and the pending HDA test to its own fork.
   Preserve this review as evidence of what needed correction.

No stabilization, constitutive model, element/quadrature default, contact
law, timestep-retry policy, or private scene run is authorized by this review.
