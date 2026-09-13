# RB-11 independent review and continuation guidance

Date: 2026-09-13. Scope: review the staged implementation, procedures and saved
results; guide the implementing session. This review does not change solver
sources, rebuild the shared targets, or certify the pending implementation.

## Verdict

**Keep the useful repairs, but correct the implementation and validation before
publishing it. It needs more than the two pending edits and a final rebuild.**
The investigation has a sound foundation: isolated public fixtures, a preserved
baseline binary, real reproductions of crashes and silent scalar misbinding,
positive controls, affected regressions, smoke comparisons, and attribution of
full-suite failures against the baseline. The saved results support that work.

There is also demonstrable scope and validation drift. A new parameter rule
rejects a supported valid 2D input; a required zero-fibre case is still silently
accepted; a new CCD warning is mathematically wrong; and one purported topology
control does not contain the topology it claims. Fix these specific problems.
Do not discard the implementation or expand this into a new contact model.

This guidance supersedes the local `rb-11-validation.md` handoff that describes
the remaining implementation work as only rebuilding the lumped-mass warning
and the changed dispatch test. RB-11 remains **implemented—validation pending**;
the physical-envelope and material-transfer stages remain open.

## Reviewed state and evidence

- Local PolyFEM HEAD: `a5aa87ea0208d0e0ddd89330bc6eb8521e0db556`, `main`,
  with 39 staged files (3,013 insertions, 92 deletions). The source patch was
  preserved before review; SHA-256 of the staged binary diff:
  `1f4c35b35e822885d80321172de120bb6db9b9c909a8f5ddeb73c4e5cb93c388`.
- Effective IPC: `bb795446812a3d3c7b5358c4cdbf26c6bbe48a16`; PolySolve:
  `ee5b296a690ce225fb34f6f375d27f9ea2a16027`; local overrides match the pins.
  RelWithDebInfo, arm64, MISO off. No active build/simulation was found.
- Current solver binary SHA-256:
  `0f6a8e41a09a377abbeac5998d442f3dcd2d0c87bd61968a199f4b5bc145054b`.
  Current test binary:
  `04fed1f8c3e127039c6efc18355e6d3251dbf50e4845c38f351ca3b6c0337e4f`.
  These are different executables, not two names for one binary. The two
  documented final source/test edits postdate these builds.
- Historical evidence inspected: `outputs/rb-11/20260913T140845Z/`.
  `matrix-baseline-v3` is 40/73 and `matrix-candidate-final` is 73/73.
  The latter used **47 named failures, 24 accepted, 2 notices**. The current
  case table has **46, 24, 3** after P2 lumping became a notice; that revised
  table has not yet produced the saved 73/73. Keep these protocols distinct.
- Saved unit logs confirm 15 cases / 121 assertions for `[input_validation]`,
  39 / 1,694 for the affected selection, and **321 cases, 317 passed,
  4 failed; 6 failed assertions** for the full suite. Grouped scene tests
  explain why six failed checks appear in four Catch cases. The recorded
  baseline comparisons and retained failures are useful evidence, not a
  reason to regenerate goldens.
- Fresh independent evidence: `outputs/rb-11/20260913T191847Z-review/`,
  including `identity.json`, `incoming-staged.patch`, `review_probe.py`,
  `probe-results.json`, inputs/hashes, commands and full logs. Five small
  configurations were each run on the preserved baseline and current solver
  (ten runs, single-threaded, 35-second per-run timeout; none timed out).
  These runs exercise the already-built code, not the two unbuilt edits.

## Corrections required before implementation publication

### 1. Make material-domain rules specific to the law and dimension

**Reproduced new regression:** an eight-triangle 2D square with
`MaterialSum(NeoHookean, HGODispersion)`, fibre `[0,1]`, `k1=1e4`, `k2=5`,
and `kappa=0.4` completes two steps on the baseline (exit 0, three VTUs),
but RB-11 rejects it at startup (exit 1, no VTU). The neighbouring `kappa=0.3`
case completes on both binaries.

`src/polyfem/assembler/Assembler.cpp:365` hardcodes `[0,1/3]`. The existing
`HGODispersion.hpp` law explicitly uses dimension `d`,
`E4 = kappa I1 + (1-d*kappa) I4 - 1`, and documents `[0,1/d]`.
The 2D aligned-to-isotropic interval therefore extends to `1/2`.

Use the existing law's dimension-dependent domain and error message. Add 2D
controls at 0.4 and 0.5, a 2D negative case above 0.5, and retain the 3D
one-third endpoint/negative controls. Cover a plain material and its composite
dispatch. This restores an existing contract; it does not select a new model.

More generally, exported parameter-name suffixes are not a complete material
validation API. Keep component identity when pairing lambda and mu; the current
`by_base` map overwrites values across MaterialSum children. Do not infer
universal domains from names, or reject a valid simulation because a tangent
has free modes, buckles or softens. Barycentre-at-`t0` checks are only sampled
startup checks for spatial/time expressions, not a guarantee at quadrature
points or later times. Bound the claim and identify unsupported laws explicitly.

### 2. Cover zero directions outside the VTK-file reader

**Reproduced unclosed required case:** replacing the unit constant fibre of
the existing NeoHookean + HGOFiber control with `[0,0,0]` still completes
both tensile steps and writes three VTUs on RB-11, just as on the baseline.
The unit-vector control also passes. File-based zero vectors are caught, but
the constant/expression path is not: the new parameter loop only checks that
the three exported components are finite. Three zeros satisfy that test.

Validate the evaluated vector's dimension, finiteness and nonzero norm before
normalization/use, with element, material/family, location/time and source
context. Exercise constant, expression and file representations, including a
valid non-unit vector whose existing normalization semantics must be retained.
Use a valid unit-vector neighbour for every negative case. Do not replace an
invalid fibre with a default direction. Treat rotation-matrix inputs according
to their separate contract, not as direction vectors.

### 3. Correct the CCD notice; preserve the CCD algorithm

**Reproduced false guidance:** on the existing contact control with
`dhat=1e-5 m`, RB-11 says contact can never enter the barrier band and suggests
metres or a larger `dhat`. The input already uses metres. Both binaries
complete, with the same logged distances. The candidate's physical
diagnostics report 20 active collisions at each accepted endpoint, positive
barrier energies, and final gaps `7.300725e-6` to `8.937688e-6 m`, all
**inside** the `1e-5 m` band.

The actual IPC expression in `tight_inclusion_ccd.cpp:43-48` is
`d_min + min((1-c)*(initial_distance-d_min), 1e-4)`, where `c` is the
conservative rescaling. `1e-4` is a **cap on additional clearance**, not a
fixed minimum gap. A small-TOI fallback can also use `d_min` directly.

Remove the impossible-contact assertion and deterministic first-iterate-gap
claims from `NonlinearElasticVarForm.cpp:739-756` and matching documentation.
Retain the measured RB-09 unit-sensitivity observation with its actual limits;
do not prescribe a larger physical contact band from this ratio alone. Name
the active CCD strategy when reporting a strategy-specific constant.
Keep CCD and the trial-displacement cap unchanged. Similarly, qualify the
`F0*L^1.5` explanation as the 3D L2 case; `NLProblem::grad_norm_rescaling`
has different rules for 2D, Euclidean and Linf norms.

### 4. Repair the test inputs and acceptance checks before relying on their count

**Reproduced fixture defect:** `g1-obstacle-tjunction` and the matching Catch
section use faces `(0,1,2), (0,2,3), (0,4,1), (1,4,5)`.
Edge `(0,1)` has two incident faces; the maximum edge degree is two.
This is a bent open surface, not the advertised edge shared by three faces.
Keep it as an open-surface control and add a real three-face edge with an
independent incidence assertion. Record whether the intended contact feature
is actually exercised, separately from successful mesh loading.

`run_matrix.py::expect_match` also accepts synthetic records with no saved
steps for `accepted`, no notice for `accepted_with_notice`, and an unrelated
early exception for `named_failure`. The review exercised those predicates;
all three returned true. This does **not** show that the saved valid runs
were incomplete: their summaries have all three intended VTUs. It shows the
runner does not enforce what the procedure promises.

Give each case an explicit expected error/context or notice and intended
step/time count. Check those fields, actual accepted outputs and required
diagnostics; fail verification on missing checks or zero selected cases.
An investigation mode may return zero while collecting failures, but a
verification mode must return nonzero for a mismatch. Assert the validation
oracle rejects deliberately incomplete/unrelated results.

Strengthen per-element transfer checks with unique values/directions within
each body, unequal body sizes and multiple geometry meshes. The current
constant-per-body top/bottom field test establishes the particular global vs
body-local repair, not all element ordering. Use an independent source-row
mapping through export; a permuted same-body file should fail that check.

## Keep the scope and remaining work explicit

- Length-based global/body-local binding is reasonable for the demonstrated
  fixed-mesh compatibility problem, where the two lengths distinguish the
  established contracts. It does not identify an arbitrary permutation or
  prove remeshing transfer. The record itself says local remeshing patches
  can fail the new length rule. Audit those call paths with persistent element
  identities and a small transfer fixture before claiming that lifecycle
  supported; if it is unsupported, diagnose that configuration before producing
  misleading accepted results. Do not silently substitute local IDs.
- Preserve the rejection of a body without a material and the explicit valid
  dispatch control. Updating that test's former warning-only expectation is
  justified by the reproduced missing-material failure.
- A lumped-mass warning can preserve compatibility without choosing new
  discretization defaults. A matching golf-ball golden does **not** establish
  physical validity of an indefinite inertia operator. Distinguish negative,
  zero and nonfinite row sums and structural obstacle zeros. Say that some
  higher-order bases have nonpositive row sums; the claim that row-sum lumping
  is positive *only* for linear elements is too broad. Keep this regime
  explicitly outside the measured physical envelope.
- The HDA remesh parameter round-trip is not an execution of remeshing with
  per-element fibres/scalars. The non-first-subdomain scalar HDA regression,
  constitutive derivative/objectivity checks, and nearly incompressible,
  anisotropic, thin/distorted-element reference comparisons are still required
  work or explicitly delimited pending stages. Do not close all of RB-11
  when the input-validation slice passes.

## Ordered continuation and stopping rule

1. Preserve the existing staged work and evidence. Read this review and the
   validation record. Remote `main` was already ahead of the reviewed local
   HEAD by `0465e3a28` and `a327e2932` (CI documentation/repairs); inspect and
   integrate that state deliberately. Do not pull/reset over the staged patch
   or claim its old binaries contain the remote changes.
2. Add the four groups of failing regressions above, then repair only those
   input checks/notices and their validation machinery. Keep the existing
   missing-file/index, scalar binding and settings repairs. No coefficient,
   friction, element, quadrature or tolerance changes are implied.
3. Freeze the candidate for verification. Record the source/dependency and
   patch identities, both binary hashes, runner/fixture hashes and command
   exits. Rebuild both targets once; run the targeted regressions and the
   corrected matrix first. Preserve failures and document expectation changes
   against a contract, not merely against observed output.
4. Run the affected selection, `[assembler]`, relevant material/cache/derivative
   tests, the lumped P2 scene against its unchanged reference, the five public
   smokes and affected HDA workflow checks. After these pass, run one full
   suite on that stable candidate because the patch affects shared readers and
   assemblers. Attribute residual failures against the same baseline/settings;
   do not repeatedly launch full suites while editing their inputs/sources.
5. Refresh the record's claims and status, retain the unmeasured stages, and
   publish the verified implementation under the project instructions. A clean
   rebuild plus the former 73/73 is insufficient while the counterexamples
   above remain. This review is not approval to publish the pending code.

The ten review runs are narrow counterexamples and controls. They are not a
replacement for the implementing session's final validation or a certification
of any material's full physical envelope.
