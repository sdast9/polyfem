# RB-11 follow-up review — 2026-09-14

Reviewed publication `ef5dffe80` and its identity record `a23ac34d9` after
the first stage-review handoff was completed. The incoming checkout was
clean, and `origin/main` was verified at `a23ac34d9`.

**Verdict:** Stage 1 still looks correct within its documented scope. Stage 2
is now following an efficient, bounded characterization protocol, and its
completed matrix is supported by the saved evidence. Two focused follow-ups
remain before treating the accompanying implementation and reporting repairs
as finished: BDF force normalization and incomplete-evidence validation.
The previous missing-run problem is resolved. Keep the completed matrix and
the documented limitations; another broad investigation is unnecessary.

## Evidence checked

Review artifacts are in the parent workspace at
`outputs/rb-11/20260914T133530Z-followup-review/`. Source files were not
modified for the numerical checks. The scripts and generated cases there
make the findings below independently reproducible.

- Frozen candidate B and the current solver both have SHA-256
  `a134f6c80a8063d4474b8cb736c44d70845b0aa95ce83260daf3a914ce5d6017`.
  The current test executable matches the recorded candidate-B SHA-256
  `62223ec2d7c53b76664f04738f3dc12934b69279e9e2c2d26cc3776a90ad1c63`.
- Fresh `[linear_elastic][rb11_envelope]`: **3 cases / 111 assertions pass**,
  exit 0, 29.4 seconds. These tests exercise Implicit Euler, not BDF startup.
- Fresh envelope `--self-test` passes. The additional incomplete-evidence
  probes below demonstrate gaps that its existing cases do not cover.
- Independently checked all **113 unique saved cases**: 51 reused candidate-A
  runs and 62 candidate-B runs, comprising the original 109 plus four
  reference refinements. All have exit 0, endpoint time 1, existing endpoint
  VTU files and parseable solver records. Every recorded input hash matches
  its file. Legacy candidate-A identity remains separately attributed.
- Re-evaluating the saved measurements reproduces all **270 checks: 269 pass,
  one fail, none not evaluated**. The retained failure is C-N, 24 iterations
  versus a target of 20 for `homogeneous-nu0.49999-n4`.
- All 81 nonlinear records contain actual iteration counts and determinant
  evidence. The reporting defects below were reproduced by removing evidence
  in isolated copies; they do not negate these existing measurements.
- Candidate B's saved Stage 1 matrix still records **93/93 expected outcomes**.
  Saved focused/affected logs support 31 cases / 920 assertions and 61 cases /
  4,770,073 assertions. The documented standard-test failure remains explicit.
  This review did not rebuild or rerun those larger selections or the full suite.
- The four selected P2-to-P3 tip changes are 0.1060%, 0.9960%, 0.1075%, and
  0.0460%. They meet the declared 1% engineering gate. This is bounded
  reference-sensitivity evidence, not a rigorous bound on continuum error;
  the nearly incompressible reference is at the edge of that gate.

## 1. BDF force normalization is still wrong

**Priority: P1.** A four-step quasistatic linear cantilever, `dt = 0.25`,
constant load, no contact, produces bit-identical displacements for
Implicit Euler, Implicit Newmark, BDF2 and BDF3. Quasistatic inertia is
correctly zero in all four. Elastic and body forces should also agree.

| Integrator | Steps | Elastic and body force ratio to Implicit Euler |
| --- | --- | --- |
| Implicit Newmark | 1–4 | 1 |
| BDF2 | 1–4 | 2.25 |
| BDF3 | 1 | 2.25 |
| BDF3 | 2–4 | 3.36111111111111 |

See `probe_linear_integrators.py`, `linear-integrator-probe.json`, and the
four `qs-*` input/log/output directories in the review artifacts. Reproduce
with a fresh output directory; the script deliberately refuses to overwrite
its existing cases.

The cause is the lifecycle, not the linear solution. In
`LinearElasticVarForm::init_linear_solve`, elastic/body weights are set once
from the initial integrator scaling. BDF begins with one history entry, so
that scaling is `dt^2`. `BDF::update_quantities` grows the history, changing
`acceleration_scaling()` to `(2/3)^2 dt^2` and then `(6/11)^2 dt^2`.
`solve_transient_linear` advances history before saving, and
`ElasticVarForm::elastic_output_fields` divides by this new scaling while
the form weights still contain the initial one.

Complete the repair by making exported forces use the scaling of the step
that was actually solved. Capture that value before advancing history,
refresh the linear elastic/body weights for each step, and retain the
matching inertia predictor and force-normalization scale for output.
An explicit saved-step normalization override for the linear output path
is one possible bounded implementation. Preserve saved velocity/acceleration
and restart history. Merely refreshing weights at the start of each step
does not fix output at BDF startup: history advances again before export.
Likewise, changing the save order alone does not cover `output_fields()`
called after the solve, as the current regression helpers do.

Required checks:

- Quasistatic static-equivalence and physical-force checks for BDF2/BDF3,
  including every startup step and a step after full history is available.
- Dynamic BDF2/BDF3 checks with nonzero inertia: exported elastic force is
  `-K u`; body force is the physical load at the saved time; inertia is
  `-M (u - x_tilde) / s_solved`; their sum vanishes on free DOFs. Check both
  individual forces and equilibrium, since a common wrong scale can leave
  the residual zero. Exercise startup, not just the last mature step.
- Keep the existing Implicit Euler tests, add an Implicit Newmark control,
  and preserve nonzero prescribed displacements and time-dependent loads.
  Check both saved fields and the after-solve output API.

The reproduced error is in force output; these quasistatic displacements
are identical. It does not invalidate the envelope's existing default-
integrator displacement/stress/conditioning measurements.

## 2. Incomplete evidence can still pass the runner

**Priority: P2.** `audit_envelope.py` exercises real saved output through
`parse_output`, `analyze` and `verify` after removing only `iterations` from
an isolated copy of `homogeneous-nu0.49999-n4`'s nonlinear record. The parser's
`int(st.get("iterations") or 0)` turns the missing count into zero. Both
C-R and C-N then pass; the actual 24-iteration C-N failure has disappeared.
The existing self-test injects `None` directly into an analyzed entry and
therefore misses this parser-to-verifier failure.

Also, remove `F_1`, `F_2`, `F_3` from the loaded endpoint mesh and recompute
`geometry_checks`: `detF_positive` disappears, and C-R still passes because
it checks positivity only when that key happens to be present. These
nonlinear benchmarks explicitly require determinant evidence.

See `parser-adversarial-probes.json` for both counterexamples. Repair the
parser to preserve missing/invalid counts as unavailable, validate each
required nonlinear step record without silently filtering malformed entries,
and require the determinant measurement for the applicable nonlinear cases.
Keep a legitimate recorded zero count distinguishable from a missing one.
Incomplete records must make C-R fail or remain not evaluated, and missing
counts must leave C-N not evaluated. Add self-tests through parsing and
analysis, including missing/null/invalid counts and absent F components.

Re-analyze the existing 113 cases after this reporting fix; a solver rerun is
not needed to correct an oracle. The real 24-iteration miss must remain.

## Bounded completion and publication

Finish the two groups above, freeze a new candidate identity, rebuild and
run the affected focused checks once the source is ready. Preserve candidate
A/B binaries, raw runs, thresholds and all unrelated checks. Reuse the
completed physical-envelope results with explicit lineage; a broad rerun
or further large P3 refinement is not required for these fixes. Update the
validation record and publish the tested repair to the fork.

At review time `RB-12 continuation` was active in the same workspace. Inspect
incoming edits and active builds before implementation; isolate or serialize
builds/publication as needed so the two tasks do not replace each other's
sources or binaries. This review publishes documentation only.
