# Restart state carries the augmented-Lagrangian multipliers — 2026-09-28

## Problem

The state-file size work ([state-file-chunks-20260927.md](state-file-chunks-20260927.md))
left one observation open. On the semi-implicit smoke geometry refined three
times (`n_refs: 3`, 107,823 DOFs, IE, friction 0.3), a run resumed from
`restart_3.json` ended 3.2e-4 away from the uninterrupted run. The restart
record ([restart-json-20260927.md](restart-json-20260927.md)) measured
roundoff on the unrefined scene. The gap was the same with the old
and new state-file layout and in a resume across binaries.

## Diagnosis

Evidence: `outputs/restart-friction-resume/20260928T021938Z/` in the parent
workspace.

1. **The first resumed step already differs.** Comparing each step's state
   file between the uninterrupted run (A) and the resumed one (B), `u` at
   step 4 differs by 2.0e-4 (1e-3 relative). B doubles the contact trim
   during that step (1.612 → 3.224) where A keeps it, and the friction lag
   ends with a different collision count (1,199 vs 1,196). Note that the
   `k/n t=` log line is printed at the *end* of a step, so step k's solve
   comes before it.
2. **Everything else in the restored state matches.** Both runs log the same
   trim (1.61353), the same per-contact κ statistics, the same minimum
   distance and "1228 of 1228 contacts continued" before step 4's first
   solve. Yet the first objective value differs (f₀ = 735,201 vs 731,787).
3. **Ruled out:**
   * HDF5 layout: the r2 state file rewritten with r3's chunked/contiguous
     layouts resumes bit-identically.
   * Friction lag non-convergence: the small scenes fail to converge it
     every step too, and resume to 1e-15.
   * CHOLMOD indefinite-matrix fallbacks: r1/r2 hit them as well.
   * The order of the step-end updates: the state is written after
     `update_quantities`.
4. **Per-form values at f₀** (temporary instrumentation of
   `FullNLProblem::value` behind an environment variable, not published):
   elastic, inertia, barrier contact and friction are bit-identical between
   A and B. Only `bc-alagrangian`, the augmented-Lagrangian Dirichlet term,
   differs: 730,898.68 vs 727,484.44. The 3,414.25 gap is the multiplier term
   −λᵀM^½(Ax − b).
5. **Cause.** `AugmentedLagrangianForm::lagr_mults_` are zeroed only in the
   form's constructor. Every AL pass adds to them (`update_lagrangian`), and
   neither `ALSolver::solve_al` (`set_initial_weight` sets only the weight)
   nor anything between steps resets them. Each step's AL stage therefore
   starts from the multipliers the previous step left; the form's own RB-06
   comment says they "persist across steps". The restart state file did not
   hold them, so a resumed run started them at zero.
6. **Why it showed only at `n_refs: 3`.** An AL pass runs only when the
   Dirichlet increment cannot be snapped onto directly (`while
   (!gate.feasible)`). The smoke scene moves its top face 0.05 per step:
   more than an element height at r3 (1/32), less at r2 (1/16). The
   multipliers read back from state files:
   * r2: exactly 0 through step 3, then 37,230 → 103,097 once contact
     interferes;
   * r3: 4.3e4 already at step 3.

   A restart at step 3 lost nothing on r2 and lost the multipliers on r3.
   Resuming r2 from `restart_4` (λ ≠ 0) with the old binary: the first
   resumed step differs (3.0e-15), final 1.0e-11. On r3 the trim controller
   reacts to the different path, which turns the lost multipliers into the
   3.2e-4 gap.

## Change

* `NonlinearElasticVarForm::save_restart_form_state` also writes
  `al_scalars` (the number of AL forms) and `al_multipliers_{i}` (N × 1) for
  every non-empty form.
* `restore_restart_form_state` puts them back
  (`restore_restart_multipliers`, via the new
  `AugmentedLagrangianForm::set_lagrange_multipliers`) before the contact
  state. It now runs whenever a state file is given, not only with contact,
  so no-contact runs with AL stages resume exactly too.
* A file without `al_scalars` (every file written before this change)
  resumes with zero multipliers and a warning. A count or size mismatch is
  a named error.
* The AL weight is not saved: `solve_al` resets it at every step.
* Fresh runs are unchanged; the state file only gains datasets.

## Validation

**Unit test.** `[restart]` "restart from restart json" now pushes the fixed
face 0.2 up in steps 1 and 4 (more than an element height, so both steps
run AL passes and step 1's multipliers reach the restart step). It requires:

* `al_scalars` and nonzero `al_multipliers_0` in the restart step's state;
* the first resumed step's `u` to be **bit-identical** to the uninterrupted
  run's;
* the final solution to match within 1e-10, as before.

With `restore_restart_multipliers` disabled, the bit-identity check fails:
4.5e-14 with classic adaptive stiffness, 2.9e-12 with semi-implicit contact
and friction (`negative-control-multipliers.log`). With it, the test passes
(3 cases, 87 assertions).

An earlier version of the test with the scene's zero Dirichlet value, and one
with a single jump in step 1, passed without the restore:
* a Dirichlet face at rest makes step 4's snap trivially feasible;
* the reduced solve then never reads λ.

**Resume matrix** (6 steps straight through vs 3 + resume, single-threaded;
`compare.txt`):

| Case | Uninterrupted: old = fixed | First resumed step \|A − B\|, old → fixed | Final \|A − B\|, old → fixed | \|λ\| at step 3 |
| --- | --- | --- | --- | --- |
| Seven small restart scenes (IE/BDF2/QS, contact, friction, classic adaptive, no contact) | byte-identical | 0 → 0 | ≤ 3.3e-15, unchanged | 0 |
| r2 IE friction, restart at 3 | byte-identical | 0 → 0 | 1.2e-15 → 1.2e-15 | 0 |
| r2 IE friction, restart at 4 | byte-identical | 3.0e-15 → **0** | 1.0e-11 → **0.0** | 3.7e4 at step 4 |
| r3 IE friction, restart at 3 | byte-identical (steps 1–4 compared) | 2.0e-4 → **0** | 3.2e-4 (pre-rebase binary) → **1.05e-15** | 4.3e4 |

With the fix, the r3 contact trim matches in every resumed step. Steps 5–6
differ at 6–7e-16, the one-ulp step-time residual the restart record
describes.

**Other checks** on the fixed build (`c2a57e393` + this change; PolySolve
`6a8c2cc9`, whose only difference from the new pin `43ca2e66` is a doc
file):

* the output/restart/contact unit selection passes: 96 cases, 11,304
  assertions (`unit-selection.log`, tags in `unit-selection-tags.txt`);
* all 13 Houdini asset test scripts pass, run from copies pointed at the
  evidence binary because the shared build was running another session's
  suite (`hda-tests.log`).

## Not covered

The FSI form and the legacy `State` output path keep their own restart code
and do not save AL multipliers.
