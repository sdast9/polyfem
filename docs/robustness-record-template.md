# RB-XX — <item title>

Date: <YYYY-MM-DD>
Status: <use robustness-plan.md vocabulary>
Selected stage: <stage and exact scope>

Replace placeholders when creating `docs/rb-XX-validation.md`. Do not fill absent
measurements with inferred values. Keep historical results when resuming; add a
dated section for the new stage and update the current status at the top.

## Contract and authorization

- User-selected item and any explicit scope/policy decisions:
- Invariant and precise success criterion:
- Dependencies read, their revisions and any unresolved limits:
- Exclusions / decisions not yet authorized:

## Baseline and reproduction

- PolyFEM branch/commit and tracked/untracked incoming changes:
- Effective IPC/PolySolve source paths and revisions, recipe pins and overrides:
- Build/compiler/platform/linear-solver/thread settings:
- Binary and relevant HDA identity/hashes:
- Evidence directory, input hashes, exact commands and working directories:
- Reproduced symptom and negative control, including exit status/assertion counts:
- If not reproduced, distinguish code inspection from experimental evidence:

## Findings and changes

| Finding | Evidence | Outcome: reproduced / fixed / unresolved / not reproduced |
| --- | --- | --- |
| <specific claim> | <command/log/measurement> | <outcome> |

Explain why the change restores the stated contract. For numerical/model changes,
record dimensions, the mathematical definition, alternatives considered and the
user's choice. List changed settings explicitly; “same settings” requires checks.
Document cache/state/reset behavior and compatibility effects where relevant.
For RB-13–RB-17 also record: physical versus weighted units; exact distance/band
statistic; tangent and predictor assumptions; rigorous bound versus empirical
estimate; uncertainty/empty-interval handling; held-out comparison; and the
coefficient/lag state associated with each measured endpoint. Distinguish k from
barrier force and tangent stiffness. Separate model changes from solver controls.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Exit / pass / fail / not run |
| --- | --- | --- | --- | --- |
| Baseline reproduction | <fixture> | <criterion> | <result> | <status> |
| New regression | <selector> | <criterion> | <cases/assertions> | <status> |
| Affected smokes | <each scene> | <intended steps> | <completed steps> | <status> |
| Physical comparison | <units/model> | <declared threshold> | <quantity/error> | <status> |
| Formatting / diff / links | <scope> | <criterion> | <result> | <status> |

- Numerical termination criterion versus independently measured residual:
- BC/reaction/gap/inversion/energy-work metrics measured, sampling scope and units:
- Missing quantities and why they are unavailable:
- Retained failed/partial runs and successful reruns, with no causal guesses:
- Test tolerances and reference derivation; any changed protocol and approval:
- Whole-suite/platform/private-scene checks not performed:

Do not mark all physical checks “passed” because the numerical solver returned.
A one-item validation does not certify the whole fork.

## Publication and reproducibility

- Rebuilt targets and exact tested source identity:
- Committed files and remote/branch/implementation commit:
- Companion pin and HDA installation/publication evidence where applicable:
- Remaining working-tree changes and ownership:
- Link checked-in small fixtures/results; identify local/private evidence that
  cannot be distributed. Do not commit private inputs, secrets, giant outputs or
  HDA backup/ directories.

Record the tested implementation commit in a subsequent documentation update if
needed. Report the final documentation commit in the completion message; do not
create repeated commits merely to put a document's own final hash inside itself.

## Next session handoff

- Completed stages versus stages still pending:
- Current status and why its acceptance criteria are/are not satisfied:
- Concrete model decision or external prerequisite required, if any:
- Next command/fixture and next item that is eligible to start:
- Updated robustness-plan.md status row / README claims:

Completion means the selected contract was validated and published, not merely
that code was written. A completed investigation with an unresolved model choice
must remain `characterized—decision pending`.
