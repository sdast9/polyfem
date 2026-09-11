# RB-17 — Integrated adaptive-barrier comparison

Date: 2026-09-11.
Status: **in progress — candidate selection prepared; integration pending**.

## Authorization and prerequisite audit

The user requested “Let's move to complete RB-17.” Read the robustness plan,
PF invariants and floor retirement, RB-14/15/16 contracts and final validation
dispositions, and RB-04 accounting record. RB-17 expressly requires authorization
of a concrete candidate before integration. The prior records leave that choice
open. The [candidate proposal](rb-17-proposal.md) gives a recommended package,
alternatives, measured tradeoffs and a comparison/acceptance protocol.

This is preparatory documentation, not completion of RB-17's integrated matrix.
No production estimator, assignment, controller, protection or stopping policy is
selected by this document. No new defect reproduction or solver run is claimed.

## Verified incoming state

PolyFEM main was clean at `12439a1f5`; IPC semi-implicit-stiffness was clean at
`af317a65d69d0ac7c5efa4bf103bf75e280c323b`; PolySolve iteration-callback was clean
at `4d372fa8a73f42bc224e31d464f1a308e1159ba8`. CMake's effective IPC and PolySolve
source paths point to the local companion checkouts and agree with recipe pins.
No existing PolyFEM/RB-15–17 simulation process was found or interrupted.
All incoming outputs remain untouched.

Read current BarrierContactForm interfaces and refresh/post_step call sites;
the existing feature cache and callback timing do not already implement the
proposed parent/outer-solve contract. Historical results in the proposal come
from the checked-in prerequisite records; they were not rerun in this audit.

Local evidence is `outputs/rb-17/20260911T142110Z-preflight/` in the parent
workspace. It records Git state, effective cache/dependency identity and hashes.
No build configuration, production binary, HDA asset or input was changed.

## Validation and publication

Documentation link checks and `git diff --check` apply to this stage. Builds,
solver tests, scenes, physical metrics and timings are **not run/not measured**:
there is no implementation to validate yet. No private scene or Teseo was run.
Publish the proposal, this record and plan status to `sdast9/polyfem:main` under
standing project instructions. The task response records the publication commit.

## Remaining work

Select the concrete candidate package or amend its estimator, parent, timing or
protection choices. Then write the exact case manifest, implement the opt-in mode,
execute ablations/reference comparisons and required build/test/smoke/HDA checks,
and publish the decision report. RB-17 remains incomplete until those acceptance
checks and evidence are delivered. Production default promotion is a later
explicit decision, even if the experimental comparison succeeds.
