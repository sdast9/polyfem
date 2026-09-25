# EF-04b — The feasible-bound stall trigger, retested where it could help

Date: 2026-09-25. Follow-up to [EF-04](ef-04-stall-trigger.md) of the
[contact-efficiency plan](contact-efficiency-plan-20260923.md). **Status: done
— measurement only; no code or default change.** In none of the regimes
EF-04 left open does `restart/alpha_basis: feasible_bound` reduce cost beyond
run-to-run noise, and under the current convergence contract the preconditioned
L-BFGS it was meant to unblock does not converge R1 under either basis.
Recommendation: keep the `absolute` default. **Decided 2026-09-25 (user):
keep `absolute`.**

Evidence: `outputs/ef-04b/20260925T022808Z/` (not in the repository): `runs/`
(one directory per run, `row.json` with the command, overrides and binary
hash), the sequence scripts `seq-1…6-*.sh` with their logs, `tables.md` /
`metrics.json` (reduced by `tools/ef04/ef04_tables.py`), `exp5/` (the
isolated worktrees, configure command and build log) and `bin/` with
`SHA256SUMS`:

* `PolyFEM_bin-ef04-9f9848f97`, sha256 `070c7b9b…` — the shared build of the
  published head `9f9848f97` (EF-04's `PolyFEM_bin-ef04`, byte-identical);
* `PolyFEM_bin-precond-9f9848f97-ps0000cd2`, sha256 `13736a9c…` — the same
  PolyFEM and IPC (`482b9eab`) against PolySolve `0000cd2`, a local
  (unpushed) merge of the experimental branch `qn-contact-experiment`
  (`92ee746`) onto the pin `448f1b8` (clean merge). Pins on main unchanged.

All runs were strictly sequential. `pmset -g log` has no sleep or dark-wake
event from the start of the evidence (02:28Z) until 11:22:42Z, when the
machine, by then on battery, entered clamshell sleep and cycled through dark
wakes until ≈12:29Z. Every run is clean except the last one of §5,
`pcR10-fb-mr200-R1` (started 11:13Z): its 900-s budget (monotonic clock, which
excludes sleep but not dark-wake throttling) spans the sleep, so its iteration
count at the timeout is not comparable; its outcome (no convergence in step 1)
is. R4 and BB ran on all threads, everything else
single-threaded (BB is R4-sized — 217,480 bases, ≈4.7 s per iteration
single-threaded — so it ran like R4; this is the one deviation from "only R4
on all threads").

## The question

EF-04 found that the feasible-bound basis removes R4's bounded-step alpha
restarts (H-E) but makes nothing cheaper: every attempt then ran to the
100-iteration soft budget, which took over the retunes, and the trim walk from
1 did not move. It left four regimes where the basis could still pay: a larger
or absent soft budget (the forced restarts might have hidden a benefit),
multi-step runs at the pinned optimum, held-out scenes, and a start near the
optimum (EF-02/03) or a solver the absolute trigger interrupts (EF-06's
preconditioned L-BFGS).

## Method

`tools/ef01/ef01_run.py` (Newton, the scene's settings, iteration and physical
diagnostics, predictors on, coefficient events discarded) with
`--set /solver/contact/semi_implicit/restart/alpha_basis='"feasible_bound"'`
and, per experiment, `restart/soft_iteration_limit`, `trim_min = trim_max`
(`--trim`) or `feasible_ratio_threshold`. Reduced by `tools/ef04/ef04_tables.py`
with EF-01's production runs as baselines and EF-01's pinned tight references
(`ref-SCENE`) for the error; for IT and BB, which EF-01 has no reference of,
the error is against the first absolute run (`--ref IT=abs-IT --ref BB=abs-BB`).
The reducer now also reports the solve attempts (count, longest; one PolySolve
minimize each) and step 1's trim walk: the accepted iteration at which the
trim first leaves 1 and first reaches 2⁻¹⁰, and the trim moves by source
(in-solve controller, stall retune, other refreshes).

**The AL pass's own cap.** R1 and R4 export `augmented_lagrangian/nonlinear/
max_iterations: 500`, and an attempt that reaches it throws "Reached iteration
limit in AL", a step failure. With the soft budget off an attempt can exceed
it, so the soft-off R4 runs (`*-soffx`) raise that cap to 5000 to make the
attempt lengths observable; R1 ran both ways (`*-soff` at 500, `*-soffx`).

## Results

### 1. Soft budget × basis

R4 step 1 (production controller from trim 1). "Walk" = the accepted
iteration at which the trim first leaves 1 / first reaches 2⁻¹⁰. Soft 100 rows
are EF-01's and EF-04's runs on the same code path.

| soft budget | basis | run | iterations | restarts | attempts (longest) | walk | error vs `ref-R4` | wall s |
|---|---|---|---|---|---|---|---|---|
| 100 | absolute | `prod-R4` / `rep-R4-prod` / `ctl-R4` | 800 / 726 / 828 | alpha 8–9 or 6, soft 3–5 | 12–13 (101) | 427 / 355 / 427 | 11 / 11 / 9.6 % | 1,854 / 1,589 / 1,865 |
| 100 | feasible bound | `fb-R4` / `fb-R4-rep` | 879 / 852 | soft 8 | 9–10 (101) | 438 / 424 | 12 / 11 % | 1,998 / 1,877 |
| 200 | absolute | `abs-s200-R4` / `-rep` | 880 / 771 | alpha 9, soft 1 / alpha 11 | 12–13 (201 / 164) | 448 / 360 | 11 / 12 % | 1,971 / 1,742 |
| 200 | feasible bound | `fb-s200-R4` / `-rep` | 699 / 706 | soft 3 / alpha 1, soft 2 | 5 (201) | 335 / 337 | 12 / 11 % | 1,561 / 1,575 |
| 400 | absolute | `abs-s400-R4` / `-rep` | 697 / 733 | alpha 3, soft 1 / alpha 9 | 6 (401) / 11 (268) | 325 / 320 | 9.4 / 9.4 % | 1,571 / 1,666 |
| 400 | feasible bound | `fb-s400-R4` / `-rep` | 751 / 803 | soft 1 | 3 (401) | 330 / 397 | 9.1 / 9.3 % | 1,679 / 1,773 |
| off (AL cap 5000) | absolute | `abs-soffx-R4` | 806 | alpha 13 | 15 (299) | 390 | 9.2 % | 1,818 |
| off (AL cap 5000) | feasible bound | `fb-soffx-R4` | 746 | none | 2 (**745**) | 390 | 9.7 % | 1,631 |

* **The basis has no effect beyond the walk's start.** Over all 15 runs,
  iterations ≈ 299 + 1.26 × (the iteration at which the trim first leaves 1),
  r = 0.91, and the residual averages 0 under both bases. The start itself is
  not moved by the basis (320–448 absolute, 330–438 feasible bound): it is the
  first time the noisy band statistic (EF-01 E2) crosses √0.9 after the
  30-iteration cadence. Every trim move in step 1 of every run is an in-solve
  ×½ (11–12 of them); restarts and their retunes move the trim at most once.
* **Pooled:** absolute 780 ± 60 iterations (n = 8), feasible bound 777 ± 70
  (n = 7). The soft-200 pair looked like a feasible-bound win (699/706 against
  880/771) until the soft-400 pair reversed it (751/803 against 697/733).
  Budgets of 200/400 average 755 against 817 at 100 for both bases together —
  a hint, not a result (n = 8 vs 5, inside ±1.5 SD); the default soft budget
  is not part of this item.
* **Soft off is not viable with feasible bound at the scene's settings:** with
  no small-α step left to count and no budget, step 1 is one 745-iteration
  attempt, which the exported AL cap of 500 turns into a step failure. Under
  the absolute basis the alpha trigger keeps attempts ≤ 299 and the step
  completes at either cap.

R1 (3 steps, single-threaded):

| soft budget | absolute: iterations (restarts) | feasible bound: iterations (restarts) |
|---|---|---|
| 100 (EF-01/EF-04) | 377, 388, 357, 362 (alpha 4–8, soft ≤ 1) | 388, 411 (alpha 3–5, soft 1) |
| 200 | 362 (alpha 8) | 375 (alpha 4) |
| 400 | 368 (alpha 7) | 397 (alpha 4) |
| off, AL cap 500 | 414 (alpha 4; longest attempt 143) | 434 (alpha 4; longest 116) |
| off, AL cap 5000 | 385 (alpha 3; longest 163) | 401 (alpha 4; longest 129) |

Errors per step against `ref-R1`: 7.5–7.6e-4, 4.2–4.5e-4, 5.1–8.0e-4 in every
run, both bases. R1's small-α steps are 85–90 % backtracked, so the basis
exempts few of them; feasible bound is 13–29 iterations above absolute at
every budget, inside the absolute spread (357–414).

### 2. Multi-step at the multi-step optimum

| scene | run | basis | iterations (per step) | restarts | error vs ref (step 1) | wall s |
|---|---|---|---|---|---|---|
| R4 1–5, pin 2⁻¹² | `s5-R4-k-12` (EF-01) | absolute | 377 (101, 66, 51, 80, 79) | alpha 2 | 9.2 % | 883 |
| | `abspin5-R4-k-12-a` / `-b` | absolute | 501 (127, 75, 77, 92, 130) / 353 (95, 57, 58, 48, 95) | alpha 4, soft 1 / none | 9.0 / 10 % | 1,185 / 871 |
| | `fbpin5-R4-k-12-a` / `-b` | feasible bound | 392 (95, 70, 75, 89, 63) / 437 (130, 55, 61, 86, 105) | none / soft 1 | 9.4 / 10 % | 949 / 1,046 |
| BBT 20, pin 2⁻¹⁸ | `abspin-BBT-k-18` / `fbpin-BBT-k-18` | absolute / feasible bound | 101 / 101, identical per step | none | identical | 23.3 / 23.4 |
| BBT 20, production | `abs-BBT` / `fb-BBT` | absolute / feasible bound | 186 / 186 | none | identical to EF-04's | 32.6 / 32.5 |

At the pinned optimum the trigger fires 0–5 times in five steps under either
basis; feasible bound (392, 437) sits inside the absolute spread (353–501).
BBT never restarts, so the basis cannot act.

### 3. Held-out scenes

| scene | run | basis | iterations | restarts | small α: at bound / backtracked | error vs the first absolute run | wall s |
|---|---|---|---|---|---|---|---|
| IT, 200 steps | `abs-IT` / `abs-IT-rep` | absolute | 4,498 / 5,457 | alpha 1, soft 5 / alpha 7, soft 10 | 15 / 257; 14 / 517 | — / diverges from step 12 | 426 / 517 |
| | `fb-IT` | feasible bound | 6,159 | alpha 14, alpha+soft 1, soft 9 | 43 / 562 | diverges from step 12 | 598 |
| | `fb30-IT` | feasible bound (0.3) | 6,387 | soft 18 | 58 / 318 | diverges from step 12 | 605 |
| BB, step 1 | `abs-BB` / `abs-BB-rep` | absolute | 473 / 473 | alpha 2, soft 3 / soft 4 | 133 / 4; 129 / 6 | — / 0.38 % | 972 / 938 |
| | `fb-BB` | feasible bound | 449 | soft 4 | 132 / 3 | 0.62 % | 897 |
| | `fb30-BB` | feasible bound (0.3) | 458 | soft 4 | 137 / 4 | 1.0 % | 921 |

* **IT** (`inertia_test`, BDF3, d̂ 1e-3, 1,760 bases) is a dynamic impact
  whose runs are not reproducible: two identical absolute runs agree to 1e-15
  until first contact (step 11–12), then part (1e-3 by step 14, 0.1 by step
  29) and end 21 % apart in total iterations. Its small-α steps are 94 %
  backtracked, so the basis can exempt only 15–58 of them. Both feasible-bound
  runs cost more (6,159 and 6,387, 13–17 % above the costlier absolute run);
  with this spread that is at best a lean against the option.
* **BB** (`ball_burst`, 217,480 bases) behaves like R4 — trim stuck at 1 for
  ≈250 iterations, 97 % of small-α steps at the bound — but its step 1 is
  cheaper and reproducible to 0.4 %. Feasible bound saves 15–24 iterations
  (3–5 %) by removing at most two alpha restarts; the absolute repeat, which
  had none, cost the same 473 as the run with two. The saving is within what
  the restart count itself explains and does not reach the walk (≤ 2⁻¹⁰
  never reached in step 1; first departure 246–267 either way).

### 4. EF-02/EF-03 enabled — skipped

Neither the step-start trim estimate (EF-02) nor the force-weighted band
controller (EF-03) is on `sdast9/polyfem:main` (checked 2026-09-25 against
`origin/main` = `9f9848f97`). This regime — the one EF-04 pointed to — remains
untested; the pinned runs of §2 stand in for it and show no benefit.

### 5. EF-06's preconditioned L-BFGS

R1, 3 steps, binary `PolyFEM_bin-precond-9f9848f97-ps0000cd2`; the method is
set by `--set /solver/nonlinear/solver="L-BFGS"` (the driver's `method` field
still says Newton). Variant B = `preconditioner: Hessian`, refresh 10,
`refresh_short_step` 0.5, `clear_pairs`; R10 = fixed refresh 10.

| run | basis | max restarts | outcome | step-1 iterations | restarts |
|---|---|---|---|---|---|
| `pc-newton-abs-R1` | absolute (Newton control) | 20 | 3 steps, 360 iterations | 195 | alpha 3, soft 2 |
| `pcB-abs-R1` / `pcB-fb-R1` | absolute / feasible bound | 20 | step 1 fails (restarts exhausted) | 1,746 / 1,864 | alpha 5, soft 15 / alpha 4, soft 16 |
| `pcR10-abs-R1` / `pcR10-fb-R1` | absolute / feasible bound | 20 | step 1 fails | 210 / 753 | alpha 20 / alpha 16, soft 4 |
| `pcB-abs-mr200-R1` / `pcB-fb-mr200-R1` | absolute / feasible bound | 200 | timeout at 900 s, still in step 1 | 13,314 / 12,764 | alpha 4, soft 130 / alpha 5, soft 124 |
| `pcR10-abs-mr200-R1` / `pcR10-fb-mr200-R1` | absolute / feasible bound | 200 | timeout at 900 s, still in step 1 | 13,981 / 11,311 (ran through the sleep) | alpha 24, soft 135 / alpha 14, soft 109 |

**Why B no longer converges R1.** In the qn-contact investigation every R1
step of variant B (`q-r1-B`) ended on "Configured directional-derivative
tolerance reached" — the slope tolerance. The later repair (PolySolve
`448f1b8`, PolyFEM `9f8f25881`) accepts that tolerance only for a direction
that solves with the Hessian, which the L-BFGS strategy does not report; B must
now reach the gradient-norm tolerance (rel 1e-10), and on R1 it does not
within 20 restarts under either basis, nor within 15 minutes (11–14 thousand
iterations, 124–160 attempts) with 200. The absolute trigger interrupts the
fixed-interval run R10 (20 alpha restarts in 210 iterations); feasible bound
exempts 52 bounded steps and lets it run 3.6× longer, into the soft budget,
before failing the same way; with 200 restarts neither basis converges. So
the obstacle EF-04 attributed to the trigger
is real for R10, but removing it does not make either variant converge: the
blocker is the convergence contract, which is EF-06's question, not this
item's.

## Conclusion

* **No regime measured here shows a benefit** from
  `restart/alpha_basis: feasible_bound`: R4 step 1 is governed by when the
  trim walk starts (r = 0.91), which neither basis moves, at soft budgets 100,
  200, 400 and off; R1, the pinned multi-step R4 and BBT are unchanged; BB
  saves 3–5 % by dropping ≤ 2 restarts; IT costs 13–17 % more on a
  non-reproducible trajectory; accuracy is inside each scene's repeat spread
  throughout.
* **One setting where it is harmful:** with the soft budget off, the basis
  leaves R4 a single 745-iteration attempt, which the scenes' exported AL cap
  (500) turns into a step failure.
* **EF-06 is blocked elsewhere** (and retired by the user's decision of
  2026-09-25, below). Variant B's R1 convergence in the qn-contact
  investigation was the slope tolerance that `448f1b8`/`9f8f25881` withdrew
  for non-Hessian directions; under the current contract neither B nor fixed
  refresh converges R1 under either basis. Whether a Hessian-preconditioned
  L-BFGS direction should count as "solves with the Hessian" for that
  tolerance is a question for EF-06 (and the user), not a trigger setting.

**Recommendation to the user:** keep the `absolute` default and leave
`feasible_bound` opt-in and unexposed in the Houdini asset. Revisit only if
EF-02/03 lands (start near the optimum).

**Decisions (user, 2026-09-25):** the default stays `absolute`. The user is no
longer pursuing L-BFGS: the slope-tolerance rule stays as it is (Newton family
only), so EF-06 is retired and the preconditioned-L-BFGS branch stays
unpinned and unproductionized.

## Reproduce

```
python3 tools/ef01/ef01_run.py fb-s200-R4 --scene R4 --steps 1 --threads 0 --timeout 2700 --out EVIDENCE --binary BIN \
    --set /solver/contact/semi_implicit/restart/alpha_basis='"feasible_bound"' \
    --set /solver/contact/semi_implicit/restart/soft_iteration_limit=200
python3 tools/ef04/ef04_tables.py EVIDENCE --ef01 EF01_EVIDENCE --ref IT=abs-IT --ref BB=abs-BB
```

The scenes R1/R4/BBT/IT/BB are the user's Houdini exports under `test_cases/`
(not in this repository).
