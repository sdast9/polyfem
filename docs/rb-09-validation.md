# RB-09 — Reference benchmarks and refinement envelope

Date: 2026-09-13
Status: characterized—limits documented (contracts, analytical references, separate sweeps, controller comparison, public-smoke refinement; the `physical_balance_pass` decision was taken and implemented on 2026-09-13 — see the decision section; the gap sensitivity and the T7 unit-conversion failure keep the item short of `validated within stated scope`)
Selected stage: the plan's full RB-09 scope on public inputs (no RB-17 candidate: retired), all sweeps separate; then the user's acceptance decision (1)–(3) below.

The contract with the benchmark definitions, quantities of interest and the
thresholds declared before the matrix ran is [rb-09-contract.md](rb-09-contract.md).
Tools: `tools/rb09/` (`reference.py`, `run_matrix.py`, `analyze.py`,
`summarize.py`, `spring_probe.cpp` + `run_probe.py`); compact results
`tools/rb09/results-20260913.json`; evidence `outputs/rb-09/20260913T115623Z/`
(2.1 GB, 40 run directories + the pilot and the probe, not distributed).

## Contract and authorization

- User request 2026-09-13: "tell me what RB-09 is and address it". No scope
  or policy decision was given; every decision boundary of the plan is
  respected: no coefficient, controller, tolerance, CCD or default change, no
  golden replacement, no acceptance threshold selected.
- Invariant / success criterion: for each benchmark, the solver's endpoint
  quantities of interest (top reaction `R_z`, integrated contact load `L_z`,
  lateral displacement `ū_x`, bottom gaps) are compared with an independent
  reference under thresholds derived from the reference's conditioning
  (T1–T14 in the contract); trends/rates across the separate sweeps are
  reported with all input hashes and every attempt retained.
- Dependencies: RB-04 record version 2 (diagnostics fields used as-is;
  `physical_balance_pass` still unavailable), RB-02/03 closed (no failure to
  resolve), RB-10 (friction excluded here, μ = 0 everywhere), RB-18–RB-21
  production law (retained). The RB-17 candidate is retired, so the plan's
  "selected candidate" column is empty by decision, not omission.
- Exclusions: friction, private scenes, Teseo, Q2+ elements, impact/high
  speed, peak stress/pressure, the GCP `contact_2d` cube-on-floor mismatch
  (separate investigation, not a semi-implicit smoke), a `units:` block
  conversion (PolyFEM's own unit declaration path; only raw-number
  conversion was tested).

## Baseline and reproduction

- PolyFEM `b924c37e6` on `main`, clean; binary `PolyFEM_bin` `6f1dcfa1…`
  built from `756070f44` (no `src/`, spec or CMake change since);
  `unit_tests` `cc05edcc…`. Effective IPC `bb795446` (`../ipc-toolkit-fork`,
  clean, = recipe pin), PolySolve `ee5b296a` (`../polysolve-merged`, clean,
  = recipe pin). macOS 26.5 arm64, Apple Clang, RelWithDebInfo,
  `Eigen::SimplicialLDLT`, `--max_threads 1` for every run (bit-reproducible;
  the duplicated `block-transient-rate0.25` / `block-transient-dt0.125`
  configuration reproduced `R_z` to the last digit). Python 3.14.7, NumPy 2.5.1.
- Evidence `outputs/rb-09/20260913T115623Z/{baseline.txt, block/, clamped/,
  spring-probe/, summary.json}`; every run directory holds the scene, the
  copied public inputs with SHA-256, the command, exit status, log, VTU
  output, `nodes.txt` and the RB-04 records. The public scene files were
  copied, never edited. Runs: 25 block + 15 clamped, all exit 0, all
  expected steps accepted, no failed attempt; one stall retune/restart in
  each of the three millimetre runs (retained, see T7).
- The pilot (`pilot/`, the base block run before the analyzer existed) is
  retained; its endpoint equals `block/block-base` to the last digit.

## Benchmarks

**A — block** (analytical continuum): public unit cube `[0,1]³` (4×4×4 P1
tets, `n_refs` 0/1/2 = 384/3 072/24 576 tets) on the public slab at
`z = −.02`, `u_x = 0` on `x = 0`, `u_y = 0` on `y = 0`, top face `u_z = −δ(t)`
with `u_x, u_y` free, μ = 0, `E = 1e7`, `ν = .45`, `d̂ = 1e-3`, `δ = .25·t` to
`t = 1`. Exact Neo-Hookean uniaxial state (`reference.py`): hard contact
`R_z = −2 994 216.48`, `dR_z/dl_z = 1.722e7`, relative reaction sensitivity
`c = 5.751 /m` to a residual gap, `l − 1 = .123169`.

**B — spring/contact** on the production `BarrierContactForm` (2D point on a
spring `k` above a floor edge, semi-implicit mode, production step flow:
Newton with `post_step`, `update_quantities`, `update_barrier_stiffness`),
exact scalar root on the realized `w·trim·κ_s`.

**C — clamped** (the public `quasistatic-semi.json` / `transient-semi.json`
with the top face fully clamped) with a same-mesh hard-contact reference
(bottom face `u_z = 0` bilateral, gap removed from the loading) and a
lifted-plane run measuring the reference's conditioning `c_h`.

## Findings

| Finding | Evidence | Outcome |
| --- | --- | --- |
| The semi-implicit barrier's reaction error against hard contact is the realized mean gap: `e_R = c·ḡ/H` on every block run (worst deviation 4.7 % of itself on the raw mm run, ≤ 3 % elsewhere, ≤ 0.4 % on the metre defaults) | T2 on 24 block endpoints; T12 on the clamped meshes (5e-5 and 1.5e-6 against 3.7e-4/5.1e-4) | reproduced — the model error is a gap error, nothing else |
| At `d̂ = 1e-3` (H = 1) the model error in `R_z` is 0.35–0.6 % (block .48 %, clamped .35–.50 %); it scales linearly with `d̂`: rates .985 (block, 4 points) and 1.018 (clamped, 3 points) | T5, T14 | reproduced |
| Against the barrier-consistent reference (exact state at the realized gap) the solver/discretisation error is 1.4e-5 in `R_z` (metre defaults), ≤ 6.4e-5 transient, 2.4e-6 in `ū_x`; the deformation gradient is homogeneous to 4e-4 | T3/T4 on all runs; independent P1 energy 315 667.7 vs exact 315 660.4 | reproduced |
| Force balance `L_z + R_z (+ I_z)` holds to ≤ 1.6e-8 relative on every run that converged on the metre force tolerance (4e-15 typical); the raw mm run stops at a 356 N residual (3.0e-4) because the stopping tolerance depends on the SI-specific `characteristic_force_density` default | T1; `block-units-mm` vs `-equal-tolerance` | reproduced (input envelope, RB-11/RB-12) |
| The realized gap is not reliably in the band: `ḡ/d̂` = .84 (dt .25), .68 (dt .125), .65 (dt .0625/.03125, 80 % of the contacting nodes below the band), .92 (`n_refs` 1), .75 (`n_refs` 2), .65/.91 (`d̂` 2e-3/4e-3), .62 with node range [.47, .85] on the clamped coarse cube, .98–.99 in millimetres. Band coverage 20–100 % | matrix table below | characterized — a controller property; every gap stays below `d̂`, so the accuracy bound `e_R ≤ c·d̂` holds throughout |
| Consequence of the gap variation: `R_z` spreads by 1.0e-3 across the mesh sweep and 1.0e-3 across the increment sweep (both within the declared 2·c·band·d̂ = 2.8e-3), 2.4e-5 on the clamped increments; the transient `dt` sequence is not monotone in `R_z` because the gap changes with the step pattern while the inertial effect is 1e-6 of the load | T6, T8, T14 | characterized |
| Unit conversion (m → mm, raw numbers) is not equivalent: `R_z` differs by 1.06e-3 (raw), 8.7e-4 (dimensional solver constants converted), 8.7e-4 (equal force tolerance); the difference is entirely the realized gap (.98–.99 vs .835 `d̂`). Cause: upstream IPC `TightInclusionCCD::ccd_strategy` caps the minimum separation of a truncated step at an absolute `1e-4` length units (`min(0.2·gap, 1e-4)`), so the first-contact iterate sits at .1·`d̂` in metres but 1e-4·`d̂` in millimetres; the emergency controller's collapse bump is then ×7.6 (m) versus the ×256 cap (mm), the trim climbs to 65 536, the linear solves lose accuracy, a stall retune/restart follows and the conditioning cap resets the trim (6 474 → 3 237 → 1 619 / 8 192) | T7 (declared 1e-6, failed as declared); logs `block-units-mm*/run.log`; `ipc-toolkit-fork/src/ipc/ccd/tight_inclusion_ccd.cpp:45-48` (unchanged upstream code) | reproduced — limit documented; no CCD change (plan: preserve CCD); RB-11/RB-12 items |
| PolyFEM's nonlinear stopping tolerance is `grad_norm · F0 · L^1.5` (3D, L2 norm; `NLProblem::grad_norm_rescaling`) with `F0 = solver/advanced/characteristic_force_density` (default 1e4) and `L` the bounding-box diagonal: it is not a force density; equal force tolerances across a length rescaling by `s` need `F0` scaled by `s^-1.5` (10 000 → .316 for mm), not by `s^-3` | `block-units-mm-converted` (F0·1e-9: tolerance 7.2e-11) vs `-equal-tolerance` (F0·1000^-1.5: 2.27951e-6 = the metre tolerance) | reproduced — RB-11/RB-12 observation |
| Classic `adaptive` and Fixed (at the classic endpoint κ = 5.796e9) give `e_R` = 1.1e-6 with the gap at 2.4e-7 m (2.4e-4 `d̂`, min 6e-8 m) at 63/66 Newton iterations and 25 s, versus semi-implicit 4.8e-3 at 34 iterations and 9 s | controller table | characterized — comparison only; no default change |
| Material contrast: `E` ×0.1 / ×10 reproduce the same relative state (`e_R` 4.81e-3, gap .8328, trim 30.47, 34 iterations): the semi-implicit law is homogeneous in the driving stiffness (the spring probe gives identical gaps for `k` = 1, 100, 1e4) | material table; spring cases | reproduced |
| Density ×10 and approach speed ×½/×2 (transient) change the endpoint only through inertia: `I_z` = 0.95 / 3.8 / 15 N (speed) and 37 N (ρ ×10) against 3.0e6 N; `e` vs the quasistatic reference 3.1e-5 / 3.8e-5 / 6.4e-5 / 1.2e-4 | density/speed tables | reproduced — the transient fixtures are quasistatic to 1e-4 |
| Load/unload: the unloading reactions retrace the loading ones at equal `δ` to 3e-10; at `t = 2` the contact is inactive (`active_count` 0, `R_z` = −4.5e-7 N against 3.0e6); both band limits acted (upward bumps 1 → 7.6 → 15.7 → 31.4 at first contact, one downward step 31.4 → 15.7 when the gap rose above the band at step 7) | T9 | reproduced |
| Coupled contact (two stacked identical blocks, deformable–deformable interface with coincident vertices + floor): floor load, interface load and `R_z` balance to 1e-14/6e-13; `e_R` = 4.85e-3 explained by the per-block effective gap `(g_floor + g_interface)/2` (.757 and .92 `d̂`); 174 Newton iterations against 34 for one block | T10 | reproduced |
| Spring/contact: the production form's equilibrium equals the exact scalar root to 4e-16·`d̂`, spring = barrier force to 5e-13, the form gradient equals `w·trim·κ_s·b'(g²)·2g` to 1.4e-14 and the hard-contact overshoot is exactly `k·g` (20 checks, 5 cases) | T11 | reproduced |
| Spring/contact characterization: with a CCD-truncated first iterate (absolute 1e-4 clearance = 1e-3 `d̂` at `d̂` = .1) the in-solve controller ramps the trim to 65 536 within 6 iterations and the gap settles at .9989 `d̂` (above the band); the between-steps gradient-balance calibration reproduces the equilibrium trim (a fixed point), and the downward band step (÷2 per 30 iterations above the band) is never reached by a 4–7-iteration solve, so the trim decays only 65 536 → 32 768 over 4 steps. Contact born inside the support (`d̂` = 1, anchor approaching through it) sits in the band at trim 2 | probe cases (`k`, `d̂`) = (100, .1), (1, .1), (1e4, .1), (100, .01) vs (100, 1) | characterized — the same mechanism as the mm block runs; the accuracy bound `g < d̂` still holds |
| The barrier's relative force error is `ḡ/compression`: when the imposed compression is smaller than `d̂` the overshoot exceeds the hard-contact load itself (spring `d̂` = 1, anchor −.25: 3.4×) | probe `d̂` = 1 steps 3–4 | reproduced — the model is accurate only for compressions ≫ `d̂` |
| Public coarse smoke: the same-mesh hard reference converges as −3.6257e6 (h), −3.3864e6 (h/2), −3.3014e6 (h/4), observed order 1.49, Richardson limit −3.2545e6: the 4×4×4 smoke's reaction is 11.4 % above the extrapolated value (8×8×8: 4.1 %, 16×16×16: 1.4 %) while the contact-model offset is −.35 / −.50 / −.50 % on the three meshes | T13 | reproduced — discretisation error dominates the contact-model error by 20–30× on the public fixture |
| Same-mesh hard-contact reference validity: no tensile bottom reaction on any mesh (bilateral constraint = unilateral solution) | `bottom_reaction_tensile_count` 0/0/0 | reproduced |
| Coefficient coverage: no active coefficient at the batch floor or cap in any run; active range .20–2.9 × the batch median; no curvature fallback; force continuation carried every persisting contact (fresh coefficients only at contact birth — step 2 for dt ≤ .0625 — 5 on the fine clamped mesh at step 4, and 275–477 per step on the stacked interface, where the coincident vertex pairs keep producing new parent pairs) | record `contact.*` fields | reproduced |

No production code changed. The unit-dependence findings are upstream
behaviours (IPC CCD, PolyFEM tolerance scaling) documented for RB-11/RB-12;
changing them is outside this item's permitted changes.

### Accuracy matrix (endpoint `t = 1`, `δ = .25`; SI units; `ḡ/d̂` mean [min, max] over the contacting bottom nodes)

| Run | `R_z` | `e_R` vs hard | `e` vs consistent | `ḡ/d̂` | band | trim | Newton / wall |
| --- | --- | --- | --- | --- | --- | --- | --- |
| block-base (`d̂` 1e-3, dt .25, `n_refs` 0) | −3 008 662.57 | 4.825e-3 | 1.4e-5 | .836 [.81, .93] | 1.0 | 31.4 | 34 / 9.0 s |
| block-dhat0.004 | −3 057 152.81 | 2.102e-2 | 3.1e-5 | .908 [.89, .96] | .8 | 23.9 | 36 / 6.5 s |
| block-dhat0.002 | −3 016 882.38 | 7.570e-3 | 6.1e-5 | .652 [.59, .85] | .2 | 4.0 | 31 / 7.3 s |
| block-dhat0.0005 | −3 001 732.77 | 2.510e-3 | 5.5e-6 | .871 [.85, .94] | 1.0 | 98.7 | 41 / 11.2 s |
| block-nrefs1 (3 072 tets) | −3 010 080.32 | 5.298e-3 | 3.0e-6 | .920 [.90, .96] | .89 | 60.3 | 68 / 17.6 s |
| block-nrefs2 (24 576 tets) | −3 007 100.41 | 4.303e-3 | 9.7e-6 | .746 [.68, .89] | .76 | 3.12 | 144 / 182 s |
| block-dt0.125 | −3 005 979.88 | 3.929e-3 | 2.9e-5 | .677 [.62, .86] | .2 | 9.19 | 45 / 9.6 s |
| block-dt0.0625 | −3 005 541.04 | 3.782e-3 | 3.2e-5 | .651 [.59, .85] | .2 | 8.0 | 75 / 7.1 s |
| block-transient-dt0.25 | −3 008 678.01 | 4.830e-3 | 2.2e-5 | .835 | 1.0 | 31.2 | 34 / 9.0 s |
| block-transient-dt0.125 | −3 006 007.14 | 3.938e-3 | 3.8e-5 | .677 | .2 | 9.20 | 45 / 9.9 s |
| block-transient-dt0.0625 | −3 005 563.92 | 3.790e-3 | 4.2e-5 | .651 | .2 | 8.0 | 75 / 7.0 s |
| block-transient-dt0.03125 | −3 005 572.32 | 3.793e-3 | 4.2e-5 | .652 | .2 | 8.0 | 113 / 7.3 s |
| block-E1e+06 / E1e+08 | −300 861.77 / −30 086 176.97 | 4.810e-3 | 1.4e-5 | .833 [.80, .93] | 1.0 | 30.5 | 34 / 9.1 s |
| block-transient-rho10 (dt .125) | −3 006 393.49 | 4.067e-3 | 1.2e-4 | .686 | .2 | 9.65 | 45 / 9.7 s |
| block-transient-rate0.125 / .25 / .5 | −3 005 986.70 / −3 006 007.14 / −3 006 083.48 | 3.93e-3 / 3.94e-3 / 3.96e-3 | 3.1e-5 / 3.8e-5 / 6.4e-5 | .677 | .2 | 9.19 | 45 / 9–10 s |
| block-adaptive (classic) | −2 994 219.82 | 1.113e-6 | 2.5e-7 | 2.4e-4 [6e-5, 1.3e-3] | 0 | κ 5.80e9 | 63 / 25.2 s |
| block-fixed (κ 5.796e9) | −2 994 219.82 | 1.113e-6 | 2.5e-7 | 2.4e-4 | 0 | κ 5.80e9 | 66 / 25.9 s |
| block-units-mm (raw) | −3 011 842.65 | 5.887e-3 | 2.5e-4 | .978 [.975, .990] | 0 | 1 619 | 59 (1 restart) / 16.1 s |
| block-units-mm-converted / -equal-tolerance | −3 011 292.55 / −3 011 292.60 | 5.703e-3 | 1.8e-7 / 1.9e-7 | .990 [.989, .996] | 0 | 8 192 | 85 / 84 (1 restart) / 6.1 s |
| block-stacked (`δ` = .5, two blocks) | −3 008 724.61 | 4.845e-3 | 9.9e-6 | floor .757 [.72, .90], interface .92 | 1.0 | 16.4 | 174 / 10.1 s |
| block-unload (t = 2) | −4.5e-7 (0 contacts) | — | — | — | — | 15.7 | 56 / 9.5 s |
| clamped-nrefs0 (public smoke) | −3 638 512.20 | 3.538e-3 (hard −3 625 685.64) | — | .619 [.47, .85] | .2 | 8.0 | 31 / 5.1 s |
| clamped-nrefs1 | −3 403 338.52 | 4.987e-3 (hard −3 386 449.09) | — | .871 [.84, .93] | 1.0 | 28.3 | 59 / 15.8 s |
| clamped-nrefs2 | −3 317 943.96 | 5.004e-3 (hard −3 301 424.01) | — | .879 [.85, .93] | 1.0 | 13.7 | 148 / 174 s |
| clamped-dt0.125 / dt0.0625 | −3 638 504.14 / −3 638 590.83 | — | spread 87 N | .619 / .623 | .2 | 8.0 | 42 / 74 |
| clamped-transient dt .25/.125/.0625 | −3 638 525.77 / −3 638 520.47 / −3 638 609.55 | vs quasistatic 3.7e-6 / 2.3e-6 / 2.7e-5 | — | .619 / .619 / .623 | .2 | 8.0 | 26 / 42 / 73 |
| clamped-dhat0.002 / dhat0.0005 | −3 651 958.64 / −3 632 095.83 | 7.246e-3 / 1.768e-3 | — | .633 / .619 | .2 | 4.0 / 16.0 | 26 / 30 |

Hard-reference conditioning on the clamped cube: `c_h` = 5.79 /m (`n_refs`
0), 5.73 /m (`n_refs` 1) from the lifted-plane runs; the clamped cube's
`min det F` is .857 (coarse) → .801 (fine), the block's .971.

## Validation

| Check | Input/configuration | Expected criterion | Measured result | Status |
| --- | --- | --- | --- | --- |
| T1 force balance | all 25 block runs | ≤ 1e-5 | ≤ 1.6e-8 on 24; raw mm 3.0e-4 (356 N residual at its looser stopping) | pass / 1 documented fail |
| T2 hard-contact error explained by the mean gap | 24 block endpoints, 7 loaded unload steps | `|e_R − c·ḡ| ≤ .1·c·ḡ + 1e-5` | max deviation 4.7 % of `e_pred` (raw mm), 1.2 % transient, ≤ .42 % metre quasistatic | pass |
| T3 barrier-consistent reaction | 24 block endpoints | ≤ `c·(g_max − g_min)/H + 1e-6` | 1.4e-5 base; max 1.2e-4 (ρ ×10); raw mm 2.5e-4 > 8.8e-5 | pass / 1 documented fail |
| T4 lateral displacement | 24 block endpoints | ≤ `|dl/dl_z|·(g_max − g_min)/H + 1e-8` | 2.4e-6 base; stacked 1.7e-4 ≤ 5.0e-4; raw mm 2.7e-5 > 9.7e-6 | pass / 1 documented fail |
| T5 `d̂` rate | `d̂` ∈ {4e-3, 2e-3, 1e-3, 5e-4} | slope 1 ± .15 | .985 | pass |
| T6 mesh / increment sweeps | `n_refs` 0/1/2; dt .25/.125/.0625 | every run T1–T4; spread ≤ 8 320 N | 2 980 N / 3 122 N | pass |
| T7 unit conversion | m vs mm raw / constants converted / equal tolerance | ≤ 1e-6 | 1.06e-3 / 8.7e-4 / 8.7e-4 | **fail (documented; upstream CCD absolute clearance + tolerance scaling)** |
| T8 transient vs quasistatic reference | dt .25 … .03125 | ≤ 1e-2, non-increasing | ≤ 4.2e-5; not monotone (gap-dominated, inertia 1e-6) | pass (rate not resolvable) |
| T9 load/unload | tend 2 | T2 per step; final 0 contacts, `|R_z| ≤ 1e-6·max` | all steps; 0 contacts, 1.5e-13 | pass |
| T10 stacked | two blocks + floor | interfaces ≤ 1e-5; T2–T4 | 1.2e-14 / 5.7e-13; pass | pass |
| T11 spring/contact | 5 (`k`, `d̂`) cases | roundoff | 20/20 checks | pass |
| T12 clamped barrier vs same-mesh hard | `n_refs` 0, 1 (lift runs) | `|e_R − c_h·ḡ| ≤ .1·c_h·ḡ + 1e-5` | 5.1e-5 / 1.5e-6 | pass |
| T13 mesh convergence | hard reference `n_refs` 0/1/2 | reported | order 1.49, limit −3.2545e6, barrier offsets −.35/−.50/−.50 % | reported |
| T14 clamped increments / transient / `d̂` | see table | spread ≤ 10 187 N; ≤ 1e-2; slope 1 ± .15 | 87 N; 2.7e-5; 1.018 | pass |
| Formatting / diff / links | new docs and tools | no introduced issue | `git diff --check`, `python3 -m py_compile`, local links | pass |

- Numerical termination versus independently measured residual: every
  metre run ends on the gradient criterion (2.28e-6 N) with a free residual
  ≤ 1.9e-6 N (stacked) and the RB-04 balance identity to ≤ 1.6e-8; the raw mm
  run ends on its own criterion (.072) with a 356 N free residual.
- Metrics: `R_z`, `L_z` per obstacle/interface, `I_z`, per-node bottom gaps,
  band coverage, coefficient range/floor/cap/median, `ū_x`, elastic/barrier/
  kinetic energies, support-work increments, `min det F`, Newton iterations,
  minimize calls, retunes, restarts, wall time (units: SI; mm runs converted).
- Missing: no peak stress/contact pressure (excluded by contract); no
  `n_refs` 2 lift run (its `c_h` is inferred from the two coarser meshes,
  not measured); the transient time-integration order is not resolvable on
  these fixtures (inertia 1e-6 of the load); no declared-`units:` conversion
  run; no friction, private, Teseo or Q2+ case.
- Retained partial/failed runs: none failed; the three mm runs carry one
  stall retune/restart each (retained in their records and logs).
- Tolerances: all declared in the contract before the matrix; none changed.
  The stacked T2/T4 evaluation uses the per-block effective gap
  `(g_floor + g_interface)/2`, a bookkeeping correction of the analyzer
  (the contract's `ḡ` definition applied to two interfaces), not a
  threshold change.
- Not performed: full unit-test suite (no C++ production change; the probe
  is standalone), other platforms, HDA tests (no HDA change).

## Proposal for the RB-04 `physical_balance_pass` threshold (decision left to the user)

Quantity: the RB-04 endpoint force balance `|L_z + R_z + I_z| / max_t |R_z|`
and the contact-model gap error `c·ḡ/H` with `c = |dR/dl_z| / |R|` (block)
or the measured `c_h` (any fixture, from a lifted-plane run). Normalization:
relative to the top reaction. Evidence: the balance is ≤ 1.6e-8 on every
metre run and 3e-4 on the raw mm run; the model error is `c·ḡ ≤ c·d̂/H` with
`ḡ/d̂` ∈ [.62, .99] across this matrix. A threshold of the form
`balance ≤ 1e-6` and `model error ≤ c·d̂/H` would pass every metre run here
and flag the raw mm run's residual. Selecting it — and whether it is an
application acceptance at all — is the user's decision under the plan's
register; nothing is enabled.

## Decision (2026-09-13) — `physical_balance_pass` selected

The user confirmed the three recommendations:

1. **`physical_balance_pass` is the endpoint force residual in physical
   units**, not an energy budget: `free_residual_norm ≤ 1e-6 · peak` and
   `|external-force balance| ≤ 1e-6 · peak`, with `peak` the run's peak total
   absolute external force (L1 of the support force and of each non-elastic
   form force over the body's DOFs, running maximum over accepted records)
   and the residual at the updated friction lag. Threshold
   `output.physical_balance_tolerance` (default 1e-6). Justification: the
   converged SI runs of this record sit at 4e-15–1.6e-8, the mis-scaled
   millimetre run at 1.2e-4–2.7e-3; the energy budget's right-endpoint
   increments carry O(Δt) quadrature remainders (RB-04: 111 325 J at dt .25
   on a 3e5 J energy) and cannot serve as a threshold.
2. **The contact-model error is reported, not gated:** `contact.gap_statistics`
   (count, mean, rms, min, max of the active-collision distances and the
   ratios to `d̂`) is in every record; the design rule is
   `relative model error ≈ mean gap / imposed compression ≤ d̂ / compression`.
3. **Engineering accuracy stays outside the flag** (a mesh comparison; the
   public 4×4×4 smoke is 11 % from its Richardson limit).

Implementation (record version 3, observational, no solver change):
`NonlinearElasticVarForm::write_physical_diagnostics` (the flag object with
threshold, normalization, `peak_external_force`, both ratios, the balance
vector, the friction-lag state and `solved_lag_free_residual_ratio`),
`BarrierContactForm::gap_statistics`, the spec option
`output/physical_balance_tolerance`, the `[physical_diagnostics]` regression
(gap statistics on the probe mesh, 3 cases / 64 assertions) and the RB-04
runner's expectations. Contract note: [rb-04-contract.md](rb-04-contract.md#version-3-2026-09-13-physical_balance_pass-selected-by-the-rb-09-decision).

| Check | Configuration | Result | Status |
| --- | --- | --- | --- |
| Affected selections | `[physical_diagnostics]` 64/3, `[coefficient_events]` 171/1, `[contact_cache]` 739/6, `[friction_lag]` 111/4, `semi-implicit*` 234/8 (assertions/cases) | all pass | pass |
| RB-04 endpoint runner | three public four-step fixtures off/on + the deliberate failure (`decision/rb04-endpoints/`) | off/on endpoints identical to 1e-16; reconstructions intact; flag true on `quasistatic-semi`/`transient-semi` at every step (ratios 3e-14–4.9e-11); false on the friction fixture at every step (free-residual ratio 1.7e-3–3.3e-3, balance 3.4e-3–9.7e-3) with the solved-lag ratio ≤ 1.1e-12 — the finite-lag mismatch at budget 2 is 0.2–0.3 % of the peak force on the public friction smoke | pass (as designed) |
| RB-09 cases rerun (`decision/block/`) | `block-base`, `block-units-mm`, `block-unload`, `block-stacked` | endpoints bit-identical to the earlier binary (max difference 0.0); flag true on base (≤ 3.2e-10), unload (all 8 steps, including the separated endpoint, normalized by the peak 3.009e6) and stacked (≤ 2.6e-11); false on the raw millimetre run at every step (2.1e-5–2.7e-3); per-collision mean gap .875 `d̂` at the base endpoint (46 collisions; the per-node mean of this record is .836) | pass (as designed) |
| Five public smokes | `run-smoke.sh` | all exit 0, no error line | pass |
| Formatting / diff / links | `git diff --check`, clang-format, spec JSON parse, Python compile, local links | no introduced issue | pass |

Not performed: the full unit-test suite; other platforms; the HDA tests (no
HDA change; the tolerance is not exposed in the Houdini node — a separate
choice).

## Publication and reproducibility

- Rebuilt targets: none (no C++ production change); the probe compiles
  against the existing `unit_tests` flags/link line (`run_probe.py`).
- Committed files: `docs/rb-09-contract.md`, `docs/rb-09-validation.md`,
  `docs/robustness-plan.md` (status row, order paragraph), `README.md`
  (state, table), `tools/rb09/{README.md, reference.py, run_matrix.py,
  analyze.py, summarize.py, spring_probe.cpp, run_probe.py,
  results-20260913.json}`; remote/commit in the completion message.
- Companion pins unchanged; no HDA change.
- Working tree: clean after the commit; evidence stays local (2.1 GB).

## Next session handoff

- Completed: contracts; Benchmark A (25 runs: `d̂`, mesh, increment,
  transient dt, material, density, speed, units ×3, controllers ×3, unload,
  stacked), Benchmark B (5 cases), Benchmark C (15 runs: 3 meshes with hard
  references and 2 lift runs, increments, transient, `d̂`); the acceptance
  decision and its implementation (record version 3).
- Status `characterized—limits documented`: T1–T6, T8–T14 pass as declared;
  T7 fails as declared with the cause traced to two upstream unit-dependent
  constants; the realized gap varies between .62 and .99 `d̂` across the
  sweeps (sensitivity documented). No application acceptance is selected, so
  the item cannot be `validated within stated scope` under the plan's rule.
- Decisions taken (user, 2026-09-13): `physical_balance_pass` as the force
  residual at 1e-6 of the peak external force (record version 3), the gap
  statistics reported, engineering accuracy outside the flag. Remaining:
  (1) RB-11/RB-12 should carry the two unit-dependence observations
  (`characteristic_force_density` scaling as `s^-1.5`, IPC's absolute 1e-4
  CCD clearance and its amplification by the emergency controller);
  (2) whether the controller's first-contact response to a CCD-truncated
  iterate (trim ramps to 65 536, gap pinned near `d̂`) deserves its own item
  — a conditioning/cost issue, not an accuracy one (the error stays below
  `c·d̂`); (3) whether the friction flag should get its own looser threshold
  (every friction step reads false at 1e-6 through the 0.2–0.3 % finite-lag
  mismatch; the solved-lag ratio is reported alongside).
- Next: `python3 tools/rb09/run_matrix.py --stage all` reproduces the matrix
  (the `n_refs` 2 runs take ~3 min each); `run_probe.py` the probe. Eligible
  items: RB-23, RB-06/RB-08, RB-11 (with these observations), RB-12.
- Plan row and README updated in this commit.

## Progress log

Append-only. Newest entry last.

- **2026-09-13 11:56Z** — Session start; baseline identity recorded; plan,
  RB-04/RB-10 records, diagnostics writer, form API and CCD code read.
- **2026-09-13 12:30Z** — Contract written (references, conditioning
  `c = 5.75 /m`, thresholds T1–T14) before any production run; pilot block
  run confirmed the scene mechanics (per-component Dirichlet, exact
  homogeneous state reproduced to 4e-4 in F).
- **2026-09-13 13:20Z** — Block matrix 22 runs, spring probe (coefficient
  convention `weight()` = `w·trim` corrected in the probe's reference; the
  hard-contact identity is an overshoot `k·g`), 20/20 checks.
- **2026-09-13 13:50Z** — mm unit dependence traced to IPC's absolute 1e-4
  CCD clearance and PolyFEM's `F0·L^1.5` tolerance; two controlled mm
  variants added; clamped matrix 15 runs; Fixed control run; analyzer fixed
  for the stacked interface gap and the unloaded endpoint; summary written.
- **2026-09-13 15:10Z** — Recommendation (1)–(3) for `physical_balance_pass`
  confirmed by the user; implemented as record version 3 with
  `contact.gap_statistics`, the spec tolerance and the regression; RB-04
  runner, RB-09 reruns (bit-identical endpoints) and the five smokes pass;
  the friction smoke's finite-lag mismatch measured at 0.2–0.3 % of the
  peak force.
