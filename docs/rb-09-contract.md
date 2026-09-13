# RB-09 — Benchmark contracts (declared before the matrix ran)

Written 2026-09-13 at PolyFEM `b924c37e6` (binary built from `756070f44`,
identical `src/`), IPC `bb795446`, PolySolve `ee5b296a`, before any RB-09
production run. The tolerances below come from the references' conditioning,
not from observed solver output; a failed threshold stays failed unless a
separately justified revised protocol is recorded in the validation record.

## Quantities of interest

All in the record's internal units (force, length, energy; SI in the base
scenes: N, m, J). Peak stress/contact pressure is **not** a quantity of
interest here: it needs mesh-aware sampling and singularity handling that
this item does not define.

| Symbol | Definition | Source in the RB-04 record / output |
| --- | --- | --- |
| `R_z` | Support (reaction) force on the prescribed top face, z component | `reactions[0].full_dof_vector` summed over the body's DOFs |
| `L_z` | Integrated contact load: z sum of the barrier force on the body (and per obstacle: the obstacle DOF sums) | `forms[barrier-contact].gradient_force_units` |
| `ū_x` | Mean x displacement of the nodes of the reference face `x = 1` (block benchmark) | endpoint DOF vector with the VTU rest coordinates |
| `g_i` | Gap of every bottom-face node above the floor plane `z = -g0` (vertex–face contact with a flat obstacle: the exact gap is the vertical distance) | endpoint DOF vector with the VTU rest coordinates |
| `ḡ`, `g_min`, `g_max` | Mean/min/max of `g_i` over the contacting nodes (those carrying a nonzero barrier force) | derived |
| band coverage | Fraction of contacting nodes with `g_i/d̂ ∈ [√trim_lower, √trim_upper] = [.7071, .9487]` (the controller's squared-gap band `[.5, .9]·d̂²`) | derived |
| coefficient coverage | Active coefficient range against the batch floor/cap, zero and fallback counts | `contact.coefficient_range`, `batch_floor`, `batch_cap`, counts |
| cost | Newton iterations (all minimizes), minimize calls, restarts, wall time | `attempt_summary`, `termination`, runner |

## Benchmark A — analytical continuum: Neo-Hookean block on a frictionless floor

**Assumptions matched to the implementation.** PolyFEM's Neo-Hookean energy
`W = μ/2 (‖F‖² − 3 − 2 ln J) + λ/2 (ln J)²`, `P = μ(F − F⁻ᵀ) + λ ln J F⁻ᵀ`,
finite strain, P1 tetrahedra, frictionless rigid flat obstacle, quasistatic
(or slow transient). The block is the public unit cube (`scenes/semi-implicit/
cube.mesh`, `[0,1]³`, 4×4×4 lattice) above the public slab at `z = −g0`,
`g0 = .02`. Boundary conditions: `u_x = 0` on the face `x = 0` and `u_y = 0` on
the face `y = 0` (symmetry planes, `dimension` flags), `u_z = −δ(t)` on the
top face `z = 1` with `u_x, u_y` free, floor contact at the bottom. These
conditions are compatible with the homogeneous state below, so the exact
solution is in the P1 space of every mesh: mesh refinement changes only the
barrier's per-node force distribution, not the continuum error.

**Exact reference (hard contact).** Homogeneous uniaxial compression
`F = diag(l, l, l_z)` with `l_z = 1 − (δ − g0)/H`, the lateral stretch `l`
from `P_xx = 0`: `μ(l² − 1) + λ ln(l² l_z) = 0` (monotone scalar equation,
bisected to 1e-15), `R_z = A·P_zz = A[μ(l_z − 1/l_z) + λ ln J / l_z]`,
`ū_x = l − 1` on `x = 1`, `L_z = −R_z`. Implementation: `tools/rb09/reference.py`.

**Barrier-consistent reference.** The same state at `l_z = 1 − (δ − g0 + ḡ)/H`
with the measured mean gap `ḡ`: it isolates the barrier's gap error (the model
error) from the discretisation/solver error.

**Conditioning at the base endpoint** (`E = 1e7`, `ν = .45`, `δ = .25`,
`g0 = .02`, `l_z = .77`): `R_z = −2.994216e6`, `dR_z/dl_z = 1.7220e7`, so a
gap `g` shifts the reaction by `c·g` relative with `c = 5.751 /length`; the
lateral stretch sensitivity `dl/dl_z` is evaluated by the same module.
Predicted model error at `d̂ = 1e-3` if the gap sits in the band:
`0.41 %–0.55 %` of `|R_z|`.

**Declared thresholds** (`c` and `dl/dl_z` from the reference at the
run's own `E`, `ν`, `δ`; `n` = free DOFs):

| ID | Check | Threshold |
| --- | --- | --- |
| T1 | Force balance: `|L_z + R_z (+ inertia_z)| / |R_z|` | `≤ 1e-5` |
| T2 | Hard-contact error explained by the gap: `e_R = |R_z − R_ref|/|R_ref|` versus `e_pred = c·ḡ/H` | `|e_R − e_pred| ≤ .1·e_pred + 1e-5` |
| T3 | Barrier-consistent reaction error `|R_z − R_ref(ḡ)|/|R_ref|` | `≤ c·(g_max − g_min)/H + 1e-6` (a priori `≤ c·.2416·d̂/H + 1e-6` when the band holds) |
| T4 | Lateral displacement `|ū_x − (l(ḡ) − 1)|` | `≤ |dl/dl_z|·(g_max − g_min)/H + 1e-8` |
| T5 | `d̂` sweep rate: least-squares slope of `log e_R` vs `log d̂` over `{4e-3, 2e-3, 1e-3, 5e-4}` | `1 ± .15` |
| T6 | Mesh/increment sweeps: every run passes T1–T4; spread of `R_z` across the sweep | `≤ 2·c·.2416·d̂/H · |R_ref|` |
| T7 | Unit conversion (m → mm: lengths ×1e3, `E` ×1e-6, `ρ` ×1e-9, `d̂`/`δ` ×1e3): `|R_mm − R_m|/|R_m|`, `|ḡ_mm/d̂_mm − ḡ_m/d̂_m|` | `≤ 1e-6`, `≤ 1e-6`; iteration counts reported (equal expected) |
| T8 | Transient (ImplicitEuler) vs the quasistatic barrier-consistent reference at the same `δ`: `|R_tr − R_ref(ḡ)|/|R_ref|` | `≤ 1e-2` at every `dt`, non-increasing with `dt`; observed order from the three finest `dt` reported |
| T9 | Load/unload (`δ = .25·t` to `t = 1`, back to 0 at `t = 2`): every unloading step versus the loading reference at equal `δ` (T2); at `t = 2` | T2 on each step; `active_count = 0`, `|R_z| ≤ 1e-6·max|R_z|` |
| T10 | Stacked blocks (coupled contact, same material): floor load, interface load, `R_z` | each interface T1; each block T2–T4 |

Material contrast (`E ∈ {1e6, 1e7, 1e8}`), density (`ρ ×10`, transient) and
approach speed (`×½, ×2`, transient, same `δ_final`, same number of steps)
are **physical** changes checked against their own exact reference (T2–T4,
T8); the unit conversion T7 must reproduce the *same* physical state.
Controllers: `semi_implicit` (default band), classic `adaptive`, and Fixed at
the classic run's endpoint stiffness — compared on T1–T4 and cost; no
production default is selected here.

## Benchmark B — spring/contact on the production form

One 2D point above a floor edge (RB-02/RB-10 probe mesh) under a
semi-implicit `BarrierContactForm` with a synthetic spring of stiffness `k`
anchored at `x_p` below the floor (driving Hessian `k·I`). Production flow
per step: Newton on `½k|x − x_p|² + w·trim·κ·b(d², d̂²)` with the form's
`post_step` controller, then `update_quantities` and the between-steps
`update_barrier_stiffness` refresh. Reference: the exact root of
`k(g − g_p) + w·trim·κ·b'(g)` on `(0, d̂)` (bisection to 1e-15 on the realized
`w·trim·κ`), hard contact `g = 0`, force `k|x_p|`.

| ID | Check | Threshold |
| --- | --- | --- |
| T11 | Equilibrium gap vs the scalar root; spring force vs barrier force; hard-contact force error `= k·g` | `|g − g_ref| ≤ 1e-10·d̂`; `1e-10` relative; identity to `1e-12` relative |
| T11b | Sweeps `k ∈ {1, 1e2, 1e4}`, `d̂ ∈ {1, .1, .01}` (`L = 1`): gap/`d̂` stays in the band after the between-steps controller | reported (controller property; not an acceptance) |

## Benchmark C — public clamped-top cube (no continuum solution)

The public `quasistatic-semi.json` / `transient-semi.json` (top face fully
clamped laterally). Reference: the **same-mesh hard-contact** solve — the
bottom face nodes get `u_z = 0` (`dimension [false,false,true]`, bilateral)
with the floor gap `g0` removed from the loading (`u_z(top) = −(δ − g0)`), valid
only where every bottom node's reaction is compressive (tensile count
reported; a nonzero count invalidates the reference at that node set). Its
conditioning `c_h = |dR_z/dg|/|R_z|` is measured by lifting the constrained
plane by `+5e-4` (a second reference run per mesh).

| ID | Check | Threshold |
| --- | --- | --- |
| T12 | Barrier vs same-mesh hard reference: `|e_R − c_h·ḡ|` | `≤ .1·c_h·ḡ + 1e-5` (the model error is explained by the mean gap) |
| T13 | Mesh sequence `n_refs ∈ {0,1,2}`: the hard reference's `R_z` convergence (Richardson order from the three meshes) and the barrier solutions' `R_z − R_hard(h)` | reported; barrier error obeys T12 on every mesh |
| T14 | Increment `dt ∈ {.25, .125, .0625}` (quasistatic) and transient `dt` sweep | quasistatic spread `≤ 2·c_h·.2416·d̂·|R|`; transient T8 against the quasistatic barrier endpoint |

## Exclusions

No friction (RB-10 owns it), no private scene, no Teseo, no Q2+ elements
(RB-22/23), no impact or high-speed case, no golden replacement, no
`contact_2d` cube-on-floor claim (its GCP path is not a semi-implicit
smoke; investigated separately). Nothing here selects `physical_balance_pass`
or an application acceptance — the measured envelope is reported so that the
user can select one. Fixed/adaptive controls are comparisons, not defaults.
