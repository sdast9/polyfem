# RB-09 reference benchmarks and refinement envelope: references, matrix, analysis

Tools behind [docs/rb-09-validation.md](../../docs/rb-09-validation.md); the
benchmark definitions and the thresholds declared before the matrix ran are
in [docs/rb-09-contract.md](../../docs/rb-09-contract.md). Public inputs only
(`scenes/semi-implicit/cube.mesh`, `slab.obj`, the two frictionless smokes),
copied into every run directory and never edited; nothing here runs Teseo
or a private scene.

```sh
# analytical references (no solver): exact Neo-Hookean uniaxial state, conditioning
python3 tools/rb09/reference.py

# Benchmark A (block, 25 runs incl. the Fixed control) and C (clamped public smokes, 15 runs)
python3 tools/rb09/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/block --stage block
python3 tools/rb09/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/block --stage block \
    --only block-fixed --fixed-kappa <trim_or_global_stiffness of block-adaptive's last record>
python3 tools/rb09/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir/clamped --stage clamped

# Benchmark B: the spring/contact probe on the production form (compiled with the unit_tests flags/link line)
python3 tools/rb09/run_probe.py --build build --output /absolute/fresh/dir/spring-probe

# per-endpoint quantities, references and the per-run checks T1-T4 (writes endpoints.json)
python3 tools/rb09/analyze.py --runs /absolute/fresh/dir/block
python3 tools/rb09/analyze.py --runs /absolute/fresh/dir/clamped
# sweep-level checks T5-T10, T12-T14, the comparison tables and the compact JSON
python3 tools/rb09/summarize.py --endpoints /absolute/fresh/dir/block/endpoints.json \
    --clamped /absolute/fresh/dir/clamped/endpoints.json --spring /absolute/fresh/dir/spring-probe/probe.json \
    --out tools/rb09/results-YYYYMMDD.json
```

`reference.py` — PolyFEM's Neo-Hookean form in homogeneous uniaxial
compression with traction-free lateral faces (`P_xx = 0` bisected to
1e-15): reaction, lateral stretch, energy density, the reaction's
sensitivity to a residual gap; and the exact 1-DOF spring/clamped-log
equilibrium.

`run_matrix.py` — Benchmark A scenes: the unit cube on the slab (initial gap
.02) with symmetry planes (`dimension` flags), the top face prescribed in z
only, μ = 0, `E 1e7`, `ν .45`, `d̂ 1e-3`, `δ = .25·t`; sweeps of `d̂`, `n_refs`,
load increment, transient `dt`, `E`, density, approach speed, a raw m → mm
conversion (plus two variants with the dimensional solver constants
converted), classic `adaptive` and Fixed controls, load/unload (`tend 2`),
and two stacked blocks. Benchmark C scenes: the public clamped-top smokes
with `n_refs`/`dt`/`d̂` variations and their same-mesh hard-contact references
(bottom face `u_z = 0` bilateral, contact off, gap removed from the loading;
`plane_lift 5e-4` measures the reference's conditioning). Every run keeps
its scene, input hashes, command, exit status, log, VTU output, `nodes.txt`
(FE nodes in the record's basis order) and the RB-04 diagnostics; existing
`run.json` files are reused, never rerun.

`analyze.py` — from the RB-04 record and `nodes.txt` alone: support force on
the top face, barrier force on the body / per obstacle / per interface,
inertia force, per-node bottom gaps (vertex–flat-obstacle: the vertical
distance is exact), band coverage, coefficient range against the batch
floor/cap, lateral displacement of the free face, energies, cost; block
endpoints get the hard-contact and barrier-consistent references and T1–T4
(stacked: the per-block effective gap `(g_floor + g_interface)/2`); hard
references get the tensile-reaction count.

`spring_probe.cpp` — one 2D point on a linear spring above a floor edge
under the production semi-implicit `BarrierContactForm`, driven through
the production step flow (Newton with the form's line-search hooks and
`post_step`, `update_quantities`, `update_barrier_stiffness`); the exact
scalar root on the realized `w·trim·κ_s` (the form's `weight()` already
carries the trim), the spring/barrier force identity, the analytical
gradient and the hard-contact overshoot `k·g`. Five `(k, d̂)` cases; the
trace records the first iterate's gap and the trim history.

Compact results of the 2026-09-13 matrix: `results-20260913.json`.
