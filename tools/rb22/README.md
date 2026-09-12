# RB-22 high-order hexahedral collision surface: scene matrix

Scene matrix and runner behind [docs/rb-22-validation.md](../../docs/rb-22-validation.md).
Public inputs only: the RB-03 4×4×4 Q1 hexahedral unit cube
(`../rb03/hex-scene/hex-cube.mesh`) pressed onto the public semi-implicit slab,
and the upstream `data/contact/examples/3D/hex_tet` two-body example (one hex,
one tet cube) with a smaller drop, a stiffer hex, `dt=0.01`, 30 steps and a
**consistent** mass matrix — the example's `lump_mass_matrix: true` row-sum
lumps quadratic elements to zero/negative corner masses and every Q2 variant
blew up at the impact step with either tessellation (recorded in the RB-22
evidence, not a collision-surface effect).

```sh
python3 tools/rb22/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir \
    [--scenes q2-dof q2-max_order ...] [--threads 1] [--timeout 300] \
    [--compare /other/run/summary.json]
```

Every scene runs single-threaded by default (deterministic identity checks),
dumps the FE collision proxy through `POLYFEM_DUMP_COLLISION_PROXY`, and is
summarized in `summary.json`: exit status, first error line, the builder's
`Collision mesh from …` diagnostic (surface vertices, exact selector vs
interpolated displacement-map rows, faces), proxy statistics from the dumped
OBJ (used vertices, faces, closedness, edge-manifoldness, Euler characteristic,
zero-area faces, SHA-256), contact counts, Newton iterations, minimum distance,
wall time and the final solution's SHA-256 / max |u|. `--compare` adds
per-scene proxy/solution identity flags against another run (used against the
pre-change baseline binary built from `34aff1fb4`).

Scene names: `q<order>[s]-<tessellation>[-adaptive]` on the hex cube
(`s` = serendipity basis; tessellation `default`, `dof`, `max_order`;
`-adaptive` = classic adaptive barrier stiffness instead of semi-implicit) and
`hextet2-q<order>-<tessellation>[-adaptive]` for the two-body scene.
`q2-default`, `q2s-default`, `q3-default` and `hextet2-q2-default` are the
former silent/crashing configurations and must now stop with the named error;
`q3-dof` / `q3-max_order` must stop with the degenerate-face error until the
Q3 hexahedral basis is repaired (see the record).

`run_ab_smokes.sh <baseline bin> <candidate bin> <fresh dir>` runs the five
public semi-implicit smokes and the RB-03 Q1 hex scene single-threaded with two
binaries and compares dumped proxies and final solutions bit-for-bit.

These are completion, conformity and identity measurements, not physical
benchmarks; no accuracy claim is made for any of the scenes.
