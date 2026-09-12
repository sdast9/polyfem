# RB-03 — Collision/FEM coordinate contract

Date: 2026-09-08. Original characterization, followed by an exact-selector
indexing repair (validated within its stated scope) and, on 2026-09-11, the
selected interpolated-stencil stiffness (local condensation with a
gap-normalized direction fallback; see the [last section](#selected-interpolated-stiffness--2026-09-11)).
See [validation](rb-03-validation.md).
This does not certify the semi-implicit model or arbitrary collision meshes.

## Coordinates and the established chain rule

Let n be the number of FEM basis nodes **including appended obstacle nodes**,
p the number of input collision-proxy vertices, m the number of included surface
vertices, and d the spatial dimension. Node-major flattening is
`[u0x,u0y,(u0z),u1x,...]` (`utils::flatten/unflatten`).

The production builder is now in
`src/polyfem/varforms/NonlinearElasticVarForm.cpp`, not `solver/forms/`.
It constructs a vertex interpolation matrix T of size p by n, and IPC constructs
an inclusion matrix S of size m by p. Write B = (S T) tensor I_d (md by nd).
When no T is supplied, IPC uses the selection map directly. The actual geometry is

```
y(u) = S X_proxy_rest + B u
u(z,t) = c(t) + Q z
y(z,t) = S X_proxy_rest + B c(t) + B Q z.
```

Here the first equation is understood in flattened coordinates; S also acts on
rest-position rows. B and Q are dimensionless; u, c, y and z have length units
for the selection fixtures. General constraint bases can change the units of z.
The rest proxy is selected, **not** interpolated from FEM rest coordinates.
Rigid-motion reproduction additionally requires compatible rest positions and
partition of unity; arbitrary external weight files need not satisfy either.

`ContactForm::compute_displaced_surface` invokes `displace_vertices(unflatten(u))`.
IPC's `to_full_dof(g)` is B^T g, and `to_full_dof(H)` is B^T H B.
`BarrierContactForm` uses these for its analytical gradient and Hessian.
For frozen coefficients and a constant map, therefore,

```
g_u = B^T g_y,        H_u = B^T H_y B
 g_z = Q^T g_u,        H_z = Q^T H_u Q
 g_u dot du = g_y dot (B du).
```

For objective units Q_obj, gradients have Q_obj/length and Hessians
Q_obj/length^2. A contact force is minus its gradient. Prescribed components
remain in the full contact gradient as reactions; free residuals omit them.
Contact work on prescribed motion is `-g_u dot dc/dt`; the objective derivative
at fixed free coordinates has the opposite sign. This is a contact contribution,
not total equilibrium or trajectory energy balance.

`NLProblem` uses `Q2_` and `Q1R1iTb_` for Q and c, or the equivalent direct
projection methods for one projectable BC form. `full_to_reduced` subtracts c;
`full_to_reduced_grad` does not. During AL, full coordinates remain available;
this audit changes neither AL feasibility preparation nor final stopping rules.
The probe exercises the actual reduced `NLProblem`/`BCLagrangianForm` path at a
nonzero prescribed obstacle target, in addition to analytical Q fixtures.

## Which index space is “full”?

IPC's `to_full_vertex_id` reverses S only: it returns a **proxy vertex ID**,
not a FEM basis-node ID when T is nonidentity. Likewise `full_num_vertices()`
and `full_ndof()` count input proxy vertices/coordinates. The derivative result
size for a rectangular T instead follows T's columns. The effective IPC header's
`to_full_dof` return-size prose does not describe this rectangular case accurately;
the implementation uses B^T and the probe checks its actual dimensions.
`BarrierContactForm::per_vertex_potential` distributes energy over input proxy
vertices; its output must not be advertised as nodal FEM energy for interpolation.

External proxy loading reads OBJ geometry and HDF5 `weight_triplets` (rows,
cols, values, with shape attribute), then permutes weight columns using
`in_node_to_node`. The builder appends obstacle geometry after the FEM proxy and
adds identity entries from each appended proxy row to its appended FEM-system
node. Consequently the obstacle's proxy ID and system-node ID generally differ.
The collision filter intentionally uses proxy IDs to reject obstacle–obstacle
pairs. That use does not need a FEM ID conversion.

## Reproduced stiffness mismatch — pre-repair baseline

The driving Hessian is the weighted elastic plus enabled inertia Hessian from
`SolveData`, in full system displacement coordinates. At refresh, the form saves
it and the mapped surface positions, then clears the stencil coefficient cache.
For each new stencil, `assign_collision_stiffness` obtains local Hessian entries
by `to_full_vertex_id`, treating proxy IDs as system-node IDs. Out-of-range
positive IDs are skipped as presumed obstacles. That is correct for identity
indexing/selection, but does not implement the T mapping.

With zero local masses the effective IPC function forms a unit surface normal
stencil vector w and returns `w^T H_local w`. The form divides this by dhat^2,
then applies the existing sign/fallback, weight, cap and trim rules described in
[RB-02](rb-02-contract.md). These policies are unchanged here.

For an orthogonal permutation P, the required coordinate transformation is
unambiguous: H_surface = P H_system P^T. The tiny fixture gives actual kappa
71.6666667 versus required 26.6666667. The identity negative control matches
71.6666667. This was a reproduced indexing defect at characterization. The exact-selector
repair below now corrects it without selecting an interpolation model or
constructing a dense production map.

For interpolation, B^T H_surface B = H_system generally has no solution: FEM
energy can vary along ker(B), although surface coordinates do not. Nor is
`B H_system B^T` automatically an energy Hessian in surface coordinates. A
force pullback B^T and a displacement lift are different operations.

## Concrete interpolation alternatives — characterized 2026-09-08

The table below is the original comparison; the selection made on 2026-09-11
is recorded in the [last section](#selected-interpolated-stiffness--2026-09-11).

The fixture averages two FEM nodes into the contact point. Endpoint stiffnesses
are 10 and 20; the averaged point's node stiffnesses are 100 and 400, each
isotropic. All values are in consistent objective/length^2 units. dhat=1,
form weight=trim=1 and a single contact leave cap and unit conversion neutral.
Its normalized stencil is proportional to `[0,-.5,0,-.5,0,1]`.

| Definition | Curvature | Meaning / limitation |
| --- | ---: | --- |
| Current proxy-ID sampling | 71.6667 | Incorrect index identification; no interpolation contract |
| `w^T B H B^T w` | 88.3333 | FEM direction B^T w; does not generally produce surface displacement w |
| `(L w)^T H (L w)`, B L = I, minimum Euclidean norm L | 338.3333 | One chosen displacement lift; introduces a metric for unresolved motion |
| `w^T (B H^-1 B^T)^-1 w` | 218.3333 | Minimum elastic energy for prescribed surface displacement; requires SPD H and independent map rows |

The probe uses an explicit small right inverse and dense inverses only for this
SPD analytical fixture. These are **not production algorithms**. No pseudoinverse,
projection, positive minimum, new coefficient law or mesh-mode disablement was
implemented. Indefinite/singular H, dependent proxy rows, and fixed obstacle
surface directions can invalidate the inverse-based option. Production elastic
H is not guaranteed SPD; constrained versions require a separately specified
admissible displacement space. See RB-02's zero/indefinite curvature cases.

At fixed geometry these choices scale the entire barrier energy and force by
kappa; [the result](../tools/rb03/results-20260908.json) records both quantities.
The existing force/Hessian chain rule can pass for **each** frozen coefficient,
so derivative agreement alone cannot choose among these physical definitions.
A decision must specify normalization, admissible/fixed motion, treatment of
rank deficiency and indefiniteness, and coefficient lifecycle before implementation.
An exact permutation repair is independent; no approval gate is claimed for it.

## Supported evidence and remaining limits

Measured: identity; selection with an omitted vertex; permutation; rectangular
interpolation; the actual external OBJ/HDF5 builder; appended fixed/moving obstacle
nodes; the actual reduced BC projection; virtual work; full derivatives; rigid
translations and finite rotations at frozen coefficients; contact action–reaction.
The fixed-stiffness contact formulation is a derivative control.

Source paths also offer sampled spline boundaries, `max_order`, and collision
proxy tessellation via max edge length. They feed displacement maps into the
same machinery, but **no high-order FEM assembly, tessellation, spline, periodic
contact, 3D contact, multi-geometry transform or arbitrary remeshing validation**
is claimed. External loading has source TODOs for per-geometry transformations,
unit scaling and ordered higher-order nodes. These paths are not disabled.
The tested external builder uses explicit identity transformation defaults and
identity input node order; input-order remapping is source-traced only.

The obstacle coefficient probe additionally returns 83.3333 from a FEM-only
100 I Hessian: one obstacle proxy ID aliases a FEM node and another is skipped.
This exposes the same ID mismatch; it does not establish a uniquely correct
interpolated obstacle curvature. The derivative fixtures use synthetic constant
H, including placeholders where stated, not an assembled continuum Hessian.
RB-03 remains characterized—decision pending for the interpolation model.
The exact-selector indexing repair below is validated within its stated scope.
RB-04 may use that scope and the proven chain rule, while retaining the
interpolated-stiffness and physical-validation limits.


## Current exact-selector implementation

The original counterexamples above describe the pre-repair baseline. The bounded
repair uses IPC's const `displacement_map()` accessor, whose rows are included
collision vertices and columns are system displacement nodes. It includes both
selection and supplied interpolation; `to_full_vertex_id` still returns proxy
IDs and remains appropriate for proxy-indexed geometry/filter operations.

Only stencil maps with distinct exact unit-selector rows use mapped system IDs
for curvature extraction. This implements the selector/permutation transform
without a dense surface Hessian. System nodes outside the supplied Hessian
contribute zero blocks, preserving the existing FEM-only obstacle convention.
The map lookup is constructed only in semi-implicit mode and assumes the mesh
map remains fixed during form use. Force/Hessian chain rules are unchanged.

Until 2026-09-11 a stencil with any non-selector or duplicate row retained
its entire legacy sampling path, including its obstacle rows. That path is now
replaced by the definition below; see the repair and interpolation stages in
the validation record for acceptance checks and publication status.

## Selected interpolated stiffness — 2026-09-11

The user chose, from the alternatives above, **local condensation with the
gap-normalized force direction as fallback** ("(ii) with (i′)"), after the
2026-09-11 decision to retain the production adaptive-barrier law. The
selection is the production definition stated precisely and extended, not a
new model:

*The production law reads the frozen driving Hessian's local block for the
stencil's nodes — every other DOF held fixed — along the unit contact
direction `w` (`wᵀ H_local w`). For a stencil whose surface vertices are
interpolated from parent nodes `P` with map rows `B`, the same quantity is the
stiffness felt by a rigid stencil displacement `w` when every DOF outside `P`
is fixed and the parents settle to minimum energy:*

```
K_s = (B H_PP⁻¹ Bᵀ)⁻¹          kappa_raw = wᵀ K_s w
```

For exact selectors there is no freedom to settle and `K_s = P H Pᵀ` exactly,
so selector stencils keep the repaired extraction bit for bit (the two code
paths coincide; the five public smokes and the `adaptive` mode are untouched).
Conventions carried over from the selector contract:

- Parent nodes outside the Hessian or with an identically zero diagonal
  block (obstacle and prescribed proxies) are fixed: they carry no motion and
  are excluded from `P`. A surface vertex left with no movable parent is
  dropped from the constraint and contributes zero, exactly as its zero block
  does for a selector stencil. A stencil with nothing movable returns the
  zero sentinel and goes through the RB-18 F2/F7 chain.
- **Fallback (i′).** When `H_PP` is not SPD (LLT fails), or the kept rows of
  `B` are dependent (column-pivoted QR rank below the row count at relative
  threshold 1e-10: duplicate proxy vertices, two rows on one node), or the
  condensed matrix is not finite, the curvature is the gap-normalized force
  direction
  `|w_K|⁴ (wᵀ B H_PP Bᵀ w) / (wᵀ B Bᵀ w)²`
  — the energy along `u = Bᵀ w` scaled so that the stencil's own motion
  along `w` is unit; it equals `wᵀ H w` for selectors and the condensed value
  for a single interpolated vertex, and never needs an inverse. Its result
  then enters the same RB-18 chain (previous κ → |·| → max|H|/d̂² → batch
  floor/cap) as any other raw curvature.
- The quadratic forms are evaluated by the toolkit's own
  `ipc::semi_implicit_stiffness` on embedded `dim·n_verts` matrices, so the
  contact direction (AUTO distance type for RB-21 parent candidates, the
  subfeature for built stencils) has a single definition.
- Cost: one dense LLT of size `dim·|P|` (≤ 12 for a hex face centroid, ≤ 48
  for a four-vertex stencil of centroids, ~120 for P3 sampled proxies) per
  fresh estimate; under RB-20 only contacts born since the published endpoint
  are estimated. Parent lists are stored once per non-selector map row.
- Diagnostics: `diagnostic_state()` reports `interpolated_condensed_count`
  and `interpolated_direction_count` for the last refresh batch, with a debug
  log line at each refresh.

Measured on the fixtures of this contract (unchanged geometry, `dhat` = trim =
weight = 1): the averaged-point fixture now reads **218.3333** (the
minimum-energy lift of the table above; legacy 71.6667), the mixed
FEM/obstacle proxy **133.3333** (the averaged FEM point condensed to
`(.25/100 + .25/100)⁻¹ = 200` on its `1/1.5` share of `w`; obstacle rows
zero; legacy 83.3333), a duplicate-row stencil **45** and an indefinite parent
block **123.75** through the fallback. The gap-normalized direction reads
198.75 on the averaged-point fixture, 9 % below the condensed value; for a
single vertex smeared over `k` decoupled parents of stiffness `h` both give
the parallel-spring value `k·h`, where the raw force direction `wᵀ B H Bᵀ w`
reads `h/k` and legacy sampling read a zero block (a hex centroid's proxy ID
lies outside the FE node range).

Limits retained: the "every other DOF fixed" local-block convention is
production's, not a global condensation (RB-13's `K_eff`); the fallback for
dependent rows is a heuristic bounded by the batch cap (a vertex duplicated on
an edge endpoint reads 630 on the unit fixture); `H_PP` is the driving Hessian
as provided (elastic plus inertia, unprojected), so a crushed block falls to
(i′) then the RB-18 chain; Q2+ hexahedral boundary faces are skipped by the
default extraction itself (no proxy is built), which is outside this item; no
private scene, Ballburst or Teseo run, and no physical certification of the
retained law.
