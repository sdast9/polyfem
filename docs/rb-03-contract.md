# RB-03 — Collision/FEM coordinate contract

Date: 2026-09-08. Characterization, with an unresolved stiffness definition and
an unrepaired indexing counterexample. See [validation](rb-03-validation.md).
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

## Reproduced stiffness mismatch

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
71.6666667. This is a reproduced indexing defect, **not repaired** in this
characterization stage. An exact selector/permutation repair can be pursued
without choosing an interpolation model; it must preserve full/reduced and
obstacle indexing, and avoid dense map reconstruction on production meshes.

For interpolation, B^T H_surface B = H_system generally has no solution: FEM
energy can vary along ker(B), although surface coordinates do not. Nor is
`B H_system B^T` automatically an energy Hessian in surface coordinates. A
force pullback B^T and a displacement lift are different operations.

## Concrete interpolation alternatives — no selection made

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
RB-03 remains characterized—decision pending, with the indexing repair pending
and the interpolation model unresolved. RB-04 may use the proven chain rule,
but must retain this limit on stiffness/physical claims.
