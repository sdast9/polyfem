# Analytical solver integration tests

`test_simulation_oracles.py` generates small P1 tetrahedral scenes and invokes
the real PolyFEM CLI with one solver thread. CTest registers it as
`simulation_oracles`. The script requires only Python's standard library and the
built solver. For a repeat with retained input files, run manifests, solver
logs and measured values, run this from the repository root with a fresh
output path.

```sh
python3 tests/test_simulation_oracles.py \
  --binary /absolute/path/to/PolyFEM_bin \
  --expect-commit FULL_SOLVER_COMMIT \
  --output /absolute/path/to/unused-output-directory
```

Repeat `--case NAME` to select cases. The commit check requires a clean
matching build. Every subprocess must report the exact executable hash and
build identity. Without `--output`, the test uses a temporary directory.

## Reference tetrahedron

The vertices are the origin, unit X and Y, and unit Z, in meters. The base is
fixed and the sloping face constrains y and z. The only free degree of
freedom is the apex x displacement `q`. A second tetrahedron, where used,
is translated 3 m in x. Contact and gravity are disabled.

With the reference length `L = 1 m`, the volume is `V = L³/6`. The only
strain is engineering shear `gamma = q/L`. The stress has
`sigma_xz = sigma_zx = G gamma`, the energy density is `G gamma²/2`, and the
apex stiffness is `K = G V/L²`. For isotropic linear elasticity,
`G = E / (2(1 + nu))`. The gradients of the shape functions at the origin,
X, Y and Z are `(-1,-1,-1)/L`, `e_x/L`, `e_y/L` and `e_z/L`.
PolyFEM's `elastic_forces` are the negative potential gradient, so the
four nodal forces follow directly from `-V sigma grad(N_i)`.

## Passing cases

- `isotropic_ab_ab`, `isotropic_ba_ba`, `isotropic_ab_ba` and
  `isotropic_ba_ab` permute geometry and material arrays independently.
  Body IDs 17 and 53 have `E = 30 kPa` and `50 kPa` at `nu = 0.25`. Apex
  point loads of `+24 N` and `-30 N` give `+12 mm` and `-9 mm` and
  stresses of `+144 Pa` and `-180 Pa`, respectively. The test selects
  each reference vertex by coordinate and checks the exported body ID,
  displacement, stress, energy and forces there.
- The mixed Hooke/isotropic cases independently permute geometry and material
  arrays. In the prescribed-shear cases, both bodies move 10 mm at the apex
  and give shear stresses of 120 Pa and 200 Pa and apex elastic forces of
  -20 N and -33.333333333 N. The two-Hooke cases use an aligned first tensor
  and a 90° frame on the second. Its local `C_44` is 26 kPa and its global
  XZ shear modulus is 20 kPa, so the material output, stress and reaction
  distinguish the tensor coefficients from their axes.
- `two_face_traction` applies x tractions of `120/sqrt(3) Pa` on the
  sloping face and `-60 Pa` on the `x = 0` face. A uniform traction on
  a P1 face gives `T area/3` at each corner. The areas are
  `sqrt(3)/2 m²` and `1/2 m²`, so the integrated x loads in vertex
  order are `(-10, 20, 10, 10) N`. The expected apex displacement is
  `5 mm`, the shear stress is `60 Pa`, and the base reaction sums to
  `(-30, 0, 0) N`.
- `consistent_mass` and `lumped_mass` independently verify Implicit Euler
  through five steps of `h = 1/8 s` from rest. The `x = 0` traction
  `288 t Pa` gives apex load `F(t) = 48 t N`. For density
  `rho = 120 kg/m³`, exact integrals of `N_i²` and `N_i N_j` are
  `V/10` and `V/20`. The consistent scalar mass has diagonal
  `m = 2 kg` and off-diagonal `1 kg`. Row-sum lumping gives
  `m = 5 kg` and zero off-diagonal. The one-DOF update is
  `q_next = [m (q + h v) + h² F_next] / (m + h² K)`.
  `v_next = (q_next - q)/h`, and `a_next = (v_next - v)/h`.
  This predicts each vertex's inertia force `-M_ia a_next`, including
  the nonzero x force on clamped vertices with consistent mass.
- The nodal load cases use the same apex load, with additional steps of
  `1/16 s` and `1/4 s`. The prescribed-support cases move all three base
  vertices by `d(t) = 0.02 t² m` and repeat the same free-DOF load as a
  nodal force and a surface traction. The elastic force is
  `K (q - d)` and the apex inertia includes the three off-diagonal
  contributions `M_ai a_base`. The first two discrete base accelerations
  are 0.02 and 0.04 m/s² for Implicit Euler from rest. The quasistatic
  nodal load gives `q = F/K` at both steps.
- `consistent_restart` and `lumped_restart` use the same analytical
  trajectory for the original run and a fresh CLI process started with
  `--checkpoint state_2.hdf5`. They check the global PVD times and VTM
  volume links and every field, including inertia, at the repeated
  checkpoint frame and at each newly solved step. The `_sync` cases
  exercise the synchronous writer with the same oracle. The resumed file
  indices start at 2 and checkpoints continue at 3. The PVD retains
  the original time origin and still lists steps 0 and 1. Those files
  belong to the original run directory.
- `initial_prescribed_inertia` sets the initial apex velocity to 0.1 m/s
  and acceleration to -0.04 m/s². The initial inertia is +0.08 N at the
  apex and +0.04 N at each clamped vertex with consistent mass. The next
  step follows the usual Implicit Euler update with the specified velocity.
- The static-frame cases keep the frame constant across both quasistatic
  increments with a `24 N` apex load delivered by surface traction.
  The omitted and identity axes both use `G_zx = 12 kPa`.
  A 90° rotation about z gives `G_yz = 5 kPa` in the same global shear.
  The expected apex displacements are `12 mm` and `28.8 mm`.
- The rotating cases use rows `(cos(theta), -sin(theta), 0)`,
  `(sin(theta), cos(theta), 0)` and `(0, 0, 1)` with `theta = pi t/2`.
  The moduli alternate between 5 and 12 kPa at odd and even seconds.
  A nonzero start time exercises the simulation clock. A contact-enabled
  case selects the nonlinear form with no active barrier on the mesh.

Forces are compared as vector components, with zero transverse and fixed
displacement components checked explicitly. Absolute tolerances are
`2e-8 m`, `2e-7 m/s`, `2e-6 m/s²`, `3e-4 Pa`, `2e-5 J/m³`,
`2e-8 N` for the applied force and `5e-5 N` for elastic and inertial
forces. The tolerances allow cross-platform assembly differences and
the mixed-law convergence threshold while remaining far below the case
separations. IDs compare exactly and PVD times use `1e-12 s`.

For the surface-traction transient, the first-step apex displacements
are `2.819548872 mm` with consistent mass and `2.586206897 mm` with
lumped mass. At step 5, the reference displacements are
`14.999780576 mm` and `15.002761425 mm`, respectively. The matching
base reactions are `-89.998902878 N` and `-90.005522850 N` in x.
The fixtures cover straight P1 tets, two constant mass policies
and one analytical time integrator.

## Baseline regression measurements

```sh
python3 tests/test_simulation_oracles.py \
  --binary /absolute/path/to/pre-fix/PolyFEM_bin \
  --expect-commit 4c5c45aa40f9350e4135ff0f1cdd811072395003 \
  --output /absolute/path/to/unused-reproduction-directory \
  --case quasistatic_nodal_load --case dynamic_nodal_load \
  --case mixed_hooke_first --case mixed_hooke_last --case rotating_frame \
  --case consistent_restart --case lumped_restart
```

These cases now run by default. The command above reproduces their failures
on the original published solver. The reverse-order mixed-material control
also passes on that revision.

| Case | Analytical expectation | Observed |
| --- | --- | --- |
| `quasistatic_nodal_load`, step 1 | `u_x = 12 mm` with `+24 N` | `24 mm`. Exported applied load is `+24 N` and elastic force is `-48 N`. |
| `dynamic_nodal_load`, step 1 | `u_x = 2.819548872 mm` with `+6 N` | `183.270676692 mm`. The reported load is `+6 N` and the equilibrium terms correspond to `390 N`. |
| `mixed_hooke_first` | Body 17 reports `C_44 = 12 kPa` and `u_x = 12 mm`. | `20 kPa` and `7.2 mm`. |
| `mixed_hooke_last` | The same Hooke parameters and load survive geometry reversal. | `12 kPa` and `12 mm`. |
| `rotating_frame` | The `24 N` apex load gives `28.8 mm` at 90° and `12 mm` at 180°. | `12 mm` at both frames. Apex elastic force was `-10 N` in each frame against `-24 N` expected. |
| `consistent_restart` | The restored apex inertia at 0.25 s is `-0.043416813 N`. | `+0.404319068 N`. |
| `lumped_restart` | The restored apex inertia at 0.25 s is `-0.228299643 N`. | `+1.055885850 N`. |

**Linear transient nodal load.** The original
`LinearElasticVarForm::solve_transient_linear` called
`RhsAssembler::set_bc` before and after the time-integrator scaling.
Both calls added the free nodal Neumann term. The first call also supplied
surface Neumann faces and the second supplies Dirichlet nodes.
`RhsAssembler` visited its stored nodal Neumann list in both calls.
This doubled the quasistatic RHS and gave the numerical factor
`(h² + 1)/h² = 65` at `h = 1/8 s` in dynamics. The default
traction controls supply the same free-DOF load and match the analytical
solutions. The repair applies the boundary conditions in one call and
keeps prescribed displacements in units of length after forming the
mass-weighted dynamic RHS.

**Mixed-law material parameters.** `AllElasticMaterials::add_multimaterial`
forwarded the second body's numeric `E` and `nu` to the Hooke assembler.
`HookeLinearElasticity::add_multimaterial` stored a single stiffness tensor
and overwrote it with the isotropic constants. Geometry reversal made
the Hooke tensor the final write. The repair binds each Hooke element
to its material tensor and frame. The prescribed-shear stresses and
reactions distinguish the moduli even when force-controlled solutions
would balance the same load.

**Time-dependent Hooke frame.** The linear solve assembled stiffness
once at time 0. The elastic force form was constructed at `t0 + dt` and
kept that first-step frame. Pointwise Cauchy stress used the saved sample
time. The repair evaluates expression-based frame stiffness and force
output at the solve time. Explicit numerical and absent frames retain
one stiffness assembly per linear schedule.

**Repeated checkpoint frame.** The initial frame after resume contains
the restored acceleration `a_2`. Its physical inertia force is
`-M a_2`. The newly constructed inertia form used the next-step
predictor, `q_2 + h v_2`, and evaluates the restored position against
that predictor. This gave `+M v_2/h` at the repeated frame. The repair
reports the initial or restored acceleration directly for inertia
output. The integrator continues to build the predictor for the next
solve from the original history.
