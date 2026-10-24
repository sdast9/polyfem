# Frozen natural configuration for 3D growth/remodeling

`GrowthNeoHookean` adds an opt-in 3D constitutive law for an external growth or remodeling integrator. It evaluates energy per original reference volume,

\[
\Psi(F)=\det(G)\,W_{NH}(F G^{-1};E,\nu).
\]

The symmetric positive-definite matrix `G` describes an effective natural configuration in material coordinates. Incompatible spatial fields can create residual stress; the mesh stays in its original reference configuration. This material does not itself evolve `G`, collagen mass, damage, or geometry.

```json
{
  "type": "GrowthNeoHookean",
  "E": "tissue_E.txt",
  "nu": 0.45,
  "g11": 1.0,
  "g22": 1.0,
  "g33": "natural_z.txt",
  "g12": 0.0,
  "g13": 0.0,
  "g23": 0.0
}
```

Diagonal components default to one and off-diagonal components to zero. Components use the existing scalar expression/per-element file mechanism and element binding/count checks. `G` is checked for finiteness and positive definiteness at evaluation, and 2D meshes are refused. The law participates in the current variational formulation, assembler factory, multimaterial dispatch, material-parameter validation, and automatic energy derivatives/stress output. Existing material defaults are unchanged.

An external damage law can supply a degraded positive `E`; `G` and damage then remain independent. Combine this law with an ordinary `NeoHookean` child in `MaterialSum` for a nongrowing bonded constituent. In mixed meshes, give implant-free cells only the tissue law, and implant cells the composite law using separate body selections. Do not emulate absence of a constituent with an invalid zero Young's modulus.

This is a frozen constitutive interface, not a complete constrained-mixture or fracture implementation. Cohort kinetics, deposition rules, mass balance, healing, fracture energy and history transfer are the external integrator's responsibility. For transient use, consistent density/inertia and growth-source treatment are also required.

Validation on macOS arm64, 2026-10-24:

- `[growth],[input_validation],[assembler_utils]`: 436 assertions in 30 test cases passed. New tests cover identity recovery, stress-free natural deformation, constrained energy, rigid-rotation invariance, original-volume scaling, finite-difference gradient/Hessian agreement, invalid natural states, per-element binding, and JSON defaults for direct/composite use.
- Two existing variational-factory/state tests: 56 assertions passed.
- Five semi-implicit smoke scenes passed; all 55 VTK collection/geometry files were byte-identical to the pre-change binary at `8c1150ab3`, using one thread. This is regression evidence, not new contact qualification.
- External CA checks passed for unloaded natural thinning, a nongrowing bonded constituent, nonuniform compatible natural deformation with mixed body selection, resumable histories, mass balance, rigid rotation/fold attachment, and damage independent of turnover. Homogeneous time refinement decreased error; a finer grid changed normalized mass by `2.39e-7` in the tested case.

Focused evidence is retained in the local CA project's `ca3d/gr_damage_20261024/evidence/`. This work does not claim a full-suite or biological validation.
