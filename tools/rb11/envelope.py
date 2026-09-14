#!/usr/bin/env python3
"""RB-11 physical-envelope characterisation: locking, thin and distorted
elements, near-incompressibility and anisotropy against analytical or
mesh-converged references.

Everything is synthetic and public: Kuhn-tetrahedralised beams and cubes
written by this module, the same NeoHookean / MaterialSum(NeoHookean,
HGODispersion) laws the Houdini node exports, single-threaded quasistatic
one-step runs. No production setting is changed; the stage characterises.

    python3 tools/rb11/envelope.py --binary build/PolyFEM_bin --output /abs/fresh/dir \
        [--stage locking thin distorted homogeneous anisotropic] [--jobs 4] [--verify]
    python3 tools/rb11/envelope.py --list

Stages (the contract, with the thresholds declared before the matrix ran, is
``docs/rb-11-envelope-contract.md``):

``locking``     cantilever 4x1x1 under a uniform body load in the linear
                regime; nu in {.3, .45, .49, .499, .4999}; P1/P2; h = .5 / .25;
                reference: P2 on h = .125 at the same nu. Quantity: the tip
                centre deflection ratio r = u_y / u_y,ref (r < 1 = locking).
``thin``        the same beam with a square section H in {1, .25, .1} and two
                cells across the section (element aspect 1, 4, 10), nu = .3;
                reference: P2 with h_x halved twice and four cells across;
                analytical Euler-Bernoulli + Timoshenko control.
``distorted``   the h = .5 beam, nu = .3, interior vertices jittered by
                .15 h and .30 h, and cells stretched 2:1 / 4:1 along the beam;
                reference: the ``locking`` reference at nu = .3.
``homogeneous`` unit cube under uniaxial compression l_z = .9 with symmetry
                planes (the RB-09 block without the floor); nu up to .49999;
                P1 on 2x2x2 and 4x4x4; reference: the exact homogeneous
                Neo-Hookean state (``tools/rb09/reference.py``). Quantities:
                Cauchy stress, lateral stretch, Newton iterations.
``anisotropic`` MaterialSum(NeoHookean, HGODispersion) on the unit cube under
                a prescribed affine deformation (all faces Dirichlet):
                fibre along x / 45 deg in xz / z, kappa in {0, 1/6, 1/3};
                reference: the law's Cauchy stress from the energy by central
                differences. Plus the fibre-reinforced cantilever (fibres
                along the axis) P1/P2 against a converged P2 reference.

Every stage also runs a LinearElasticity twin of its coarse cases with the
reduced stiffness matrix exported (``output/data/stiffness_mat``) and reports
its condition number (dense symmetric eigenvalues after removing the Dirichlet
rows) - at F = I the Neo-Hookean tangent is the linear-elastic operator, so
this is the first Newton system's conditioning exactly.
"""
import argparse
import hashlib
import itertools
import json
import math
import os
import subprocess
import sys
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "rb09"))
import fixtures  # noqa: E402
import reference as rb09_reference  # noqa: E402

# the VTU reader lives in the Houdini tree beside the polyfem checkout; a
# worktree elsewhere finds it through the ancestors or POLYFEM_HDA_COMMON
_HDA_COMMON = [Path(os.environ["POLYFEM_HDA_COMMON"])] if os.environ.get("POLYFEM_HDA_COMMON") else []
_HDA_COMMON += [parent / "houdini_HDAs" / "src" / "common" for parent in HERE.parents]
for _candidate in _HDA_COMMON:
    if (_candidate / "vtu_parser.py").exists():
        sys.path.insert(0, str(_candidate))
        break
try:
    import vtu_parser  # noqa: E402
except Exception:  # pragma: no cover - the HDA tree is optional
    vtu_parser = None

E_REF = 1.0e6
RHO = 1000.0
L_BEAM = 4.0
# Body load that puts the thick beam's tip deflection at about 1e-3 L (Euler-
# Bernoulli: q L^4 / (8 E I) with q = f A, f = rho * rhs_y): linear regime.
RHS_Y_UNIT = 0.0104
NU_LOCKING = [0.3, 0.45, 0.49, 0.499, 0.4999]
NU_HOMOGENEOUS = [0.3, 0.45, 0.49, 0.499, 0.4999, 0.49999]
DENSE_EIG_LIMIT = 6000

# ---------------------------------------------------------------------------
# meshes


def beam_mesh(nx, ny, nz, L=L_BEAM, H=1.0, W=1.0, jitter=0.0, seed=0):
    """Kuhn tetrahedralisation (6 tets per cell) of [0,L]x[0,H]x[0,W].

    ``jitter`` moves every interior vertex by a uniform random offset of up to
    ``jitter * h_min`` per coordinate (fixed seed); the boundary stays planar.
    Returns (V, T, quality) with T positively oriented and quality the
    min/max tet volume ratio and the worst edge-ratio."""
    xs = np.linspace(0.0, L, nx + 1)
    ys = np.linspace(0.0, H, ny + 1)
    zs = np.linspace(0.0, W, nz + 1)
    V = np.array([(x, y, z) for x in xs for y in ys for z in zs])

    def vid(i, j, k):
        return (i * (ny + 1) + j) * (nz + 1) + k

    T = []
    for i, j, k in itertools.product(range(nx), range(ny), range(nz)):
        c = {(a, b, d): vid(i + a, j + b, k + d) for a in (0, 1) for b in (0, 1) for d in (0, 1)}
        for perm in itertools.permutations(range(3)):
            path = [(0, 0, 0)]
            cur = [0, 0, 0]
            for axis in perm:
                cur[axis] = 1
                path.append(tuple(cur))
            T.append([c[p] for p in path])
    T = np.array(T, dtype=int)
    T = fixtures.orient(V, T)

    if jitter > 0:
        rng = np.random.default_rng(seed)
        h_min = min(L / nx, H / ny, W / nz)
        interior = np.array([
            0 < i < nx and 0 < j < ny and 0 < k < nz
            for i in range(nx + 1) for j in range(ny + 1) for k in range(nz + 1)])
        V = V.copy()
        V[interior] += rng.uniform(-jitter * h_min, jitter * h_min, size=(int(interior.sum()), 3))
        vols = np.array([fixtures.tet_volume(V, t) for t in T])
        if vols.min() <= 0:
            raise RuntimeError("jitter %g inverted a tetrahedron (min volume %g)" % (jitter, vols.min()))

    vols = np.array([fixtures.tet_volume(V, t) for t in T])
    edge_ratio = 0.0
    for t in T:
        P = V[t]
        e = [np.linalg.norm(P[a] - P[b]) for a, b in itertools.combinations(range(4), 2)]
        edge_ratio = max(edge_ratio, max(e) / min(e))
    quality = {"n_vertices": int(len(V)), "n_tets": int(len(T)), "min_volume": float(vols.min()),
               "max_volume": float(vols.max()), "volume_ratio": float(vols.min() / vols.max()),
               "max_edge_ratio": float(edge_ratio)}
    return V, T, quality


def cube_mesh(n):
    V, T = fixtures.tet_cube(n=n)
    return V, T, {"n_vertices": int(len(V)), "n_tets": int(len(T))}


# ---------------------------------------------------------------------------
# scenes


def output_block(stiffness_path=None):
    out = {
        "log": {"level": "info"},
        "json": "out.json",
        "paraview": {"file_name": "out.pvd", "options": {"tensor_values": True, "material": False}},
    }
    if stiffness_path:
        out["data"] = {"stiffness_mat": stiffness_path}
    return out


def solver_block():
    return {
        "max_threads": 1,
        "linear": {"solver": "Eigen::SimplicialLDLT"},
        # 1e3 below PolyFEM's default 1e-5 (the stop is grad_norm_tol * F0 * L^1.5 in
        # the mass-weighted L2 norm, RB-09); the first protocol's 1e-10 sat below
        # the roundoff floor of the 56k-DOF reference at nu = .4999 (see the record)
        "nonlinear": {"grad_norm_tol": 1e-8, "rel_grad_norm_tol": 1e-12, "max_iterations": 200},
    }


def neo_hookean(nu, E=E_REF):
    return {"type": "NeoHookean", "E": E, "nu": nu, "rho": RHO}


def linear_twin(material):
    """The LinearElasticity law with the same Lame parameters."""
    if material["type"] == "NeoHookean":
        return {"type": "LinearElasticity", "E": material["E"], "nu": material["nu"], "rho": RHO}
    if material["type"] == "MaterialSum":
        nh = next(m for m in material["models"] if m["type"] == "NeoHookean")
        return {"type": "LinearElasticity", "E": nh["E"], "nu": nh["nu"], "rho": RHO}
    raise ValueError(material["type"])


def fibre_sum(nu, k1, k2, kappa, fibre, E=E_REF):
    return {
        "type": "MaterialSum",
        "rho": RHO,
        "models": [
            {"type": "NeoHookean", "E": E, "nu": nu},
            {"type": "HGODispersion", "k1": k1, "k2": k2, "kappa": kappa,
             "fiber_direction": [float(v) for v in fibre]},
        ],
    }


def cantilever_scene(mesh_file, material, order, rhs_y, stiffness_path=None):
    return {
        "geometry": [{"mesh": mesh_file,
                      "surface_selection": [{"id": 1, "axis": "-x", "position": 1e-6}]}],
        "space": {"discr_order": order},
        "materials": material,
        "time": {"dt": 1.0, "time_steps": 1, "quasistatic": True},
        "contact": {"enabled": False},
        "boundary_conditions": {
            "dirichlet_boundary": [{"id": 1, "value": [0, 0, 0]}],
            "rhs": [0.0, rhs_y, 0.0],
        },
        "solver": solver_block(),
        "output": output_block(stiffness_path),
    }


def homogeneous_scene(mesh_file, material, order, lz, stiffness_path=None):
    """Symmetry planes x = 0, y = 0, z = 0 and u_z = lz - 1 on the top face:
    the exact solution is the homogeneous uniaxial state (RB-09 block)."""
    return {
        "geometry": [{"mesh": mesh_file,
                      "surface_selection": [
                          {"id": 1, "axis": "-x", "position": 1e-6},
                          {"id": 2, "axis": "-y", "position": 1e-6},
                          {"id": 3, "axis": "-z", "position": 1e-6},
                          {"id": 4, "axis": "+z", "position": 1.0 - 1e-6},
                      ]}],
        "space": {"discr_order": order},
        "materials": material,
        "time": {"dt": 1.0, "time_steps": 1, "quasistatic": True},
        "contact": {"enabled": False},
        "boundary_conditions": {
            "dirichlet_boundary": [
                {"id": 1, "value": [0, 0, 0], "dimension": [True, False, False]},
                {"id": 2, "value": [0, 0, 0], "dimension": [False, True, False]},
                {"id": 3, "value": [0, 0, 0], "dimension": [False, False, True]},
                {"id": 4, "value": [0, 0, lz - 1.0], "dimension": [False, False, True]},
            ],
        },
        "solver": solver_block(),
        "output": output_block(stiffness_path),
    }


def affine_scene(mesh_file, material, order, F):
    """Every boundary face prescribed with u = (F - I) X: the exact solution
    is the homogeneous deformation F on any mesh."""
    G = np.asarray(F, dtype=float) - np.eye(3)
    exprs = ["%.17g*x + %.17g*y + %.17g*z" % tuple(G[i]) for i in range(3)]
    sel = [{"id": 1, "axis": "-x", "position": 1e-6}, {"id": 1, "axis": "+x", "position": 1.0 - 1e-6},
           {"id": 1, "axis": "-y", "position": 1e-6}, {"id": 1, "axis": "+y", "position": 1.0 - 1e-6},
           {"id": 1, "axis": "-z", "position": 1e-6}, {"id": 1, "axis": "+z", "position": 1.0 - 1e-6}]
    return {
        "geometry": [{"mesh": mesh_file, "surface_selection": sel}],
        "space": {"discr_order": order},
        "materials": material,
        "time": {"dt": 1.0, "time_steps": 1, "quasistatic": True},
        "contact": {"enabled": False},
        "boundary_conditions": {"dirichlet_boundary": [{"id": 1, "value": exprs}]},
        "solver": solver_block(),
        "output": output_block(),
    }


# ---------------------------------------------------------------------------
# analytical references


def timoshenko_tip(q, L, E, nu, H, W):
    """Uniform load q per unit length on a cantilever of rectangular section
    H (bending direction) x W: Euler-Bernoulli plus Timoshenko shear
    (Cowper's shear coefficient for a rectangle)."""
    I = W * H ** 3 / 12.0
    A = H * W
    G = E / (2.0 * (1.0 + nu))
    kappa = 10.0 * (1.0 + nu) / (12.0 + 11.0 * nu)
    eb = q * L ** 4 / (8.0 * E * I)
    shear = q * L ** 2 / (2.0 * kappa * G * A)
    return {"euler_bernoulli": eb, "timoshenko": eb + shear, "shear_part": shear}


def psi_neo_hookean(F, mu, lam):
    J = np.linalg.det(F)
    return 0.5 * mu * (np.sum(F * F) - 3.0 - 2.0 * math.log(J)) + 0.5 * lam * math.log(J) ** 2


def psi_hgo_dispersion(F, k1, k2, kappa, a, k_chi=100.0):
    a = np.asarray(a, dtype=float)
    a = a / np.linalg.norm(a)
    C = F.T @ F
    i1 = np.trace(C)
    i4 = a @ C @ a
    E4 = kappa * i1 + (1.0 - 3.0 * kappa) * i4 - 1.0
    chi = 1.0 / (1.0 + math.exp(-k_chi * E4))
    return (k1 / (2.0 * k2)) * chi * (math.exp(k2 * E4 * E4) - 1.0)


def material_psi(material):
    if material["type"] == "NeoHookean":
        mu, lam = rb09_reference.lame(material["E"], material["nu"])
        return lambda F: psi_neo_hookean(F, mu, lam)
    if material["type"] == "MaterialSum":
        parts = [material_psi(m) for m in material["models"]]
        return lambda F: sum(p(F) for p in parts)
    if material["type"] == "HGODispersion":
        return lambda F: psi_hgo_dispersion(F, material["k1"], material["k2"], material.get("kappa", 0.0),
                                            material["fiber_direction"])
    raise ValueError(material["type"])


def cauchy_stress(material, F, h=1e-6):
    """sigma = J^-1 P F^T with P = d psi / dF by central differences."""
    psi = material_psi(material)
    F = np.asarray(F, dtype=float)
    P = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            Fp = F.copy()
            Fm = F.copy()
            Fp[i, j] += h
            Fm[i, j] -= h
            P[i, j] = (psi(Fp) - psi(Fm)) / (2.0 * h)
    return P @ F.T / np.linalg.det(F)


# ---------------------------------------------------------------------------
# case matrix


class Case:
    def __init__(self, name, stage, mesh, scene, expect, reference=None, note=""):
        self.name = name
        self.stage = stage
        self.mesh = mesh              # (V, T, quality)
        self.scene = scene
        self.expect = expect          # dict consumed by the analysis
        self.reference = reference    # name of the reference case, if any
        self.note = note


def build_cases(stages):
    cases = []
    meshes = {}

    def mesh(key, builder):
        if key not in meshes:
            meshes[key] = builder()
        return meshes[key]

    def add(case):
        cases.append(case)

    # -- locking ------------------------------------------------------------
    if "locking" in stages or "distorted" in stages or "anisotropic" in stages:
        for nu in NU_LOCKING:
            if "locking" not in stages and nu != 0.3:
                continue
            ref_name = "locking-ref-nu%g" % nu
            m = mesh("beam-32x8x8", lambda: beam_mesh(32, 8, 8))
            add(Case(ref_name, "locking", m, cantilever_scene("mesh.msh", neo_hookean(nu), 2, RHS_Y_UNIT),
                     {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "reference_role": True, "nu": nu, "order": 2, "h": 0.125},
                     note="converged reference (P2, h = .125)"))
            if "locking" not in stages:
                continue
            for order, (nx, ny, nz, h) in itertools.product((1, 2), ((8, 2, 2, 0.5), (16, 4, 4, 0.25))):
                m = mesh("beam-%dx%dx%d" % (nx, ny, nz), lambda nx=nx, ny=ny, nz=nz: beam_mesh(nx, ny, nz))
                add(Case("locking-nu%g-P%d-h%g" % (nu, order, h), "locking", m,
                         cantilever_scene("mesh.msh", neo_hookean(nu), order, RHS_Y_UNIT),
                         {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "nu": nu, "order": order, "h": h},
                         reference=ref_name))
                if h == 0.5:
                    add(Case("locking-nu%g-P%d-h%g-linear" % (nu, order, h), "locking", m,
                             cantilever_scene("mesh.msh", linear_twin(neo_hookean(nu)), order, RHS_Y_UNIT, "K.mtx"),
                             {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "nu": nu, "order": order, "h": h,
                              "conditioning": True, "linear_twin": True},
                             reference=ref_name))

    # -- thin -----------------------------------------------------------------
    if "thin" in stages:
        for H in (1.0, 0.25, 0.1):
            rhs = RHS_Y_UNIT * H * H  # keeps the tip deflection near 1e-3 L
            ref_name = "thin-ref-H%g" % H
            if H == 1.0:
                m = mesh("beam-32x8x8", lambda: beam_mesh(32, 8, 8))
                nx_ref = 32
            else:
                m = mesh("beam-64x4x4-H%g" % H, lambda H=H: beam_mesh(64, 4, 4, H=H, W=H))
                nx_ref = 64
            add(Case(ref_name, "thin", m, cantilever_scene("mesh.msh", neo_hookean(0.3), 2, rhs),
                     {"kind": "tip", "tip": (L_BEAM, H / 2, H / 2), "reference_role": True, "H": H, "order": 2,
                      "h": L_BEAM / nx_ref, "rhs_y": rhs, "analytical": timoshenko_tip(rhs * RHO * H * H, L_BEAM, E_REF, 0.3, H, H)},
                     note="converged reference (P2, %d cells along, four across)" % nx_ref))
            for order in (1, 2):
                for nx in (8, 16):
                    m = mesh("beam-%dx2x2-H%g" % (nx, H), lambda nx=nx, H=H: beam_mesh(nx, 2, 2, H=H, W=H))
                    aspect = (L_BEAM / nx) / (H / 2)
                    add(Case("thin-H%g-P%d-nx%d" % (H, order, nx), "thin", m,
                             cantilever_scene("mesh.msh", neo_hookean(0.3), order, rhs),
                             {"kind": "tip", "tip": (L_BEAM, H / 2, H / 2), "H": H, "order": order,
                              "h": L_BEAM / nx, "aspect": aspect, "rhs_y": rhs},
                             reference=ref_name))
                    if nx == 8:
                        add(Case("thin-H%g-P%d-nx%d-linear" % (H, order, nx), "thin", m,
                                 cantilever_scene("mesh.msh", linear_twin(neo_hookean(0.3)), order, rhs, "K.mtx"),
                                 {"kind": "tip", "tip": (L_BEAM, H / 2, H / 2), "H": H, "order": order,
                                  "h": L_BEAM / nx, "aspect": aspect, "conditioning": True, "linear_twin": True},
                                 reference=ref_name))

    # -- distorted --------------------------------------------------------------
    if "distorted" in stages:
        ref_name = "locking-ref-nu0.3"
        variants = [
            ("regular", lambda: beam_mesh(8, 2, 2)),
            ("jitter0.15", lambda: beam_mesh(8, 2, 2, jitter=0.15, seed=11)),
            ("jitter0.30", lambda: beam_mesh(8, 2, 2, jitter=0.30, seed=11)),
            ("stretch2", lambda: beam_mesh(4, 2, 2)),
            ("stretch4", lambda: beam_mesh(2, 2, 2)),
        ]
        for label, builder in variants:
            m = mesh("beam-distorted-" + label, builder)
            for order in (1, 2):
                add(Case("distorted-%s-P%d" % (label, order), "distorted", m,
                         cantilever_scene("mesh.msh", neo_hookean(0.3), order, RHS_Y_UNIT),
                         {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "order": order, "variant": label},
                         reference=ref_name))
                add(Case("distorted-%s-P%d-linear" % (label, order), "distorted", m,
                         cantilever_scene("mesh.msh", linear_twin(neo_hookean(0.3)), order, RHS_Y_UNIT, "K.mtx"),
                         {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "order": order, "variant": label,
                          "conditioning": True, "linear_twin": True},
                         reference=ref_name))

    # -- homogeneous --------------------------------------------------------------
    if "homogeneous" in stages:
        lz = 0.9
        for nu in NU_HOMOGENEOUS:
            exact = rb09_reference.uniaxial(lz, E_REF, nu)
            for n in (2, 4):
                m = mesh("cube-%d" % n, lambda n=n: cube_mesh(n))
                add(Case("homogeneous-nu%g-n%d" % (nu, n), "homogeneous", m,
                         homogeneous_scene("mesh.msh", neo_hookean(nu), 1, lz),
                         {"kind": "homogeneous", "nu": nu, "n": n, "lz": lz,
                          "exact": {"l": exact["l"], "sigma_zz": exact["sigma_zz"], "J": exact["J"]},
                          "sigma_ref": cauchy_stress(neo_hookean(nu), np.diag([exact["l"], exact["l"], lz])).tolist()}))
                if n == 2:
                    add(Case("homogeneous-nu%g-n%d-linear" % (nu, n), "homogeneous", m,
                             homogeneous_scene("mesh.msh", linear_twin(neo_hookean(nu)), 1, lz, "K.mtx"),
                             {"kind": "homogeneous", "nu": nu, "n": n, "lz": lz, "conditioning": True,
                              "linear_twin": True}))

    # -- anisotropic ----------------------------------------------------------------
    if "anisotropic" in stages:
        F = np.array([[1.10, 0.0, 0.03], [0.0, 0.96, 0.0], [0.0, 0.0, 0.97]])
        fibres = {"x": (1.0, 0.0, 0.0), "xz45": (1.0, 0.0, 1.0), "z": (0.0, 0.0, 1.0)}
        m = mesh("cube-2", lambda: cube_mesh(2))
        for (flabel, fibre), kappa in itertools.product(fibres.items(), (0.0, 1.0 / 6.0, 1.0 / 3.0)):
            mat = fibre_sum(0.3, E_REF, 1.0, kappa, fibre)
            add(Case("anisotropic-affine-%s-kappa%.3f" % (flabel, kappa), "anisotropic", m,
                     affine_scene("mesh.msh", mat, 1, F),
                     {"kind": "affine", "F": F.tolist(), "sigma_ref": cauchy_stress(mat, F).tolist(),
                      "fibre": flabel, "kappa": kappa}))
        # a scaled fibre vector must give the same stress (HGODispersion normalises)
        mat = fibre_sum(0.3, E_REF, 1.0, 0.0, (3.0, 0.0, 0.0))
        add(Case("anisotropic-affine-x-scaled-fibre", "anisotropic", m, affine_scene("mesh.msh", mat, 1, F),
                 {"kind": "affine", "F": F.tolist(), "sigma_ref": cauchy_stress(mat, F).tolist(),
                  "fibre": "x (length 3)", "kappa": 0.0}))
        # fibre-reinforced bending: fibres along the axis, k1 = E
        mat = fibre_sum(0.3, E_REF, 1.0, 0.0, (1.0, 0.0, 0.0))
        ref_name = "anisotropic-bending-ref"
        mref = mesh("beam-32x8x8", lambda: beam_mesh(32, 8, 8))
        add(Case(ref_name, "anisotropic", mref, cantilever_scene("mesh.msh", mat, 2, RHS_Y_UNIT),
                 {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "reference_role": True, "order": 2, "h": 0.125},
                 note="converged reference (P2, h = .125), fibres along x"))
        for order, (nx, ny, nz, h) in itertools.product((1, 2), ((8, 2, 2, 0.5), (16, 4, 4, 0.25))):
            mb = mesh("beam-%dx%dx%d" % (nx, ny, nz), lambda nx=nx, ny=ny, nz=nz: beam_mesh(nx, ny, nz))
            add(Case("anisotropic-bending-P%d-h%g" % (order, h), "anisotropic", mb,
                     cantilever_scene("mesh.msh", mat, order, RHS_Y_UNIT),
                     {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "order": order, "h": h},
                     reference=ref_name))
    # -- refine (contract amendment 1): one p-refinement of selected references ----
    if "refine" in stages:
        m = mesh("beam-32x8x8", lambda: beam_mesh(32, 8, 8))
        add(Case("refine-locking-nu0.3-P3", "refine", m, cantilever_scene("mesh.msh", neo_hookean(0.3), 3, RHS_Y_UNIT),
                 {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "reference_role": True, "refines": "locking-ref-nu0.3", "order": 3, "h": 0.125},
                 note="P3 on the reference mesh (control)"))
        add(Case("refine-locking-nu0.4999-P3", "refine", m, cantilever_scene("mesh.msh", neo_hookean(0.4999), 3, RHS_Y_UNIT),
                 {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "reference_role": True, "refines": "locking-ref-nu0.4999", "order": 3, "h": 0.125},
                 note="P3 on the reference mesh (worst near-incompressible)"))
        mt = mesh("beam-64x4x4-H0.1", lambda: beam_mesh(64, 4, 4, H=0.1, W=0.1))
        rhs = RHS_Y_UNIT * 0.1 * 0.1
        add(Case("refine-thin-H0.1-P3", "refine", mt, cantilever_scene("mesh.msh", neo_hookean(0.3), 3, rhs),
                 {"kind": "tip", "tip": (L_BEAM, 0.05, 0.05), "reference_role": True, "refines": "thin-ref-H0.1", "order": 3, "h": L_BEAM / 64,
                  "rhs_y": rhs, "analytical": timoshenko_tip(rhs * RHO * 0.1 * 0.1, L_BEAM, E_REF, 0.3, 0.1, 0.1)},
                 note="P3 on the thin reference mesh"))
        mat = fibre_sum(0.3, E_REF, 1.0, 0.0, (1.0, 0.0, 0.0))
        add(Case("refine-anisotropic-bending-P3", "refine", m, cantilever_scene("mesh.msh", mat, 3, RHS_Y_UNIT),
                 {"kind": "tip", "tip": (L_BEAM, 0.5, 0.5), "reference_role": True, "refines": "anisotropic-bending-ref", "order": 3, "h": 0.125},
                 note="P3 on the reference mesh, fibres along x"))
    return cases


# ---------------------------------------------------------------------------
# running



# ---------------------------------------------------------------------------
# inputs, execution records, parsing (kept separate: an execution record is
# written before any output is parsed, and a parser failure is recorded per
# case instead of cancelling the queue)

INPUT_FILES = ("scene.json", "mesh.msh")
T_END = 1.0


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def render_inputs(case):
    """The exact bytes the generator writes for a case (mesh + scene)."""
    V, T, quality = case.mesh
    tmp = Path(os.environ.get("TMPDIR", "/tmp")) / ("rb11-envelope-render-%d" % os.getpid())
    tmp.mkdir(parents=True, exist_ok=True)
    fixtures.write_msh41(str(tmp / "mesh.msh"), V, T)
    fixtures.dump_json(tmp / "scene.json", case.scene)
    data = {name: (tmp / name).read_bytes() for name in INPUT_FILES}
    for name in INPUT_FILES:
        (tmp / name).unlink()
    return data


def case_meta(case):
    return {"name": case.name, "stage": case.stage, "expect": case.expect, "reference": case.reference,
            "note": case.note, "mesh_quality": case.mesh[2]}


def write_inputs(case, case_dir, rendered):
    case_dir.mkdir(parents=True, exist_ok=True)
    for name, data in rendered.items():
        (case_dir / name).write_bytes(data)
    fixtures.dump_json(case_dir / "case.json", case_meta(case))


def input_hashes(case_dir):
    return {name: sha256_of(case_dir / name) for name in INPUT_FILES if (case_dir / name).exists()}


def inputs_match(case_dir, rendered):
    """True when the on-disk inputs are byte-identical to the generator's."""
    for name, data in rendered.items():
        f = case_dir / name
        if not f.exists() or f.read_bytes() != data:
            return False
    return True


def execute(binary, binary_sha, case_dir, timeout):
    """Run the solver and persist the execution record before anything is parsed."""
    cmd = [str(binary), "--json", "scene.json", "--output_dir", "out"]
    env = dict(os.environ, OMP_NUM_THREADS="1")
    started = time.time()
    timed_out = False
    signal = None
    rc = None
    with open(case_dir / "log.txt", "w") as log:
        try:
            proc = subprocess.run(cmd, cwd=str(case_dir), stdout=log, stderr=subprocess.STDOUT,
                                  timeout=timeout, env=env)
            rc = proc.returncode
            if rc is not None and rc < 0:
                signal = -rc
        except subprocess.TimeoutExpired:
            timed_out = True
    record = {
        "schema": "rb11-envelope-run", "version": 2,
        "command": cmd, "binary": str(binary), "binary_sha256": binary_sha,
        "input_sha256": input_hashes(case_dir),
        "exit": rc, "signal": signal, "timed_out": timed_out,
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(started)),
        "wall_seconds": time.time() - started,
    }
    (case_dir / "run.json").write_text(json.dumps(record, indent=2))
    return record


def parse_output(case_dir):
    """Read out.json / out.pvd into a parsed record; never raises."""
    parsed = {"status": "ok", "errors": []}
    out_json = case_dir / "out" / "out.json"
    try:
        if not out_json.exists():
            parsed["status"] = "no out.json"
            return parsed
        d = json.loads(out_json.read_text())
        parsed["num_dofs"] = d.get("num_dofs")
        parsed["num_elements"] = d.get("num_elements")
        parsed["time_solving"] = d.get("time_solving")
        info = d.get("solver_info")
        if isinstance(info, list):
            # every step record must carry an integer iteration count and an
            # outcome; a missing, null or invalid count leaves the Newton count
            # UNAVAILABLE (never zero: a recorded zero is a real measurement),
            # and malformed entries are counted, not dropped (follow-up review
            # 2026-09-14: `int(missing or 0)` had turned the real 24-iteration
            # miss into a pass)
            parsed["solver_kind"] = "nonlinear"
            counts, outcomes, reasons, grads = [], [], [], []
            malformed = []
            for k, step in enumerate(info):
                st = step.get("info") if isinstance(step, dict) else None
                if not isinstance(st, dict):
                    malformed.append("step %d: no info record" % k)
                    continue
                it = st.get("iterations")
                if "iterations" not in st:
                    malformed.append("step %d: iterations missing" % k)
                elif isinstance(it, bool) or not isinstance(it, int) or it < 0:
                    malformed.append("step %d: iterations %r is not a nonnegative integer" % (k, it))
                elif not isinstance(st.get("outcome"), str):
                    malformed.append("step %d: outcome %r is not a string" % (k, st.get("outcome")))
                else:
                    counts.append(it)
                outcomes.append(st.get("outcome"))
                reasons.append(st.get("termination_reason"))
                grads.append(st.get("gradNorm"))
            parsed["newton_steps"] = len(info)
            parsed["malformed_steps"] = malformed
            complete = bool(info) and not malformed
            parsed["newton_count_status"] = "recorded" if complete else "unavailable"
            parsed["newton_iterations"] = sum(counts) if complete else None
            parsed["newton_outcomes"] = outcomes
            parsed["termination_reasons"] = reasons
            parsed["grad_norm"] = grads
            if not info:
                parsed["errors"].append("solver_info list carries no step record")
            parsed["errors"].extend(malformed)
        elif isinstance(info, dict):
            parsed["solver_kind"] = "linear"
            parsed["linear_status"] = info.get("solver_info")
        elif info is None:
            parsed["solver_kind"] = "unknown"
            parsed["errors"].append("solver_info missing")
        else:
            parsed["solver_kind"] = "unknown"
            parsed["errors"].append("solver_info of unexpected type %s" % type(info).__name__)
        pvd = case_dir / "out" / "out.pvd"
        if pvd.exists():
            import re
            entries = re.findall(r'timestep="([^"]+)"[^>]*file="([^"]+)"', pvd.read_text())
            if entries:
                t_last, f_last = entries[-1]
                parsed["pvd_last_time"] = float(t_last)
                parsed["pvd_last_file"] = f_last
                parsed["pvd_steps"] = len(entries)
            else:
                parsed["errors"].append("out.pvd lists no timestep")
        else:
            parsed["errors"].append("out.pvd missing")
    except Exception as exc:  # noqa: BLE001 - recorded, never fatal
        parsed["status"] = "parse error"
        parsed["errors"].append("%s: %s" % (type(exc).__name__, exc))
    if parsed["errors"] and parsed["status"] == "ok":
        parsed["status"] = "incomplete"
    return parsed


# ---------------------------------------------------------------------------
# analysis (measurements only; verification is separate)


def read_endpoint_vtu(case_dir, parsed):
    """The VTU of the PVD's last timestep; None with a reason otherwise."""
    if vtu_parser is None:
        return None, "vtu_parser unavailable"
    f = parsed.get("pvd_last_file")
    if not f:
        return None, "no PVD endpoint"
    path = case_dir / "out" / Path(f).name
    if path.suffix == ".vtm":
        # PolyFEM writes step_N.vtm collections next to step_N.vtu
        path = path.with_suffix(".vtu")
    if not path.exists():
        return None, "endpoint file %s missing" % path.name
    try:
        return vtu_parser.read_vtu(str(path)), None
    except Exception as exc:  # noqa: BLE001
        return None, "vtu parse error: %s" % exc


def tip_deflection(mesh, tip):
    P = mesh["points"]
    d = np.linalg.norm(P - np.asarray(tip), axis=1)
    i = int(np.argmin(d))
    u = mesh["point_data"]["solution"]
    return float(u[i, 1]), float(d[i]), float(u[:, 1].min())


def stress_tensors(mesh):
    pd = mesh["point_data"]
    return np.stack([pd["cauchy_stess_1"], pd["cauchy_stess_2"], pd["cauchy_stess_3"]], axis=1)


def read_market_symmetric(path):
    """Eigen's saveMarket: '%%MatrixMarket matrix coordinate real general'."""
    rows, cols, vals = [], [], []
    n = None
    with open(path) as fh:
        for line in fh:
            if line.startswith("%"):
                continue
            parts = line.split()
            if n is None:
                n = int(parts[0])
                continue
            if len(parts) < 3:
                continue
            rows.append(int(parts[0]) - 1)
            cols.append(int(parts[1]) - 1)
            vals.append(float(parts[2]))
    K = np.zeros((n, n))
    K[rows, cols] = vals
    return K


def condition_number(case_dir):
    path = case_dir / "K.mtx"
    if not path.exists():
        return {"status": "missing"}
    K = read_market_symmetric(path)
    n = K.shape[0]
    # Dirichlet rows/cols were replaced by unit rows in dirichlet_solve
    unit = np.array([np.count_nonzero(K[i]) == 1 and K[i, i] == 1.0 for i in range(n)])
    free = ~unit
    Kf = K[np.ix_(free, free)]
    if Kf.shape[0] > DENSE_EIG_LIMIT:
        return {"status": "skipped", "reason": "%d free dofs > dense limit" % Kf.shape[0]}
    if Kf.shape[0] == 0:
        return {"status": "skipped", "reason": "no free dofs"}
    asym = float(np.abs(Kf - Kf.T).max())
    w = np.linalg.eigvalsh(0.5 * (Kf + Kf.T))
    return {"status": "ok", "n_free": int(Kf.shape[0]), "n_dirichlet": int(unit.sum()),
            "lambda_min": float(w[0]), "lambda_max": float(w[-1]),
            "condition_number": float(w[-1] / w[0]) if w[0] > 0 else float("inf"),
            "asymmetry": asym, "min_eigenvalue_positive": bool(w[0] > 0)}


def geometry_checks(mesh, expect):
    """Sample-array consistency, finiteness, det(F) and the tip sample."""
    g = {}
    P = mesh["points"]
    pd = mesh["point_data"]
    g["n_points"] = int(P.shape[0])
    sizes = {k: int(v.shape[0]) for k, v in pd.items()}
    g["arrays_match_points"] = all(n == P.shape[0] for n in sizes.values())
    g["all_finite"] = all(bool(np.isfinite(v).all()) for v in pd.values())
    g["detF_available"] = all(k in pd for k in ("F_1", "F_2", "F_3"))
    if g["detF_available"]:
        F = np.stack([pd["F_1"], pd["F_2"], pd["F_3"]], axis=1)
        detF = np.linalg.det(F)
        g["detF_min"] = float(detF.min())
        g["detF_max"] = float(detF.max())
        g["detF_positive"] = bool(np.isfinite(detF).all() and detF.min() > 0)
    if expect.get("kind") == "tip":
        _, dist, _ = tip_deflection(mesh, expect["tip"])
        g["tip_sample_distance"] = dist
        g["tip_sample_exact"] = bool(dist <= 1e-9)
    return g


def analyze(case_dir, meta, run, parsed, references):
    expect = meta["expect"]
    entry = {"name": meta["name"], "stage": meta["stage"], "reference": meta.get("reference"), "note": meta.get("note"),
             "expect": expect, "mesh_quality": meta.get("mesh_quality"),
             "lineage": run.get("lineage"), "binary_sha256": run.get("binary_sha256"),
             "exit": run.get("exit"), "signal": run.get("signal"), "timed_out": run.get("timed_out"),
             "wall_seconds": run.get("wall_seconds"), "exit_recorded": run.get("exit") is not None,
             "parse": parsed, "num_dofs": parsed.get("num_dofs"), "num_elements": parsed.get("num_elements"),
             "newton_iterations": parsed.get("newton_iterations"), "newton_outcomes": parsed.get("newton_outcomes"),
             "solver_kind": parsed.get("solver_kind")}
    if run.get("exit") != 0:
        entry["status"] = "run failed" if run.get("exit") is not None else "exit not recorded"
        if run.get("exit") is None and parsed.get("status") in ("ok", "incomplete"):
            entry["status"] = "exit not recorded"
        else:
            return entry
    mesh, why = read_endpoint_vtu(case_dir, parsed)
    if mesh is None:
        entry["status"] = "no endpoint output (%s)" % why
        return entry
    entry["endpoint_time"] = parsed.get("pvd_last_time")
    entry["geometry"] = geometry_checks(mesh, expect)
    kind = expect["kind"]
    if kind == "tip":
        uy, dist, uy_min = tip_deflection(mesh, expect["tip"])
        entry["tip_uy"] = uy
        entry["min_uy"] = uy_min
        if expect.get("analytical"):
            entry["analytical"] = expect["analytical"]
        ref = references.get(meta.get("reference"))
        if ref is not None and ref.get("tip_uy"):
            entry["ratio_to_reference"] = uy / ref["tip_uy"]
            entry["reference_lineage"] = ref.get("lineage")
    elif kind in ("homogeneous", "affine"):
        S = stress_tensors(mesh)
        S_ref = np.asarray(expect["sigma_ref"]) if "sigma_ref" in expect else None
        entry["sigma_mean"] = S.mean(axis=0).tolist()
        entry["sigma_spread"] = float(np.abs(S - S.mean(axis=0)).max())
        if S_ref is not None:
            scale = float(np.abs(S_ref).max())
            entry["sigma_ref"] = S_ref.tolist()
            entry["sigma_rel_error"] = float(np.abs(S - S_ref).max() / scale)
            entry["sigma_spread_rel"] = entry["sigma_spread"] / scale
        if kind == "homogeneous":
            P = mesh["points"]
            u = mesh["point_data"]["solution"]
            on_face = np.abs(P[:, 0] - 1.0) < 1e-9
            ux = u[on_face, 0]
            entry["lateral_stretch"] = float(1.0 + ux.mean())
            entry["lateral_stretch_spread"] = float(ux.max() - ux.min())
            if "exact" in expect:
                entry["lateral_stretch_rel_error"] = float(abs(entry["lateral_stretch"] - expect["exact"]["l"]) / abs(expect["exact"]["l"] - 1.0))
    if expect.get("conditioning"):
        entry["conditioning"] = condition_number(case_dir)
    entry["status"] = "ok" if entry.get("status") is None else entry["status"]
    return entry


# ---------------------------------------------------------------------------
# verification against docs/rb-11-envelope-contract.md: every check reports
# pass / fail / not evaluated with its value, threshold and cases


class Checks:
    def __init__(self):
        self.items = []

    def add(self, cid, cases, status, value=None, threshold=None, note=""):
        self.items.append({"id": cid, "cases": cases if isinstance(cases, list) else [cases],
                           "status": status, "value": value, "threshold": threshold, "note": note})

    def failures(self):
        return [c for c in self.items if c["status"] == "fail"]

    def not_evaluated(self):
        return [c for c in self.items if c["status"] == "not evaluated"]


def verify(entries, manifest):
    """`manifest` is the explicit list of case names the report claims to cover."""
    if not manifest:
        raise ValueError("verification needs a nonempty case manifest")
    by_name = {e["name"]: e for e in entries}
    unknown = sorted(set(by_name) - set(manifest))
    if unknown:
        raise ValueError("entries outside the manifest: %s" % unknown)
    checks = Checks()

    def get(name):
        e = by_name.get(name)
        if e is None or e.get("status") != "ok":
            return None
        return e

    # C-R: every manifest case ran, exited 0, reached t_end with finite, valid output
    for name in manifest:
        e = by_name.get(name)
        if e is None:
            checks.add("C-R", name, "not evaluated", note="not run")
            continue
        if not e.get("exit_recorded"):
            checks.add("C-R", name, "not evaluated", note="process exit not recorded (%s)" % e.get("status"))
            continue
        if e.get("status") != "ok":
            checks.add("C-R", name, "fail", value=e.get("status"), note="exit %s" % e.get("exit"))
            continue
        problems = []
        if e.get("endpoint_time") is None or abs(e["endpoint_time"] - T_END) > 1e-12:
            problems.append("endpoint time %s != %g" % (e.get("endpoint_time"), T_END))
        g = e.get("geometry", {})
        if not g.get("arrays_match_points"):
            problems.append("array sizes")
        if not g.get("all_finite"):
            problems.append("nonfinite field")
        if e.get("solver_kind") == "nonlinear":
            # applicable evidence is required, not merely checked when present
            if e.get("newton_iterations") is None:
                problems.append("Newton count unavailable (%s)" % "; ".join(e["parse"].get("malformed_steps") or ["no step record"]))
            if not e.get("newton_outcomes") or any(o != "converged" for o in e["newton_outcomes"]):
                problems.append("termination %s" % e.get("newton_outcomes"))
            if not g.get("detF_available"):
                problems.append("no det(F) evidence (F arrays absent)")
            elif not g.get("detF_positive"):
                problems.append("det(F) <= 0 or nonfinite")
        elif e.get("solver_kind") == "linear":
            if e["parse"].get("linear_status") != "Success":
                problems.append("linear solve %s" % e["parse"].get("linear_status"))
        else:
            problems.append("unknown solver record")
        if e["expect"]["kind"] == "tip" and not g.get("tip_sample_exact"):
            problems.append("tip sample %.3g away" % g.get("tip_sample_distance", float("nan")))
        if e["expect"].get("conditioning"):
            c = e.get("conditioning") or {}
            if c.get("status") != "ok" or not c.get("min_eigenvalue_positive"):
                problems.append("stiffness %s" % (c.get("status") or "missing"))
        checks.add("C-R", name, "fail" if problems else "pass", value="; ".join(problems) if problems else "ok")

    for name in manifest:
        # the expectation comes from any entry of the case (even a failed run);
        # measured values only from a usable one, so a missing run leaves its
        # comparisons "not evaluated" instead of silently absent
        e_any = by_name.get(name)
        if e_any is None:
            continue
        ex = e_any["expect"]
        e = get(name) or {}
        stage = e_any["stage"]
        # C-T: the linear twin's tip agrees with its nonlinear run
        if ex["kind"] == "tip" and ex.get("linear_twin"):
            partner = get(name.replace("-linear", "")) or {}
            if partner.get("tip_uy") is None or e.get("tip_uy") is None:
                checks.add("C-T", [name, name.replace("-linear", "")], "not evaluated", threshold=1e-2, note="a run or its tip is missing")
            else:
                rel = abs(e["tip_uy"] - partner["tip_uy"]) / abs(partner["tip_uy"])
                checks.add("C-T", [name, partner["name"]], "pass" if rel < 1e-2 else "fail", value=rel, threshold=1e-2)
        # C-A: converged references against Timoshenko
        if ex["kind"] == "tip" and ex.get("reference_role") and ex.get("analytical"):
            if e.get("tip_uy") is None:
                checks.add("C-A", name, "not evaluated", threshold=0.10)
            else:
                rel = abs(e["tip_uy"] - (-ex["analytical"]["timoshenko"])) / ex["analytical"]["timoshenko"]
                checks.add("C-A", name, "pass" if rel < 0.10 else "fail", value=rel, threshold=0.10)
        # C-REF (amendment 1): a P2 reference against its P3 refinement
        if ex["kind"] == "tip" and ex.get("refines"):
            ref = get(ex["refines"]) or {}
            if e.get("tip_uy") is None or ref.get("tip_uy") is None:
                checks.add("C-REF", [name, ex["refines"]], "not evaluated", threshold=1e-2, note="a run is missing")
            else:
                rel = abs(e["tip_uy"] - ref["tip_uy"]) / abs(e["tip_uy"])
                checks.add("C-REF", [name, ex["refines"]], "pass" if rel <= 1e-2 else "fail", value=rel, threshold=1e-2)
        # C-B / C-L1: tip ratios of the coarse nonlinear beams
        if ex["kind"] == "tip" and not ex.get("reference_role") and not ex.get("linear_twin"):
            r = e.get("ratio_to_reference")
            if r is None:
                checks.add("C-B", name, "not evaluated", note="run or reference %s missing" % e_any.get("reference"))
                if stage == "locking" and ex.get("order") == 2 and ex.get("h") == 0.25:
                    checks.add("C-L1", name, "not evaluated", threshold=0.95)
            else:
                checks.add("C-B", name, "pass" if 0 < r < 1.05 else "fail", value=r, threshold="(0, 1.05)")
                if stage == "locking" and ex.get("order") == 2 and ex.get("h") == 0.25:
                    checks.add("C-L1", name, "pass" if r >= 0.95 else "fail", value=r, threshold=0.95)
        # C-H: homogeneous state
        if ex["kind"] == "homogeneous" and not ex.get("linear_twin"):
            for key, cid in (("sigma_rel_error", "C-H stress"), ("lateral_stretch_rel_error", "C-H stretch"), ("sigma_spread_rel", "C-H uniform")):
                v = e.get(key)
                if v is None:
                    checks.add(cid, name, "not evaluated", threshold=1e-6)
                else:
                    checks.add(cid, name, "pass" if v < 1e-6 else "fail", value=v, threshold=1e-6)
            # C-N: cost target
            it = e.get("newton_iterations")
            if it is None:
                checks.add("C-N", name, "not evaluated", threshold=20)
            else:
                checks.add("C-N", name, "pass" if it <= 20 else "fail", value=it, threshold=20)
        # C-F: affine state
        if ex["kind"] == "affine":
            for key, cid in (("sigma_rel_error", "C-F stress"), ("sigma_spread_rel", "C-F uniform")):
                v = e.get(key)
                if v is None:
                    checks.add(cid, name, "not evaluated", threshold=1e-6)
                else:
                    checks.add(cid, name, "pass" if v < 1e-6 else "fail", value=v, threshold=1e-6)
    # C-F scaled fibre: same stress as the unit fibre
    a, b = get("anisotropic-affine-x-kappa0.000"), get("anisotropic-affine-x-scaled-fibre")
    if "anisotropic-affine-x-scaled-fibre" in manifest:
        if a is None or b is None or a.get("sigma_mean") is None or b.get("sigma_mean") is None:
            checks.add("C-F scaled fibre", ["anisotropic-affine-x-kappa0.000", "anisotropic-affine-x-scaled-fibre"], "not evaluated")
        else:
            d = float(np.abs(np.asarray(a["sigma_mean"]) - np.asarray(b["sigma_mean"])).max() / np.abs(np.asarray(a["sigma_mean"])).max())
            checks.add("C-F scaled fibre", ["anisotropic-affine-x-kappa0.000", "anisotropic-affine-x-scaled-fibre"], "pass" if d < 1e-6 else "fail", value=d, threshold=1e-6)
    # C-L2: monotone locking with nu for P1 (both h) and P2 h = .5
    for order, h in ((1, 0.5), (1, 0.25), (2, 0.5)):
        names = ["locking-nu%g-P%d-h%g" % (nu, order, h) for nu in NU_LOCKING]
        if not all(n in manifest for n in names):
            continue
        ratios = [(get(n) or {}).get("ratio_to_reference") for n in names]
        if any(r is None for r in ratios):
            checks.add("C-L2", names, "not evaluated", note="a ratio is missing")
        else:
            ok = all(x >= y - 1e-3 for x, y in zip(ratios, ratios[1:]))
            checks.add("C-L2", names, "pass" if ok else "fail", value=ratios, threshold="non-increasing (tol 1e-3)")
    return checks


# ---------------------------------------------------------------------------
# reporting


def fmt(v, spec="%.4g"):
    if v is None:
        return "-"
    if isinstance(v, float):
        return spec % v
    return str(v)


def summarize(entries, checks, out_dir, lineage_note=""):
    lines = ["# RB-11 envelope matrix", ""]
    if lineage_note:
        lines += [lineage_note, ""]
    stages = []
    for e in entries:
        if e["stage"] not in stages:
            stages.append(e["stage"])
    for stage in stages:
        rows = [e for e in entries if e["stage"] == stage]
        lines.append("## %s" % stage)
        lines.append("")
        if stage == "homogeneous":
            lines.append("| case | candidate | dofs | Newton | sigma_zz | rel. stress err | lateral stretch | rel. err | cond(K) | status |")
            lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
            for e in rows:
                c = e.get("conditioning") or {}
                sm = e.get("sigma_mean")
                lines.append("| `%s` | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    e["name"], (e.get("lineage") or {}).get("candidate", "-"), e.get("num_dofs"), fmt(e.get("newton_iterations")),
                    fmt(sm[2][2] if sm else None, "%.6g"), fmt(e.get("sigma_rel_error"), "%.2e"), fmt(e.get("lateral_stretch"), "%.8g"),
                    fmt(e.get("lateral_stretch_rel_error"), "%.2e"), fmt(c.get("condition_number"), "%.3e"), e["status"]))
        else:
            lines.append("| case | candidate | dofs | Newton | tip u_y / stress err | ratio to ref / spread | cond(K) | lambda_min | status |")
            lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
            for e in rows:
                c = e.get("conditioning") or {}
                cand = (e.get("lineage") or {}).get("candidate", "-")
                if e["expect"]["kind"] == "affine":
                    lines.append("| `%s` | %s | %s | %s | %s | %s | - | - | %s |" % (
                        e["name"], cand, e.get("num_dofs"), fmt(e.get("newton_iterations")), fmt(e.get("sigma_rel_error"), "%.2e"),
                        fmt(e.get("sigma_spread_rel"), "%.2e"), e["status"]))
                    continue
                lines.append("| `%s` | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    e["name"], cand, e.get("num_dofs"), fmt(e.get("newton_iterations")), fmt(e.get("tip_uy"), "%.6g"),
                    fmt(e.get("ratio_to_reference"), "%.5g"), fmt(c.get("condition_number"), "%.3e"),
                    fmt(c.get("lambda_min"), "%.3e"), e["status"]))
        lines.append("")
    lines.append("## Contract checks")
    lines.append("")
    lines.append("| check | status | value | threshold | cases | note |")
    lines.append("| --- | --- | --- | --- | --- | --- |")
    for c in checks.items:
        if c["status"] == "pass" and c["id"] == "C-R":
            continue  # one row per case would swamp the table; failures and non-evaluations are listed
        val = c["value"]
        if isinstance(val, list):
            val = ", ".join(fmt(v, "%.4g") for v in val)
        lines.append("| %s | %s | %s | %s | %s | %s |" % (c["id"], c["status"], fmt(val), fmt(c["threshold"]),
                                                       ", ".join("`%s`" % n for n in c["cases"]), c["note"]))
    n_pass = sum(1 for c in checks.items if c["status"] == "pass")
    lines.append("")
    lines.append("%d checks: %d pass, %d fail, %d not evaluated (C-R passes omitted from the table)." % (
        len(checks.items), n_pass, len(checks.failures()), len(checks.not_evaluated())))
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")
    (out_dir / "summary.json").write_text(json.dumps(entries, indent=2, default=float))
    (out_dir / "verify.json").write_text(json.dumps({"checks": checks.items, "failures": checks.failures(),
                                                     "not_evaluated": checks.not_evaluated()}, indent=2, default=float))


# ---------------------------------------------------------------------------
# self-test of the parser and the oracle on hollow records


def self_test():
    import tempfile
    failures = []
    tmp = Path(tempfile.mkdtemp(prefix="rb11-envelope-selftest-"))

    def expect(cond, msg):
        if not cond:
            failures.append(msg)

    # 1. linear solver_info is a dict, not a list of step records
    d = tmp / "linear"
    (d / "out").mkdir(parents=True)
    (d / "out" / "out.json").write_text(json.dumps({"solver_info": {"solver_info": "Success"}, "num_dofs": 3}))
    (d / "out" / "out.pvd").write_text('<VTKFile><Collection><DataSet timestep="0" file="step_0.vtm"/><DataSet timestep="1" file="step_1.vtm"/></Collection></VTKFile>')
    p = parse_output(d)
    expect(p["status"] == "ok" and p["solver_kind"] == "linear" and p["linear_status"] == "Success" and p["pvd_last_time"] == 1.0,
           "linear solver_info dict not parsed: %s" % p)
    # 2. nonlinear list
    d = tmp / "nonlinear"
    (d / "out").mkdir(parents=True)
    (d / "out" / "out.json").write_text(json.dumps({"solver_info": [{"info": {"iterations": 3, "outcome": "converged"}, "t": 1}]}))
    (d / "out" / "out.pvd").write_text('<VTKFile><Collection><DataSet timestep="1" file="step_1.vtm"/></Collection></VTKFile>')
    p = parse_output(d)
    expect(p["solver_kind"] == "nonlinear" and p["newton_iterations"] == 3 and p["newton_outcomes"] == ["converged"], "nonlinear list not parsed: %s" % p)
    # 3. missing and malformed output never raise
    p = parse_output(tmp / "absent")
    expect(p["status"] == "no out.json", "missing output: %s" % p)
    d = tmp / "malformed"
    (d / "out").mkdir(parents=True)
    (d / "out" / "out.json").write_text("{not json")
    p = parse_output(d)
    expect(p["status"] == "parse error", "malformed output: %s" % p)
    # 4. a string solver_info (the crash of 2026-09-13) is reported, not raised
    d = tmp / "string"
    (d / "out").mkdir(parents=True)
    (d / "out" / "out.json").write_text(json.dumps({"solver_info": "Success"}))
    p = parse_output(d)
    expect(p["status"] == "incomplete" and p["solver_kind"] == "unknown", "string solver_info: %s" % p)
    # 5. verification refuses an empty manifest and names outside it
    try:
        verify([], [])
        expect(False, "empty manifest accepted")
    except ValueError:
        pass
    try:
        verify([{"name": "x", "status": "ok", "stage": "locking", "expect": {"kind": "tip"}}], ["y"])
        expect(False, "entry outside the manifest accepted")
    except ValueError:
        pass
    # 6. a hollow run (exit 0, no endpoint) fails C-R; an unrecorded exit is not evaluated
    hollow = {"name": "locking-nu0.3-P1-h0.5", "stage": "locking", "status": "no endpoint output (no PVD endpoint)", "exit": 0,
              "exit_recorded": True, "expect": {"kind": "tip"}, "parse": {}}
    unrecorded = {"name": "locking-nu0.3-P1-h0.5-linear", "stage": "locking", "status": "exit not recorded", "exit": None,
                  "exit_recorded": False, "expect": {"kind": "tip", "linear_twin": True}, "parse": {}}
    checks = verify([hollow, unrecorded], [hollow["name"], unrecorded["name"]])
    st = {tuple(c["cases"]) + (c["id"],): c["status"] for c in checks.items}
    expect(st.get((hollow["name"], "C-R")) == "fail", "hollow run not failed: %s" % st)
    expect(st.get((unrecorded["name"], "C-R")) == "not evaluated", "unrecorded exit evaluated: %s" % st)
    expect(any(c["id"] == "C-T" and c["status"] == "not evaluated" for c in checks.items), "missing C-T partner not reported")
    # 7. a wrong endpoint time or a non-converged Newton run fails C-R
    bad = {"name": "homogeneous-nu0.3-n2", "stage": "homogeneous", "status": "ok", "exit": 0, "exit_recorded": True,
           "expect": {"kind": "homogeneous"}, "parse": {}, "endpoint_time": 0.5, "solver_kind": "nonlinear",
           "newton_iterations": 4, "newton_outcomes": ["converged"], "geometry": {"arrays_match_points": True, "all_finite": True, "detF_positive": True}}
    checks = verify([bad], [bad["name"]])
    expect(any(c["id"] == "C-R" and c["status"] == "fail" and "endpoint" in str(c["value"]) for c in checks.items), "wrong endpoint accepted")
    bad["endpoint_time"] = 1.0
    bad["newton_outcomes"] = ["max_iterations"]
    checks = verify([bad], [bad["name"]])
    expect(any(c["id"] == "C-R" and c["status"] == "fail" and "termination" in str(c["value"]) for c in checks.items), "unconverged run accepted")
    expect(any(c["id"] == "C-N" and c["status"] == "pass" for c in checks.items), "C-N not evaluated on a recorded count")
    bad["newton_outcomes"] = ["converged"]
    bad["newton_iterations"] = None
    checks = verify([bad], [bad["name"]])
    expect(any(c["id"] == "C-N" and c["status"] == "not evaluated" for c in checks.items), "missing iteration count treated as a pass")
    # 8. parser -> analysis -> verifier on a synthetic case directory with a real
    #    (tiny, ASCII) endpoint VTU: missing / null / invalid / zero Newton counts,
    #    malformed step records and absent F arrays (follow-up review 2026-09-14)
    def synthetic_case(name, iterations, with_F=True, extra_step=None, outcome="converged"):
        cd = tmp / name
        (cd / "out").mkdir(parents=True)
        info = {"outcome": outcome, "termination_reason": "Gradient vector norm too small", "gradNorm": 1e-9}
        if iterations != "absent":
            info["iterations"] = iterations
        steps = [{"info": info, "t": 1, "type": "rc"}]
        if extra_step is not None:
            steps.append(extra_step)
        (cd / "out" / "out.json").write_text(json.dumps({"solver_info": steps, "num_dofs": 12, "num_elements": 1}))
        (cd / "out" / "out.pvd").write_text('<VTKFile><Collection><DataSet timestep="0" file="step_0.vtm"/><DataSet timestep="1" file="step_1.vtm"/></Collection></VTKFile>')
        pts = "0 0 0  1 0 0  0 1 0  0 0 1"
        sol = " ".join("%g %g %g" % (0.03 * x, 0.03 * y, -0.1 * z) for x, y, z in ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)))
        F_rows = {"F_1": "1.03 0 0 " * 4, "F_2": "0 1.03 0 " * 4, "F_3": "0 0 0.9 " * 4}
        S_rows = {"cauchy_stess_1": "0 0 0 " * 4, "cauchy_stess_2": "0 0 0 " * 4, "cauchy_stess_3": "0 0 -100000 " * 4}
        arrays = {"solution": sol}
        arrays.update(S_rows)
        if with_F:
            arrays.update(F_rows)
        da = "".join('<DataArray type="Float64" Name="%s" NumberOfComponents="3" format="ascii">%s</DataArray>' % (k, v) for k, v in arrays.items())
        (cd / "out" / "step_1.vtu").write_text(
            '<?xml version="1.0"?><VTKFile type="UnstructuredGrid" version="0.1" byte_order="LittleEndian"><UnstructuredGrid>'
            '<Piece NumberOfPoints="4" NumberOfCells="1"><Points><DataArray type="Float64" NumberOfComponents="3" format="ascii">%s</DataArray></Points>'
            '<Cells><DataArray type="Int64" Name="connectivity" format="ascii">0 1 2 3</DataArray><DataArray type="Int64" Name="offsets" format="ascii">4</DataArray>'
            '<DataArray type="UInt8" Name="types" format="ascii">10</DataArray></Cells><PointData>%s</PointData></Piece></UnstructuredGrid></VTKFile>' % (pts, da))
        meta = {"name": name, "stage": "homogeneous", "reference": None, "note": "", "mesh_quality": {},
                "expect": {"kind": "homogeneous", "nu": 0.3, "n": 2, "lz": 0.9, "exact": {"l": 1.03, "sigma_zz": -1e5, "J": 0.95},
                           "sigma_ref": [[0, 0, 0], [0, 0, 0], [0, 0, -1e5]]}}
        run = {"exit": 0, "binary_sha256": "test", "lineage": {"candidate": "selftest"}}
        entry = analyze(cd, meta, run, parse_output(cd), {})
        checks = verify([entry], [name])
        return entry, {(c["id"], c["status"]): c for c in checks.items}

    if vtu_parser is None:
        failures.append("vtu_parser unavailable: the parser-to-verifier probes did not run")
    else:
        e, st = synthetic_case("full-evidence", 3)
        expect(("C-R", "pass") in st and ("C-N", "pass") in st and st[("C-N", "pass")]["value"] == 3, "full evidence: %s %s" % (e.get("status"), sorted(st)))
        e, st = synthetic_case("real-miss", 24)
        expect(("C-R", "pass") in st and ("C-N", "fail") in st and st[("C-N", "fail")]["value"] == 24, "24-iteration miss not reported: %s" % sorted(st))
        for label, value in (("absent", "absent"), ("null", None), ("string", "3"), ("negative", -1), ("bool", True)):
            e, st = synthetic_case("count-" + label, value)
            expect(e.get("newton_iterations") is None and e["parse"].get("newton_count_status") == "unavailable",
                   "%s iteration count parsed as %r" % (label, e.get("newton_iterations")))
            expect(("C-R", "fail") in st and "Newton count unavailable" in str(st[("C-R", "fail")]["value"]), "%s count: C-R %s" % (label, sorted(st)))
            expect(("C-N", "not evaluated") in st, "%s count: C-N %s" % (label, sorted(st)))
        e, st = synthetic_case("count-zero", 0)
        expect(e.get("newton_iterations") == 0 and e["parse"].get("newton_count_status") == "recorded" and ("C-N", "pass") in st and st[("C-N", "pass")]["value"] == 0,
               "a recorded zero count is not kept distinct from a missing one: %s" % sorted(st))
        e, st = synthetic_case("malformed-step", 3, extra_step={"t": 2, "type": "rc"})
        expect(e.get("newton_iterations") is None and ("C-R", "fail") in st and ("C-N", "not evaluated") in st, "malformed step record accepted: %s" % sorted(st))
        e, st = synthetic_case("no-F", 3, with_F=False)
        expect(("C-R", "fail") in st and "det(F) evidence" in str(st[("C-R", "fail")]["value"]), "absent F arrays accepted: %s" % sorted(st))
        e, st = synthetic_case("unconverged", 3, outcome="max_iterations")
        expect(("C-R", "fail") in st and "termination" in str(st[("C-R", "fail")]["value"]), "unconverged synthetic run accepted: %s" % sorted(st))

    import shutil
    shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print("self-test FAILED:")
        for f in failures:
            print("  -", f)
        return 1
    print("self-test passed (parser, manifest, hollow records, endpoint/termination gates, parser-to-verifier evidence probes)")
    return 0


# ---------------------------------------------------------------------------
# driver


def load_record(case_dir):
    run_json = case_dir / "run.json"
    if not run_json.exists():
        return None
    try:
        return json.loads(run_json.read_text())
    except Exception:  # noqa: BLE001
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--binary")
    ap.add_argument("--output", help="candidate directory: cases executed by this invocation are written here")
    ap.add_argument("--stage", nargs="*", default=["locking", "thin", "distorted", "homogeneous", "anisotropic"])
    ap.add_argument("--only", nargs="*", default=None, help="case names to run (must be in the manifest)")
    ap.add_argument("--reuse", nargs="*", default=[], metavar="DIR",
                    help="earlier candidate directories whose recorded exit-0 runs with byte-identical inputs are reused")
    ap.add_argument("--reuse-binary-sha", default=None,
                    help="binary identity to attribute to legacy reused records that carry none (from that candidate's identity file)")
    ap.add_argument("--analyze-only", action="store_true", help="run no solver; parse and verify what exists")
    ap.add_argument("--report-dir", default=None, help="write summary/verify files here instead of into --output (keeps an earlier report intact)")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--timeout", type=float, default=1800.0)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--verify", action="store_true", help="exit 1 when a contract check fails or a manifest case is not evaluated")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    cases = build_cases(set(args.stage))
    manifest = [c.name for c in cases]
    if args.only:
        unknown = sorted(set(args.only) - set(manifest))
        if unknown:
            ap.error("unknown case names: %s" % unknown)
        cases = [c for c in cases if c.name in set(args.only)]
        manifest = [c.name for c in cases]
    if args.list:
        for c in cases:
            print("%-44s %-12s ref=%s  %s" % (c.name, c.stage, c.reference, c.note))
        print(len(cases), "cases")
        return 0
    if not args.output:
        ap.error("--output is required")
    out_dir = Path(args.output).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    reuse_dirs = [Path(d).resolve() for d in args.reuse]
    binary = Path(args.binary).resolve() if args.binary else None
    binary_sha = sha256_of(binary) if binary else None
    if binary_sha:
        (out_dir / "candidate.json").write_text(json.dumps({"binary": str(binary), "binary_sha256": binary_sha,
                                                            "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}, indent=2))

    # 1. locate or create every case's inputs; decide reuse per case
    plan = {}  # name -> (case_dir, record or None, lineage)
    to_run = []
    for c in cases:
        rendered = render_inputs(c)
        chosen = None
        for rd in reuse_dirs:
            cd = rd / c.name
            rec = load_record(cd)
            if rec is None or rec.get("exit") != 0 or not inputs_match(cd, rendered):
                continue
            sha = rec.get("binary_sha256") or args.reuse_binary_sha or "unrecorded"
            rec = dict(rec, lineage={"candidate": rd.name, "dir": str(cd), "binary_sha256": sha,
                                     "record_version": rec.get("version", 1), "reused": True})
            chosen = (cd, rec)
            break
        if chosen is None:
            cd = out_dir / c.name
            rec = load_record(cd)
            # the output directory's own record is reused when it belongs to this
            # binary (or when no solver runs at all: analysis of what exists)
            same_binary = binary_sha is None or rec is not None and rec.get("binary_sha256") == binary_sha
            if rec is not None and rec.get("exit") == 0 and inputs_match(cd, rendered) and same_binary:
                rec = dict(rec, lineage={"candidate": out_dir.name, "dir": str(cd), "binary_sha256": rec.get("binary_sha256"),
                                         "record_version": rec.get("version", 1), "reused": True})
                chosen = (cd, rec)
            else:
                if rec is not None and not args.analyze_only:
                    stale = cd.with_name(cd.name + ".stale-%d" % int(time.time()))
                    cd.rename(stale)
                    print("stale record moved to", stale.name, flush=True)
                if not args.analyze_only:
                    write_inputs(c, cd, rendered)
                    to_run.append(c)
                chosen = (cd, None)
        plan[c.name] = chosen
    for c in cases:
        fixtures.dump_json(plan[c.name][0] / "case.json", case_meta(c)) if (plan[c.name][0]).exists() else None

    # 2. execute the missing cases (references first: they are the slow runs)
    if to_run and not args.analyze_only:
        if binary is None:
            ap.error("--binary is required to run %d missing case(s)" % len(to_run))
        ordered = sorted(to_run, key=lambda c: (0 if c.expect.get("reference_role") else 1, c.name))
        t0 = time.time()

        def work(c):
            try:
                rec = execute(binary, binary_sha, out_dir / c.name, args.timeout)
                print("%-44s exit=%s wall=%.1fs" % (c.name, rec.get("exit"), rec.get("wall_seconds", 0)), flush=True)
                return c.name, rec
            except Exception as exc:  # noqa: BLE001 - one failure must not cancel the queue
                print("%-44s EXECUTION ERROR %s" % (c.name, exc), flush=True)
                traceback.print_exc()
                return c.name, {"exit": None, "error": str(exc)}

        with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
            for name, rec in pool.map(work, ordered):
                rec = dict(rec, lineage={"candidate": out_dir.name, "dir": str(out_dir / name), "binary_sha256": binary_sha,
                                         "record_version": 2, "reused": False})
                plan[name] = (out_dir / name, rec)
        print("ran %d cases in %.0f s" % (len(ordered), time.time() - t0), flush=True)

    # 3. parse and analyse (per case, never fatal), references first
    metas = {c.name: case_meta(c) for c in cases}
    ordered = sorted(cases, key=lambda c: (0 if c.expect.get("reference_role") else 1, c.name))
    entries = {}
    for c in ordered:
        cd, rec = plan[c.name]
        if rec is None:
            rec = load_record(cd) or {"exit": None}
        try:
            parsed = parse_output(cd)
            entries[c.name] = analyze(cd, metas[c.name], rec, parsed, entries)
        except Exception as exc:  # noqa: BLE001
            entries[c.name] = {"name": c.name, "stage": c.stage, "expect": c.expect, "reference": c.reference,
                               "status": "analysis error: %s" % exc, "exit": rec.get("exit"), "exit_recorded": rec.get("exit") is not None,
                               "lineage": rec.get("lineage"), "parse": {}}
    ordered_entries = [entries[c.name] for c in cases]

    # 4. verify against the manifest and report
    checks = verify(ordered_entries, manifest)
    candidates = sorted({(e.get("lineage") or {}).get("candidate", "?") + ":" + str((e.get("lineage") or {}).get("binary_sha256", "?"))[:12] for e in ordered_entries})
    note = "Candidates (directory:binary sha prefix): " + ", ".join(candidates)
    report_dir = Path(args.report_dir).resolve() if args.report_dir else out_dir
    report_dir.mkdir(parents=True, exist_ok=True)
    summarize(ordered_entries, checks, report_dir, note)
    print(open(report_dir / "summary.md").read())
    n_missing = sum(1 for c in checks.items if c["id"] == "C-R" and c["status"] != "pass")
    if args.verify and (checks.failures() or n_missing):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
