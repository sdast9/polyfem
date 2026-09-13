"""RB-11 fixture builders: tiny synthetic meshes and scene JSON.

Everything here is generated, public and small (a 2x2x2 tetrahedralised unit
cube, 48 tets); the matrix in ``cases.py`` perturbs these into the invalid
variants the plan requires and keeps a valid control next to each.
"""
import itertools
import json

import numpy as np


def tet_cube(n=2, size=1.0, origin=(0.0, 0.0, 0.0)):
    """Structured tetrahedralisation of an axis-aligned cube (6 tets/cell).

    Returns ``(V, T)`` with ``V`` an (m,3) float array and ``T`` an (k,4) int
    array of positively oriented tets (0-based)."""
    xs = np.linspace(0.0, size, n + 1)
    V = np.array([(origin[0] + x, origin[1] + y, origin[2] + z)
                  for x in xs for y in xs for z in xs])

    def vid(i, j, k):
        return (i * (n + 1) + j) * (n + 1) + k

    T = []
    # Kuhn subdivision along the main diagonal (0,0,0)->(1,1,1)
    for i, j, k in itertools.product(range(n), repeat=3):
        c = {(a, b, d): vid(i + a, j + b, k + d)
             for a in (0, 1) for b in (0, 1) for d in (0, 1)}
        for perm in itertools.permutations(range(3)):
            path = [(0, 0, 0)]
            cur = [0, 0, 0]
            for axis in perm:
                cur[axis] = 1
                path.append(tuple(cur))
            T.append([c[p] for p in path])
    T = np.array(T, dtype=int)
    T = orient(V, T)
    return V, T


def tri_square(n=2, size=1.0, origin=(0.0, 0.0)):
    """Structured triangulation of a square (2 triangles per cell, CCW)."""
    xs = np.linspace(0.0, size, n + 1)
    V = np.array([(origin[0] + x, origin[1] + y) for x in xs for y in xs])

    def vid(i, j):
        return i * (n + 1) + j

    T = []
    for i in range(n):
        for j in range(n):
            a, b, c, d = vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)
            T.append([a, b, c])
            T.append([a, c, d])
    return V, np.array(T, dtype=int)


def tet_volume(V, t):
    a, b, c, d = (V[i] for i in t)
    return float(np.dot(np.cross(b - a, c - a), d - a)) / 6.0


def orient(V, T):
    T = T.copy()
    for r in range(len(T)):
        if tet_volume(V, T[r]) < 0:
            T[r, [2, 3]] = T[r, [3, 2]]
    return T


def boundary_faces(T):
    count = {}
    for t in T:
        for f in ((t[0], t[2], t[1]), (t[0], t[1], t[3]), (t[0], t[3], t[2]), (t[1], t[2], t[3])):
            key = tuple(sorted(f))
            count.setdefault(key, []).append(f)
    return [fs[0] for fs in count.values() if len(fs) == 1]


def write_msh41(path, V, T, body_ids=None, node_tags=None, elem_tag_offset=1,
                bad_node_refs=None):
    """Gmsh 4.1 ASCII writer for tets; one volume entity per body id.

    ``node_tags`` overrides the (1-based) node tag of each vertex, and
    ``bad_node_refs`` is a list of (element row, corner, tag) substitutions
    applied verbatim -- used to build invalid-index fixtures."""
    V = np.asarray(V)
    T = np.asarray(T)
    dim = V.shape[1]
    n_corners = T.shape[1]
    elem_type = {3: 2, 4: 4}[n_corners]  # Gmsh: 2 = triangle, 4 = tetrahedron
    if body_ids is None:
        body_ids = np.ones(len(T), dtype=int)
    body_ids = np.asarray(body_ids)
    if node_tags is None:
        node_tags = np.arange(1, len(V) + 1)
    bodies = sorted(set(int(b) for b in body_ids))
    V3 = np.hstack([V, np.zeros((len(V), 3 - dim))]) if dim < 3 else V
    lo, hi = V3.min(axis=0), V3.max(axis=0)
    lines = ["$MeshFormat", "4.1 0 8", "$EndMeshFormat"]
    lines += ["$PhysicalNames", str(len(bodies))]
    for b in bodies:
        lines.append(f'{dim} {b} "body_{b}"')
    lines.append("$EndPhysicalNames")
    lines += ["$Entities", f"0 0 {len(bodies)} 0" if dim == 2 else f"0 0 0 {len(bodies)}"]
    for b in bodies:
        lines.append(f"{b} {lo[0]} {lo[1]} {lo[2]} {hi[0]} {hi[1]} {hi[2]} 1 {b} 0")
    lines.append("$EndEntities")
    lines += ["$Nodes", f"1 {len(V)} {node_tags.min()} {node_tags.max()}",
              f"{dim} {bodies[0]} 0 {len(V)}"]
    lines += [str(int(t)) for t in node_tags]
    lines += [f"{x:.17g} {y:.17g} {z:.17g}" for x, y, z in V3]
    lines.append("$EndNodes")
    lines += ["$Elements", f"{len(bodies)} {len(T)} {elem_tag_offset} {elem_tag_offset + len(T) - 1}"]
    refs = {}
    for (row, corner, tag) in (bad_node_refs or []):
        refs[(row, corner)] = tag
    tag = elem_tag_offset
    for b in bodies:
        rows = [r for r in range(len(T)) if body_ids[r] == b]
        lines.append(f"{dim} {b} {elem_type} {len(rows)}")
        for r in rows:
            corners = [refs.get((r, c), int(node_tags[T[r, c]])) for c in range(n_corners)]
            lines.append(f"{tag} " + " ".join(str(c) for c in corners))
            tag += 1
    lines.append("$EndElements")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def write_medit(path, V, T, refs=None, raw_tets=None):
    """MEDIT .mesh writer (1-based). ``raw_tets`` bypasses ``T`` verbatim."""
    lines = ["MeshVersionFormatted 1", "Dimension", "3", "Vertices", str(len(V))]
    lines += [f"{x:.17g} {y:.17g} {z:.17g} 0" for x, y, z in V]
    tets = raw_tets if raw_tets is not None else [[int(i) + 1 for i in t] for t in T]
    lines += ["Tetrahedra", str(len(tets))]
    for r, t in enumerate(tets):
        ref = 0 if refs is None else int(refs[r])
        lines.append(" ".join(str(i) for i in t) + f" {ref}")
    lines.append("End")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def edge_degrees(F):
    """{(a, b): number of faces containing the undirected edge}."""
    count = {}
    for f in F:
        n = len(f)
        for k in range(n):
            a, b = int(f[k]), int(f[(k + 1) % n])
            key = (min(a, b), max(a, b))
            count[key] = count.get(key, 0) + 1
    return count


def read_obj_faces(path):
    F = []
    for line in open(path):
        parts = line.split()
        if parts and parts[0] == "f":
            F.append(tuple(int(p.split("/")[0]) - 1 for p in parts[1:]))
    return F


def write_obj(path, V, F):
    lines = [f"v {x:.17g} {y:.17g} {z:.17g}" for x, y, z in V]
    lines += ["f " + " ".join(str(int(i) + 1) for i in f) for f in F]
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def slab_obj(path, z=0.0, half=1.5):
    V = np.array([(-half, -half, z), (half + 1, -half, z), (half + 1, half + 1, z), (-half, half + 1, z)])
    F = [(0, 1, 2), (0, 2, 3)]
    write_obj(path, V, F)
    return V, F


def base_scene(mesh="cube.msh", contact=False, transient=False, steps=2, dt=0.25,
               material=None, top_value=("0", "0", "-0.05*t"), dim=3):
    """A clamped-bottom, prescribed-top compression of the unit cube (or, with
    dim=2, of the unit square: clamped y = 0, prescribed y = 1)."""
    axis = "z" if dim == 3 else "y"
    if dim == 2 and top_value == ("0", "0", "-0.05*t"):
        top_value = ("0", "-0.05*t")
    scene = {
        "geometry": [
            {
                "mesh": mesh,
                "surface_selection": [
                    {"id": 1, "axis": "-" + axis, "position": 0.001},
                    {"id": 2, "axis": "+" + axis, "position": 0.999},
                ],
            }
        ],
        "materials": material or {"type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000},
        "time": {"dt": dt, "time_steps": steps, "quasistatic": not transient},
        "contact": {"enabled": bool(contact), "dhat": 1e-3},
        "boundary_conditions": {
            "dirichlet_boundary": [
                {"id": 1, "value": [0] * dim},
                {"id": 2, "value": list(top_value)},
            ]
        },
        "solver": {
            "max_threads": 1,
            "linear": {"solver": "Eigen::SimplicialLDLT"},
            "contact": {"barrier_stiffness": "semi_implicit"},
        },
        "output": {
            "log": {"level": "debug"},
            "paraview": {"file_name": "out.pvd", "options": {"material": True, "body_ids": True}},
        },
    }
    if contact:
        scene["geometry"].append({"mesh": "slab.obj", "is_obstacle": True})
        # bottom is free to land on the slab; only the top is prescribed
        scene["boundary_conditions"]["dirichlet_boundary"] = [
            {"id": 2, "value": list(top_value)}]
        scene["geometry"][0]["surface_selection"] = [
            {"id": 2, "axis": "+z", "position": 0.999}]
    return scene


def dump_json(path, data):
    with open(path, "w") as fh:
        json.dump(data, fh, indent=2)
