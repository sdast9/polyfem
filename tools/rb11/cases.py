"""RB-11 input-validation case matrix.

Each case writes an isolated scene directory (mesh files + ``scene.json``) and
declares the outcome the plan's acceptance requires:

* ``named_failure`` -- the input is genuinely invalid; PolyFEM must stop early
  with a named error (exit status 1, "PolyFEM stopped: ...") and write no
  accepted simulation output;
* ``accepted`` -- the input is valid (a control, a valid codimensional or open
  surface, a valid nonuniform material) and the run must complete (exit 0,
  all steps);
* ``accepted_with_notice`` -- valid but inside a documented envelope limit
  (a startup notice is expected, the run completes).

The runner records what actually happened; a mismatch is a finding, not a
test failure in itself.
"""
import copy
import os

import numpy as np

import fixtures as fx

CASES = {}


def case(name, group, expect, description, control=None, expect_error=None, steps=3):
    """Register a case. ``expect_error`` is the contract phrase a named failure
    must carry in its log; ``steps`` the number of saved steps (step_0 .. step_N)
    an accepted run must write (2 solved steps -> 3 files)."""
    if expect == "named_failure" and not expect_error:
        raise ValueError(f"{name}: a named failure needs its expected error phrase")

    def wrap(fn):
        CASES[name] = {"name": name, "group": group, "expect": expect,
                       "description": description, "control": control, "build": fn,
                       "expect_error": expect_error, "steps": steps}
        return fn
    return wrap


def _check(d, spec):
    """Sidecar for the runner: which exported field to verify and how."""
    fx.dump_json(os.path.join(d, "check.json"), spec)


def _cube(dirpath, **kw):
    V, T = fx.tet_cube(2)
    fx.write_msh41(os.path.join(dirpath, "cube.msh"), V, T, **kw)
    return V, T


def _two_body_cube(dirpath, **kw):
    """Unit cube; z<0.5 is body 1, z>=0.5 is body 2 (24 tets each).

    The msh writer emits one element block per body, so PolyFEM's global
    element order is body 1's tets (in generator order) followed by body 2's.
    ``V``, ``T`` and ``body`` are returned in that global order."""
    V, T = fx.tet_cube(2)
    cz = V[T].mean(axis=1)[:, 2]
    body = np.where(cz < 0.5, 1, 2)
    order = np.concatenate([np.flatnonzero(body == 1), np.flatnonzero(body == 2)])
    T, body = T[order], body[order]
    fx.write_msh41(os.path.join(dirpath, "cube.msh"), V, T, body_ids=body, **kw)
    return V, T, body


# ---------------------------------------------------------------------------
# Controls
# ---------------------------------------------------------------------------

@case("ctl-nocontact", "control", "accepted", "valid 2x2x2 tet cube, clamped bottom, 5 % compression, 2 quasistatic steps")
def _(d):
    _cube(d)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("ctl-contact", "control", "accepted", "valid cube 1 cm above the public-style slab, semi-implicit contact, 2 steps")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


@case("ctl-transient", "control", "accepted", "valid cube, transient (implicit Euler), consistent mass, P1")
def _(d):
    _cube(d)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(transient=True))


@case("ctl-two-bodies", "control", "accepted", "valid two-body cube with one material per body id")
def _(d):
    _two_body_cube(d)
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": 2e6, "nu": 0.3, "rho": 1000}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


# ---------------------------------------------------------------------------
# G1: geometry / topology
# ---------------------------------------------------------------------------

@case("g1-missing-mesh-file", "geometry", "named_failure", "geometry.mesh names a file that does not exist", "ctl-nocontact", expect_error='Unable to load the FE mesh')
def _(d):
    s = fx.base_scene(mesh="does_not_exist.msh")
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-msh-unknown-node-tag", "geometry", "named_failure", "Gmsh element references node tag 99 (no such node)", "ctl-nocontact", expect_error='references node tag 99, which is not a node of the file')
def _(d):
    _cube(d, bad_node_refs=[(5, 2, 99)])
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-msh-node-tag-zero", "geometry", "named_failure", "Gmsh element references node tag 0 (tags are 1-based)", "ctl-nocontact", expect_error='references node tag 0, which is not a node of the file')
def _(d):
    _cube(d, bad_node_refs=[(5, 2, 0)])
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-medit-index-out-of-range", "geometry", "named_failure", "MEDIT .mesh tetrahedron references vertex 999 of 27", "ctl-medit", expect_error='Unable to load the FE mesh')
def _(d):
    V, T = fx.tet_cube(2)
    raw = [[int(i) + 1 for i in t] for t in T]
    raw[5][2] = 999
    fx.write_medit(os.path.join(d, "cube.mesh"), V, T, raw_tets=raw)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(mesh="cube.mesh"))


@case("ctl-medit", "control", "accepted", "valid cube as a MEDIT .mesh file", None)
def _(d):
    V, T = fx.tet_cube(2)
    fx.write_medit(os.path.join(d, "cube.mesh"), V, T)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(mesh="cube.mesh"))


@case("g1-inverted-tet", "geometry", "named_failure", "one rest tetrahedron with negative orientation", "ctl-nocontact", expect_error='element 7 is flipped')
def _(d):
    V, T = fx.tet_cube(2)
    T = T.copy()
    T[7, [2, 3]] = T[7, [3, 2]]
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-degenerate-tet", "geometry", "named_failure", "one rest tetrahedron with zero volume (a vertex moved into the opposite face plane)", "ctl-nocontact", expect_error='is flipped')
def _(d):
    V, T = fx.tet_cube(2)
    V = V.copy()
    # move the interior vertex (0.5,0.5,0.5) onto the plane of one of its tets' faces
    centre = np.where((np.abs(V - 0.5) < 1e-12).all(axis=1))[0][0]
    tets = [r for r in range(len(T)) if centre in T[r]]
    t = T[tets[0]]
    others = [v for v in t if v != centre]
    a, b, c = (V[v] for v in others)
    n = np.cross(b - a, c - a)
    n /= np.linalg.norm(n)
    V[centre] -= np.dot(V[centre] - a, n) * n  # now coplanar: exactly one degenerate tet
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-repeated-vertex-in-tet", "geometry", "named_failure", "one tetrahedron lists the same vertex twice", "ctl-nocontact", expect_error='lists vertex 4 twice')
def _(d):
    V, T = fx.tet_cube(2)
    T = T.copy()
    T[3, 3] = T[3, 2]
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-duplicate-element", "geometry", "named_failure", "the same tetrahedron appears twice (double stiffness and mass)", "ctl-nocontact", expect_error='are the same element')
def _(d):
    V, T = fx.tet_cube(2)
    T = np.vstack([T, T[[11]]])
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene())


@case("g1-pinched-fe-mesh", "geometry", "accepted", "two cubes sharing exactly one vertex (nonmanifold but valid FE connectivity), no contact", "ctl-nocontact")
def _(d):
    V1, T1 = fx.tet_cube(2)
    V2, T2 = fx.tet_cube(2, origin=(1.0, 1.0, 1.0))
    # merge the shared corner (1,1,1)
    shared1 = np.where((np.abs(V1 - 1.0) < 1e-12).all(axis=1))[0][0]
    shared2 = np.where((np.abs(V2 - 1.0) < 1e-12).all(axis=1))[0][0]
    keep = [i for i in range(len(V2)) if i != shared2]
    remap = {old: new + len(V1) for new, old in enumerate(keep)}
    remap[shared2] = shared1
    V = np.vstack([V1, V2[keep]])
    T = np.vstack([T1, np.vectorize(remap.get)(T2)])
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    s = fx.base_scene()
    s["geometry"][0]["surface_selection"] = [
        {"id": 1, "axis": "-z", "position": 0.001},
        {"id": 2, "axis": "+z", "position": 1.999}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-obstacle-bad-face-index", "geometry", "named_failure", "obstacle OBJ face references vertex 99 of 4", "ctl-contact", expect_error='face 2 references vertex 98')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    Vs, Fs = fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    with open(os.path.join(d, "slab.obj"), "a") as fh:
        fh.write("f 1 2 99\n")
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


@case("g1-obstacle-duplicate-face", "geometry", "named_failure", "obstacle OBJ lists one triangle twice (duplicate collision topology)", "ctl-contact", expect_error='faces 0 and 2 are the same face')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    with open(os.path.join(d, "slab.obj"), "a") as fh:
        fh.write("f 1 2 3\n")
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


@case("g1-obstacle-tjunction", "geometry", "accepted", "obstacle surface with an edge shared by three faces (the slab plus a fin below and a fin above its front edge): valid nonmanifold obstacle; the runner asserts the incidence from the written OBJ", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    # slab z = 0 over [-1.5, 2.5]^2; edge (0,1) is its front edge at y = -1.5,
    # 1.5 away from the cube (y in [0, 1]); a fin hangs below it and one stands above it
    Vs = np.array([(-1.5, -1.5, 0.0), (2.5, -1.5, 0.0), (2.5, 2.5, 0.0), (-1.5, 2.5, 0.0),
                   (0.5, -1.5, -1.0), (0.5, -1.5, 1.0)])
    Fs = [(0, 1, 2), (0, 2, 3), (0, 4, 1), (0, 1, 5)]  # edge (0,1) in three faces
    degree = max(fx.edge_degrees(Fs).values())
    assert degree == 3, degree
    fx.write_obj(os.path.join(d, "slab.obj"), Vs, Fs)
    _check(d, {"obstacle_max_edge_degree": 3, "obstacle": "slab.obj"})
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


@case("g1-obstacle-edges-only", "geometry", "accepted", "codimensional obstacle: the slab's edges only (extract: edges)", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["geometry"][1]["extract"] = "edges"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-obstacle-points-only", "geometry", "accepted", "codimensional obstacle: a point cloud (extract: points) under the cube", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    g = np.linspace(0.1, 0.9, 5)
    Vp = np.array([(x, y, 0.0) for x in g for y in g])
    fx.write_obj(os.path.join(d, "slab.obj"), Vp, [])
    s = fx.base_scene(contact=True)
    s["geometry"][1]["extract"] = "points"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-initial-intersection-obstacle", "geometry", "named_failure", "cube rest configuration intersects the obstacle slab", "ctl-contact", expect_error='initial solution has intersections')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, -0.1))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


@case("g1-initial-intersection-two-bodies", "geometry", "named_failure", "two FE cubes overlap in the rest configuration", "ctl-contact", expect_error='initial solution has intersections')
def _(d):
    V1, T1 = fx.tet_cube(2, origin=(0, 0, 0.01))
    V2, T2 = fx.tet_cube(2, origin=(0.5, 0.5, 0.51))
    fx.write_msh41(os.path.join(d, "cube.msh"), V1, T1, body_ids=np.ones(len(T1), int))
    fx.write_msh41(os.path.join(d, "cube2.msh"), V2, T2, body_ids=2 * np.ones(len(T2), int))
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["geometry"].insert(1, {"mesh": "cube2.msh", "volume_selection": 2})
    s["geometry"][0]["volume_selection"] = 1
    s["materials"] = [{"id": 1, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000},
                      {"id": 2, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-obstacle-extract-volume", "geometry", "accepted", "obstacle with extract: volume (the spec default; the reader documents it as an alias for surface on obstacles)", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["geometry"][1]["extract"] = "volume"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g1-degenerate-obstacle-face", "geometry", "named_failure", "obstacle OBJ contains a zero-area (collinear) triangle", "ctl-contact", expect_error='face 2 has zero area')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    Vs, Fs = fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    with open(os.path.join(d, "slab.obj"), "a") as fh:
        fh.write("v 0.5 -1.5 0.0\n")  # on the edge v1-v2
        fh.write("f 1 5 2\n")
    fx.dump_json(os.path.join(d, "scene.json"), fx.base_scene(contact=True))


# ---------------------------------------------------------------------------
# G2: units
# ---------------------------------------------------------------------------

@case("g2-geometry-unit-dimension", "units", "named_failure", "geometry.unit is a mass unit (kg) for a length", "ctl-nocontact", expect_error='Cannot convert kg to m')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["geometry"][0]["unit"] = "kg"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g2-geometry-unit-unknown", "units", "named_failure", "geometry.unit is not a unit (\"furlongs_per_fortnight\")", "ctl-nocontact", expect_error='Cannot convert')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["geometry"][0]["unit"] = "furlongs_per_fortnight"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g2-material-unit-dimension", "units", "named_failure", "E given in metres ({value: 1e6, unit: m})", "ctl-nocontact", expect_error='Cannot convert m to Pa')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"]["E"] = {"value": 1e6, "unit": "m"}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g2-time-unit-dimension", "units", "named_failure", "dt given in metres", "ctl-nocontact", expect_error='Invalid input json')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["time"]["dt"] = {"value": 0.25, "unit": "m"}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g2-units-mm-raw", "units", "accepted_with_notice", "declared millimetre unit system with raw-mm numbers (RB-09 envelope: SI-default force tolerance, absolute CCD clearance)", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, size=1000.0, origin=(0, 0, 10.0))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0, half=1500.0)
    s = fx.base_scene(contact=True, top_value=("0", "0", "-50*t"))
    s["units"] = {"length": "mm", "mass": "kg", "time": "s"}
    s["geometry"][0]["surface_selection"] = [{"id": 2, "axis": "+z", "position": 1009.0}]
    s["materials"] = {"type": "NeoHookean", "E": 1.0, "nu": 0.3, "rho": 1e-6}  # 1e6 Pa = 1 kg/(mm s^2)
    s["contact"]["dhat"] = 1.0
    _check(d, {"log_contains": ["characteristic_force_density is the SI default", "caps the per-step clearance at 1e-4 length units = 0.0001 dhat"]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g2-material-unit-consistent", "units", "accepted", "E given as {value: 1, unit: MPa} (must equal the 1e6 Pa control)", "ctl-nocontact")
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"]["E"] = {"value": 1.0, "unit": "MPa"}
    fx.dump_json(os.path.join(d, "scene.json"), s)


# ---------------------------------------------------------------------------
# G3: material parameters
# ---------------------------------------------------------------------------

def _mat_case(name, desc, control="ctl-nocontact", expect="named_failure", transient=False, log_contains=None, expect_error=None, **params):
    @case(name, "material", expect, desc, control, expect_error=expect_error)
    def _(d):
        _cube(d)
        s = fx.base_scene(transient=transient)
        s["materials"].update(params)
        if log_contains:
            _check(d, {"log_contains": log_contains})
        fx.dump_json(os.path.join(d, "scene.json"), s)


_mat_case("g3-E-negative", "E = -1e6", E=-1e6, expect_error="Young's modulus must be positive")
_mat_case("g3-E-zero", "E = 0", E=0.0, expect_error='shear modulus must be positive')
_mat_case("g3-nu-half", "nu = 0.5 (lambda infinite for a compressible law)", nu=0.5, expect_error='must be finite')
_mat_case("g3-nu-above-half", "nu = 0.7 (lambda negative)", nu=0.7, expect_error="Poisson's ratio must lie in (-1, 1/2)")
_mat_case("g3-nu-below-minus-one", "nu = -1.5", nu=-1.5, expect_error="Poisson's ratio must lie in (-1, 1/2)")
_mat_case("g3-E-nan-expression", "E = \"0/0\" (NaN expression)", E="0/0", expect_error='must be finite')
_mat_case("g3-E-inf-expression", "E = \"1/0\" (infinite expression)", E="1/0", expect_error='must be finite')
_mat_case("g3-rho-negative-transient", "rho = -1000 in a transient run", transient=True, rho=-1000, expect_error='density must be nonnegative')
_mat_case("g3-rho-zero-transient", "rho = 0 in a transient run (the inertia term vanishes; valid but must be announced)", expect="accepted_with_notice", transient=True, rho=0,
          log_contains=["density is zero"])
_mat_case("g3-rho-zero-quasistatic", "rho = 0 in a quasistatic run without gravity (valid: massless)", expect="accepted", rho=0)
_mat_case("g3-nu-negative-valid", "nu = -0.2 (auxetic, valid)", expect="accepted", nu=-0.2)


@case("g3-E-file-missing", "material", "named_failure", "E names a value file that does not exist (currently parsed as an expression)", "ctl-nocontact", expect_error='is not an existing value file')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"]["E"] = "materials/E_values.txt"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-short", "material", "named_failure", "per-element E file with 10 rows for 48 elements", "g3-E-file-valid", expect_error='has 10 entries')
def _(d):
    _cube(d)
    np.savetxt(os.path.join(d, "E.txt"), np.full(10, 1e6))
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-long", "material", "named_failure", "per-element E file with 60 rows for 48 elements (does not describe this mesh)", "g3-E-file-valid", expect_error='has 60 entries')
def _(d):
    _cube(d)
    np.savetxt(os.path.join(d, "E.txt"), np.full(60, 1e6))
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-nonfinite", "material", "named_failure", "per-element E file containing a nan row", "g3-E-file-valid", expect_error='Cannot read material matrix')
def _(d):
    _cube(d)
    vals = np.full(48, 1e6)
    vals[17] = np.nan
    np.savetxt(os.path.join(d, "E.txt"), vals)
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-valid", "material", "accepted", "per-element E file with one row per element (valid nonuniform material); exported E must match", None)
def _(d):
    _cube(d)
    V, T = fx.tet_cube(2)
    cz = V[T].mean(axis=1)[:, 2]
    vals = np.where(cz < 0.5, 1e6, 3e6)  # lower half 1e6, upper half 3e6, one row per element
    np.savetxt(os.path.join(d, "E.txt"), vals, fmt="%.10g")
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    _check(d, {"field": "E", "top_expect": 3e6, "bottom_expect": 1e6})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-two-bodies-global", "material", "accepted", "two bodies, material array, one GLOBAL-length E file (HDA contract): the exported E per element must match the file row of that element", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_cube(d)
    # global rows: body-1 elements (first 24 in file order) 1e6, body-2 rows 3e6
    vals = np.where(body == 1, 1e6, 3e6)
    np.savetxt(os.path.join(d, "E.txt"), vals, fmt="%.10g")
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000}]
    _check(d, {"field": "E", "top_expect": 3e6, "bottom_expect": 1e6})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-two-bodies-local", "material", "accepted", "two bodies, material array, one BODY-LOCAL E file per body (upstream #333 contract): exported E must match", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_cube(d)
    n1 = int((body == 1).sum())
    n2 = int((body == 2).sum())
    np.savetxt(os.path.join(d, "E1.txt"), np.full(n1, 1e6), fmt="%.10g")
    np.savetxt(os.path.join(d, "E2.txt"), np.full(n2, 2e6), fmt="%.10g")
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": "E1.txt", "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": "E2.txt", "nu": 0.3, "rho": 1000}]
    _check(d, {"field": "E", "top_expect": 2e6, "bottom_expect": 1e6})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-missing-material-for-body", "material", "named_failure", "two bodies, materials only for body 1 (body 2 has no material)", "ctl-two-bodies", expect_error='No material for body id [2]')
def _(d):
    _two_body_cube(d)
    s = fx.base_scene()
    s["materials"] = [{"id": 1, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-material-id-unmatched", "material", "named_failure", "material array names body 7; the mesh has bodies 1 and 2 only", "ctl-two-bodies", expect_error='No material for body ids [1, 2]')
def _(d):
    _two_body_cube(d)
    s = fx.base_scene()
    s["materials"] = [{"id": 7, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-lump-mass-p2-transient", "material", "accepted_with_notice", "lump_mass_matrix with quadratic elements in a transient run: row-sum lumping is indefinite on the P2 corners (RB-22 observation); a warning, because upstream lumps quadratic bodies in every contact example", "g3-lump-mass-p1-transient")
def _(d):
    _cube(d)
    s = fx.base_scene(transient=True)
    s["space"] = {"discr_order": 2}
    s["solver"]["advanced"] = {"lump_mass_matrix": True}
    _check(d, {"log_contains": ["nonpositive nodal mass"]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-lump-mass-p1-transient", "material", "accepted", "lump_mass_matrix with linear elements in a transient run (valid)", None)
def _(d):
    _cube(d)
    s = fx.base_scene(transient=True)
    s["solver"]["advanced"] = {"lump_mass_matrix": True}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-consistent-mass-p2-transient", "material", "accepted", "quadratic elements, consistent mass, transient (valid control for the lumped P2 case)", None)
def _(d):
    _cube(d)
    s = fx.base_scene(transient=True)
    s["space"] = {"discr_order": 2}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-hgo-k2-zero", "material", "named_failure", "HGOFiber with k2 = 0 (the exponential fibre law divides by k2)", "g3-hgo-valid", expect_error='k2 must be positive')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"] = {"type": "MaterialSum", "models": [
        {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
        {"type": "HGOFiber", "k1": 1e4, "k2": 0.0, "fiber_direction": [0, 0, 1]}], "rho": 1000}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-hgo-dispersion-kappa-out-of-range", "material", "named_failure", "HGODispersion with kappa = 0.6 (the dispersion parameter lives in [0, 1/3])", "g3-hgo-valid", expect_error='kappa must lie in [0, 1/3]')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"] = {"type": "MaterialSum", "models": [
        {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
        {"type": "HGODispersion", "k1": 1e4, "k2": 5.0, "kappa": 0.6, "fiber_direction": [0, 0, 1]}], "rho": 1000}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-hgo-valid", "material", "accepted", "MaterialSum NeoHookean + HGOFiber with a constant fibre (valid)", None)
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["materials"] = {"type": "MaterialSum", "models": [
        {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
        {"type": "HGOFiber", "k1": 1e4, "k2": 5.0, "fiber_direction": [0, 0, 1]}], "rho": 1000}
    fx.dump_json(os.path.join(d, "scene.json"), s)


# ---------------------------------------------------------------------------
# G4: fibres
# ---------------------------------------------------------------------------

def _fiber_vtk(path, vectors, field="FIB_DIR1"):
    with open(path, "w") as fh:
        fh.write("# vtk DataFile Version 3.0\nRB-11 fibres\nASCII\nDATASET UNSTRUCTURED_GRID\n")
        fh.write(f"CELL_DATA {len(vectors)}\nVECTORS {field} float\n")
        for v in vectors:
            fh.write(f"{v[0]:.9f} {v[1]:.9f} {v[2]:.9f}\n")


def _hgo_file_scene(field="FIB_DIR1"):
    s = fx.base_scene()
    s["materials"] = {"type": "MaterialSum", "models": [
        {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
        {"type": "HGOFiber", "k1": 1e4, "k2": 5.0,
         "fiber_direction": {"type": "per_element_file", "path": "fibers.vtk", "field": field}}], "rho": 1000}
    return s


@case("g4-fiber-file-valid", "fiber", "accepted", "per-element fibre file, one unit vector per element (valid); exported fiber_direction must match", "g3-hgo-valid")
def _(d):
    V, T = _cube(d)
    c = V[T].mean(axis=1) - np.array([0.5, 0.5, -0.75])
    c /= np.linalg.norm(c, axis=1, keepdims=True)
    _fiber_vtk(os.path.join(d, "fibers.vtk"), c)
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-zero-vector", "fiber", "named_failure", "per-element fibre file with a zero vector on element 5", "g4-fiber-file-valid", expect_error='Zero-length fiber vector')
def _(d):
    V, T = _cube(d)
    c = np.tile([0.0, 0.0, 1.0], (48, 1))
    c[5] = 0
    _fiber_vtk(os.path.join(d, "fibers.vtk"), c)
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-file-short", "fiber", "named_failure", "per-element fibre file with 10 vectors for 48 elements", "g4-fiber-file-valid", expect_error='has 10 vectors')
def _(d):
    _cube(d)
    _fiber_vtk(os.path.join(d, "fibers.vtk"), np.tile([0.0, 0.0, 1.0], (10, 1)))
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-file-long", "fiber", "named_failure", "per-element fibre file with 60 vectors for 48 elements (does not describe this mesh)", "g4-fiber-file-valid", expect_error='has 60 vectors')
def _(d):
    _cube(d)
    _fiber_vtk(os.path.join(d, "fibers.vtk"), np.tile([0.0, 0.0, 1.0], (60, 1)))
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-field-missing", "fiber", "named_failure", "per-element fibre file without the requested VECTORS field", "g4-fiber-file-valid", expect_error='not found under CELL_DATA')
def _(d):
    _cube(d)
    _fiber_vtk(os.path.join(d, "fibers.vtk"), np.tile([0.0, 0.0, 1.0], (48, 1)), field="OTHER")
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-nonfinite", "fiber", "named_failure", "per-element fibre file with a nan component", "g4-fiber-file-valid", expect_error='is not three finite numbers')
def _(d):
    _cube(d)
    c = np.tile([0.0, 0.0, 1.0], (48, 1))
    c[9, 0] = np.nan
    _fiber_vtk(os.path.join(d, "fibers.vtk"), c)
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-file-two-bodies-global", "fiber", "accepted", "two bodies, material array, one GLOBAL-length fibre file on body 2: exported fibres on body 2 must match the file rows of those elements", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_cube(d)
    # global rows: body-1 rows point +x, body-2 rows point +y
    c = np.where((body == 1)[:, None], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])
    _fiber_vtk(os.path.join(d, "fibers.vtk"), c)
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "MaterialSum", "models": [
            {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
            {"type": "HGOFiber", "k1": 1e4, "k2": 5.0,
             "fiber_direction": {"type": "per_element_file", "path": "fibers.vtk", "field": "FIB_DIR1"}}], "rho": 1000}]
    _check(d, {"field": "fiber_direction", "top_expect": [0.0, 1.0, 0.0]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


# ---------------------------------------------------------------------------
# G5: prescribed motion
# ---------------------------------------------------------------------------

@case("g5-duplicate-dirichlet-id", "bc", "named_failure", "two dirichlet_boundary entries for id 2 with different values", "ctl-nocontact", expect_error='two entries for id 2')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["boundary_conditions"]["dirichlet_boundary"].append({"id": 2, "value": ["0", "0", "0.05*t"]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g5-conflicting-shared-nodes", "bc", "named_failure", "two adjacent Dirichlet faces prescribe different z-values on their shared edge nodes", "g5-consistent-shared-nodes", expect_error='Conflicting Dirichlet values')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["geometry"][0]["surface_selection"] = [
        {"id": 1, "axis": "-z", "position": 0.001},
        {"id": 2, "axis": "+z", "position": 0.999},
        {"id": 3, "axis": "+x", "position": 0.999}]
    s["boundary_conditions"]["dirichlet_boundary"] = [
        {"id": 1, "value": [0, 0, 0]},
        {"id": 2, "value": ["0", "0", "-0.05*t"]},
        {"id": 3, "value": ["0", "0", "0.05*t*z"]}]  # agrees with the bottom (0 at z=0) but not with the top edge (+0.05t vs -0.05t at z=1)
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g5-consistent-shared-nodes", "bc", "accepted", "three Dirichlet faces with different ids whose values agree on every shared edge (valid)", None)
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["geometry"][0]["surface_selection"] = [
        {"id": 1, "axis": "-z", "position": 0.001},
        {"id": 2, "axis": "+z", "position": 0.999},
        {"id": 3, "axis": "+x", "position": 0.999}]
    s["boundary_conditions"]["dirichlet_boundary"] = [
        {"id": 1, "value": [0, 0, 0]},
        {"id": 2, "value": ["0", "0", "-0.05*t"]},
        {"id": 3, "value": ["0", "0", "-0.05*t*z"]}]  # 0 on the bottom edge, -0.05t on the top edge: consistent with both
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g5-component-split-shared-nodes", "bc", "accepted", "adjacent faces prescribe different components on the shared edge (dimension flags; valid symmetry-plane pattern)", None)
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["geometry"][0]["surface_selection"] = [
        {"id": 1, "axis": "-z", "position": 0.001},
        {"id": 2, "axis": "+z", "position": 0.999},
        {"id": 3, "axis": "-x", "position": 0.001}]
    s["boundary_conditions"]["dirichlet_boundary"] = [
        {"id": 1, "value": [0, 0, 0], "dimension": [False, False, True]},
        {"id": 2, "value": ["0", "0", "-0.05*t"]},
        {"id": 3, "value": [0, 0, 0], "dimension": [True, False, False]}]
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g5-fully-prescribed-body", "bc", "accepted", "every node Dirichlet (a unit cube of 6 tets whose 8 vertices are all on the boundary, the whole boundary prescribed): zero free DOFs, a valid trivial solve whose solution is the prescribed values (RBR-05; was SIGSEGV after step_0)", "ctl-nocontact")
def _(d):
    V, T = fx.tet_cube(1)
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    s = fx.base_scene(transient=True)
    s["geometry"][0]["surface_selection"] = 1
    s["boundary_conditions"]["dirichlet_boundary"] = [{"id": 1, "value": ["0.05*t", "0", "0"]}]
    s["output"]["paraview"]["options"]["velocity"] = True
    fx.dump_json(os.path.join(d, "scene.json"), s)
    _check(d, {"log_contains": ["No free degrees of freedom: every DOF is prescribed; the step's solution is the prescribed values"]})


@case("g5-obstacle-displacement-conflict", "bc", "named_failure", "an obstacle surface id carries both an obstacle_displacements entry and a dirichlet_boundary entry with different values", "ctl-contact", expect_error='prescribed twice with different values')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["geometry"][1]["surface_selection"] = 5
    s["boundary_conditions"]["obstacle_displacements"] = [{"id": 5, "value": ["0", "0", "0.01*t"]}]
    s["boundary_conditions"]["dirichlet_boundary"].append({"id": 5, "value": ["0", "0", "-0.01*t"]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


# ---------------------------------------------------------------------------
# G6: solver settings that alias silently
# ---------------------------------------------------------------------------

@case("g6-broad-phase-sap", "settings", "accepted", "solver.contact.CCD.broad_phase = sweep_and_prune: offered by the spec but absent from the enum map, so it silently ran the hash grid (RB-05 observation); the run must use the sweep-and-prune broad phase", "ctl-contact")
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["solver"]["contact"]["CCD"] = {"broad_phase": "sweep_and_prune"}
    _check(d, {"log_contains": ["broad phase SweepAndPrune"], "log_excludes": ["broad phase HashGrid"]})
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g6-broad-phase-unknown", "settings", "named_failure", "solver.contact.CCD.broad_phase = \"hashgrid\" (a misspelling the spec rejects only when strict validation is on)", "ctl-contact", expect_error='Invalid input json')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["solver"]["contact"]["CCD"] = {"broad_phase": "hashgrid"}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g6-broad-phase-stq", "settings", "named_failure", "solver.contact.CCD.broad_phase = sweep_and_tiniest_queue (same)", "ctl-contact", expect_error='requires CUDA')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["solver"]["contact"]["CCD"] = {"broad_phase": "STQ"}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g6-dt-zero", "settings", "named_failure", "time.dt = 0 with tend (division by zero in the step count)", "ctl-nocontact", expect_error='time.dt must be a positive')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["time"] = {"dt": 0.0, "tend": 0.5, "quasistatic": True}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g6-tend-before-t0", "settings", "named_failure", "time.tend < time.t0", "ctl-nocontact", expect_error='time.tend must be a finite time after time.t0')
def _(d):
    _cube(d)
    s = fx.base_scene()
    s["time"] = {"t0": 1.0, "tend": 0.5, "time_steps": 2, "quasistatic": True}
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g6-dhat-zero", "settings", "named_failure", "contact.dhat = 0", "ctl-contact", expect_error='contact.dhat must be a positive')
def _(d):
    V, T = fx.tet_cube(2, origin=(0, 0, 0.01))
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T)
    fx.slab_obj(os.path.join(d, "slab.obj"), z=0.0)
    s = fx.base_scene(contact=True)
    s["contact"]["dhat"] = 0.0
    fx.dump_json(os.path.join(d, "scene.json"), s)


def build_all(root, names=None):
    out = []
    for name, c in CASES.items():
        if names and name not in names:
            continue
        d = os.path.join(root, name)
        os.makedirs(d, exist_ok=False)
        c["build"](d)
        out.append({k: v for k, v in c.items() if k != "build"})
    return out


# ---------------------------------------------------------------------------
# Review additions (2026-09-13): law- and dimension-specific domains, fibre
# representations, per-element transfer with unique values, oracle controls
# ---------------------------------------------------------------------------

def _square(d):
    V, T = fx.tri_square(2)
    fx.write_msh41(os.path.join(d, "square.msh"), V, T)
    return V, T


def _hgo_dispersion_material(kappa, dim, composite=True):
    fibre = [0, 1] if dim == 2 else [0, 0, 1]
    hgo = {"type": "HGODispersion", "k1": 1e4, "k2": 5.0, "kappa": kappa, "fiber_direction": fibre}
    if not composite:
        hgo["rho"] = 1000
        return hgo
    return {"type": "MaterialSum", "models": [{"type": "NeoHookean", "E": 1e6, "nu": 0.3}, hgo], "rho": 1000}


def _kappa_case(name, kappa, dim, composite, expect, desc, expect_error=None):
    @case(name, "material", expect, desc, "g3-hgo-valid", expect_error=expect_error)
    def _(d):
        if dim == 2:
            _square(d)
            s = fx.base_scene(mesh="square.msh", dim=2)
        else:
            _cube(d)
            s = fx.base_scene()
        s["materials"] = _hgo_dispersion_material(kappa, dim, composite)
        fx.dump_json(os.path.join(d, "scene.json"), s)


_kappa_case("g3-hgo-dispersion-2d-kappa-0.4", 0.4, 2, True, "accepted", "2D HGODispersion (composite) with kappa = 0.4: valid, the law's domain is [0, 1/d] = [0, 1/2] in 2D (review counterexample)")
_kappa_case("g3-hgo-dispersion-2d-kappa-0.5", 0.5, 2, True, "accepted", "2D HGODispersion (composite) at the endpoint kappa = 1/2: valid")
_kappa_case("g3-hgo-dispersion-2d-kappa-0.6", 0.6, 2, True, "named_failure", "2D HGODispersion (composite) with kappa = 0.6 > 1/2", expect_error="kappa must lie in [0, 1/2]")
_kappa_case("g3-hgo-dispersion-2d-plain-kappa-0.4", 0.4, 2, False, "accepted", "2D HGODispersion as a plain material with kappa = 0.4: valid (dispatch without a composite)")
_kappa_case("g3-hgo-dispersion-2d-plain-kappa-0.6", 0.6, 2, False, "named_failure", "2D HGODispersion as a plain material with kappa = 0.6", expect_error="kappa must lie in [0, 1/2]")
_kappa_case("g3-hgo-dispersion-kappa-third", 1.0 / 3.0, 3, True, "accepted", "3D HGODispersion at the endpoint kappa = 1/3: valid")
_kappa_case("g3-hgo-dispersion-kappa-negative", -0.1, 3, True, "named_failure", "3D HGODispersion with kappa = -0.1", expect_error="kappa must lie in [0, 1/3]")


def _fibre_case(name, fibre, expect, desc, expect_error=None, dim=3, check=None, law="HGOFiber"):
    """NeoHookean + a fibre law with the given direction. Normalisation is the
    law's business and differs: HGODispersion normalises the direction
    (I4Bar_generic(normalize=true)); HGOFiber and ActiveFiber use the raw
    vector, so its length is part of their model (a pre-stretch). The checks
    only refuse zero, nonfinite and wrongly sized directions."""
    @case(name, "fiber", expect, desc, "g3-hgo-valid", expect_error=expect_error)
    def _(d):
        if dim == 2:
            _square(d)
            s = fx.base_scene(mesh="square.msh", dim=2)
        else:
            _cube(d)
            s = fx.base_scene()
        model = {"type": law, "k1": 1e4, "k2": 5.0, "fiber_direction": fibre}
        if law == "HGODispersion":
            model["kappa"] = 0.1
        s["materials"] = {"type": "MaterialSum", "models": [
            {"type": "NeoHookean", "E": 1e6, "nu": 0.3}, model], "rho": 1000}
        if check:
            _check(d, check)
        fx.dump_json(os.path.join(d, "scene.json"), s)


_fibre_case("g4-fiber-constant-zero", [0, 0, 0], "named_failure", "constant fibre direction [0, 0, 0] (review counterexample: was accepted, the fibre term silently vanished)", expect_error="is a zero vector")
_fibre_case("g4-fiber-constant-nonunit", [0, 0, 2], "accepted", "constant fibre direction [0, 0, 2] under HGODispersion, which normalises the direction: valid; the exported reference direction must point along +z",
            check={"field": "fiber_direction", "top_expect": [0.0, 0.0, 1.0], "normalize": True}, law="HGODispersion")
_fibre_case("g4-fiber-expression-zero", ["0*x", "0", "0*z"], "named_failure", "expression fibre direction that evaluates to zero everywhere", expect_error="fibre direction is a zero vector")
_fibre_case("g4-fiber-expression-valid", ["cos(x)", "0", "sin(x)"], "accepted", "unit-length expression fibre direction (cos x, 0, sin x): valid, nonzero everywhere")
_fibre_case("g4-fiber-wrong-dimension", [0, 0, 1], "named_failure", "a 3-component fibre direction on a 2D problem", expect_error="components but the problem is 2D", dim=2)
_fibre_case("g4-fiber-2d-constant-valid", [0, 1], "accepted", "2D constant fibre direction [0, 1]: valid", dim=2)


def _unique_values(n, base=1e6):
    return base * (1.0 + np.arange(n) / (2.0 * n))


def _transfer_check(d, V, T, body, values=None, vectors=None, field="E", bodies=None, expected_status="pass"):
    cent = V[T].mean(axis=1)
    spec = {"per_element": {"field": field, "centroids": cent.tolist(), "body_ids": [int(b) for b in body]}}
    if values is not None:
        spec["per_element"]["values"] = [float(v) for v in values]
    if vectors is not None:
        spec["per_element"]["vectors"] = [[float(x) for x in v] for v in vectors]
    if bodies is not None:
        spec["per_element"]["bodies"] = bodies  # only these bodies are checked
    spec["expected_status"] = expected_status
    _check(d, spec)


@case("g3-E-file-unique-per-element", "material", "accepted", "single body, per-element E file with a unique value per element: every output element must carry its own row (transfer oracle with unique values)")
def _(d):
    V, T = _cube(d)
    vals = _unique_values(len(T))
    np.savetxt(os.path.join(d, "E.txt"), vals, fmt="%.12g")
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    _transfer_check(d, V, T, np.ones(len(T), int), values=vals)
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-permuted-oracle", "material", "accepted", "same as the unique-value case but the file rows are permuted within the body: PolyFEM cannot tell (a valid input), and the transfer oracle MUST report a mismatch -- this case documents that the oracle discriminates element order")
def _(d):
    V, T = _cube(d)
    vals = _unique_values(len(T))
    rng = np.random.default_rng(11)
    np.savetxt(os.path.join(d, "E.txt"), vals[rng.permutation(len(T))], fmt="%.12g")
    s = fx.base_scene()
    s["materials"]["E"] = "E.txt"
    _transfer_check(d, V, T, np.ones(len(T), int), values=vals, expected_status="fail")
    fx.dump_json(os.path.join(d, "scene.json"), s)


def _two_body_unequal(d, **kw):
    """Unit cube; z < 0.25 is body 1 (8 tets), the rest body 2 (40 tets); global order body 1 then 2."""
    V, T = fx.tet_cube(2)
    cz = V[T].mean(axis=1)[:, 2]
    body = np.where(cz < 0.25, 1, 2)
    order = np.concatenate([np.flatnonzero(body == 1), np.flatnonzero(body == 2)])
    T, body = T[order], body[order]
    fx.write_msh41(os.path.join(d, "cube.msh"), V, T, body_ids=body, **kw)
    return V, T, body


@case("g3-E-file-two-bodies-unequal-global", "material", "accepted", "unequal bodies (8 + 40 tets), material array, one global-length E file with unique values: each element of both bodies must carry its own row", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_unequal(d)
    vals = _unique_values(len(T))
    np.savetxt(os.path.join(d, "E.txt"), vals, fmt="%.12g")
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000}]
    _transfer_check(d, V, T, body, values=vals)
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-two-bodies-unequal-local", "material", "accepted", "unequal bodies (8 + 40 tets), one body-local E file per body with unique values (upstream #333 contract): each element must carry the row of its body-local index", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_unequal(d)
    n1, n2 = int((body == 1).sum()), int((body == 2).sum())
    v1, v2 = _unique_values(n1, 1e6), _unique_values(n2, 3e6)
    np.savetxt(os.path.join(d, "E1.txt"), v1, fmt="%.12g")
    np.savetxt(os.path.join(d, "E2.txt"), v2, fmt="%.12g")
    expected = np.concatenate([v1, v2])  # global order is body 1 then body 2, each in local order
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": "E1.txt", "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": "E2.txt", "nu": 0.3, "rho": 1000}]
    _transfer_check(d, V, T, body, values=expected)
    fx.dump_json(os.path.join(d, "scene.json"), s)


@case("g3-E-file-two-meshes-global", "material", "accepted", "two geometry meshes (body 1: cube at the origin, body 2: cube shifted by 2 in x), one global-length E file (96 unique rows in geometry order): every element of both meshes must carry its own row", "ctl-two-bodies")
def _(d):
    V1, T1 = fx.tet_cube(2)
    V2, T2 = fx.tet_cube(2, origin=(2.0, 0.0, 0.0))
    fx.write_msh41(os.path.join(d, "cube.msh"), V1, T1, body_ids=np.ones(len(T1), int))
    fx.write_msh41(os.path.join(d, "cube2.msh"), V2, T2, body_ids=2 * np.ones(len(T2), int))
    vals = _unique_values(len(T1) + len(T2))
    np.savetxt(os.path.join(d, "E.txt"), vals, fmt="%.12g")
    s = fx.base_scene()
    s["geometry"][0]["volume_selection"] = 1
    s["geometry"].append({"mesh": "cube2.msh", "volume_selection": 2, "surface_selection": [
        {"id": 1, "axis": "-z", "position": 0.001}, {"id": 2, "axis": "+z", "position": 0.999}]})
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "NeoHookean", "E": "E.txt", "nu": 0.3, "rho": 1000}]
    V = np.vstack([V1, V2])
    T = np.vstack([T1, T2 + len(V1)])
    body = np.concatenate([np.ones(len(T1), int), 2 * np.ones(len(T2), int)])
    _transfer_check(d, V, T, body, values=vals)
    fx.dump_json(os.path.join(d, "scene.json"), s)


def _unique_directions(n, seed=3):
    theta = 2 * np.pi * np.arange(n) / n
    v = np.stack([np.cos(theta), np.sin(theta), 0.3 + 0.4 * np.arange(n) / n], axis=1)
    return v / np.linalg.norm(v, axis=1, keepdims=True)


@case("g4-fiber-file-unique-per-element", "fiber", "accepted", "per-element fibre file with a unique direction per element (single body): the exported reference direction of every element must be its own row", "g3-hgo-valid")
def _(d):
    V, T = _cube(d)
    dirs = _unique_directions(len(T))
    _fiber_vtk(os.path.join(d, "fibers.vtk"), dirs)
    _transfer_check(d, V, T, np.ones(len(T), int), vectors=dirs, field="fiber_direction")
    fx.dump_json(os.path.join(d, "scene.json"), _hgo_file_scene())


@case("g4-fiber-file-two-bodies-unequal-global", "fiber", "accepted", "unequal bodies, fibre model on body 2 only, one global-length fibre file with unique directions: body 2's elements must carry their own rows", "ctl-two-bodies")
def _(d):
    V, T, body = _two_body_unequal(d)
    dirs = _unique_directions(len(T))
    _fiber_vtk(os.path.join(d, "fibers.vtk"), dirs)
    s = fx.base_scene()
    s["materials"] = [
        {"id": 1, "type": "NeoHookean", "E": 1e6, "nu": 0.3, "rho": 1000},
        {"id": 2, "type": "MaterialSum", "models": [
            {"type": "NeoHookean", "E": 1e6, "nu": 0.3},
            {"type": "HGOFiber", "k1": 1e4, "k2": 5.0,
             "fiber_direction": {"type": "per_element_file", "path": "fibers.vtk", "field": "FIB_DIR1"}}], "rho": 1000}]
    _transfer_check(d, V, T, body, vectors=dirs, field="fiber_direction", bodies=[2])
    fx.dump_json(os.path.join(d, "scene.json"), s)
