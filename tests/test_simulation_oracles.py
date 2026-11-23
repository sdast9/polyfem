"""Closed-form assembly and continuation checks through PolyFEM_bin."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from vtu_binary import read_arrays


VOLUME = 1 / 6
DT = 1 / 8
STEPS = 5
SPLIT = 2
BODY_A, BODY_B = 17, 53
ROTATION_90 = [0, -1, 0, 1, 0, 0, 0, 0, 1]
VERTICES = ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))
FIELD_TOLERANCES = {
    "solution": 2e-8, "velocity": 2e-7, "acceleration": 2e-6,
    "elastic_forces": 5e-5, "body_forces": 2e-8, "inertia_forces": 5e-5,
    "cauchy_stess_1": 3e-4, "cauchy_stess_2": 3e-4, "cauchy_stess_3": 3e-4,
    "energy": 2e-5, "body_ids": 0, "HookeLinearElasticity/C_44": 1e-8, "C_44": 1e-8,
}


def require(condition, detail):
    if not condition:
        raise AssertionError(detail)


def read_json(path):
    return json.loads(path.read_text())


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


@dataclass(frozen=True)
class Tet:
    body_id: int
    offset: int = 0

    @property
    def points(self):
        return tuple((x + self.offset, y, z) for x, y, z in VERTICES)

    def write(self, folder):
        tag = 10 * self.body_id
        mesh = folder / f"body-{self.body_id}.msh"
        rows = [f"{i} {x} {y} {z}" for i, (x, y, z) in enumerate(self.points, 1)]
        mesh.write_text("\n".join([
            "$MeshFormat", "2.2 0 8", "$EndMeshFormat", "$Nodes", "4", *rows,
            "$EndNodes", "$Elements", "1",
            f"1 4 2 {self.body_id} {self.body_id} 1 2 3 4", "$EndElements", "",
        ]))
        faces = folder / f"faces-{self.body_id}.txt"
        faces.write_text(f"{tag + 1} 0 1 2\n{tag + 2} 1 2 3\n{tag + 3} 0 2 3\n")
        return {"mesh": mesh.name, "volume_selection": self.body_id,
                "surface_selection": [{"file": faces.name}],
                "point_selection": {"id": tag + 4, "axis": "z", "position": 0.99}}

    def clamps(self):
        tag = 10 * self.body_id
        # The base is fixed, and the sloping face also fixes the apex y and z.
        return [{"id": tag + 1, "value": [0, 0, 0]},
                {"id": tag + 2, "value": [0, 0, 0], "dimension": [False, True, True]}]


A, B = Tet(BODY_A), Tet(BODY_B, 3)


def isotropic(tet):
    return {"type": "LinearElasticity", "id": tet.body_id, "rho": 120,
            "E": 30000 if tet == A else 50000, "nu": 0.25}


def hooke(frame=None):
    material = {"type": "HookeLinearElasticity", "id": BODY_A, "rho": 120,
                "elasticity_tensor": [80000, 70000, 60000, 0.2, 0.17, 0.13, 5000, 12000, 9000]}
    if frame is not None:
        material["fiber_direction"] = frame
    return material


def scene(tets, materials, nodal=(), tractions=()):
    return {
        "materials": materials,
        "units": {"length": "m", "mass": "kg", "time": "s", "characteristic_length": 1},
        "space": {"discr_order": 1},
        "solver": {"linear": {"solver": "Eigen::SimplicialLDLT"}, "max_threads": 1},
        "contact": {"enabled": False},
        "boundary_conditions": {
            "rhs": [0, 0, 0],
            "dirichlet_boundary": [entry for tet in tets for entry in tet.clamps()],
            "nodal_neumann_boundary": [{"id": 10 * tet.body_id + 4, "value": [f, 0, 0]}
                                      for tet, f in nodal],
            "neumann_boundary": [{"id": face, "value": [tx, 0, 0]} for face, tx in tractions],
        },
        "output": {
            "log": {"level": "info"},
            "paraview": {"file_name": "sim.pvd", "volume": True,
                         "options": {"use_hdf5": False, "tensor_values": True, "forces": True,
                                     "body_ids": True, "velocity": True, "acceleration": True}},
            "advanced": {"vis_boundary_only": False},
        },
    }


class Case:
    def __init__(self, root, name):
        self.folder = root / name
        self.folder.mkdir()
        self.metrics = {}

    def compare(self, name, actual, expected, atol):
        require(len(actual) == len(expected), (name, actual, expected))
        require(all(math.isfinite(value) for value in actual), (name, actual))
        error = max(abs(a - b) for a, b in zip(actual, expected))
        require(name not in self.metrics, f"duplicate measurement {name}")
        self.metrics[name] = {"actual": actual, "expected": expected, "atol": atol, "error": error}

    def finish(self):
        write_json(self.folder / "measurements.json", self.metrics)
        require(self.metrics, "no measurements")
        failures = [(name, m) for name, m in self.metrics.items() if m["error"] > m["atol"]]
        if failures:
            name, m = failures[0]
            raise AssertionError(f'{len(failures)}/{len(self.metrics)} checks failed. {name}: '
                                 f'got {m["actual"]}, expected {m["expected"]}, atol {m["atol"]}')
        return len(self.metrics)


class Solver:
    def __init__(self, binary, root, expected_commit):
        self.binary = binary
        self.env = dict(os.environ, OMP_NUM_THREADS="1", VECLIB_MAXIMUM_THREADS="1",
                        TMPDIR=str(root / "tmp"))
        (root / "tmp").mkdir()
        self.identity = json.loads(subprocess.check_output([str(binary), "--build_info"],
                                                           cwd=root, env=self.env, timeout=15))
        self.sha256 = hashlib.sha256(binary.read_bytes()).hexdigest()
        if expected_commit:
            version = self.identity["sources"]["polyfem"]
            require(version["commit"] == expected_commit and not version["dirty"], version)
        write_json(root / "identity.json", {"path": str(binary), "sha256": self.sha256,
                                            "build_info": self.identity})

    def solve(self, folder, tets=(), config=None, checkpoint=None):
        folder.mkdir()
        output = folder / "output"
        output.mkdir()
        if checkpoint is None:
            config["geometry"] = [tet.write(folder) for tet in tets]
            write_json(folder / "params.json", config)
            input_args = ["-j", "params.json"]
        else:
            input_args = ["--checkpoint", str(checkpoint.resolve(strict=True))]
        with (folder / "solve.log").open("w") as log:
            process = subprocess.run([str(self.binary), *input_args, "-o", str(output),
                                      "--max_threads", "1"], cwd=folder, env=self.env,
                                     stdout=log, stderr=subprocess.STDOUT, timeout=45)
        require(process.returncode == 0, (folder / "solve.log").read_text()[-4000:])
        manifest = read_json(output / "run-manifest.json")
        require(manifest["completion"]["exit_status"] == 0, manifest["completion"])
        require(manifest["build"] == self.identity, "solve build identity differs from --build_info")
        executable = manifest["process"]["executable"]
        require(Path(executable["path"]).resolve() == self.binary
                and executable["sha256"] == self.sha256, executable)
        return output


def read_fields(path, fields):
    arrays = {}
    for attrs, data in read_arrays(path, set(fields) | {None}):
        name = attrs.get("Name", "points")
        require(name not in arrays, (path, name))
        storage = {"Float64": "d", "Float32": "f", "Int32": "i", "Int64": "q"}[attrs["type"]]
        components = int(attrs.get("NumberOfComponents", "1"))
        arrays[name] = list(struct.iter_unpack("<" + storage * components, data))
    require(set(arrays) == set(fields) | {"points"}, (path, list(arrays), fields))
    require(all(len(v) == len(arrays["points"]) for v in arrays.values()), path)
    return arrays


def shear_fields(u, shear_modulus, applied_x):
    # grad(u) has only an xz component. On the unit tet u_x = u * z.
    # PolyFEM's nodal elastic force is minus the gradient of elastic energy.
    tau = shear_modulus * u
    n = VOLUME * tau
    return {
        "solution": [(0, 0, 0)] * 3 + [(u, 0, 0)],
        "elastic_forces": [(n, 0, n), (0, 0, -n), (0, 0, 0), (-n, 0, 0)],
        "body_forces": [(force, 0, 0) for force in applied_x],
        "cauchy_stess_1": [(0, 0, tau)] * 4,
        "cauchy_stess_2": [(0, 0, 0)] * 4,
        "cauchy_stess_3": [(tau, 0, 0)] * 4,
        "energy": [(tau * u / 2,)] * 4,
    }


def check_tet(case, label, path, tet, expected):
    expected = dict(expected, body_ids=[(tet.body_id,)] * 4)
    arrays = read_fields(path, expected)
    for vertex, point in enumerate(tet.points):
        ids = [i for i, p in enumerate(arrays["points"]) if math.dist(p, point) <= 1e-9]
        require(len(ids) == 1, f"{path}: expected one reference point at {point}, got {ids}")
        for field, rows in expected.items():
            case.compare(f"{label}.body{tet.body_id}.vertex{vertex}.{field}",
                         arrays[field][ids[0]], rows[vertex], FIELD_TOLERANCES[field])


def loaded_bodies(case, solver, tets, materials, prescribed=False):
    loads = [(A, 24), (B, -30)]
    config = scene(tets, materials, nodal=() if prescribed else loads)
    local_shear = {m["id"]: m["elasticity_tensor"][7] for m in materials if m["type"] == "HookeLinearElasticity"}
    config["output"]["paraview"]["options"]["material"] = bool(local_shear)
    if prescribed:
        for clamp in config["boundary_conditions"]["dirichlet_boundary"]:
            if clamp["id"] % 10 == 2:
                clamp["value"] = ["0.01*z", 0, 0]
                clamp["dimension"] = [True, True, True]
    out = solver.solve(case.folder / "run", tets, config)
    for (tet, f), g in zip(loads, (12000, 20000)):
        u = 0.01 if prescribed else f / (VOLUME * g)
        expected = shear_fields(u, g, (0, 0, 0, 0 if prescribed else f))
        if tet.body_id in local_shear:
            name = "C_44" if len(local_shear) == len(materials) else "HookeLinearElasticity/C_44"
            expected[name] = [(local_shear[tet.body_id],)] * 4
        check_tet(case, "static", out / "sim.vtu", tet, expected)


def mixed_bodies(case, solver, reverse_geometry=False, reverse_materials=False, prescribed=False):
    tets = [B, A] if reverse_geometry else [A, B]
    materials = [hooke(), isotropic(B)]
    loaded_bodies(case, solver, tets, materials[::-1] if reverse_materials else materials, prescribed)


def two_hooke_bodies(case, solver, reversed_order=False):
    # A is aligned, B is rotated 90 degrees about z. Its local Gzx differs
    # from the Gyz that carries the prescribed global xz shear.
    other = dict(hooke(ROTATION_90), id=BODY_B,
                 elasticity_tensor=[90000, 85000, 75000, 0.2, 0.17, 0.13, 20000, 26000, 15000])
    if reversed_order:
        loaded_bodies(case, solver, [B, A], [other, hooke()], prescribed=True)
    else:
        loaded_bodies(case, solver, [A, B], [hooke(), other], prescribed=True)


def traction(case, solver):
    # Each vertex receives T * area / 3 from each incident loaded face.
    # On {X,Y,Z}, T = 120/sqrt(3), area = sqrt(3)/2. On {O,Y,Z}, T = -60, area = 1/2.
    tractions = [(10 * BODY_A + 2, 120 / math.sqrt(3)), (10 * BODY_A + 3, -60)]
    config = scene([A], [isotropic(A)], tractions=tractions)
    out = solver.solve(case.folder / "run", [A], config)
    check_tet(case, "static", out / "sim.vtu", A, shear_fields(0.005, 12000, (-10, 20, 10, 10)))


def check_timeline(case, label, out, dt, first, last):
    entries = ET.parse(out / "sim.pvd").findall("./Collection/DataSet")
    times = [float(entry.attrib["timestep"]) for entry in entries]
    # The collection uses the original global time origin, including on a resume.
    case.compare(f"{label}.pvd_time", times, [i * dt for i in range(last + 1)], 1e-12)
    require([entry.attrib["file"] for entry in entries] == [f"step_{i}.vtm" for i in range(last + 1)], out)
    expected = {f"step_{i}.vtu" for i in range(first, last + 1)}
    require({path.name for path in out.glob("step_*.vtu")} == expected, (out, expected))
    for i in range(first, last + 1):
        vtm = out / f"step_{i}.vtm"
        volume = ET.parse(vtm).find("./vtkMultiBlockDataSet/Block[@name='Volume']/DataSet")
        require(volume is not None and volume.attrib["file"] == f"step_{i}.vtu", vtm)


def quasistatic_frames(case, solver, frame, moduli, nodal=False):
    # The x=0 face has area 1/2. A uniform 6*f Pa gives its three nodes f N each.
    f = 24
    load = {"nodal": [(A, f)]} if nodal else {"tractions": [(10 * BODY_A + 3, 6 * f)]}
    config = scene([A], [hooke(frame)], **load)
    config["time"] = {"dt": 1, "time_steps": len(moduli), "quasistatic": True}
    out = solver.solve(case.folder / "run", [A], config)
    check_timeline(case, "run", out, 1, 0, len(moduli))
    for index, g in enumerate(moduli, 1):
        check_tet(case, f"step{index}", out / f"step_{index}.vtu", A,
                  shear_fields(f / (VOLUME * g), g, (0, 0, 0, f) if nodal else (f, 0, f, f)))


def dynamic_reference(lumped, nodal=False, dt=DT, base_motion=0):
    # Integrals of N_i^2 and N_i N_j on a tetrahedron are V/10 and V/20.
    # Density 120 on volume 1/6 gives 2 kg at the diagonal and 1 kg elsewhere,
    # or 5 kg at the diagonal after row-sum lumping.
    mass_diag, mass_off = (5, 0) if lumped else (2, 1)
    g = 12000
    u = v = acceleration = 0
    base_u = base_v = base_acceleration = 0
    reference = []
    for i in range(STEPS + 1):
        time = i * dt
        f = 48 * time
        if i > 0:
            next_base_u = base_motion * time**2
            next_base_v = (next_base_u - base_u) / dt
            base_acceleration = (next_base_v - base_v) / dt
            base_u, base_v = next_base_u, next_base_v

            stiffness = VOLUME * g
            free_rhs = f + stiffness * base_u - 3 * mass_off * base_acceleration
            next_u = (mass_diag * (u + dt * v) + dt**2 * free_rhs) / (mass_diag + dt**2 * stiffness)
            next_v = (next_u - u) / dt
            acceleration = (next_v - v) / dt
            u, v = next_u, next_v
        base_inertia = -(mass_diag + 2 * mass_off) * base_acceleration - mass_off * acceleration
        apex_inertia = -mass_diag * acceleration - 3 * mass_off * base_acceleration
        reference.append(dict(shear_fields(u - base_u, g, (0, 0, 0, f) if nodal else (f, 0, f, f)),
                              solution=[(base_u, 0, 0)] * 3 + [(u, 0, 0)],
                              velocity=[(base_v, 0, 0)] * 3 + [(v, 0, 0)],
                              acceleration=[(base_acceleration, 0, 0)] * 3 + [(acceleration, 0, 0)],
                              inertia_forces=[(base_inertia, 0, 0)] * 3
                              + [(apex_inertia, 0, 0)]))
    return reference


def transient(case, solver, lumped, checkpoints, nodal=False, dt=DT, base_motion=0):
    load = {"nodal": [(A, "48*t")]} if nodal else {"tractions": [(10 * BODY_A + 3, "288*t")]}
    config = scene([A], [isotropic(A)], **load)
    config["time"] = {"dt": dt, "time_steps": STEPS, "integrator": "ImplicitEuler"}
    config["solver"]["advanced"] = {"lump_mass_matrix": lumped}
    if base_motion:
        config["boundary_conditions"]["dirichlet_boundary"][0]["value"][0] = f"{base_motion}*t*t"
    if checkpoints:
        config["output"]["checkpoint"] = {"path": "state_{:d}.hdf5"}
    return solver.solve(case.folder / "run", [A], config)


def check_transient(case, output, lumped, nodal=False, dt=DT, base_motion=0):
    check_timeline(case, "run", output, dt, 0, STEPS)
    for i, expected in enumerate(dynamic_reference(lumped, nodal, dt, base_motion)):
        check_tet(case, f"step{i}", output / f"step_{i}.vtu", A, expected)


def dynamics(case, solver, lumped, nodal=False, dt=DT, base_motion=0):
    full = transient(case, solver, lumped, False, nodal, dt, base_motion)
    check_transient(case, full, lumped, nodal, dt, base_motion)


def restart(case, solver, lumped, check_saved_inertia=False):
    full = transient(case, solver, lumped, True)
    check_transient(case, full, lumped)
    resumed = solver.solve(case.folder / "resume", checkpoint=full / f"state_{SPLIT}.hdf5")
    check_timeline(case, "resume", resumed, DT, SPLIT, STEPS)
    reference = dynamic_reference(lumped)
    # The strict reproducer also checks the repeated frame's inertia force.
    # That field currently uses the next-step predictor at restart.
    saved = reference[SPLIT].copy()
    if not check_saved_inertia:
        del saved["inertia_forces"]
    check_tet(case, f"resumed_step{SPLIT}", resumed / f"step_{SPLIT}.vtu", A, saved)
    for i in range(SPLIT + 1, STEPS + 1):
        check_tet(case, f"resumed_step{i}", resumed / f"step_{i}.vtu", A, reference[i])
    renewed = {path.name for path in resumed.glob("state_*.hdf5")}
    require(renewed == {f"state_{i}.hdf5" for i in range(SPLIT + 1, STEPS + 1)}, (resumed, renewed))


def default_cases():
    return {
        "isotropic_ab_ab": lambda c, s: loaded_bodies(c, s, [A, B], [isotropic(A), isotropic(B)]),
        "isotropic_ba_ba": lambda c, s: loaded_bodies(c, s, [B, A], [isotropic(B), isotropic(A)]),
        "isotropic_ab_ba": lambda c, s: loaded_bodies(c, s, [A, B], [isotropic(B), isotropic(A)]),
        "isotropic_ba_ab": lambda c, s: loaded_bodies(c, s, [B, A], [isotropic(A), isotropic(B)]),
        "mixed_hooke_first": mixed_bodies,
        "mixed_hooke_last": lambda c, s: mixed_bodies(c, s, reverse_geometry=True),
        "mixed_hooke_first_reversed_materials": lambda c, s: mixed_bodies(c, s, reverse_materials=True),
        "mixed_hooke_last_reversed_materials": lambda c, s: mixed_bodies(c, s, True, True),
        "mixed_hooke_prescribed": lambda c, s: mixed_bodies(c, s, prescribed=True),
        "mixed_hooke_prescribed_reversed": lambda c, s: mixed_bodies(c, s, True, True, prescribed=True),
        "two_hooke_bodies": two_hooke_bodies,
        "two_hooke_bodies_reversed": lambda c, s: two_hooke_bodies(c, s, reversed_order=True),
        "two_face_traction": traction,
        "consistent_mass": lambda c, s: dynamics(c, s, False),
        "lumped_mass": lambda c, s: dynamics(c, s, True),
        "quasistatic_nodal_load": lambda c, s: quasistatic_frames(c, s, None, [12000, 12000], nodal=True),
        "dynamic_nodal_load": lambda c, s: dynamics(c, s, False, nodal=True),
        "dynamic_nodal_load_half_dt": lambda c, s: dynamics(c, s, False, nodal=True, dt=DT / 2),
        "dynamic_nodal_load_moving_support": lambda c, s: dynamics(c, s, False, nodal=True, dt=2 * DT, base_motion=0.02),
        "dynamic_traction_moving_support": lambda c, s: dynamics(c, s, False, dt=2 * DT, base_motion=0.02),
        "consistent_restart": lambda c, s: restart(c, s, False),
        "lumped_restart": lambda c, s: restart(c, s, True),
        "static_frame_0": lambda c, s: quasistatic_frames(c, s, None, [12000, 12000]),
        "static_frame_90": lambda c, s: quasistatic_frames(c, s, ROTATION_90, [5000, 5000]),
    }


def run_suite(cases, description):
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--expect-commit")
    parser.add_argument("--output", type=Path, help="Create this directory and retain inputs, logs, and measurements")
    parser.add_argument("--case", choices=cases, action="append", help="Select one or more named cases")
    args = parser.parse_args()
    binary = args.binary.resolve(strict=True)

    def run(root):
        solver = Solver(binary, root, args.expect_commit)
        results = {}
        require(len(set(args.case or ())) == len(args.case or ()), "--case names must be distinct")
        for name in args.case or cases:
            case = Case(root, name)
            try:
                cases[name](case, solver)
                count = case.finish()
                results[name] = {"status": "passed", "numerical_checks": count}
                print(f"PASS {name} ({count} numeric checks)", flush=True)
            except (AssertionError, OSError, ValueError, subprocess.SubprocessError) as error:
                results[name] = {"status": "failed", "error": str(error)}
                print(f"FAIL {name}: {error}", file=sys.stderr, flush=True)
        write_json(root / "results.json", results)
        passed = sum(result["status"] == "passed" for result in results.values())
        print(f"{passed}/{len(results)} cases passed", flush=True)
        return int(passed != len(results))

    if args.output:
        root = args.output.resolve()
        root.mkdir(parents=True)
        return run(root)
    with tempfile.TemporaryDirectory(prefix="polyfem-simulation-oracles-") as temporary:
        return run(Path(temporary))


if __name__ == "__main__":
    sys.exit(run_suite(default_cases(), __doc__))
