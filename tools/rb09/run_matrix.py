"""RB-09 benchmark matrix: generate isolated scenes, run them, keep everything.

Usage:
  python3 tools/rb09/run_matrix.py --binary build/PolyFEM_bin --output /absolute/fresh/dir \
      --stage block|clamped|all [--only name,name] [--threads 1] [--dry-run]

Benchmark A (`block`): the public unit cube on the public slab (initial gap
g0 = .02) with symmetry planes u_x = 0 on x = 0 and u_y = 0 on y = 0, the top
face prescribed in z only, frictionless contact — the homogeneous uniaxial
compression whose exact Neo-Hookean solution is in tools/rb09/reference.py.
Benchmark C (`clamped`): the public clamped-top smokes (quasistatic and
transient) with their same-mesh hard-contact references (bottom face u_z = 0,
bilateral, floor gap removed from the loading; a lifted-plane run measures
the reference's conditioning).

Every run is an isolated directory with the scene, copied public inputs
(never modified in place), input hashes, the command, exit status, log and
output with the RB-04 physical diagnostics on. Nothing here runs Teseo or a
private scene. Existing run.json files are reused (resume), never rerun.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

HERE = Path(__file__).resolve().parent
SCENES = HERE.parents[1] / 'scenes' / 'semi-implicit'
G0 = 0.02        # public slab: z = -0.02 under the cube's bottom face z = 0
BASE_E, BASE_NU, BASE_RHO = 1e7, 0.45, 1000.0
BASE_DHAT, BASE_DT, BASE_TEND, BASE_RATE = 1e-3, 0.25, 1.0, 0.25

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('--binary', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--stage', default='block', choices=['block', 'clamped', 'all'])
parser.add_argument('--threads', type=int, default=1)
parser.add_argument('--only', default='')
parser.add_argument('--dry-run', action='store_true', help='write the scenes, run nothing')
parser.add_argument('--timeout', type=float, default=3600)
parser.add_argument('--fixed-kappa', type=float, default=None, help='adds block-fixed: Fixed barrier stiffness at this value (the classic adaptive endpoint stiffness)')
args = parser.parse_args()
binary = args.binary.resolve()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=True)


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def slab_obj(z, scale=1.0):
    s = scale
    return f"v {-1*s} {-1*s} {z}\nv {2*s} {-1*s} {z}\nv {2*s} {2*s} {z}\nv {-1*s} {2*s} {z}\nf 1 2 3\nf 1 3 4\n"


def press_expr(rate, t_end_load=None, scale=1.0):
    """Top displacement -rate*t (m/s * s), optionally reversed after t_end_load."""
    if t_end_load is None:
        return f'-{rate * scale}*t'
    # load to t_end_load then unload at the same rate: rate*(t - 2*max(0, t - t_end_load))
    return f'-{rate * scale}*(t-2*max(0,t-{t_end_load}))'


def block_scene(cfg):
    """Benchmark A scene. cfg keys: dhat, dt, tend, rate, E, nu, rho, n_refs,
    quasistatic, barrier, fixed_kappa, unit_scale, unload_at, stacked."""
    s = cfg.get('unit_scale', 1.0)          # 1 = metres, 1000 = millimetres
    n_refs = cfg.get('n_refs', 0)
    cube = {'mesh': 'cube.mesh', 'n_refs': n_refs,
            'surface_selection': [
                {'id': 2, 'axis': '+z', 'position': 0.99 * s},
                {'id': 3, 'axis': '-x', 'position': 0.01 * s},
                {'id': 4, 'axis': '-y', 'position': 0.01 * s}]}
    if s != 1.0:
        cube['transformation'] = {'scale': [s, s, s]}
    geometry = [cube, {'mesh': 'slab.obj', 'is_obstacle': True}]
    top_ids = [2]
    if cfg.get('stacked'):
        # second identical block above the first with the same initial gap g0
        upper = json.loads(json.dumps(cube))
        upper['transformation'] = {'translation': [0, 0, (1 + G0) * s]}
        if s != 1.0:
            upper['transformation']['scale'] = [s, s, s]
        upper['surface_selection'] = [
            {'id': 5, 'axis': '+z', 'position': (1 + G0 + 0.99) * s},
            {'id': 3, 'axis': '-x', 'position': 0.01 * s},
            {'id': 4, 'axis': '-y', 'position': 0.01 * s}]
        # the lower block's top face must stay free: reselect it as untagged (id 0 = no BC)
        cube['surface_selection'] = [
            {'id': 3, 'axis': '-x', 'position': 0.01 * s},
            {'id': 4, 'axis': '-y', 'position': 0.01 * s}]
        geometry.insert(1, upper)
        top_ids = [5]
    dirichlet = [{'id': 3, 'value': ['0', '0', '0'], 'dimension': [True, False, False]},
                 {'id': 4, 'value': ['0', '0', '0'], 'dimension': [False, True, False]}]
    for tid in top_ids:
        dirichlet.append({'id': tid, 'value': ['0', '0', press_expr(cfg['rate'], cfg.get('unload_at'), s)],
                          'dimension': [False, False, True]})
    solver_contact = {'barrier_stiffness': cfg.get('barrier', 'semi_implicit')}
    if cfg.get('barrier') == 'fixed':
        solver_contact['barrier_stiffness'] = cfg['fixed_kappa']
    if cfg.get('ccd_tolerance') is not None:
        solver_contact['CCD'] = {'tolerance': cfg['ccd_tolerance']}
    scene = {
        'geometry': geometry,
        # mm-N-s system for s = 1000: E in N/mm^2 (E/s^2), the mass unit is the
        # tonne so rho is in t/mm^3 (rho/s^4); forces stay in N, time in s.
        'materials': {'type': 'NeoHookean', 'E': cfg['E'] / s**2, 'nu': cfg['nu'], 'rho': cfg['rho'] / s**4},
        'time': {'tend': cfg['tend'], 'dt': cfg['dt'], 'quasistatic': cfg['quasistatic'], 'integrator': 'ImplicitEuler'},
        'contact': {'enabled': True, 'dhat': cfg['dhat'] * s, 'friction_coefficient': 0.0},
        'boundary_conditions': {'dirichlet_boundary': dirichlet},
        'solver': {'contact': solver_contact, 'linear': {'solver': 'Eigen::SimplicialLDLT'}},
        'output': {'log': {'level': 'debug'}, 'paraview': {'file_name': 'run.pvd'}, 'physical_diagnostics': True,
                   'data': {'nodes': 'nodes.txt'}},
    }
    if cfg.get('characteristic_force_density') is not None:
        # the tolerance scale (default 10000, an SI force density) converted with the units
        scene['solver']['advanced'] = {'characteristic_force_density': cfg['characteristic_force_density']}
    return scene, slab_obj(-G0 * s, s)


def clamped_scene(cfg):
    """Benchmark C scene: the public clamped-top smoke, or its same-mesh
    hard-contact reference (bilateral u_z on the bottom face, gap removed from
    the loading, contact off, optional lifted plane)."""
    name = 'transient-semi.json' if not cfg['quasistatic'] else 'quasistatic-semi.json'
    scene = json.loads((SCENES / name).read_text())
    scene['geometry'][0]['n_refs'] = cfg.get('n_refs', 0)
    scene['time']['dt'] = cfg['dt']
    scene['time']['tend'] = cfg['tend']
    scene['contact']['dhat'] = cfg['dhat']
    scene['output']['physical_diagnostics'] = True
    scene['output']['paraview']['file_name'] = 'run.pvd'
    scene['output']['data'] = {'nodes': 'nodes.txt'}
    scene['solver']['contact'] = {'barrier_stiffness': cfg.get('barrier', 'semi_implicit')}
    if cfg.get('barrier') == 'fixed':
        scene['solver']['contact']['barrier_stiffness'] = cfg['fixed_kappa']
    slab = (SCENES / 'slab.obj').read_text()
    if cfg.get('hard_reference'):
        lift = cfg.get('plane_lift', 0.0)
        scene['geometry'] = [scene['geometry'][0]]      # no obstacle
        scene['geometry'][0]['surface_selection'].append({'id': 6, 'axis': '-z', 'position': 0.01})
        scene['contact']['enabled'] = False
        scene['solver'].pop('contact', None)
        # top: -rate*t + g0 once the gap would have closed (t >= g0/rate); the
        # bottom plane sits at +lift (bilateral); loading before closure is a
        # rigid translation that the hard reference does not need to follow.
        rate = cfg['rate']
        scene['boundary_conditions']['dirichlet_boundary'] = [
            {'id': 2, 'value': ['0', '0', f'-{rate}*t+{G0}']},
            {'id': 6, 'value': ['0', '0', f'{lift}'], 'dimension': [False, False, True]}]
    return scene, slab


def configurations(stage):  # noqa: C901
    base = dict(dhat=BASE_DHAT, dt=BASE_DT, tend=BASE_TEND, rate=BASE_RATE, E=BASE_E, nu=BASE_NU, rho=BASE_RHO,
                n_refs=0, quasistatic=True, barrier='semi_implicit')
    runs = []

    def add(name, group, **over):
        cfg = dict(base)
        cfg.update(over)
        cfg.update(name=name, group=group, benchmark='block')
        runs.append(cfg)

    if stage in ('block', 'all'):
        add('block-base', 'base')
        for d in (4e-3, 2e-3, 5e-4):
            add(f'block-dhat{d:g}', 'dhat', dhat=d)
        for r in (1, 2):
            add(f'block-nrefs{r}', 'mesh', n_refs=r)
        for dt in (0.125, 0.0625):
            add(f'block-dt{dt:g}', 'increment', dt=dt)
        for dt in (0.25, 0.125, 0.0625, 0.03125):
            add(f'block-transient-dt{dt:g}', 'transient', dt=dt, quasistatic=False)
        for E in (1e6, 1e8):
            add(f'block-E{E:g}', 'material', E=E)
        add('block-units-mm', 'units', unit_scale=1000.0)
        # the same conversion with PolyFEM's dimensional solver constants converted too:
        # characteristic_force_density 1e4 N/m^3 -> 1e-5 N/mm^3, CCD tolerance 1e-6 m -> 1e-3 mm
        add('block-units-mm-converted', 'units', unit_scale=1000.0, characteristic_force_density=1e-5, ccd_tolerance=1e-3)
        # the same with the stopping tolerance made equal in newtons: the L2 gradient tolerance is
        # grad_norm * F0 * L^1.5 (NLProblem::grad_norm_rescaling), so F0 scales by 1000^-1.5
        add('block-units-mm-equal-tolerance', 'units', unit_scale=1000.0, characteristic_force_density=1e4 * 1000.0 ** -1.5, ccd_tolerance=1e-3)
        add('block-transient-rho10', 'density', rho=10 * BASE_RHO, dt=0.125, quasistatic=False)
        # approach speed: same final press .25, same 8 steps
        add('block-transient-rate0.125', 'speed', rate=0.125, tend=2.0, dt=0.25, quasistatic=False)
        add('block-transient-rate0.25', 'speed', rate=0.25, tend=1.0, dt=0.125, quasistatic=False)
        add('block-transient-rate0.5', 'speed', rate=0.5, tend=0.5, dt=0.0625, quasistatic=False)
        add('block-adaptive', 'controller', barrier='adaptive')
        if args.fixed_kappa is not None:
            # Fixed at the classic run's endpoint stiffness (declared comparison, not a default)
            add('block-fixed', 'controller', barrier='fixed', fixed_kappa=args.fixed_kappa)
        add('block-unload', 'unload', tend=2.0, unload_at=1.0)
        add('block-stacked', 'coupled', stacked=True, rate=0.5)
    if stage in ('clamped', 'all'):
        for r in (0, 1, 2):
            runs.append(dict(base, name=f'clamped-nrefs{r}', group='mesh', benchmark='clamped', n_refs=r))
            runs.append(dict(base, name=f'clamped-hard-nrefs{r}', group='mesh', benchmark='clamped', n_refs=r, hard_reference=True))
            if r < 2:
                runs.append(dict(base, name=f'clamped-hard-lift-nrefs{r}', group='mesh', benchmark='clamped', n_refs=r, hard_reference=True, plane_lift=5e-4))
        for dt in (0.125, 0.0625):
            runs.append(dict(base, name=f'clamped-dt{dt:g}', group='increment', benchmark='clamped', dt=dt))
        for dt in (0.25, 0.125, 0.0625):
            runs.append(dict(base, name=f'clamped-transient-dt{dt:g}', group='transient', benchmark='clamped', dt=dt, quasistatic=False))
        for d in (2e-3, 5e-4):
            runs.append(dict(base, name=f'clamped-dhat{d:g}', group='dhat', benchmark='clamped', dhat=d))
    return runs


only = set(filter(None, args.only.split(',')))
matrix = []
for cfg in configurations(args.stage):
    if only and cfg['name'] not in only:
        continue
    run_dir = out / cfg['name']
    if (run_dir / 'run.json').exists():
        matrix.append(json.loads((run_dir / 'run.json').read_text()))
        continue
    run_dir.mkdir(parents=True, exist_ok=True)
    scene, slab = (block_scene if cfg['benchmark'] == 'block' else clamped_scene)(cfg)
    (run_dir / 'scene.json').write_text(json.dumps(scene, indent=2) + '\n')
    shutil.copy(SCENES / 'cube.mesh', run_dir / 'cube.mesh')
    (run_dir / 'slab.obj').write_text(slab)
    record = dict(cfg)
    record['inputs'] = {f: sha(run_dir / f) for f in ['scene.json', 'cube.mesh', 'slab.obj']}
    record['binary_sha256'] = sha(binary)
    record['command'] = [str(binary), '--json', 'scene.json', '-o', 'output', '--log_level', 'debug', '--max_threads', str(args.threads)]
    if args.dry_run:
        record['status'] = 'not run (dry run)'
        matrix.append(record)
        continue
    start = time.time()
    with open(run_dir / 'run.log', 'w') as log:
        try:
            proc = subprocess.run(record['command'], cwd=run_dir, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            record['exit_status'] = proc.returncode
        except subprocess.TimeoutExpired:
            record['exit_status'] = 'timeout'
    record['wall_seconds'] = time.time() - start
    text = (run_dir / 'run.log').read_text()
    record['error_lines'] = sum('[error]' in line for line in text.splitlines())
    pvd = run_dir / 'output' / 'run.pvd'
    if pvd.exists():
        record['saved_times'] = [float(t) for t in re.findall(r'timestep="([^"]+)"', pvd.read_text())]
    diagnostics = run_dir / 'output' / 'physical-diagnostics.jsonl'
    if diagnostics.exists():
        lines = diagnostics.read_text().splitlines()
        record['diagnostic_records'] = len(lines)
        record['accepted_steps'] = sum(json.loads(line).get('outcome') == 'accepted' for line in lines)
    expected_steps = round(scene['time']['tend'] / scene['time']['dt'])
    record['expected_steps'] = expected_steps
    record['status'] = 'complete' if record.get('exit_status') == 0 and record.get('accepted_steps') == expected_steps else 'incomplete or failed'
    (run_dir / 'run.json').write_text(json.dumps(record, indent=2) + '\n')
    matrix.append(record)
    print(f"{cfg['name']:<36} exit={record.get('exit_status')} steps={record.get('accepted_steps')}/{expected_steps} errors={record['error_lines']} {record['wall_seconds']:.1f}s", flush=True)

(out / 'matrix.json').write_text(json.dumps(matrix, indent=2) + '\n')
print(json.dumps({'runs': len(matrix), 'complete': sum(r.get('status') == 'complete' for r in matrix)}))
