"""Initial and repeated checkpoint-frame forces through nonlinear elasticity."""

import math
import sys

from test_simulation_oracles import (
    A, BODY_A, DT, FIELD_TOLERANCES, SPLIT, STEPS, Tet, check_tet, check_timeline,
    dynamic_reference, isotropic, read_fields, require, run_suite, scene,
)


INITIAL = (0.1, -0.04)
FORCE_FIELDS = ('body_forces', 'elastic_forces', 'inertia_forces', 'contact_forces')


def check_frames(case, label, output, reference, first):
    check_timeline(case, label, output, DT, first, STEPS, contact_surface=True)
    for i in range(first, STEPS + 1):
        step = f'{label}.step{i}'
        expected = dict(reference[i], contact_forces=[(0, 0, 0)] * 4)
        volume = output / f'step_{i}.vtu'
        check_tet(case, step, volume, A, expected)
        surface = read_fields(output / f'step_{i}_surf_contact.vtu', FORCE_FIELDS)
        for vertex, point in enumerate(A.points):
            indices = [j for j, p in enumerate(surface['points']) if math.dist(p, point) <= 1e-9]
            require(len(indices) == 1, (step, point, indices))
            for field in FORCE_FIELDS:
                case.compare(f'{step}.surface.{vertex}.{field}', surface[field][indices[0]],
                             expected[field][vertex], FIELD_TOLERANCES[field])
            if i > 0 and vertex == 3:
                # Only the apex x DOF is free. It has no reaction, so these
                # four forces balance at every solved (including restored) step.
                balance = sum(surface[field][indices[0]][0] for field in FORCE_FIELDS)
                case.compare(f'{step}.apex_x_balance', (balance,), (0,), 5e-5)


def nonlinear_transient(case, solver, lumped, asynchronous, nodal):
    load = {'nodal': [(A, '48*t')]} if nodal else {'tractions': [(10 * BODY_A + 3, '288*t')]}
    config = scene([A], [isotropic(A)], **load)
    # Contact selects the nonlinear path, but the isolated tet never has an
    # active stencil. This leaves one analytically solvable shear mode.
    config['contact'] = {'enabled': True, 'dhat': 1e-3}
    config['time'] = {'dt': DT, 'time_steps': STEPS, 'integrator': 'ImplicitEuler'}
    config['solver']['advanced'] = {'lump_mass_matrix': lumped}
    config['output']['paraview']['async'] = asynchronous
    config['output']['paraview']['options']['contact_forces'] = True
    config['output']['checkpoint'] = {'path': 'state_{:d}.hdf5'}
    config['initial_conditions'] = {
        name: [{'id': BODY_A, 'value': [f'{value}*z', 0, 0]}]
        for name, value in zip(('velocity', 'acceleration'), INITIAL)
    }
    # Consistent and lumped rows have 2/5 kg at the apex. For a ramp 48 t N,
    # the one-DOF implicit Euler update is M a + 2000 u = 48 t.
    # Initial acceleration is supplied explicitly, so step 0 need not balance.
    reference = dynamic_reference(lumped, nodal, initial_kinematics=INITIAL)
    full = solver.solve(case.folder / 'run', [A], config)
    resumed = solver.solve(case.folder / 'resume', checkpoint=full / f'state_{SPLIT}.hdf5')
    check_frames(case, 'continuous', full, reference, 0)
    check_frames(case, 'resumed', resumed, reference, SPLIT)
    require({p.name for p in resumed.glob('state_*.hdf5')}
            == {f'state_{i}.hdf5' for i in range(SPLIT + 1, STEPS + 1)}, resumed)


GAP = 3 / 32
SUPPORT = 1 / 8
CONTACT_STIFFNESS = 10000
PLANE = ((-2, -2, 0), (6, -2, 0), (-2, 6, 0))
OBSTACLE_ID = 91


class AxialTet(Tet):
    @property
    def points(self):
        return ((0, 0, 1 + GAP), (0, 1, 1 + GAP), (1, 0, 1 + GAP), (0, 0, GAP))

    def write(self, folder):
        geometry = super().write(folder)
        selection = folder / 'apex.txt'
        selection.write_text(f'{10 * self.body_id + 4} 3\n')
        geometry['point_selection'] = [{'file': selection.name}]
        return geometry

    def clamps(self):
        tag = 10 * self.body_id
        return [{'id': tag + 1, 'value': [0, 0, 0]},
                {'id': tag + 2, 'value': [0, 0, 0], 'dimension': [True, True, False]}]


class ContactPlane:
    def write(self, folder):
        mesh = folder / 'plane.obj'
        mesh.write_text(''.join(f'v {x} {y} {z}\n' for x, y, z in PLANE) + 'f 1 2 3\n')
        return {'mesh': mesh.name, 'is_obstacle': True, 'surface_selection': OBSTACLE_ID}


def axial_reference(times):
    # The apex is strictly inside the plane triangle's projection. Every
    # other solid vertex and obstacle edge is outside the support. This
    # leaves one vertex-face interaction with s=gap^2 and h=dhat^2.
    out = []
    u_prev, v_prev = 0, -0.01
    for i, t in enumerate(times):
        u = -0.016 * t - 0.04 * t**2
        v = (u - u_prev) / DT if i else v_prev
        a = (v - v_prev) / DT if i else -0.04
        gap = GAP + u
        require(0 < gap < SUPPORT, (t, gap))
        s, h = gap**2, SUPPORT**2
        # Differentiate the clamped log B(s)=-(s-h)^2 log(s/h).
        b_prime = -2 * (s - h) * math.log(s / h) - (s - h)**2 / s
        contact = -2 * gap * CONTACT_STIFFNESS * b_prime
        # E=30000 Pa, nu=1/4, V=1/6 m^3 give K=(lambda+2*mu)V=6000 N/m.
        # For rho=120 kg/m^3, M_aa=2 kg and each M_ia=1 kg.
        # Implicit Euler balance on the free z DOF is M a + K u = F + C.
        elastic = -6000 * u
        applied = 2 * a - elastic - contact
        out.append({'solution': u, 'velocity': v, 'acceleration': a,
                    'body_forces': applied, 'elastic_forces': elastic,
                    'inertia_forces': -2 * a, 'contact_forces': contact})
        u_prev, v_prev = u, v
    return out


def check_active_frames(case, label, output, tet, reference, first):
    check_timeline(case, label, output, DT, first, STEPS, contact_surface=True)
    for i in range(first, STEPS + 1):
        step = f'{label}.step{i}'
        expected = reference[i]
        for surface in (False, True):
            kind = 'contact' if surface else 'volume'
            names = FORCE_FIELDS if surface else tuple(expected)
            suffix = '_surf_contact' if surface else ''
            arrays = read_fields(output / f'step_{i}{suffix}.vtu', names)
            rows = [j for j, p in enumerate(arrays['points']) if math.dist(p, tet.points[3]) <= 1e-9]
            require(len(rows) == 1, (step, kind, rows))
            apex = rows[0]
            for field in names:
                case.compare(f'{step}.{kind}.{field}', arrays[field][apex], (0, 0, expected[field]),
                             FIELD_TOLERANCES[field])
            balance = sum(arrays[field][apex][2] for field in FORCE_FIELDS)
            case.compare(f'{step}.{kind}.balance', (balance,), (0,), 5e-5)
            if surface:
                # The plane carries equal-and-opposite force with the exact
                # barycentric weights at (0, 0, 0).
                for j, (point, weight) in enumerate(zip(PLANE, (0.5, 0.25, 0.25))):
                    rows = [r for r, p in enumerate(arrays['points']) if math.dist(p, point) <= 1e-9]
                    require(len(rows) == 1, (step, point, rows))
                    case.compare(f'{step}.obstacle{j}.contact_forces', arrays['contact_forces'][rows[0]],
                                 (0, 0, -weight * expected['contact_forces']), FIELD_TOLERANCES['contact_forces'])


def active_contact_restart(case, solver):
    tet = AxialTet(BODY_A)
    times = [i * DT for i in range(STEPS + 1)]
    reference = axial_reference(times)
    require(reference[SPLIT]['contact_forces'] > 1, reference[SPLIT])
    config = scene([tet], [isotropic(A)], nodal=[(tet, 0)])
    config['contact'] = {'enabled': True, 'dhat': SUPPORT, 'friction_coefficient': 0}
    config['solver']['contact'] = {'barrier_stiffness': CONTACT_STIFFNESS}
    # Use an unscaled gradient norm to resolve the existing 2e-8 N contact
    # oracle even when the problem's force scale is much larger than 1 N.
    config['solver']['nonlinear'] = {'norm_type': 'Euclidean', 'grad_norm_tol': 1e-10,
                                     'first_grad_norm_tol': 1e-12}
    config['time'] = {'dt': DT, 'time_steps': STEPS, 'integrator': 'ImplicitEuler'}
    load = config['boundary_conditions']['nodal_neumann_boundary'][0]
    load['value'] = [0, 0, 1]
    load['interpolation'] = [{'type': 'piecewise_linear', 'points': times, 'extend': 'constant',
                              'values': [state['body_forces'] for state in reference]}]
    config['boundary_conditions']['obstacle_displacements'] = [{'id': OBSTACLE_ID, 'value': [0, 0, 0]}]
    config['output']['paraview']['async'] = True
    config['output']['paraview']['options']['contact_forces'] = True
    config['output']['checkpoint'] = {'path': 'state_{:d}.hdf5'}
    config['initial_conditions'] = {
        name: [{'id': BODY_A, 'value': [0, 0, f'{reference[0][name]}*({1 + GAP}-z)']}]
        for name in ('velocity', 'acceleration')
    }
    full = solver.solve(case.folder / 'run', [tet, ContactPlane()], config)
    resumed = solver.solve(case.folder / 'resume', checkpoint=full / f'state_{SPLIT}.hdf5')
    check_active_frames(case, 'continuous', full, tet, reference, 0)
    check_active_frames(case, 'resumed', resumed, tet, reference, SPLIT)
    for name, row in tuple(case.metrics.items()):
        if name.startswith('resumed.'):
            prior = case.metrics[name.replace('resumed.', 'continuous.', 1)]
            case.compare(f'continuation.{name}', row['actual'], prior['actual'], row['atol'])


if __name__ == '__main__':
    sys.exit(run_suite({
        'consistent_nodal_async': lambda c, s: nonlinear_transient(c, s, False, True, True),
        'lumped_traction_sync': lambda c, s: nonlinear_transient(c, s, True, False, False),
        'active_contact_restart': active_contact_restart,
    }, __doc__))
