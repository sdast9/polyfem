"""Initial and repeated checkpoint-frame forces through nonlinear elasticity."""

import math
import sys

from test_simulation_oracles import (
    A, BODY_A, DT, FIELD_TOLERANCES, SPLIT, STEPS, check_tet, check_timeline,
    dynamic_reference, isotropic, read_fields, require, run_suite, scene,
)


INITIAL = (0.1, -0.04)


def check_frames(case, label, output, reference, first):
    check_timeline(case, label, output, DT, first, STEPS, contact_surface=True)
    for i in range(first, STEPS + 1):
        step = f'{label}.step{i}'
        expected = dict(reference[i], contact_forces=[(0, 0, 0)] * 4)
        volume = output / f'step_{i}.vtu'
        check_tet(case, step, volume, A, expected)
        names = ('body_forces', 'elastic_forces', 'inertia_forces', 'contact_forces')
        surface = read_fields(output / f'step_{i}_surf_contact.vtu', names)
        for vertex, point in enumerate(A.points):
            indices = [j for j, p in enumerate(surface['points']) if math.dist(p, point) <= 1e-9]
            require(len(indices) == 1, (step, point, indices))
            for field in names:
                case.compare(f'{step}.surface.{vertex}.{field}', surface[field][indices[0]],
                             expected[field][vertex], FIELD_TOLERANCES[field])
            if i > 0 and vertex == 3:
                # Only the apex x DOF is free. It has no reaction, so these
                # four forces balance at every solved (including restored) step.
                balance = sum(surface[field][indices[0]][0] for field in names)
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


if __name__ == '__main__':
    sys.exit(run_suite({
        'consistent_nodal_async': lambda c, s: nonlinear_transient(c, s, False, True, True),
        'lumped_traction_sync': lambda c, s: nonlinear_transient(c, s, True, False, False),
    }, __doc__))
