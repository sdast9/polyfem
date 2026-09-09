"""Independent exact P1 mass work for the fixed-dt implicit-Euler cube fixture."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/pf08'))
from measure_fem import arrays


def mass_dot(data, a, b):
    """Exact integral rho*a_h dot b_h for P1 fields on the saved tetrahedra."""
    total = 0.
    for start, end, kind in zip(np.r_[0, data['offsets'][:-1]], data['offsets'], data['types']):
        if kind != 10:
            continue
        tet = data['connectivity'][start:end]
        X = data['points'][tet]
        volume = abs(np.linalg.det((X[1:]-X[0]).T))/6
        aa, bb = a[tet], b[tet]
        total += 1000*volume/20*(np.sum(aa*bb)+aa.sum(axis=0)@bb.sum(axis=0))
    return float(total)


def verify_transient(directory, dt, frames):
    previous = arrays(directory/'output/step_0.vtu')
    previous_velocity = np.zeros_like(previous['displacement'])
    work = dissipation = 0.
    for frame in frames:
        data = arrays(directory/'output'/f"step_{frame['step']}.vtu")
        velocity = (data['displacement']-previous['displacement'])/dt
        change = velocity-previous_velocity
        kinetic = .5*mass_dot(data, velocity, velocity)
        dissipation += .5*mass_dot(data, change, change)
        work += mass_dot(data, change, velocity)
        # The fixture starts at rest; initial K is zero.
        assert abs(work-kinetic-dissipation) < 1e-9*(1+abs(work))
        assert abs(kinetic-frame['kinetic_energy']) < 1e-9*(1+kinetic)
        measured_work = frame['component_costs_right']['inertia']
        assert abs(work-measured_work) < 1e-9*(1+abs(work)), (work, measured_work)
        frame['independent_kinetic_energy'] = kinetic
        frame['independent_accumulated_inertial_work'] = work
        frame['independent_accumulated_IE_dissipation'] = dissipation
        frame['inertial_work_reference_error'] = work-measured_work
        previous, previous_velocity = data, velocity
