"""Strict reproductions of material, load, and restart-output defects."""

import sys

from test_simulation_oracles import (
    A, B, dynamics, hooke, isotropic, loaded_bodies, quasistatic_frames, restart, run_suite,
)


def mixed_materials(case, solver, reversed_geometry):
    tets = [B, A] if reversed_geometry else [A, B]
    loaded_bodies(case, solver, tets, [hooke(), isotropic(B)])


def rotating_frame(case, solver):
    # Z rotation by pi*t/2. Only the frame changes across the two increments.
    c, s = "cos(pi*t/2)", "sin(pi*t/2)"
    quasistatic_frames(case, solver, [c, "-" + s, 0, s, c, 0, 0, 0, 1], [5000, 12000])


if __name__ == "__main__":
    sys.exit(run_suite({
        "mixed_hooke_first": lambda c, s: mixed_materials(c, s, False),
        "mixed_hooke_last": lambda c, s: mixed_materials(c, s, True),
        "rotating_frame": rotating_frame,
        "quasistatic_nodal_load": lambda c, s: quasistatic_frames(c, s, None, [12000, 12000], nodal=True),
        "dynamic_nodal_load": lambda c, s: dynamics(c, s, False, nodal=True),
        "consistent_restart_initial_inertia": lambda c, s: restart(c, s, False, check_saved_inertia=True),
        "lumped_restart_initial_inertia": lambda c, s: restart(c, s, True, check_saved_inertia=True),
    }, __doc__))
