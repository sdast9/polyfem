"""Strict reproductions of material, load, and restart-output defects."""

import sys

from test_simulation_oracles import quasistatic_frames, restart, run_suite


def rotating_frame(case, solver):
    # Z rotation by pi*t/2. Only the frame changes across the two increments.
    c, s = "cos(pi*t/2)", "sin(pi*t/2)"
    quasistatic_frames(case, solver, [c, "-" + s, 0, s, c, 0, 0, 0, 1], [5000, 12000])


if __name__ == "__main__":
    sys.exit(run_suite({
        "rotating_frame": rotating_frame,
        "consistent_restart_initial_inertia": lambda c, s: restart(c, s, False, check_saved_inertia=True),
        "lumped_restart_initial_inertia": lambda c, s: restart(c, s, True, check_saved_inertia=True),
    }, __doc__))
