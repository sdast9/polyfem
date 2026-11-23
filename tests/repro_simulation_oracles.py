"""Strict reproductions of material, load, and restart-output defects."""

import sys

from test_simulation_oracles import restart, run_suite


if __name__ == "__main__":
    sys.exit(run_suite({
        "consistent_restart_initial_inertia": lambda c, s: restart(c, s, False, check_saved_inertia=True),
        "lumped_restart_initial_inertia": lambda c, s: restart(c, s, True, check_saved_inertia=True),
    }, __doc__))
