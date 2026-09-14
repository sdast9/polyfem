# IPC Toolkit (https://github.com/sdast9/ipc-toolkit)
# Fork of ipc-sim/ipc-toolkit with per-collision stiffness_scale,
# NormalCollisions::compute_avg_distance, NormalCollision::parents (the
# candidate contributions that built a collision, RB-21) for the semi-implicit
# barrier mode, and the opt-in BroadPhaseBudget enforced before allocation
# with an exception-safe Candidates::build (RB-05). The pin follows the
# fork's semi-implicit-stiffness branch; c24d803e only enables its CI.
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#c24d803e6b175d71a610f3ec85bd33c042e2be0d")
