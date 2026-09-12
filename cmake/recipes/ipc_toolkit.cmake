# IPC Toolkit (https://github.com/sdast9/ipc-toolkit)
# Fork of ipc-sim/ipc-toolkit with per-collision stiffness_scale,
# NormalCollisions::compute_avg_distance, NormalCollision::parents (the
# candidate contributions that built a collision, RB-21) for the semi-implicit
# barrier mode, and the opt-in BroadPhaseBudget enforced before allocation
# with an exception-safe Candidates::build (RB-05).
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#bb795446812a3d3c7b5358c4cdbf26c6bbe48a16")
