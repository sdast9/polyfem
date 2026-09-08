# IPC Toolkit (https://github.com/sdast9/ipc-toolkit)
# Fork of ipc-sim/ipc-toolkit with per-collision stiffness_scale and
# NormalCollisions::compute_avg_distance for the semi-implicit barrier mode.
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#af317a65d69d0ac7c5efa4bf103bf75e280c323b")
