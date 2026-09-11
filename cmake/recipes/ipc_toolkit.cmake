# IPC Toolkit (https://github.com/sdast9/ipc-toolkit)
# Fork of ipc-sim/ipc-toolkit with per-collision stiffness_scale,
# NormalCollisions::compute_avg_distance and NormalCollision::parents (the
# candidate contributions that built a collision, RB-21) for the semi-implicit
# barrier mode.
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#e3c8d3fe653d5de46c6aa34c400f79119af719f7")
