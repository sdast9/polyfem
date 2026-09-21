# IPC Toolkit (https://github.com/sdast9/ipc-toolkit)
# Fork of ipc-sim/ipc-toolkit with per-collision stiffness_scale,
# NormalCollisions::compute_avg_distance, NormalCollision::parents (the
# candidate contributions that built a collision, RB-21) for the semi-implicit
# barrier mode, and the opt-in BroadPhaseBudget enforced before allocation
# with an exception-safe Candidates::build (RB-05); since a28de2db the
# budget's counts are checked arithmetic (a count beyond size_t is "at
# least", never wrapped), a hash grid the key cannot index is refused by
# name (BroadPhaseUnrepresentable) and non-finite positions are refused
# before any conversion (RBR-02; 482b9eab is a tests-only follow-up for the
# Windows lane, library sources identical). The pin follows the fork's
# semi-implicit-stiffness branch.
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#482b9eab2f81bbc5ee59586f80ffb041bcd488dd")
