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
# Windows lane, library sources identical); since 75600955 it pins
# Tight-Inclusion 1.1.0, whose default bucket depth-first root finder bounds
# the memory of iteration-capped CCD queries (RB-24). The pin follows the
# fork's semi-implicit-stiffness branch. cf99893b merges upstream 869e489e
# (block assembly, templated/SIMD geometry and CPU/CUDA LBVH), preserving
# the fork contracts and refusing unsupported CUDA LBVH resource budgets.
# f8dafef3 (CI-06 canonical smooth-contact order): SmoothCollisionsBuilder
# merge gathers each thread's deduplicated maps and appends them, and
# face-vertex/edge-edge lists, in ascending primitive-id order instead of
# Abseil-seeded robin_map iteration order, so GCP/SmoothContact energy,
# gradient and Hessian sums are reproducible across processes.
# 8a353ad9 (cloud/parallel-edge-fix, solve_spd_2x2 refinement): the branchless
# Cramer solve of the 2x2 edge-edge/point-triangle Gram systems takes one
# step of iterative refinement where its relative residual exceeds 1e-12
# (nearly parallel edges, cond(A) ~ 1e7), which restores the residual the
# pivoted LDLT solve met and that the Debug assertion enforces; systems at
# rounding level keep their bits.
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage("gh:sdast9/ipc-toolkit#8a353ad9d93422384cf8a85ec2a2a58ae7a0bc54")
