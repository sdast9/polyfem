# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, including the PF-06
# objective directional-derivative correction and the RB-19 line-search
# gradient-norm fallback with a finite characteristic-energy bound, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1). The pin
# follows the fork's iteration-callback branch; 427e1458 also corrects dense
# BFGS to apply the current secant update before computing its direction, and
# 30f3a3a8 validates the secant pairs before they enter either approximation
# (BFGS audit stage 1, docs/bfgs-curvature-safeguard-20260922.md), 440cd55c
# discards that history when the problem reports a changed objective (stage 2,
# docs/bfgs-objective-generation-20260922.md), fcab19f0 adds the opt-in
# per-iteration nonlinear diagnostics stage 4 measured the contact scenes with
# (docs/bfgs-contact-diagnostics-20260922.md), fc62a67b adds the opt-in
# feasibility-respecting Wolfe line search and line_search_extend (stage 3,
# docs/bfgs-wolfe-line-search-20260922.md) and c874cd59 names the methods a
# forward solve cannot run and repairs ADAM's first step (stage 5,
# docs/bfgs-forward-methods-20260922.md); 448f1b8e applies the slope and
# step-length tolerances only under a Hessian-based direction
# (docs/slope-tolerance-repair-20260923.md). 6099b9cd merges upstream
# da4e7fe (hybrid linear solvers and large-index support), with a pinned
# Hypre thread-mpi-backend snapshot and native default/large-index validation;
# the nonlinear source tree is unchanged from 448f1b8e. 6a8c2cc9 makes Armijo
# (and RobustArmijo, Wolfe's fallback) refuse an uphill direction as a failed
# search instead of asserting (ADAM directions are unscreened;
# docs/armijo-uphill-direction-20260927.md in the PolySolve repository;
# 43ca2e66 adds its CI result, docs only).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#43ca2e661069ba3971e16ec4c26969ea1f1d41cc")
