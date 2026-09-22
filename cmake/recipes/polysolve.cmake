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
# docs/bfgs-forward-methods-20260922.md).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#c874cd59ca68afa6e7fc6b044581bd37bc2c536a")
