# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, including the PF-06
# objective directional-derivative correction and the RB-19 line-search
# gradient-norm fallback with a finite characteristic-energy bound, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1). The pin
# follows the fork's iteration-callback branch; 427e1458 also corrects dense
# BFGS to apply the current secant update before computing its direction, and
# 30f3a3a8 validates the secant pairs before they enter either approximation
# (BFGS audit stage 1, docs/bfgs-curvature-safeguard-20260922.md).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#30f3a3a8b0aa291da1c7f738e86bc134a8a269a1")
