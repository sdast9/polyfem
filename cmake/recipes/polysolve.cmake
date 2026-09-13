# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, including the PF-06
# objective directional-derivative correction and the RB-19 line-search
# gradient-norm fallback with a finite characteristic-energy bound, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#ee5b296a690ce225fb34f6f375d27f9ea2a16027")
