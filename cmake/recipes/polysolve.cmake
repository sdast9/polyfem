# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, including the PF-06
# objective directional-derivative correction and the RB-19 line-search
# gradient-norm fallback with a finite characteristic-energy bound, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1). The pin
# follows the fork's iteration-callback branch; bce32a39 only enables its CI.
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#bce32a39a2c8f0a64cb8ffa85b89f0ee773df0ec")
