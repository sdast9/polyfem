# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#012658e5d95b086f20c8e0e5f247473a2a229825")
