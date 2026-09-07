# PolySolve (https://github.com/sdast9/polysolve)
# Fork of polyfem/polysolve with the iteration-callback / direction-filter
# additions required by the semi-implicit barrier solver, including the PF-06
# objective directional-derivative correction, merged with
# polyfem/polysolve@a7727e33 (residual problems, Eigen 5.0.1).
# License: MIT

if(TARGET polysolve)
    return()
endif()

message(STATUS "Third-party: creating target 'polysolve'")

include(CPM)
CPMAddPackage("gh:sdast9/polysolve#4d372fa8a73f42bc224e31d464f1a308e1159ba8")
