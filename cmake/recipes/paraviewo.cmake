# Polyfem Solvers (https://github.com/polyfem/paraviewo)
# License: MIT

if(TARGET paraviewo::paraviewo)
    return()
endif()

message(STATUS "Third-party: creating target 'paraviewo::paraviewo'")

include(CPM)
# The patch writes N x 3 HDF5 fields in dense row chunks instead of h5pp's
# square 256 x 256 guess (85x padding per chunk: minutes per written frame
# and ~80 s to read one 2.4M-point frame). PATCHES is part of CPM's source
# cache key, so the unpatched cached checkout is left untouched.
CPMAddPackage(
    URI "gh:polyfem/paraviewo#1c01aae7ad37020f751eb598c62ae4ae74a3ada7"
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/patches/paraviewo-hdf5-row-chunks.patch"
)
