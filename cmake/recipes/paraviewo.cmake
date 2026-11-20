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
    URI "gh:polyfem/paraviewo#42f6d4e45de1e536851cb96268a5e78f22cbb42a"
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/patches/paraviewo-hdf5-row-chunks.patch"
)
