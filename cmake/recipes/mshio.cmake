# MshIO (https://github.com/qnzhou/MshIO)
# License: Apache-2.0

if(TARGET mshio)
    return()
endif()

message(STATUS "Third-party: creating target 'mshio'")


include(CPM)
# The patch reads every value of the ASCII $MeshFormat, $PhysicalNames,
# $Entities, $Nodes and $Elements sections as a checked token: a value that
# does not convert completely (a numpy repr such as "np.float64(0.0)", "1.2.3",
# nan), a count that disagrees with the data or a missing end marker stops the
# load with the line, the token and the value that was expected. Upstream's
# reader left the stream failed on such a value and looped forever looking for
# the section's end marker. CPM keys its source cache on the patch's path, not
# its content: rename the patch when changing it.
CPMAddPackage(
    URI "gh:qnzhou/MshIO#29d0263b45bbbb2931ecbe892d0d7f0f3a493d0c"
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/patches/mshio-checked-ascii-read.patch"
)
