# Clipper (https://sourceforge.net/projects/polyclipping)
# License: (BSL1.0)

if(TARGET clipper::clipper)
    return()
endif()

message(STATUS "Third-party: creating target 'clipper::clipper'")

# The second URL is a byte-identical copy of the SourceForge archive on the
# data fork's third-party-mirror branch, pinned by commit; it is tried only if
# SourceForge fails (Build 36811109219 of 2026-10-01 failed on every Linux and
# macOS lane with HTTP 522). The MD5 check applies to either download.
include(CPM)
CPMAddPackage(
    NAME clipper_clipper
    URL https://sourceforge.net/projects/polyclipping/files/clipper_ver6.4.2.zip
        https://raw.githubusercontent.com/sdast9/polyfem-data/4ddf4b35da5b2de8c6645055e2f7e794a7843874/clipper_ver6.4.2.zip
    URL_MD5 100b4ec56c5308bac2d10f3966e35e11
    DOWNLOAD_ONLY TRUE
)

add_library(clipper_clipper ${clipper_clipper_SOURCE_DIR}/cpp/clipper.cpp)
target_include_directories(clipper_clipper SYSTEM PUBLIC ${clipper_clipper_SOURCE_DIR}/cpp)
add_library(clipper::clipper ALIAS clipper_clipper)