################################################################################
# CI-08: install rules and CPack TGZ/ZIP packaging for the PolyFEM_bin CLI.
#
# app/CMakeLists.txt already installs/packages polyfem_app (the GUI) when
# POLYFEM_WITH_APP is on; nothing previously installed PolyFEM_bin itself
# (the executable the Houdini workflow and this fork's CLI users need).
# This module adds that, following the same relocatable-RPATH /
# RUNTIME_DEPENDENCY_SET pattern app/CMakeLists.txt uses, scoped to the CLI so
# it does not change the developer build (`cmake --build . --target
# PolyFEM_bin` is unaffected; packaging only happens on `cmake --install` /
# `cpack`, which nothing runs implicitly).
#
# Skipped when POLYFEM_WITH_APP is also on: that build already calls
# include(CPack) for its own GUI component, and CPack is meant to be
# configured once per project. Packaging the CLI alongside the GUI in one
# configure is not part of this item; build them separately.
################################################################################

if(NOT POLYFEM_TOPLEVEL_PROJECT OR POLYFEM_WITH_APP)
    return()
endif()

set(POLYFEM_CLI_RUNTIME_COMPONENT cli)

# PolyFEM_bin currently links no third-party shared library outside the
# default system search path (TBB and MKL are linked statically in the
# tested configuration), so CMake's automatic RPATH computation embeds no
# RPATH/RUNPATH section at all -- and `file(RPATH_CHANGE)` at install time
# refuses to rewrite a section that does not exist. Force a (possibly
# no-op) build-time RPATH so install-time relocation always has something
# valid to rewrite, in case a future configuration (a different platform,
# POLYFEM_THREADING, or a shared TBB) does bundle a runtime library.
if(APPLE)
    set_target_properties(${PROJECT_NAME}_bin PROPERTIES
        BUILD_RPATH "@loader_path"
        INSTALL_RPATH "@executable_path/../lib")
elseif(UNIX)
    set_target_properties(${PROJECT_NAME}_bin PROPERTIES
        BUILD_RPATH "$ORIGIN"
        INSTALL_RPATH "$ORIGIN/../lib")
endif()

if(WIN32)
    install(TARGETS ${PROJECT_NAME}_bin
        RUNTIME_DEPENDENCY_SET polyfem_bin_runtime_deps
        RUNTIME DESTINATION .
        COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})

    install(RUNTIME_DEPENDENCY_SET polyfem_bin_runtime_deps
        DIRECTORIES "$<TARGET_FILE_DIR:${PROJECT_NAME}_bin>"
        PRE_EXCLUDE_REGEXES
            "api-ms-"
            "ext-ms-"
            "[Hh][Vv][Ss][Ii][Ff][Ii][Ll][Ee][Tt][Rr][Uu][Ss][Tt]\\.dll"
        POST_EXCLUDE_REGEXES "[Ww][Ii][Nn][Dd][Oo][Ww][Ss]/[Ss]ystem32/.*"
        RUNTIME DESTINATION .
        COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})

    set(CPACK_GENERATOR "ZIP")
    set(POLYFEM_CLI_PACKAGE_PLATFORM "Windows")
else()
    install(TARGETS ${PROJECT_NAME}_bin
        RUNTIME_DEPENDENCY_SET polyfem_bin_runtime_deps
        RUNTIME DESTINATION bin
        COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})

    # Bundle everything the executable links dynamically except the base
    # system libraries (glibc, libstdc++, the dynamic loader): those are
    # assumed present on the target distribution (docs/ci-08-validation.md
    # records the measured glibc baseline). Third-party runtime libraries
    # (TBB) are copied in; see that document for what remains a system
    # requirement (the embedded Python runtime).
    install(RUNTIME_DEPENDENCY_SET polyfem_bin_runtime_deps
        DIRECTORIES "$<TARGET_FILE_DIR:${PROJECT_NAME}_bin>"
        POST_EXCLUDE_REGEXES "^/lib/.*" "^/usr/lib/.*" "^/usr/lib64/.*"
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})

    set(CPACK_GENERATOR "TGZ")
    if(APPLE)
        set(POLYFEM_CLI_PACKAGE_PLATFORM "macOS")
    else()
        set(POLYFEM_CLI_PACKAGE_PLATFORM "Linux")
    endif()
endif()

install(FILES "${PROJECT_SOURCE_DIR}/README.md"
    DESTINATION .
    RENAME "README.txt"
    COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})

install(FILES "${PROJECT_SOURCE_DIR}/LICENSE"
    DESTINATION .
    COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT}
    OPTIONAL)

if(EXISTS "${PROJECT_SOURCE_DIR}/docs/ci-08-validation.md")
    install(FILES "${PROJECT_SOURCE_DIR}/docs/ci-08-validation.md"
        DESTINATION .
        RENAME "PACKAGE-NOTES.md"
        COMPONENT ${POLYFEM_CLI_RUNTIME_COMPONENT})
endif()

set(CPACK_PACKAGE_VENDOR "PolyFEM")
set(CPACK_PACKAGE_NAME "polyfem_cli")
set(CPACK_PACKAGE_FILE_NAME "PolyFEM-CLI-${POLYFEM_CLI_PACKAGE_PLATFORM}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "PolyFEM_bin - the PolyFEM command-line solver")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}/packages")
set(CPACK_PACKAGING_INSTALL_PREFIX "/")
set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_COMPONENTS_ALL ${POLYFEM_CLI_RUNTIME_COMPONENT})
set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};${PROJECT_NAME};${POLYFEM_CLI_RUNTIME_COMPONENT};/")
set(CPACK_VERBATIM_VARIABLES ON)

include(CPack)
