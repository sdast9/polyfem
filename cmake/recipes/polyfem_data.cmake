# data (https://github.com/sdast9/polyfem-data, branch fable-fixtures: the
# upstream https://github.com/polyfem/polyfem-data set plus this fork's fixture
# changes -- CI-03 (2026-09-21): three historical friction fixtures state the
# lag budget their references were generated under and get current-defaults
# twins; see docs/ci-03-validation.md)
# License: MIT

if(TARGET polyfem::data)
    return()
endif()

include(ExternalProject)

set(POLYFEM_DATA_DIR "${PROJECT_SOURCE_DIR}/data/" CACHE PATH "Where should polyfem download and look for test data?")
option(POLYFEM_USE_EXISTING_DATA_DIR "Use and existing data directory instead of downloading it" OFF)

if(POLYFEM_USE_EXISTING_DATA_DIR)
    ExternalProject_Add(
        polyfem_data_download
        PREFIX ${FETCHCONTENT_BASE_DIR}/polyfem-test-data
        SOURCE_DIR ${POLYFEM_DATA_DIR}

        # NOTE: No download step
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
    )
else()
    ExternalProject_Add(
        polyfem_data_download
        PREFIX ${FETCHCONTENT_BASE_DIR}/polyfem-test-data
        SOURCE_DIR ${POLYFEM_DATA_DIR}
        GIT_REPOSITORY https://github.com/sdast9/polyfem-data
        GIT_TAG e6ed5cf2d6514ef2595d28a23400521e2b3c717c
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        LOG_DOWNLOAD ON
    )
endif()

# Create a dummy target for convenience
add_library(polyfem_data INTERFACE)
add_library(polyfem::data ALIAS polyfem_data)

add_dependencies(polyfem_data polyfem_data_download)

target_compile_definitions(polyfem_data INTERFACE POLYFEM_DATA_DIR=\"${POLYFEM_DATA_DIR}\")
