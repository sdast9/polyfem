# ###############################################################################
# Tests
# ###############################################################################

set(test_sources
  test_al_solver.cpp
  test_direction_filter.cpp
  test_contact_cache.cpp
  test_contact_stiffness_mapping.cpp
  test_hex_collision_surface.cpp
  test_semi_implicit_coefficients.cpp
  test_kappa_continuity.cpp
  test_friction_lag.cpp
  test_resource_containment.cpp
  test_input_validation.cpp
  test_material_envelope.cpp
  test_linear_elastic_time.cpp
  test_run_manifest.cpp
  test_assembler.cpp
  test_bases.cpp
  test_clipping.cpp
  test_cmesh.cpp
  test_collision_proxy.cpp
  test_geometry_utils.cpp
  test_hdf5.cpp
  test_interpolation.cpp
  test_jacobian.cpp
  test_matrix.cpp
  test_ncmesh.cpp
  test_normal.cpp
  test_navier_stokes_forms.cpp
  test_output.cpp
  test_periodic_boundary.cpp
  test_problem.cpp
  test_quadrature.cpp
  test_rbf.cpp
  test_restart.cpp
  test_tbb.cpp
  test_time_integrators.cpp
  test_utils.cpp
  test_varform_factory.cpp
  verify_run.cpp
  test_hgo_dispersion.cpp
)

if(POLYFEM_WITH_OPTIMIZATION)
  list(APPEND test_sources
    test_diff.cpp
    test_opt.cpp
    test_form_derivatives.cpp
    test_parametrizations.cpp
  )
endif()

add_executable(unit_tests ${test_sources})
# set_property(TARGET unit_tests PROPERTY COMPILE_WARNING_AS_ERROR ON)

################################################################################
# Required Libraries
################################################################################

target_link_libraries(unit_tests PUBLIC polyfem::polyfem)

include(polyfem_warnings)
target_link_libraries(unit_tests PUBLIC polyfem::warnings)

include(catch2)
target_link_libraries(unit_tests PUBLIC Catch2::Catch2WithMain)

include(finite_diff)
target_link_libraries(unit_tests PUBLIC finitediff::finitediff)

include(polyfem_data)
target_link_libraries(unit_tests PUBLIC polyfem::data)

include(polyfem_pref)
target_link_libraries(unit_tests PUBLIC polyfem::pref)

include(polyfem_polyspline)
target_link_libraries(unit_tests PUBLIC polyfem::polyspline)

include(polyfem_diff)
target_link_libraries(unit_tests PUBLIC polyfem::diff)

################################################################################
# Compiler options
################################################################################

target_compile_definitions(unit_tests PUBLIC -DPOLYFEM_TEST_DIR=\"${CMAKE_SOURCE_DIR}/tests\")

target_compile_definitions(unit_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

################################################################################
# Register tests
################################################################################

FetchContent_GetProperties(catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Register tests
set(PARSE_CATCH_TESTS_ADD_TO_CONFIGURE_DEPENDS ON)
catch_discover_tests(unit_tests ADD_TAGS_AS_LABELS EXTRA_ARGS --rng-seed 1)

# RB-12: the executable's contract end to end (build identity, a public
# smoke with its run manifest, the exit statuses of a resource failure and of
# a refused input), through the real PolyFEM_bin. Standard-library Python.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND AND TARGET PolyFEM_bin)
  add_test(NAME cli_contract
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/rb12/cli_check.py"
            --binary "$<TARGET_FILE:PolyFEM_bin>"
            --output "${CMAKE_CURRENT_BINARY_DIR}/cli_contract" --keep)
  set_tests_properties(cli_contract PROPERTIES LABELS "rb12;cli" TIMEOUT 1200)
else()
  message(STATUS "cli_contract test not registered (Python3 interpreter or PolyFEM_bin missing)")
endif()
