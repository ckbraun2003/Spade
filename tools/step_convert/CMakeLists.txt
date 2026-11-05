# tools/step_convert: the STEP converter's tests (CORE-6 D, Task S2; TD-16). The converter is a Python script
# (step_convert.py) that needs OpenCASCADE through the pinned cadquery-ocp wheels (requirements.txt), run as its
# own process; nothing here builds, links or installs it.
#
# THE LABEL IS `step`, NEVER `spade` (L24). Both gates select `-L spade`, so these tests are outside them with no
# edit to scripts/. They run on the development box, on any branch touching tools/step_convert/ or tools/mesh_pack/:
#     ctest -L step
# OCP is found by `import OCP` in the python that runs the test, or by the environment variable SPADE_STEP_PYTHON
# naming a venv's python: no CMake option. Without OCP a case that needs it prints why and exits 77, which
# SKIP_RETURN_CODE turns into a ctest skip -- never a pass, and a skip does not count toward a green gate.
#
# They are plain add_test calls, one per case, not gtest-discovered, so tests/AppendSpadeLabels.cmake (which labels
# spade_tests' discovered tests `spade`) never touches them. Registered only when CMake finds a Python.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(NOT Python3_Interpreter_FOUND)
    message(STATUS "step_convert: no Python found; the `step` tests are not registered")
    return()
endif()

set(_step_cases
    test_a_deflection_is_required_and_needs_a_unit
    test_the_packer_must_be_found
    test_a_cube_is_closed_exact_and_carries_its_provenance
    test_a_cylinder_with_a_hole_is_genus_one_and_within_the_deflection_bound
    test_an_assembly_is_placed_by_its_transforms
    test_the_files_own_unit_is_kept
    test_a_coarse_deflection_is_flagged_by_the_cross_check_and_still_written
    test_a_file_that_is_not_step_is_refused_by_name
    test_a_named_solid_can_be_picked_and_an_unknown_one_is_a_usage_error)

foreach(_case IN LISTS _step_cases)
    add_test(NAME StepConvert.${_case}
             COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/test_step_convert.py
                     $<TARGET_FILE:spade_mesh_pack> ${PROJECT_SOURCE_DIR}/tests/golden/step ${_case})
    set_tests_properties(StepConvert.${_case} PROPERTIES
        LABELS "step"
        SKIP_RETURN_CODE 77
        TIMEOUT 120)
endforeach()
