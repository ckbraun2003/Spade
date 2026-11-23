# tests/sdk_install_refusal_check.cmake -- run by the ctest SdkInstall.RefusesADirectoryThatIsNotABuildTreeBeforeRemovingAnything (Windows only).
#
# What the test means: sdk_install_geo_fetch.cmake, given a directory that is not a CMake build tree, must (1) REFUSE: exit non-zero, (2) say so BY NAME:
# "is not a CMake build tree", and (3) remove NOTHING first. The earlier registration was WILL_FAIL plus PASS_REGULAR_EXPRESSION, and ctest reads those two
# together as "the regex was found, so invert it": the correct refusal failed the test (gate 52, 2026-11-23). It also could not see (3): it ran on tests/, where
# no scratch prefix exists, so a script that removed first would have passed.
#
# This script builds a scratch directory under the build tree with NO CMakeCache.txt and a sentinel file where the install script's scratch prefix would be
# (<dir>/sdk_install_test/keep.txt), runs the install script on it, and checks all three. SCRIPT defaults to the real script; a mutated copy can be given to
# show the check fails.
#   cmake -DBUILD_DIR=<the real build tree> -DCONFIG=<config> [-DSCRIPT=<script>] -P sdk_install_refusal_check.cmake

if(NOT BUILD_DIR OR NOT EXISTS "${BUILD_DIR}/CMakeCache.txt")
    message(FATAL_ERROR "sdk_install_refusal_check: BUILD_DIR '${BUILD_DIR}' is not a CMake build tree; the scratch directory goes under it")
endif()
if(NOT SCRIPT)
    set(SCRIPT "${CMAKE_CURRENT_LIST_DIR}/sdk_install_geo_fetch.cmake")
endif()

set(scratch "${BUILD_DIR}/sdk_refusal_scratch")
file(REMOVE_RECURSE "${scratch}")
file(MAKE_DIRECTORY "${scratch}/sdk_install_test")
file(WRITE "${scratch}/sdk_install_test/keep.txt" "this file must survive a refusal\n")

execute_process(COMMAND "${CMAKE_COMMAND}" -DBUILD_DIR=${scratch} -DCONFIG=${CONFIG} -P "${SCRIPT}"
    RESULT_VARIABLE rc OUTPUT_VARIABLE out ERROR_VARIABLE err)

set(problems "")
if(rc EQUAL 0)
    string(APPEND problems "the script exited 0 on a directory that is not a build tree (it must refuse); ")
endif()
if(NOT "${out}${err}" MATCHES "is not a CMake build tree")
    string(APPEND problems "the output does not say 'is not a CMake build tree': ${out}${err}; ")
endif()
if(NOT EXISTS "${scratch}/sdk_install_test/keep.txt")
    string(APPEND problems "the sentinel under the scratch prefix is gone: the script removed something before refusing; ")
endif()
file(REMOVE_RECURSE "${scratch}")
if(problems)
    message(FATAL_ERROR "sdk_install_refusal_check: ${problems}")
endif()
message(STATUS "SdkInstall refusal: exit ${rc}, named, nothing removed")
