# tests/sdk_install_geo_fetch.cmake -- run by the ctest SdkInstall.GeoFetchIsInstalledAndAnswersHelp (Windows only).
#
# Installs the SDK (the default components, not the editor's) into a fixed prefix under the build directory, then checks that the map fetch
# tool is in it and starts: <prefix>/bin/spade_geo_fetch.exe exists and `--help` exits 0 and prints its usage. Nothing else runs the SDK
# install on Windows (scripts/consumer-smoke.sh runs only in the Linux container; scripts/package.ps1 installs the editor component), so
# this is what keeps an install rule for the tool from going missing.
#
# The prefix is derived from BUILD_DIR alone and is removed first, so a stale tool from an earlier run cannot hide a missing rule. A
# BUILD_DIR that is empty, or not a directory holding a CMakeCache.txt, is refused before anything is removed.
#   cmake -DBUILD_DIR=<build tree> -DCONFIG=<config> -P sdk_install_geo_fetch.cmake

if(NOT BUILD_DIR OR NOT EXISTS "${BUILD_DIR}/CMakeCache.txt")
    message(FATAL_ERROR "sdk_install_geo_fetch: BUILD_DIR '${BUILD_DIR}' is not a CMake build tree (no CMakeCache.txt)")
endif()
set(prefix "${BUILD_DIR}/sdk_install_test")
file(REMOVE_RECURSE "${prefix}")

execute_process(COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --prefix "${prefix}" --config "${CONFIG}"
    RESULT_VARIABLE install_rc OUTPUT_QUIET ERROR_VARIABLE install_err)
if(NOT install_rc EQUAL 0)
    message(FATAL_ERROR "sdk_install_geo_fetch: cmake --install failed (exit ${install_rc}): ${install_err}")
endif()

set(tool "${prefix}/bin/spade_geo_fetch.exe")
if(NOT EXISTS "${tool}")
    message(FATAL_ERROR "spade_geo_fetch.exe is missing from the SDK install (${tool}): tools/geo_fetch/CMakeLists.txt must install the target with RUNTIME DESTINATION bin on Windows")
endif()

execute_process(COMMAND "${tool}" --help RESULT_VARIABLE help_rc OUTPUT_VARIABLE help_out ERROR_VARIABLE help_err)
if(NOT help_rc EQUAL 0)
    message(FATAL_ERROR "the installed spade_geo_fetch.exe --help exited ${help_rc}: ${help_err}")
endif()
if(NOT help_out MATCHES "usage: spade_geo_fetch")
    message(FATAL_ERROR "the installed spade_geo_fetch.exe --help did not print its usage")
endif()
message(STATUS "SdkInstall: spade_geo_fetch.exe is installed and answers --help")
