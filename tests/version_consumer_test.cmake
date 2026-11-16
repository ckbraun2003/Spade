# SpadeVersion.IncludedFromAnotherProject (tests/CMakeLists.txt): cmake/SpadeVersion.cmake builds the version header from ANY project that
# includes it, not only from the repository's root. sandbox/standalone (the SL2b guard, built against an installed prefix) is such a
# project: its PROJECT_SOURCE_DIR is sandbox/standalone, so a file that finds packaging/common/VERSION relative to PROJECT_SOURCE_DIR
# fails there ("spade_version.hpp: No such file or directory", gate 30's Linux leg).
#
# This writes a throwaway project outside the repository that includes the file, builds spade_version_header, and checks the header
# it wrote: it exists, and its kVersion is the first word of packaging/common/VERSION.
#
#   cmake -DREPO_DIR=<repo> -DWORK_DIR=<dir> -DGENERATOR=<name> -DMAKE_PROGRAM=<ninja> -P version_consumer_test.cmake
cmake_minimum_required(VERSION 3.28)

foreach(_required REPO_DIR WORK_DIR GENERATOR MAKE_PROGRAM)
    if(NOT DEFINED ${_required} OR "${${_required}}" STREQUAL "")
        message(FATAL_ERROR "SpadeVersion consumer test: -D${_required} is required")
    endif()
endforeach()
if(NOT EXISTS "${REPO_DIR}/cmake/SpadeVersion.cmake")
    message(FATAL_ERROR "SpadeVersion consumer test: no ${REPO_DIR}/cmake/SpadeVersion.cmake")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}/consumer")
file(WRITE "${WORK_DIR}/consumer/CMakeLists.txt"
"cmake_minimum_required(VERSION 3.28)
project(version_consumer NONE)
include(\"${REPO_DIR}/cmake/SpadeVersion.cmake\")
")

execute_process(
    COMMAND "${CMAKE_COMMAND}" -S "${WORK_DIR}/consumer" -B "${WORK_DIR}/build" -G "${GENERATOR}"
            "-DCMAKE_MAKE_PROGRAM=${MAKE_PROGRAM}"
    RESULT_VARIABLE _configure_rc OUTPUT_VARIABLE _configure_out ERROR_VARIABLE _configure_out)
if(NOT _configure_rc EQUAL 0)
    message(FATAL_ERROR "SpadeVersion consumer test: configuring the consumer failed (${_configure_rc}):\n${_configure_out}")
endif()

execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${WORK_DIR}/build" --target spade_version_header
    RESULT_VARIABLE _build_rc OUTPUT_VARIABLE _build_out ERROR_VARIABLE _build_out)
if(NOT _build_rc EQUAL 0)
    message(FATAL_ERROR "SpadeVersion consumer test: building spade_version_header failed (${_build_rc}):\n${_build_out}")
endif()

set(_header "${WORK_DIR}/build/generated/spade_version.hpp")
if(NOT EXISTS "${_header}")
    message(FATAL_ERROR "SpadeVersion consumer test: ${_header} was not written")
endif()

file(READ "${REPO_DIR}/packaging/common/VERSION" _version_text)
string(REGEX MATCH "^[ \t\r\n]*([0-9]+\\.[0-9]+\\.[0-9]+)" _matched "${_version_text}")
if(NOT _matched)
    message(FATAL_ERROR "SpadeVersion consumer test: packaging/common/VERSION does not begin with MAJOR.MINOR.PATCH")
endif()
set(_version "${CMAKE_MATCH_1}")
file(READ "${_header}" _header_text)
string(FIND "${_header_text}" "kVersion = \"${_version}\"" _at)
if(_at EQUAL -1)
    message(FATAL_ERROR "SpadeVersion consumer test: ${_header} does not say kVersion = \"${_version}\":\n${_header_text}")
endif()
message(STATUS "SpadeVersion consumer test: a project outside the repository root builds the header (${_version})")
