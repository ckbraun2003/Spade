# Writes OUT, flow_movie's build stamp: the commit SOURCE_DIR's tree is at
# and whether its tracked files have uncommitted changes, as they are when the
# binary is built. The flow_movie_identity target runs it before every build
# of the tool; OUT is rewritten only when its text changes, so an unchanged
# tree recompiles nothing.
#
#   cmake -DSOURCE_DIR=<repo> -DOUT=<header> -P stamp_identity.cmake
set(commit "unknown")
set(dirty 0)
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse --short HEAD
        OUTPUT_VARIABLE sha RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(rc EQUAL 0 AND NOT sha STREQUAL "")
        set(commit "${sha}")
    endif()
    execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" status --porcelain --untracked-files=no
        OUTPUT_VARIABLE changes RESULT_VARIABLE rc ERROR_QUIET)
    if(rc EQUAL 0 AND NOT changes STREQUAL "")
        set(dirty 1)
    endif()
endif()
set(text "// Generated at build time by tools/flow_movie/stamp_identity.cmake; do not edit.
#pragma once
#define FLOW_MOVIE_COMMIT \"${commit}\"
#define FLOW_MOVIE_DIRTY ${dirty}
")
set(old "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL text)
    file(WRITE "${OUT}" "${text}")
endif()
