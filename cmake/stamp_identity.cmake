# Writes OUT, the engine's build stamp (core/build_identity.hpp): the commit
# SOURCE_DIR's sources are at and whether their tracked files have uncommitted
# changes, as they are when the engine is built. The spade_build_stamp target
# (engine/CMakeLists.txt) runs it before every build of spade_core, and every
# tool reads the engine's stamp rather than taking its own (TD-9). OUT is
# rewritten only when its text changes, so an unchanged tree recompiles nothing.
#
#   cmake -DSOURCE_DIR=<Spade's source root> -DOUT=<header>
#         [-DARCHIVE_FILE=<file>] [-DSTAMP_SKIP_GIT=ON] -P stamp_identity.cmake
#
# The commit, first match wins; a commit is 40 lowercase hex:
#   1. git, when SOURCE_DIR is a work tree's top level (`rev-parse
#      --show-prefix` is empty there): `rev-parse HEAD`, dirty when `status
#      --porcelain --untracked-files=no` lists anything. A SOURCE_DIR nested in
#      another repository (a copy unpacked into a consumer's tree) is not a top
#      level, so it never borrows that repository's commit. The prefix, not a
#      comparison of --show-toplevel with SOURCE_DIR: git prints the path as it
#      is on disk, which need not match SOURCE_DIR's case or form on Windows.
#   2. ARCHIVE_FILE (default SOURCE_DIR/cmake/archive_commit.txt): `git archive`
#      writes the commit it archives there (export-subst, .gitattributes), so a
#      tree built from an archive, the Docker leg or a source tarball, names its
#      commit too. Never dirty: an archive holds a commit's files exactly.
#   3. "unknown", never dirty.
# STAMP_SKIP_GIT=ON skips 1, so a test inside a checkout reaches 2 and 3
# (tests/stamp_identity_test.cmake).
cmake_minimum_required(VERSION 3.28)

function(stamp_full_commit text out)
    string(LENGTH "${text}" n)
    if(n EQUAL 40 AND "${text}" MATCHES "^[0-9a-f]+$")
        set(${out} TRUE PARENT_SCOPE)
    else()
        set(${out} FALSE PARENT_SCOPE)
    endif()
endfunction()

set(commit "unknown")
set(dirty 0)
set(found FALSE)

if(NOT STAMP_SKIP_GIT)
    find_package(Git QUIET)
    if(GIT_FOUND)
        execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse --show-prefix
            OUTPUT_VARIABLE prefix RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(rc EQUAL 0 AND prefix STREQUAL "")
            execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse HEAD
                OUTPUT_VARIABLE sha RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
            stamp_full_commit("${sha}" full)
            if(rc EQUAL 0 AND full)
                set(commit "${sha}")
                set(found TRUE)
                execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" status --porcelain --untracked-files=no
                    OUTPUT_VARIABLE changes RESULT_VARIABLE rc ERROR_QUIET)
                if(rc EQUAL 0 AND NOT changes STREQUAL "")
                    set(dirty 1)
                endif()
            endif()
        endif()
    endif()
endif()

if(NOT found)
    if(NOT DEFINED ARCHIVE_FILE)
        set(ARCHIVE_FILE "${SOURCE_DIR}/cmake/archive_commit.txt")
    endif()
    if(EXISTS "${ARCHIVE_FILE}")
        file(READ "${ARCHIVE_FILE}" archived)
        string(STRIP "${archived}" archived)
        stamp_full_commit("${archived}" full)
        if(full)
            set(commit "${archived}")
        endif()
    endif()
endif()

set(text "// Generated at build time by cmake/stamp_identity.cmake; do not edit.
#pragma once
#define SPADE_BUILD_COMMIT \"${commit}\"
#define SPADE_BUILD_DIRTY ${dirty}
")
set(old "")
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL text)
    file(WRITE "${OUT}" "${text}")
endif()
