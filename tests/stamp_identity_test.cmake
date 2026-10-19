# StampIdentity.ArchiveFallback (tests/CMakeLists.txt): cmake/stamp_identity.cmake's
# fallbacks, which a build inside a git checkout never reaches on its own, and
# the two files one run writes: the header spade::build_identity() compiles in
# and the package fragment spadeConfig.cmake includes (spade_GIT_COMMIT,
# spade_GIT_DIRTY). Each case runs the stamp script on fixtures this script
# writes under WORK_DIR (in the build tree) and checks both files:
#   - they name one build, in every case, including SPADE_SOURCE itself (git's
#     answer when it is a checkout, dirty or not);
#   - a substituted archive file names its commit, clean;
#   - an unsubstituted `$Format:%H$`, a short or malformed commit, and no file
#     at all are "unknown", clean;
#   - a source dir nested inside a git work tree (the build tree is one, when it
#     sits in a checkout) does not borrow that tree's commit: it reads its own
#     archive file;
#   - each file is rewritten only when its text changes.
#
#   cmake -DSTAMP_SCRIPT=<cmake/stamp_identity.cmake> -DSPADE_SOURCE=<Spade's root>
#         -DWORK_DIR=<dir> -P stamp_identity_test.cmake
cmake_minimum_required(VERSION 3.28)

if(NOT EXISTS "${STAMP_SCRIPT}")
    message(FATAL_ERROR "StampIdentity: no stamp script at ${STAMP_SCRIPT}")
endif()
file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")

set(sha "0123456789abcdef0123456789abcdef01234567")
set(other "fedcba9876543210fedcba9876543210fedcba98")
set_property(GLOBAL PROPERTY stamp_failures "")

function(fail text)
    message(STATUS "StampIdentity FAIL: ${text}")
    set_property(GLOBAL APPEND PROPERTY stamp_failures "${text}")
endfunction()

# Runs the stamp script for SOURCE_DIR `source`, with the extra -D arguments
# given after `expected`, into <WORK_DIR>/<name>.hpp and the package fragment
# <WORK_DIR>/<name>.cmake. Both must name one build; unless `expected` is "*",
# that build must be commit `expected`, clean.
function(check_stamp name source expected)
    set(out "${WORK_DIR}/${name}.hpp")
    set(config_out "${WORK_DIR}/${name}.cmake")
    execute_process(COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${source}" "-DOUT=${out}" "-DCONFIG_OUT=${config_out}"
                            ${ARGN} -P "${STAMP_SCRIPT}"
        RESULT_VARIABLE rc OUTPUT_VARIABLE printed ERROR_VARIABLE printed)
    if(NOT rc EQUAL 0)
        fail("${name}: the stamp script failed (${rc}): ${printed}")
        return()
    endif()
    foreach(written IN ITEMS "${out}" "${config_out}")
        if(NOT EXISTS "${written}")
            fail("${name}: the stamp script wrote no ${written}")
            return()
        endif()
    endforeach()
    file(READ "${out}" header)
    if(NOT header MATCHES "#define SPADE_BUILD_COMMIT \"([^\"]*)\"\n#define SPADE_BUILD_DIRTY ([01])\n")
        fail("${name}: the header names no commit and dirty flag:\n${header}")
        return()
    endif()
    set(commit "${CMAKE_MATCH_1}")
    set(dirty "${CMAKE_MATCH_2}")
    file(READ "${config_out}" config)
    if(NOT config MATCHES "set[(]spade_GIT_COMMIT \"([^\"]*)\"[)]\nset[(]spade_GIT_DIRTY ([01])[)]\n")
        fail("${name}: the package fragment sets no spade_GIT_COMMIT and spade_GIT_DIRTY:\n${config}")
        return()
    endif()
    if(NOT CMAKE_MATCH_1 STREQUAL commit OR NOT CMAKE_MATCH_2 STREQUAL dirty)
        fail("${name}: the header says ${commit}, dirty ${dirty}; the package fragment says ${CMAKE_MATCH_1}, dirty ${CMAKE_MATCH_2}")
        return()
    endif()
    if(NOT expected STREQUAL "*" AND (NOT commit STREQUAL expected OR NOT dirty STREQUAL "0"))
        fail("${name}: expected commit ${expected}, clean; both files say ${commit}, dirty ${dirty}")
        return()
    endif()
    message(STATUS "StampIdentity ok: ${name} -> ${commit}, dirty ${dirty}, in the header and the package fragment")
endfunction()

# The source tree itself: whatever it is, the two files agree.
check_stamp(source-root "${SPADE_SOURCE}" "*")

# The archive's fallback, with git skipped: inside a checkout git would answer.
file(WRITE "${WORK_DIR}/archived.txt" "${sha}\n")
check_stamp(archived "${WORK_DIR}" "${sha}" "-DARCHIVE_FILE=${WORK_DIR}/archived.txt" -DSTAMP_SKIP_GIT=ON)
file(WRITE "${WORK_DIR}/archived-crlf.txt" "${sha}\r\n")
check_stamp(archived-crlf "${WORK_DIR}" "${sha}" "-DARCHIVE_FILE=${WORK_DIR}/archived-crlf.txt" -DSTAMP_SKIP_GIT=ON)
file(WRITE "${WORK_DIR}/unsubstituted.txt" "$Format:%H$")
check_stamp(unsubstituted "${WORK_DIR}" "unknown" "-DARCHIVE_FILE=${WORK_DIR}/unsubstituted.txt" -DSTAMP_SKIP_GIT=ON)
file(WRITE "${WORK_DIR}/short.txt" "0123456789ab\n")
check_stamp(short "${WORK_DIR}" "unknown" "-DARCHIVE_FILE=${WORK_DIR}/short.txt" -DSTAMP_SKIP_GIT=ON)
file(WRITE "${WORK_DIR}/uppercase.txt" "0123456789ABCDEF0123456789ABCDEF01234567\n")
check_stamp(uppercase "${WORK_DIR}" "unknown" "-DARCHIVE_FILE=${WORK_DIR}/uppercase.txt" -DSTAMP_SKIP_GIT=ON)
check_stamp(missing "${WORK_DIR}" "unknown" "-DARCHIVE_FILE=${WORK_DIR}/no-such-file.txt" -DSTAMP_SKIP_GIT=ON)

# Git not skipped, and the default archive file (SOURCE_DIR/cmake/
# archive_commit.txt): a directory that is not a work tree's top level reads it.
file(WRITE "${WORK_DIR}/nested/cmake/archive_commit.txt" "${other}\n")
check_stamp(nested "${WORK_DIR}/nested" "${other}")

# Rewritten only when its text changes: the same inputs a second later leave
# both files as they were; a new commit rewrites them.
file(TIMESTAMP "${WORK_DIR}/archived.hpp" header_before "%s" UTC)
file(TIMESTAMP "${WORK_DIR}/archived.cmake" config_before "%s" UTC)
execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 1.5)
check_stamp(archived "${WORK_DIR}" "${sha}" "-DARCHIVE_FILE=${WORK_DIR}/archived.txt" -DSTAMP_SKIP_GIT=ON)
file(TIMESTAMP "${WORK_DIR}/archived.hpp" header_after "%s" UTC)
file(TIMESTAMP "${WORK_DIR}/archived.cmake" config_after "%s" UTC)
if(NOT header_before STREQUAL header_after)
    fail("archived: the unchanged header was rewritten (${header_before} -> ${header_after}), which recompiles its includer every build")
endif()
if(NOT config_before STREQUAL config_after)
    fail("archived: the unchanged package fragment was rewritten (${config_before} -> ${config_after})")
endif()
file(WRITE "${WORK_DIR}/archived.txt" "${other}\n")
check_stamp(archived "${WORK_DIR}" "${other}" "-DARCHIVE_FILE=${WORK_DIR}/archived.txt" -DSTAMP_SKIP_GIT=ON)

get_property(failures GLOBAL PROPERTY stamp_failures)
list(LENGTH failures n)
if(n GREATER 0)
    message(FATAL_ERROR "StampIdentity: ${n} case(s) failed")
endif()
message(STATUS "StampIdentity: every case passed")
