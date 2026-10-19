# StampIdentity.ArchiveFallback (tests/CMakeLists.txt): cmake/stamp_identity.cmake's
# fallbacks, which a build inside a git checkout never reaches on its own. Each
# case runs the stamp script on fixtures this script writes under WORK_DIR (in
# the build tree) and checks the header it writes:
#   - a substituted archive file names its commit, clean;
#   - an unsubstituted `$Format:%H$`, a short or malformed commit, and no file
#     at all are "unknown", clean;
#   - a source dir nested inside a git work tree (the build tree is one, when it
#     sits in a checkout) does not borrow that tree's commit: it reads its own
#     archive file;
#   - the header is rewritten only when its text changes.
#
#   cmake -DSTAMP_SCRIPT=<cmake/stamp_identity.cmake> -DWORK_DIR=<dir> -P stamp_identity_test.cmake
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

# Runs the stamp script for SOURCE_DIR `source` into <WORK_DIR>/<name>.hpp, with
# the extra -D arguments given after `expected`, and checks the commit it names
# and that it says clean.
function(check_stamp name source expected)
    set(out "${WORK_DIR}/${name}.hpp")
    execute_process(COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${source}" "-DOUT=${out}" ${ARGN} -P "${STAMP_SCRIPT}"
        RESULT_VARIABLE rc OUTPUT_VARIABLE printed ERROR_VARIABLE printed)
    if(NOT rc EQUAL 0)
        fail("${name}: the stamp script failed (${rc}): ${printed}")
        return()
    endif()
    if(NOT EXISTS "${out}")
        fail("${name}: the stamp script wrote no ${out}")
        return()
    endif()
    file(READ "${out}" text)
    string(FIND "${text}" "#define SPADE_BUILD_COMMIT \"${expected}\"\n" at_commit)
    string(FIND "${text}" "#define SPADE_BUILD_DIRTY 0\n" at_clean)
    if(at_commit EQUAL -1 OR at_clean EQUAL -1)
        fail("${name}: expected commit \"${expected}\", clean; the header reads:\n${text}")
        return()
    endif()
    message(STATUS "StampIdentity ok: ${name} -> ${expected}")
endfunction()

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

# Rewritten only when its text changes: the same inputs a second later leave the
# file as it was; a new commit rewrites it.
file(TIMESTAMP "${WORK_DIR}/archived.hpp" before "%s" UTC)
execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 1.5)
check_stamp(archived "${WORK_DIR}" "${sha}" "-DARCHIVE_FILE=${WORK_DIR}/archived.txt" -DSTAMP_SKIP_GIT=ON)
file(TIMESTAMP "${WORK_DIR}/archived.hpp" after "%s" UTC)
if(NOT before STREQUAL after)
    fail("archived: the unchanged header was rewritten (${before} -> ${after}), which recompiles its includer every build")
endif()
file(WRITE "${WORK_DIR}/archived.txt" "${other}\n")
check_stamp(archived "${WORK_DIR}" "${other}" "-DARCHIVE_FILE=${WORK_DIR}/archived.txt" -DSTAMP_SKIP_GIT=ON)

get_property(failures GLOBAL PROPERTY stamp_failures)
list(LENGTH failures n)
if(n GREATER 0)
    message(FATAL_ERROR "StampIdentity: ${n} case(s) failed")
endif()
message(STATUS "StampIdentity: every case passed")
