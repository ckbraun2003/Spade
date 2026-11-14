# SpadeVersion.cmake -- the generated version header (L73a). Included from the root CMakeLists.txt before the sandbox and tests, and
# from sandbox/standalone/CMakeLists.txt before it adds the sandbox.
#
# spade_version_header regenerates <build>/generated/spade_version.hpp on every build (cmake/SpadeVersionGen.cmake, written only
# when it changes). spade_version is an INTERFACE target carrying its include directory and the build-order dependency: link it
# (spade_sandbox, spade_tests) and `#include "spade_version.hpp"` finds spade::version::kVersion, kGitSha, kDirty and kString.
# THE REPOSITORY ROOT IS FOUND FROM THIS FILE, NOT FROM PROJECT_SOURCE_DIR: sandbox/standalone (the SL2b guard, built against an
# installed prefix) includes this file from its own project, where PROJECT_SOURCE_DIR is sandbox/standalone and has no packaging/.
# The header is not installed into the SDK prefix; it belongs to the editor's build only.
cmake_path(GET CMAKE_CURRENT_LIST_DIR PARENT_PATH SPADE_VERSION_ROOT)
set(SPADE_VERSION_FILE "${SPADE_VERSION_ROOT}/packaging/common/VERSION")
set(SPADE_VERSION_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(SPADE_VERSION_HEADER "${SPADE_VERSION_GEN_DIR}/spade_version.hpp")

add_custom_target(spade_version_header
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${SPADE_VERSION_ROOT}
        -DVERSION_FILE=${SPADE_VERSION_FILE}
        -DOUT=${SPADE_VERSION_HEADER}
        -P ${CMAKE_CURRENT_LIST_DIR}/SpadeVersionGen.cmake
    BYPRODUCTS ${SPADE_VERSION_HEADER}
    COMMENT "spade_version.hpp"
    VERBATIM)

add_library(spade_version INTERFACE)
target_include_directories(spade_version INTERFACE "${SPADE_VERSION_GEN_DIR}")
add_dependencies(spade_version spade_version_header)
