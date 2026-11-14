# SpadeVersion.cmake -- the generated version header (L73a). Included from the root CMakeLists.txt before the sandbox and tests.
#
# spade_version_header regenerates <build>/generated/spade_version.hpp on every build (cmake/SpadeVersionGen.cmake, written only
# when it changes). spade_version is an INTERFACE target carrying its include directory and the build-order dependency: link it
# (spade_sandbox, spade_tests) and `#include "spade_version.hpp"` finds spade::version::kVersion, kGitSha, kDirty and kString.
set(SPADE_VERSION_FILE "${PROJECT_SOURCE_DIR}/packaging/common/VERSION")
set(SPADE_VERSION_GEN_DIR "${CMAKE_BINARY_DIR}/generated")
set(SPADE_VERSION_HEADER "${SPADE_VERSION_GEN_DIR}/spade_version.hpp")

add_custom_target(spade_version_header
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
        -DVERSION_FILE=${SPADE_VERSION_FILE}
        -DOUT=${SPADE_VERSION_HEADER}
        -P ${PROJECT_SOURCE_DIR}/cmake/SpadeVersionGen.cmake
    BYPRODUCTS ${SPADE_VERSION_HEADER}
    COMMENT "spade_version.hpp"
    VERBATIM)

add_library(spade_version INTERFACE)
target_include_directories(spade_version INTERFACE "${SPADE_VERSION_GEN_DIR}")
add_dependencies(spade_version spade_version_header)
