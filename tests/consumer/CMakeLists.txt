# ------------------------------------------------------------------------------
# Out-of-tree find_package(spade) consumer smoke (S5 Task 8: the kat build
# wiring). This is a SEPARATE, STANDALONE CMake project -- NOT add_subdirectory'd
# from CMakeLists.txt or tests/CMakeLists.txt -- because that is the
# whole point of it: the rest of tests/ links spade::* as in-tree
# build-tree targets and never touches spadeConfig.cmake/spadeTargets.cmake at
# all, so it cannot catch a defect in either. This project is configured and
# built against an INSTALLED Spade tree (CMAKE_PREFIX_PATH pointing at a
# `cmake --install` scratch prefix), which is what actually exercises the
# installable package this task adds -- and, per this task's brief, it is the
# enforcement mechanism for the class of problem a PRIVATE static-library
# dependency of an installed target (e.g. spade_world's yaml-cpp link, landing
# in a parallel lane) creates for a consumer's FINAL link: an out-of-tree
# executable that only links spade::sim/spade::vehicles is the one place in this
# repo that would fail to link if such a dependency's symbols are not reachable
# from the installed tree.
#
# .github/workflows/spade.yml used to drive this (install the engine build,
# then configure+build+run this project against that prefix) but was deleted
# 2026-09-18; scripts/test.ps1 is this repo's gate now, though it does not run
# this out-of-tree smoke either -- see main.cpp for what this project actually
# does.
# ------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.28)
project(spade_consumer_smoke LANGUAGES CXX)

include(FetchContent)

# ------------------------------------------------------------------------------
# glm -- the consumer's own obligation. spadeConfig.cmake.in (cmake/)
# documents WHY: glm is header-only and BUILD_INTERFACE-only in Spade's own
# export, so find_package(spade) alone cannot supply it. Pinned to the SAME tag
# vendor/CMakeLists.txt vendors (GIT_TAG 1.0.1) and copied in the same
# stanza shape rather than abstracted, so the pin stays visible here without
# cross-referencing the other file.
# ------------------------------------------------------------------------------
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
)
set(GLM_TEST_ENABLE OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glm)

# ------------------------------------------------------------------------------
# yaml-cpp -- the consumer's OTHER obligation (integration follow-up to S5
# Task 5's world-file reader/writer). spadeConfig.cmake.in (cmake/)
# documents WHY, at length: unlike glm (headers only), yaml-cpp is a COMPILED
# archive spade_world's world_file.hpp API needs at LINK time, and
# spadeConfig.cmake checks for it unconditionally because it cannot see, at
# configure time, whether main.cpp below will call into that API (it does --
# see main.cpp for the actual world-file round trip that exercises this).
# Pinned to the SAME tag vendor/CMakeLists.txt vendors (GIT_TAG 0.8.0)
# and copied in the same stanza shape (down to the CMAKE_POLICY_VERSION_MINIMUM
# workaround yaml-cpp 0.8.0's own ancient cmake_minimum_required needs), for
# the same reason glm's stanza above is copied rather than abstracted: the pin
# stays visible here without cross-referencing the other file.
# ------------------------------------------------------------------------------
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        0.8.0
    EXCLUDE_FROM_ALL
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

find_package(spade CONFIG REQUIRED)

add_executable(spade_consumer_smoke main.cpp)
target_compile_features(spade_consumer_smoke PRIVATE cxx_std_23)

# yaml-cpp::yaml-cpp is DELIBERATELY NOT listed here. It must reach this
# executable's final link line PURELY TRANSITIVELY -- but NOT through a
# spade::world INSTALL_INTERFACE link entry of its own: engine/CMakeLists.txt's
# spade_world yaml-cpp-link comment records that a $<INSTALL_INTERFACE:
# yaml-cpp::yaml-cpp> entry was TRIED and EMPIRICALLY REJECTED by
# install(EXPORT) validation, which resolves any name appearing in an exported
# target's INTERFACE_LINK_LIBRARIES -- genex-wrapped or not -- against every
# target this project knows about, and fails if it names a real,
# non-exported target (yaml-cpp::yaml-cpp, FetchContent-built, is exactly
# that). The propagation actually happens one layer up, in THIS project's own
# find_package(spade CONFIG REQUIRED) call above: spadeConfig.cmake.in
# (cmake/) confirms yaml-cpp::yaml-cpp exists (find_package(yaml-cpp)
# plus a FATAL_ERROR check) and then does, at CONFIGURE TIME, in the
# consumer's own CMake run --
#     set_property(TARGET spade::world APPEND PROPERTY
#                  INTERFACE_LINK_LIBRARIES yaml-cpp::yaml-cpp)
# -- appending onto the IMPORTED spade::world target that include()'d
# spadeTargets.cmake just defined here, which touches nothing Spade's own
# build ever validates. That set_property() call IS the thing this whole
# integration follow-up exists to prove works end to end. Linking
# yaml-cpp::yaml-cpp here directly would make this smoke pass whether or not
# that config-time propagation actually works, which is exactly the "PASSES
# WITH THE GAP INTACT" failure mode this file's yaml-cpp FetchContent block
# above exists to close: it only PROVIDES the target under the name
# spadeConfig.cmake.in's check expects, it does not itself consume it.
target_link_libraries(spade_consumer_smoke PRIVATE
    spade::sim
    spade::vehicles
    glm::glm
)
